// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "bvh4.h"
#include "bvh4_builder_fast.h"
#include "bvh4_statistics.h"
#include "bvh4_builder_binner.h"
#include "builders/primrefgen.h"

#include "geometry/bezier1.h"
#include "geometry/bezier1i.h"
#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"

#include <algorithm>

#define THRESHOLD_FOR_SUBTREE_RECURSION 128
#define THRESHOLD_FOR_SINGLE_THREADED 50000 // FIXME: measure if this is really optimal, maybe disable only parallel splits

#define DBG(x) 

//#define PROFILE

namespace embree
{
  namespace isa
  {
    static double dt = 0.0f;

    std::auto_ptr<BVH4BuilderFast::GlobalState> BVH4BuilderFast::g_state(NULL);

    BVH4BuilderFast::BVH4BuilderFast (BVH4* bvh, size_t logBlockSize, size_t logSAHBlockSize, bool needVertices, size_t primBytes, 
				      const size_t minLeafSize, const size_t maxLeafSize)
      : bvh(bvh), numPrimitives(0), prims(NULL), bytesPrims(0), logBlockSize(logBlockSize), logSAHBlockSize(logSAHBlockSize), needVertices(needVertices), primBytes(primBytes), 
	minLeafSize(minLeafSize), maxLeafSize(maxLeafSize)
    {
      needAllThreads = true;
    }

    BVH4BezierBuilderFast::BVH4BezierBuilderFast (BVH4* bvh, Scene* scene, 
						  size_t logBlockSize, size_t logSAHBlockSize, bool needVertices, size_t primBytes, 
						  const size_t minLeafSize, const size_t maxLeafSize)
      : scene(scene), geom(NULL), BVH4BuilderFast(bvh,logBlockSize,logSAHBlockSize,needVertices,primBytes,minLeafSize,maxLeafSize) {}
    
    BVH4BezierBuilderFast::BVH4BezierBuilderFast (BVH4* bvh, BezierCurves* geom, 
						  size_t logBlockSize, size_t logSAHBlockSize, bool needVertices, size_t primBytes, 
						  const size_t minLeafSize, const size_t maxLeafSize)
      : scene(geom->parent), geom(geom), BVH4BuilderFast(bvh,logBlockSize,logSAHBlockSize,needVertices,primBytes,minLeafSize,maxLeafSize) 
    {
      needAllThreads = geom->size() > THRESHOLD_FOR_SINGLE_THREADED;
    }

    BVH4TriangleBuilderFast::BVH4TriangleBuilderFast (BVH4* bvh, Scene* scene, 
						      size_t logBlockSize, size_t logSAHBlockSize, bool needVertices, size_t primBytes, 
						      const size_t minLeafSize, const size_t maxLeafSize)
      : scene(scene), mesh(NULL), BVH4BuilderFast(bvh,logBlockSize,logSAHBlockSize,needVertices,primBytes,minLeafSize,maxLeafSize) {}

    BVH4TriangleBuilderFast::BVH4TriangleBuilderFast (BVH4* bvh, TriangleMesh* mesh, 
						      size_t logBlockSize, size_t logSAHBlockSize, bool needVertices, size_t primBytes, 
						      const size_t minLeafSize, const size_t maxLeafSize)
      : scene(mesh->parent), mesh(mesh), BVH4BuilderFast(bvh,logBlockSize,logSAHBlockSize,needVertices,primBytes,minLeafSize,maxLeafSize) 
    {
      needAllThreads = mesh->numTriangles > THRESHOLD_FOR_SINGLE_THREADED;
    }

    BVH4UserGeometryBuilderFast::BVH4UserGeometryBuilderFast (BVH4* bvh, Scene* scene)
      : scene(scene), geom(NULL),        BVH4BuilderFast(bvh,0,0,false,sizeof(AccelSetItem),1,1) {}

    BVH4UserGeometryBuilderFast::BVH4UserGeometryBuilderFast (BVH4* bvh, UserGeometryBase* geom)
      : scene(geom->parent), geom(geom), BVH4BuilderFast(bvh,0,0,false,sizeof(AccelSetItem),1,1)
    {
      needAllThreads = geom->size() > THRESHOLD_FOR_SINGLE_THREADED;
    }

    BVH4Bezier1BuilderFast::BVH4Bezier1BuilderFast (BVH4* bvh, Scene* scene)
      : BVH4BezierBuilderFast(bvh,scene,0,0,false,sizeof(Bezier1),1,1) {}

    BVH4Bezier1iBuilderFast::BVH4Bezier1iBuilderFast (BVH4* bvh, Scene* scene)
      : BVH4BezierBuilderFast(bvh,scene,0,0,false,sizeof(Bezier1i),1,1) {}

    BVH4Triangle1BuilderFast::BVH4Triangle1BuilderFast (BVH4* bvh, Scene* scene)
      : BVH4TriangleBuilderFast(bvh,scene,0,0,false,sizeof(Triangle1),2,inf) {}

    BVH4Triangle4BuilderFast::BVH4Triangle4BuilderFast (BVH4* bvh, Scene* scene)
      : BVH4TriangleBuilderFast(bvh,scene,2,2,false,sizeof(Triangle4),4,inf) {}

#if defined(__AVX__)
    BVH4Triangle8BuilderFast::BVH4Triangle8BuilderFast (BVH4* bvh, Scene* scene)
      : BVH4TriangleBuilderFast(bvh,scene,3,2,false,sizeof(Triangle8),8,inf) {}
#endif
    
    BVH4Triangle1vBuilderFast::BVH4Triangle1vBuilderFast (BVH4* bvh, Scene* scene)
      : BVH4TriangleBuilderFast(bvh,scene,0,0,false,sizeof(Triangle1v),2,inf) {}

    BVH4Triangle4vBuilderFast::BVH4Triangle4vBuilderFast (BVH4* bvh, Scene* scene)
      : BVH4TriangleBuilderFast(bvh,scene,2,2,false,sizeof(Triangle4v),4,inf) {}
    
    BVH4Triangle4iBuilderFast::BVH4Triangle4iBuilderFast (BVH4* bvh, Scene* scene)
      : BVH4TriangleBuilderFast(bvh,scene,2,2,true,sizeof(Triangle4i),4,inf) {}

    BVH4Triangle1BuilderFast::BVH4Triangle1BuilderFast (BVH4* bvh, TriangleMesh* mesh)
      : BVH4TriangleBuilderFast(bvh,mesh,0,0,false,sizeof(Triangle1),2,inf) {}


    BVH4Bezier1BuilderFast::BVH4Bezier1BuilderFast (BVH4* bvh, BezierCurves* geom)
      : BVH4BezierBuilderFast(bvh,geom,0,0,false,sizeof(Bezier1),1,1) {}

    BVH4Bezier1iBuilderFast::BVH4Bezier1iBuilderFast (BVH4* bvh, BezierCurves* geom)
      : BVH4BezierBuilderFast(bvh,geom,0,0,false,sizeof(Bezier1i),1,1) {}

    BVH4Triangle4BuilderFast::BVH4Triangle4BuilderFast (BVH4* bvh, TriangleMesh* mesh)
      : BVH4TriangleBuilderFast(bvh,mesh,2,2,false,sizeof(Triangle4),4,inf) {}

#if defined(__AVX__)
    BVH4Triangle8BuilderFast::BVH4Triangle8BuilderFast (BVH4* bvh, TriangleMesh* mesh)
      : BVH4TriangleBuilderFast(bvh,mesh,3,2,false,sizeof(Triangle8),8,inf) {}
#endif
    
    BVH4Triangle1vBuilderFast::BVH4Triangle1vBuilderFast (BVH4* bvh, TriangleMesh* mesh)
      : BVH4TriangleBuilderFast(bvh,mesh,0,0,false,sizeof(Triangle1v),2,inf) {}

    BVH4Triangle4vBuilderFast::BVH4Triangle4vBuilderFast (BVH4* bvh, TriangleMesh* mesh)
      : BVH4TriangleBuilderFast(bvh,mesh,2,2,false,sizeof(Triangle4v),4,inf) {}
    
    BVH4Triangle4iBuilderFast::BVH4Triangle4iBuilderFast (BVH4* bvh, TriangleMesh* mesh)
      : BVH4TriangleBuilderFast(bvh,mesh,2,2,true,sizeof(Triangle4i),4,inf) {}
   
    BVH4BuilderFast::~BVH4BuilderFast () 
    {
      if (prims) os_free(prims,bytesPrims); prims = NULL;
      bvh->alloc.shrink(); 
    }
    
    void BVH4BuilderFast::build(size_t threadIndex, size_t threadCount) 
    {
      /* calculate size of scene */
      size_t numPrimitivesOld = numPrimitives;
      bvh->numPrimitives = numPrimitives = number_of_primitives();
      needAllThreads = numPrimitives > THRESHOLD_FOR_SINGLE_THREADED;
	  
      /* initialize BVH */
      bvh->init(numPrimitives, needAllThreads ? (threadCount+1) : 1); // threadCount+1 for toplevel build

      /* skip build for empty scene */
      if (numPrimitives == 0) 
	return;
      
      /* verbose mode */
      if (g_verbose >= 1)
        std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderFast ... " << std::flush;
      
      /* allocate build primitive array */
      if (numPrimitivesOld != numPrimitives)
      {
	if (prims) os_free(prims,bytesPrims);
	bytesPrims = numPrimitives * sizeof(PrimRef);
        prims = (PrimRef* ) os_malloc(bytesPrims);  memset(prims,0,bytesPrims);
      }
      
#if defined(PROFILE)
      
      double dt_min = pos_inf;
      double dt_avg = 0.0f;
      double dt_max = neg_inf;
      for (size_t i=0; i<20; i++) 
      {
        if (!needAllThreads) {
          build_sequential(threadIndex,threadCount);
        } 
        else {
          if (!g_state.get()) g_state.reset(new GlobalState(threadCount));
          TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel,this,threadCount,"build_parallel");
        }
        dt_min = min(dt_min,dt);
        dt_avg = dt_avg + dt;
        dt_max = max(dt_max,dt);
      }
      dt_avg /= double(20);
      
      std::cout << "[DONE]" << std::endl;
      std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
      std::cout << BVH4Statistics(bvh).str();
      
#else
      
      if (!needAllThreads) {
	build_sequential(threadIndex,threadCount);
      } 
      else {
	if (!g_state.get()) g_state.reset(new GlobalState(threadCount));
	TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel,this,threadCount,"build_parallel");
      }
      
      /* verbose mode */
      if (g_verbose >= 2) {
	std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
	std::cout << BVH4Statistics(bvh).str();
      }

      /* benchmark mode */
      if (g_benchmark) {
	BVH4Statistics stat(bvh);
	std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
      }
#endif
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    size_t BVH4BezierBuilderFast::number_of_primitives() 
    {
      if (geom) return geom->size();
      else      return scene->numBezierCurves;
    }
    
    void BVH4BezierBuilderFast::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      if (geom) PrimRefArrayGenFromGeometry<BezierCurves>::generate_sequential(threadIndex, threadCount, geom , prims, pinfo);
      else      PrimRefArrayGen                          ::generate_sequential(threadIndex, threadCount, scene, BEZIER_CURVES, 1, prims, pinfo);
    }

    void BVH4BezierBuilderFast::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, PrimInfo& pinfo) 
    {
      if (geom) PrimRefArrayGenFromGeometry<BezierCurves>::generate_parallel(threadIndex, threadCount, geom , prims, pinfo);
      else      PrimRefArrayGen                          ::generate_parallel(threadIndex, threadCount, scene, BEZIER_CURVES, 1, prims, pinfo);
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    size_t BVH4TriangleBuilderFast::number_of_primitives() 
    {
      if (mesh) return mesh->numTriangles;
      else      return scene->numTriangles;
    }
    
    void BVH4TriangleBuilderFast::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      if (mesh) PrimRefArrayGenFromGeometry<TriangleMesh>::generate_sequential(threadIndex, threadCount, mesh , prims, pinfo);
      else      PrimRefArrayGen                          ::generate_sequential(threadIndex, threadCount, scene, TRIANGLE_MESH, 1, prims, pinfo);
    }

    void BVH4TriangleBuilderFast::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, PrimInfo& pinfo) 
    {
      if (mesh) PrimRefArrayGenFromGeometry<TriangleMesh>::generate_parallel(threadIndex, threadCount, mesh , prims, pinfo);
      else      PrimRefArrayGen                          ::generate_parallel(threadIndex, threadCount, scene, TRIANGLE_MESH, 1, prims, pinfo);
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    size_t BVH4UserGeometryBuilderFast::number_of_primitives() 
    {
      if (geom) return geom->size();
      else      return scene->numUserGeometries1;
    }
    
    void BVH4UserGeometryBuilderFast::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      if (geom) PrimRefArrayGenFromGeometry<UserGeometryBase>::generate_sequential(threadIndex, threadCount, geom , prims, pinfo);
      else      PrimRefArrayGen                              ::generate_sequential(threadIndex, threadCount, scene, USER_GEOMETRY, 1, prims, pinfo);
    }

    void BVH4UserGeometryBuilderFast::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, PrimInfo& pinfo) 
    {
      if (geom) PrimRefArrayGenFromGeometry<UserGeometryBase>::generate_parallel(threadIndex, threadCount, geom , prims, pinfo);
      else      PrimRefArrayGen                              ::generate_parallel(threadIndex, threadCount, scene, USER_GEOMETRY, 1, prims, pinfo);
    }
 
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4Bezier1BuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
            
      /* allocate leaf node */
      Bezier1* accel = (Bezier1*) leafAlloc.malloc(items*sizeof(Bezier1));
      *current.parent = bvh->encodeLeaf((char*)accel,items);
      
      for (size_t i=0; i<items; i++) 
      {	
	const size_t geomID = prims[start+i].geomID();
        const size_t primID = prims[start+i].primID();
	const BezierCurves* curves = scene->getBezierCurves(geomID);
	const size_t id = curves->curve(primID);
	const Vec3fa& p0 = curves->vertex(id+0);
	const Vec3fa& p1 = curves->vertex(id+1);
	const Vec3fa& p2 = curves->vertex(id+2);
	const Vec3fa& p3 = curves->vertex(id+3);
	new (&accel[i]) Bezier1(p0,p1,p2,p3,0.0f,1.0f,geomID,primID);
      }
    }

    void BVH4Bezier1iBuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
            
      /* allocate leaf node */
      Bezier1i* accel = (Bezier1i*) leafAlloc.malloc(items*sizeof(Bezier1i));
      *current.parent = bvh->encodeLeaf((char*)accel,items);
      
      for (size_t i=0; i<items; i++) 
      {	
	const size_t geomID = prims[start+i].geomID();
        const size_t primID = prims[start+i].primID();
	const BezierCurves* curves = scene->getBezierCurves(geomID);
	const Vec3fa& p0 = curves->vertex(curves->curve(primID));
	new (&accel[i]) Bezier1i(&p0,geomID,primID);
      }
    }

    void BVH4Triangle1BuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
            
      /* allocate leaf node */
      Triangle1* accel = (Triangle1*) leafAlloc.malloc(items*sizeof(Triangle1));
      *current.parent = bvh->encodeLeaf((char*)accel,items);
      
      for (size_t i=0; i<items; i++) 
      {	
        const size_t geomID = prims[start+i].geomID();
        const size_t primID = prims[start+i].primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        
        const ssef v0 = select(0x7,(ssef)mesh->vertex(tri.v[0]),zero);
        const ssef v1 = select(0x7,(ssef)mesh->vertex(tri.v[1]),zero);
        const ssef v2 = select(0x7,(ssef)mesh->vertex(tri.v[2]),zero);
        
        const ssef e1 = v0 - v1;
        const ssef e2 = v2 - v0;	     
        const ssef normal = cross(e1,e2);
        
        store4f_nt(&accel[i].v0,cast(insert<3>(cast(v0),primID)));
        store4f_nt(&accel[i].v1,cast(insert<3>(cast(v1),geomID)));
        store4f_nt(&accel[i].v2,cast(insert<3>(cast(v2),mesh->mask)));
        store4f_nt(&accel[i].Ng,cast(insert<3>(cast(normal),0)));
      }
    }
    
    void BVH4Triangle4BuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
      assert(items<=4);
      
      /* allocate leaf node */
      Triangle4* accel = (Triangle4*) leafAlloc.malloc(sizeof(Triangle4));
      *current.parent = bvh->encodeLeaf((char*)accel,1);
      
      ssei vgeomID = -1, vprimID = -1, vmask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<items; i++)
      {
        const size_t geomID = prims[start+i].geomID();
        const size_t primID = prims[start+i].primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        vmask   [i] = mesh->mask;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(accel,Triangle4(v0,v1,v2,vgeomID,vprimID,vmask));
    }

#if defined(__AVX__)

    void BVH4Triangle8BuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
      assert(items<=8);
      
      /* allocate leaf node */
      Triangle8* accel = (Triangle8*) leafAlloc.malloc(sizeof(Triangle8));
      *current.parent = bvh->encodeLeaf((char*)accel,1);
      
      avxi vgeomID = -1, vprimID = -1, vmask = -1;
      avx3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<items; i++)
      {
        const size_t geomID = prims[start+i].geomID();
        const size_t primID = prims[start+i].primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        vmask   [i] = mesh->mask;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      new (accel) Triangle8(v0,v1,v2,vgeomID,vprimID,vmask);
      Triangle8::store_nt(accel,Triangle8(v0,v1,v2,vgeomID,vprimID,vmask));
    }
#endif
    
    void BVH4Triangle1vBuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
      assert(items<=4);
      
      /* allocate leaf node */
      Triangle1v* accel = (Triangle1v*) leafAlloc.malloc(items*sizeof(Triangle1v));
      *current.parent = bvh->encodeLeaf((char*)accel,items);
      
      for (size_t i=0; i<items; i++) 
      {	
        const size_t geomID = prims[start+i].geomID();
        const size_t primID = prims[start+i].primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        
        const ssef v0 = select(0x7,(ssef)mesh->vertex(tri.v[0]),zero);
        const ssef v1 = select(0x7,(ssef)mesh->vertex(tri.v[1]),zero);
        const ssef v2 = select(0x7,(ssef)mesh->vertex(tri.v[2]),zero);
        
        const ssef e1 = v0 - v1;
        const ssef e2 = v2 - v0;	     
        const ssef normal = cross(e1,e2);
        
        store4f_nt(&accel[i].v0,cast(insert<3>(cast(v0),primID)));
        store4f_nt(&accel[i].v1,cast(insert<3>(cast(v1),geomID)));
        store4f_nt(&accel[i].v2,cast(insert<3>(cast(v2),mesh->mask)));
      }
    }
    
    void BVH4Triangle4vBuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
      assert(items<=4);
      
      /* allocate leaf node */
      Triangle4v* accel = (Triangle4v*) leafAlloc.malloc(sizeof(Triangle4v));
      *current.parent = bvh->encodeLeaf((char*)accel,1);
      
      ssei vgeomID = -1, vprimID = -1, vmask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<items; i++)
      {
        const size_t geomID = prims[start+i].geomID();
        const size_t primID = prims[start+i].primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        vmask   [i] = mesh->mask;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4v::store_nt(accel,Triangle4v(v0,v1,v2,vgeomID,vprimID,vmask));
    }

    void BVH4Triangle4iBuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
      assert(items<=4);
      
      /* allocate leaf node */
      Triangle4i* accel = (Triangle4i*) leafAlloc.malloc(sizeof(Triangle4i));
      *current.parent = bvh->encodeLeaf((char*)accel,1);
      
      ssei geomID = -1, primID = -1;
      Vec3f* v0[4] = { NULL, NULL, NULL, NULL };
      ssei v1 = zero, v2 = zero;
      
      for (size_t i=0; i<items; i++)
      {
	const PrimRef& prim = prims[start+i];
	const TriangleMesh* mesh = scene->getTriangleMesh(prim.geomID());
	const TriangleMesh::Triangle& tri = mesh->triangle(prim.primID());
	geomID[i] = prim.geomID();
	primID[i] = prim.primID();
	v0[i] = (Vec3f*) &mesh->vertex(tri.v[0]); 
	v1[i] = (int*)&mesh->vertex(tri.v[1])-(int*)v0[i]; 
	v2[i] = (int*)&mesh->vertex(tri.v[2])-(int*)v0[i]; 
      }

      for (size_t i=items; i<4; i++)
      {
	geomID[i] = -1;
	primID[i] = -1;
	v0[i] = v0[0];
	v1[i] = 0; 
	v2[i] = 0;
      }
    
      new (accel) Triangle4i(v0,v1,v2,geomID,primID);
    }

    void BVH4UserGeometryBuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      size_t start = current.begin;
      
      /* allocate leaf node */
      AccelSetItem* accel = (AccelSetItem*) leafAlloc.malloc(sizeof(AccelSetItem)*items);
      *current.parent = bvh->encodeLeaf(accel,items);
      
      for (size_t i=0; i<items; i++)
      {
	const PrimRef& prim = prims[start+i];
	accel[i].accel = (AccelSet*) (UserGeometryBase*) scene->get(prim.geomID()); //(*accels)[prim.geomID()];
        accel[i].item  = prim.primID();
      }
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4BuilderFast::splitFallback(PrimRef * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
    {
      const unsigned int center = (current.begin + current.end)/2;
      
      CentGeomBBox3fa left; left.reset();
      for (size_t i=current.begin; i<center; i++)
        left.extend(primref[i].bounds());
      leftChild.init(left,current.begin,center);
      
      CentGeomBBox3fa right; right.reset();
      for (size_t i=center; i<current.end; i++)
        right.extend(primref[i].bounds());	
      rightChild.init(right,center,current.end);
    }

    __forceinline void BVH4BuilderFast::splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t numThreads)
    {
      /* calculate binning function */
      PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);
      ObjectPartition::Split split = ObjectPartition::find(prims,current.begin,current.end,pinfo,logBlockSize);
      
      /* if we cannot find a valid split, enforce an arbitrary split */
      if (unlikely(!split.valid())) splitFallback(prims,current,leftChild,rightChild);
      
      /* partitioning of items */
      else split.partition(prims, current.begin, current.end, leftChild, rightChild);
    }
    
    void BVH4BuilderFast::splitParallel(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t numThreads)
    {
      /* use primitive array temporarily for parallel splits */
      PrimRef* tmp = (PrimRef*) bvh->alloc.curPtr();
	  PrimInfo pinfo(current.begin,current.end,current.geomBounds,current.centBounds);

      /* parallel binning of centroids */
      const float sah = g_state->parallelBinner.find(pinfo,prims,tmp,logBlockSize,threadID,numThreads);

      /* if we cannot find a valid split, enforce an arbitrary split */
      if (unlikely(sah == float(inf))) splitFallback(prims,current,leftChild,rightChild);
      
      /* parallel partitioning of items */
      else g_state->parallelBinner.partition(pinfo,tmp,prims,leftChild,rightChild,threadID,numThreads);
    }
    
    __forceinline void BVH4BuilderFast::split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads)
    {
      if (mode == BUILD_TOP_LEVEL) splitParallel(current,left,right,threadID,numThreads);		  
      else                         splitSequential(current,left,right,threadID,numThreads);
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4BuilderFast::createLeaf(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, size_t threadIndex, size_t threadCount)
    {
      if (current.depth > BVH4::maxBuildDepthLeaf) 
        throw std::runtime_error("depth limit reached");
      
      /* create leaf for few primitives */
      if (current.size() <= minLeafSize) {
        createSmallLeaf(current,leafAlloc,threadIndex);
        return;
      }
      
      /* first split level */
      BuildRecord record0, record1;
      splitFallback(prims,current,record0,record1);
      
      /* second split level */
      BuildRecord children[4];
      splitFallback(prims,record0,children[0],children[1]);
      splitFallback(prims,record1,children[2],children[3]);

      /* allocate node */
      Node* node = (Node*) nodeAlloc.malloc(sizeof(Node)); node->clear();
      *current.parent = bvh->encodeNode(node);
      
      /* recurse into each child */
      for (size_t i=0; i<4; i++) 
      {
        node->set(i,children[i].geomBounds);
        children[i].parent = &node->child(i);
        children[i].depth = current.depth+1;
        createLeaf(children[i],nodeAlloc,leafAlloc,threadIndex,threadCount);
      }
      BVH4::compact(node); // move empty nodes to the end
    }
    
    __forceinline void BVH4BuilderFast::recurse_continue(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, const size_t mode, const size_t threadID, const size_t numThreads)
    {
      if (mode == BUILD_TOP_LEVEL) {
      	g_state->heap.push(current);
      }
      else if (mode == RECURSE_PARALLEL && current.size() > THRESHOLD_FOR_SUBTREE_RECURSION) {
        if (!g_state->threadStack[threadID].push(current))
          recurse(current,nodeAlloc,leafAlloc,RECURSE_SEQUENTIAL,threadID,numThreads);
      }
      else
        recurse(current,nodeAlloc,leafAlloc,mode,threadID,numThreads);
    }
    
    void BVH4BuilderFast::recurse(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, const size_t mode, const size_t threadID, const size_t numThreads)
    {
      __aligned(64) BuildRecord children[BVH4::N];
      
      /* create leaf node */
      if (current.depth >= BVH4::maxBuildDepth || current.size() <= minLeafSize) {
        assert(mode != BUILD_TOP_LEVEL);
        createLeaf(current,nodeAlloc,leafAlloc,threadID,numThreads);
        return;
      }

      /* fill all 4 children by always splitting the one with the largest surface area */
      unsigned int numChildren = 1;
      children[0] = current;

      do {
        
        /* find best child with largest bounding box area */
        int bestChild = -1;
        float bestArea = neg_inf;
        for (unsigned int i=0; i<numChildren; i++)
        {
          /* ignore leaves as they cannot get split */
          if (children[i].size() <= minLeafSize)
            continue;
          
          /* remember child with largest area */
          if (children[i].sceneArea() > bestArea) { 
            bestArea = children[i].sceneArea();
            bestChild = i;
          }
        }
        if (bestChild == -1) break;
        
        /*! split best child into left and right child */
        __aligned(64) BuildRecord left, right;
        split(children[bestChild],left,right,mode,threadID,numThreads);
        
        /* add new children left and right */
	left.init(current.depth+1); 
	right.init(current.depth+1);
        children[bestChild] = children[numChildren-1];
        children[numChildren-1] = left;
        children[numChildren+0] = right;
        numChildren++;
        
      } while (numChildren < BVH4::N);

      /* create leaf node if no split is possible */
      if (numChildren == 1) {
        assert(mode != BUILD_TOP_LEVEL);
        createLeaf(current,nodeAlloc,leafAlloc,threadID,numThreads);
        return;
      }
      
      /* allocate node */
      Node* node = (Node*) nodeAlloc.malloc(sizeof(Node)); node->clear();
      *current.parent = bvh->encodeNode(node);
      
      /* recurse into each child */
      for (unsigned int i=0; i<numChildren; i++) 
      {  
        node->set(i,children[i].geomBounds);
        children[i].parent = &node->child(i);
        recurse_continue(children[i],nodeAlloc,leafAlloc,mode,threadID,numThreads);
      }
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4BuilderFast::buildSubTrees(size_t threadID, size_t numThreads, size_t taskIndex, size_t taskCount, TaskScheduler::Event* taskGroup)
    {
      __aligned(64) Allocator nodeAlloc(&bvh->alloc);
      __aligned(64) Allocator leafAlloc(&bvh->alloc);
      
      while (true) 
      {
	BuildRecord br;
	if (!g_state->heap.pop(br))
        {
          /* global work queue empty => try to steal from neighboring queues */	  
          bool success = false;
          for (size_t i=0; i<numThreads; i++)
          {
            if (g_state->threadStack[(threadID+i)%numThreads].pop(br)) {
              success = true;
              break;
            }
          }
          /* found nothing to steal ? */
          if (!success) break; // FIXME: may loose threads
        }
        
        /* process local work queue */
	recurse(br,nodeAlloc,leafAlloc,RECURSE_PARALLEL,threadID,numThreads);
	while (g_state->threadStack[threadID].pop(br))
          recurse(br,nodeAlloc,leafAlloc,RECURSE_PARALLEL,threadID,numThreads);
      }
    }

    void BVH4BuilderFast::build_sequential(size_t threadIndex, size_t threadCount) 
    {
      /* start measurement */
      double t0 = 0.0f;
      if (g_verbose >= 2) t0 = getSeconds();
      
      /* initialize node and leaf allocator */
      bvh->alloc.clear();
      __aligned(64) Allocator nodeAlloc(&bvh->alloc);
      __aligned(64) Allocator leafAlloc(&bvh->alloc);
     
      /* create prim refs */
      PrimInfo pinfo(empty);
      create_primitive_array_sequential(threadIndex, threadCount, pinfo);
      bvh->bounds = pinfo.geomBounds;

      /* create initial build record */
      BuildRecord br;
      br.init(pinfo,0,numPrimitives);
      br.depth = 1;
      br.parent = &bvh->root;

      /* build BVH in single thread */
      recurse(br,nodeAlloc,leafAlloc,RECURSE_SEQUENTIAL,threadIndex,threadCount);

      /* stop measurement */
      if (g_verbose >= 2) dt = getSeconds()-t0;
    }

    void BVH4BuilderFast::build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
    {
      /* all worker threads enter tasking system */
      if (TaskScheduler::enter(threadIndex,threadCount))
	return;

      /* start measurement */
      double t0 = 0.0f;
      if (g_verbose >= 2) t0 = getSeconds();

      /* calculate list of primrefs */
      PrimInfo pinfo(empty);
      create_primitive_array_parallel(threadIndex, threadCount, pinfo);
      bvh->bounds = pinfo.geomBounds;
      
      /* initialize node and leaf allocator */
      bvh->alloc.clear();
      __aligned(64) Allocator nodeAlloc(&bvh->alloc);
      __aligned(64) Allocator leafAlloc(&bvh->alloc);

      /* create initial build record */
      BuildRecord br;
      br.init(pinfo,0,numPrimitives);
      br.depth = 1;
      br.parent = &bvh->root;
      
      /* initialize thread-local work stacks */
      for (size_t i=0; i<threadCount; i++)
        g_state->threadStack[i].reset();
      
      /* push initial build record to global work stack */
      g_state->heap.reset();
      g_state->heap.push(br);

      /* work in multithreaded toplevel mode until sufficient subtasks got generated */
      while (g_state->heap.size() < 2*threadCount)
      {
        BuildRecord br;

        /* pop largest item for better load balancing */
	if (!g_state->heap.pop(br)) 
          break;
        
        /* guarantees to create no leaves in this stage */
        if (br.size() <= minLeafSize) {
	  g_state->heap.push(br);
          break;
	}

        recurse(br,nodeAlloc,leafAlloc,BUILD_TOP_LEVEL,threadIndex,threadCount);
      }

      std::sort(g_state->heap.begin(),g_state->heap.end(),BuildRecord::Greater());

      /* now process all created subtasks on multiple threads */
      TaskScheduler::dispatchTask(_buildSubTrees, this, threadIndex, threadCount );
      
      /* release all threads again */
      TaskScheduler::leave(threadIndex,threadCount);
      
      /* stop measurement */
      if (g_verbose >= 2) dt = getSeconds()-t0;
    }
    
    Builder* BVH4Bezier1BuilderFast    (void* bvh, Scene* scene, size_t mode) { return new class BVH4Bezier1BuilderFast ((BVH4*)bvh,scene); }
    Builder* BVH4Bezier1iBuilderFast   (void* bvh, Scene* scene, size_t mode) { return new class BVH4Bezier1iBuilderFast ((BVH4*)bvh,scene); }
    Builder* BVH4Triangle1BuilderFast  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle1BuilderFast ((BVH4*)bvh,scene); }
    Builder* BVH4Triangle4BuilderFast  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle4BuilderFast ((BVH4*)bvh,scene); }
#if defined(__AVX__)
    Builder* BVH4Triangle8BuilderFast  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle8BuilderFast ((BVH4*)bvh,scene); }
#endif
    Builder* BVH4Triangle1vBuilderFast (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle1vBuilderFast((BVH4*)bvh,scene); }
    Builder* BVH4Triangle4vBuilderFast (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle4vBuilderFast((BVH4*)bvh,scene); }
    Builder* BVH4Triangle4iBuilderFast (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle4iBuilderFast((BVH4*)bvh,scene); }
    Builder* BVH4UserGeometryBuilderFast(void* bvh, Scene* scene, size_t mode) { return new class BVH4UserGeometryBuilderFast((BVH4*)bvh,scene); }

    Builder* BVH4Bezier1MeshBuilderFast    (void* bvh, BezierCurves* geom, size_t mode) { return new class BVH4Bezier1BuilderFast ((BVH4*)bvh,geom); }
    Builder* BVH4Bezier1iMeshBuilderFast   (void* bvh, BezierCurves* geom, size_t mode) { return new class BVH4Bezier1iBuilderFast ((BVH4*)bvh,geom); }
    Builder* BVH4Triangle1MeshBuilderFast  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Triangle1BuilderFast ((BVH4*)bvh,mesh); }
    Builder* BVH4Triangle4MeshBuilderFast  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Triangle4BuilderFast ((BVH4*)bvh,mesh); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderFast  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Triangle8BuilderFast ((BVH4*)bvh,mesh); }
#endif
    Builder* BVH4Triangle1vMeshBuilderFast (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Triangle1vBuilderFast((BVH4*)bvh,mesh); }
    Builder* BVH4Triangle4vMeshBuilderFast (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Triangle4vBuilderFast((BVH4*)bvh,mesh); }
    Builder* BVH4Triangle4iMeshBuilderFast (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Triangle4iBuilderFast((BVH4*)bvh,mesh); }
    Builder* BVH4UserGeometryMeshBuilderFast   (void* bvh, UserGeometryBase* geom, size_t mode) { return new class BVH4UserGeometryBuilderFast((BVH4*)bvh,geom); }
  }
}
