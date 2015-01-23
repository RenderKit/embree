// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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
#include "builders/primrefgen.h"

#include "geometry/bezier1v.h"
#include "geometry/bezier1i.h"
#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"
#include "geometry/subdivpatch1.h"

#include "geometry/grid.h"
#include "common/subdiv/feature_adaptive_gregory.h"
#include "common/subdiv/feature_adaptive_bspline.h"
#include "geometry/subdivpatch1cached.h"
#include "geometry/virtual_accel.h"

#include <algorithm>



#define DBG(x) 

//#define PROFILE

namespace embree
{
  namespace isa
  {
    //static double dt = 0.0f;

    static const size_t THRESHOLD_FOR_SUBTREE_RECURSION = 128;
    static const size_t THRESHOLD_FOR_SINGLE_THREADED = 50000; // FIXME: measure if this is really optimal, maybe disable only parallel splits

    BVH4BuilderFast::BVH4BuilderFast (LockStepTaskScheduler* scheduler, BVH4* bvh, size_t listMode, size_t logBlockSize, size_t logSAHBlockSize, 
				      bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize)
      : scheduler(scheduler), state(nullptr), bvh(bvh), numPrimitives(0), prims(NULL), bytesPrims(0), listMode(listMode), logBlockSize(logBlockSize), logSAHBlockSize(logSAHBlockSize), 
	needVertices(needVertices), primBytes(primBytes), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize)
    {
      size_t maxLeafPrims = BVH4::maxLeafBlocks*(size_t(1)<<logBlockSize);
      if (maxLeafPrims < this->maxLeafSize) this->maxLeafSize = maxLeafPrims;
      needAllThreads = true;
    }

    template<typename Primitive>
    BVH4BuilderFastT<Primitive>::BVH4BuilderFastT (BVH4* bvh, Scene* scene, size_t listMode, size_t logBlockSize, size_t logSAHBlockSize, 
						   bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize,bool parallel)
      : scene(scene), BVH4BuilderFast(&scene->lockstep_scheduler,bvh,listMode,logBlockSize,logSAHBlockSize,needVertices,primBytes,minLeafSize,maxLeafSize) { needAllThreads = parallel; }
    
    template<> BVH4BezierBuilderFast  <Bezier1v>   ::BVH4BezierBuilderFast   (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<Bezier1v>   (bvh,scene,listMode,0,0,false,sizeof(Bezier1v),1,1,true) {}
    template<> BVH4BezierBuilderFast  <Bezier1i>  ::BVH4BezierBuilderFast   (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<Bezier1i>  (bvh,scene,listMode,0,0,false,sizeof(Bezier1i),1,1,true) {}
    template<> BVH4TriangleBuilderFast<Triangle1> ::BVH4TriangleBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<Triangle1> (bvh,scene,listMode,0,0,false,sizeof(Triangle1),2,inf,true) {}
    template<> BVH4TriangleBuilderFast<Triangle4> ::BVH4TriangleBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<Triangle4> (bvh,scene,listMode,2,2,false,sizeof(Triangle4),4,inf,true) {}


#if defined(__AVX__)
    template<> BVH4TriangleBuilderFast<Triangle8> ::BVH4TriangleBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<Triangle8> (bvh,scene,listMode,3,2,false,sizeof(Triangle8),8,inf,true) {}
#endif
    template<> BVH4TriangleBuilderFast<Triangle1v>::BVH4TriangleBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<Triangle1v>(bvh,scene,listMode,0,0,false,sizeof(Triangle1v),2,inf,true) {}
    template<> BVH4TriangleBuilderFast<Triangle4v>::BVH4TriangleBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<Triangle4v>(bvh,scene,listMode,2,2,false,sizeof(Triangle4v),4,inf,true) {}
    template<> BVH4TriangleBuilderFast<Triangle4i>::BVH4TriangleBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<Triangle4i>(bvh,scene,listMode,2,2,true,sizeof(Triangle4i),4,inf,true) {}
    template<> BVH4UserGeometryBuilderFastT<AccelSetItem>::BVH4UserGeometryBuilderFastT (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<AccelSetItem>(bvh,scene,listMode,0,0,false,sizeof(AccelSetItem),1,1,true) {}

    template<> BVH4BezierBuilderFast  <Bezier1v>   ::BVH4BezierBuilderFast   (BVH4* bvh, BezierCurves* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<Bezier1v>   (bvh,geom->parent,listMode,0,0,false,sizeof(Bezier1v)   ,1,1,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}
    template<> BVH4BezierBuilderFast  <Bezier1i>  ::BVH4BezierBuilderFast   (BVH4* bvh, BezierCurves* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<Bezier1i>  (bvh,geom->parent,listMode,0,0,false,sizeof(Bezier1i)  ,1,1,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}
    template<> BVH4TriangleBuilderFast<Triangle1> ::BVH4TriangleBuilderFast (BVH4* bvh, TriangleMesh* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<Triangle1> (bvh,geom->parent,listMode,0,0,false,sizeof(Triangle1) ,2,inf,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}
    template<> BVH4TriangleBuilderFast<Triangle4> ::BVH4TriangleBuilderFast (BVH4* bvh, TriangleMesh* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<Triangle4> (bvh,geom->parent,listMode,2,2,false,sizeof(Triangle4) ,4,inf,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}
#if defined(__AVX__)
    template<> BVH4TriangleBuilderFast<Triangle8> ::BVH4TriangleBuilderFast (BVH4* bvh, TriangleMesh* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<Triangle8> (bvh,geom->parent,listMode,3,2,false,sizeof(Triangle8) ,8,inf,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}
#endif
    template<> BVH4TriangleBuilderFast<Triangle1v>::BVH4TriangleBuilderFast (BVH4* bvh, TriangleMesh* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<Triangle1v>(bvh,geom->parent,listMode,0,0,false,sizeof(Triangle1v),2,inf,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}
    template<> BVH4TriangleBuilderFast<Triangle4v>::BVH4TriangleBuilderFast (BVH4* bvh, TriangleMesh* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<Triangle4v>(bvh,geom->parent,listMode,2,2,false,sizeof(Triangle4v),4,inf,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}
    template<> BVH4TriangleBuilderFast<Triangle4i>::BVH4TriangleBuilderFast (BVH4* bvh, TriangleMesh* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<Triangle4i>(bvh,geom->parent,listMode,2,2,true ,sizeof(Triangle4i),4,inf,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}

    template<> BVH4UserGeometryBuilderFastT<AccelSetItem>::BVH4UserGeometryBuilderFastT (BVH4* bvh, UserGeometryBase* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<AccelSetItem>(bvh,geom->parent,listMode,0,0,false,sizeof(AccelSetItem),1,1,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}

    template<> BVH4SubdivBuilderFast<SubdivPatch1>::BVH4SubdivBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : geom(NULL), BVH4BuilderFastT<SubdivPatch1>(bvh,scene,listMode,0,0,false,sizeof(SubdivPatch1),1,1,true) {}
    template<> BVH4SubdivBuilderFast<SubdivPatch1>::BVH4SubdivBuilderFast (BVH4* bvh, SubdivMesh* geom, size_t listMode) 
      : geom(geom), BVH4BuilderFastT<SubdivPatch1>(bvh,geom->parent,listMode,0,0,false,sizeof(SubdivPatch1),1,1,geom->size() > THRESHOLD_FOR_SINGLE_THREADED) {}

    BVH4SubdivPatch1CachedBuilderFast::BVH4SubdivPatch1CachedBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : BVH4BuilderFastT<PrimRef>(bvh,scene,listMode,0,0,false,32,1,1,true),fastUpdateMode(false),fastUpdateMode_numFaces(0)
    { 
      //this->bvh->alloc2.init(4096,4096); 
    
    } 

    
    template<> BVH4TriangleBuilderFastSweep<Triangle4> ::BVH4TriangleBuilderFastSweep (BVH4* bvh, Scene *scene, size_t listMode) 
      : BVH4TriangleBuilderFast<Triangle4> (bvh,scene,listMode),tmp(NULL) {
      centroids[0] = NULL;
      centroids[1] = NULL;
      centroids[2] = NULL;      
      needAllThreads = false;
    }

    BVH4TopLevelBuilderFastT::BVH4TopLevelBuilderFastT (LockStepTaskScheduler* scheduler, BVH4* bvh) 
      : prims_i(NULL), N(0), BVH4BuilderFast(scheduler,bvh,0,0,0,false,0,1,1) {}

    BVH4BuilderFast::~BVH4BuilderFast () 
    {
      if (prims) os_free(prims,bytesPrims); prims = NULL;
      bvh->alloc.shrink(); 
    }
    
    void test_partition();

    void BVH4BuilderFast::build(size_t threadIndex, size_t threadCount) 
    {
      // test_partition(); 

      /* start measurement */
      double t0 = 0.0f;
      if (g_verbose >= 1) t0 = getSeconds();

      /* calculate size of scene */
      size_t numPrimitivesOld = numPrimitives;
      bvh->numPrimitives = numPrimitives = number_of_primitives();
      bool parallel = needAllThreads && numPrimitives > THRESHOLD_FOR_SINGLE_THREADED;

      /* initialize BVH */
      if (numPrimitivesOld != numPrimitives)
        bvh->init(sizeof(BVH4::Node),numPrimitives, parallel ? (threadCount+1) : 1); // threadCount+1 for toplevel build

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
        prims = (PrimRef* ) os_malloc(bytesPrims);  
        memset(prims,0,bytesPrims);
      }

#if defined(PROFILE)
      
      double dt_min = pos_inf;
      double dt_avg = 0.0f;
      double dt_max = neg_inf;
#define ITERATIONS 20
      for (size_t i=0; i<ITERATIONS; i++) 
      {
        double t0 = getSeconds();
#endif

        if (!parallel) {
          build_sequential(threadIndex,threadCount);
        } 
        else {
          state.reset(new GlobalState());
          size_t numActiveThreads = threadCount;
          //size_t numActiveThreads = min(threadCount,getNumberOfCores());
          build_parallel(threadIndex,numActiveThreads,0,1);
          state.reset(NULL);
        }

#if defined(PROFILE)
        double dt = getSeconds()-t0;
        dt_min = min(dt_min,dt);
        dt_avg = dt_avg + dt;
        dt_max = max(dt_max,dt);
      }
      dt_avg /= double(ITERATIONS);
      
      std::cout << "[DONE]" << std::endl;
      std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
      std::cout << BVH4Statistics(bvh).str();
#endif
      
      /* stop measurement */
      double dt = 0.0f;
      if (g_verbose >= 1) dt = getSeconds()-t0;

      /* verbose mode */
      if (g_verbose >= 1) {
	std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
	std::cout << "  bvh4::alloc : "; bvh->alloc.print_statistics();
	std::cout << "  bvh4::alloc2: "; bvh->alloc2.print_statistics();
      }
      if (g_verbose >= 2)
	std::cout << BVH4Statistics(bvh).str();
      
      /* benchmark mode */
      if (g_benchmark) {
	BVH4Statistics stat(bvh);
	std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
      }
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    template<typename Primitive>
    size_t BVH4BezierBuilderFast<Primitive>::number_of_primitives() 
    {
      if (geom) return geom->size();
      else      return this->scene->numBezierCurves;
    }
    
    template<typename Primitive>
    void BVH4BezierBuilderFast<Primitive>::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      if (geom) PrimRefArrayGenFromGeometry<BezierCurves>::generate_sequential(threadIndex, threadCount, geom , this->prims, pinfo);
      else      PrimRefArrayGen                          ::generate_sequential(threadIndex, threadCount, this->scene, BEZIER_CURVES, 1, this->prims, pinfo);
    }

    template<typename Primitive>
    void BVH4BezierBuilderFast<Primitive>::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) 
    {
      if (geom) PrimRefArrayGenFromGeometry<BezierCurves>::generate_parallel(threadIndex, threadCount, scheduler, geom , this->prims, pinfo);
      else      PrimRefArrayGen                          ::generate_parallel(threadIndex, threadCount, scheduler, this->scene, BEZIER_CURVES, 1, this->prims, pinfo);
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    template<typename Primitive>
    size_t BVH4TriangleBuilderFast<Primitive>::number_of_primitives() 
    {
      if (geom) return geom->numTriangles;
      else      return this->scene->numTriangles;
    }
    
    template<typename Primitive>
    void BVH4TriangleBuilderFast<Primitive>::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      if (geom) PrimRefArrayGenFromGeometry<TriangleMesh>::generate_sequential(threadIndex, threadCount, geom , this->prims, pinfo);
      else      PrimRefArrayGen                          ::generate_sequential(threadIndex, threadCount, this->scene, TRIANGLE_MESH, 1, this->prims, pinfo);
    }

    template<typename Primitive>
    void BVH4TriangleBuilderFast<Primitive>::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) 
    {
      if (geom) PrimRefArrayGenFromGeometry<TriangleMesh>::generate_parallel(threadIndex, threadCount, scheduler, geom , this->prims, pinfo);
      else      PrimRefArrayGen                          ::generate_parallel(threadIndex, threadCount, scheduler, this->scene, TRIANGLE_MESH, 1, this->prims, pinfo);
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    template<typename Primitive>
    size_t BVH4UserGeometryBuilderFastT<Primitive>::number_of_primitives() 
    {
      if (geom) return geom->size();
      else      return this->scene->numUserGeometries1;
    }
    
    template<typename Primitive>
    void BVH4UserGeometryBuilderFastT<Primitive>::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      if (geom) PrimRefArrayGenFromGeometry<UserGeometryBase>::generate_sequential(threadIndex, threadCount, geom , this->prims, pinfo);
      else      PrimRefArrayGen                              ::generate_sequential(threadIndex, threadCount, this->scene, USER_GEOMETRY, 1, this->prims, pinfo);
    }

    template<typename Primitive>
    void BVH4UserGeometryBuilderFastT<Primitive>::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) 
    {
      if (geom) PrimRefArrayGenFromGeometry<UserGeometryBase>::generate_parallel(threadIndex, threadCount, scheduler, geom , this->prims, pinfo);
      else      PrimRefArrayGen                              ::generate_parallel(threadIndex, threadCount, scheduler, this->scene, USER_GEOMETRY, 1, this->prims, pinfo);
    }


    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

   template<typename Primitive>
   void BVH4SubdivBuilderFast<Primitive>::build(size_t threadIndex, size_t threadCount)
    {
      this->bvh->alloc2.reset();
      size_t numPatches = 0;
      for (size_t i=0; i<this->scene->size(); i++) 
      {
	const Geometry* geom = this->scene->get(i);
        if (geom == NULL || !geom->isEnabled()) continue;
	if (geom->type != SUBDIV_MESH) continue;
        SubdivMesh* subdiv_mesh = (SubdivMesh*)geom;
        subdiv_mesh->initializeHalfEdgeStructures();
	numPatches += subdiv_mesh->size();
      }
      BVH4BuilderFast::build(threadIndex,threadCount);
    }
 
    template<typename Primitive>
    size_t BVH4SubdivBuilderFast<Primitive>::number_of_primitives() 
    {
      if (geom) return geom->size();
      else      return this->scene->numSubdivPatches;
    }
    
    template<typename Primitive>
    void BVH4SubdivBuilderFast<Primitive>::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      if (geom) PrimRefArrayGenFromGeometry<SubdivMesh>::generate_sequential(threadIndex, threadCount, geom , this->prims, pinfo);
      else      PrimRefArrayGen                        ::generate_sequential(threadIndex, threadCount, this->scene, SUBDIV_MESH, 1, this->prims, pinfo);
    }

    template<typename Primitive>
    void BVH4SubdivBuilderFast<Primitive>::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) 
    {
      if (geom) PrimRefArrayGenFromGeometry<SubdivMesh>::generate_parallel(threadIndex, threadCount, scheduler, geom , this->prims, pinfo);
      else      PrimRefArrayGen                        ::generate_parallel(threadIndex, threadCount, scheduler, this->scene, SUBDIV_MESH, 1, this->prims, pinfo);
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    BVH4SubdivGridBuilderFast::BVH4SubdivGridBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : BVH4BuilderFastT<PrimRef>(bvh,scene,listMode,0,0,false,sizeof(Grid),1,1,true) { this->bvh->alloc2.init(4096,4096); } 

   void BVH4SubdivGridBuilderFast::build(size_t threadIndex, size_t threadCount)
   {
      /* initialize all half edge structures */
     new (&iter) Scene::Iterator<SubdivMesh>(this->scene);
     for (size_t i=0; i<iter.size(); i++)
       if (iter[i]) iter[i]->initializeHalfEdgeStructures();
     
     /* initialize allocator and parallel_for_for_prefix_sum */
     this->bvh->alloc2.reset();
     pstate.init(iter,size_t(1024));
     
     BVH4BuilderFast::build(threadIndex,threadCount);
   }

    size_t BVH4SubdivGridBuilderFast::number_of_primitives() 
    {
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        size_t s = 0;
        for (size_t f=r.begin(); f!=r.end(); ++f) 
	{
          if (!mesh->valid(f)) continue;
	  
	  feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),
					       [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4])
	  {
	    //if (!patch.isRegular()) { s++; return; }
 	    const float l0 = patch.ring[0].edge_level;
	    const float l1 = patch.ring[1].edge_level;
	    const float l2 = patch.ring[2].edge_level;
	    const float l3 = patch.ring[3].edge_level;
	    const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
	    const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
	    const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
	    const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
	    const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
	    const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
	    s += Grid::getNumEagerLeaves(pattern_x.size(),pattern_y.size());
	  });
	}
        return PrimInfo(s,empty,empty);
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });

      return pinfo.size();
    }
    
    void BVH4SubdivGridBuilderFast::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
	FastAllocator::ThreadLocal& alloc = *bvh->alloc2.threadLocal();

	PrimInfo s(empty);
        for (size_t f=r.begin(); f!=r.end(); ++f) {
          if (!mesh->valid(f)) continue;
	  
	  feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),
					       [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4])
	  {
	    /*if (!patch.isRegular())
	    {
	      Grid* leaf = (Grid*) bvh->alloc2.malloc(sizeof(Grid),16);
	      new (leaf) Grid(id,mesh->id,f);3
	      const BBox3fa bounds = leaf->quad(scene,patch,uv[0],uv[1],uv[2],uv[3]);
	      prims[base.size()+s.size()] = PrimRef(bounds,BVH4::encodeTypedLeaf(leaf,0));
	      s.add(bounds);
	      return;
	      }*/

	    const float l0 = patch.ring[0].edge_level;
	    const float l1 = patch.ring[1].edge_level;
	    const float l2 = patch.ring[2].edge_level;
	    const float l3 = patch.ring[3].edge_level;
	    const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
	    const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
	    const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
	    const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
	    const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
	    const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
	    const int nx = pattern_x.size();
	    const int ny = pattern_y.size();

	    GregoryPatch patcheval; patcheval.init(patch);
	    //BSplinePatch patcheval; patcheval.init(patch);
            size_t N = Grid::createEager(mesh->id,f,scene,patcheval,alloc,&prims[base.size()+s.size()],0,nx,0,ny,uv,pattern0,pattern1,pattern2,pattern3,pattern_x,pattern_y);
	    assert(N == Grid::getNumEagerLeaves(nx,ny));
	    for (size_t i=0; i<N; i++)
	      s.add(prims[base.size()+s.size()].bounds());
	  });
        }
        return s;
	  }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
    }
    
    void BVH4SubdivGridBuilderFast::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) {
      create_primitive_array_sequential(threadIndex, threadCount, pinfo);  // FIXME: parallelize
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    BVH4SubdivGridEagerBuilderFast::BVH4SubdivGridEagerBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : BVH4BuilderFastT<PrimRef>(bvh,scene,listMode,0,0,false,0,1,1,true) { this->bvh->alloc2.init(4096,4096); } 

   void BVH4SubdivGridEagerBuilderFast::build(size_t threadIndex, size_t threadCount)
   {
      /* initialize all half edge structures */
     new (&iter) Scene::Iterator<SubdivMesh>(this->scene);
     for (size_t i=0; i<iter.size(); i++)
       if (iter[i]) iter[i]->initializeHalfEdgeStructures();

     /* initialize allocator and parallel_for_for_prefix_sum */
     this->bvh->alloc2.reset();
     pstate.init(iter,size_t(1024));

     BVH4BuilderFast::build(threadIndex,threadCount);
   }

    size_t BVH4SubdivGridEagerBuilderFast::number_of_primitives() 
    {
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        size_t s = 0;
        for (size_t f=r.begin(); f!=r.end(); ++f) 
	{
          if (!mesh->valid(f)) continue;
	  
	  feature_adaptive_subdivision_eval(mesh->getHalfEdge(f),mesh->getVertexBuffer(),
					    [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4], const int id)
	  {
 	    const float l0 = patch.ring[0].edge_level;
	    const float l1 = patch.ring[1].edge_level;
	    const float l2 = patch.ring[2].edge_level;
	    const float l3 = patch.ring[3].edge_level;
	    const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
	    const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
	    const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
	    const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
	    const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
	    const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
	    s += Grid::getNumEagerLeaves(pattern_x.size(),pattern_y.size());
	  });
	}
        return PrimInfo(s,empty,empty);
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });

      return pinfo.size();
    }
    
    void BVH4SubdivGridEagerBuilderFast::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
	FastAllocator::ThreadLocal& alloc = *bvh->alloc2.threadLocal();

	PrimInfo s(empty);
        for (size_t f=r.begin(); f!=r.end(); ++f) {
          if (!mesh->valid(f)) continue;
	  
	  feature_adaptive_subdivision_eval(mesh->getHalfEdge(f),mesh->getVertexBuffer(),
					    [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4], const int id)
	  {
	    const float l0 = patch.ring[0].edge_level;
	    const float l1 = patch.ring[1].edge_level;
	    const float l2 = patch.ring[2].edge_level;
	    const float l3 = patch.ring[3].edge_level;
	    const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
	    const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
	    const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
	    const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
	    const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
	    const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
	    const int nx = pattern_x.size();
	    const int ny = pattern_y.size();
	    size_t N = Grid::createEager(mesh->id,f,scene,patch,alloc,&prims[base.size()+s.size()],0,nx,0,ny,uv,pattern0,pattern1,pattern2,pattern3,pattern_x,pattern_y);
	    assert(N == Grid::getNumEagerLeaves(nx,ny));
	    for (size_t i=0; i<N; i++)
	      s.add(prims[base.size()+s.size()].bounds());
	  });
        }
        return s;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
    }
    
    void BVH4SubdivGridEagerBuilderFast::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) {
      create_primitive_array_sequential(threadIndex, threadCount, pinfo);  // FIXME: parallelize
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    BVH4SubdivGridLazyBuilderFast::BVH4SubdivGridLazyBuilderFast (BVH4* bvh, Scene* scene, size_t listMode) 
      : BVH4BuilderFastT<Grid::LazyLeaf*>(bvh,scene,listMode,0,0,false,0,1,1,true) { this->bvh->alloc2.init(4096,4096); } 

    template<>
    void BVH4BuilderFastT<Grid::LazyLeaf*>::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      assert(current.size() == 1);
      BVH4::NodeRef node = prims[current.begin].ID();
      size_t ty = 0;
      Grid::LazyLeaf* leaf = (Grid::LazyLeaf*) node.leaf(ty);
      leaf->parent = current.parent;
      *current.parent = node;
      assert(ty == 1);
    }

    void BVH4SubdivGridLazyBuilderFast::build(size_t threadIndex, size_t threadCount)
    {
      /* initialize all half edge structures */
      new (&iter) Scene::Iterator<SubdivMesh>(this->scene);
      for (size_t i=0; i<iter.size(); i++)
	if (iter[i]) iter[i]->initializeHalfEdgeStructures();
      
      /* initialize allocator and parallel_for_for_prefix_sum */
      this->bvh->alloc2.reset();
      pstate.init(iter,size_t(1024));
      
      BVH4BuilderFast::build(threadIndex,threadCount);
    }

    size_t BVH4SubdivGridLazyBuilderFast::number_of_primitives() 
    {
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        size_t s = 0;
        for (size_t f=r.begin(); f!=r.end(); ++f) 
	{
          if (!mesh->valid(f)) continue;
	  
	  feature_adaptive_subdivision_eval(mesh->getHalfEdge(f),mesh->getVertexBuffer(),
					    [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4], const int id)
	  {
 	    const float l0 = patch.ring[0].edge_level;
	    const float l1 = patch.ring[1].edge_level;
	    const float l2 = patch.ring[2].edge_level;
	    const float l3 = patch.ring[3].edge_level;
	    const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
	    const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
	    const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
	    const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
	    const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
	    const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
	    s += Grid::getNumLazyLeaves(pattern_x.size(),pattern_y.size());
	  });
	}
        return PrimInfo(s,empty,empty);
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });

      return pinfo.size();
    }
    
    void BVH4SubdivGridLazyBuilderFast::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
	FastAllocator::ThreadLocal& alloc = *bvh->alloc2.threadLocal();
	
	PrimInfo s(empty);
        for (size_t f=r.begin(); f!=r.end(); ++f) {
          if (!mesh->valid(f)) continue;
	  
	  feature_adaptive_subdivision_eval(mesh->getHalfEdge(f),mesh->getVertexBuffer(),
					    [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4], const int id)
	  {
	    const float l0 = patch.ring[0].edge_level;
	    const float l1 = patch.ring[1].edge_level;
	    const float l2 = patch.ring[2].edge_level;
	    const float l3 = patch.ring[3].edge_level;
	    const DiscreteTessellationPattern pattern0(l0,subdiv[0]);
	    const DiscreteTessellationPattern pattern1(l1,subdiv[1]);
	    const DiscreteTessellationPattern pattern2(l2,subdiv[2]);
	    const DiscreteTessellationPattern pattern3(l3,subdiv[3]);
	    const DiscreteTessellationPattern pattern_x = pattern0.size() > pattern2.size() ? pattern0 : pattern2;
	    const DiscreteTessellationPattern pattern_y = pattern1.size() > pattern3.size() ? pattern1 : pattern3;
	    const int nx = pattern_x.size();
	    const int ny = pattern_y.size();
	    size_t N = Grid::createLazy(bvh,NULL,mesh->id,f,id,nx,ny,alloc,&prims[base.size()+s.size()]);
	    assert(N == Grid::getNumLazyLeaves(nx,ny));
	    for (size_t i=0; i<N; i++)
	      s.add(prims[base.size()+s.size()].bounds());
	  });
        }
        return s;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
    }
    
    void BVH4SubdivGridLazyBuilderFast::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) {
      create_primitive_array_sequential(threadIndex, threadCount, pinfo);  // FIXME: parallelize
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

#define DBG_CACHE_BUILDER(x) 

    BBox3fa getBounds(const SubdivPatch1Base &p, const SubdivMesh* const mesh)
    {
#if FORCE_TESSELLATION_BOUNDS == 1
      
#if !defined(_MSC_VER) || defined(__INTEL_COMPILER)
      __aligned(64) float u_array[(p.grid_size_simd_blocks + 1) * 16]; // +16 for unaligned access
      __aligned(64) float v_array[(p.grid_size_simd_blocks + 1) * 16]; // +16 for unaligned access

#else
      const size_t array_elements = (p.grid_size_simd_blocks + 1) * 8;

      float *const ptr = (float*)_malloca(2 *array_elements * sizeof(float) + 64);
      float *const uv_arrays = (float*)ALIGN_PTR(ptr, 64);
      float *const u_array = &uv_arrays[array_elements*0];
      float *const v_array = &uv_arrays[array_elements*1];
#endif

      gridUVTessellator(p.level, p.grid_u_res, p.grid_v_res, u_array, v_array);

      if (unlikely(p.needsStiching()))
        stichUVGrid(p.level, p.grid_u_res, p.grid_v_res, u_array, v_array);

      const unsigned int real_grid_size = p.grid_u_res*p.grid_v_res;
      for (size_t i = real_grid_size; i<p.grid_size_simd_blocks * 16; i++)
          {
            u_array[i] = 1.0f;
            v_array[i] = 1.0f;
          }

      
      BBox3fa b(empty);
      assert(p.grid_size_simd_blocks >= 1);

#if !defined(__AVX__)

      for (size_t i = 0; i<p.grid_size_simd_blocks * 2; i++)
        {
          ssef u = load4f(&u_array[i * 4]);
          ssef v = load4f(&v_array[i * 4]);

          sse3f vtx = p.eval4(u, v);

          /* eval displacement function */
          if (unlikely(mesh->displFunc != NULL))
            {
              const Vec2f uv0 = p.getUV(0);
              const Vec2f uv1 = p.getUV(1);
              const Vec2f uv2 = p.getUV(2);
              const Vec2f uv3 = p.getUV(3);

              const ssef patch_uu = bilinear_interpolate(uv0.x, uv1.x, uv2.x, uv3.x, u, v);
              const ssef patch_vv = bilinear_interpolate(uv0.y, uv1.y, uv2.y, uv3.y, u, v);

              sse3f nor = p.normal4(u, v);

              nor = normalize_safe(nor);

              mesh->displFunc(mesh->userPtr,
                              p.geom,
                              p.prim,
                              (const float*)&patch_uu,
                              (const float*)&patch_vv,
                              (const float*)&nor.x,
                              (const float*)&nor.y,
                              (const float*)&nor.z,
                              (float*)&vtx.x,
                              (float*)&vtx.y,
                              (float*)&vtx.z,
                              4);
            }
          b.extend(getBBox3fa(vtx));
        }

#else

      for (size_t i = 0; i<p.grid_size_simd_blocks; i++)
        {
          avxf u = load8f(&u_array[i * 8]);
          avxf v = load8f(&v_array[i * 8]);          
          avx3f vtx = p.eval8(u, v);

          /* eval displacement function */
          if (unlikely(mesh->displFunc != NULL))
            {
              const Vec2f uv0 = p.getUV(0);
              const Vec2f uv1 = p.getUV(1);
              const Vec2f uv2 = p.getUV(2);
              const Vec2f uv3 = p.getUV(3);

              const avxf patch_uu = bilinear_interpolate(uv0.x, uv1.x, uv2.x, uv3.x, u, v);
              const avxf patch_vv = bilinear_interpolate(uv0.y, uv1.y, uv2.y, uv3.y, u, v);

              avx3f nor = p.normal8(u, v);

              nor = normalize_safe(nor);

              mesh->displFunc(mesh->userPtr,
                              p.geom,
                              p.prim,
                              (const float*)&patch_uu,
                              (const float*)&patch_vv,
                              (const float*)&nor.x,
                              (const float*)&nor.y,
                              (const float*)&nor.z,
                              (float*)&vtx.x,
                              (float*)&vtx.y,
                              (float*)&vtx.z,
                              8);
            }
          b.extend(getBBox3fa(vtx));
        }
#endif

      b.lower.a = 0.0f;
      b.upper.a = 0.0f;

#if defined(DEBUG)
      isfinite(b.lower.x);
      isfinite(b.lower.y);
      isfinite(b.lower.z);

      isfinite(b.upper.x);
      isfinite(b.upper.y);
      isfinite(b.upper.z);
#endif

#else
      BBox3fa b = patch.bounds();
      if (unlikely(isGregoryPatch()))
        {
          b.extend(GregoryPatch::extract_f_m_Vec3fa(patch.v, 0));
          b.extend(GregoryPatch::extract_f_m_Vec3fa(patch.v, 1));
          b.extend(GregoryPatch::extract_f_m_Vec3fa(patch.v, 2));
          b.extend(GregoryPatch::extract_f_m_Vec3fa(patch.v, 3));
        }
#endif

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
      _freea(ptr);
#endif
      return b;
    }

    void BVH4SubdivPatch1CachedBuilderFast::build(size_t threadIndex, size_t threadCount)
    {
      fastUpdateMode = true;
      fastUpdateMode_numFaces = 0;

      /* initialize all half edge structures */
      new (&iter) Scene::Iterator<SubdivMesh>(this->scene);
      for (size_t i=0; i<iter.size(); i++)
        if (iter[i]) 
          {
            iter[i]->initializeHalfEdgeStructures();
            fastUpdateMode_numFaces += iter[i]->size();
            if (!iter[i]->checkLevelUpdate()) fastUpdateMode = false;
          }
      DBG_CACHE_BUILDER( DBG_PRINT( fastUpdateMode_numFaces ) );

      pstate.init(iter,size_t(1024));

      this->bvh->scene = this->scene; // FIXME: remove

      /* deactivate fast update mode */
      if (bvh->numPrimitives == 0 || 
          bvh->numPrimitives != fastUpdateMode_numFaces ||
          bvh->root          == BVH4::emptyNode)
        fastUpdateMode = false;

      //fastUpdateMode = false;

      /* force sequential code path for fast update */
      if (fastUpdateMode)
        needAllThreads = false;

      BVH4BuilderFast::build(threadIndex,threadCount);
    }

    size_t BVH4SubdivPatch1CachedBuilderFast::number_of_primitives() 
    {
      /* in fast update mode we know the number of primitives in advance */
      if (fastUpdateMode) return fastUpdateMode_numFaces;

      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        size_t s = 0;
        for (size_t f=r.begin(); f!=r.end(); ++f) 
        {          
          if (!mesh->valid(f)) continue;
          
          feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),
                                               [&](const CatmullClarkPatch& patch, const Vec2f uv[4], const int subdiv[4])
                                               {
                                                 s++;
                                               });
        }
        return PrimInfo(s,empty,empty);
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo(a.size()+b.size(),empty,empty); });

      DBG_CACHE_BUILDER( DBG_PRINT(fastUpdateMode) );
      DBG_CACHE_BUILDER( DBG_PRINT(pinfo) );
      DBG_CACHE_BUILDER( DBG_PRINT(needAllThreads) );
      return pinfo.size();
    }

    void debugGridBorders(const SubdivPatch1Base &patch,
                          const SubdivMesh* const geom)
    {
      assert( patch.grid_size_simd_blocks >= 1 );
#if !defined(_MSC_VER) || defined(__INTEL_COMPILER)
      __aligned(64) float grid_x[(patch.grid_size_simd_blocks+1)*8]; 
      __aligned(64) float grid_y[(patch.grid_size_simd_blocks+1)*8];
      __aligned(64) float grid_z[(patch.grid_size_simd_blocks+1)*8]; 
        
      __aligned(64) float grid_u[(patch.grid_size_simd_blocks+1)*8]; 
      __aligned(64) float grid_v[(patch.grid_size_simd_blocks+1)*8];
     
#else
      const size_t array_elements = (patch.grid_size_simd_blocks + 1) * 8;
      float *const ptr = (float*)_malloca(5 * array_elements * sizeof(float) + 64);
      float *const grid_arrays = (float*)ALIGN_PTR(ptr,64);

      float *grid_x = &grid_arrays[array_elements * 0];
      float *grid_y = &grid_arrays[array_elements * 1];
      float *grid_z = &grid_arrays[array_elements * 2];
      float *grid_u = &grid_arrays[array_elements * 3];
      float *grid_v = &grid_arrays[array_elements * 4];

        
#endif   

      DBG_PRINT( patch.grid_size_simd_blocks );

      evalGrid(patch,grid_x,grid_y,grid_z,grid_u,grid_v,geom);

      DBG_PRINT(patch.grid_u_res);
      DBG_PRINT(patch.grid_v_res);
      
      DBG_PRINT("top");
      for (size_t x=0;x<patch.grid_u_res;x++)
        {
          const size_t offset = patch.gridOffset(0,x);
          std::cout << x << " -> " << Vec2f(grid_u[offset],grid_v[offset]) << " ";
          std::cout << " / ";
          std::cout << Vec3f(grid_x[offset],grid_y[offset],grid_z[offset]) << " ";
          std::cout << std::endl;
        }

      DBG_PRINT("right");
      for (size_t y=0;y<patch.grid_v_res;y++)
        {
          const size_t offset = patch.gridOffset(y,patch.grid_u_res-1);
          std::cout << y << " -> " << Vec2f(grid_u[offset],grid_v[offset]) << " ";
          std::cout << " / ";
          std::cout << Vec3f(grid_x[offset],grid_y[offset],grid_z[offset]) << " ";
          std::cout << std::endl;
        }

      DBG_PRINT("buttom");
      for (ssize_t x=patch.grid_u_res-1;x>=0;x--)
        {
          const size_t offset = patch.gridOffset(patch.grid_v_res-1,x);
          std::cout << x << " -> " << Vec2f(grid_u[offset],grid_v[offset]) << " ";
          std::cout << " / ";
          std::cout << Vec3f(grid_x[offset],grid_y[offset],grid_z[offset]) << " ";
          std::cout << std::endl;
        }


      DBG_PRINT("left");
      for (ssize_t y=patch.grid_v_res-1;y>=0;y--)
        {
          const size_t offset = patch.gridOffset(y,0);
          std::cout << y << " -> " << Vec2f(grid_u[offset],grid_v[offset]) << " ";
          std::cout << " / ";
          std::cout << Vec3f(grid_x[offset],grid_y[offset],grid_z[offset]) << " ";
          std::cout << std::endl;
        }

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
      _freea(ptr);
#endif      
    }
    
    void BVH4SubdivPatch1CachedBuilderFast::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      /* Allocate memory for gregory and b-spline patches */
     if (this->bvh->size_data_mem < sizeof(SubdivPatch1Cached) * numPrimitives) 
        {
         DBG_CACHE_BUILDER(std::cout << "DEALLOCATING SUBDIVPATCH1CACHED MEMORY" << std::endl);
          if (this->bvh->data_mem) 
            os_free( this->bvh->data_mem, this->bvh->size_data_mem );
          this->bvh->data_mem      = NULL;
          this->bvh->size_data_mem = 0;
        }

     if (bvh->data_mem == NULL)
       {
         DBG_CACHE_BUILDER(std::cout << "ALLOCATING SUBDIVPATCH1CACHED MEMORY FOR " << numPrimitives << " PRIMITIVES" << std::endl);
         this->bvh->size_data_mem = sizeof(SubdivPatch1Cached) * numPrimitives;
         this->bvh->data_mem      = os_malloc( this->bvh->size_data_mem );        
       }
        
      SubdivPatch1Cached *const subdiv_patches = (SubdivPatch1Cached *)this->bvh->data_mem;

      double t0 = getSeconds();

      pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](SubdivMesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo s(empty);
        for (size_t f=r.begin(); f!=r.end(); ++f) 
	{
          if (!mesh->valid(f)) continue;

	  if (unlikely(fastUpdateMode == false))
            {
            
              feature_adaptive_subdivision_gregory(f,mesh->getHalfEdge(f),mesh->getVertexBuffer(),[&](const CatmullClarkPatch& ipatch, const Vec2f uv[4], const int subdiv[4])
                                                   {
                                                     //std::cout.precision(12);

                                                     float edge_level[4] = {
                                                       ipatch.ring[0].edge_level,
                                                       ipatch.ring[1].edge_level,
                                                       ipatch.ring[2].edge_level,
                                                       ipatch.ring[3].edge_level
                                                     };
              
                                                     for (size_t i=0;i<4;i++)
                                                       edge_level[i] = adjustDiscreteTessellationLevel(edge_level[i],subdiv[i]);
              
                                                     const unsigned int patchIndex = base.size()+s.size();
                                                     assert(patchIndex < numPrimitives);
                                                     subdiv_patches[patchIndex] = SubdivPatch1Cached(ipatch, mesh->id, f, mesh, uv, edge_level);
              
                                                     /* compute patch bounds */
                                                     const BBox3fa bounds = getBounds(subdiv_patches[patchIndex],mesh);
                                                     assert(bounds.lower.x <= bounds.upper.x);
                                                     assert(bounds.lower.y <= bounds.upper.y);
                                                     assert(bounds.lower.z <= bounds.upper.z);
              
                                                     prims[patchIndex] = PrimRef(bounds,patchIndex);
                                                     s.add(bounds);
                                                   });
            }
	  else
            {
	      const SubdivMesh::HalfEdge* first_half_edge = mesh->getHalfEdge(f);

	      float edge_level[4] = {
		first_half_edge[0].edge_level,
		first_half_edge[1].edge_level,
		first_half_edge[2].edge_level,
		first_half_edge[3].edge_level
	      };
              
	      const unsigned int patchIndex = base.size()+s.size();
              
	      subdiv_patches[patchIndex].updateEdgeLevels(edge_level,mesh);
              
	      /* compute patch bounds */
	      const BBox3fa bounds = getBounds(subdiv_patches[patchIndex],mesh);
              if (bounds.lower.x > bounds.upper.x)
                DBG_PRINT(bounds);
	      assert(bounds.lower.x <= bounds.upper.x);
	      assert(bounds.lower.y <= bounds.upper.y);
	      assert(bounds.lower.z <= bounds.upper.z);
	      
	      prims[patchIndex] = PrimRef(bounds,patchIndex);
	      s.add(bounds);	      
	    }
        }
        return s;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a, b); });

      
      t0 = getSeconds()-t0;
      DBG_CACHE_BUILDER(std::cout << "create prims in " << 1000.0f*t0 << "ms " << std::endl);
    }
    
    void BVH4SubdivPatch1CachedBuilderFast::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) {
      create_primitive_array_sequential(threadIndex, threadCount, pinfo);  // FIXME: parallelize
    }

    void BVH4SubdivPatch1CachedBuilderFast::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      assert(items <= 1);
      const unsigned int patchIndex = prims[current.begin].ID();
      SubdivPatch1Cached *const subdiv_patches = (SubdivPatch1Cached *)this->bvh->data_mem;
      *current.parent = bvh->encodeLeaf((char*)&subdiv_patches[patchIndex],1);
    }

    void BVH4SubdivPatch1CachedBuilderFast::build_sequential(size_t threadIndex, size_t threadCount)
    {
      if (fastUpdateMode)
        {
          double t0 = 0.0;

          t0 = getSeconds();

          /* calculate list of primrefs */
          PrimInfo pinfo(empty);
          create_primitive_array_parallel(threadIndex, threadCount, scheduler, pinfo);
          bvh->bounds = pinfo.geomBounds;

          t0 = getSeconds()-t0;
          DBG_CACHE_BUILDER(std::cout << "create prim refs in " << 1000.0f*t0 << "ms " << std::endl);

          t0 = getSeconds();
          if (bvh->root != BVH4::emptyNode)
            refit(bvh->root);
          t0 = getSeconds()-t0;

          /* verbose mode */
          DBG_CACHE_BUILDER(std::cout << "Refit done in " << 1000.0f*t0 << "ms " << std::endl);
        }
      else       
        BVH4BuilderFast::build_sequential(threadIndex,threadCount);
    }

    BBox3fa BVH4SubdivPatch1CachedBuilderFast::refit(NodeRef& ref)
    {
      /* this is a empty node */
      if (unlikely(ref == BVH4::emptyNode))
        return BBox3fa( empty );

      assert(ref != BVH4::invalidNode);

      /* this is a leaf node */
      if (unlikely(ref.isLeaf()))
        {
          size_t num;
          SubdivPatch1Cached *sptr = (SubdivPatch1Cached*)ref.leaf(num);
          const size_t index = ((size_t)sptr - (size_t)this->bvh->data_mem) / sizeof(SubdivPatch1Cached);
          assert(index < numPrimitives);
          return prims[index].bounds(); 
        }
      
      /* recurse if this is an internal node */
      Node* node = ref.node();
      const BBox3fa bounds0 = refit(node->child(0));
      const BBox3fa bounds1 = refit(node->child(1));
      const BBox3fa bounds2 = refit(node->child(2));
      const BBox3fa bounds3 = refit(node->child(3));
      
      /* AOS to SOA transform */
      BBox<sse3f> bounds;
      transpose((ssef&)bounds0.lower,(ssef&)bounds1.lower,(ssef&)bounds2.lower,(ssef&)bounds3.lower,bounds.lower.x,bounds.lower.y,bounds.lower.z);
      transpose((ssef&)bounds0.upper,(ssef&)bounds1.upper,(ssef&)bounds2.upper,(ssef&)bounds3.upper,bounds.upper.x,bounds.upper.y,bounds.upper.z);
      
      /* set new bounds */
      node->lower_x = bounds.lower.x;
      node->lower_y = bounds.lower.y;
      node->lower_z = bounds.lower.z;
      node->upper_x = bounds.upper.x;
      node->upper_y = bounds.upper.y;
      node->upper_z = bounds.upper.z;
      
      /* return merged bounds */
      const float lower_x = reduce_min(bounds.lower.x);
      const float lower_y = reduce_min(bounds.lower.y);
      const float lower_z = reduce_min(bounds.lower.z);
      const float upper_x = reduce_max(bounds.upper.x);
      const float upper_y = reduce_max(bounds.upper.y);
      const float upper_z = reduce_max(bounds.upper.z);
      return BBox3fa(Vec3fa(lower_x,lower_y,lower_z),
                    Vec3fa(upper_x,upper_y,upper_z));
    }



    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    size_t BVH4TopLevelBuilderFastT::number_of_primitives() {
      return N;
    }
    
    void BVH4TopLevelBuilderFastT::create_primitive_array_sequential(size_t threadIndex, size_t threadCount, PrimInfo& pinfo)
    {
      for (size_t i=0; i<N; i++) {
	pinfo.add(prims_i[i].bounds(),prims_i[i].center2());
	prims[i] = prims_i[i];
      }
    }

    void BVH4TopLevelBuilderFastT::create_primitive_array_parallel  (size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimInfo& pinfo) 
    {
      for (size_t i=0; i<N; i++) {
	pinfo.add(prims_i[i].bounds(),prims_i[i].center2());
	prims[i] = prims_i[i];
      }
    }
 
    void BVH4TopLevelBuilderFastT::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = current.size();
      assert(items <= 1);
      *current.parent = (BVH4::NodeRef) prims[current.begin].ID();
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    template<typename Primitive>
    void BVH4BuilderFastT<Primitive>::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      size_t items = Primitive::blocks(current.size());
      size_t start = current.begin;
            
      /* allocate leaf node */
      Primitive* accel = (Primitive*) leafAlloc.malloc(items*sizeof(Primitive));
      *current.parent = bvh->encodeLeaf((char*)accel,listMode ? listMode : items);
      
      for (size_t i=0; i<items; i++) 
	accel[i].fill(prims,start,current.end,scene,listMode);
    }

    template<>
    void BVH4BuilderFastT<PrimRef>::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID) {
      if (current.size() != 1) THROW_RUNTIME_ERROR("bvh4_builder_fast: internal error");
      *current.parent = (BVH4::NodeRef) prims[current.begin].ID();
    }

    void BVH4BuilderFast::createLeaf(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, size_t threadIndex, size_t threadCount)
    {
      if (current.depth > BVH4::maxBuildDepthLeaf) 
        THROW_RUNTIME_ERROR("depth limit reached");
      
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
      const float sah = state->parallelBinner.find(pinfo,prims,tmp,logBlockSize,threadID,numThreads,scheduler);

      /* if we cannot find a valid split, enforce an arbitrary split */
      if (unlikely(sah == float(inf))) splitFallback(prims,current,leftChild,rightChild);
      
      /* parallel partitioning of items */
      else 
        {
          state->parallelBinner.partition(pinfo,tmp,prims,leftChild,rightChild,threadID,numThreads,scheduler);
        }
    }
    
    __forceinline void BVH4BuilderFast::split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads)
    {
      if (mode == BUILD_TOP_LEVEL) splitParallel(current,left,right,threadID,numThreads);		  
      else                         splitSequential(current,left,right,threadID,numThreads);
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    __forceinline void BVH4BuilderFast::recurse_continue(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, const size_t mode, const size_t threadID, const size_t numThreads)
    {
      if (mode == BUILD_TOP_LEVEL) {
      	state->heap.push(current);
      }
      else if (mode == RECURSE_PARALLEL && current.size() > THRESHOLD_FOR_SUBTREE_RECURSION) {
        if (!state->threadStack[threadID].push(current))
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
    
    void BVH4BuilderFast::buildSubTrees(size_t threadID, size_t numThreads)
    {
      __aligned(64) Allocator nodeAlloc(&bvh->alloc);
      __aligned(64) Allocator leafAlloc(&bvh->alloc);
      
      while (true) 
      {
	BuildRecord br;
	if (!state->heap.pop(br))
        {
          /* global work queue empty => try to steal from neighboring queues */	  
          bool success = false;
          for (size_t i=0; i<numThreads; i++)
          {
            if (state->threadStack[(threadID+i)%numThreads].pop(br)) {
              success = true;
              break;
            }
          }
          /* found nothing to steal ? */
          if (!success) break; // FIXME: may loose threads
        }
        
        /* process local work queue */
	recurse(br,nodeAlloc,leafAlloc,RECURSE_PARALLEL,threadID,numThreads);
	while (state->threadStack[threadID].pop(br))
          recurse(br,nodeAlloc,leafAlloc,RECURSE_PARALLEL,threadID,numThreads);
      }
      _mm_sfence(); // make written leaves globally visible
    }

    void BVH4BuilderFast::build_sequential(size_t threadIndex, size_t threadCount) 
    {      
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
      br.init(pinfo,0,pinfo.size());
      br.depth = 1;
      br.parent = &bvh->root;

      /* build BVH in single thread */
      recurse(br,nodeAlloc,leafAlloc,RECURSE_SEQUENTIAL,threadIndex,threadCount);
      _mm_sfence(); // make written leaves globally visible
    }

    void BVH4BuilderFast::build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount) 
    {
      //double T0 = getSeconds();

      /* calculate list of primrefs */
      PrimInfo pinfo(empty);
      create_primitive_array_parallel(threadIndex, threadCount, scheduler, pinfo);
      bvh->bounds = pinfo.geomBounds;

      /* initialize node and leaf allocator */
      bvh->alloc.clear();
      __aligned(64) Allocator nodeAlloc(&bvh->alloc);
      __aligned(64) Allocator leafAlloc(&bvh->alloc);

      /* create initial build record */
      BuildRecord br;
      br.init(pinfo,0,pinfo.size());
      br.depth = 1;
      br.parent = &bvh->root;
      
      /* initialize thread-local work stacks */
      for (size_t i=0; i<threadCount; i++)
        state->threadStack[i].reset();
      
      /* push initial build record to global work stack */
      state->heap.reset();
      state->heap.push(br);

      //double T1 = getSeconds();

      /* work in multithreaded toplevel mode until sufficient subtasks got generated */
      while (state->heap.size() < 2*threadCount)
      {
        BuildRecord br;

        /* pop largest item for better load balancing */
	if (!state->heap.pop(br)) 
          break;
        
        /* guarantees to create no leaves in this stage */
        if (br.size() <= max(minLeafSize,THRESHOLD_FOR_SINGLE_THREADED)) {
	  state->heap.push(br);
          break;
	}

        recurse(br,nodeAlloc,leafAlloc,BUILD_TOP_LEVEL,threadIndex,threadCount);
      }
      _mm_sfence(); // make written leaves globally visible

      std::sort(state->heap.begin(),state->heap.end(),BuildRecord::Greater());
      

      /* now process all created subtasks on multiple threads */
      scheduler->dispatchTask(task_buildSubTrees, this, threadIndex, threadCount );
    
      //double T2 = getSeconds();

      //PRINT(1000.0f*(T1-T0));
      //PRINT(1000.0f*(T2-T1));
    }


    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    BVH4BuilderFastGeneric::BVH4BuilderFastGeneric (BVH4* bvh, PrimRef* prims, size_t N, const MakeLeaf& makeLeaf, size_t listMode, 
                                                    size_t logBlockSize, size_t logSAHBlockSize, bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize)
      : makeLeaf(makeLeaf), BVH4BuilderFast(NULL,bvh,listMode,logBlockSize,logSAHBlockSize,needVertices,primBytes,minLeafSize,maxLeafSize)
    {
      this->prims = prims;
      this->N = N;
    }

    BVH4BuilderFastGeneric::~BVH4BuilderFastGeneric () {
      prims = NULL;
    }

    void BVH4BuilderFastGeneric::build(size_t threadIndex, size_t threadCount)
    {
      bvh->init(sizeof(BVH4::Node),N*N,1);
      
      /* initialize node and leaf allocator */
      bvh->alloc.clear();
      __aligned(64) Allocator nodeAlloc(&bvh->alloc);
      __aligned(64) Allocator leafAlloc(&bvh->alloc);
     
      /* calculate bounding box */
      PrimInfo pinfo(empty);
      for (size_t i=0; i<N; i++) pinfo.add(prims[i].bounds());
      bvh->bounds = pinfo.geomBounds;

      /* create initial build record */
      BuildRecord br;
      br.init(pinfo,0,pinfo.size());
      br.depth = 1;
      br.parent = &bvh->root;

      /* build BVH in single thread */
      recurse(br,nodeAlloc,leafAlloc,RECURSE_SEQUENTIAL,threadIndex,threadCount);
      _mm_sfence(); // make written leaves globally visible
    }

    void BVH4BuilderFastGeneric::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID) 
    {
      *current.parent = makeLeaf(leafAlloc, &prims[current.begin], current.size());
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    static __forceinline bool compare_x(const Vec3fa& v0, const Vec3fa& v1) { return v0.x < v1.x; }
    static __forceinline bool compare_y(const Vec3fa& v0, const Vec3fa& v1) { return v0.y < v1.y; }
    static __forceinline bool compare_z(const Vec3fa& v0, const Vec3fa& v1) { return v0.z < v1.z; }
    
    template<typename Primitive>
    void BVH4TriangleBuilderFastSweep<Primitive>::build_sequential(size_t threadIndex, size_t threadCount) 
    {
      PING;
      BVH4 *bvh = BVH4TriangleBuilderFast<Primitive>::bvh;
      /* initialize node and leaf allocator */
      bvh->alloc.clear();
      __aligned(64) Allocator nodeAlloc(&bvh->alloc);
      __aligned(64) Allocator leafAlloc(&bvh->alloc);
     
      /* create prim refs */
      PrimInfo pinfo(empty);
      BVH4TriangleBuilderFast<Primitive>::create_primitive_array_sequential(threadIndex, threadCount, pinfo);
      bvh->bounds = pinfo.geomBounds;

      DBG_PRINT(pinfo.size());
      
      /* for sweep builder */
      for (size_t i=0;i<3;i++)
        centroids[i] = (Vec3fa*)_mm_malloc(sizeof(Vec3fa)*pinfo.size(),64);
      tmp = (float*) _mm_malloc(sizeof(float) *pinfo.size(),64);
      
      for (size_t i=0;i<pinfo.size();i++)
        {
          Vec3fa centroid = BVH4TriangleBuilderFast<Primitive>::prims[i].center2() * 0.5;
          centroid.a = i;
          centroids[0][i] = centroid;
          centroids[1][i] = centroid;
          centroids[2][i] = centroid;          
        }
      std::sort(centroids[0],centroids[0]+pinfo.size(),compare_x);
      std::sort(centroids[1],centroids[1]+pinfo.size(),compare_y);
      std::sort(centroids[2],centroids[2]+pinfo.size(),compare_z);      
      
      /* create initial build record */
      BuildRecord br;
      br.init(pinfo,0,pinfo.size());
      br.depth = 1;
      br.parent = &bvh->root;

      /* build BVH in single thread */

      recurse_sweep(br,nodeAlloc,leafAlloc,threadIndex,threadCount);
      DBG_PRINT("HERE");
      exit(0);
     
      _mm_sfence(); // make written leaves globally visible

      _mm_free(tmp);
      for (size_t i=0;i<3;i++)
        _mm_free(centroids[i]);
    }


    template<typename Primitive>
    void BVH4TriangleBuilderFastSweep<Primitive>::recurse_sweep(BuildRecord& current,
                                                                Allocator& nodeAlloc,
                                                                Allocator& leafAlloc,
                                                                const size_t threadID,
                                                                const size_t numThreads)
    {
      PING;
      __aligned(64) BuildRecord children[BVH4::N];
      /* create leaf node */
      if (current.depth >= BVH4::maxBuildDepth ||
          current.size() <= BVH4TriangleBuilderFast<Primitive>::minLeafSize)
        {
          BVH4TriangleBuilderFast<Primitive>::createLeaf(current,nodeAlloc,leafAlloc,threadID,numThreads);
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
        //splitSequential_sweep(children[bestChild],left,right,centroids_x,centroids_y,centroids_z,tmp,threadID,numThreads);
        
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
        BVH4TriangleBuilderFast<Primitive>::createLeaf(current,nodeAlloc,leafAlloc,threadID,numThreads);
        return;
      }
      
      /* allocate node */
      Node* node = (Node*) nodeAlloc.malloc(sizeof(Node));
      node->clear();
      *current.parent = bvh->encodeNode(node);
      
      /* recurse into each child */
      for (unsigned int i=0; i<numChildren; i++) 
        {  
          node->set(i,children[i].geomBounds);
          children[i].parent = &node->child(i);
          recurse_sweep(children[i],nodeAlloc,leafAlloc,threadID,numThreads);
        }     
    }
    
       struct SplitSweep
      {
	/*! construct an invalid split by default */
	__forceinline SplitSweep()
	  : sah(inf), dim(-1), pos(0) {}
	
	/*! constructs specified split */
	__forceinline SplitSweep(float sah, int dim, int pos)
	  : sah(sah), dim(dim), pos(pos) {}
	
	/*! tests if this split is valid */
	__forceinline bool valid() const { return dim != -1; }

	/*! calculates surface area heuristic for performing the split */
	__forceinline float splitSAH() const { return sah; }
       	

	/*! stream output */
	friend std::ostream& operator<<(std::ostream& cout, const SplitSweep& split) {
	  return cout << "Split { sah = " << split.sah << ", dim = " << split.dim << ", pos = " << split.pos << "}";
	}
	
      public:
	float sah;       //!< SAH cost of the split
	int dim;         //!< split dimension
	int pos;         //!< bin index for splitting
      };

    void getBestSweepSplit(SplitSweep &split,
			   const size_t begin,
			   const size_t end,
			   const Vec3fa *const centroids,
			   float *const tmp,
			   const size_t N,
			   const size_t dim)
    {
      const size_t size = end-begin;
      assert(size >= 2);
      BBox3fa bounds;     
      bounds = empty;
      for (ssize_t i=end-1;i>=begin;i--) { bounds.extend( centroids[i] ); tmp[i] = area( bounds ); }
      bounds = centroids[begin];
      for (size_t i=begin+1;i<end-1;i++) { 
	bounds.extend( centroids[i-1] );
	const float lArea  = area( bounds );
	const float rArea = tmp[i];
	const size_t lItems  = i-begin;
	const size_t rItems = size - lItems;
	assert(lItems + rItems == size);
	const float sah = (lItems+(N-1))/N * lArea + (rItems+(N-1))/N * rArea;
	if (unlikely(sah < split.splitSAH()))
	  split = SplitSweep(sah,dim,i);
      } 
    }

    template<typename T, typename Compare>
    size_t partition(T *const prims, const size_t begin, const size_t end, const Compare &cmp)
    {
      assert(begin <= end);
      T* l = prims + begin;
      T* r = prims + end - 1;
          
      while(1)
	{
	  while (likely(l <= r && cmp(*l))) 
            {
              ++l;
            }
	  while (likely(l <= r && !cmp(*r)))
            {
              --r;
            }
	  if (r<l) break;

	  std::swap(*l,*r);
	  l++; r--;
	}
          
      return l - prims;
    }

    template<typename Primitive>
    void BVH4TriangleBuilderFastSweep<Primitive>::splitSequential_sweep(BuildRecord& current,
							     BuildRecord& leftChild,
							     BuildRecord& rightChild,
							     const size_t threadID,
							     const size_t numThreads)
    {
    PING;
       /* calculate binning function */
      PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);

      std::sort(centroids[0]+current.begin,centroids[0]+current.end,compare_x);
      std::sort(centroids[1]+current.begin,centroids[1]+current.end,compare_y);
      std::sort(centroids[2]+current.begin,centroids[2]+current.end,compare_z);      

      SplitSweep bestSplit;
      getBestSweepSplit(bestSplit,current.begin,current.end,centroids[0],tmp,4,0);
      getBestSweepSplit(bestSplit,current.begin,current.end,centroids[1],tmp,4,1);
      getBestSweepSplit(bestSplit,current.begin,current.end,centroids[2],tmp,4,2);

      if (unlikely(!bestSplit.valid())) 
	splitFallback(prims,current,leftChild,rightChild);
      else
	{
	  const size_t center = current.begin + bestSplit.pos;
	  const size_t dim   = bestSplit.dim;
	  const size_t dim_1 = (bestSplit.dim + 1)%3;
	  const size_t dim_2 = (bestSplit.dim + 2)%3;
	  memcpy(&centroids[dim_1][current.begin],&centroids[dim][current.begin],sizeof(Vec3fa)*current.size());
	  memcpy(&centroids[dim_2][current.begin],&centroids[dim][current.begin],sizeof(Vec3fa)*current.size());

	  CentGeomBBox3fa left; 
	  left.reset();
	  for (size_t i=current.begin; i<center; i++)
	    left.extend(centroids[dim][i]);
	  leftChild.init(left,current.begin,center);

	  CentGeomBBox3fa right; 
	  right.reset();
	  for (size_t i=center; i<current.end; i++)
	    right.extend(centroids[dim][i]);
	  rightChild.init(right,center,current.end);
	  
	  //const size_t mid_1 = partition(centroids[dim_1],current.begin,current.end,[&]( const Vec3fa &v ) { return v[split.dim]
	}           
    }

    template<typename Primitive>    
    void BVH4TriangleBuilderFastSweep<Primitive>::createSmallLeaf(BuildRecord& current, Allocator& leafAlloc, size_t threadID)
    {
      PING;
      size_t items = Primitive::blocks(current.size());
      size_t start = current.begin;
            
      /* allocate leaf node */
      Primitive* accel = (Primitive*) leafAlloc.malloc(items*sizeof(Primitive));
      *current.parent = BVH4TriangleBuilderFast<Primitive>::bvh->encodeLeaf((char*)accel, BVH4TriangleBuilderFast<Primitive>::listMode ? BVH4TriangleBuilderFast<Primitive>::listMode : items);
      
      assert(current.size() <= BVH4::N);
      PrimRef local[BVH4::N];
      for (size_t i=0; i<items; i++) 
	local[i] = prims[current.begin+i];
      
      for (size_t i=0; i<items; i++) 
	accel[i].fill(local,start,items,BVH4TriangleBuilderFast<Primitive>::scene,BVH4TriangleBuilderFast<Primitive>::listMode);
    }
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================


    // will be removed soon, just for testing 
    static void test_partition()
    {
#if 1
#define MAX_SIZE 1024*1024*128

        PING;
        PrimRef *test_array  = (PrimRef*)_mm_malloc(MAX_SIZE*sizeof(PrimRef),64);
        double t_parallel,t_serial;

        double t_parallel_total,t_serial_total;

	for (size_t s=1;s<MAX_SIZE;s+=s)        
          {
            DBG_PRINT(s);
            srand48(s*32323);


            t_parallel_total = 0.0;
#define ITERATIONS 20

            size_t mids[ITERATIONS];
            PrimInfo total;
            total.reset();

            for (size_t j=0;j<ITERATIONS;j++)
              {
                for (size_t i=0;i<s;i++)
                  {
                    float x = drand48();
                    float y = drand48();
                    float z = drand48();
                    BBox3fa b;
                    const float f = 0.1f;
                    b.lower = Vec3fa(x-f,y-f,z-f);
                    b.upper = Vec3fa(x+f,y+f,z+f);
                    test_array[i] = PrimRef(b,i);
                    total.add(b,b,1);
                  }

                t_parallel = getSeconds();        
                    

                PrimInfo leftInfo;
                PrimInfo rightInfo;
                PrimInfo init;
                leftInfo.reset();
                rightInfo.reset();
                init.reset();

                //parallel_partition pp(test_array,s, [] (unsigned int &t) { DBG_PRINT(t);} );
                //size_t mid = pp.parition(pivot);

		// size_t numLeft = 0;
		// size_t numRight = 0;
		// size_t init = 0;

#if 1

                size_t mid = parallel_in_place_partitioning_static<PrimRef,PrimInfo>(test_array,              
										   s,
										   init,
                                                                                   leftInfo,
                                                                                   rightInfo,
                                                                                   [] (const PrimRef &ref) { return ref.lower.x < 0.5f; },
                                                                                   [] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.extend(ref.bounds()); },
                                                                                   [] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); }
                                                                                   );
#else
                    
                size_t mid = parallel_in_place_partitioning<128,PrimRef,PrimInfo>(test_array,
										s,
										init,
										leftInfo,
										rightInfo,
										[] (const PrimRef &ref) { return ref.lower.x < 0.5f; },
										[] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.extend(ref.bounds()); },
										[] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); }
										);
#endif

                t_parallel = getSeconds() - t_parallel;        
                t_parallel_total += t_parallel;

		/*
		DBG_PRINT(numLeft);
		DBG_PRINT(numRight);
		DBG_PRINT(numLeft+numRight);
		DBG_PRINT(s);
		DBG_PRINT(mid);
		*/

                mids[j] = mid;
                for (size_t i=0;i<mid;i++)
                  if (test_array[i].lower.x >= 0.5f)
                    {
                      DBG_PRINT("left error");
                      DBG_PRINT(i);
                      DBG_PRINT(test_array[i]);
                    }

                for (size_t i=mid;i<s;i++)
                  if (test_array[i].lower.x < 0.5f)
                    {
                      DBG_PRINT("right error");
                      DBG_PRINT(i);
                      DBG_PRINT(test_array[i]);
                    }
		//assert(numLeft+numRight == s);
              }
            t_parallel_total /= ITERATIONS;

            srand48(s*32323);
#if 1
            t_serial_total = 0.0;
            for (size_t j=0;j<ITERATIONS;j++)
              {
                for (size_t i=0;i<s;i++)
                  {
                    float x = drand48();
                    float y = drand48();
                    float z = drand48();
                    BBox3fa b;
                    const float f = 0.1f;
                    b.lower = Vec3fa(x-f,y-f,z-f);
                    b.upper = Vec3fa(x+f,y+f,z+f);
                    test_array[i] = PrimRef(b,i);
                  }


                t_serial = getSeconds();        

                PrimInfo leftInfo;
                PrimInfo rightInfo;
                PrimInfo init;
                leftInfo.reset();
                rightInfo.reset();
                init.reset();

                size_t mid_serial = serial_in_place_partitioning<PrimRef,PrimInfo>(test_array,
                                                                                   s,
                                                                                   init,
                                                                                   leftInfo,
                                                                                   rightInfo,
                                                                                   [] (const PrimRef &ref) { return ref.lower.x < 0.5f; },
                                                                                   [] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.extend(ref.bounds()); },
                                                                                   [] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); }
                                                                                   );

                t_serial = getSeconds() - t_serial;        
                t_serial_total += t_serial;
                assert(mid_serial == mids[j]);

                for (size_t i=0;i<mid_serial;i++)
                  if (test_array[i].lower.x >= 0.5f)
                    {
                      DBG_PRINT("left error");
                      DBG_PRINT(i);
                      DBG_PRINT(test_array[i]);
                    }

                for (size_t i=mid_serial;i<s;i++)
                  if (test_array[i].lower.x < 0.5f)
                    {
                      DBG_PRINT("right error");
                      DBG_PRINT(i);
                      DBG_PRINT(test_array[i]);
                    }

              }
            t_serial_total /= ITERATIONS;
#endif
            std::cout << " t_parallel_total = " << 1000.0f*t_parallel_total << "ms, perf = " << 1E-6*double(s)/t_parallel_total << " Mprim/s" << std::endl;
            std::cout << " t_serial_total = " << 1000.0f*t_serial_total << "ms, perf = " << 1E-6*double(s)/t_serial_total << " Mprim/s" << std::endl;
            DBG_PRINT(t_serial_total/t_parallel_total);
            
             
          }
        exit(0);

#endif
  }




    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    Builder* BVH4Bezier1vBuilderFast    (void* bvh, Scene* scene, size_t mode) { return new class BVH4BezierBuilderFast<Bezier1v>  ((BVH4*)bvh,scene,mode); }
    Builder* BVH4Bezier1iBuilderFast   (void* bvh, Scene* scene, size_t mode) { return new class BVH4BezierBuilderFast<Bezier1i>((BVH4*)bvh,scene,mode); }
    Builder* BVH4Triangle1BuilderFast  (void* bvh, Scene* scene, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle1> ((BVH4*)bvh,scene,mode); }
    Builder* BVH4Triangle4BuilderFast  (void* bvh, Scene* scene, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle4> ((BVH4*)bvh,scene,mode); }

    Builder* BVH4Triangle4BuilderFastSweep  (void* bvh, Scene* scene, size_t mode) { return new class BVH4TriangleBuilderFastSweep<Triangle4> ((BVH4*)bvh,scene,mode); }


#if defined(__AVX__)
    Builder* BVH4Triangle8BuilderFast  (void* bvh, Scene* scene, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle8> ((BVH4*)bvh,scene,mode); }
#endif
    Builder* BVH4Triangle1vBuilderFast (void* bvh, Scene* scene, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle1v>((BVH4*)bvh,scene,mode); }
    Builder* BVH4Triangle4vBuilderFast (void* bvh, Scene* scene, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle4v>((BVH4*)bvh,scene,mode); }
    Builder* BVH4Triangle4iBuilderFast (void* bvh, Scene* scene, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle4i>((BVH4*)bvh,scene,mode); }
    Builder* BVH4UserGeometryBuilderFast(void* bvh, Scene* scene, size_t mode) { return new class BVH4UserGeometryBuilderFastT<AccelSetItem>((BVH4*)bvh,scene,mode); }

    Builder* BVH4Bezier1vMeshBuilderFast    (void* bvh, BezierCurves* geom, size_t mode) { return new class BVH4BezierBuilderFast<Bezier1v>  ((BVH4*)bvh,geom,mode); }
    Builder* BVH4Bezier1iMeshBuilderFast   (void* bvh, BezierCurves* geom, size_t mode) { return new class BVH4BezierBuilderFast<Bezier1i> ((BVH4*)bvh,geom,mode); }
    Builder* BVH4Triangle1MeshBuilderFast  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle1> ((BVH4*)bvh,mesh,mode); }
    Builder* BVH4Triangle4MeshBuilderFast  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle4> ((BVH4*)bvh,mesh,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderFast  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle8> ((BVH4*)bvh,mesh,mode); }
#endif
    Builder* BVH4Triangle1vMeshBuilderFast (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle1v>((BVH4*)bvh,mesh,mode); }
    Builder* BVH4Triangle4vMeshBuilderFast (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle4v>((BVH4*)bvh,mesh,mode); }
    Builder* BVH4Triangle4iMeshBuilderFast (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4TriangleBuilderFast<Triangle4i>((BVH4*)bvh,mesh,mode); }
    Builder* BVH4UserGeometryMeshBuilderFast   (void* bvh, UserGeometryBase* geom, size_t mode) { return new class BVH4UserGeometryBuilderFastT<AccelSetItem>((BVH4*)bvh,geom,mode); }

    Builder* BVH4SubdivPatch1BuilderFast(void* bvh, Scene* scene, size_t mode) { return new class BVH4SubdivBuilderFast<SubdivPatch1>((BVH4*)bvh,scene,mode); }
    Builder* BVH4SubdivGridBuilderFast(void* bvh, Scene* scene, size_t mode) { return new class BVH4SubdivGridBuilderFast((BVH4*)bvh,scene,mode); }
    Builder* BVH4SubdivGridEagerBuilderFast(void* bvh, Scene* scene, size_t mode) { return new class BVH4SubdivGridEagerBuilderFast((BVH4*)bvh,scene,mode); }
    Builder* BVH4SubdivGridLazyBuilderFast(void* bvh, Scene* scene, size_t mode) { return new class BVH4SubdivGridLazyBuilderFast((BVH4*)bvh,scene,mode); }
    Builder* BVH4SubdivPatch1CachedBuilderFast(void* bvh, Scene* scene, size_t mode) { return new class BVH4SubdivPatch1CachedBuilderFast((BVH4*)bvh,scene,mode); }

  }
}
