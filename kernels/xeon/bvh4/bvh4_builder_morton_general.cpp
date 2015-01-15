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

#if defined(USE_TBB)

#define PROFILE_MORTON_GENERAL

#include "bvh4.h"
#include "bvh4_builder_morton_general.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"


#define DBG(x) 


namespace embree 
{
  namespace isa
  {
    static double dt = 0.0f;

    struct AllocBVH4Node
    {
      __forceinline BVH4::Node* operator() (BuildRecord& current, BuildRecord* children, size_t numChildren, FastAllocator::ThreadLocal2* alloc)
      {
        BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
        *current.parent = BVH4::encodeNode(node);
        for (size_t i=0; i<numChildren; i++)
          children[i].parent = &node->child(i);
        return node;
      }
    };

    struct SetBVH4Bounds
    {
      __forceinline BBox3fa operator() (BVH4::Node* node, const BBox3fa* bounds, size_t N)
      {
        BBox3fa res = empty;
        for (size_t i=0; i<N; i++) {
          const BBox3fa b = bounds[i];
          res.extend(b);
          node->set(i,b);
        }
        return res;
      }
    };
    
    struct CreateTriangle4Leaf
    {
      __forceinline CreateTriangle4Leaf (Scene* scene, MortonID32Bit* morton, size_t encodeShift, size_t encodeMask)
        : scene(scene), morton(morton), encodeShift(encodeShift), encodeMask(encodeMask) {}

      void operator() (BuildRecord& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        ssef lower(pos_inf);
        ssef upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=4);
        
        /* allocate leaf node */
        Triangle4* accel = (Triangle4*) alloc->alloc1.malloc(sizeof(Triangle4));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        ssei vgeomID = -1, vprimID = -1, vmask = -1;
        sse3f v0 = zero, v1 = zero, v2 = zero;
        
        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = index >> encodeShift; 
          const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(ssef)p0,(ssef)p1,(ssef)p2);
          upper = max(upper,(ssef)p0,(ssef)p1,(ssef)p2);
          vgeomID [i] = geomID;
          vprimID [i] = primID;
          vmask   [i] = mesh->mask;
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
        }
        Triangle4::store_nt(accel,Triangle4(v0,v1,v2,vgeomID,vprimID,vmask,false));
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
      }
      
      Scene* scene;
      MortonID32Bit* morton;
      size_t encodeShift;
      size_t encodeMask;
    };
      
    struct CalculateBounds
    {
      __forceinline CalculateBounds (Scene* scene, size_t encodeShift, size_t encodeMask)
        : scene(scene), encodeShift(encodeShift), encodeMask(encodeMask) {}

      __forceinline const BBox3fa operator() (const MortonID32Bit& morton)
      {
        const size_t index = morton.index;
        const size_t primID = index & encodeMask; 
        const size_t geomID = index >> encodeShift; 
        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
        return mesh->bounds(primID);
      }

    private:
      Scene* scene;
      size_t encodeShift;
      size_t encodeMask;
    };

    class BVH4BuilderMortonGeneral2 : public Builder
    {
    public:

      BVH4BuilderMortonGeneral2 (BVH4* bvh, Scene* scene, TriangleMesh* mesh, size_t listMode, size_t logBlockSize, bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), listMode(listMode), logBlockSize(logBlockSize), needVertices(needVertices), primBytes(primBytes), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
	topLevelItemThreshold(0), encodeShift(0), encodeMask(-1), morton(NULL), bytesMorton(0), numPrimitives(0), numAllocatedPrimitives(0), numAllocatedNodes(0)
      {
        needAllThreads = true;
      }
      
      /*! Destruction */
      ~BVH4BuilderMortonGeneral2 ()
      {
        if (morton) os_free(morton,bytesMorton);
        bvh->alloc.shrink();
      }

       /* build function */
      void build(size_t threadIndex, size_t threadCount) 
      {
        if (g_verbose >= 2)
        std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) << "::BVH4BuilderMortonGeneral ... " << std::flush;
      
      /* calculate size of scene */
      size_t numPrimitivesOld = numPrimitives;
      numPrimitives = scene->numTriangles;
      
      /* calculate groupID, primID encoding */
      Scene::Iterator<TriangleMesh,1> iter2(scene);
      size_t numGroups = iter2.size();
      size_t maxPrimsPerGroup = iter2.maxPrimitivesPerGeometry();
      encodeShift = __bsr(maxPrimsPerGroup) + 1;
      encodeMask = ((size_t)1 << encodeShift)-1;
      size_t maxGroups = ((size_t)1 << (31-encodeShift))-1;
      if (maxPrimsPerGroup > encodeMask || numGroups > maxGroups) 
        THROW_RUNTIME_ERROR("encoding error in morton builder");
      
      /* preallocate arrays */
      if (numPrimitivesOld != numPrimitives)
      {
	bvh->init(sizeof(BVH4::Node),numPrimitives,threadCount);
        if (morton) os_free(morton,bytesMorton);
	bytesMorton = ((numPrimitives+4)&(-4)) * sizeof(MortonID32Bit);
        morton = (MortonID32Bit* ) os_malloc(bytesMorton); memset(morton,0,bytesMorton);
      }
         
#if defined(PROFILE_MORTON_GENERAL)
      
      double dt_min = pos_inf;
      double dt_avg = 0.0f;
      double dt_max = neg_inf;
      for (size_t k=0; k<20; k++) 
      {
        double t0 = getSeconds();
#endif

        //bvh->alloc.init(numPrimitives*sizeof(BVH4::Node),numPrimitives*sizeof(BVH4::Node));
      size_t bytesAllocated = (numPrimitives+7)/8*sizeof(BVH4::Node) + size_t(1.2f*(numPrimitives+3)/4)*sizeof(Triangle4);
      bvh->alloc2.init(bytesAllocated,2*bytesAllocated); // FIXME: not working if scene size changes, initial block has to get reallocated as used as temporary data
      
      /* compute scene bounds */
      Scene::Iterator<TriangleMesh,1> iter1(scene);
      const CentGeomBBox3fa bounds = parallel_for_for_reduce( iter1, CentGeomBBox3fa(empty), [&](TriangleMesh* mesh, const range<size_t>& r, size_t k) -> CentGeomBBox3fa
      {
        CentGeomBBox3fa bounds(empty);
        for (size_t i=r.begin(); i<r.end(); i++) bounds.extend(mesh->bounds(i));
        return bounds;
      }, [] (CentGeomBBox3fa a, const CentGeomBBox3fa& b) { a.merge(b); return a; });

      /* compute morton codes */
      Scene::Iterator<TriangleMesh,1> iter(scene);
      MortonID32Bit* dest = (MortonID32Bit*) bvh->alloc2.ptr();
      MortonCodeGenerator::MortonCodeMapping mapping(bounds.centBounds);
      parallel_for_for( iter, [&](TriangleMesh* mesh, const range<size_t>& r, size_t k) 
      {
        MortonCodeGenerator generator(mapping,&dest[k]);
        for (size_t i=r.begin(); i<r.end(); i++) {
          generator(mesh->bounds(i),i | (mesh->id << encodeShift));
        }
      });

      /* create BVH */
      AllocBVH4Node allocNode;
      SetBVH4Bounds setBounds;
      CreateTriangle4Leaf createLeaf(scene,morton,encodeShift,encodeMask);
      CalculateBounds calculateBounds(scene,encodeShift,encodeMask);
      BVH4BuilderMortonGeneral<FastAllocator::ThreadLocal2*,AllocBVH4Node,SetBVH4Bounds,CreateTriangle4Leaf,CalculateBounds> builder
        (allocNode,setBounds,createLeaf,calculateBounds,(BVH4*)bvh,scene,4,BVH4::maxBuildDepth,4,inf);
      BVH4::NodeRef root = builder.build(dest,morton,numPrimitives,encodeShift,encodeMask);
      bvh->set(root,bounds.geomBounds,numPrimitives);

#if defined(PROFILE_MORTON_GENERAL)
        double dt = getSeconds()-t0;
        dt_min = min(dt_min,dt);
        if (k != 0) dt_avg = dt_avg + dt;
        dt_max = max(dt_max,dt);
      }
      dt_avg /= double(19);
      
      std::cout << "[DONE]" << std::endl;
      std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
      std::cout << BVH4Statistics(bvh).str();
#endif

      if (g_verbose >= 2) {
        //std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
        std::cout << "  bvh4::alloc : "; bvh->alloc.print_statistics();
	std::cout << "  bvh4::alloc2: "; bvh->alloc2.print_statistics();
        std::cout << BVH4Statistics(bvh).str();
      }

      /* benchmark mode */
      if (g_benchmark) {
	BVH4Statistics stat(bvh);
	//std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
      }
      }

    public:
      BVH4* bvh;               //!< Output BVH
      
      Scene* scene;
      size_t logBlockSize;
      size_t primBytes; 
      size_t minLeafSize;
      size_t maxLeafSize;
      size_t listMode;

      size_t topLevelItemThreshold;
      size_t encodeShift;
      size_t encodeMask;

      bool needVertices;
            
    public:
      MortonID32Bit* __restrict__ morton;
      size_t bytesMorton;
      
    public:
      size_t numPrimitives;
      size_t numAllocatedPrimitives;
      size_t numAllocatedNodes;
    };
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    

      Builder* BVH4Triangle4BuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { 
        return new class BVH4BuilderMortonGeneral2 ((BVH4*)bvh,scene,NULL,mode,2,false,sizeof(Triangle4),4,inf); 
      }
  }
}

#else

#include "common/builder.h"

namespace embree 
{
  namespace isa
  {
    Builder* BVH4Triangle4BuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { 
      return NULL;
    }
  }
}
#endif
  
  
