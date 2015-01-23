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
#include "bvh4_statistics.h"
#include "builders/bvh_builder_binned_sah2.h"

#include "algorithms/parallel_for_for.h"
#include "algorithms/parallel_for_for_prefix_sum.h"

#include "geometry/triangle4.h"

#define PROFILE

namespace embree
{
  namespace isa
  {
    typedef FastAllocator::ThreadLocal2 Allocator;

    struct CreateAlloc
    {
      __forceinline CreateAlloc (BVH4* bvh) : bvh(bvh) {}
      __forceinline Allocator* operator() () const { return bvh->alloc2.threadLocal2();  }

      BVH4* bvh;
    };

    struct CreateBVH4Node
    {
      __forceinline CreateBVH4Node (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline void operator() (const isa::BuildRecord2<BVH4::NodeRef>& current, BuildRecord2<BVH4::NodeRef>* children, const size_t N, Allocator* alloc) 
      {
        BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i].pinfo.geomBounds);
          children[i].parent = &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
      }

      BVH4* bvh;
    };

    template<typename Primitive>
    struct CreateLeaf
    {
      __forceinline CreateLeaf (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline void operator() (const BuildRecord2<BVH4::NodeRef>& current, PrimRef* prims, Allocator* alloc) // FIXME: why are prims passed here but not for createNode
      {
        size_t items = Primitive::blocks(current.pinfo.size());
        size_t start = current.pinfo.begin;
        Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.pinfo.end,bvh->scene,false);
        }
        *current.parent = node;
      }

      BVH4* bvh;
    };
    
    template<typename Ty, size_t timeSteps>
    PrimInfo CreatePrimRefArray(Scene* scene, vector_t<PrimRef>& prims)
    {
      ParallelForForPrefixSumState<PrimInfo> pstate;
      Scene::Iterator<Ty,timeSteps> iter(scene);

      /* first try */
      pstate.init(iter,size_t(1024));
      PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Ty* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
      {
        PrimInfo pinfo(empty);
        for (ssize_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!mesh->valid(j,&bounds)) continue;
          const PrimRef prim(bounds,mesh->id,j);
          pinfo.add(prim.bounds(),prim.center2());
          prims[k++] = prim;
        }
        return pinfo;
      }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });

      /* if we need to filter out geometry, run again */
      if (pinfo.size() != prims.size())
      {
        pinfo = parallel_for_for_prefix_sum( pstate, iter, PrimInfo(empty), [&](Ty* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
        {
          k = base.begin;
          PrimInfo pinfo(empty);
          for (ssize_t j=r.begin(); j<r.end(); j++)
          {
            BBox3fa bounds = empty;
            if (!mesh->valid(j,&bounds)) continue;
            const PrimRef prim(bounds,mesh->id,j);
            pinfo.add(prim.bounds(),prim.center2());
            prims[k++] = prim;
          }
          return pinfo;
        }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
      }
      return pinfo;
    }
    
    struct BVH4Triangle4BuilderBinnedSAH2Class : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations

      BVH4Triangle4BuilderBinnedSAH2Class (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      void build(size_t threadIndex, size_t threadCount) 
      {
        /* start measurement */
        double t0 = 0.0f;
        if (g_verbose >= 1) t0 = getSeconds();

        /* calculate scene size */
        //const size_t numPrimitives = scene->numTriangles;
        const size_t numPrimitives = scene->getNumPrimitives<TriangleMesh,1>();
        
        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
      
        /* verbose mode */
        if (g_verbose >= 1)
          std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderFastNew ... " << std::flush;

#if defined(PROFILE)
      
        double dt_min = pos_inf;
        double dt_avg = 0.0f;
        double dt_max = neg_inf;
        for (size_t i=0; i<20; i++) 
        {
          double t0 = getSeconds();
#endif
          
          /* reserve data */
          bvh->alloc2.init(numPrimitives*sizeof(PrimRef),numPrimitives*sizeof(BVH4::Node)); 
          prims.resize(numPrimitives);
          
          /* build BVH */
          PrimInfo pinfo = CreatePrimRefArray<TriangleMesh,1>(scene,prims);
          BVH4::NodeRef root = bvh_builder_binned_sah2_internal<BVH4::NodeRef>(CreateAlloc(bvh),CreateBVH4Node(bvh),CreateLeaf<Triangle4>(bvh),
                                                                              prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,4,4,4*BVH4::maxLeafBlocks,BVH4::travCost,1.0f);
          bvh->set(root,pinfo.geomBounds,pinfo.size());
          
#if defined(PROFILE)
          double dt = getSeconds()-t0;
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
      }
    };
    
    Builder* BVH4Triangle4BuilderBinnedSAH2  (void* bvh, Scene* scene, size_t mode) { return new BVH4Triangle4BuilderBinnedSAH2Class((BVH4*)bvh,scene); }
  }
}
