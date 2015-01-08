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
#include "bvh4_statistics.h"
#include "bvh4_builder_generic.h"

#include "algorithms/parallel_for_for.h"
#include "algorithms/parallel_for_for_prefix_sum.h"

#include "geometry/triangle4.h"

#define PROFILE

namespace embree
{
  namespace isa
  {
    typedef LinearAllocatorPerThread::ThreadAllocator Allocator;
    //typedef FastAllocator::Thread Allocator;

    //__aligned(64) LinearAllocatorPerThread::ThreadAllocator* g_alloc;

    struct CreateBVH4Node
    {
      __forceinline CreateBVH4Node (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline BVH4::NodeRef operator() (BuildRecord<BVH4::NodeRef>* children, const size_t N, Allocator& alloc)
      {
        //FastAllocator::Thread& alloc = *bvh->alloc2.instance();
        BVH4::Node* node = (BVH4::Node*) alloc.malloc(sizeof(BVH4::Node)); node->clear();
        //BVH4::Node* node = (BVH4::Node*) g_alloc->malloc(sizeof(BVH4::Node)); node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i].geomBounds);
          children[i].parent = &node->child(i);
        }
        return bvh->encodeNode(node);
      }

      BVH4* bvh;
    };

    template<typename Primitive>
    struct CreateLeaf
    {
      __forceinline CreateLeaf (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline BVH4::NodeRef operator() (const BuildRecord<BVH4::NodeRef>& current, PrimRef* prims, Allocator& alloc)
      {
        size_t items = Primitive::blocks(current.size());
        size_t start = current.begin;
        //FastAllocator::Thread& alloc = *bvh->alloc2.instance();
        Primitive* accel = (Primitive*) alloc.malloc(items*sizeof(Primitive));
        //Primitive* accel = (Primitive*) g_alloc->malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.end,bvh->scene,false);
        }
        return node;
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
    
    struct BVH4Triangle4BuilderFastClass : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      vector_t<PrimRef> prims; // FIXME: use os_malloc in vector_t for large allocations

      BVH4Triangle4BuilderFastClass (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}

      void build(size_t threadIndex, size_t threadCount) 
      {
         /* start measurement */
        double t0 = 0.0f;
        if (g_verbose >= 1) t0 = getSeconds();

        /* calculate scene size */
        const size_t numPrimitives = scene->numTriangles;
        
        /* skip build for empty scene */
        if (numPrimitives == 0) {
          prims.resize(numPrimitives);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
      
        /* verbose mode */
        if (g_verbose >= 1)
          std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4BuilderFastNew ... " << std::flush;

        //bvh->alloc2.init(numPrimitives*sizeof(PrimRef),numPrimitives*sizeof(BVH4::Node)); 

#if defined(PROFILE)
      
      double dt_min = pos_inf;
      double dt_avg = 0.0f;
      double dt_max = neg_inf;
      for (size_t i=0; i<20; i++) 
      {
        double t0 = getSeconds();
#endif

        if (i == 0) bvh->init(sizeof(BVH4::Node),numPrimitives,1);
        bvh->alloc.clear();
        //g_alloc = new LinearAllocatorPerThread::ThreadAllocator(&bvh->alloc);
        //bvh->alloc2.reset();
        
        /* build BVH */
        prims.resize(numPrimitives);
        //memset(prims.data(),0,prims.size()*sizeof(PrimRef));

        //vector_t<PrimRef> tmp; tmp.resize(numPrimitives); 
        //PrimRef* tmp_prims = (PrimRef*) tmp.data();
        //PrimRef* tmp_prims = (PrimRef*) bvh->alloc.curPtr();
        PrimRef* tmp_prims = NULL;

        //double T0 = getSeconds();
        PrimInfo pinfo = CreatePrimRefArray<TriangleMesh,1>(scene,prims);
        //double T1 = getSeconds();
        CreateBVH4Node createNode(bvh);
        CreateLeaf<Triangle4> createLeaf(bvh);
        BVHBuilderGeneric<BVH4::NodeRef,CreateBVH4Node,CreateLeaf<Triangle4> > builder(bvh,createNode,createLeaf,prims.data(),tmp_prims,pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,2,4,4*BVH4::maxLeafBlocks);
        BVH4::NodeRef root = builder();
        //double T2 = getSeconds();
        bvh->set(root,pinfo.geomBounds,pinfo.size());

        //PRINT(1000.0f*(T1-T0));
        //PRINT(1000.0f*(T2-T1));

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

    Builder* BVH4Triangle4BuilderFastNew  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle4BuilderFastClass((BVH4*)bvh,scene); }
  }
}
