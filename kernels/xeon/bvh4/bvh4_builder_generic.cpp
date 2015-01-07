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

#include "bvh4_builder_generic.h"
#include "bvh4.h"

#include "geometry/triangle4.h"

#include "algorithms/parallel_for_for.h"
#include "algorithms/parallel_for_for_prefix_sum.h"

namespace embree
{
  namespace isa
  {
    struct CreateBVH4Node
    {
      __forceinline CreateBVH4Node (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline BVH4::NodeRef operator() (BVHBuilderGeneric<BVH4::NodeRef>::BuildRecord* children, const size_t N)
      {
        FastAllocator::Thread& alloc = *bvh->alloc2.instance();
        BVH4::Node* node = (BVH4::Node*) alloc.malloc(sizeof(BVH4::Node)); node->clear();
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
      
      __forceinline BVH4::NodeRef operator() (BVHBuilderGeneric<BVH4::NodeRef>::BuildRecord& current, PrimRef* prims)
      {
        PRINT("leaf");
        for (size_t i=current.begin; i<current.end; i++)
          PRINT3(i,prims[i].geomID(),prims[i].primID());

        FastAllocator::Thread& alloc = *bvh->alloc2.instance();
        size_t items = Primitive::blocks(current.size());
        size_t start = current.begin;
        Primitive* accel = (Primitive*) alloc.malloc(items*sizeof(Primitive));
        //Primitive* accel = (Primitive*) malloc(items*sizeof(Primitive));
        BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.end,bvh->scene,false);
          PRINT2(i,accel[i].v0);
          PRINT2(i,accel[i].e1);
          PRINT2(i,accel[i].e2);
          PRINT2(i,accel[i].Ng);
          PRINT2(i,accel[i].geomIDs);
          PRINT2(i,accel[i].primIDs);
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
        /* calculate scene size */
        const size_t numPrimitives = scene->numTriangles;
        prims.resize(numPrimitives);
        
        /* skip build for empty scene */
        if (numPrimitives == 0) {
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
      
        /* build BVH */
        PrimInfo pinfo = CreatePrimRefArray<TriangleMesh,1>(scene,prims);
        BVHBuilderGeneric<BVH4::NodeRef> builder(prims.data(),pinfo.size(),BVH4::N,2,1,4*BVH4::maxLeafBlocks,BVH4::maxBuildDepthLeaf);
        CreateBVH4Node createNode(bvh);
        CreateLeaf<Triangle4> createLeaf(bvh);
        BVH4::NodeRef root = builder(createNode,createLeaf);
        bvh->set(root,pinfo.geomBounds,pinfo.size());
      }
    };

    Builder* BVH4Triangle4BuilderFastNew  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle4BuilderFastClass((BVH4*)bvh,scene); }
  }
}
