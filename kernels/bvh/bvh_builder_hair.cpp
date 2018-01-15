// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "../builders/bvh_builder_hair.h"
#include "../builders/bvh_builder_msmblur_hair.h"
#include "../builders/primrefgen.h"

#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"

#if defined(EMBREE_GEOMETRY_HAIR)

namespace embree
{
  namespace isa
  {
    template<int N, typename Primitive>
    struct BVHNHairBuilderSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      mvector<PrimRef> prims;
      BVHBuilderHair::Settings settings;

      BVHNHairBuilderSAH (BVH* bvh, Scene* scene)
        : bvh(bvh), scene(scene), prims(scene->device,0) {}
      
      void build() 
      {
        /* if we use the primrefarray for allocations we have to take it back from the BVH */
        if (settings.finished_range_threshold != size_t(inf))
          bvh->alloc.unshare(prims);
      
        /* fast path for empty BVH */
        const size_t numPrimitives = scene->getNumPrimitives<NativeCurves,false>();
        if (numPrimitives == 0) {
          prims.clear();
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "HairBuilderSAH");

        /* create primref array */
        prims.resize(numPrimitives);
        const PrimInfo pinfo = createPrimRefArray<NativeCurves,false>(scene,prims,scene->progressInterface);

        /* estimate acceleration structure size */
        const size_t node_bytes = pinfo.size()*sizeof(typename BVH::UnalignedNode)/(4*N);
        const size_t leaf_bytes = pinfo.size()*sizeof(Primitive);
        bvh->alloc.init_estimate(node_bytes+leaf_bytes);
        
        /* builder settings */
        settings.branchingFactor = N;
        settings.maxDepth = BVH::maxBuildDepthLeaf;
        settings.logBlockSize = 0;
        settings.minLeafSize = 1;
        settings.maxLeafSize = BVH::maxLeafBlocks;
        settings.finished_range_threshold = numPrimitives/1000;
        if (settings.finished_range_threshold < 1000)
          settings.finished_range_threshold = inf;

        /* creates a leaf node */
        auto createLeaf = [&] (const PrimRef* prims, const range<size_t>& set, const FastAllocator::CachedAllocator& alloc) -> NodeRef
          {
            size_t start = set.begin();
            size_t items = set.size();
            Primitive* accel = (Primitive*) alloc.malloc1(items*sizeof(Primitive),BVH::byteAlignment);
            for (size_t i=0; i<items; i++) {
              accel[i].fill(prims,start,set.end(),bvh->scene);
            }
            return bvh->encodeLeaf((char*)accel,items);
          };

        auto reportFinishedRange = [&] (const range<size_t>& range) -> void
          {
            PrimRef* begin = prims.data()+range.begin();
            PrimRef* end   = prims.data()+range.end(); // FIXME: extended end for spatial split builder!!!!!
            size_t bytes = (size_t)end - (size_t)begin;
            bvh->alloc.addBlock(begin,bytes);
          };
          
        /* build hierarchy */
        typename BVH::NodeRef root = BVHBuilderHair::build<NodeRef>
          (typename BVH::CreateAlloc(bvh),
           typename BVH::AlignedNode::Create(),
           typename BVH::AlignedNode::Set(),
           typename BVH::UnalignedNode::Create(),
           typename BVH::UnalignedNode::Set(),
           createLeaf,scene->progressInterface,
           reportFinishedRange,
           scene,prims.data(),pinfo,settings);
        
        bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());

        /* if we allocated using the primrefarray we have to keep it alive */
        if (settings.finished_range_threshold != size_t(inf))
          bvh->alloc.share(prims);
        
        /* clear temporary data for static geometry */
        if (scene->isStatic()) {
          bvh->shrink();
        }
        bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };
    
    /*! entry functions for the builder */
    Builder* BVH4Bezier1vBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<4,Bezier1v>((BVH4*)bvh,scene); }
    Builder* BVH4Bezier1iBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<4,Bezier1i>((BVH4*)bvh,scene); }

#if defined(__AVX__)
    Builder* BVH8Bezier1vBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<8,Bezier1v>((BVH8*)bvh,scene); }
    Builder* BVH8Bezier1iBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<8,Bezier1i>((BVH8*)bvh,scene); }
#endif

  }
}
#endif
