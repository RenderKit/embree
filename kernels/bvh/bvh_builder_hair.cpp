// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "kernels/builders/bvh_builder_hair.h"
#include "kernels/builders/primrefgen.h"

#include "kernels/geometry/pointi.h"
#include "kernels/geometry/linei.h"
#include "kernels/geometry/curveNi.h"
#include "kernels/geometry/curveNv.h"

#if defined(EMBREE_GEOMETRY_CURVE) || defined(EMBREE_GEOMETRY_POINT)

namespace embree
{
  namespace isa
  {
    template<int N, typename CurvePrimitive, typename LinePrimitive, typename PointPrimitive>
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
        const size_t numPrimitives = scene->getNumPrimitives(Geometry::MTY_CURVES,false);
        if (numPrimitives == 0) {
          bvh->clear();
          prims.clear();
          return;
        }

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "HairBuilderSAH");

        /* create primref array */
        prims.resize(numPrimitives);
        const PrimInfo pinfo = createPrimRefArray(scene,Geometry::MTY_CURVES,false,numPrimitives,prims,scene->progressInterface);

        /* estimate acceleration structure size */
        const size_t node_bytes = pinfo.size()*sizeof(typename BVH::OBBNode)/(4*N);
        const size_t leaf_bytes = CurvePrimitive::bytes(pinfo.size());
        bvh->alloc.init_estimate(node_bytes+leaf_bytes);
        
        /* builder settings */
        settings.branchingFactor = N;
        settings.maxDepth = BVH::maxBuildDepthLeaf;
        settings.logBlockSize = bsf(CurvePrimitive::max_size());
        settings.minLeafSize = CurvePrimitive::max_size();
        settings.maxLeafSize = CurvePrimitive::max_size();
        settings.finished_range_threshold = numPrimitives/1000;
        if (settings.finished_range_threshold < 1000)
          settings.finished_range_threshold = inf;

        /* creates a leaf node */
        auto createLeaf = [&] (const PrimRef* prims, const range<size_t>& set, const FastAllocator::CachedAllocator& alloc) -> NodeRef {
          
          if (set.size() == 0)
            return BVH::emptyNode;

          const unsigned int geomID0 = prims[set.begin()].geomID();
          if (scene->get(geomID0)->getTypeMask() & Geometry::MTY_POINTS)
            return PointPrimitive::createLeaf(bvh,prims,set,alloc);
          else if (scene->get(geomID0)->getCurveBasis() == Geometry::GTY_BASIS_LINEAR)
            return LinePrimitive::createLeaf(bvh,prims,set,alloc);
          else
            return CurvePrimitive::createLeaf(bvh,prims,set,alloc);
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
           typename BVH::AABBNode::Create(),
           typename BVH::AABBNode::Set(),
           typename BVH::OBBNode::Create(),
           typename BVH::OBBNode::Set(),
           createLeaf,scene->progressInterface,
           reportFinishedRange,
           scene,prims.data(),pinfo,settings);
        
        bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
        
        /* if we allocated using the primrefarray we have to keep it alive */
        if (settings.finished_range_threshold != size_t(inf))
          bvh->alloc.share(prims);
        
        /* clear temporary data for static geometry */
        if (scene->isStaticAccel()) {
          prims.clear();
        }
        bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };
    
    /*! entry functions for the builder */
    Builder* BVH4Curve4vBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<4,Curve4v,Line4i,Point4i>((BVH4*)bvh,scene); }
    Builder* BVH4Curve4iBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<4,Curve4i,Line4i,Point4i>((BVH4*)bvh,scene); }

#if defined(__AVX__)
    Builder* BVH8Curve8vBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<8,Curve8v,Line8i,Point8i>((BVH8*)bvh,scene); }
    Builder* BVH4Curve8iBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<4,Curve8i,Line8i,Point8i>((BVH4*)bvh,scene); }
#endif

  }
}
#endif
