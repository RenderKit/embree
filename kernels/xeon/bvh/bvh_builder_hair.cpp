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

#include "bvh_builder_hair.h"
#include "../builders/primrefgen.h"

#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"

namespace embree
{
  namespace isa
  {
    template<int N, typename Primitive>
    struct BVHNHairBuilderSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::Node Node;
      typedef typename BVH::UnalignedNode UnalignedNode;
      typedef typename BVH::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      mvector<BezierPrim> prims;

      BVHNHairBuilderSAH (BVH* bvh, Scene* scene)
        : bvh(bvh), scene(scene), prims(scene->device) {}
      
      void build(size_t, size_t) 
      {
        /* progress monitor */
        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);

        /* fast path for empty BVH */
        const size_t numPrimitives = scene->getNumPrimitives<BezierCurves,1>();
        if (numPrimitives == 0) {
          prims.clear();
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderHairSAH");

        //profile(1,5,numPrimitives,[&] (ProfileTimer& timer) {
        
        /* create primref array */
        bvh->alloc.init_estimate(numPrimitives*sizeof(Primitive));
        prims.resize(numPrimitives);
        const PrimInfo pinfo = createBezierRefArray<1>(scene,prims,virtualprogress);
        
        /* build hierarchy */
        typename BVH::NodeRef root = bvh_obb_builder_binned_sah<N>
          (
            [&] () { return bvh->alloc.threadLocal2(); },

            [&] (const PrimInfo* children, const size_t numChildren, 
                 HeuristicArrayBinningSAH<BezierPrim> alignedHeuristic, 
                 FastAllocator::ThreadLocal2* alloc) -> Node*
            {
              Node* node = (Node*) alloc->alloc0.malloc(sizeof(Node),16); node->clear();
              for (size_t i=0; i<numChildren; i++)
                node->set(i,children[i].geomBounds);
              return node;
            },
            
            [&] (const PrimInfo* children, const size_t numChildren, 
                 UnalignedHeuristicArrayBinningSAH<BezierPrim> unalignedHeuristic, 
                 FastAllocator::ThreadLocal2* alloc) -> UnalignedNode*
            {
              UnalignedNode* node = (UnalignedNode*) alloc->alloc0.malloc(sizeof(UnalignedNode),16); node->clear();
              for (size_t i=0; i<numChildren; i++) 
              {
                const LinearSpace3fa space = unalignedHeuristic.computeAlignedSpace(children[i]); 
                const PrimInfo       sinfo = unalignedHeuristic.computePrimInfo(children[i],space);
                node->set(i,OBBox3fa(space,sinfo.geomBounds));
              }
              return node;
            },

            [&] (size_t depth, const PrimInfo& pinfo, FastAllocator::ThreadLocal2* alloc) -> NodeRef
            {
              size_t items = pinfo.size();
              size_t start = pinfo.begin;
              Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
              NodeRef node = bvh->encodeLeaf((char*)accel,items);
              for (size_t i=0; i<items; i++) {
                accel[i].fill(prims.data(),start,pinfo.end,bvh->scene,false);
              }
              return node;
            },
            progress,
            prims.data(),pinfo,N,BVH::maxBuildDepthLeaf,1,1,BVH::maxLeafBlocks);
        
        bvh->set(root,pinfo.geomBounds,pinfo.size());
        
        //});
        
        /* clear temporary data for static geometry */
        if (scene->isStatic()) {
          prims.clear();
          bvh->shrink();
        }
        bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };


    template<int N, typename Primitive>
    struct BVHNHairMBBuilderSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeMB NodeMB;
      typedef typename BVH::UnalignedNodeMB UnalignedNodeMB;
      typedef typename BVH::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      mvector<BezierPrim> prims;

      BVHNHairMBBuilderSAH (BVH* bvh, Scene* scene)
        : bvh(bvh), scene(scene), prims(scene->device) {}
      
      void build(size_t, size_t) 
      {
        /* progress monitor */
        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);

        /* fast path for empty BVH */
        const size_t numPrimitives = scene->getNumPrimitives<BezierCurves,2>();
        if (numPrimitives == 0) {
          prims.clear();
          bvh->set(BVH::emptyNode,empty,0);
          return;
        }
        
        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderMBHairSAH");

        //profile(1,5,numPrimitives,[&] (ProfileTimer& timer) {

        /* create primref array */
        bvh->alloc.init_estimate(numPrimitives*sizeof(Primitive));
        prims.resize(numPrimitives);
        const PrimInfo pinfo = createBezierRefArray<2>(scene,prims,virtualprogress);
        
        NodeRef root = bvh_obb_builder_binned_sah<N>
          (
            [&] () { return bvh->alloc.threadLocal2(); },

            [&] (const PrimInfo* children, const size_t numChildren, HeuristicArrayBinningSAH<BezierPrim> alignedHeuristic, FastAllocator::ThreadLocal2* alloc) -> NodeMB*
            {
              NodeMB* node = (NodeMB*) alloc->alloc0.malloc(sizeof(NodeMB),16); node->clear();
              for (size_t i=0; i<numChildren; i++) 
              {
                std::pair<BBox3fa,BBox3fa> bounds = alignedHeuristic.computePrimInfoMB(scene,children[i]);
                node->set(i,bounds.first,bounds.second);
              }
              return node;
            },
            
            [&] (const PrimInfo* children, const size_t numChildren, UnalignedHeuristicArrayBinningSAH<BezierPrim> unalignedHeuristic, FastAllocator::ThreadLocal2* alloc) -> UnalignedNodeMB*
            {
              UnalignedNodeMB* node = (UnalignedNodeMB*) alloc->alloc0.malloc(sizeof(UnalignedNodeMB),16); node->clear();
              for (size_t i=0; i<numChildren; i++) 
              {
                const AffineSpace3fa space = unalignedHeuristic.computeAlignedSpaceMB(scene,children[i]); 
                UnalignedHeuristicArrayBinningSAH<BezierPrim>::PrimInfoMB pinfo = unalignedHeuristic.computePrimInfoMB(scene,children[i],space);
                node->set(i,space,pinfo.s0t0,pinfo.s1t1);
              }
              return node;
            },

            [&] (size_t depth, const PrimInfo& pinfo, FastAllocator::ThreadLocal2* alloc) -> NodeRef
            {
              size_t items = pinfo.size();
              size_t start = pinfo.begin;
              Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
              NodeRef node = bvh->encodeLeaf((char*)accel,items);
              for (size_t i=0; i<items; i++) {
                accel[i].fill(prims.data(),start,pinfo.end,bvh->scene,false);
              }
              return node;
            },
            progress,
            prims.data(),pinfo,N,BVH::maxBuildDepthLeaf,1,1,BVH::maxLeafBlocks);
        
        bvh->set(root,pinfo.geomBounds,pinfo.size());

        //});
        
        /* clear temporary data for static geometry */
        if (scene->isStatic()) {
          prims.clear();
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
    Builder* BVH4Bezier1iMBBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVHNHairMBBuilderSAH<4,Bezier1i>((BVH4*)bvh,scene); }

#if defined(__AVX__)
    Builder* BVH8Bezier1vBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<8,Bezier1v>((BVH8*)bvh,scene); }
    Builder* BVH8Bezier1iBuilder_OBB_New   (void* bvh, Scene* scene, size_t mode) { return new BVHNHairBuilderSAH<8,Bezier1i>((BVH8*)bvh,scene); }
    Builder* BVH8Bezier1iMBBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVHNHairMBBuilderSAH<8,Bezier1i>((BVH8*)bvh,scene); }
#endif
  }
}
