// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "bvh.h"
#include "bvh_builder.h"
#include "../builders/bvh_builder_msmblur.h"

#include "../builders/bvh_builder_hair.h"
#include "../builders/bvh_builder_msmblur_hair.h"

#include "../builders/primrefgen.h"
#include "../builders/splitter.h"

#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"
#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/object.h"
#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"

#include "../common/state.h"

#define PROFILE 0
#define PROFILE_RUNS 20

namespace embree
{
  namespace isa
  {
    MAYBE_UNUSED static const float travCost = 1.0f;
    MAYBE_UNUSED static const size_t DEFAULT_SINGLE_THREAD_THRESHOLD = 1024;

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N>
    struct VirtualCreateLeaf
    {
      typedef BVHN<N> BVH;
      virtual size_t operator() (BVH* bvh, PrimRef* prims, const range<size_t>& range, const FastAllocator::CachedAllocator& alloc) const = 0;
    };

    template<int N, typename Primitive0, typename Primitive1, typename Primitive2, typename Primitive3, typename Primitive4, typename Primitive5>
    struct CreateMultiLeaf6 : public VirtualCreateLeaf<N>
    {
      typedef BVHN<N> BVH;
      
      size_t operator() (BVH* bvh, PrimRef* prims, const range<size_t>& range, const FastAllocator::CachedAllocator& alloc) const
      {
        assert(range.size() > 0);
        const Leaf::Type ty = prims[range.begin()].type();
        switch (ty) {
        case 0: return Primitive0::createLeaf(alloc,prims,range,bvh);
        case 1: return Primitive1::createLeaf(alloc,prims,range,bvh);
        case 2: return Primitive2::createLeaf(alloc,prims,range,bvh);
        case 3: return Primitive3::createLeaf(alloc,prims,range,bvh);
        case 4: return Primitive4::createLeaf(alloc,prims,range,bvh);
        case 5: return Primitive5::createLeaf(alloc,prims,range,bvh);
        default: assert(false); return BVH::emptyNode;
        }
      }
    };

    template<int N>
    struct BVHNBuilderMultiSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      Geometry* mesh;
      Geometry::Type type;
      const VirtualCreateLeaf<N>& createLeaf;
      mvector<PrimRef> prims;
      GeneralBVHBuilder::Settings settings;
      bool primrefarrayalloc;

      BVHNBuilderMultiSAH (BVH* bvh, Scene* scene, Geometry::Type type, const VirtualCreateLeaf<N>& createLeaf, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize,
                           bool primrefarrayalloc = false)
        : bvh(bvh), scene(scene), mesh(nullptr), type(type), createLeaf(createLeaf), prims(scene->device,0),
          settings(sahBlockSize, minLeafSize, min(maxLeafSize,/*Primitive::max_size()*/4*BVH::maxLeafBlocks), travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(primrefarrayalloc) {} // FIXME: minLeafSize too small

      //BVHNBuilderMultiSAH (BVH* bvh, Geometry* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
      //  : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device,0), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(false) {}

      void build()
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
          mesh->numPrimitivesChanged = false;
        }

        /* if we use the primrefarray for allocations we have to take it back from the BVH */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.unshare(prims);

	/* skip build for empty scene */
        const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives(type,false);
        if (numPrimitives == 0) {
          bvh->clear();
          prims.clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderMultiSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif

            /* create primref array */
            if (primrefarrayalloc) {
              settings.primrefarrayalloc = numPrimitives/1000;
              if (settings.primrefarrayalloc < 1000)
                settings.primrefarrayalloc = inf;
            }

            /* enable os_malloc for static scenes or dynamic scenes with static geometry */
            if (mesh == NULL || mesh->isStatic())
              bvh->alloc.setOSallocation(true);

            /* initialize allocator */
            const size_t node_bytes = numPrimitives*sizeof(typename BVH::AlignedNodeMB)/(4*N);
            const size_t leaf_bytes = numPrimitives*44; //size_t(1.2*Primitive::blocks(numPrimitives)*sizeof(Primitive));
            bvh->alloc.init_estimate(node_bytes+leaf_bytes);
            settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,numPrimitives,node_bytes+leaf_bytes);
            prims.resize(numPrimitives); 

            PrimInfo pinfo = //mesh ?
              //createPrimRefArray<Mesh>  (mesh ,prims,bvh->scene->progressInterface) :
              createMultiPrimRefArray(scene,type,false,prims,bvh->scene->progressInterface);

            /* pinfo might has zero size due to invalid geometry */
            if (unlikely(pinfo.size() == 0))
            {
              bvh->clear();
              prims.clear();
              return;
            }

            /* call BVH builder */
            settings.branchingFactor = N;
            settings.maxDepth = BVH::maxBuildDepthLeaf;
            NodeRef root = BVHBuilderBinnedSAH::build<NodeRef>
              (FastAllocator::Create(&bvh->alloc),
               typename BVH::AlignedNode::Create2(),
               typename BVH::AlignedNode::Set3(&bvh->alloc,prims.data()),
               [&] (const range<size_t>& range, const FastAllocator::CachedAllocator& alloc) -> NodeRef { 
                  return createLeaf(bvh,prims.data(),range,alloc);
               },
               bvh->scene->progressInterface,prims.data(),pinfo,settings);
  
            bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
            bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));

#if PROFILE
          });
#endif

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();

        /* if we allocated using the primrefarray we have to keep it alive */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.share(prims);

        /* for static geometries we can do some cleanups */
        else if (staticGeom) {
          bvh->shrink();
          prims.clear();
        }
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N>
    struct VirtualCreateMSMBlurLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRecordMB4D NodeRecordMB4D;

      virtual const NodeRecordMB4D operator() (BVH* bvh, const SetMB& set, const FastAllocator::CachedAllocator& alloc) const = 0;
    };

    template<int N, typename Primitive0, typename Primitive1, typename Primitive2, typename Primitive3, typename Primitive4, typename Primitive5>
    struct CreateMSMBlurMultiLeaf6 : public VirtualCreateMSMBlurLeaf<N>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRecordMB4D NodeRecordMB4D;

      __forceinline const NodeRecordMB4D operator() (BVH* bvh, const SetMB& set, const FastAllocator::CachedAllocator& alloc) const
      {
        assert(set.object_range.size() > 0);
        const Leaf::Type ty = (*set.prims)[set.object_range.begin()].type();
        switch (ty) {
        case 0: return Primitive0::createLeafMB(set,alloc,bvh);
        case 1: return Primitive1::createLeafMB(set,alloc,bvh);
        case 2: return Primitive2::createLeafMB(set,alloc,bvh);
        case 3: return Primitive3::createLeafMB(set,alloc,bvh);
        case 4: return Primitive4::createLeafMB(set,alloc,bvh);
        case 5: return Primitive5::createLeafMB(set,alloc,bvh);
        default: assert(false); return NodeRecordMB4D(BVH::emptyNode,empty,empty);
        }
      }

      BVH* bvh;
    };

    /* Motion blur BVH with 4D nodes and internal time splits */
    template<int N>
    struct BVHNMultiBuilderMBlurSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;
      typedef typename BVHN<N>::NodeRecordMB NodeRecordMB;
      typedef typename BVHN<N>::AlignedNodeMB AlignedNodeMB;

      BVH* bvh;
      Scene* scene;
      Geometry::Type type;
      const VirtualCreateMSMBlurLeaf<N>& createLeaf;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;

      BVHNMultiBuilderMBlurSAH (BVH* bvh, Scene* scene, Geometry::Type type, const VirtualCreateMSMBlurLeaf<N>& createLeaf, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), type(type), createLeaf(createLeaf), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,/*Primitive::max_size()*/BVH::maxLeafBlocks)) {} // FIXME: Primitive::max_size() assumed to be 4

      void build()
      {
	/* skip build for empty scene */
        const size_t numPrimitives = scene->getNumPrimitives(type,true);
        if (numPrimitives == 0) { bvh->clear(); return; }

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderMultiMBlurSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif

        //const size_t numTimeSteps = scene->getNumTimeSteps<Mesh,true>();
        //const size_t numTimeSegments = numTimeSteps-1; assert(numTimeSteps > 1);

        //if (numTimeSegments == 1)
        //  buildSingleSegment(numPrimitives);
        //else
          buildMultiSegment(numPrimitives);

#if PROFILE
          });
#endif

	/* clear temporary data for static geometry */
        if (scene->isStatic()) bvh->shrink();
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void buildMultiSegment(size_t numPrimitives)
      {
        /* create primref array */
        mvector<PrimRefMB> prims(scene->device,numPrimitives);
        PrimInfoMB pinfo = createMultiPrimRefArrayMSMBlur(scene,type,prims,bvh->scene->progressInterface);

        /* estimate acceleration structure size */
        const size_t node_bytes = pinfo.num_time_segments*sizeof(AlignedNodeMB)/(4*N);
        const size_t leaf_bytes = size_t(1.2*pinfo.num_time_segments*44);//sizeof(Primitive)); // FIXME: assumes 44 bytes for primitive
        bvh->alloc.init_estimate(node_bytes+leaf_bytes);

        /* settings for BVH build */
        BVHBuilderMSMBlur::Settings settings;
        settings.branchingFactor = N;
        settings.maxDepth = BVH::maxDepth;
        settings.logBlockSize = __bsr(sahBlockSize);
        settings.minLeafSize = minLeafSize;
        settings.maxLeafSize = maxLeafSize;
        settings.travCost = travCost;
        settings.intCost = intCost;
        settings.singleLeafTimeSegment = true; //Primitive::singleTimeSegment; // FIXME: this is very conservative
        settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,pinfo.size(),node_bytes+leaf_bytes);
        
        /* build hierarchy */
        auto root =
          BVHBuilderMSMBlur::build<NodeRef>(prims,pinfo,scene->device,
                                            VirtualRecalculatePrimRef(scene),
                                            typename BVH::CreateAlloc(bvh),
                                            typename BVH::AlignedNodeMB4D::Create(),
                                            typename BVH::AlignedNodeMB4D::Set(),
                                            [&] (const BVHBuilderMSMBlur::BuildRecord& current, const FastAllocator::CachedAllocator& alloc) { 
                                              return createLeaf(bvh,current.prims,alloc);
                                            },
                                            bvh->scene->progressInterface,
                                            settings);

        bvh->set(root.ref,root.lbounds,pinfo.num_time_segments);
      }

      void clear() {
      }
    };

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N>
    struct BVHNOBBBuilderMultiSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      Geometry::Type type;
      const VirtualCreateLeaf<N>& createLeaf;
      mvector<PrimRef> prims;

      BVHNOBBBuilderMultiSAH (BVH* bvh, Scene* scene, Geometry::Type type, const VirtualCreateLeaf<N>& createLeaf)
        : bvh(bvh), scene(scene), type(type), createLeaf(createLeaf), prims(scene->device,0) {}
      
      void build() 
      {
        /* fast path for empty BVH */
        const size_t numPrimitives = scene->getNumPrimitives(type,false);
        if (numPrimitives == 0) {
          prims.clear();
          bvh->clear();
          return;
        }

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "MultiOBBBuilderSAH");

        //profile(1,5,numPrimitives,[&] (ProfileTimer& timer) {
        
        /* create primref array */
        prims.resize(numPrimitives);
        const PrimInfo pinfo = createMultiPrimRefArray(scene,type,false,prims,scene->progressInterface);

        /* estimate acceleration structure size */
        const size_t node_bytes = pinfo.size()*sizeof(typename BVH::UnalignedNode)/(4*N);
        const size_t leaf_bytes = pinfo.size()*44; //sizeof(Primitive); FIXME wrong size
        bvh->alloc.init_estimate(node_bytes+leaf_bytes);
        
        /* builder settings */
        BVHBuilderHair::Settings settings;
        settings.branchingFactor = N;
        settings.maxDepth = BVH::maxBuildDepthLeaf;
        settings.logBlockSize = 0;
        settings.minLeafSize = 1;
        settings.maxLeafSize = BVH::maxLeafBlocks;

        /* build hierarchy */
        typename BVH::NodeRef root = BVHBuilderHair::build<NodeRef>
          (typename BVH::CreateAlloc(bvh),
           typename BVH::AlignedNode::Create(),
           typename BVH::AlignedNode::Set(),
           typename BVH::UnalignedNode::Create(),
           typename BVH::UnalignedNode::Set(),
           [&] (size_t depth, const range<size_t>& range, const FastAllocator::CachedAllocator& alloc) -> NodeRef {
            return createLeaf(bvh,prims.data(),range,alloc);
           },
           scene->progressInterface,scene,prims.data(),pinfo,settings);
        
        bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
        
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

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

#if defined(__AVX__)
  
    struct BVH8MultiFastSceneBuilderSelect : public Builder 
    {
      BVH8* bvh;
      Scene* scene;
      Geometry::Type type;
      Ref<Builder> builder;

      BVH8MultiFastSceneBuilderSelect ( BVH8* bvh, Scene* scene, Geometry::Type type )
        : bvh(bvh), scene(scene), type(type) {}

      virtual void build() 
      {
        static CreateMultiLeaf6<8,
                                Triangle4,
                                Triangle4vMB,
                                Quad4v,
                                Quad4iMB,
                                Bezier1v,
                                Bezier1iMB> createLeaf;

        static CreateMSMBlurMultiLeaf6<8,
                                       Triangle4,
                                       Triangle4vMB,
                                       Quad4v,
                                       Quad4iMB,
                                       Bezier1v,
                                       Bezier1iMB> createLeafMB;
        
        if (!builder) 
        {
          const size_t num1 = scene->getNumPrimitives(type,false);
          const size_t num2 = scene->getNumPrimitives(type,true);
          
          if ((type & Geometry::BEZIER_CURVES) && scene->getNumPrimitives(Geometry::BEZIER_CURVES,true))
          {
            if (num1 < num2) {
              assert(false);
            } else {
              builder = new BVHNOBBBuilderMultiSAH<8>((BVH8*)bvh,scene,type,createLeaf); //,4,1.0f,4,inf); 
            }
          }
          else
          {
            if (num1 < num2) {
              builder = new BVHNMultiBuilderMBlurSAH<8>((BVH8*)bvh,scene,type,createLeafMB,4,1.0f,4,inf); 
            } else {
              builder = new BVHNBuilderMultiSAH<8>((BVH8*)bvh,scene,type,createLeaf,4,1.0f,4,inf); 
            }
          }
        }
        builder->build();
      }

      virtual void deleteGeometry(size_t geomID) {
        if (builder) builder->deleteGeometry(geomID);
      }

      virtual void clear() {
        if (builder) builder->clear();
      }
    };

    Builder* BVH8MultiFastSceneBuilder     (void* bvh, Scene* scene, Geometry::Type type) { 
      return new BVH8MultiFastSceneBuilderSelect((BVH8*)bvh,scene,type);
    }

#endif
  }
}
