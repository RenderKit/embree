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

#include "../builders/primrefgen.h"
#include "../builders/splitter.h"

#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"
#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglei.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei_mb.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/quadi_mb.h"
#include "../geometry/object.h"

#include "../common/state.h"

// FIXME: remove after removing BVHNBuilderMBlurRootTimeSplitsSAH
#include "../../common/algorithms/parallel_for_for.h"
#include "../../common/algorithms/parallel_for_for_prefix_sum.h"

#define PROFILE 0
#define PROFILE_RUNS 20

namespace embree
{
  namespace isa
  {
    MAYBE_UNUSED static const float travCost = 1.0f;
    MAYBE_UNUSED static const size_t DEFAULT_SINGLE_THREAD_THRESHOLD = 1024;
    MAYBE_UNUSED static const size_t HIGH_SINGLE_THREAD_THRESHOLD    = 3*1024;

    typedef FastAllocator::ThreadLocal2 Allocator;

    template<int N, typename Primitive>
    struct CreateLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      __forceinline CreateLeaf (BVH* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      template<typename BuildRecord>
      __forceinline NodeRef operator() (const BuildRecord& current, Allocator* alloc) const
      {
        size_t n = current.prims.size();
        size_t items = Primitive::blocks(n);
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1->malloc(items*sizeof(Primitive),BVH::byteAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene);
        }
        return node;
      }

      BVH* bvh;
      PrimRef* prims;
    };


    template<int N, typename Primitive>
    struct CreateLeafQuantized
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      __forceinline CreateLeafQuantized (BVH* bvh, PrimRef* prims) : bvh(bvh), prims(prims) {}
      
      __forceinline NodeRef operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) const
      {
        size_t n = current.prims.size();
        size_t items = Primitive::blocks(n);
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1->malloc(items*sizeof(Primitive),BVH::byteAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,current.prims.end(),bvh->scene);
        }
        return node;
      }

      BVH* bvh;
      PrimRef* prims;
    };

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      GeneralBVHBuilder::Settings settings;
      bool primrefarrayalloc;
      
      BVHNBuilderSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, 
                      const size_t mode, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD, bool primrefarrayalloc = false)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), 
          settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, singleThreadThreshold), primrefarrayalloc(primrefarrayalloc) {}

      BVHNBuilderSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, 
                      const size_t mode, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, singleThreadThreshold) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build_group(GeometryGroup* group)
      {
        /* we reset the allocator when the group size changed */
        if (group && group->numPrimitivesChanged) {
          bvh->alloc.clear();
          group->numPrimitivesChanged = false;
        }

	/* skip build for empty scene */
        size_t numPrimitives = 0;
        for (size_t i=0; i<group->size(); i++)
          numPrimitives += (*group)[i]->size();

        if (numPrimitives == 0) {
          prims.clear();
          bvh->clear();
          return;
        }
        
        /* create primref array */
        prims.resize(numPrimitives);
        PrimInfo pinfo = createGroupPrimRefArray<Mesh>(group ,prims,bvh->scene->progressInterface);
          
        /* pinfo might has zero size due to invalid geometry */
        if (unlikely(pinfo.size() == 0)) {
          prims.clear(); bvh->clear(); return;
        }
        
        /* call BVH builder */            
        bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef),settings.singleThreadThreshold != DEFAULT_SINGLE_THREAD_THRESHOLD);
        NodeRef root = BVHNBuilderVirtual<N>::build(&bvh->alloc,CreateLeaf<N,Primitive>(bvh,prims.data()),bvh->scene->progressInterface,prims.data(),pinfo,settings);
        bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
        bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));

	/* clear temporary data for static geometry */
	if (group->isStatic()) {
          prims.clear(); 
          bvh->shrink();
        }
	bvh->cleanup();
      }

      void build() 
      {
        if (mesh && mesh->getType() == Geometry::GROUP) {
          build_group((GeometryGroup*)mesh);
          return;
        }

        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
          mesh->numPrimitivesChanged = false;
        }

        /* if we use the primrefarray for allocations we have to take it back from the BVH */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.unshare(prims);

	/* skip build for empty scene */
        const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,false>();
        if (numPrimitives == 0) {
          bvh->clear();
          prims.clear();
          return;
        }
        
        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderSAH");
        
#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif

            /* create primref array */
            if (primrefarrayalloc) {
              settings.primrefarrayalloc = numPrimitives/1000;
              if (settings.primrefarrayalloc < 1000)
                settings.primrefarrayalloc = inf;
            }
            bvh->alloc.init_estimate(numPrimitives*sizeof(PrimRef),settings.singleThreadThreshold != DEFAULT_SINGLE_THREAD_THRESHOLD,settings.primrefarrayalloc != size_t(inf));
            prims.resize(numPrimitives); 

            PrimInfo pinfo = mesh ? 
              createPrimRefArray<Mesh>  (mesh ,prims,bvh->scene->progressInterface) : 
              createPrimRefArray<Mesh,false>(scene,prims,bvh->scene->progressInterface);

            /* pinfo might has zero size due to invalid geometry */
            if (unlikely(pinfo.size() == 0))
            {
              bvh->clear();
              prims.clear();
              return;
            }

            /* call BVH builder */            
            NodeRef root = BVHNBuilderVirtual<N>::build(&bvh->alloc,CreateLeaf<N,Primitive>(bvh,prims.data()),bvh->scene->progressInterface,prims.data(),pinfo,settings);
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

    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderSAHQuantized : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      GeneralBVHBuilder::Settings settings;

      BVHNBuilderSAHQuantized (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, singleThreadThreshold) {}

      BVHNBuilderSAHQuantized (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, singleThreadThreshold) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build() 
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
          mesh->numPrimitivesChanged = false;
        }

	/* skip build for empty scene */
        const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,false>();
        if (numPrimitives == 0) {
          prims.clear();
          bvh->clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::QBVH" + toString(N) + "BuilderSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif
            /* create primref array */
            prims.resize(numPrimitives);
            PrimInfo pinfo = mesh ? 
              createPrimRefArray<Mesh>  (mesh ,prims,bvh->scene->progressInterface) : 
              createPrimRefArray<Mesh,false>(scene,prims,bvh->scene->progressInterface);
        
            /* call BVH builder */
            bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef),settings.singleThreadThreshold != DEFAULT_SINGLE_THREAD_THRESHOLD);
            NodeRef root = BVHNBuilderQuantizedVirtual<N>::build(&bvh->alloc,CreateLeafQuantized<N,Primitive>(bvh,prims.data()),bvh->scene->progressInterface,prims.data(),pinfo,settings);
            bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
            //bvh->layoutLargeNodes(pinfo.size()*0.005f); // FIXME: COPY LAYOUT FOR LARGE NODES !!!
#if PROFILE
          }); 
#endif	

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
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

    template<int N, typename Primitive>
    struct CreateMSMBlurLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::NodeRecordMB NodeRecordMB;

      __forceinline CreateMSMBlurLeaf (BVH* bvh, PrimRef* prims, size_t time) : bvh(bvh), prims(prims), time(time) {}
      
      __forceinline NodeRecordMB operator() (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) const
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        Primitive* accel = (Primitive*) alloc->alloc1->malloc(items*sizeof(Primitive),BVH::byteAlignment);
        NodeRef node = bvh->encodeLeaf((char*)accel,items);

        LBBox3fa allBounds = empty;
        for (size_t i=0; i<items; i++)
          allBounds.extend(accel[i].fillMB(prims, start, current.prims.end(), bvh->scene, time, bvh->numTimeSteps));
        
        return NodeRecordMB(node,allBounds);
      }

      BVH* bvh;
      PrimRef* prims;
      size_t time;
    };

    /* Motion blur BVH with multiple roots */
    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderMSMBlurSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;
      BVH* bvh;
      Scene* scene;
      mvector<PrimRef> prims; 
      GeneralBVHBuilder::Settings settings;

      BVHNBuilderMSMBlurSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), scene(scene), prims(scene->device), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, singleThreadThreshold) {}

      void build() 
      {
	/* skip build for empty scene */
        const size_t numPrimitives = scene->getNumPrimitives<Mesh,true>();

        if (numPrimitives == 0) {
          prims.clear();
          bvh->clear();
          return;
        }      

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderMSMBlurSAH");
	
        /* allocate buffers */
        bvh->numTimeSteps = scene->getNumTimeSteps<Mesh,true>();
        const size_t numTimeSegments = bvh->numTimeSteps-1; assert(bvh->numTimeSteps > 1);
        prims.resize(numPrimitives);
        bvh->alloc.init_estimate(numPrimitives*sizeof(PrimRef)*numTimeSegments,settings.singleThreadThreshold != DEFAULT_SINGLE_THREAD_THRESHOLD);
        NodeRef* roots = (NodeRef*) bvh->alloc.threadLocal()->malloc(sizeof(NodeRef)*numTimeSegments,BVH::byteNodeAlignment);

        /* build BVH for each timestep */
        avector<BBox3fa> bounds(bvh->numTimeSteps);
        size_t num_bvh_primitives = 0;
        for (size_t t=0; t<numTimeSegments; t++)
        {
          /* call BVH builder */
          NodeRef root; LBBox3fa tbounds;
          const PrimInfo pinfo = createPrimRefArrayMBlur<Mesh>(t,bvh->numTimeSteps,scene,prims,bvh->scene->progressInterface);
          if (pinfo.size()) {
            auto rootRecord = BVHNBuilderMblurVirtual<N>::build(&bvh->alloc,CreateMSMBlurLeaf<N,Primitive>(bvh,prims.data(),t),bvh->scene->progressInterface,prims.data(),pinfo,settings);
            root = rootRecord.ref;
            tbounds = rootRecord.lbounds;
          } else {
            tbounds = LBBox3fa(empty);
            root = BVH::emptyNode;
          }
          roots[t] = root;
          bounds[t+0] = tbounds.bounds0;
          bounds[t+1] = tbounds.bounds1;
          num_bvh_primitives = max(num_bvh_primitives,pinfo.size());
        }
        bvh->set(NodeRef((size_t)roots),LBBox3fa(bounds),num_bvh_primitives);
        bvh->msmblur = true;

	/* clear temporary data for static geometry */
	if (scene->isStatic()) 
        {
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

    /* Motion blur BVH with 4D nodes and root time splits */
    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderMBlurRootTimeSplitsSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;
      typedef typename BVHN<N>::NodeRecordMB NodeRecordMB;
      typedef typename BVHN<N>::NodeRecordMB4D NodeRecordMB4D;
      typedef typename BVHN<N>::AlignedNodeMB AlignedNodeMB;
      typedef typename BVHN<N>::AlignedNodeMB4D AlignedNodeMB4D;
      BVH* bvh;
      Scene* scene;
      mvector<PrimRef> prims; 
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const size_t mode;
      const size_t singleThreadThreshold;
      
      BVHNBuilderMBlurRootTimeSplitsSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, size_t mode, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), scene(scene), prims(scene->device), 
          sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)), mode(mode), singleThreadThreshold(singleThreadThreshold) {}
      
      NodeRef recurse(const range<size_t>& dti)
      {
        assert(dti.size() > 0);
        if (dti.size() == 1)
        {
          const BBox1f dt(float(dti.begin())/float(bvh->numTimeSteps-1),
                          float(dti.end  ())/float(bvh->numTimeSteps-1));

          const PrimInfo pinfo = createPrimRefArrayMBlur<Mesh>(dti.begin(),bvh->numTimeSteps,scene,prims,bvh->scene->progressInterface);

          /* settings for BVH build */
          GeneralBVHBuilder::Settings settings;
          settings.branchingFactor = N;
          settings.maxDepth = BVH::maxBuildDepthLeaf;
          settings.logBlockSize = __bsr(sahBlockSize);
          settings.minLeafSize = minLeafSize;
          settings.maxLeafSize = maxLeafSize;
          settings.travCost = travCost;
          settings.intCost = intCost;
          settings.singleThreadThreshold = singleThreadThreshold;

          auto root = BVHBuilderBinnedSAH::build<NodeRecordMB>
            (typename BVH::CreateAlloc(bvh),typename BVH::AlignedNodeMB::Create2(),typename BVH::AlignedNodeMB::Set2Global(dt),
             CreateMSMBlurLeaf<N,Primitive>(bvh,prims.data(),dti.begin()),bvh->scene->progressInterface,
             prims.data(),pinfo,settings);
          
          return root.ref;
        }
        else
        {
          std::vector<range<size_t>> c;
          c.push_back(dti);
          while (c.size() < N)
          {
            std::pop_heap(c.begin(),c.end());
            auto r = c.back();
            if (r.size() == 1) break;
            c.pop_back();
            auto r2 = r.split();
            c.push_back(std::get<0>(r2)); std::push_heap(c.begin(),c.end());
            c.push_back(std::get<1>(r2)); std::push_heap(c.begin(),c.end());
          }
          
          LBBox3fa lbounds = empty;
          AlignedNodeMB4D* node = (AlignedNodeMB4D*) bvh->alloc.threadLocal2()->alloc0->malloc(sizeof(AlignedNodeMB4D), BVH::byteNodeAlignment); node->clear();
          for (size_t i=0; i<c.size(); i++) 
          {
            NodeRef cnode = recurse(c[i]);

            const BBox1f dt(float(c[i].begin())/float(bvh->numTimeSteps-1),
                            float(c[i].end  ())/float(bvh->numTimeSteps-1));

            LBBox3fa cbounds = linearBounds(scene,dt);

            node->set(i,NodeRecordMB4D(cnode,cbounds,dt));
            lbounds.extend(cbounds);
          }
          return bvh->encodeNode(node);
        }
      }
      
      void build() 
      {
	/* skip build for empty scene */
        const size_t numPrimitives = scene->getNumPrimitives<Mesh,true>();
        if (numPrimitives == 0) { prims.clear(); bvh->clear(); return;  }      
        
        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderMBlurRootTimeSplitsSAH");
	
        /* allocate buffers */
        const size_t numTimeSteps = scene->getNumTimeSteps<Mesh,true>();
        const size_t numTimeSegments = numTimeSteps-1; assert(numTimeSteps > 1);
        prims.resize(numPrimitives);
        bvh->alloc.init_estimate(numPrimitives*sizeof(PrimRef)*numTimeSegments);
        bvh->numTimeSteps = numTimeSteps;
        
        NodeRef root = recurse(make_range(size_t(0),numTimeSegments));
        LBBox3fa rootBounds = linearBounds(scene,BBox1f(0.0f,1.0f));
        bvh->set(root,rootBounds,numPrimitives);
        
        /* clear temporary data for static geometry */
        if (scene->isStatic()) 
        {
          prims.clear();
          bvh->shrink();
        }
        bvh->cleanup();
        bvh->postBuild(t0);
      }
        
      void clear() {
        prims.clear();
      }

      LBBox3fa linearBounds(Scene* scene, BBox1f t0t1)
      {
        ParallelForForPrefixSumState<LBBox3fa> pstate;
        Scene::Iterator<Mesh,true> iter(scene);

        /* first try */
        pstate.init(iter,size_t(1024));
        LBBox3fa lbounds = parallel_for_for_prefix_sum( pstate, iter, LBBox3fa(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const LBBox3fa& base) -> LBBox3fa
        {
          LBBox3fa lbounds(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            LBBox3fa bounds = empty;
            if (!mesh->linearBounds(j,t0t1,bounds)) continue;
            lbounds.extend(bounds);
          }
          return lbounds;
        }, [](const LBBox3fa& a, const LBBox3fa& b) -> LBBox3fa { return merge(a,b); });

        return lbounds;
      }
    };

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

    template<int N, typename Mesh, typename Primitive>
    struct CreateMBlurLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::NodeRecordMB4D NodeRecordMB4D;

      __forceinline CreateMBlurLeaf (BVH* bvh) : bvh(bvh) {}
      
      __forceinline const NodeRecordMB4D operator() (const BVHBuilderMSMBlur::BuildRecord& current, Allocator* alloc) const
      {
        size_t items = Primitive::blocks(current.prims.object_range.size());
        size_t start = current.prims.object_range.begin();
        Primitive* accel = (Primitive*) alloc->alloc1->malloc(items*sizeof(Primitive),BVH::byteNodeAlignment);
        NodeRef node = bvh->encodeLeaf((char*)accel,items);
        LBBox3fa allBounds = empty;
        for (size_t i=0; i<items; i++)
          allBounds.extend(accel[i].fillMB(current.prims.prims->data(), start, current.prims.object_range.end(), bvh->scene, current.prims.time_range));
        return NodeRecordMB4D(node,allBounds,current.prims.time_range);
      }

      BVH* bvh;
    };

    /* Motion blur BVH with 4D nodes and internal time splits */
    template<int N, typename Mesh, typename Primitive>
    struct BVHNBuilderMBlurSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;
      typedef typename BVHN<N>::NodeRecordMB NodeRecordMB;
      typedef typename BVHN<N>::AlignedNodeMB AlignedNodeMB;

      BVH* bvh;
      Scene* scene;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const size_t singleThreadThreshold;

      BVHNBuilderMBlurSAH (BVH* bvh, Scene* scene, 
                           const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, 
                           const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), scene(scene), 
          sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)), 
          singleThreadThreshold(singleThreadThreshold) {}

      void build() 
      {
	/* skip build for empty scene */
        const size_t numPrimitives = scene->getNumPrimitives<Mesh,true>();
        if (numPrimitives == 0) { bvh->clear(); return; }
        
        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderMBlurSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif

        const size_t numTimeSteps = scene->getNumTimeSteps<Mesh,true>();
        const size_t numTimeSegments = numTimeSteps-1; assert(numTimeSteps > 1);
        bvh->numTimeSteps = numTimeSteps;

        if (numTimeSegments == 1)
          buildSingleSegment(numPrimitives);
        else
          buildMultiSegment(numPrimitives);

#if PROFILE
          });
#endif

	/* clear temporary data for static geometry */
        if (scene->isStatic()) bvh->shrink();
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void buildSingleSegment(size_t numPrimitives)
      { 
        /* create primref array */
        mvector<PrimRef> prims(scene->device,numPrimitives);
        const PrimInfo pinfo = createPrimRefArrayMBlur<Mesh>(0,2,scene,prims,bvh->scene->progressInterface);

        /* estimate acceleration structure size */
        const size_t node_bytes = pinfo.size()*sizeof(AlignedNodeMB)/(4*N);
        const size_t leaf_bytes = size_t(1.2*Primitive::blocks(pinfo.size())*sizeof(Primitive));
        bvh->alloc.init_estimate(node_bytes+leaf_bytes);

        /* settings for BVH build */
        GeneralBVHBuilder::Settings settings;
        settings.branchingFactor = N;
        settings.maxDepth = BVH::maxBuildDepthLeaf;
        settings.logBlockSize = __bsr(sahBlockSize);
        settings.minLeafSize = minLeafSize;
        settings.maxLeafSize = maxLeafSize;
        settings.travCost = travCost;
        settings.intCost = intCost;
        settings.singleThreadThreshold = singleThreadThreshold;

        /* build hierarchy */
        auto root = BVHBuilderBinnedSAH::build<NodeRecordMB>
          (typename BVH::CreateAlloc(bvh),typename BVH::AlignedNodeMB::Create2(),typename BVH::AlignedNodeMB::Set2(),
           CreateMSMBlurLeaf<N,Primitive>(bvh,prims.data(),0),bvh->scene->progressInterface,
           prims.data(),pinfo,settings);

        bvh->set(root.ref,root.lbounds,pinfo.size());
      }

      void buildMultiSegment(size_t numPrimitives)
      {
        /* create primref array */
        mvector<PrimRefMB> prims(scene->device,numPrimitives);
        PrimInfoMB pinfo = createPrimRefArrayMSMBlur<Mesh>(scene,prims,bvh->scene->progressInterface);

        /* estimate acceleration structure size */
        const size_t node_bytes = pinfo.num_time_segments*sizeof(AlignedNodeMB)/(4*N);
        const size_t leaf_bytes = size_t(1.2*Primitive::blocks(pinfo.num_time_segments)*sizeof(Primitive));
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
        settings.singleLeafTimeSegment = Primitive::singleTimeSegment;
        settings.singleThreadThreshold = singleThreadThreshold;
        
        /* build hierarchy */
        auto root =
          BVHBuilderMSMBlur::build<NodeRef>(prims,pinfo,scene->device,
                                             RecalculatePrimRef<Mesh>(scene),
                                             typename BVH::CreateAlloc(bvh),
                                             typename BVH::CreateAlignedNodeMB4D(),
                                             typename BVH::SetAlignedNodeMB4D(),
                                             CreateMBlurLeaf<N,Mesh,Primitive>(bvh),
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

    template<int N, typename Mesh, typename Primitive, typename Splitter>
    struct BVHNBuilderFastSpatialSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims0;
      GeneralBVHBuilder::Settings settings;
      const float splitFactor;

      BVHNBuilderFastSpatialSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), scene(scene), mesh(nullptr), prims0(scene->device), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, singleThreadThreshold),
          splitFactor(scene->device->max_spatial_split_replications) {}

      BVHNBuilderFastSpatialSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims0(bvh->device), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, singleThreadThreshold),
          splitFactor(scene->device->max_spatial_split_replications) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build() 
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
          mesh->numPrimitivesChanged = false;
        }

	/* skip build for empty scene */
        const size_t numOriginalPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,false>();
        if (numOriginalPrimitives == 0) {
          prims0.clear();
          bvh->clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderFastSpatialSAH");

        /* create primref array */
        const size_t numSplitPrimitives = max(numOriginalPrimitives,size_t(splitFactor*numOriginalPrimitives));
        prims0.resize(numSplitPrimitives);
        PrimInfo pinfo = mesh ? 
          createPrimRefArray<Mesh>  (mesh ,prims0,bvh->scene->progressInterface) : 
          createPrimRefArray<Mesh,false>(scene,prims0,bvh->scene->progressInterface);

        Splitter splitter(scene);

        bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));

        settings.branchingFactor = N;
        settings.maxDepth = BVH::maxBuildDepthLeaf;

        NodeRef root = BVHBuilderBinnedFastSpatialSAH::build<NodeRef>(
          typename BVH::CreateAlloc(bvh),
          typename BVH::AlignedNode::Create2(),
          typename BVH::AlignedNode::Set2(),
          CreateLeaf<N,Primitive>(bvh,prims0.data()),
          splitter,
          bvh->scene->progressInterface,
          prims0.data(),
          numSplitPrimitives,
          pinfo,settings);

        bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());      
        bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));

	/* clear temporary data for static geometry */
	bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic();
	if (staticGeom) {
          prims0.clear();
          bvh->shrink();
        }
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims0.clear();
      }
    };

    /************************************************************************************/ 
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/


#if defined(EMBREE_GEOMETRY_LINES)
    Builder* BVH4Line4iMeshBuilderSAH     (void* bvh, LineSegments* mesh, size_t mode) { return new BVHNBuilderSAH<4,LineSegments,Line4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4Line4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,LineSegments,Line4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4Line4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMSMBlurSAH<4,LineSegments,Line4i>((BVH4*)bvh,scene ,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4MB4DLine4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<4,LineSegments,Line4i>((BVH4*)bvh,scene ,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
#if defined(__AVX__)
    Builder* BVH8Line4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,LineSegments,Line4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH8Line4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMSMBlurSAH<8,LineSegments,Line4i>((BVH8*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH8MB4DLine4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<8,LineSegments,Line4i>((BVH8*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
#endif
#endif

#if defined(EMBREE_GEOMETRY_HAIR)
    Builder* BVH4Bezier1vSceneBuilderSAH   (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,NativeCurves,Bezier1v>((BVH4*)bvh,scene,1,1.0f,1,inf,mode,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4Bezier1iSceneBuilderSAH   (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,NativeCurves,Bezier1i>((BVH4*)bvh,scene,1,1.0f,1,inf,mode,HIGH_SINGLE_THREAD_THRESHOLD); }
#endif

#if defined(EMBREE_GEOMETRY_TRIANGLES)
    Builder* BVH4Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4v>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    
    Builder* BVH4Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode,DEFAULT_SINGLE_THREAD_THRESHOLD,true); }

    Builder* BVH4Triangle4vMBSceneBuilderSAH (void* bvh, Scene* scene,       size_t mode) { return new BVHNBuilderMSMBlurSAH<4,TriangleMesh,Triangle4vMB>((BVH4*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4Triangle4iMBSceneBuilderSAH (void* bvh, Scene* scene,       size_t mode) { return new BVHNBuilderMSMBlurSAH<4,TriangleMesh,Triangle4iMB>((BVH4*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4MB4DTriangle4iMBSceneBuilderRootTimeSplitsSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurRootTimeSplitsSAH<4,TriangleMesh,Triangle4iMB>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4MB4DTriangle4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<4,TriangleMesh,Triangle4iMB>((BVH4*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4MB4DTriangle4vMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<4,TriangleMesh,Triangle4vMB>((BVH4*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }

    Builder* BVH4Triangle4SceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<4,TriangleMesh,Triangle4,TriangleSplitterFactory>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vSceneBuilderFastSpatialSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<4,TriangleMesh,Triangle4v,TriangleSplitterFactory>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderFastSpatialSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<4,TriangleMesh,Triangle4i,TriangleSplitterFactory>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }


    Builder* BVH4QuantizedTriangle4iSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
#if defined(__AVX__)
    Builder* BVH8Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4vMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4v>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4iMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4i>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode); }

    Builder* BVH8Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4vSceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4v>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode,DEFAULT_SINGLE_THREAD_THRESHOLD,true); }
    Builder* BVH8Triangle4vMBSceneBuilderSAH (void* bvh, Scene* scene,       size_t mode) { return new BVHNBuilderMSMBlurSAH<8,TriangleMesh,Triangle4vMB>((BVH8*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH8Triangle4iMBSceneBuilderSAH (void* bvh, Scene* scene,       size_t mode) { return new BVHNBuilderMSMBlurSAH<8,TriangleMesh,Triangle4iMB>((BVH8*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH8MB4DTriangle4iMBSceneBuilderRootTimeSplitsSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurRootTimeSplitsSAH<8,TriangleMesh,Triangle4iMB>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8MB4DTriangle4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<8,TriangleMesh,Triangle4iMB>((BVH8*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH8MB4DTriangle4vMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<8,TriangleMesh,Triangle4vMB>((BVH8*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH8QuantizedTriangle4iSceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<8,TriangleMesh,Triangle4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4SceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<8,TriangleMesh,Triangle4,TriangleSplitterFactory>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4vSceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<8,TriangleMesh,Triangle4v,TriangleSplitterFactory>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }

#endif
#endif

#if defined(EMBREE_GEOMETRY_QUADS)
    Builder* BVH4Quad4vMeshBuilderSAH     (void* bvh, QuadMesh* mesh, size_t mode)     { return new BVHNBuilderSAH<4,QuadMesh,Quad4v>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4iMeshBuilderSAH     (void* bvh, QuadMesh* mesh, size_t mode)     { return new BVHNBuilderSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,QuadMesh,Quad4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode,DEFAULT_SINGLE_THREAD_THRESHOLD,true); }
    Builder* BVH4Quad4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMSMBlurSAH<4,QuadMesh,Quad4iMB>((BVH4*)bvh,scene ,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4MB4DQuad4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<4,QuadMesh,Quad4iMB>((BVH4*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH4QuantizedQuad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<4,QuadMesh,Quad4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4QuantizedQuad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4vSceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<4,QuadMesh,Quad4v,QuadSplitterFactory>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }

#if defined(__AVX__)
    Builder* BVH8Quad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,QuadMesh,Quad4v>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Quad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,QuadMesh,Quad4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode,DEFAULT_SINGLE_THREAD_THRESHOLD,true); }
    Builder* BVH8Quad4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMSMBlurSAH<8,QuadMesh,Quad4iMB>((BVH8*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH8MB4DQuad4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<8,QuadMesh,Quad4iMB>((BVH8*)bvh,scene,4,1.0f,4,inf,HIGH_SINGLE_THREAD_THRESHOLD); }
    Builder* BVH8QuantizedQuad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<8,QuadMesh,Quad4v>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8QuantizedQuad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<8,QuadMesh,Quad4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Quad4vMeshBuilderSAH     (void* bvh, QuadMesh* mesh, size_t mode)     { return new BVHNBuilderSAH<8,QuadMesh,Quad4v>((BVH8*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH8Quad4vSceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<8,QuadMesh,Quad4v,QuadSplitterFactory>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }

#endif
#endif

#if defined(EMBREE_GEOMETRY_USER)

    Builder* BVH4VirtualSceneBuilderSAH    (void* bvh, Scene* scene, size_t mode) {
      int minLeafSize = scene->device->object_accel_min_leaf_size;
      int maxLeafSize = scene->device->object_accel_max_leaf_size;
      return new BVHNBuilderSAH<4,AccelSet,Object>((BVH4*)bvh,scene,4,1.0f,minLeafSize,maxLeafSize,mode);
    }

    Builder* BVH4VirtualMeshBuilderSAH    (void* bvh, AccelSet* mesh, size_t mode) {
      return new BVHNBuilderSAH<4,AccelSet,Object>((BVH4*)bvh,mesh,4,1.0f,1,inf,mode);
    }

    Builder* BVH4VirtualMBSceneBuilderSAH    (void* bvh, Scene* scene, size_t mode) {
      int minLeafSize = scene->device->object_accel_mb_min_leaf_size;
      int maxLeafSize = scene->device->object_accel_mb_max_leaf_size;
      return new BVHNBuilderMSMBlurSAH<4,AccelSet,Object>((BVH4*)bvh,scene,4,1.0f,minLeafSize,maxLeafSize);
    }

    Builder* BVH4MB4DVirtualMBSceneBuilderSAH    (void* bvh, Scene* scene, size_t mode) {
      int minLeafSize = scene->device->object_accel_mb_min_leaf_size;
      int maxLeafSize = scene->device->object_accel_mb_max_leaf_size;
      return new BVHNBuilderMBlurSAH<4,AccelSet,Object>((BVH4*)bvh,scene,4,1.0f,minLeafSize,maxLeafSize);
    }
#endif
  }
}
