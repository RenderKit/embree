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

#include "bvh.h"
#include "bvh_builder.h"
#include "../builders/bvh_builder_msmblur.h"

#include "../builders/primrefgen.h"
#include "../builders/splitter.h"

#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/object.h"
#include "../geometry/subgrid.h"

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

    template<int N, typename Primitive>
    struct CreateLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      __forceinline CreateLeaf (BVH* bvh) : bvh(bvh) {}

      __forceinline NodeRef operator() (const PrimRef* prims, const range<size_t>& set, const FastAllocator::CachedAllocator& alloc) const
      {
        size_t n = set.size();
        size_t items = Primitive::blocks(n);
        size_t start = set.begin();
        Primitive* accel = (Primitive*) alloc.malloc1(items*sizeof(Primitive),BVH::byteAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,set.end(),bvh->scene);
        }
        return node;
      }

      BVH* bvh;
    };


    template<int N, typename Primitive>
    struct CreateLeafQuantized
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      __forceinline CreateLeafQuantized (BVH* bvh) : bvh(bvh) {}

      __forceinline NodeRef operator() (const PrimRef* prims, const range<size_t>& set, const FastAllocator::CachedAllocator& alloc) const
      {
        size_t n = set.size();
        size_t items = Primitive::blocks(n);
        size_t start = set.begin();
        Primitive* accel = (Primitive*) alloc.malloc1(items*sizeof(Primitive),BVH::byteAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,items);
        for (size_t i=0; i<items; i++) {
          accel[i].fill(prims,start,set.end(),bvh->scene);
        }
        return node;
      }

      BVH* bvh;
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
                      const size_t mode, bool primrefarrayalloc = false)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device,0),
          settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(primrefarrayalloc) {}

      BVHNBuilderSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device,0), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(false) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build()
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
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

            /* enable os_malloc for two level build */
            if (mesh)
              bvh->alloc.setOSallocation(true);

            /* initialize allocator */
            const size_t node_bytes = numPrimitives*sizeof(typename BVH::AlignedNodeMB)/(4*N);
            const size_t leaf_bytes = size_t(1.2*Primitive::blocks(numPrimitives)*sizeof(Primitive));
            bvh->alloc.init_estimate(node_bytes+leaf_bytes);
            settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,numPrimitives,node_bytes+leaf_bytes);
            prims.resize(numPrimitives); 

            PrimInfo pinfo = mesh ?
              createPrimRefArray(mesh,prims,bvh->scene->progressInterface) :
              createPrimRefArray(scene,Mesh::geom_type,false,prims,bvh->scene->progressInterface);

            /* pinfo might has zero size due to invalid geometry */
            if (unlikely(pinfo.size() == 0))
            {
              bvh->clear();
              prims.clear();
              return;
            }

            /* call BVH builder */
            NodeRef root = BVHNBuilderVirtual<N>::build(&bvh->alloc,CreateLeaf<N,Primitive>(bvh),bvh->scene->progressInterface,prims.data(),pinfo,settings);
            bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
            bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));

#if PROFILE
          });
#endif

        /* if we allocated using the primrefarray we have to keep it alive */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.share(prims);

        /* for static geometries we can do some cleanups */
        else if (scene && scene->isStaticAccel()) {
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

      BVHNBuilderSAHQuantized (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device,0), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD) {}

      BVHNBuilderSAHQuantized (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device,0), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build()
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
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
              createPrimRefArray(mesh,prims,bvh->scene->progressInterface) :
              createPrimRefArray(scene,Mesh::geom_type,false,prims,bvh->scene->progressInterface);

            /* enable os_malloc for two level build */
            if (mesh)
              bvh->alloc.setOSallocation(true);

            /* call BVH builder */
            const size_t node_bytes = numPrimitives*sizeof(typename BVH::QuantizedNode)/(4*N);
            const size_t leaf_bytes = size_t(1.2*Primitive::blocks(numPrimitives)*sizeof(Primitive));
            bvh->alloc.init_estimate(node_bytes+leaf_bytes);
            settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,numPrimitives,node_bytes+leaf_bytes);
            NodeRef root = BVHNBuilderQuantizedVirtual<N>::build(&bvh->alloc,CreateLeafQuantized<N,Primitive>(bvh),bvh->scene->progressInterface,prims.data(),pinfo,settings);
            bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
            //bvh->layoutLargeNodes(pinfo.size()*0.005f); // FIXME: COPY LAYOUT FOR LARGE NODES !!!
#if PROFILE
          });
#endif

	/* clear temporary data for static geometry */
	if (scene && scene->isStaticAccel()) {
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
    struct CreateMBlurLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::NodeRecordMB NodeRecordMB;

      __forceinline CreateMBlurLeaf (BVH* bvh, PrimRef* prims, size_t time) : bvh(bvh), prims(prims), time(time) {}

      __forceinline NodeRecordMB operator() (const PrimRef* prims, const range<size_t>& set, const FastAllocator::CachedAllocator& alloc) const
      {
        size_t items = Primitive::blocks(set.size());
        size_t start = set.begin();
        Primitive* accel = (Primitive*) alloc.malloc1(items*sizeof(Primitive),BVH::byteAlignment);
        NodeRef node = bvh->encodeLeaf((char*)accel,items);

        LBBox3fa allBounds = empty;
        for (size_t i=0; i<items; i++)
          allBounds.extend(accel[i].fillMB(prims, start, set.end(), bvh->scene, time));

        return NodeRecordMB(node,allBounds);
      }

      BVH* bvh;
      PrimRef* prims;
      size_t time;
    };


    template<int N, typename Mesh, typename Primitive>
    struct CreateMSMBlurLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::NodeRecordMB4D NodeRecordMB4D;

      __forceinline CreateMSMBlurLeaf (BVH* bvh) : bvh(bvh) {}

      __forceinline const NodeRecordMB4D operator() (const BVHBuilderMSMBlur::BuildRecord& current, const FastAllocator::CachedAllocator& alloc) const
      {
        size_t items = Primitive::blocks(current.prims.object_range.size());
        size_t start = current.prims.object_range.begin();
        Primitive* accel = (Primitive*) alloc.malloc1(items*sizeof(Primitive),BVH::byteNodeAlignment);
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

      BVHNBuilderMBlurSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)) {}

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

        if (numTimeSegments == 1)
          buildSingleSegment(numPrimitives);
        else
          buildMultiSegment(numPrimitives);

#if PROFILE
          });
#endif

	/* clear temporary data for static geometry */
        if (scene->isStaticAccel()) bvh->shrink();
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void buildSingleSegment(size_t numPrimitives)
      {
        /* create primref array */
        mvector<PrimRef> prims(scene->device,numPrimitives);
        const PrimInfo pinfo = createPrimRefArrayMBlur(scene,Mesh::geom_type,prims,bvh->scene->progressInterface,0);

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
        settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,pinfo.size(),node_bytes+leaf_bytes);

        /* build hierarchy */
        auto root = BVHBuilderBinnedSAH::build<NodeRecordMB>
          (typename BVH::CreateAlloc(bvh),typename BVH::AlignedNodeMB::Create2(),typename BVH::AlignedNodeMB::Set2(),
           CreateMBlurLeaf<N,Primitive>(bvh,prims.data(),0),bvh->scene->progressInterface,
           prims.data(),pinfo,settings);

        bvh->set(root.ref,root.lbounds,pinfo.size());
      }

      void buildMultiSegment(size_t numPrimitives)
      {
        /* create primref array */
        mvector<PrimRefMB> prims(scene->device,numPrimitives);
        PrimInfoMB pinfo = createPrimRefArrayMSMBlur(scene,Mesh::geom_type,prims,bvh->scene->progressInterface);

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
        settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,pinfo.size(),node_bytes+leaf_bytes);
        
        /* build hierarchy */
        auto root =
          BVHBuilderMSMBlur::build<NodeRef>(prims,pinfo,scene->device,
                                             RecalculatePrimRef<Mesh>(scene),
                                             typename BVH::CreateAlloc(bvh),
                                             typename BVH::AlignedNodeMB4D::Create(),
                                             typename BVH::AlignedNodeMB4D::Set(),
                                             CreateMSMBlurLeaf<N,Mesh,Primitive>(bvh),
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

      BVHNBuilderFastSpatialSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(scene), mesh(nullptr), prims0(scene->device,0), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD),
          splitFactor(scene->device->max_spatial_split_replications) {}

      BVHNBuilderFastSpatialSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims0(bvh->device,0), settings(sahBlockSize, minLeafSize, min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks), travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD),
          splitFactor(scene->device->max_spatial_split_replications) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build()
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
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
          createPrimRefArray(mesh,prims0,bvh->scene->progressInterface) :
          createPrimRefArray(scene,Mesh::geom_type,false,prims0,bvh->scene->progressInterface);

        Splitter splitter(scene);

        /* enable os_malloc for two level build */
        if (mesh)
          bvh->alloc.setOSallocation(true);

        const size_t node_bytes = pinfo.size()*sizeof(typename BVH::AlignedNode)/(4*N);
        const size_t leaf_bytes = size_t(1.2*Primitive::blocks(pinfo.size())*sizeof(Primitive));
        bvh->alloc.init_estimate(node_bytes+leaf_bytes);
        settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,pinfo.size(),node_bytes+leaf_bytes);

        settings.branchingFactor = N;
        settings.maxDepth = BVH::maxBuildDepthLeaf;

        NodeRef root = BVHBuilderBinnedFastSpatialSAH::build<NodeRef>(
          typename BVH::CreateAlloc(bvh),
          typename BVH::AlignedNode::Create2(),
          typename BVH::AlignedNode::Set2(),
          CreateLeaf<N,Primitive>(bvh),
          splitter,
          bvh->scene->progressInterface,
          prims0.data(),
          numSplitPrimitives,
          pinfo,settings);

        bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
        bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));

	/* clear temporary data for static geometry */
	if (scene && scene->isStaticAccel()) {
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

    struct SubGridBuildData {
      unsigned short sx,sy;
      unsigned int primID;

      __forceinline SubGridBuildData() {};
      __forceinline SubGridBuildData(const unsigned int sx, const unsigned int sy, const unsigned int primID) : sx(sx), sy(sy), primID(primID) {};

    };

    template<int N, typename Primitive>
    struct CreateLeafGrid
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

      __forceinline CreateLeafGrid (BVH* bvh, const SubGridBuildData * const sgrids) : bvh(bvh),sgrids(sgrids) {}

      __forceinline NodeRef operator() (const PrimRef* prims, const range<size_t>& set, const FastAllocator::CachedAllocator& alloc) const
      {
        const size_t items = set.size(); //Primitive::blocks(n);
        const size_t start = set.begin();

        /* collect all subsets with unique geomIDs */
        assert(items <= N);
        unsigned int geomIDs[N];
        unsigned int num_geomIDs = 1;
        geomIDs[0] = prims[start].geomID();

        for (size_t i=1;i<items;i++)
        {
          bool found = false;
          const unsigned int new_geomID = prims[start+i].geomID();
          for (size_t j=0;j<num_geomIDs;j++)
            if (new_geomID == geomIDs[j])
            { found = true; break; }
          if (!found) 
            geomIDs[num_geomIDs++] = new_geomID;
        }

        /* allocate all leaf memory in one single block */
        SubGridQBVHN<N>* accel = (SubGridQBVHN<N>*) alloc.malloc1(num_geomIDs*sizeof(SubGridQBVHN<N>),BVH::byteAlignment);
        typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,num_geomIDs);

        for (size_t g=0;g<num_geomIDs;g++)
        {
          unsigned int x[N];
          unsigned int y[N];
          unsigned int primID[N];
          BBox3fa bounds[N];
          unsigned int pos = 0;
          for (size_t i=0;i<items;i++)
          {
            if (unlikely(prims[start+i].geomID() != geomIDs[g])) continue;

            const SubGridBuildData  &sgrid_bd = sgrids[prims[start+i].primID()];                      
            x[pos] = sgrid_bd.sx;
            y[pos] = sgrid_bd.sy;
            primID[pos] = sgrid_bd.primID;
            bounds[pos] = prims[start+i].bounds();
            pos++;
          }
          new (&accel[g]) SubGridQBVHN<N>(x,y,primID,bounds,geomIDs[g],pos);
        }

        return node;
      }

      BVH* bvh;
      const SubGridBuildData * const sgrids;
    };


    template<int N>
    struct BVHNBuilderSAHGrid : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;
      
      BVH* bvh;
      Scene* scene;
      GridMesh* mesh;
      mvector<PrimRef> prims;
      mvector<SubGridBuildData> sgrids;
      GeneralBVHBuilder::Settings settings;
      bool primrefarrayalloc;

      BVHNBuilderSAHGrid (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode, bool primrefarrayalloc = false)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device,0), sgrids(scene->device,0), settings(sahBlockSize, minLeafSize, maxLeafSize, travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(primrefarrayalloc) {}

      BVHNBuilderSAHGrid (BVH* bvh, GridMesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device,0), sgrids(scene->device,0), settings(sahBlockSize, minLeafSize, maxLeafSize, travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(false) {}

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

      void build()
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
        }

        /* if we use the primrefarray for allocations we have to take it back from the BVH */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.unshare(prims);

	/* skip build for empty scene */
        //const size_t numGrids = mesh ? mesh->size() : scene->getNumPrimitives<GridMesh,false>();
        size_t numPrimitives = 0;

        Scene::Iterator<GridMesh,false> iter(scene);
        for (size_t s=0;s<scene->size();s++)
        {
          GridMesh *gmesh = iter.at(s);
          if (gmesh == nullptr) continue;
          for (size_t i=0;i<gmesh->size();i++)
            numPrimitives += gmesh->getNumSubGrids(i);
        }

        if (numPrimitives == 0) {
          bvh->clear();
          prims.clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderSAH");

        /* create primref array */
        if (primrefarrayalloc) {
          settings.primrefarrayalloc = numPrimitives/1000;
          if (settings.primrefarrayalloc < 1000)
            settings.primrefarrayalloc = inf;
        }

        /* enable os_malloc for two level build */
        if (mesh)
          bvh->alloc.setOSallocation(true);

        /* initialize allocator */
        const size_t node_bytes = numPrimitives*sizeof(typename BVH::AlignedNodeMB)/(4*N);
        const size_t leaf_bytes = size_t(1.2*(float)numPrimitives/N * sizeof(SubGridQBVHN<N>));
        //PRINT(node_bytes);
        //PRINT(leaf_bytes);
        //PRINT(sizeof(SubGridQBVHN<N>));

        bvh->alloc.init_estimate(node_bytes+leaf_bytes);
        settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,numPrimitives,node_bytes+leaf_bytes);
        prims.resize(numPrimitives); 
        sgrids.resize(numPrimitives); 

        // TODO: replace this with proper PrimRefArray generation
        //PrimInfo pinfo = mesh ?
        //  createPrimRefArray(mesh,prims,bvh->scene->progressInterface) :
        //  createPrimRefArray(scene,GridMesh::geom_type,false,prims,bvh->scene->progressInterface);

        PrimInfo pinfo(empty);
        size_t p_index = 0;
        for (size_t s=0;s<scene->size();s++)
        {
          GridMesh *gmesh = iter.at(s);
          if (gmesh == nullptr) continue;
          for (size_t i=0;i<gmesh->size();i++)
          {
            const GridMesh::Grid &g = gmesh->grid(i);
            for (size_t y=0;y<(size_t)g.resY-1;y+=2)
              for (size_t x=0;x<(size_t)g.resX-1;x+=2)
              {
                BBox3fa bounds = empty;
                if (!gmesh->buildBounds(g,x,y,&bounds)) continue; // get bounds of subgrid
                const PrimRef prim(bounds,s,p_index);
                pinfo.add_center2(prim);
                sgrids[p_index] = SubGridBuildData(x | g.get3x3FlagsX(x), y | g.get3x3FlagsY(y), i);
                prims[p_index++] = prim;                
              }
          }
        }

        /* pinfo might has zero size due to invalid geometry */
        if (unlikely(pinfo.size() == 0))
        {
          bvh->clear();
          sgrids.clear();
          prims.clear();
          return;
        }

        assert(p_index == numPrimitives);

        /* call BVH builder */
        NodeRef root = BVHNBuilderVirtual<N>::build(&bvh->alloc,CreateLeafGrid<N,SubGridQBVHN<N>>(bvh,sgrids.data()),bvh->scene->progressInterface,prims.data(),pinfo,settings);
        bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
        bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));


        /* if we allocated using the primrefarray we have to keep it alive */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.share(prims);

        /* for static geometries we can do some cleanups */
        else if (scene && scene->isStaticAccel()) {
          bvh->shrink();
          prims.clear();
        }
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        sgrids.clear();
        prims.clear();
      }
    };

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/

#if defined(EMBREE_GEOMETRY_CURVES)
    Builder* BVH4Line4iMeshBuilderSAH     (void* bvh, LineSegments* mesh, size_t mode) { return new BVHNBuilderSAH<4,LineSegments,Line4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Line4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,LineSegments,Line4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode,true); }
    Builder* BVH4Line4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<4,LineSegments,Line4i>((BVH4*)bvh,scene ,4,1.0f,4,inf); }
#if defined(__AVX__)
    Builder* BVH8Line4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,LineSegments,Line4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode,true); }
    Builder* BVH8Line4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<8,LineSegments,Line4i>((BVH8*)bvh,scene,4,1.0f,4,inf); }
#endif
#endif

#if defined(EMBREE_GEOMETRY_TRIANGLES)
    Builder* BVH4Triangle4MeshBuilderSAH  (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4v>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iMeshBuilderSAH (void* bvh, TriangleMesh* mesh, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }

    Builder* BVH4Triangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4vSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Triangle4iSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode,true); }

    Builder* BVH4Triangle4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf); }
    Builder* BVH4Triangle4vMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<4,TriangleMesh,Triangle4vMB>((BVH4*)bvh,scene,4,1.0f,4,inf); }

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
    Builder* BVH8Triangle4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,TriangleMesh,Triangle4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode,true); }
    Builder* BVH8Triangle4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<8,TriangleMesh,Triangle4i>((BVH8*)bvh,scene,4,1.0f,4,inf); }
    Builder* BVH8Triangle4vMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<8,TriangleMesh,Triangle4vMB>((BVH8*)bvh,scene,4,1.0f,4,inf); }

    Builder* BVH8QuantizedTriangle4iSceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<8,TriangleMesh,Triangle4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8QuantizedTriangle4SceneBuilderSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<8,TriangleMesh,Triangle4>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }

    Builder* BVH8Triangle4SceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<8,TriangleMesh,Triangle4,TriangleSplitterFactory>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Triangle4vSceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<8,TriangleMesh,Triangle4v,TriangleSplitterFactory>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }

#endif
#endif

#if defined(EMBREE_GEOMETRY_QUADS)
    Builder* BVH4Quad4vMeshBuilderSAH     (void* bvh, QuadMesh* mesh, size_t mode)     { return new BVHNBuilderSAH<4,QuadMesh,Quad4v>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4iMeshBuilderSAH     (void* bvh, QuadMesh* mesh, size_t mode)     { return new BVHNBuilderSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,mesh,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,QuadMesh,Quad4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode,true); }
    Builder* BVH4Quad4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf); }
    Builder* BVH4QuantizedQuad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<4,QuadMesh,Quad4v>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4QuantizedQuad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAHQuantized<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH4Quad4vSceneBuilderFastSpatialSAH  (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderFastSpatialSAH<4,QuadMesh,Quad4v,QuadSplitterFactory>((BVH4*)bvh,scene,4,1.0f,4,inf,mode); }

#if defined(__AVX__)
    Builder* BVH8Quad4vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,QuadMesh,Quad4v>((BVH8*)bvh,scene,4,1.0f,4,inf,mode); }
    Builder* BVH8Quad4iSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderSAH<8,QuadMesh,Quad4i>((BVH8*)bvh,scene,4,1.0f,4,inf,mode,true); }
    Builder* BVH8Quad4iMBSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHNBuilderMBlurSAH<8,QuadMesh,Quad4i>((BVH8*)bvh,scene,4,1.0f,4,inf); }
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
      return new BVHNBuilderMBlurSAH<4,AccelSet,Object>((BVH4*)bvh,scene,4,1.0f,minLeafSize,maxLeafSize);
    }

#if defined(__AVX__)

    Builder* BVH8VirtualSceneBuilderSAH    (void* bvh, Scene* scene, size_t mode) {
      int minLeafSize = scene->device->object_accel_min_leaf_size;
      int maxLeafSize = scene->device->object_accel_max_leaf_size;
      return new BVHNBuilderSAH<8,AccelSet,Object>((BVH8*)bvh,scene,8,1.0f,minLeafSize,maxLeafSize,mode);
    }

    Builder* BVH8VirtualMeshBuilderSAH    (void* bvh, AccelSet* mesh, size_t mode) {
      return new BVHNBuilderSAH<8,AccelSet,Object>((BVH8*)bvh,mesh,8,1.0f,1,inf,mode);
    }

    Builder* BVH8VirtualMBSceneBuilderSAH    (void* bvh, Scene* scene, size_t mode) {
      int minLeafSize = scene->device->object_accel_mb_min_leaf_size;
      int maxLeafSize = scene->device->object_accel_mb_max_leaf_size;
      return new BVHNBuilderMBlurSAH<8,AccelSet,Object>((BVH8*)bvh,scene,8,1.0f,minLeafSize,maxLeafSize);
    }

#endif
#endif


#if defined(EMBREE_GEOMETRY_GRID)
    Builder* BVH4GridMeshBuilderSAH  (void* bvh, GridMesh* mesh, size_t mode) { return new BVHNBuilderSAHGrid<4>((BVH4*)bvh,mesh,4,1.0f,1,4,mode); }
    Builder* BVH4GridSceneBuilderSAH (void* bvh, Scene* scene, size_t mode)   { return new BVHNBuilderSAHGrid<4>((BVH4*)bvh,scene,4,1.0f,1,4,mode); } // FIXME: check whether cost factors are correct

#if defined(__AVX__)
    Builder* BVH8GridMeshBuilderSAH  (void* bvh, GridMesh* mesh, size_t mode) { return new BVHNBuilderSAHGrid<8>((BVH8*)bvh,mesh,8,1.0f,4,8,mode); }
    Builder* BVH8GridSceneBuilderSAH (void* bvh, Scene* scene, size_t mode)   { return new BVHNBuilderSAHGrid<8>((BVH8*)bvh,scene,8,1.0f,4,8,mode); } // FIXME: check whether cost factors are correct
#endif
#endif

  }
}
