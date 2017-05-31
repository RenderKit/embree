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

#pragma once

#include "../bvh/bvh.h"
#include "../geometry/primitive.h"
#include "../builders/bvh_builder_msmblur.h"
#include "../builders/heuristic_binning_array_aligned.h"
#include "../builders/heuristic_binning_array_unaligned.h"
#include "../builders/heuristic_timesplit_array.h"
#include "bvh_builder_hair.h"

namespace embree
{
  namespace isa
  {
    struct BVHBuilderHairMSMBlur
    {
      typedef CommonBuildSettings Settings;

      struct BuildRecord
      {
      public:
	__forceinline BuildRecord () {}

        __forceinline BuildRecord (size_t depth)
          : depth(depth) {}

        __forceinline BuildRecord (const SetMB& prims, size_t depth)
          : depth(depth), prims(prims) {}

        __forceinline size_t size() const {
          return prims.object_range.size();
        }

      public:
	size_t depth;       //!< depth of the root of this subtree
	SetMB prims;        //!< the list of primitives
      };

      struct BuildRecordSplit : public BuildRecord
      {
        __forceinline BuildRecordSplit () {}

        __forceinline BuildRecordSplit (size_t depth) 
          : BuildRecord(depth) {}

        __forceinline BuildRecordSplit (const BuildRecord& record, const BinSplit<MBLUR_NUM_OBJECT_BINS>& split)
          : BuildRecord(record), split(split) {}
        
        BinSplit<MBLUR_NUM_OBJECT_BINS> split;
      };

      template<typename NodeRef,
        typename RecalculatePrimRef,
        typename CreateAllocFunc,
        typename CreateAlignedNodeFunc,
        typename SetAlignedNodeFunc,
        typename CreateAlignedNodeMBFunc,
        typename SetAlignedNodeMBFunc,
        typename CreateUnalignedNodeFunc,
        typename SetUnalignedNodeFunc,
        typename CreateUnalignedNodeMBFunc,
        typename SetUnalignedNodeMBFunc,
        typename CreateLeafFunc,
        typename CreateLeafMBFunc,
        typename ProgressMonitor>

        class BuilderT
        {
          ALIGNED_CLASS;

          static const size_t MAX_BRANCHING_FACTOR =  8;         //!< maximal supported BVH branching factor
          static const size_t MIN_LARGE_LEAF_LEVELS = 8;         //!< create balanced tree if we are that many levels before the maximal tree depth
          static const size_t SINGLE_THREADED_THRESHOLD = 4096;  //!< threshold to switch to single threaded build

          typedef BVHNodeRecordMB<NodeRef> NodeRecordMB;
          typedef BVHNodeRecordMB4D<NodeRef> NodeRecordMB4D;
          typedef BinSplit<MBLUR_NUM_OBJECT_BINS> Split;

          typedef FastAllocator::CachedAllocator Allocator;
          typedef LocalChildListT<BuildRecord,MAX_BRANCHING_FACTOR> LocalChildList;
          typedef LocalChildListT<BuildRecordSplit,MAX_BRANCHING_FACTOR> LocalChildListSplit;

          typedef HeuristicMBlurTemporalSplit<PrimRefMB,RecalculatePrimRef,MBLUR_NUM_TEMPORAL_BINS> HeuristicTemporal;
          typedef HeuristicArrayBinningMB<PrimRefMB,MBLUR_NUM_OBJECT_BINS> HeuristicBinning;
          typedef UnalignedHeuristicArrayBinningMB<PrimRefMB,MBLUR_NUM_OBJECT_BINS> UnalignedHeuristicBinning;

        public:

          BuilderT (Scene* scene,
                    const RecalculatePrimRef& recalculatePrimRef,
                    const CreateAllocFunc& createAlloc,
                    const CreateAlignedNodeFunc& createAlignedNode,
                    const SetAlignedNodeFunc& setAlignedNode,
                    const CreateAlignedNodeMBFunc& createAlignedNodeMB,
                    const SetAlignedNodeMBFunc& setAlignedNodeMB,
                    const CreateUnalignedNodeFunc& createUnalignedNode,
                    const SetUnalignedNodeFunc& setUnalignedNode,
                    const CreateUnalignedNodeMBFunc& createUnalignedNodeMB,
                    const SetUnalignedNodeMBFunc& setUnalignedNodeMB,
                    const CreateLeafFunc& createLeaf,
                    const CreateLeafMBFunc& createLeafMB,
                    const ProgressMonitor& progressMonitor,
                    const Settings settings)

            : cfg(settings),
            scene(scene),
            recalculatePrimRef(recalculatePrimRef),
            createAlloc(createAlloc),
            createAlignedNode(createAlignedNode), setAlignedNode(setAlignedNode),
            createAlignedNodeMB(createAlignedNodeMB), setAlignedNodeMB(setAlignedNodeMB),
            createUnalignedNode(createUnalignedNode), setUnalignedNode(setUnalignedNode),
            createUnalignedNodeMB(createUnalignedNodeMB), setUnalignedNodeMB(setUnalignedNodeMB),
            createLeaf(createLeaf),
            createLeafMB(createLeafMB),
            progressMonitor(progressMonitor),
            unalignedHeuristic(scene),
            temporalSplitHeuristic(scene->device,recalculatePrimRef) {}

        private:

          /*! array partitioning */
          __forceinline std::unique_ptr<mvector<PrimRefMB>> split(const Split& split, const SetMB& set, SetMB& lset, SetMB& rset)
          {
            /* perform object split */
            if (likely(split.data == Split::SPLIT_OBJECT)) {
              alignedHeuristic.split(split,set,lset,rset);
            }
            /* perform temporal split */
            else if (likely(split.data == Split::SPLIT_TEMPORAL)) {
              return temporalSplitHeuristic.split(split,set,lset,rset);
            }
            /* perform fallback split */
            else if (unlikely(split.data == Split::SPLIT_FALLBACK)) {
              set.deterministic_order();
              splitAtCenter(set,lset,rset);
            }
            /* perform type split */
            else if (unlikely(split.data == Split::SPLIT_TYPE)) {
              splitByType(set,lset,rset);
            }
            else
              assert(false);

            return std::unique_ptr<mvector<PrimRefMB>>();
          }

          /*! finds the best fallback split */
          __noinline Split findFallback(const SetMB& set)
          {
            if (set.size() == 0)
              return Split(0.0f,Split::SPLIT_NONE);

            /* if a leaf can only hold a single time-segment, we might have to do additional temporal splits */
            if (cfg.singleLeafTimeSegment)
            {
              /* test if one primitive has more than one time segment in time range, if so split time */
              for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++)
              {
                const PrimRefMB& prim = (*set.prims)[i];
                const range<int> itime_range = getTimeSegmentRange(set.time_range,(float)prim.totalTimeSegments());
                if (itime_range.size() > 1) {
                  const int icenter = (itime_range.begin() + itime_range.end())/2;
                  const float splitTime = float(icenter)/float(prim.totalTimeSegments());
                  return Split(0.0f,(unsigned)Split::SPLIT_TEMPORAL,0,splitTime);
                }
              }
            }

            /* if primitives are of different type perform type splits */
            if (!set.oneType())
              return Split(0.0f,Split::SPLIT_TYPE);

            /* if the leaf is too large we also have to perform additional splits */
            if (set.size() > cfg.maxLeafSize)
              return Split(0.0f,Split::SPLIT_FALLBACK);

            /* otherwise perform no splits anymore */
            return Split(0.0f,Split::SPLIT_NONE);
          }

          /*! performs fallback split */
          void splitAtCenter(const SetMB& set, SetMB& lset, SetMB& rset)
          {
            mvector<PrimRefMB>& prims = *set.prims;

            const size_t begin = set.object_range.begin();
            const size_t end   = set.object_range.end();
            const size_t center = (begin + end)/2;

            PrimInfoMB linfo = empty;
            for (size_t i=begin; i<center; i++)
              linfo.add_primref(prims[i]);

            PrimInfoMB rinfo = empty;
            for (size_t i=center; i<end; i++)
              rinfo.add_primref(prims[i]);

            new (&lset) SetMB(linfo,set.prims,range<size_t>(begin,center),set.time_range);
            new (&rset) SetMB(rinfo,set.prims,range<size_t>(center,end  ),set.time_range);
          }

          /*! split by primitive type */
          void splitByType(const SetMB& set, SetMB& lset, SetMB& rset)
          {
            assert(set.size());
            mvector<PrimRefMB>& prims = *set.prims;
            const size_t begin = set.object_range.begin();
            const size_t end   = set.object_range.end();
          
            Leaf::Type type = prims[begin].type();
            PrimInfoMB linfo = empty;
            PrimInfoMB rinfo = empty;
            size_t center = serial_partitioning(prims.data(),begin,end,linfo,rinfo,
                                                [&] ( const PrimRefMB& prim ) { return prim.type() == type; },
                                                [ ] ( PrimInfoMB& a, const PrimRefMB& b ) { a.add_primref(b); });
            
            new (&lset) SetMB(linfo,set.prims,range<size_t>(begin,center),set.time_range);
            new (&rset) SetMB(rinfo,set.prims,range<size_t>(center,end  ),set.time_range);
          }

          const NodeRecordMB4D createLargeLeaf(const BuildRecord& in, Allocator alloc)
          {
            /* this should never occur but is a fatal error */
            if (in.depth > cfg.maxDepth)
              throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");

            /* replace already found split by fallback split */
            const BuildRecordSplit current(BuildRecord(in.prims,in.depth),findFallback(in.prims)); // FIXME: findFallback invoked too often!

            /* create leaf for few primitives */
            if (current.split.data == Split::SPLIT_NONE)
              return createLeafMB(current.prims,alloc);

            /* fill all children by always splitting the largest one */
            bool hasTimeSplits = false;
            NodeRecordMB4D values[MAX_BRANCHING_FACTOR];
            LocalChildListSplit children(current);

            do {
              /* find best child with largest bounding box area */
              size_t bestChild = -1;
              size_t bestSize = 0;
              for (size_t i=0; i<children.size(); i++)
              {
                /* ignore leaves as they cannot get split */
                if (children[i].split.data == Split::SPLIT_NONE)
                  continue;

                /* remember child with largest size */
                if (children[i].size() > bestSize) {
                  bestSize = children[i].size();
                  bestChild = i;
                }
              }
              if (bestChild == -1) break;

              /* perform best found split */
              BuildRecordSplit& brecord = children[bestChild];
              BuildRecordSplit lrecord(current.depth+1);
              BuildRecordSplit rrecord(current.depth+1);
              std::unique_ptr<mvector<PrimRefMB>> new_vector = split(brecord.split,brecord.prims,lrecord.prims,rrecord.prims);
              hasTimeSplits |= new_vector != nullptr;

              /* find new splits */
              lrecord.split = findFallback(lrecord.prims);
              rrecord.split = findFallback(rrecord.prims);
              children.split(bestChild,lrecord,rrecord,std::move(new_vector));

            } while (children.size() < cfg.branchingFactor);

            /* create node */
            NodeRef node = createAlignedNodeMB(alloc, hasTimeSplits);

            /* recurse into each child and perform reduction */
            LBBox3fa gbounds = empty;
            for (size_t i=0; i<children.size(); i++) {
              values[i] = createLargeLeaf(children[i],alloc);
              gbounds.extend(values[i].lbounds);
              setAlignedNodeMB(node,i,values[i]);
            }

            /* calculate geometry bounds of this node */
            if (hasTimeSplits)
              return NodeRecordMB4D(node,current.prims.linearBounds(recalculatePrimRef),current.prims.time_range);
            else
              return NodeRecordMB4D(node,gbounds,current.prims.time_range);
          }

          /*! performs split */
          std::unique_ptr<mvector<PrimRefMB>> split(const BuildRecord& current, BuildRecord& lrecord, BuildRecord& rrecord, bool& aligned, bool& timesplit)
          {
            /* variable to track the SAH of the best splitting approach */
            float bestSAH = inf;
            const float leafSAH = current.prims.leafSAH();

            /* perform standard binning in aligned space */
            HeuristicBinning::Split alignedObjectSplit = alignedHeuristic.find(current.prims,0);
            float alignedObjectSAH = alignedObjectSplit.splitSAH();
            bestSAH = min(alignedObjectSAH,bestSAH);

            /* perform standard binning in unaligned space */
            UnalignedHeuristicBinning::Split unalignedObjectSplit;
            LinearSpace3fa uspace;
            float unalignedObjectSAH = inf;
            if (current.prims.isType(Leaf::TY_HAIR,Leaf::TY_HAIR_MB) && alignedObjectSAH > 0.7f*leafSAH) {
              uspace = unalignedHeuristic.computeAlignedSpaceMB(scene,current.prims);
              const SetMB sset = current.prims.primInfo(recalculatePrimRef,uspace);
              unalignedObjectSplit = unalignedHeuristic.find(sset,0,uspace);
              unalignedObjectSAH = 1.3f*unalignedObjectSplit.splitSAH(); // makes unaligned splits more expensive
              bestSAH = min(unalignedObjectSAH,bestSAH);
            }

            /* do temporal splits only if previous approaches failed to produce good SAH and the the time range is large enough */
            float temporal_split_sah = inf;
            typename HeuristicTemporal::Split temporal_split;
            if (bestSAH > 0.5f*leafSAH) {
              if (current.prims.time_range.size() > 1.01f/float(current.prims.max_num_time_segments)) {
                temporal_split = temporalSplitHeuristic.find(current.prims, size_t(0));
                temporal_split_sah = temporal_split.splitSAH();
                bestSAH = min(temporal_split_sah,bestSAH);
              }
            }

            /* perform fallback split if SAH heuristics failed */
            if (unlikely(!std::isfinite(bestSAH))) {
              current.prims.deterministic_order();
              splitAtCenter(current.prims,lrecord.prims,rrecord.prims);
            }
            /* perform aligned split if this is best */
            else if (likely(bestSAH == alignedObjectSAH)) {
              alignedHeuristic.split(alignedObjectSplit,current.prims,lrecord.prims,rrecord.prims);
            }
            /* perform unaligned split if this is best */
            else if (likely(bestSAH == unalignedObjectSAH)) {
              unalignedHeuristic.split(unalignedObjectSplit,uspace,current.prims,lrecord.prims,rrecord.prims);
              aligned = false;
            }
            /* perform temporal split if this is best */
            else if (likely(bestSAH == temporal_split_sah)) {
              timesplit = true;
              return temporalSplitHeuristic.split(temporal_split,current.prims,lrecord.prims,rrecord.prims);
            }
            else
              assert(false);

            return std::unique_ptr<mvector<PrimRefMB>>();
          }

          /*! recursive build */
          NodeRecordMB4D recurse(BuildRecord& current, Allocator alloc, bool toplevel)
          {
            if (current.prims.isType(Leaf::TY_HAIR))
            {
              PrimRefMB* primsMB = current.prims.prims->data()+current.prims.object_range.begin();
              PrimRef* prims = (PrimRef*) primsMB;
              convert_PrimRefMBArray_To_PrimRefArray(primsMB,prims,current.prims.object_range.size());
              NodeRef ref = BVHBuilderHair::build<NodeRef>
                (createAlloc,
                 createAlignedNode,
                 setAlignedNode,
                 createUnalignedNode,
                 setUnalignedNode,
                 createLeaf,
                 progressMonitor,
                 scene, prims, PrimInfo(0,current.prims.object_range.size(),current.prims),
                 cfg);
              convert_PrimRefArray_To_PrimRefMBArray(prims,primsMB,current.prims.object_range.size());
              return NodeRecordMB4D(ref,LBBox3fa(current.prims.geomBounds),current.prims.time_range);
            }

            /* get thread local allocator */
            if (!alloc)
              alloc = createAlloc();

            /* call memory monitor function to signal progress */
            if (toplevel && current.size() <= SINGLE_THREADED_THRESHOLD)
              progressMonitor(current.size());

            /* create leaf node */
            if (current.depth+MIN_LARGE_LEAF_LEVELS >= cfg.maxDepth || current.size() <= cfg.minLeafSize) {
              current.prims.deterministic_order();
              return createLargeLeaf(current,alloc);
            }

            /* fill all children by always splitting the one with the largest surface area */
            LocalChildList children(current);
            bool aligned = true;
            bool timesplit = false;

            do {

              /* find best child with largest bounding box area */
              ssize_t bestChild = -1;
              float bestArea = neg_inf;
              for (size_t i=0; i<children.size(); i++)
              {
                /* ignore leaves as they cannot get split */
                if (children[i].size() <= cfg.minLeafSize)
                  continue;

                /* remember child with largest area */
                const float A = children[i].prims.halfArea();
                if (A > bestArea) {
                  bestArea = children[i].prims.halfArea();
                  bestChild = i;
                }
              }
              if (bestChild == -1) break;

              /*! split best child into left and right child */
              BuildRecord left(current.depth+1);
              BuildRecord right(current.depth+1);
              std::unique_ptr<mvector<PrimRefMB>> new_vector = split(children[bestChild],left,right,aligned,timesplit);
              children.split(bestChild,left,right,std::move(new_vector));

            } while (children.size() < cfg.branchingFactor);

            /* create time split node */
            if (timesplit)
            {
              const NodeRef node = createAlignedNodeMB(alloc,true);

              /* spawn tasks or ... */
              if (current.size() > SINGLE_THREADED_THRESHOLD)
              {
                parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                    for (size_t i=r.begin(); i<r.end(); i++) {
                      const auto child = recurse(children[i],nullptr,true);
                      setAlignedNodeMB(node,i,child);
                      _mm_mfence(); // to allow non-temporal stores during build
                    }
                  });
              }
              /* ... continue sequential */
              else {
                for (size_t i=0; i<children.size(); i++) {
                  const auto child = recurse(children[i],alloc,false);
                  setAlignedNodeMB(node,i,child);
                }
              }

              const LBBox3fa bounds = current.prims.linearBounds(recalculatePrimRef);
              return NodeRecordMB4D(node,bounds,current.prims.time_range);
            }

            /* create aligned node */
            else if (aligned)
            {
              const NodeRef node = createAlignedNodeMB(alloc,false);

              /* spawn tasks or ... */
              if (current.size() > SINGLE_THREADED_THRESHOLD)
              {
                LBBox3fa cbounds[MAX_BRANCHING_FACTOR];
                parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                    for (size_t i=r.begin(); i<r.end(); i++) {
                      const auto child = recurse(children[i],nullptr,true);
                      setAlignedNodeMB(node,i,child);
                      cbounds[i] = child.lbounds;
                      _mm_mfence(); // to allow non-temporal stores during build
                    }
                  });

                LBBox3fa bounds = empty;
                for (size_t i=0; i<children.size(); i++)
                  bounds.extend(cbounds[i]);

                return NodeRecordMB4D(node,bounds,current.prims.time_range);
              }
              /* ... continue sequentially */
              else
              {
                LBBox3fa bounds = empty;
                for (size_t i=0; i<children.size(); i++) {
                  const auto child = recurse(children[i],alloc,false);
                  setAlignedNodeMB(node,i,child);
                  bounds.extend(child.lbounds);
                }
                return NodeRecordMB4D(node,bounds,current.prims.time_range);
              }
            }

            /* create unaligned node */
            else
            {
              const NodeRef node = createUnalignedNodeMB(alloc);

              /* spawn tasks or ... */
              if (current.size() > SINGLE_THREADED_THRESHOLD)
              {
                parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                    for (size_t i=r.begin(); i<r.end(); i++) {
                      const auto child = recurse(children[i],nullptr,true);
                      if (children[i].prims.isType(Leaf::TY_HAIR_MB)) {
                        const LinearSpace3fa space = unalignedHeuristic.computeAlignedSpaceMB(scene,children[i].prims);
                        const LBBox3fa lbounds = children[i].prims.linearBounds(recalculatePrimRef,space);
                        setUnalignedNodeMB(node,i,child.ref,space,lbounds,children[i].prims.time_range);
                      } else {
                        setUnalignedNodeMB(node,i,child.ref,one,child.lbounds,children[i].prims.time_range);
                      }
                        _mm_mfence(); // to allow non-temporal stores during build
                    }
                  });
              }
              /* ... continue sequentially */
              else
              {
                for (size_t i=0; i<children.size(); i++) {
                  const auto child = recurse(children[i],alloc,false);
                  if (children[i].prims.isType(Leaf::TY_HAIR_MB)) {
                    const LinearSpace3fa space = unalignedHeuristic.computeAlignedSpaceMB(scene,children[i].prims);
                    const LBBox3fa lbounds = children[i].prims.linearBounds(recalculatePrimRef,space);
                    setUnalignedNodeMB(node,i,child.ref,space,lbounds,children[i].prims.time_range);
                  } else {
                    setUnalignedNodeMB(node,i,child.ref,one,child.lbounds,children[i].prims.time_range);
                  }
                }
              }

              const LBBox3fa bounds = current.prims.linearBounds(recalculatePrimRef);
              return NodeRecordMB4D(node,bounds,current.prims.time_range);
            }
          }

        public:

          /*! entry point into builder */
          NodeRecordMB4D operator() (mvector<PrimRefMB>& prims, const PrimInfoMB& pinfo)
          {
            BuildRecord record(SetMB(pinfo,&prims),1);
            auto root = recurse(record,nullptr,true);
            _mm_mfence(); // to allow non-temporal stores during build
            return root;
          }

        private:
          Settings cfg;
          Scene* scene;
          const RecalculatePrimRef& recalculatePrimRef;
          const CreateAllocFunc& createAlloc;
          const CreateAlignedNodeFunc& createAlignedNode;
          const SetAlignedNodeFunc& setAlignedNode;
          const CreateAlignedNodeMBFunc& createAlignedNodeMB;
          const SetAlignedNodeMBFunc& setAlignedNodeMB;
          const CreateUnalignedNodeFunc& createUnalignedNode;
          const SetUnalignedNodeFunc& setUnalignedNode;
          const CreateUnalignedNodeMBFunc& createUnalignedNodeMB;
          const SetUnalignedNodeMBFunc& setUnalignedNodeMB;
          const CreateLeafFunc& createLeaf;
          const CreateLeafMBFunc& createLeafMB;
          const ProgressMonitor& progressMonitor;

        private:
          HeuristicBinning alignedHeuristic;
          UnalignedHeuristicBinning unalignedHeuristic;
          HeuristicTemporal temporalSplitHeuristic;
        };

      template<typename NodeRef,
        typename RecalculatePrimRef,
        typename CreateAllocFunc,
        typename CreateAlignedNodeFunc,
        typename SetAlignedNodeFunc,
        typename CreateAlignedNodeMBFunc,
        typename SetAlignedNodeMBFunc,
        typename CreateUnalignedNodeFunc,
        typename SetUnalignedNodeFunc,
        typename CreateUnalignedNodeMBFunc,
        typename SetUnalignedNodeMBFunc,
        typename CreateLeafFunc,
        typename CreateLeafMBFunc,
        typename ProgressMonitor>

        static BVHNodeRecordMB4D<NodeRef> build (Scene* scene, mvector<PrimRefMB>& prims, const PrimInfoMB& pinfo,
                                               const RecalculatePrimRef& recalculatePrimRef,
                                               const CreateAllocFunc& createAlloc,
                                               const CreateAlignedNodeFunc& createAlignedNode,
                                               const SetAlignedNodeFunc& setAlignedNode,
                                               const CreateAlignedNodeMBFunc& createAlignedNodeMB,
                                               const SetAlignedNodeMBFunc& setAlignedNodeMB,
                                               const CreateUnalignedNodeFunc& createUnalignedNode,
                                               const SetUnalignedNodeFunc& setUnalignedNode,
                                               const CreateUnalignedNodeMBFunc& createUnalignedNodeMB,
                                               const SetUnalignedNodeMBFunc& setUnalignedNodeMB,
                                               const CreateLeafFunc& createLeaf,
                                               const CreateLeafMBFunc& createLeafMB,
                                               const ProgressMonitor& progressMonitor,
                                               const Settings settings)
        {
          typedef BuilderT<NodeRef,RecalculatePrimRef,CreateAllocFunc,
            CreateAlignedNodeFunc,SetAlignedNodeFunc,
            CreateAlignedNodeMBFunc,SetAlignedNodeMBFunc,
            CreateUnalignedNodeFunc,SetUnalignedNodeFunc,
            CreateUnalignedNodeMBFunc,SetUnalignedNodeMBFunc,
            CreateLeafFunc,CreateLeafMBFunc,ProgressMonitor> Builder;

          Builder builder(scene,recalculatePrimRef,createAlloc,
                          createAlignedNode,setAlignedNode,
                          createAlignedNodeMB,setAlignedNodeMB,
                          createUnalignedNode,setUnalignedNode,
                          createUnalignedNodeMB,setUnalignedNodeMB,
                          createLeaf,createLeafMB,progressMonitor,settings);

          return builder(prims,pinfo);
        }
    };
  }
}
