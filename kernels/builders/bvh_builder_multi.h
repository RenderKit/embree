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

#define MBLUR_NUM_TEMPORAL_BINS 2
#define MBLUR_NUM_OBJECT_BINS   32

#include "../bvh/bvh.h"
#include "../common/primref_mb.h"
#include "heuristic_binning_array_aligned.h"
#include "heuristic_timesplit_array.h"

#include "bvh_builder_hair.h"
#include "bvh_builder_msmblur_hair.h"

namespace embree
{
  namespace isa
  {
    struct BVHBuilderMulti
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

        __forceinline friend bool operator> (const BuildRecord& a, const BuildRecord& b) {
          return a.prims.size() > b.prims.size();
        }

        __forceinline size_t size() const {
          return prims.size();
        }

      public:
	size_t depth;                     //!< Depth of the root of this subtree.
	SetMB prims;                      //!< The list of primitives.
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

      template<
        typename NodeRef,
        typename RecalculatePrimRef,
        typename Allocator,
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
        typename ProgressMonitor>

        class BuilderT
        {
          ALIGNED_CLASS;
          static const size_t MAX_BRANCHING_FACTOR = 8;        //!< maximal supported BVH branching factor
          static const size_t MIN_LARGE_LEAF_LEVELS = 8;        //!< create balanced tree if we are that many levels before the maximal tree depth

          typedef BVHNodeRecordMB4D<NodeRef> NodeRecordMB4D;
          typedef BinSplit<MBLUR_NUM_OBJECT_BINS> Split;
          typedef mvector<PrimRefMB>* PrimRefVector;
          typedef SharedVector<mvector<PrimRefMB>> SharedPrimRefVector;
          typedef LocalChildListT<BuildRecord,MAX_BRANCHING_FACTOR> LocalChildList;
          typedef LocalChildListT<BuildRecordSplit,MAX_BRANCHING_FACTOR> LocalChildListSplit;

        public:

          BuilderT (Scene* scene,
                    const RecalculatePrimRef recalculatePrimRef,
                    const CreateAllocFunc createAlloc,
                    const CreateAlignedNodeFunc createAlignedNode,
                    const SetAlignedNodeFunc setAlignedNode,
                    const CreateAlignedNodeMBFunc createAlignedNodeMB,
                    const SetAlignedNodeMBFunc setAlignedNodeMB,
                    const CreateUnalignedNodeFunc createUnalignedNode,
                    const SetUnalignedNodeFunc setUnalignedNode,
                    const CreateUnalignedNodeMBFunc createUnalignedNodeMB,
                    const SetUnalignedNodeMBFunc setUnalignedNodeMB,
                    const CreateLeafFunc& createLeaf,
                    const ProgressMonitor progressMonitor,
                    const Settings& default_settings,
                    const Settings* type_settings)
            : default_settings(default_settings),
            type_settings(type_settings),
            scene(scene),
            heuristicObjectSplit(),
            heuristicTemporalSplit(scene->device, recalculatePrimRef),
            recalculatePrimRef(recalculatePrimRef), createAlloc(createAlloc), 
            createAlignedNode(createAlignedNode), setAlignedNode(setAlignedNode), createAlignedNodeMB(createAlignedNodeMB), setAlignedNodeMB(setAlignedNodeMB), 
            createUnalignedNode(createUnalignedNode), setUnalignedNode(setUnalignedNode), createUnalignedNodeMB(createUnalignedNodeMB), setUnalignedNodeMB(setUnalignedNodeMB), 
            createLeaf(createLeaf),
            progressMonitor(progressMonitor)
          {
            if (default_settings.branchingFactor > MAX_BRANCHING_FACTOR)
              throw_RTCError(RTC_UNKNOWN_ERROR,"bvh_builder: branching factor too large");
          }

          const Settings& getSettings(unsigned type_mask) const 
          {
            /* if only one bit is set use that types settings */
            if (((type_mask-1) & type_mask) == 0) 
              return type_settings[Leaf::selectTy(type_mask)];
            
            /* otherwise use the default settings */
            else return default_settings;
          }

          /*! finds the best split */
          const Split find(const SetMB& set)
          {
            const Settings& cfg = getSettings(set.types);

            /* split out hair when approaching leaf level */
            if (set.hasType(Leaf::TY_HAIR) && !set.oneType() && set.size() < 200) {
              return Split(0.0f,Split::SPLIT_TYPE,Leaf::TY_HAIR);
            }

            /* split out hair when approaching leaf level */
            if (set.hasType(Leaf::TY_HAIR_MB) && !set.oneType() && set.size() < 200) {
              return Split(0.0f,Split::SPLIT_TYPE,Leaf::TY_HAIR_MB);
            }
               
            /* first try standard object split */
            const Split object_split = heuristicObjectSplit.find(set,cfg.logBlockSize);
            const float object_split_sah = object_split.splitSAH();

            /* if there is no motion blur geometry return object split */
            if (!set.hasMBlur())
              return object_split;

            /* test temporal splits only when object split was bad */
            const float leaf_sah = set.leafSAH(cfg.logBlockSize);
            if (object_split_sah < 0.50f*leaf_sah)
              return object_split;

            /* do temporal splits only if the the time range is big enough */
            if (set.time_range.size() > 1.01f/float(set.max_num_time_segments))
            {
              const Split temporal_split = heuristicTemporalSplit.find(set,cfg.logBlockSize);
              const float temporal_split_sah = temporal_split.splitSAH();

              /* take temporal split if it improved SAH */
              if (temporal_split_sah < object_split_sah)
                return temporal_split;
            }

            return object_split;
          }

          /*! array partitioning */
          __forceinline std::unique_ptr<mvector<PrimRefMB>> split(const Split& split, const SetMB& set, SetMB& lset, SetMB& rset)
          {
            /* perform object split */
            if (likely(split.data == Split::SPLIT_OBJECT)) {
              heuristicObjectSplit.split(split,set,lset,rset);
            }
            /* perform temporal split */
            else if (likely(split.data == Split::SPLIT_TEMPORAL)) {
              return heuristicTemporalSplit.split(split,set,lset,rset);
            }
            /* perform fallback split */
            else if (unlikely(split.data == Split::SPLIT_FALLBACK)) {
              set.deterministic_order();
              splitAtCenter(set,lset,rset);
            }
            /* perform type split */
            else if (unlikely(split.data == Split::SPLIT_TYPE)) {
              splitByType(set,lset,rset,(Leaf::Type)split.dim);
            }
            else
              assert(false);

            return std::unique_ptr<mvector<PrimRefMB>>();
          }

          __forceinline bool sameType(const SetMB& set)
          {
            //assert(set.object_range.size());
            if (set.object_range.size() == 0) return true;
            Leaf::Type ty0 = (*set.prims)[set.object_range.begin()].type();
            for (size_t i=set.object_range.begin()+1; i<set.object_range.end(); i++)
              if (ty0 != (*set.prims)[i].type()) 
                return false;
            
            return true;
          }

          /*! finds the best fallback split */
          __noinline Split findFallback(const SetMB& set)
          {
            const Settings& cfg = getSettings(set.types);

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
            if (!set.oneType()) {
              return Split(0.0f,Split::SPLIT_TYPE,Leaf::selectTy(set.types));
            }

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
          void splitByType(const SetMB& set, SetMB& lset, SetMB& rset, const Leaf::Type type)
          {
            mvector<PrimRefMB>& prims = *set.prims;
            const size_t begin = set.object_range.begin();
            const size_t end   = set.object_range.end();
            
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
            const Settings& cfg = getSettings(in.prims.types);
         
            /* this should never occur but is a fatal error */
            if (in.depth > cfg.maxDepth)
              throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");

            /* replace already found split by fallback split */
            const BuildRecordSplit current(BuildRecord(in.prims,in.depth),findFallback(in.prims)); // FIXME: findFallback invoked too often!

            /* create leaf for few primitives */
            if (current.split.data == Split::SPLIT_NONE)
              return createLeaf(current.prims,alloc);

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

            /* recurse into each child and perform reduction */
            for (size_t i=0; i<children.size(); i++) {
              values[i] = createLargeLeaf(children[i],alloc);
            }

            float area = 0.0f, cost = 0.0f;
            for (size_t i=0; i<children.size(); i++) {
              area += values[i].area;
              cost += values[i].cost;
            }

            if (hasTimeSplits || useNodeMB(values,children.size())) 
            {
              auto node = createAlignedNodeMB(alloc, hasTimeSplits);
              LBBox3fa gbounds = empty;
              for (size_t i=0; i<children.size(); i++) {
                setAlignedNodeMB(node,i,values[i]);
                gbounds.extend(values[i].lbounds);
              }
              
              if (unlikely(hasTimeSplits))
                gbounds = current.prims.linearBounds(recalculatePrimRef);

              return NodeRecordMB4D(node,gbounds,current.prims.time_range,area,cost);
            }
            else
            {
              auto node = createAlignedNode(alloc);
              LBBox3fa gbounds = empty;
              for (size_t i=0; i<children.size(); i++) {
                setAlignedNode(node,i,values[i].ref,values[i].lbounds.bounds());
                gbounds.extend(values[i].lbounds);
              }
              return NodeRecordMB4D(node,gbounds,current.prims.time_range,area,cost);   
            }          
          }

          __forceinline bool useNodeMB(NodeRecordMB4D values[MAX_BRANCHING_FACTOR], size_t N) const 
          {
            float lA = 0.0f;
            float A = 0.0f;
            for (size_t i=0; i<N; i++) {
              lA += expectedApproxHalfArea(values[i].lbounds);
              A  += halfArea(values[i].lbounds.bounds());
            }
            return lA <= 0.6f*A;
          }

          const NodeRecordMB4D recurse(const BuildRecord& current, Allocator alloc, bool toplevel)
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
                 type_settings[Leaf::TY_HAIR]);
              convert_PrimRefArray_To_PrimRefMBArray(prims,primsMB,current.prims.object_range.size());
              return NodeRecordMB4D(ref,LBBox3fa(current.prims.geomBounds),current.prims.time_range,0.0f,4.0f*current.size());
            }

            if (current.prims.isType(Leaf::TY_HAIR_MB)) 
            {
              NodeRecordMB4D r = BVHBuilderHairMSMBlur::build<NodeRef>
                (scene, *current.prims.prims, current.prims,
                 recalculatePrimRef,
                 createAlloc,
                 createAlignedNode,
                 setAlignedNode,
                 createAlignedNodeMB,
                 setAlignedNodeMB,
                 createUnalignedNode,
                 setUnalignedNode,
                 createUnalignedNodeMB,
                 setUnalignedNodeMB,
                 createLeaf,
                 createLeaf,
                 progressMonitor,
                 type_settings[Leaf::TY_HAIR_MB]);
              r.area = 0.0f;
              r.cost = 4.0f*current.size();
              return r;
            }

            const Settings& cfg = getSettings(current.prims.types);

            /* get thread local allocator */
            if (!alloc)
              alloc = createAlloc();

            /* call memory monitor function to signal progress */
            if (toplevel && current.size() <= cfg.singleThreadThreshold)
              progressMonitor(current.size());

            /*! find best split */
            const Split csplit = find(current.prims);

            /*! compute leaf and split cost */
            const float leafSAH  = cfg.intCost*current.prims.leafSAH(cfg.logBlockSize);
            const float splitSAH = cfg.travCost*current.prims.halfArea()+cfg.intCost*csplit.splitSAH();
            assert((current.size() == 0) || ((leafSAH >= 0) && (splitSAH >= 0)));

            /*! create a leaf node when threshold reached or SAH tells us to stop */
            if (current.size() <= cfg.minLeafSize || current.depth+MIN_LARGE_LEAF_LEVELS >= cfg.maxDepth || (current.size() <= cfg.maxLeafSize && leafSAH <= splitSAH)) {
              current.prims.deterministic_order();
              return createLargeLeaf(current,alloc);
            }

            /*! perform initial split */
            SetMB lprims,rprims;
            std::unique_ptr<mvector<PrimRefMB>> new_vector = split(csplit,current.prims,lprims,rprims);
            bool hasTimeSplits = new_vector != nullptr;
            NodeRecordMB4D values[MAX_BRANCHING_FACTOR];
            LocalChildList children(current);
            {
              BuildRecord lrecord(lprims,current.depth+1);
              BuildRecord rrecord(rprims,current.depth+1);
              children.split(0,lrecord,rrecord,std::move(new_vector));
            }

            /*! split until node is full or SAH tells us to stop */
            while (children.size() < cfg.branchingFactor) 
            {
              /*! find best child to split */
              float bestArea = neg_inf;
              ssize_t bestChild = -1;
              for (size_t i=0; i<children.size(); i++)
              {
                if (children[i].size() <= cfg.minLeafSize) continue;
                if (expectedApproxHalfArea(children[i].prims.geomBounds) > bestArea) {
                  bestChild = i; bestArea = expectedApproxHalfArea(children[i].prims.geomBounds);
                }
              }
              if (bestChild == -1) break;

              /* perform split */
              BuildRecord& brecord = children[bestChild];
              BuildRecord lrecord(current.depth+1);
              BuildRecord rrecord(current.depth+1);
              Split csplit = find(brecord.prims);
              std::unique_ptr<mvector<PrimRefMB>> new_vector = split(csplit,brecord.prims,lrecord.prims,rrecord.prims);
              hasTimeSplits |= new_vector != nullptr;
              children.split(bestChild,lrecord,rrecord,std::move(new_vector));
            }

            /* sort buildrecords for simpler shadow ray traversal */
            //children.sort();
            
            /* spawn tasks */
            if (unlikely(current.size() > cfg.singleThreadThreshold))
            {
              /*! parallel_for is faster than spawing sub-tasks */
              parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                  for (size_t i=r.begin(); i<r.end(); i++) {
                    values[i] = recurse(children[i],nullptr,true);
                  }
                });
            }
            /* recurse into each child */
            else
            {
              //for (size_t i=0; i<children.size(); i++)
              for (ssize_t i=children.size()-1; i>=0; i--) {
                values[i] = recurse(children[i],alloc,false);
              }
            }
            
            int order[MAX_BRANCHING_FACTOR];
            for (size_t i=0; i<children.size(); i++) order[i] = i;
#if 1
            std::sort(order,order+children.size(),[&] ( const int a, const int b ) {
                return values[a].area/values[a].cost < values[b].area/values[b].cost;
                //return values[a].cost > values[b].cost;
              });
#else
            std::sort(order,order+children.size(),[&] ( const int a, const int b ) {
                return children[a].size() > children[b].size();
              });
#endif
            
            float area = 0.0f, cost = 0.0f;
            for (size_t i=0; i<children.size(); i++) {
              area += values[i].area;
              cost += values[i].cost;
            }

            if (hasTimeSplits || useNodeMB(values,children.size())) 
            {
              auto node = createAlignedNodeMB(alloc, hasTimeSplits);
              LBBox3fa gbounds = empty;
              for (size_t i=0; i<children.size(); i++) {
                setAlignedNodeMB(node,i,values[order[i]]);
                gbounds.extend(values[i].lbounds);
              }
              
              if (unlikely(hasTimeSplits))
                gbounds = current.prims.linearBounds(recalculatePrimRef);

              return NodeRecordMB4D(node,gbounds,current.prims.time_range,area,cost);
            }
            else
            {
              auto node = createAlignedNode(alloc);
              LBBox3fa gbounds = empty;
              for (size_t i=0; i<children.size(); i++) {
                setAlignedNode(node,i,values[order[i]].ref,values[order[i]].lbounds.bounds());
                gbounds.extend(values[i].lbounds);
              }
              return NodeRecordMB4D(node,gbounds,current.prims.time_range,area,cost);   
            }     
          }

          /*! builder entry function */
          __forceinline const NodeRecordMB4D operator() (mvector<PrimRefMB>& prims, const PrimInfoMB& pinfo)
          {
            const SetMB set(pinfo,&prims);
            auto ret = recurse(BuildRecord(set,1),nullptr,true);
            _mm_mfence(); // to allow non-temporal stores during build
            return ret;
          }

        private:
          const Settings& default_settings;
          const Settings* type_settings;
          Scene* scene;
          HeuristicArrayBinningMB<PrimRefMB,MBLUR_NUM_OBJECT_BINS> heuristicObjectSplit;
          HeuristicMBlurTemporalSplit<PrimRefMB,RecalculatePrimRef,MBLUR_NUM_TEMPORAL_BINS> heuristicTemporalSplit;
          const RecalculatePrimRef recalculatePrimRef;
          const CreateAllocFunc createAlloc;
          const CreateAlignedNodeFunc createAlignedNode;
          const SetAlignedNodeFunc setAlignedNode;
          const CreateAlignedNodeMBFunc createAlignedNodeMB;
          const SetAlignedNodeMBFunc setAlignedNodeMB;
          const CreateUnalignedNodeFunc createUnalignedNode;
          const SetUnalignedNodeFunc setUnalignedNode;
          const CreateUnalignedNodeMBFunc createUnalignedNodeMB;
          const SetUnalignedNodeMBFunc setUnalignedNodeMB;
          const CreateLeafFunc& createLeaf;
          const ProgressMonitor progressMonitor;
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
        typename ProgressMonitorFunc>

        static const BVHNodeRecordMB4D<NodeRef> build(mvector<PrimRefMB>& prims,
                                                      const PrimInfoMB& pinfo,
                                                      Scene* scene,
                                                      const RecalculatePrimRef recalculatePrimRef,
                                                      const CreateAllocFunc createAlloc,
                                                      const CreateAlignedNodeFunc createAlignedNode,
                                                      const SetAlignedNodeFunc setAlignedNode,
                                                      const CreateAlignedNodeMBFunc createAlignedNodeMB,
                                                      const SetAlignedNodeMBFunc setAlignedNodeMB,
                                                      const CreateUnalignedNodeFunc createUnalignedNode,
                                                      const SetUnalignedNodeFunc setUnalignedNode,
                                                      const CreateUnalignedNodeMBFunc createUnalignedNodeMB,
                                                      const SetUnalignedNodeMBFunc setUnalignedNodeMB,
                                                      const CreateLeafFunc& createLeaf,
                                                      const ProgressMonitorFunc progressMonitor,
                                                      const Settings& default_settings,
                                                      const Settings* type_settings)
      {
          typedef BuilderT<
            NodeRef,
            RecalculatePrimRef,
            decltype(createAlloc()),
            CreateAllocFunc,
            CreateAlignedNodeFunc,
            SetAlignedNodeFunc,
            CreateAlignedNodeMBFunc,
            SetAlignedNodeMBFunc,
            CreateUnalignedNodeFunc,
            SetUnalignedNodeFunc,
            CreateUnalignedNodeMBFunc,
            SetUnalignedNodeMBFunc,
            CreateLeafFunc,
            ProgressMonitorFunc> Builder;

          Builder builder(scene,
                          recalculatePrimRef,
                          createAlloc,
                          createAlignedNode,
                          setAlignedNode,
                          createAlignedNodeMB,
                          setAlignedNodeMB,
                          createUnalignedNode,
                          setUnalignedNode,
                          createUnalignedNodeMB,
                          setUnalignedNodeMB,
                          createLeaf,
                          progressMonitor,
                          default_settings,
                          type_settings);


          return builder(prims,pinfo);
        }
    };
  }
}
