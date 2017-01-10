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

#include "heuristic_binning.h"

namespace embree
{
  namespace isa
  {
    /*! Performs standard object binning */
    template<typename PrimRef, size_t BINS>
      struct HeuristicArrayBinningSAH
      {
        typedef BinSplit<BINS> Split;
        typedef BinInfoT<BINS,PrimRef,BBox3fa> Binner;
        typedef range<size_t> Set;

#if defined(__AVX512F__)
        static const size_t PARALLEL_THRESHOLD = 4*768;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 768;
        static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 768;
#else
        static const size_t PARALLEL_THRESHOLD = 3 * 1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 128;
#endif
        __forceinline HeuristicArrayBinningSAH ()
          : prims(nullptr) {}

        /*! remember prim array */
        __forceinline HeuristicArrayBinningSAH (PrimRef* prims)
          : prims(prims) {}

        const LBBox3fa computePrimInfoMB(size_t timeSegment, size_t numTimeSteps, Scene* scene, const PrimInfo& pinfo)
        {
          LBBox3fa allBounds = empty;
          for (size_t i=pinfo.begin; i<pinfo.end; i++) // FIXME: parallelize
          {
            BezierPrim& prim = prims[i];
            const size_t geomID = prim.geomID();
            const BezierCurves* curves = scene->getBezierCurves(geomID);
            allBounds.extend(curves->linearBounds(prim.primID(),timeSegment,numTimeSteps));
          }
          return allBounds;
        }

        /*! finds the best split */
        __noinline const Split find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          if (likely(pinfo.size() < PARALLEL_THRESHOLD))
            return find_template<false>(set,pinfo,logBlockSize);
          else
            return find_template<true>(set,pinfo,logBlockSize);
        }

        template<bool parallel>
        __forceinline const Split find_template(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          binner.template bin_serial_or_parallel<parallel>(prims,set.begin(),set.end(),PARALLEL_FIND_BLOCK_SIZE,mapping);
          return binner.best(mapping,logBlockSize);
        }

        __forceinline const Split find(const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Set set(pinfo.begin,pinfo.end);
          return find(set,pinfo,logBlockSize);
        }

        /*! array partitioning */
        void split(const Split& spliti, const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right)
        {
          Set lset,rset;
          Set set(pinfo.begin,pinfo.end);
          split(spliti,pinfo,set,left,lset,right,rset);
        }

        /*! array partitioning */
        __forceinline void split(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
        {
          if (likely(pinfo.size() < PARALLEL_THRESHOLD))
            split_template<false>(split,pinfo,set,left,lset,right,rset);
          else
            split_template<true>(split,pinfo,set,left,lset,right,rset);
        }

        template<bool parallel>
        __forceinline void split_template(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
        {
          if (!split.valid()) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset);
          }

          const size_t begin = set.begin();
          const size_t end   = set.end();
          CentGeomBBox3fa local_left(empty);
          CentGeomBBox3fa local_right(empty);
          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          const unsigned int splitDimMask = (unsigned int)1 << splitDim;

#if defined(__AVX512F__)
          const vint16 vSplitPos(splitPos);
          const vbool16 vSplitMask( splitDimMask );
          auto isLeft = [&] (const PrimRef &ref) { return split.mapping.bin_unsafe(ref,vSplitPos,vSplitMask); };
#else
          const vint4 vSplitPos(splitPos);
          const vbool4 vSplitMask( (int)splitDimMask );
          auto isLeft = [&] (const PrimRef &ref) { return any(((vint4)split.mapping.bin_unsafe(center2(ref.bounds())) < vSplitPos) & vSplitMask); };
#endif
          size_t center = serial_or_parallel_partitioning<parallel>(
            prims,begin,end,empty,local_left,local_right,isLeft,
            [] (CentGeomBBox3fa& pinfo,const PrimRef &ref) { pinfo.extend(ref.bounds()); },
            [] (CentGeomBBox3fa& pinfo0,const CentGeomBBox3fa &pinfo1) { pinfo0.merge(pinfo1); },
            PARALLEL_PARTITION_BLOCK_SIZE);

          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) range<size_t>(begin,center);
          new (&rset) range<size_t>(center,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
        }

        void deterministic_order(const Set& set)
        {
          /* required as parallel partition destroys original primitive order */
          std::sort(&prims[set.begin()],&prims[set.end()]);
        }

        void deterministic_order(const PrimInfo& pinfo)
        {
          /* required as parallel partition destroys original primitive order */
          std::sort(&prims[pinfo.begin],&prims[pinfo.end]);
        }

        /*! array partitioning */
        void splitFallback(const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right)
        {
          Set lset,rset;
          Set set(pinfo.begin,pinfo.end);
          splitFallback(set,left,lset,right,rset);
        }

        void splitFallback(const Set& set, PrimInfo& linfo, Set& lset, PrimInfo& rinfo, Set& rset)
        {
          const size_t begin = set.begin();
          const size_t end   = set.end();
          const size_t center = (begin + end)/2;

          CentGeomBBox3fa left; left.reset();
          for (size_t i=begin; i<center; i++)
            left.extend(prims[i].bounds());
          new (&linfo) PrimInfo(begin,center,left.geomBounds,left.centBounds);

          CentGeomBBox3fa right; right.reset();
          for (size_t i=center; i<end; i++)
            right.extend(prims[i].bounds());
          new (&rinfo) PrimInfo(center,end,right.geomBounds,right.centBounds);

          new (&lset) range<size_t>(begin,center);
          new (&rset) range<size_t>(center,end);
        }

      private:
        PrimRef* const prims;
      };

    struct SetMB 
    {
      static const size_t PARALLEL_THRESHOLD = 3 * 1024;
      static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
      static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 128;

      typedef mvector<PrimRefMB>* PrimRefVector;

      __forceinline SetMB() {}
      
      __forceinline SetMB(PrimRefVector prims, range<size_t> object_range, BBox1f time_range)
        : prims(prims), object_range(object_range), time_range(time_range) {}
      
      __forceinline SetMB(PrimRefVector prims, BBox1f time_range = BBox1f(0.0f,1.0f))
        : prims(prims), object_range(range<size_t>(0,prims->size())), time_range(time_range) {}
      
      template<typename RecalculatePrimRef>
      __forceinline LBBox3fa linearBounds(const RecalculatePrimRef& recalculatePrimRef) const
      {
        auto reduce = [&](const range<size_t>& r) -> LBBox3fa
        {
          LBBox3fa cbounds(empty);
          for (size_t j = r.begin(); j < r.end(); j++)
          {
            PrimRefMB& ref = (*prims)[j];
            auto bn = recalculatePrimRef.linearBounds(ref, time_range);
            cbounds.extend(bn.first);
          };
          return cbounds;
        };
        
        return parallel_reduce(object_range.begin(), object_range.end(), PARALLEL_FIND_BLOCK_SIZE, PARALLEL_THRESHOLD, LBBox3fa(empty),
                               reduce,
                                   [&](const LBBox3fa& b0, const LBBox3fa& b1) -> LBBox3fa { return merge(b0, b1); });
      }
      
    public:
      PrimRefVector prims;
      range<size_t> object_range;
      BBox1f time_range;
    };

    /*! Performs standard object binning */
    template<size_t BINS>
      struct HeuristicArrayBinningMB
      {
        typedef BinSplit<BINS> Split;
#if MBLUR_BIN_LBBOX
        typedef BinInfoT<BINS,PrimRefMB,LBBox3fa> ObjectBinner;
#else
        typedef BinInfoT<BINS,PrimRefMB,BBox3fa> ObjectBinner;
#endif
        static const size_t PARALLEL_THRESHOLD = 3 * 1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 128;

        /*! finds the best split */
        const Split find(const SetMB& set, const PrimInfoMB& pinfo, const size_t logBlockSize)
        {
          ObjectBinner binner(empty); // FIXME: this clear can be optimized away
          const BinMapping<BINS> mapping(pinfo.centBounds,pinfo.size());
          binner.bin_parallel(set.prims->data(),set.object_range.begin(),set.object_range.end(),PARALLEL_FIND_BLOCK_SIZE,PARALLEL_THRESHOLD,mapping);
          Split osplit = binner.best(mapping,logBlockSize);
          osplit.sah *= pinfo.time_range.size();
          if (!osplit.valid()) osplit.data = Split::SPLIT_FALLBACK; // use fallback split
          return osplit;
        }
        
        /*! array partitioning */
        __forceinline void split(const Split& split, const PrimInfoMB& pinfo, const SetMB& set, PrimInfoMB& left, SetMB& lset, PrimInfoMB& right, SetMB& rset)
        {
          const size_t begin = set.object_range.begin();
          const size_t end   = set.object_range.end();
          left = empty;
          right = empty;
          const vint4 vSplitPos(split.pos);
          const vbool4 vSplitMask(1 << split.dim);
          auto isLeft = [&] (const PrimRefMB &ref) { return any(((vint4)split.mapping.bin_unsafe(ref) < vSplitPos) & vSplitMask); };
          auto reduction = [] (PrimInfoMB& pinfo, const PrimRefMB& ref) { pinfo.add_primref(ref); };
          auto reduction2 = [] (PrimInfoMB& pinfo0,const PrimInfoMB& pinfo1) { pinfo0.merge(pinfo1); };
          size_t center = parallel_partitioning(set.prims->data(),begin,end,empty,left,right,isLeft,reduction,reduction2,PARALLEL_PARTITION_BLOCK_SIZE,PARALLEL_THRESHOLD);
          left.begin  = begin;  left.end = center; left.time_range = pinfo.time_range;
          right.begin = center; right.end = end;   right.time_range = pinfo.time_range;
          new (&lset) SetMB(set.prims,range<size_t>(begin,center),set.time_range);
          new (&rset) SetMB(set.prims,range<size_t>(center,end  ),set.time_range);
        }
      };
  }
}
