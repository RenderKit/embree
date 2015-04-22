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

#pragma once

#include "heuristic_binning.h"

namespace embree
{
  namespace isa
  { 
    /*! Performs standard object binning */
    template<typename PrimRef, size_t BINS = 32>
      struct HeuristicArrayBinningSAH
      {
        typedef BinSplit<BINS> Split;
        typedef BinInfo<BINS,PrimRef> Binner;
        typedef range<size_t> Set;
        
        static const size_t PARALLEL_THRESHOLD = 10000;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 4096;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 64;

        __forceinline HeuristicArrayBinningSAH ()
          : prims(nullptr) {}
        
        /*! remember prim array */
        __forceinline HeuristicArrayBinningSAH (PrimRef* prims)
          : prims(prims) {}

        const std::pair<BBox3fa,BBox3fa> computePrimInfoMB(Scene* scene, const PrimInfo& pinfo)
        {
          BBox3fa bounds0 = empty;
          BBox3fa bounds1 = empty;
          for (size_t i=pinfo.begin; i<pinfo.end; i++) // FIXME: parallelize
          {
            Bezier1v& prim = prims[i];
            const size_t geomID = prim.geomID();
            const BezierCurves* curves = scene->getBezierCurves(geomID);
            bounds0.extend(curves->bounds(prim.primID(),0));
            bounds1.extend(curves->bounds(prim.primID(),1));
          }
          return std::pair<BBox3fa,BBox3fa>(bounds0,bounds1);
        }

        /*! finds the best split */
        const Split find(const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Set set(pinfo.begin,pinfo.end);
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) return sequential_find(set,pinfo,logBlockSize);
          else                                           return   parallel_find(set,pinfo,logBlockSize);
        }

        /*! finds the best split */
        const Split find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) return sequential_find(set,pinfo,logBlockSize);
          else                                           return   parallel_find(set,pinfo,logBlockSize);
        }
        
        /*! finds the best split */
        const Split sequential_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          binner.bin(prims,set.begin(),set.end(),mapping);
          return binner.best(mapping,logBlockSize);
        }
        
        /*! finds the best split */
        const Split parallel_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          binner = parallel_reduce(set.begin(),set.end(),PARALLEL_FIND_BLOCK_SIZE,binner,
                                   [&] (const range<size_t>& r) -> Binner { Binner binner(empty); binner.bin(prims+r.begin(),r.size(),mapping); return binner; },
                                   [&] (const Binner& b0, const Binner& b1) -> Binner { Binner r = b0; r.merge(b1,mapping.size()); return r; });
          return binner.best(mapping,logBlockSize);
        }
        
        /*! array partitioning */
        void split(const Split& spliti, const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right) 
        {
          Set lset,rset;
          Set set(pinfo.begin,pinfo.end);
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
            sequential_split(spliti,set,left,lset,right,rset);
          else
            parallel_split(spliti,set,left,lset,right,rset);
        }
        
        /*! array partitioning */
        void split(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
            sequential_split(split,set,left,lset,right,rset);
          else                                
            parallel_split(split,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void sequential_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
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
          const unsigned int splitDimMask = (unsigned int)1 << splitDim; // FIXME: also use in unaligned and spatial binner

          size_t center = serial_partitioning(prims,begin,end,local_left,local_right,
                                              [&] (const PrimRef& ref) { return split.mapping.bin_unsafe(center2(ref.bounds()))[splitDim] < splitPos; },
                                              [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); });
          
          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) range<size_t>(begin,center);
          new (&rset) range<size_t>(center,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
        }
        
        /*! array partitioning */
        void parallel_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
        {
          if (!split.valid()) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset);
          }
          
          const size_t begin = set.begin();
          const size_t end   = set.end();
          left.reset(); 
          right.reset();
          PrimInfo init; init.reset();
          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          
          const size_t mid = parallel_in_place_partitioning<PARALLEL_PARITION_BLOCK_SIZE,PrimRef,PrimInfo>
	  (&prims[begin],end-begin,init,left,right,
	   [&] (const PrimRef &ref) { return split.mapping.bin_unsafe(center2(ref.bounds()))[splitDim] < splitPos; },
	   [] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.add(ref.bounds()); },
	   [] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); });
          
          const size_t center = begin+mid;
          left.begin  = begin;  left.end  = center; // FIXME: remove?
          right.begin = center; right.end = end;
          
          new (&lset) range<size_t>(begin,center);
          new (&rset) range<size_t>(center,end);
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
  }
}
