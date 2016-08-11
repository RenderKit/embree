// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#define SPATIAL_DOUBLE_BUFFERED 0

namespace embree
{
  namespace isa
  { 

    /* serial partitioning */
    template<typename T, typename V, typename Compare, typename Reduction_T>
      __forceinline size_t serial_partitioning(T* const array, 
                                               T* const left,
                                               T* const right, 
                                               const size_t begin,
                                               const size_t end, 
                                               V& leftReduction,
                                               V& rightReduction,
                                               const Compare& cmp, 
                                               const Reduction_T& reduction_t)
    {
      T* l = left;
      T* r = right;

      for (size_t i=begin;i<end;i++)
      {
        if (cmp(array[i]))
        {
          reduction_t(leftReduction,array[i]);
          *l++ = array[i];
        }
        else
        {
          reduction_t(rightReduction,array[i]);
          *r-- = array[i];
        }
      }
      return begin + l - left;
    }

template<size_t BINS>
	  struct SpatialSplit
	  {
		  typedef BinSplit<BINS> Split;
		  Split split;
		  size_t leftCount;
		  size_t rightCount;

		  __forceinline SpatialSplit () {}

		  __forceinline SpatialSplit(const Split &other) 
		  {
			  split = other;
			  
		  }

		  __forceinline SpatialSplit(const SpatialSplit& other)
		  {
			  split = other.split;
		  }

		  __forceinline SpatialSplit& operator= (const SpatialSplit& other)
		  {
			  split = other.split;
			  return *this;
		  }
  };


    /*! Performs standard object binning */
#if defined(__AVX512F__)
    template<typename PrimRef, size_t BINS = 16>
#else
      template<typename PrimRef, size_t BINS = 32>
#endif
      struct HeuristicArraySpatialSAH
      {
        typedef BinSplit<BINS> Split;
		//typedef SpatialSplit<BINS> Split;
        typedef BinInfo<BINS,PrimRef> Binner;
        typedef range<size_t> Set;

#if defined(__AVX512F__)
        static const size_t PARALLEL_THRESHOLD = 3*1024; 
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 768;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 128;
#else
        static const size_t PARALLEL_THRESHOLD = 3*1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 128;
#endif
        __forceinline HeuristicArraySpatialSAH ()
          : prims0(nullptr), prims1(nullptr) {}
        
        /*! remember prim array */
        __forceinline HeuristicArraySpatialSAH (PrimRef* prims0,PrimRef* prims1)
          : prims0(prims0), prims1(prims1) {}

        /*! finds the best split */
        const Split find(const PrimInfo& pinfo, const size_t logBlockSize)
        {
          //Set set(pinfo.begin,pinfo.end);
          //if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
          return sequential_find(set,pinfo,logBlockSize);
          //else
          //  return   parallel_find(set,pinfo,logBlockSize);
        }

        /*! finds the best split */
        const Split find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          //std::cout << std::endl;
          //PING;
          //PRINT(pinfo);
		  //PRINT(pinfo.index);
          //if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
          return sequential_find(set,pinfo,logBlockSize);
          //else
          //  return   parallel_find(set,pinfo,logBlockSize);
        }
        
        /*! finds the best split */
        const Split sequential_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
#if SPATIAL_DOUBLE_BUFFERED == 1
          PrimRef* const source = (pinfo.index % 2) ? prims1 : prims0;
#else
          PrimRef* const source = prims0;
#endif
          //PRINT(pinfo.geomBounds);

          for (size_t i=set.begin();i<set.end();i++)
            assert(subset(source[i].bounds(),pinfo.geomBounds));

          Binner binner(empty); // FIXME: this clear can be optimized away
          const BinMapping<BINS> mapping(pinfo);
          binner.bin(source,set.begin(),set.end(),mapping);
          return binner.best(mapping,logBlockSize);
        }
        
        /*! finds the best split */
        __noinline const Split parallel_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          const BinMapping<BINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround
          binner = parallel_reduce(set.begin(),set.end(),PARALLEL_FIND_BLOCK_SIZE,binner,
                                   [&] (const range<size_t>& r) -> Binner { Binner binner(empty); binner.bin(prims0+r.begin(),r.size(),_mapping); return binner; },
                                   [&] (const Binner& b0, const Binner& b1) -> Binner { Binner r = b0; r.merge(b1,_mapping.size()); return r; });
          return binner.best(mapping,logBlockSize);
        }
        
        /*! array partitioning */
        /* void split(const Split& spliti, const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right)  */
        /* { */
        /*   Set lset,rset; */
        /*   Set set(pinfo.begin,pinfo.end); */
        /*   if (likely(pinfo.size() < PARALLEL_THRESHOLD))  */
        /*     sequential_split(spliti,set,left,lset,right,rset); */
        /*   else */
        /*     parallel_split(spliti,set,left,lset,right,rset); */
        /* } */
        
        /*! array partitioning */
        void split(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          //std::cout << std::endl;
          //if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
          //PING;
          sequential_split(split,set,left,lset,right,rset,pinfo.index);
          //PRINT(split);
          //PRINT(pinfo);
          //PRINT(left);
          //PRINT(right);

          //else                                
          //parallel_split(split,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void sequential_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset, const size_t index) 
        {
          // determine input and output primref arrays
#if SPATIAL_DOUBLE_BUFFERED == 1
          PrimRef* const source = (index % 2) ? prims1 : prims0;
          PrimRef* const dest   = (index % 2) ? prims0 : prims1;
#else
          PrimRef* const source = prims0;
          //PrimRef* const dest   = prims1;
#endif
          if (unlikely(!split.valid())) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset,index);
          }
          
		  //const size_t countBinningLeft = 

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
#else
          const vint4 vSplitPos(splitPos);
          const vbool4 vSplitMask( (int)splitDimMask );
#endif

#if SPATIAL_DOUBLE_BUFFERED == 1
          size_t center = serial_partitioning(source,
                                              dest + begin,
                                              dest + end - 1,
#else
                                              size_t center = serial_partitioning(source,
#endif
                                                                                  begin,end,local_left,local_right,
                                                                                  [&] (const PrimRef& ref) { 
#if defined(__AVX512F__)
                                                                                    return split.mapping.bin_unsafe(ref,vSplitPos,vSplitMask);                                                 
#else
                                                                                    return any(((vint4)split.mapping.bin_unsafe(center2(ref.bounds())) < vSplitPos) & vSplitMask); 
#endif
                                                                                  },
                                                                                  [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); });          
          
                                              //PRINT(center);
                                              new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds,index+1);
                                              new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds,index+1);
                                              new (&lset) range<size_t>(begin,center);
                                              new (&rset) range<size_t>(center,end);
                                              assert(area(left.geomBounds) >= 0.0f);
                                              assert(area(right.geomBounds) >= 0.0f);

#if SPATIAL_DOUBLE_BUFFERED == 1
                                              for (size_t i=begin;i<center;i++)
                                                assert(subset(dest[i].bounds(),local_left.geomBounds));
                                              for (size_t i=center;i<end;i++)
                                                assert(subset(dest[i].bounds(),local_right.geomBounds));
#endif
                                              }
        
          /*! array partitioning */
          __noinline void parallel_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
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
            const unsigned int splitDimMask = (unsigned int)1 << splitDim;

#if defined(__AVX512F__)
            const vint16 vSplitPos(splitPos);
            const vbool16 vSplitMask( (int)splitDimMask );
            auto isLeft = [&] (const PrimRef &ref) { return split.mapping.bin_unsafe(ref,vSplitPos,vSplitMask); };
#else
            const vint4 vSplitPos(splitPos);
            const vbool4 vSplitMask( (int)splitDimMask );
            auto isLeft = [&] (const PrimRef &ref) { return any(((vint4)split.mapping.bin_unsafe(center2(ref.bounds())) < vSplitPos) & vSplitMask); };

#endif
            const size_t mid = parallel_in_place_partitioning_static<PARALLEL_PARITION_BLOCK_SIZE,PrimRef,PrimInfo>(
              &prims0[begin],end-begin,init,left,right,isLeft,
              [] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.add(ref.bounds()); },
              [] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); });
          
            const size_t center = begin+mid;
            left.begin  = begin;  left.end  = center; 
            right.begin = center; right.end = end;
          
            new (&lset) range<size_t>(begin,center);
            new (&rset) range<size_t>(center,end);
          }
        
          void deterministic_order(const Set& set) 
          {
            /* required as parallel partition destroys original primitive order */
            std::sort(&prims0[set.begin()],&prims0[set.end()]);
          }

          void deterministic_order(const PrimInfo& pinfo) 
          {
            /* required as parallel partition destroys original primitive order */
            std::sort(&prims0[pinfo.begin],&prims0[pinfo.end]);
          }
        
          /*! array partitioning */
          /* void splitFallback(const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right)  */
          /* { */
          /*   Set lset,rset; */
          /*   Set set(pinfo.begin,pinfo.end); */
          /*   splitFallback(set,left,lset,right,rset); */
          /* } */
        
          void splitFallback(const Set& set, 
                             PrimInfo& linfo, Set& lset, 
                             PrimInfo& rinfo, Set& rset,
                             const size_t index)
          {
            FATAL("HERE");
            const size_t begin = set.begin();
            const size_t end   = set.end();
            const size_t center = (begin + end)/2;

#if SPATIAL_DOUBLE_BUFFERED == 1
            PrimRef* const source = (index % 2) ? prims1 : prims0;
            PrimRef* const dest   = (index % 2) ? prims0 : prims1;
#else
            PrimRef* const source = prims0;
            PrimRef* const dest   = prims1;
#endif
          
            CentGeomBBox3fa left; left.reset();
            for (size_t i=begin; i<center; i++)
            {
              left.extend(source[i].bounds());
              dest[i] = source[i];
            }
            new (&linfo) PrimInfo(begin,center,left.geomBounds,left.centBounds,index+1);
          
            CentGeomBBox3fa right; right.reset();
            for (size_t i=center; i<end; i++)
            {
              right.extend(source[i].bounds());	
              dest[i] = source[i];
            }
            new (&rinfo) PrimInfo(center,end,right.geomBounds,right.centBounds,index+1);
          
            new (&lset) range<size_t>(begin,center);
            new (&rset) range<size_t>(center,end);
          }
        
        private:
          PrimRef* const prims0;
          PrimRef* const prims1;
        };
      }
  }
