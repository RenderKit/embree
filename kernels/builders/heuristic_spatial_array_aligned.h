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
#include "heuristic_spatial.h"

namespace embree
{
  namespace isa
  { 
#define ENABLE_SPATIAL_SPLITS 1
#define DBG_PRINT(x)

    /*! Performs standard object binning */
#if defined(__AVX512F__)
    template<typename SplitPrimitive, typename PrimRef, size_t OBJECT_BINS = 16, size_t SPATIAL_BINS = 16>
#else
      template<typename SplitPrimitive, typename PrimRef, size_t OBJECT_BINS = 32, size_t SPATIAL_BINS = 16>
#endif
      struct HeuristicArraySpatialSAH
      {
        typedef BinSplit<OBJECT_BINS> Split;
        typedef BinInfo<OBJECT_BINS,PrimRef> ObjectBinner;
        typedef SpatialBinInfo<SPATIAL_BINS,PrimRef> SpatialBinner;
        typedef extended_range<size_t> Set;
        typedef SpatialBinSplit<SPATIAL_BINS> SpatialSplit;

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
          : prims0(nullptr) {}
        
        /*! remember prim array */
        __forceinline HeuristicArraySpatialSAH (const SplitPrimitive& splitPrimitive, PrimRef* prims0)
          : prims0(prims0), splitPrimitive(splitPrimitive) {}


        /*! compute extended ranges */
        __forceinline void setExtentedRanges(const Set& set, Set& lset, Set& rset)
        {
          /* PING; */
          /* DBG_PRINT(set); */
          /* DBG_PRINT(lset); */
          /* DBG_PRINT(rset); */

          assert(set.ext_range_size() > 0);
          const size_t parent_size          = set.size();
          const size_t ext_range_size       = set.ext_range_size();
          const size_t left_size            = lset.size();
          //const size_t right_size           = rset.size();
          const float left_factor           = (float)left_size / parent_size;
          //const float right_factor          = (float)right_size / parent_size;
          const size_t left_ext_range_size  = min((size_t)(left_factor * ext_range_size),ext_range_size);
          const size_t right_ext_range_size = ext_range_size - left_ext_range_size;

          /* std::cout << std::endl; */
          /* DBG_PRINT(parent_size); */
          /* DBG_PRINT(left_size); */
          /* DBG_PRINT(right_size); */

          /* DBG_PRINT(ext_range_size); */
          /* DBG_PRINT(left_ext_range_size); */
          /* DBG_PRINT(right_ext_range_size); */
          /* DBG_PRINT(left_factor); */
          //exit(0);
          /* DBG_PRINT("UPDATE"); */
          lset.set_ext_range(lset.end() + left_ext_range_size);
          rset.set_ext_range(rset.end() + right_ext_range_size);

          /* DBG_PRINT(lset); */
          /* DBG_PRINT(rset); */

        }

        __noinline void moveExtentedRange(const Set& set, const Set& lset, Set& rset)
        {
          const size_t left_ext_range_size = lset.ext_range_size();
          const size_t right_size = rset.size();
          /* PING; */
          /* DBG_PRINT(set); */
          /* DBG_PRINT(lset); */
          /* DBG_PRINT(rset); */
          /* DBG_PRINT(left_ext_range_size); */
          /* DBG_PRINT(right_size); */

          // has the left child an extended range ?
          if (left_ext_range_size > 0)
          {
            PrimRef* const source = prims0;
            // left extended range smaller than right range ?
            if (left_ext_range_size < right_size)
            {
              // only move a small part of of the beginning of the right range to the end
              for (size_t i=rset.begin();i<rset.begin()+left_ext_range_size;i++)
                source[i+right_size] = source[i];
            }
            else
            {
              // no overlap, move entire right range to new location, can be made fully parallel
              for (size_t i=rset.begin();i<rset.end();i++)
                source[i+left_ext_range_size] = source[i];
            }
            // update right range
            assert(rset.ext_end() + left_ext_range_size == set.ext_end());
            rset.move_right(left_ext_range_size);
            /* DBG_PRINT(rset); */
            //exit(0);
          }
        }

        /*! finds the best split */
        const Split find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          //std::cout << std::endl;
          //PING;
          //DBG_PRINT(pinfo);
          //DBG_PRINT(pinfo.index);
          //if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
          return sequential_find(set,pinfo,logBlockSize);
          //else
          //  return   parallel_find(set,pinfo,logBlockSize);
        }

        /*! finds the best object split */
        __noinline const Split object_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo &info)
        {
          PrimRef* const source = prims0;
          ObjectBinner binner(empty); // FIXME: this clear can be optimized away
          const BinMapping<OBJECT_BINS> mapping(pinfo);
          binner.bin(source,set.begin(),set.end(),mapping);
          Split s = binner.best(mapping,logBlockSize);
          s.lcount = (unsigned int)binner.getLeftCount(mapping,s);
          binner.getSplitInfo(mapping, s, info);
          return s;
        }


        /*! finds the best object split */
        __noinline const SpatialSplit spatial_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          PrimRef* const source = prims0;
          SpatialBinner binner(empty); 
          const SpatialBinMapping<SPATIAL_BINS> mapping(pinfo);
          binner.bin(splitPrimitive,source,set.begin(),set.end(),mapping);
          // todo: find best spatial split not exeeding the extended range
          SpatialSplit s = binner.best(pinfo,mapping,logBlockSize);

          //s.lcount = binner.getLeftCount(mapping,s);
          return s;
        }


        /*! subdivides primitives based on a spatial split */
        __noinline Set sequential_create_spatial_splits(const Set& set, const SpatialSplit &split, const SpatialBinMapping<SPATIAL_BINS> &mapping)
        {
          assert(set.has_ext_range());
          const size_t max_ext_range_size = set.ext_range_size();
          const size_t ext_range_start = set.end();
          size_t ext_elements = 0;
          PrimRef* source = prims0;
          
          DBG_PRINT(set.size());
          DBG_PRINT(max_ext_range_size);
          DBG_PRINT(ext_range_start);
          const float fpos = split.mapping.pos(split.pos,split.dim);

          //const BinMapping<OBJECT_BINS> test_mapping(16,2.0f*split.mapping.ofs,split.mapping.scale*0.5f);

          for (size_t i=set.begin();i<set.end();i++)
          {
            int bin0 = split.mapping.bin(source[i].lower)[split.dim];
            int bin1 = split.mapping.bin(source[i].upper)[split.dim];
            if (unlikely(bin0 < split.pos && bin1 >= split.pos))
            {
              //if (source[i].lower[split.dim] < fpos &&  source[i].upper[split.dim] > fpos)
              {
                PrimRef left,right;
                splitPrimitive(source[i],split.dim,fpos,left,right);

                //DBG_PRINT(source[i]);
                //DBG_PRINT(left);
                //DBG_PRINT(right);
                //DBG_PRINT(split.pos);
                //DBG_PRINT(split.dim);
                //DBG_PRINT(fpos);
                //DBG_PRINT(left.bounds().size()[split.dim] < 1E-5f);
                //DBG_PRINT(right.bounds().size()[split.dim] < 1E-5f);

                //const vint4 bin_left = (vint4)split.mapping.bin(center(left.bounds()));
                //const vint4 bin_right = (vint4)split.mapping.bin(center(right.bounds()));

                //DBG_PRINT(bin_left);
                //DBG_PRINT(bin_right);

                //assert(bin_left[split.dim]  <  split.pos);
                //assert(bin_right[split.dim] >= split.pos);

                source[i] = left;
                assert(ext_elements <= max_ext_range_size);
                source[ext_range_start+ext_elements++] = right;              
              }
            }
          }
          DBG_PRINT(ext_elements);
          DBG_PRINT(split.left);
          DBG_PRINT(split.right);
          DBG_PRINT(split.left+split.right);
          DBG_PRINT(set.size()+ext_elements);
          assert(split.left+split.right==set.size()+ext_elements);     
          assert(set.end()+ext_elements<=set.ext_end());
          return Set(set.begin(),set.end()+ext_elements,set.ext_end());
        }
        
        /*! finds the best split */
        const Split sequential_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          PrimRef* const source = prims0;

          for (size_t i=set.begin();i<set.end();i++)
            assert(subset(source[i].bounds(),pinfo.geomBounds));
		  SplitInfo info;
          Split object_split = object_find(set,pinfo,logBlockSize,info);
#if ENABLE_SPATIAL_SPLITS == 1
          if (unlikely(set.has_ext_range()))
          {
			const BBox3fa overlap = intersect(info.leftBounds, info.rightBounds);
			if (safeArea(overlap) >= 0.2f*safeArea(pinfo.geomBounds))
				{
			     SpatialSplit spatial_split = spatial_find(set, pinfo, logBlockSize);
					  /* valid spatial split, better SAH and number of splits do not exceed extended range */
					  if (spatial_split.sah < object_split.sah &&
						  spatial_split.left + spatial_split.right - set.size() <= set.ext_range_size())
					  {
						  DBG_PRINT(object_split);
						  DBG_PRINT(set.has_ext_range());
						  DBG_PRINT(set);
						  PRINT(spatial_split);
						  DBG_PRINT(spatial_split.left + spatial_split.right - set.size());
						  //exit(0);
						  DBG_PRINT("virtual split");
						  set = sequential_create_spatial_splits(set, spatial_split, spatial_split.mapping);
						  std::cout << std::endl;
						  // mark that we have a spatial split 
						  object_split.lcount = (unsigned int)-1;
						  object_split.sah = spatial_split.sah;
						  object_split.dim = spatial_split.dim;
						  object_split.pos = spatial_split.pos;
						  object_split.mapping.ofs = spatial_split.mapping.ofs;
						  object_split.mapping.scale = spatial_split.mapping.scale;
					  }
				  }
          }
#endif
          return object_split;
        }
        
        /*! finds the best split */
        __noinline const Split parallel_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          ObjectBinner binner(empty);
          const BinMapping<OBJECT_BINS> mapping(pinfo);
          const BinMapping<OBJECT_BINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround
          binner = parallel_reduce(set.begin(),set.end(),PARALLEL_FIND_BLOCK_SIZE,binner,
                                   [&] (const range<size_t>& r) -> ObjectBinner { ObjectBinner binner(empty); binner.bin(prims0+r.begin(),r.size(),_mapping); return binner; },
                                   [&] (const ObjectBinner& b0, const ObjectBinner& b1) -> ObjectBinner { ObjectBinner r = b0; r.merge(b1,_mapping.size()); return r; });
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
          sequential_split(split,set,left,lset,right,rset);
          //DBG_PRINT(split);
          //DBG_PRINT(pinfo);
          //DBG_PRINT(left);
          //DBG_PRINT(right);

          //else                                
          //parallel_split(split,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void sequential_object_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          PrimRef* const source = prims0;
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

          size_t center = serial_partitioning(source,
                                              begin,end,local_left,local_right,
                                              [&] (const PrimRef& ref) { 
#if defined(__AVX512F__)
                                                return split.mapping.bin_unsafe(ref,vSplitPos,vSplitMask);                                                 
#else
                                                return any(((vint4)split.mapping.bin_unsafe(center2(ref.bounds())) < vSplitPos) & vSplitMask); 
#endif
                                              },
                                              [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); });          
          
          //assert(center == begin + split.lcount);
          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) extended_range<size_t>(begin,center,center);
          new (&rset) extended_range<size_t>(center,end,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);

          // verify that the left and right ranges are correct
          for (size_t i=lset.begin();i<lset.end();i++)
            assert(subset(source[i].bounds(),local_left.geomBounds));
          for (size_t i=rset.begin();i<rset.end();i++)
            assert(subset(source[i].bounds(),local_right.geomBounds));
        }


        /*! array partitioning */
        void sequential_spatial_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          assert(split.lcount == (unsigned int)-1);

          PrimRef* const source = prims0;
          const size_t begin = set.begin();
          const size_t end   = set.end();
          CentGeomBBox3fa local_left(empty);
          CentGeomBBox3fa local_right(empty);
          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          const unsigned int splitDimMask = (unsigned int)1 << splitDim; 

          const SpatialBinMapping<SPATIAL_BINS> mapping(split.mapping.ofs,split.mapping.scale);

          const vint4 vSplitPos(splitPos);
          const vbool4 vSplitMask( (int)splitDimMask );

          size_t center = serial_partitioning(source,
                                              begin,end,local_left,local_right,
                                              [&] (const PrimRef& ref) { 
                                                return any(((vint4)mapping.bin(ref.bounds().lower) < vSplitPos) & vSplitMask); 
                                              },
                                              [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); });          
          
          //assert(center == begin + split.lcount);
          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) extended_range<size_t>(begin,center,center);
          new (&rset) extended_range<size_t>(center,end,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);

          // verify that the left and right ranges are correct
          for (size_t i=lset.begin();i<lset.end();i++)
            assert(subset(source[i].bounds(),local_left.geomBounds));
          for (size_t i=rset.begin();i<rset.end();i++)
            assert(subset(source[i].bounds(),local_right.geomBounds));
        }


        /*! array partitioning */
        void sequential_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          // determine input and output primref arrays

          if (unlikely(!split.valid())) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset);
          }
          
          if (unlikely(split.lcount == (unsigned int)-1))
          {
            DBG_PRINT("PARTITIONING");
            sequential_spatial_split(split,set,left,lset,right,rset);
            //exit(0);
          }
          else
            sequential_object_split(split,set,left,lset,right,rset);

          // if we have an extended range, move right split range
          if (set.has_ext_range()) 
          {
            setExtentedRanges(set,lset,rset);
            moveExtentedRange(set,lset,rset);
          }

        }
        
#if 0
          /*! array partitioning */
          __noinline void parallel_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
          {
          PrimRef* const source = prims0;

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
          
          new (&lset) extended_range<size_t>(begin,center,center);
          new (&rset) extended_range<size_t>(center,end,end);
          }
#endif

          void deterministic_order(const Set& set) 
          {
            /* required as parallel partition destroys original primitive order */
            std::sort(&prims0[set.begin()],&prims0[set.end()]);
          }

          void splitFallback(const Set& set, 
                             PrimInfo& linfo, Set& lset, 
                             PrimInfo& rinfo, Set& rset)
          {
            const size_t begin = set.begin();
            const size_t end   = set.end();
            const size_t center = (begin + end)/2;

            PrimRef* const source = prims0;
          
            CentGeomBBox3fa left; left.reset();
            for (size_t i=begin; i<center; i++)
            {
              left.extend(source[i].bounds());
            }
            new (&linfo) PrimInfo(begin,center,left.geomBounds,left.centBounds);
          
            CentGeomBBox3fa right; right.reset();
            for (size_t i=center; i<end; i++)
            {
              right.extend(source[i].bounds());	
            }
            new (&rinfo) PrimInfo(center,end,right.geomBounds,right.centBounds);         
            new (&lset) extended_range<size_t>(begin,center,center);
            new (&rset) extended_range<size_t>(center,end,end);

            // if we have an extended range
            if (set.has_ext_range()) 
            {
              setExtentedRanges(set,lset,rset);
              moveExtentedRange(set,lset,rset);              
            }

          }
        
        private:
          PrimRef* const prims0;
          const SplitPrimitive& splitPrimitive;

        };
      }
  }
