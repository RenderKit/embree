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

// TODO: 
//       - adjust parallel build thresholds
//       - do statistics about how many nodes are opened
  
#pragma once

#include "heuristic_binning.h"
#include "heuristic_spatial.h"

#define COMMON_GEOMID_TERMINATION    1
#define USE_SUBTREE_SIZE_FOR_BINNING 1
#define ENABLE_OPENING_SPLITS        1

#define DBG_PRINT(x)

namespace embree
{
  namespace isa
  { 
    template<typename ObjectSplit>
      struct SplitOpenMerge
      {
        __forceinline SplitOpenMerge () {}
        
        __forceinline SplitOpenMerge (const SplitOpenMerge& other) 
        {
          opened = other.opened;
          sah    = other.sah;
          split  = other.objectSplit();
        }
        
        __forceinline SplitOpenMerge& operator= (const SplitOpenMerge& other) 
        {
          opened = other.opened;
          sah    = other.sah;
          split  = other.objectSplit();
          return *this;
        }
          
          __forceinline     ObjectSplit&  objectSplit()        { return split; }
        __forceinline const ObjectSplit&  objectSplit() const  { return split; }
                
        __forceinline SplitOpenMerge (const ObjectSplit& objectSplit, float sah, bool opened = false)
          : opened(opened), sah(sah), split(objectSplit)
        {
        }
                
        __forceinline float splitSAH() const { 
          return sah; 
        }
        
        __forceinline bool valid() const {
          return sah < float(inf);
        }
        
      public:
        bool opened;
        float sah;
        __aligned(16) ObjectSplit split;
      };
    
    /*! Performs standard object binning */
#if defined(__AVX512F__)
    template<typename NodeOpenerFunc, typename PrimRef, size_t OBJECT_BINS = 16>
#else
      template<typename NodeOpenerFunc, typename PrimRef, size_t OBJECT_BINS = 32>
#endif
      struct HeuristicArrayOpenMergeSAH
      {
        typedef BinSplit<OBJECT_BINS> ObjectSplit;
        typedef BinInfoT<OBJECT_BINS,PrimRef,BBox3fa> ObjectBinner;

        typedef extended_range<size_t> Set;
        typedef SplitOpenMerge<ObjectSplit> Split;
        
        static const size_t PARALLEL_THRESHOLD = 3*1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 128;

        static const size_t MOVE_STEP_SIZE = 64;
        static const size_t CREATE_SPLITS_STEP_SIZE = 64;

        __forceinline HeuristicArrayOpenMergeSAH ()
          : prims0(nullptr) {}
        
        /*! remember prim array */
        __forceinline HeuristicArrayOpenMergeSAH (const NodeOpenerFunc& nodeOpenerFunc, PrimRef* prims0, const PrimInfo &root_info)
          : prims0(prims0), nodeOpenerFunc(nodeOpenerFunc), root_info(root_info) {}


        /*! compute extended ranges */
        __noinline void setExtentedRanges(const Set& set, Set& lset, Set& rset, const size_t lweight, const size_t rweight)
        {
          assert(set.ext_range_size() > 0);
          const float left_factor           = (float)lweight / (lweight + rweight);
          const size_t ext_range_size       = set.ext_range_size();
          const size_t left_ext_range_size  = min((size_t)(floorf(left_factor * ext_range_size)),ext_range_size);
          const size_t right_ext_range_size = ext_range_size - left_ext_range_size;
          lset.set_ext_range(lset.end() + left_ext_range_size);
          rset.set_ext_range(rset.end() + right_ext_range_size);
        }

        /*! move ranges */
        __noinline void moveExtentedRange(const Set& set, const Set& lset, const PrimInfo& left, Set& rset, PrimInfo& right)
        {
          const size_t left_ext_range_size = lset.ext_range_size();
          const size_t right_size = rset.size();

          /* has the left child an extended range? */
          if (left_ext_range_size > 0)
          {
            /* left extended range smaller than right range ? */
            if (left_ext_range_size < right_size)
            {
              /* only move a small part of the beginning of the right range to the end */
              parallel_for( rset.begin(), rset.begin()+left_ext_range_size, MOVE_STEP_SIZE, [&](const range<size_t>& r) {                  
                  for (size_t i=r.begin(); i<r.end(); i++)
                    prims0[i+right_size] = prims0[i];
                });
            }
            else
            {
              /* no overlap, move entire right range to new location, can be made fully parallel */
              parallel_for( rset.begin(), rset.end(), MOVE_STEP_SIZE,  [&](const range<size_t>& r) {
                  for (size_t i=r.begin(); i<r.end(); i++)
                    prims0[i+left_ext_range_size] = prims0[i];
                });
            }
            /* update right range */
            assert(rset.ext_end() + left_ext_range_size == set.ext_end());
            rset.move_right(left_ext_range_size);
            right.begin = rset.begin();
            right.end = rset.end();
          }
        }

        /*! finds the best split */
        const Split find(Set& set, PrimInfo& pinfo, const size_t logBlockSize)
        {
          /* need to avoid splitting single element ranges */
          if (pinfo.size() <= 1)
          {
            return  Split(ObjectSplit(),inf,false);
          }

#if COMMON_GEOMID_TERMINATION == 1
          bool commonGeomID = true;
          const unsigned int geomID = prims0[set.begin()].geomID;
          for (size_t i=set.begin()+1;i<set.end();i++)
            if (unlikely(prims0[i].geomID != geomID)) { commonGeomID = false; break; }
#else
          const bool commonGeomID = false;
#endif
          assert(pinfo.size() > 1);
          SplitInfo oinfo;
          const ObjectSplit object_split = object_find(set,pinfo,logBlockSize,oinfo);
          const float object_split_sah = object_split.splitSAH();

#if ENABLE_OPENING_SPLITS == 1
          if (unlikely(set.has_ext_range() && !commonGeomID))
          {
            const float OPENED_SAH_THRESHOLD = 1.25f;
            //const float OPENED_SAH_THRESHOLD = 8.0f;

            const ObjectSplit opened_object_split = opened_object_find(set, pinfo, logBlockSize);            
            const float opened_object_split_sah   = opened_object_split.splitSAH();

            DBG_PRINT(object_split_sah);
            DBG_PRINT(opened_object_split_sah);

            if (opened_object_split.valid() && opened_object_split_sah < OPENED_SAH_THRESHOLD*object_split_sah )
              {          
                DBG_PRINT("OPENING SPLIT");
                // && opened_object_split.left + opened_object_split.right - set.size() <= set.ext_range_size()
                return Split(opened_object_split,opened_object_split_sah,true);
              }
          }
#endif
          DBG_PRINT("OPENING SPLIT");
          return Split(object_split,object_split_sah,false);
        }

        /*! finds the best object split */
        __forceinline const ObjectSplit object_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo &info)
        {
          if (pinfo.size() < PARALLEL_THRESHOLD) return sequential_object_find(set,pinfo,logBlockSize,info);
          else                                   return parallel_object_find  (set,pinfo,logBlockSize,info);
        }

        /*! finds the best object split */
        __noinline const ObjectSplit sequential_object_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo &info)
        {
          ObjectBinner binner(empty); 
          const BinMapping<OBJECT_BINS> mapping(pinfo.centBounds,OBJECT_BINS);
#if USE_SUBTREE_SIZE_FOR_BINNING == 1
          binner.binSubTreeRefs(prims0,set.begin(),set.end(),mapping);
#else
          binner.bin(prims0,set.begin(),set.end(),mapping);
#endif
          ObjectSplit s = binner.best(mapping,logBlockSize);
          binner.getSplitInfo(mapping, s, info);
          return s;
        }

        /*! finds the best split */
        __noinline const ObjectSplit parallel_object_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo &info)
        {
          ObjectBinner binner(empty);
          const BinMapping<OBJECT_BINS> mapping(pinfo.centBounds,OBJECT_BINS);
          const BinMapping<OBJECT_BINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround
          binner = parallel_reduce(set.begin(),set.end(),PARALLEL_FIND_BLOCK_SIZE,binner,
                                   [&] (const range<size_t>& r) -> ObjectBinner { ObjectBinner binner(empty); 
#if USE_SUBTREE_SIZE_FOR_BINNING == 1
                                     binner.binSubTreeRefs(prims0+r.begin(),r.size(),_mapping); 
#else
                                     binner.bin(prims0+r.begin(),r.size(),_mapping); 
#endif
                                     return binner; },
                                   [&] (const ObjectBinner& b0, const ObjectBinner& b1) -> ObjectBinner { ObjectBinner r = b0; r.merge(b1,_mapping.size()); return r; });
          ObjectSplit s = binner.best(mapping,logBlockSize);
          binner.getSplitInfo(mapping, s, info);
          return s;
        }

        /*! finds the best opened object split */
        __forceinline const ObjectSplit opened_object_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          if (pinfo.size() < PARALLEL_THRESHOLD) 
            return sequential_opened_object_find(set, pinfo, logBlockSize); 
          else                                   
            return parallel_opened_object_find  (set, pinfo, logBlockSize); 
        }

        /*! finds the best opened object split */
        __noinline const ObjectSplit sequential_opened_object_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          ObjectBinner binner(empty); 
          BBox3fa cent2Bounds(pinfo.centBounds); // FIXME: empty or parent centBounds, might need both...

          assert(set.begin() == pinfo.begin);
          assert(set.end() == pinfo.end);

          for (size_t i=set.begin();i<set.end();i++)
          {
            cent2Bounds.extend(prims0[i].center2());
            PrimRef refs[8];
            size_t n = nodeOpenerFunc(prims0[i],refs);
            for (size_t j=0;j<n;j++)
              cent2Bounds.extend(refs[j].center2());
          }

          const BinMapping<OBJECT_BINS> mapping(cent2Bounds,OBJECT_BINS);          
          const BinMapping<OBJECT_BINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround

          for (size_t i=set.begin();i<set.end();i++)
          {
            PrimRef refs[8];
            size_t n = nodeOpenerFunc(prims0[i],refs);
#if USE_SUBTREE_SIZE_FOR_BINNING == 1
            binner.binSubTreeRefs(refs,n,_mapping); 
#else
            binner.bin(refs,n,_mapping); 
#endif
          }            
          return binner.best(mapping,logBlockSize); 
        }

        __noinline const ObjectSplit parallel_opened_object_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          ObjectBinner binner(empty);

          const BBox3fa cent2Bounds = parallel_reduce(set.begin(), set.end(),  BBox3fa(empty), [&] (const range<size_t>& r) -> BBox3fa {
              BBox3fa bounds(empty);
              for (size_t i=r.begin(); i<r.end(); i++) {
                PrimRef refs[8];
                size_t n = nodeOpenerFunc(prims0[i],refs);
                for (size_t j=0;j<n;j++)
                  bounds.extend(refs[j].center2());
              }
              return bounds;
            }, [] (const BBox3fa& a, const BBox3fa& b) { return merge(a,b); });

          const BinMapping<OBJECT_BINS> mapping(cent2Bounds,OBJECT_BINS);
          const BinMapping<OBJECT_BINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround
          binner = parallel_reduce(set.begin(),set.end(),PARALLEL_FIND_BLOCK_SIZE,binner,
                                   [&] (const range<size_t>& r) -> ObjectBinner { 
                                     ObjectBinner binner(empty); 
                                     for (size_t i=r.begin();i<r.end();i++)
                                     {
                                       PrimRef refs[8];
                                       size_t n = nodeOpenerFunc(prims0[i],refs);
#if USE_SUBTREE_SIZE_FOR_BINNING == 1
                                       binner.binSubTreeRefs(refs,n,_mapping); 
#else
                                       binner.bin(refs,n,_mapping); 
#endif
                                     }
                                     return binner; },
                                   [&] (const ObjectBinner& b0, const ObjectBinner& b1) -> ObjectBinner { ObjectBinner r = b0; r.merge(b1,_mapping.size()); return r; }
            );
          return binner.best(mapping,logBlockSize);
        }


        /*! open primref */
        __noinline void create_opened_object_splits(Set& set, PrimInfo& pinfo, const ObjectSplit &split, const BinMapping<OBJECT_BINS> &mapping)
        {
          assert(set.has_ext_range());
          const size_t max_ext_range_size = set.ext_range_size();
          const size_t ext_range_start = set.end();

          /* atomic counter for number of primref splits */
          std::atomic<size_t> ext_elements;
          ext_elements.store(0);
          
          const float fpos = split.mapping.pos(split.pos,split.dim);

          parallel_for( set.begin(), set.end(), CREATE_SPLITS_STEP_SIZE, [&](const range<size_t>& r) {
              for (size_t i=r.begin();i<r.end();i++)
              {
                if (unlikely(prims0[i].lower[split.dim] < fpos && prims0[i].upper[split.dim] > fpos))
                {
                  PrimRef refs[8];
                  size_t n = nodeOpenerFunc(prims0[i],refs);
                  if (likely(n==1)) continue;

                  const size_t ID = ext_elements.fetch_add(n-1);

                  /* break if the number of subdivided elements are greater than the maximal allowed size */
                  if (unlikely(ID + n - 1 >= max_ext_range_size)) { 
                    ext_elements.fetch_add(-(n-1));
#if 0
                    PRINT("EXCEED"); 
#endif
                    break; 
                  }
                  /* only write within the correct bounds */
                  assert(ID < max_ext_range_size);
                  prims0[i] = refs[0];
                  assert(prims0[i].numPrimitives);
                  for (size_t j=1;j<n;j++)
                  {
                    assert(ID+j-1 < max_ext_range_size);
                    prims0[ext_range_start+ID+j-1] = refs[j];     
                    assert(prims0[ext_range_start+ID+j-1].numPrimitives);
                  }
                }
              }
            });

          assert(ext_elements.load() < max_ext_range_size);

          const size_t numExtElements = min(max_ext_range_size,ext_elements.load());          
          //assert(numExtElements <= max_ext_range_size);
          assert(set.end()+numExtElements<=set.ext_end());
          Set nset(set.begin(),set.end()+numExtElements,set.ext_end());
          pinfo.begin = nset.begin();
          pinfo.end   = nset.end();
          set = nset;
        }
        
        /*! array partitioning */
        void split(const Split& split, const PrimInfo& pinfo_i, const Set& set_i, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          Set set = set_i;
          PrimInfo pinfo = pinfo_i; 

          /* valid split */
          if (unlikely(!split.valid())) {
            deterministic_order(set);
            splitFallback(set,left,lset,right,rset);
            return;
          }

          std::pair<size_t,size_t> ext_weights(0,0);
          if (unlikely(split.opened))
          {
            create_opened_object_splits(set,pinfo,split.objectSplit(), split.objectSplit().mapping); 

            /* opened split */
            if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
              ext_weights = sequential_opened_object_split(split.objectSplit(),set,left,lset,right,rset);
            else
              ext_weights = parallel_opened_object_split(split.objectSplit(),set,left,lset,right,rset);


            /* FIXME: does actually happen with paritially opened list of nodes */
            if (lset.size() == 0 || rset.size() == 0)
            {
              DBG_PRINT("FALLBACK");
              deterministic_order(set);
              splitFallback(set,left,lset,right,rset);
              DBG_PRINT(lset);
              DBG_PRINT(rset);
              return;
            }
            
            assert(lset.size() >= 1);
            assert(rset.size() >= 1);
              
          }
          else
          {
            /* object split */
            if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
              ext_weights = sequential_object_split(split.objectSplit(),set,left,lset,right,rset);
            else
              ext_weights = parallel_object_split(split.objectSplit(),set,left,lset,right,rset);
          }

          /* if we have an extended range, set extended child ranges and move right split range */
          if (unlikely(set.has_ext_range())) 
          {
            setExtentedRanges(set,lset,rset,ext_weights.first,ext_weights.second);
            moveExtentedRange(set,lset,left,rset,right);
          }

          assert(lset.begin() == left.begin);
          assert(lset.end()   == left.end);
          assert(lset.size()  == left.size());
          assert(rset.begin() == right.begin);
          assert(rset.end()   == right.end);
          assert(rset.size()  == right.size());
        }

        /*! array partitioning */
        std::pair<size_t,size_t> sequential_object_split(const ObjectSplit& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          const size_t begin = set.begin();
          const size_t end   = set.end();
          PrimInfo local_left(empty);
          PrimInfo local_right(empty);
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

          size_t center = serial_partitioning(prims0,
                                              begin,end,local_left,local_right,
                                              [&] (const PrimRef& ref) { 
                                                return split.mapping.bin_unsafe(ref,vSplitPos,vSplitMask);
                                              },
                                              [] (PrimInfo& pinfo,const PrimRef& ref) { pinfo.add(ref.bounds()); });          
          
          const size_t left_weight  = local_left.end;
          const size_t right_weight = local_right.end;

          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) extended_range<size_t>(begin,center,center);
          new (&rset) extended_range<size_t>(center,end,end);

          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
          return std::pair<size_t,size_t>(left_weight,right_weight);
        }


        /*! array partitioning */
        __noinline std::pair<size_t,size_t> sequential_opened_object_split(const ObjectSplit& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          const size_t begin = set.begin();
          const size_t end   = set.end();
          PrimInfo local_left(empty);
          PrimInfo local_right(empty);
          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          const unsigned int splitDimMask = (unsigned int)1 << splitDim; 

          /* init opened object mapping */
          const BinMapping<OBJECT_BINS> &mapping = split.mapping;
          const vint4 vSplitPos(splitPos);
          const vbool4 vSplitMask( (int)splitDimMask );

          size_t center = serial_partitioning(prims0,
                                              begin,end,local_left,local_right,
                                              [&] (const PrimRef& ref) {
                                                const Vec3fa c = ref.center2();
                                                return any(((vint4)mapping.bin(c) < vSplitPos) & vSplitMask); 
                                              },
                                              [] (PrimInfo& pinfo,const PrimRef& ref) { pinfo.add(ref.bounds()); });          
          const size_t left_weight  = local_left.end;
          const size_t right_weight = local_right.end;
          
          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) extended_range<size_t>(begin,center,center);
          new (&rset) extended_range<size_t>(center,end,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);

#if DEBUG
          for (size_t i=lset.begin();i<lset.end();i++)
            assert(inside(left.centBounds,prims0[i].center2()));

          for (size_t i=rset.begin();i<rset.end();i++)
            assert(inside(right.centBounds,prims0[i].center2()));
#endif
          return std::pair<size_t,size_t>(left_weight,right_weight);
        }


        
        /*! array partitioning */
        __noinline std::pair<size_t,size_t> parallel_object_split(const ObjectSplit& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
        {
          const size_t begin = set.begin();
          const size_t end   = set.end();
          left.reset(); 
          right.reset();
          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          const unsigned int splitDimMask = (unsigned int)1 << splitDim;

#if defined(__AVX512F__)
          const vint16 vSplitPos(splitPos);
          const vbool16 vSplitMask( (int)splitDimMask );
#else
          const vint4 vSplitPos(splitPos);
          const vbool4 vSplitMask( (int)splitDimMask );
#endif
          auto isLeft = [&] (const PrimRef &ref) { return split.mapping.bin_unsafe(ref,vSplitPos,vSplitMask); };

          const size_t center = parallel_partitioning(
            prims0,begin,end,EmptyTy(),left,right,isLeft,
            [] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.add(ref.bounds()); },
            [] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); },
            PARALLEL_PARTITION_BLOCK_SIZE);

          const size_t left_weight  = left.end;
          const size_t right_weight = right.end;
          
          left.begin  = begin;  left.end  = center; 
          right.begin = center; right.end = end;
          
          new (&lset) extended_range<size_t>(begin,center,center);
          new (&rset) extended_range<size_t>(center,end,end);

          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
          return std::pair<size_t,size_t>(left_weight,right_weight);
        }

        /*! array partitioning */
        __noinline std::pair<size_t,size_t> parallel_opened_object_split(const ObjectSplit& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
        {
          const size_t begin = set.begin();
          const size_t end   = set.end();
          left.reset(); 
          right.reset();

          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          const unsigned int splitDimMask = (unsigned int)1 << splitDim;

          /* init opened object mapping */
          const BinMapping<OBJECT_BINS> &mapping = split.mapping;
          const vint4 vSplitPos(splitPos);
          const vbool4 vSplitMask( (int)splitDimMask );

          auto isLeft = [&] (const PrimRef &ref) { 
            const Vec3fa c = ref.center2();
            return any(((vint4)mapping.bin(c) < vSplitPos) & vSplitMask); };

          const size_t center = parallel_partitioning(
            prims0,begin,end,EmptyTy(),left,right,isLeft,
            [] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.add(ref.bounds()); },
            [] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); },
            PARALLEL_PARTITION_BLOCK_SIZE);

          const size_t left_weight  = left.end;
          const size_t right_weight = right.end;
          
          left.begin  = begin;  left.end  = center; 
          right.begin = center; right.end = end;
          
          new (&lset) extended_range<size_t>(begin,center,center);
          new (&rset) extended_range<size_t>(center,end,end);

          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);

#if DEBUG
          for (size_t i=lset.begin();i<lset.end();i++)
            assert(inside(left.centBounds,prims0[i].center2()));

          for (size_t i=rset.begin();i<rset.end();i++)
            assert(inside(right.centBounds,prims0[i].center2()));
#endif

          return std::pair<size_t,size_t>(left_weight,right_weight);
        }

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

          PrimInfo left(empty);
          for (size_t i=begin; i<center; i++)
          {
            left.add(prims0[i].bounds());
          }
          const size_t lweight = left.end;
          new (&linfo) PrimInfo(begin,center,left.geomBounds,left.centBounds);
          
          PrimInfo right(empty);
          for (size_t i=center; i<end; i++)
          {
            right.add(prims0[i].bounds());	
          }
          const size_t rweight = right.end;
          new (&rinfo) PrimInfo(center,end,right.geomBounds,right.centBounds);         

          new (&lset) extended_range<size_t>(begin,center,center);
          new (&rset) extended_range<size_t>(center,end,end);

          /* if we have an extended range */
          if (set.has_ext_range()) 
          {
            setExtentedRanges(set,lset,rset,lweight,rweight);
            moveExtentedRange(set,lset,linfo,rset,rinfo);              
          }
        }
        
      private:
        PrimRef* const prims0;
        const NodeOpenerFunc& nodeOpenerFunc;
        const PrimInfo& root_info;
      };
  }
}
