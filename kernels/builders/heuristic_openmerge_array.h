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
//       - openNodesBasedOnExtend should consider max extended size
  
#pragma once

#include "heuristic_binning.h"
#include "heuristic_spatial.h"

#define USE_SUBTREE_SIZE_FOR_BINNING 1

#define OPEN_STATS(x) 

#define MAX_OPENED_CHILD_NODES 8
#define MAX_EXTEND_THRESHOLD   0.1f

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
          sah    = other.sah;
          objectSplit()  = other.objectSplit();
        }
        
        __forceinline SplitOpenMerge& operator= (const SplitOpenMerge& other) 
        {
          sah    = other.sah;
          objectSplit()  = other.objectSplit();
          return *this;
        }
          
        __forceinline       ObjectSplit&  objectSplit()        { return split; }
        __forceinline const ObjectSplit&  objectSplit() const  { return split; }
        
        __forceinline SplitOpenMerge (const ObjectSplit& objectSplit, float sah)
          : sah(sah), split(objectSplit)
        {
        }
                
        __forceinline float splitSAH() const { 
          return sah; 
        }
        
        __forceinline bool valid() const {
          return sah < float(inf);
        }
        
      public:
        float sah;
        ObjectSplit split;
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
        
        static const size_t PARALLEL_THRESHOLD = 2*1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 512;
        static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 128;

        static const size_t MOVE_STEP_SIZE = 64;
        static const size_t CREATE_SPLITS_STEP_SIZE = 128;

        __forceinline HeuristicArrayOpenMergeSAH ()
          : prims0(nullptr) {}
        
        /*! remember prim array */
        __forceinline HeuristicArrayOpenMergeSAH (const NodeOpenerFunc& nodeOpenerFunc, PrimRef* prims0, const PrimInfo &root_info)
          : prims0(prims0), nodeOpenerFunc(nodeOpenerFunc), root_info(root_info), inv_root_area(1.0f / area(root_info.geomBounds)) { 
          OPEN_STATS(stat_ext_elements.store(0));
        }

        __forceinline ~HeuristicArrayOpenMergeSAH()
        {
          OPEN_STATS(PRINT(stat_ext_elements.load()));
        }

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

        // ==========================================================================
        // ==========================================================================
        // ==========================================================================

        __forceinline std::pair<size_t,bool> getProperties(const Set& set, const PrimInfo& pinfo)
        {
          const Vec3fa diag = pinfo.geomBounds.size();
          const size_t dim = maxDim(diag);
          assert(diag[dim] > 0.0f);
          const float inv_max_extend = 1.0f / diag[dim];
          const unsigned int geomID = prims0[pinfo.begin].geomID();

          if (pinfo.size() < PARALLEL_THRESHOLD)
          {
            bool commonGeomID = true;
            size_t opens = 0;
            for (size_t i=pinfo.begin;i<pinfo.end;i++)
            {
              commonGeomID &= prims0[i].geomID() == geomID; 
              if (!prims0[i].node.isLeaf() && prims0[i].bounds().size()[dim] * inv_max_extend > MAX_EXTEND_THRESHOLD) opens += MAX_OPENED_CHILD_NODES-1; //FIXME              
            }
            return std::pair<size_t,bool>(opens,commonGeomID);
          }
          else
          {
            std::pair<size_t,bool> emptyProp(0,true);
            return parallel_reduce(set.begin(),set.end(),PARALLEL_FIND_BLOCK_SIZE,emptyProp,
                                   [&] (const range<size_t>& r) -> std::pair<size_t,bool> { 
                                     bool commonGeomID = true;
                                     size_t opens = 0;
                                     for (size_t i=r.begin();i<r.end();i++)
                                     {
                                       commonGeomID &= prims0[i].geomID() == geomID; 
                                       if (!prims0[i].node.isLeaf() && prims0[i].bounds().size()[dim] * inv_max_extend > MAX_EXTEND_THRESHOLD) opens += MAX_OPENED_CHILD_NODES-1; //FIXME
                                     }
                                     return std::pair<size_t,bool>(opens,commonGeomID); },
                                   [&] (const std::pair<size_t,bool>& b0, const std::pair<size_t,bool>& b1) -> std::pair<size_t,bool> { return std::pair<size_t,bool>(b0.first+b1.first,b0.second && b1.second); });
          }
        }

        //FIXME: should consider maximum available extended size 
        __forceinline size_t openNodesBasedOnExtend(const Set& set, PrimInfo& pinfo)
        {
          const Vec3fa diag = pinfo.geomBounds.size();
          const size_t dim = maxDim(diag);
          assert(diag[dim] > 0.0f);
          const float inv_max_extend = 1.0f / diag[dim];
          const size_t ext_range_start = set.end();

          if (pinfo.size() < PARALLEL_THRESHOLD) {
            size_t extra_elements = 0;
            for (size_t i=pinfo.begin;i<pinfo.end;i++)
            {
              if (!prims0[i].node.isLeaf() && prims0[i].bounds().size()[dim] * inv_max_extend > MAX_EXTEND_THRESHOLD)
              {
                PrimRef tmp[MAX_OPENED_CHILD_NODES];
                const size_t n = nodeOpenerFunc(prims0[i],tmp);
                assert(extra_elements + n-1 <= set.ext_range_size());
                for (size_t j=0;j<n;j++)
                  pinfo.extend(tmp[j].bounds());
                  
                prims0[i] = tmp[0];
                for (size_t j=1;j<n;j++)
                  prims0[ext_range_start+extra_elements+j-1] = tmp[j]; 
                extra_elements += n-1;
              }
            }
            return extra_elements;
          }
          else {
            std::atomic<size_t> ext_elements;
            ext_elements.store(0);
            PrimInfo info = parallel_reduce( pinfo.begin, pinfo.end, CREATE_SPLITS_STEP_SIZE, PrimInfo(empty), [&](const range<size_t>& r) -> PrimInfo {
                PrimInfo info(empty);
                for (size_t i=r.begin();i<r.end();i++)
                  if (!prims0[i].node.isLeaf() && prims0[i].bounds().size()[dim] * inv_max_extend > MAX_EXTEND_THRESHOLD)
                  {
                    PrimRef tmp[MAX_OPENED_CHILD_NODES];
                    const size_t n = nodeOpenerFunc(prims0[i],tmp);
                    const size_t ID = ext_elements.fetch_add(n-1);
                    assert(ID + n-1 <= set.ext_range_size());

                    for (size_t j=0;j<n;j++)
                      info.extend(tmp[j].bounds());

                    prims0[i] = tmp[0];
                    for (size_t j=1;j<n;j++)
                      prims0[ext_range_start+ID+j-1] = tmp[j]; 
                  }
                return info;
              }, [] (const PrimInfo& a, const PrimInfo& b) { return PrimInfo::merge(a,b); });
            pinfo.centBounds.extend(info.centBounds);
            assert(ext_elements.load() <= set.ext_range_size());
            return ext_elements.load();
          }
        } 
                 
        // ==========================================================================
        // ==========================================================================
        // ==========================================================================


        const Split find(Set& set, PrimInfo& pinfo, const size_t logBlockSize)
        {
          /* single element */
          if (pinfo.size() == 1)
            return Split(ObjectSplit(),inf);

          /* too small */
          const float area_factor = area(pinfo.geomBounds) * inv_root_area;
          if (area_factor < 1E-5f) 
            set.set_ext_range(set.end()); /* disable opening */

          if (unlikely(set.has_ext_range()))
          {
            /* disjoint test */
            bool disjoint = false;
            const size_t D = 4;
            if (pinfo.size() <= D)
            {
              disjoint = true;
              for (size_t j=pinfo.begin;j<pinfo.end-1;j++)
                for (size_t i=pinfo.begin+1;i<pinfo.end;i++)
                  if (conjoint(prims0[j].bounds(),prims0[i].bounds()))
                  { disjoint = false; break; }        
            }
            if (disjoint) set.set_ext_range(set.end()); /* disable opening */
          }

          std::pair<float,bool> p(0.0f,false);

          /* common geomID */
          if (set.has_ext_range())
          {
            p =  getProperties(set,pinfo);
            const size_t est_new_elements = p.first;
            const bool commonGeomID       = p.second;
            if (commonGeomID)
            {
              const size_t max_ext_range_size = set.ext_range_size();
              size_t extra_elements = 0;

              if (est_new_elements <= max_ext_range_size)
                extra_elements = openNodesBasedOnExtend(set,pinfo);

              pinfo.end += extra_elements;
              Set nset(set.begin(),set.end()+extra_elements,set.ext_end());
              set = nset;            
              set.set_ext_range(set.end()); /* disable opening */
            }
          }

          if (set.has_ext_range())
          {
            const size_t max_ext_range_size = set.ext_range_size();
            size_t extra_elements = 0;

            const size_t est_new_elements = p.first;
            if (est_new_elements <= max_ext_range_size)
              extra_elements = openNodesBasedOnExtend(set,pinfo);

            OPEN_STATS( if (extra_elements) stat_ext_elements += extra_elements );

            pinfo.end += extra_elements;
            Set nset(set.begin(),set.end()+extra_elements,set.ext_end());
            set = nset;         
            if (set.ext_range_size() <= 1) set.set_ext_range(set.end()); /* disable opening */
          }
        
            
          /* find best split */
          SplitInfo oinfo;
          const ObjectSplit object_split = object_find(set,pinfo,logBlockSize,oinfo);
          const float object_split_sah = object_split.splitSAH();
          return Split(object_split,object_split_sah);
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

          /* object split */
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
            ext_weights = sequential_object_split(split.objectSplit(),set,left,lset,right,rset);
          else
            ext_weights = parallel_object_split(split.objectSplit(),set,left,lset,right,rset);

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
        const float inv_root_area;

        /* statistics */
        std::atomic<size_t> stat_ext_elements;
      };
  }
}
