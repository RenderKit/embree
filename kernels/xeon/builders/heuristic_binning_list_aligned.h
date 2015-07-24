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
      struct HeuristicListBinningSAH
      {
        typedef BinSplit<BINS> Split;
        typedef BinInfo<BINS,PrimRef> Binner;
        typedef atomic_set<PrimRefBlockT<PrimRef> > Set;
        typedef typename atomic_set<PrimRefBlockT<PrimRef> >::item Set_item;
        
        __forceinline HeuristicListBinningSAH () {}
        
        /*! finds the best split */
        const Split find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo& sinfo_o)
        {
          if (likely(pinfo.size() < 10000)) return sequential_find(set,pinfo,logBlockSize,sinfo_o);
          else                              return   parallel_find(set,pinfo,logBlockSize,sinfo_o);
        }
        
        /*! finds the best split */
        const Split sequential_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo& sinfo_o)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          typename Set::iterator i=set;
          while (typename Set::item* block = i.next()) {
            binner.bin(block->base(),block->size(),mapping);
          }
          const Split split = binner.best(mapping,logBlockSize);
          binner.getSplitInfo(mapping,split,sinfo_o);
          return split;
        }

        /*! finds the best split */
        const Split parallel_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo& sinfo_o)
        {
          const BinMapping<BINS> mapping(pinfo);
          const BinMapping<BINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround
          typename Set::iterator i=set;
          const size_t threadCount = TaskSchedulerTBB::threadCount();
          const Binner binner = parallel_reduce(size_t(0),threadCount,Binner(empty), [&] (const range<size_t>& r) -> Binner
          {
            Binner binner(empty);
            while (Set_item* block = i.next()) {
              binner.bin(block->base(),block->size(),_mapping);
            }
            return binner;
          },[](const Binner& a, const Binner& b) -> Binner { return HeuristicListBinningSAH<PrimRef,BINS>::Binner::reduce(a,b); });
          
          const Split split = binner.best(mapping,logBlockSize);
          binner.getSplitInfo(mapping,split,sinfo_o);
          return split;
        }

        /*! splits a list of primitives */
        void split(const Split& split, const PrimInfo& pinfo, Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (likely(pinfo.size() < 10000)) sequential_split(split,set,left,lset,right,rset);
          else                                parallel_split(split,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void sequential_split(const Split& split, Set& prims, PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
        {
          if (!split.valid()) {
            deterministic_order(prims);
            return splitFallback(prims,linfo_o,lprims_o,rinfo_o,rprims_o);
          }
          
          PrimRefList::item* lblock = lprims_o.insert(new PrimRefList::item);
          PrimRefList::item* rblock = rprims_o.insert(new PrimRefList::item);
          linfo_o.reset();
          rinfo_o.reset();
          
          size_t numLeft = 0; CentGeomBBox3fa leftBounds(empty);
          size_t numRight = 0; CentGeomBBox3fa rightBounds(empty);
      
          while (PrimRefList::item* block = prims.take()) 
          {
            for (size_t i=0; i<block->size(); i++) 
            {
              const PrimRef& prim = block->at(i); 
              const Vec3fa center = center2(prim.bounds());
              const int4 bin = int4(split.mapping.bin_unsafe(center));
              
              if (bin[split.dim] < split.pos) 
              {
                leftBounds.extend(prim.bounds()); numLeft++;
                if (likely(lblock->insert(prim))) continue; 
                lblock = lprims_o.insert(new PrimRefList::item);
                lblock->insert(prim);
              } 
              else 
              {
                rightBounds.extend(prim.bounds()); numRight++;
                if (likely(rblock->insert(prim))) continue;
                rblock = rprims_o.insert(new PrimRefList::item);
                rblock->insert(prim);
              }
            }
            delete block;
          }

          linfo_o.add(leftBounds.geomBounds,leftBounds.centBounds,numLeft);
          rinfo_o.add(rightBounds.geomBounds,rightBounds.centBounds,numRight);
        }

        /*! array partitioning */
        void parallel_split(const Split& split, Set& prims, PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
        {
          if (!split.valid()) {
            deterministic_order(prims);
            return splitFallback(prims,linfo_o,lprims_o,rinfo_o,rprims_o);
          }
                    
          linfo_o.reset();
          rinfo_o.reset();
          
          const size_t threadCount = TaskSchedulerTBB::threadCount();
          const PrimInfo2 info = parallel_reduce(size_t(0),threadCount,PrimInfo2(empty), [&] (const range<size_t>& r) -> PrimInfo2
          {
            PrimRefList::item* lblock = nullptr;
            PrimRefList::item* rblock = nullptr;
            size_t numLeft = 0; CentGeomBBox3fa leftBounds(empty);
            size_t numRight = 0; CentGeomBBox3fa rightBounds(empty);

            while (PrimRefList::item* block = prims.take()) 
            {
              if (lblock == nullptr) lblock = lprims_o.insert(new PrimRefList::item);
              if (rblock == nullptr) rblock = rprims_o.insert(new PrimRefList::item);

              for (size_t i=0; i<block->size(); i++) 
              {
                const PrimRef& prim = block->at(i); 
                const Vec3fa center = center2(prim.bounds());
                const int4 bin = int4(split.mapping.bin_unsafe(center));
                
                if (bin[split.dim] < split.pos) 
                {
                  leftBounds.extend(prim.bounds()); numLeft++;
                  if (likely(lblock->insert(prim))) continue; 
                  lblock = lprims_o.insert(new PrimRefList::item);
                  lblock->insert(prim);
                } 
                else 
                {
                  rightBounds.extend(prim.bounds()); numRight++;
                  if (likely(rblock->insert(prim))) continue;
                  rblock = rprims_o.insert(new PrimRefList::item);
                  rblock->insert(prim);
                }
              }
              delete block;
            }
            return PrimInfo2(PrimInfo(numLeft ,leftBounds .geomBounds,leftBounds .centBounds),
                             PrimInfo(numRight,rightBounds.geomBounds,rightBounds.centBounds));
          }, [] (const PrimInfo2& a, const PrimInfo2& b) -> PrimInfo2 { return PrimInfo2::merge(a,b); });

          linfo_o.merge(info.left);
          rinfo_o.merge(info.right);
        }
        
        __forceinline void deterministic_order(const Set& set) // FIXME: implement me, without this trees are not deterministic
        {
          /* required as parallel partition destroys original primitive order */
          //std::sort(&prims[set.begin()],&prims[set.end()]);
        }

        void splitFallback(Set& prims, PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o)
        {
          size_t num = 0;
          BBox3fa lbounds = empty, rbounds = empty;
          PrimRefList::item* lblock = lprims_o.insert(new PrimRefList::item);
          PrimRefList::item* rblock = rprims_o.insert(new PrimRefList::item);
          linfo_o.reset();
          rinfo_o.reset();
          
          while (PrimRefList::item* block = prims.take()) 
          {
            for (size_t i=0; i<block->size(); i++) 
            {
              const PrimRef& prim = block->at(i); 
              const BBox3fa bounds = prim.bounds();
              
              if ((num++)%2) 
              {
                linfo_o.add(bounds,prim.center2()); 
                if (likely(lblock->insert(prim))) continue; 
                lblock = lprims_o.insert(new PrimRefList::item);
                lblock->insert(prim);
              } 
              else 
              {
                rinfo_o.add(bounds,prim.center2()); 
                if (likely(rblock->insert(prim))) continue;
                rblock = rprims_o.insert(new PrimRefList::item);
                rblock->insert(prim);
              }
            }
            delete block;
          }
        }
      };
  }
}
