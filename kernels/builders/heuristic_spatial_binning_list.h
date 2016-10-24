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
#include "primrefblock.h"

namespace embree
{
  namespace isa
  {
    template<typename ObjectSplit, typename SpatialSplit>
      struct Split2
      {
        __forceinline Split2 () {}
        
        __forceinline Split2 (const Split2& other) 
        {
          spatial = other.spatial;
          sah = other.sah;
          if (spatial) spatialSplit() = other.spatialSplit();
          else         objectSplit()  = other.objectSplit();
        }
        
        __forceinline Split2& operator= (const Split2& other) 
        {
          spatial = other.spatial;
          sah = other.sah;
          if (spatial) spatialSplit() = other.spatialSplit();
          else         objectSplit()  = other.objectSplit();
          return *this;
        }
          
          __forceinline     ObjectSplit&  objectSplit()        { return *(      ObjectSplit*)data; }
        __forceinline const ObjectSplit&  objectSplit() const  { return *(const ObjectSplit*)data; }
        
        __forceinline       SpatialSplit& spatialSplit()       { return *(      SpatialSplit*)data; }
        __forceinline const SpatialSplit& spatialSplit() const { return *(const SpatialSplit*)data; }
        
        __forceinline Split2 (const ObjectSplit& objectSplit, float sah)
          : spatial(false), sah(sah) 
        {
          new (data) ObjectSplit(objectSplit);
        }
        
        __forceinline Split2 (const SpatialSplit& spatialSplit, float sah)
          : spatial(true), sah(sah) 
        {
          new (data) SpatialSplit(spatialSplit);
        }
        
        __forceinline float splitSAH() const { 
          return sah; 
        }
        
      public:
        bool spatial;
        float sah;
        __aligned(16) char data[sizeof(ObjectSplit) > sizeof(SpatialSplit) ? sizeof(ObjectSplit) : sizeof(SpatialSplit)];
        /*union {
          ObjectSplit objectSplit;
          SpatialSplit spatialSplit;
          };*/
      };

    /*! Performs standard object binning */
    template<typename PrimRef, typename SplitPrimitive, size_t OBINS, size_t SBINS>
      struct HeuristicObjectSplitAndSpatialSplitBlockListBinningSAH
      {
        typedef BinSplit<OBINS> ObjectSplit;
        typedef BinInfo<OBINS,PrimRef> ObjectBinner;

        typedef SpatialBinSplit<SBINS> SpatialSplit;
        typedef SpatialBinInfo<SBINS,PrimRef> SpatialBinner;

        typedef atomic_set<PrimRefBlockT<PrimRef> > Set;
        typedef typename atomic_set<PrimRefBlockT<PrimRef> >::item Set_item;
        typedef Split2<ObjectSplit,SpatialSplit> Split;
        
        __forceinline HeuristicObjectSplitAndSpatialSplitBlockListBinningSAH () {
        }

        /*! remember scene for later splits */
        __forceinline HeuristicObjectSplitAndSpatialSplitBlockListBinningSAH (const SplitPrimitive& splitPrimitive) 
          : splitPrimitive(splitPrimitive) {}
        
        /*! finds the best split */
        const Split find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          SplitInfo oinfo;
          const ObjectSplit objectSplit  = object_find(set,pinfo,logBlockSize,oinfo);
          const float objectSplitSAH = objectSplit.splitSAH();

          const BBox3fa overlap = intersect(oinfo.leftBounds,oinfo.rightBounds);
          if (safeArea(overlap) < 0.2f*safeArea(pinfo.geomBounds)) 
            return Split(objectSplit,objectSplitSAH);

          const SpatialSplit spatialSplit = spatial_find(set,pinfo,logBlockSize);
          const float spatialSplitSAH = spatialSplit.splitSAH();
          if (objectSplitSAH < spatialSplitSAH) return Split(objectSplit,objectSplitSAH);
          else                                  return Split(spatialSplit,spatialSplitSAH);
        }

        /*! finds the best split */
        const ObjectSplit object_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo& sinfo_o)
        {
          if (likely(pinfo.size() < 10000)) return object_sequential_find(set,pinfo,logBlockSize,sinfo_o);
          else                              return   object_parallel_find(set,pinfo,logBlockSize,sinfo_o);
        }
        
        /*! finds the best split */
        const ObjectSplit object_sequential_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo& sinfo_o)
        {
          ObjectBinner binner(empty);
          const BinMapping<OBINS> mapping(pinfo);
          typename Set::iterator i=set;
          while (typename Set::item* block = i.next()) {
            binner.bin(block->base(),block->size(),mapping);
          }
          const ObjectSplit split = binner.best(mapping,logBlockSize);
          binner.getSplitInfo(mapping,split,sinfo_o);
          return split;
        }

        /*! finds the best split */
        const ObjectSplit object_parallel_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo& sinfo_o)
        {
          const BinMapping<OBINS> mapping(pinfo);
          const BinMapping<OBINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround
          typename Set::iterator i=set;
          const size_t threadCount = TaskScheduler::threadCount();
          const ObjectBinner binner = parallel_reduce(size_t(0),threadCount,ObjectBinner(empty), [&] (const range<size_t>& r) -> ObjectBinner
          {
            ObjectBinner binner(empty);
            while (Set_item* block = i.next()) {
              binner.bin(block->base(),block->size(),_mapping);
            }
            return binner;
          },[](const ObjectBinner& a, const ObjectBinner& b) -> ObjectBinner { return ObjectBinner::reduce(a,b); });
          
          const ObjectSplit split = binner.best(mapping,logBlockSize);
          binner.getSplitInfo(mapping,split,sinfo_o);
          return split;
        }

        /*! finds the best split */
        const SpatialSplit spatial_find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          if (likely(pinfo.size() < 10000)) return spatial_sequential_find(set,pinfo,logBlockSize);
          else                              return   spatial_parallel_find(set,pinfo,logBlockSize);		  
        }
        
        /*! finds the best split */
        const SpatialSplit spatial_sequential_find(Set& prims, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          SpatialBinner binner(empty);
          const SpatialBinMapping<SBINS> mapping(pinfo);
          PrimRefList::iterator i=prims;
          while (PrimRefList::item* block = i.next())
            binner.bin(splitPrimitive,block->base(),block->size(),mapping);
          return binner.best(pinfo,mapping,logBlockSize);
        }

        /*! finds the best split */
        const SpatialSplit spatial_parallel_find(Set& prims, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          const SpatialBinMapping<SBINS> mapping(pinfo);
          const SpatialBinMapping<SBINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround
          PrimRefList::iterator i=prims;

          const size_t threadCount = TaskScheduler::threadCount();
          const SpatialBinner binner = parallel_reduce(size_t(0),threadCount,SpatialBinner(empty), [&] (const range<size_t>& r) -> SpatialBinner 
          {
            SpatialBinner binner(empty);
            while (PrimRefList::item* block = i.next()) {
              binner.bin(splitPrimitive,block->base(),block->size(),_mapping);
            }
            return binner;
          },[](const SpatialBinner& a, const SpatialBinner& b) -> SpatialBinner { return SpatialBinner::reduce(a,b); });
          
          return binner.best(pinfo,mapping,logBlockSize);
        }

        /*! splits a list of primitives */
        void split(const Split& split, const PrimInfo& pinfo, Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (split.spatial) return spatial_split(split.spatialSplit(),pinfo,set,left,lset,right,rset);
          else               return  object_split(split.objectSplit() ,pinfo,set,left,lset,right,rset);
        }
        
        /*! splits a list of primitives */
        void object_split(const ObjectSplit& split, const PrimInfo& pinfo, Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (likely(pinfo.size() < 10000)) object_sequential_split(split,set,left,lset,right,rset);
          else                                object_parallel_split(split,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void object_sequential_split(const ObjectSplit& split, Set& prims, PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
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
              const vint4 bin = vint4(split.mapping.bin_unsafe(center));
              
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
        void object_parallel_split(const ObjectSplit& split, Set& prims, PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
        {
          if (!split.valid()) {
            deterministic_order(prims);
            return splitFallback(prims,linfo_o,lprims_o,rinfo_o,rprims_o);
          }
                    
          linfo_o.reset();
          rinfo_o.reset();
          
          const size_t threadCount = TaskScheduler::threadCount();
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
                const vint4 bin = vint4(split.mapping.bin_unsafe(center));
                
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

        /*! splits a list of primitives */
        void spatial_split(const SpatialSplit& split, const PrimInfo& pinfo, Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (likely(pinfo.size() < 10000)) spatial_sequential_split(split,set,left,lset,right,rset);
          else                                spatial_parallel_split(split,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void spatial_sequential_split(const SpatialSplit& split, Set& prims, PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
        {
          if (!split.valid()) {
            deterministic_order(prims);
            return splitFallback(prims,linfo_o,lprims_o,rinfo_o,rprims_o);
          }
          
          PrimRefList::item* lblock = lprims_o.insert(new PrimRefList::item);
          PrimRefList::item* rblock = rprims_o.insert(new PrimRefList::item);
          linfo_o.reset();
          rinfo_o.reset();

          /* sort each primitive to left, right, or left and right */
          while (PrimRefList::item* block = prims.take()) 
          {
            for (size_t i=0; i<block->size(); i++) 
            {
              PrimRef& prim = block->at(i); 
              const BBox3fa bounds = prim.bounds();
              int bin0 = split.mapping.bin(bounds.lower)[split.dim];
              int bin1 = split.mapping.bin(bounds.upper)[split.dim];

              const int splits = prim.geomID() >> 24;
              if (splits == 1) {
                const vint4 bin = split.mapping.bin(center(prim.bounds()));
                bin0 = bin1 = bin[split.dim];
              }

              /* sort to the left side */
              if (bin1 < split.pos)
              {
                linfo_o.add(bounds,center2(bounds));
                if (likely(lblock->insert(prim))) continue; 
                lblock = lprims_o.insert(new PrimRefList::item);
                lblock->insert(prim);
                continue;
              }
              
              /* sort to the right side */
              if (bin0 >= split.pos)
              {
                rinfo_o.add(bounds,center2(bounds));
                if (likely(rblock->insert(prim))) continue;
                rblock = rprims_o.insert(new PrimRefList::item);
                rblock->insert(prim);
                continue;
              }
              
              PrimRef left,right;
              float fpos = split.mapping.pos(split.pos,split.dim);
              splitPrimitive(prim,split.dim,fpos,left,right);
              int lsplits = splits/2, rsplits = lsplits+splits%2;
              
              if (!left.bounds().empty()) 
              {
                left.lower.a = (left.lower.a & 0x00FFFFFF) | (lsplits << 24);
                
                linfo_o.add(left.bounds(),center2(left.bounds()));
                if (!lblock->insert(left)) {
                  lblock = lprims_o.insert(new PrimRefList::item);
                  lblock->insert(left);
                }
              }
              
              if (!right.bounds().empty()) 
              {
                right.lower.a = (right.lower.a & 0x00FFFFFF) | (rsplits << 24);

                rinfo_o.add(right.bounds(),center2(right.bounds()));
                if (!rblock->insert(right)) {
                  rblock = rprims_o.insert(new PrimRefList::item);
                  rblock->insert(right);
                }
              }
            }
            delete block;
          }
        }

        /*! array partitioning */
        void spatial_parallel_split(const SpatialSplit& split, Set& prims, PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
        {
          if (!split.valid()) {
            deterministic_order(prims);
            return splitFallback(prims,linfo_o,lprims_o,rinfo_o,rprims_o);
          }

          linfo_o.reset();
          rinfo_o.reset();

          const size_t threadCount = TaskScheduler::threadCount();
          const PrimInfo2 info = parallel_reduce(size_t(0),threadCount,PrimInfo2(empty), [&] (const range<size_t>& r) -> PrimInfo2
          {
            PrimInfo linfo(empty);
            PrimInfo rinfo(empty);
            PrimRefList::item* lblock = nullptr;
            PrimRefList::item* rblock = nullptr;

            /* sort each primitive to left, right, or left and right */
            while (PrimRefList::item* block = prims.take()) 
            {
              if (lblock == nullptr) lblock = lprims_o.insert(new PrimRefList::item);
              if (rblock == nullptr) rblock = rprims_o.insert(new PrimRefList::item);
              
              for (size_t i=0; i<block->size(); i++) 
              {
                PrimRef& prim = block->at(i); 
                const BBox3fa bounds = prim.bounds();
                int bin0 = split.mapping.bin(bounds.lower)[split.dim];
                int bin1 = split.mapping.bin(bounds.upper)[split.dim];
                
                const int splits = prim.geomID() >> 24;
                if (splits == 1) {
                  const vint4 bin = split.mapping.bin(center(prim.bounds()));
                  bin0 = bin1 = bin[split.dim];
                }
                
                /* sort to the left side */
                if (bin1 < split.pos)
                {
                  linfo.add(bounds,center2(bounds));
                  if (likely(lblock->insert(prim))) continue; 
                  lblock = lprims_o.insert(new PrimRefList::item);
                  lblock->insert(prim);
                  continue;
                }
                
                /* sort to the right side */
                if (bin0 >= split.pos)
                {
                  rinfo.add(bounds,center2(bounds));
                  if (likely(rblock->insert(prim))) continue;
                  rblock = rprims_o.insert(new PrimRefList::item);
                  rblock->insert(prim);
                  continue;
                }
                
                PrimRef left,right;
                float fpos = split.mapping.pos(split.pos,split.dim);
                splitPrimitive(prim,split.dim,fpos,left,right);
                int lsplits = splits/2, rsplits = lsplits+splits%2;
                
                if (!left.bounds().empty()) 
                {
                  left.lower.a = (left.lower.a & 0x00FFFFFF) | (lsplits << 24);
                  
                  linfo.add(left.bounds(),center2(left.bounds()));
                  if (!lblock->insert(left)) {
                    lblock = lprims_o.insert(new PrimRefList::item);
                    lblock->insert(left);
                  }
                }
                
                if (!right.bounds().empty()) 
                {
                  right.lower.a = (right.lower.a & 0x00FFFFFF) | (rsplits << 24);
                  
                  rinfo.add(right.bounds(),center2(right.bounds()));
                  if (!rblock->insert(right)) {
                    rblock = rprims_o.insert(new PrimRefList::item);
                    rblock->insert(right);
                  }
                }
              }
              delete block;
            }
            return PrimInfo2(linfo,rinfo);
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

      private:
        const SplitPrimitive& splitPrimitive;
      };
  }
}

