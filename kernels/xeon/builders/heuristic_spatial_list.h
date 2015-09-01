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

#include "heuristic_spatial.h"
#include "primrefblock.h"

namespace embree
{
  namespace isa
  {
    /*! Performs standard object binning */
    template<typename SplitPrimitive, typename PrimRef, size_t BINS = 16>
      struct HeuristicSpatialBlockListBinningSAH
      {
        typedef SpatialBinSplit<BINS> Split;
        typedef SpatialBinInfo<BINS,PrimRef> Binner;
        typedef atomic_set<PrimRefBlockT<PrimRef> > Set;
        
        __forceinline HeuristicSpatialBlockListBinningSAH () {}

        /*! remember scene for later splits */
        __forceinline HeuristicSpatialBlockListBinningSAH (const SplitPrimitive& splitPrimitive) 
          : splitPrimitive(splitPrimitive) {}
        
        /*! finds the best split */
        const Split find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          if (likely(pinfo.size() < 10000)) return sequential_find(set,pinfo,logBlockSize);
          else                              return   parallel_find(set,pinfo,logBlockSize);
        }
        
        /*! finds the best split */
        const Split sequential_find(Set& prims, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Binner binner(empty);
          const SpatialBinMapping<BINS> mapping(pinfo);
          PrimRefList::iterator i=prims;
          while (PrimRefList::item* block = i.next())
            binner.bin(splitPrimitive,block->base(),block->size(),pinfo,mapping);
          return binner.best(pinfo,mapping,logBlockSize);
        }

        /*! finds the best split */
        const Split parallel_find(Set& prims, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          const SpatialBinMapping<BINS> mapping(pinfo);
          const SpatialBinMapping<BINS>& _mapping = mapping; // CLANG 3.4 parser bug workaround
          PrimRefList::iterator i=prims;

          const size_t threadCount = TaskSchedulerTBB::threadCount();
          const Binner binner = parallel_reduce(size_t(0),threadCount,Binner(empty), [&] (const range<size_t>& r) -> Binner 
          {
            Binner binner(empty);
            while (PrimRefList::item* block = i.next()) {
              binner.bin(splitPrimitive,block->base(),block->size(),pinfo,_mapping);
            }
            return binner;
          },[](const Binner& a, const Binner& b) -> Binner { return SpatialBinInfo<BINS,PrimRef>::reduce(a,b); });
          
          return binner.best(pinfo,mapping,logBlockSize);
        }
        
        /*! splits a list of primitives */
        void split(const Split& split, const PrimInfo& pinfo, Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (likely(pinfo.size() < 10000)) sequential_split(split,set,left,lset,right,rset);
          else                                parallel_split(split,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void sequential_split(const Split& split, Set& prims, 
                              PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
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
                const int4 bin = split.mapping.bin(center(prim.bounds()));
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
              //assert(prim.geomID() >> 24);

              /* split and sort to left and right */
              //TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID());
              //TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF); // FIXME: hack !!
              //TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
              //const Vec3fa v0 = mesh->vertex(tri.v[0]);
              //const Vec3fa v1 = mesh->vertex(tri.v[1]);
              //const Vec3fa v2 = mesh->vertex(tri.v[2]);
              
              PrimRef left,right;
              float fpos = split.mapping.pos(split.pos,split.dim);
              //splitTriangle(prim,split.dim,fpos,v0,v1,v2,left,right);
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
        void parallel_split(const Split& split, Set& prims, 
                            PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
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
                  const int4 bin = split.mapping.bin(center(prim.bounds()));
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
                //assert(prim.geomID() >> 24);
                
                /* split and sort to left and right */
                //TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID());
                //TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID() & 0x00FFFFFF); // FIXME: hack !!
                //TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
                //const Vec3fa v0 = mesh->vertex(tri.v[0]);
                //const Vec3fa v1 = mesh->vertex(tri.v[1]);
                //const Vec3fa v2 = mesh->vertex(tri.v[2]);
                
                PrimRef left,right;
                float fpos = split.mapping.pos(split.pos,split.dim);
                //splitTriangle(prim,split.dim,fpos,v0,v1,v2,left,right);
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
        
        void deterministic_order(const Set& set) // FIXME: implement me
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

      private:
        const SplitPrimitive& splitPrimitive;
      };
  }
}

