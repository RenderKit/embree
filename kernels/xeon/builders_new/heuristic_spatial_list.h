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
#include "builders/primrefblock.h"

namespace embree
{
  namespace isa
  {
    /*! Performs standard object binning */
    template<typename PrimRef, size_t BINS = 16>
      struct HeuristicSpatialBlockListBinningSAH
      {
        typedef SpatialBinSplit<BINS> Split;
        typedef SpatialBinInfo<BINS,PrimRef> Binner;
        typedef atomic_set<PrimRefBlockT<PrimRef> > Set;
        
        __forceinline HeuristicSpatialBlockListBinningSAH () {}

        /*! remember scene for later splits */
        __forceinline HeuristicSpatialBlockListBinningSAH (Scene* scene) 
          : scene(scene) {}
        
        /*! finds the best split */
        const Split find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          //if (likely(pinfo.size() < 10000)) // FIXME: implement parallel code path
            return sequential_find(set,pinfo,logBlockSize);
            //else                              return   parallel_find(set,pinfo,logBlockSize);
        }
        
        /*! finds the best split */
        const Split sequential_find(Set& prims, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Binner binner(empty);
          const SpatialBinMapping<BINS> mapping(pinfo);
          PrimRefList::iterator i=prims;
          while (PrimRefList::item* block = i.next())
            binner.bin(scene,block->base(),block->size(),pinfo,mapping);
          return binner.best(pinfo,mapping,logBlockSize);
        }
        
        /*! splits a list of primitives */
        void split(const Split& split, const PrimInfo& pinfo, Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          //if (likely(pinfo.size() < 10000)) // FIXME: implement parallel code path
            sequential_split(split,set,left,lset,right,rset);
            //else                                parallel_split(split,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void sequential_split(const Split& split, Set& prims, 
                              PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) 
        {
          if (!split.valid()) {
            //deterministic_order(set); // FIXME: enable
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
              const PrimRef& prim = block->at(i); 
              const BBox3fa bounds = prim.bounds();
              const int bin0 = split.mapping.bin(bounds.lower)[split.dim];
              const int bin1 = split.mapping.bin(bounds.upper)[split.dim];
              
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
              
              /* split and sort to left and right */
              TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID());
              TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
              const Vec3fa v0 = mesh->vertex(tri.v[0]);
              const Vec3fa v1 = mesh->vertex(tri.v[1]);
              const Vec3fa v2 = mesh->vertex(tri.v[2]);
              
              PrimRef left,right;
              float fpos = split.mapping.pos(split.pos,split.dim);
              splitTriangle(prim,split.dim,fpos,v0,v1,v2,left,right);
              
              if (!left.bounds().empty()) {
                linfo_o.add(left.bounds(),center2(left.bounds()));
                if (!lblock->insert(left)) {
                  lblock = lprims_o.insert(new PrimRefList::item);
                  lblock->insert(left);
                }
              }
              
              if (!right.bounds().empty()) {
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
        
        //void deterministic_order(const Set& set) 
        //{
        /* required as parallel partition destroys original primitive order */
        //std::sort(&prims[set.begin()],&prims[set.end()]);
        //}

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
        Scene* scene;
      };
  }
}

