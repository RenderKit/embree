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

#include "builders/priminfo.h"
#include "geometry/bezier1v.h"

#include "algorithms/parallel_reduce.h"
#include "algorithms/parallel_partition.h"

namespace embree
{
  namespace isa
  { 
    /*! Performs standard object binning */
    struct HeuristicStrandSplit
    {
      typedef range<size_t> Set;
  
      static const size_t PARALLEL_THRESHOLD = 10000;
      static const size_t PARALLEL_FIND_BLOCK_SIZE = 4096;
      static const size_t PARALLEL_PARITION_BLOCK_SIZE = 64;

      /*! stores all information to perform some split */
      struct Split
      {    
	/*! construct an invalid split by default */
	__forceinline Split()
	  : sah(inf), axis0(zero), axis1(zero) {}
	
	/*! constructs specified split */
	__forceinline Split(const float sah, const Vec3fa& axis0, const Vec3fa& axis1)
	  : sah(sah), axis0(axis0), axis1(axis1) {}
	
	/*! calculates standard surface area heuristic for the split */
	__forceinline float splitSAH() const { return sah; }

        /*! test if this split is valid */
        __forceinline bool valid() const { return sah != float(inf); }
		
      public:
	float sah;             //!< SAH cost of the split
	Vec3fa axis0, axis1;   //!< axis the two strands are aligned into
      };

      __forceinline HeuristicStrandSplit ()
        : prims(NULL) {}
      
      /*! remember prim array */
      __forceinline HeuristicStrandSplit (BezierPrim* prims)
        : prims(prims) {}
      
      /*! finds the best split */
      const Split find(const PrimInfo& pinfo)
      {
        Set set(pinfo.begin,pinfo.end);
        //if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
          return sequential_find(set,pinfo);
          //else                                           return   parallel_find(set,pinfo);
      }
      
      /*! finds the best split */
      const Split sequential_find(const Set& set, const PrimInfo& pinfo)
      {
        /* first curve determines first axis */
        Vec3fa axis0 = normalize(prims[set.begin()].p3 - prims[set.begin()].p0);
      
        /* find 2nd axis that is most misaligned with first axis */
        float bestCos = 1.0f;
        Vec3fa axis1 = axis0;
        for (size_t i=set.begin(); i<set.end(); i++) 
        {
          Vec3fa axisi = prims[i].p3 - prims[i].p0;
          float leni = length(axisi);
          if (leni == 0.0f) continue;
          axisi /= leni;
          float cos = abs(dot(axisi,axis0));
          if (cos < bestCos) { bestCos = cos; axis1 = axisi; }
        }
      
        /* partition the two strands */
        size_t lnum = 0, rnum = 0;
        BBox3fa lbounds = empty, rbounds = empty;
        const LinearSpace3fa space0 = frame(axis0).transposed();
        const LinearSpace3fa space1 = frame(axis1).transposed();
        
        for (size_t i=set.begin(); i<set.end(); i++)
        {
          BezierPrim& prim = prims[i];
          const Vec3fa axisi = normalize(prim.p3-prim.p0);
          const float cos0 = abs(dot(axisi,axis0));
          const float cos1 = abs(dot(axisi,axis1));
          
          if (cos0 > cos1) { lnum++; lbounds.extend(prim.bounds(space0)); }
          else             { rnum++; rbounds.extend(prim.bounds(space1)); }
        }
      
        /*! return an invalid split if we do not partition */
        if (lnum == 0 || rnum == 0) 
          return Split(inf,axis0,axis1);
      
        /*! calculate sah for the split */
        const float sah = float(lnum)*halfArea(lbounds) + float(rnum)*halfArea(rbounds);
        return Split(sah,axis0,axis1);
      }
      
      /*! array partitioning */
      void split(const Split& spliti, const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right) 
      {
        Set lset,rset;
        Set set(pinfo.begin,pinfo.end);
        //if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
          sequential_split(spliti,set,left,lset,right,rset);
          //else
          //parallel_split(spliti,set,left,lset,right,rset);
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
        
        size_t center = serial_partitioning(prims,begin,end,local_left,local_right,
                                            [&] (const BezierPrim& prim) { 
                                              const Vec3fa axisi = normalize(prim.p3-prim.p0);
                                              const float cos0 = abs(dot(axisi,split.axis0));
                                              const float cos1 = abs(dot(axisi,split.axis1));
                                              return cos0 > cos1;
                                            },
                                            [] (CentGeomBBox3fa& pinfo,const BezierPrim& ref) { pinfo.extend(ref.bounds()); });
        
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
      BezierPrim* const prims;
    };
  }
}
