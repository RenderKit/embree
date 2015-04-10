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
      struct UnalignedHeuristicArrayBinningSAH
      {
        typedef BinSplit<BINS> Split;
        typedef BinInfo<BINS,PrimRef> Binner;
        typedef range<size_t> Set;

         /*! computes bounding box of bezier curves for motion blur */
        struct PrimInfoMB 
        {
          PrimInfo pinfo;
          BBox3fa s0t0;
          //BBox3fa s0t1_s1t0;
          BBox3fa s1t1;
        };

        __forceinline UnalignedHeuristicArrayBinningSAH ()
          : prims(nullptr) {}
        
        /*! remember prim array */
        __forceinline UnalignedHeuristicArrayBinningSAH (PrimRef* prims)
          : prims(prims) {}

        const LinearSpace3fa computeAlignedSpace(const PrimInfo& pinfo)
        {
          /*! find first curve that defines valid direction */
          Vec3fa axis(0,0,1);
          for (size_t i=pinfo.begin; i<pinfo.end; i++)
          {
            const Bezier1v& prim = prims[i];
            const Vec3fa axis1 = normalize(prim.p3 - prim.p0);
            if (length(prim.p3 - prim.p0) > 1E-9f) {
              axis = axis1;
              break;
            }
          }
          return frame(axis).transposed();
        }

        const std::pair<AffineSpace3fa,AffineSpace3fa> computeAlignedSpaceMB(Scene* scene, const PrimInfo& pinfo)
        {
          /*! find first curve that defines valid direction */
          Vec3fa p0(0,0,0);
          Vec3fa p1(0,0,0);
          Vec3fa axis0(0,0,1), axisb0(0,1,0);
          Vec3fa axis1(0,0,1), axisb1(0,1,0);
          //Vec3fa axis2(0,1,0);
          for (size_t i=pinfo.begin; i<pinfo.end; i++)
          {
            const Bezier1v& prim = prims[i];
            const size_t geomID = prim.geomID<0>();
            const size_t primID = prim.primID<0>();
            const BezierCurves* curves = scene->getBezierCurves(geomID);
            const int curve = curves->curve(primID);
            
            const Vec3fa a3 = curves->vertex(curve+3,0);
            const Vec3fa a2 = curves->vertex(curve+2,0);
            const Vec3fa a1 = curves->vertex(curve+1,0);
            const Vec3fa a0 = curves->vertex(curve+0,0);
            
            const Vec3fa b3 = curves->vertex(curve+3,1);
            const Vec3fa b2 = curves->vertex(curve+2,1);
            const Vec3fa b1 = curves->vertex(curve+1,1);
            const Vec3fa b0 = curves->vertex(curve+0,1);
            
            if (length(a3 - a0) > 1E-9f && length(a1 - a0) > 1E-9f &&
                length(b3 - b0) > 1E-9f && length(b1 - b0) > 1E-9f) 
            {
              axis0 = normalize(a3 - a0); axisb0 = normalize(a1 - a0); 
              axis1 = normalize(b3 - b0); axisb1 = normalize(b1 - b0); 
              p0 = a0; p1 = b0;
              /*if (length(b3-a3) > 1E-9f) axis2 = b3-a3;
                else if (length(b0-a0) > 1E-9f) axis2 = b0-a0;
                else axis2 = Vec3fa(1,0,0);*/ // FIXME: not correct
              break;
            }
          }
#if 1
          //LinearSpace3fa space01 = frame(0.5f*axis0 + 0.5f*axis1).transposed();
          //AffineSpace3fa space0 = frame(axis0).transposed();
          //AffineSpace3fa space1 = frame(axis1).transposed();
          
          const Vec3fa space0_dx = normalize(axis0);
          const Vec3fa space0_dy = normalize(cross(space0_dx,axisb0));
          const Vec3fa space0_dz = normalize(cross(space0_dx,space0_dy));
          LinearSpace3fa space0(space0_dz,space0_dy,space0_dx);
          space0 = space0.transposed();
          
          const Vec3fa space1_dx = normalize(axis1);
          const Vec3fa space1_dy = normalize(cross(space1_dx,axisb1));
          const Vec3fa space1_dz = normalize(cross(space1_dx,space1_dy));
          LinearSpace3fa space1(space1_dz,space1_dy,space1_dx);
          space1 = space1.transposed();
          
          //space0.p = -xfmVector(space0.l,p0);
          //space1.p = -xfmVector(space1.l,p1);
          //space0 = space01;
          //space1 = space0;
#else
          const Vec3fa space0_dx = normalize(axis0);
          const Vec3fa space0_dy = normalize(cross(space0_dx,axis2));
          const Vec3fa space0_dz = normalize(cross(space0_dx,space0_dy));
          AffineSpace3fa space0(space0_dz,space0_dy,space0_dx,zero);
          space0.l = space0.l.transposed();
          
          const Vec3fa space1_dx = normalize(axis1);
          const Vec3fa space1_dy = normalize(cross(space1_dx,axis2));
          const Vec3fa space1_dz = normalize(cross(space1_dx,space1_dy));
          AffineSpace3fa space1(space1_dz,space1_dy,space1_dx,zero);
          space1.l = space1.l.transposed();
          
          space0.p = -xfmVector(space0.l,p0);
          space1.p = -xfmVector(space1.l,p1);
#endif 
          
          return std::pair<AffineSpace3fa,AffineSpace3fa>(space0,space1);
        }
        
        const PrimInfo computePrimInfo(const PrimInfo& pinfo, const LinearSpace3fa& space)
        {
          BBox3fa geomBounds = empty;
          BBox3fa centBounds = empty;
          for (size_t i=pinfo.begin; i<pinfo.end; i++) { // FIXME: parallel
            const BBox3fa bounds = prims[i].bounds(space);
            geomBounds.extend(bounds);
            centBounds.extend(center2(bounds));
          }
          return PrimInfo(pinfo.begin,pinfo.end,geomBounds,centBounds);
        }
        
        const PrimInfoMB computePrimInfoMB(Scene* scene, const PrimInfo& pinfo, const AffineSpace3fa& space)
        {
          size_t N = 0;
          BBox3fa centBounds = empty;
          BBox3fa geomBounds = empty;
          BBox3fa s0t0 = empty, s1t1 = empty;
          for (size_t i=pinfo.begin; i<pinfo.end; i++)  // FIXME: parallelize
          {
            const Bezier1v& prim = prims[i];
            const size_t geomID = prim.geomID<0>();
            const size_t primID = prim.primID<0>();

            N++;
            const BBox3fa bounds = prim.bounds(space);
            geomBounds.extend(bounds);
            centBounds.extend(center2(bounds));

            const BezierCurves* curves = scene->getBezierCurves(prim.geomID<0>());
            s0t0.extend(curves->bounds(space,prim.primID<0>(),0));
            s1t1.extend(curves->bounds(space,prim.primID<0>(),1));
          }
          
          PrimInfoMB ret;
          ret.pinfo = PrimInfo(N,geomBounds,centBounds);
          ret.s0t0 = s0t0;
          ret.s1t1 = s1t1;
          return ret;
        }
        
        /*! finds the best split */
        const Split find(const PrimInfo& pinfo, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          Set set(pinfo.begin,pinfo.end);
          if (likely(pinfo.size() < 10000)) return sequential_find(set,pinfo,logBlockSize,space);
          else                              return   parallel_find(set,pinfo,logBlockSize,space);
        }
        
        /*! finds the best split */
        const Split find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          if (likely(pinfo.size() < 10000)) return sequential_find(set,pinfo,logBlockSize,space);
          else                              return   parallel_find(set,pinfo,logBlockSize,space);
        }

        /*! finds the best split */
        const Split sequential_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          binner.bin(prims,set.begin(),set.end(),mapping,space);
          return binner.best(mapping,logBlockSize);
        }
        
        /*! finds the best split */
        const Split parallel_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          binner = parallel_reduce(set.begin(),set.end(),size_t(4096),binner,
                                   [&] (const range<size_t>& r) -> Binner { Binner binner(empty); binner.bin(prims+r.begin(),r.size(),mapping,space); return binner; },
                                   [&] (const Binner& b0, const Binner& b1) -> Binner { Binner r = b0; r.merge(b1,mapping.size()); return r; });
          return binner.best(mapping,logBlockSize);
        }
        
        /*! array partitioning */
        void split(const Split& spliti, const LinearSpace3fa& space, const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right) 
        {
          Set lset,rset;
          Set set(pinfo.begin,pinfo.end);
          if (likely(pinfo.size() < 10000)) sequential_split(spliti,space,set,left,lset,right,rset);
          else                                parallel_split(spliti,space,set,left,lset,right,rset);
        }

        /*! array partitioning */
        void split(const Split& split, const LinearSpace3fa& space, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (likely(set.size() < 10000)) sequential_split(split,space,set,left,lset,right,rset);
          else                              parallel_split(split,space,set,left,lset,right,rset);
        }
        
        /*! array partitioning */
        void sequential_split(const Split& split, const LinearSpace3fa& space, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
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
          size_t center = serial_partitioning(prims,begin,end,local_left,local_right,
                                              [&] (const PrimRef& ref) { return split.mapping.bin_unsafe(center2(ref.bounds(space)))[splitDim] < splitPos; },
                                              [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); });
          
          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) range<size_t>(begin,center);
          new (&rset) range<size_t>(center,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
        }
        
        /*! array partitioning */
        void parallel_split(const Split& split, const LinearSpace3fa& space, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
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
          
          const size_t mid = parallel_in_place_partitioning<128,PrimRef,PrimInfo>
	  (&prims[begin],end-begin,init,left,right,
	   [&] (const PrimRef &ref) { return split.mapping.bin_unsafe(center2(ref.bounds(space)))[splitDim] < splitPos; },
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
