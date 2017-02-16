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

#pragma once

#include "heuristic_binning.h"

namespace embree
{
  namespace isa
  { 
    /*! Performs standard object binning */
    template<typename PrimRef, size_t BINS>
      struct UnalignedHeuristicArrayBinningSAH
      {
        typedef BinSplit<BINS> Split;
        typedef BinInfoT<BINS,PrimRef,BBox3fa> Binner;
        typedef range<size_t> Set;

         /*! computes bounding box of bezier curves for motion blur */
        struct PrimInfoMB 
        {
          PrimInfo pinfo;
          BBox3fa s0t0;
          BBox3fa s1t1;
        };

        __forceinline UnalignedHeuristicArrayBinningSAH ()
          : prims(nullptr) {}
        
        /*! remember prim array */
        __forceinline UnalignedHeuristicArrayBinningSAH (PrimRef* prims)
          : prims(prims) {}

        const LinearSpace3fa computeAlignedSpace(const range<size_t>& set)
        {
          /*! find first curve that defines valid direction */
          Vec3fa axis(0,0,1);
          for (size_t i=set.begin(); i<set.end(); i++)
          {
            const BezierPrim& prim = prims[i];
            const Vec3fa axis1 = normalize(prim.p3 - prim.p0);
            if (sqr_length(prim.p3 - prim.p0) > 1E-18f) {
              axis = axis1;
              break;
            }
          }
          return frame(axis).transposed();
        }

        const AffineSpace3fa computeAlignedSpaceMB(Scene* scene, const range<size_t>& pinfo)
        {
          /*! find first curve that defines valid directions */
          Vec3fa axis0(0,0,1);
          Vec3fa axis1(0,0,1);

          for (size_t i=pinfo.begin(); i<pinfo.end(); i++)
          {
            const BezierPrim& prim = prims[i];
            const size_t geomID = prim.geomID();
            const size_t primID = prim.primID();
            const BezierCurves* curves = scene->getBezierCurves(geomID);
            const int curve = curves->curve(primID);
            
            const Vec3fa a3 = curves->vertex(curve+3,0);
            //const Vec3fa a2 = curves->vertex(curve+2,0);
            //const Vec3fa a1 = curves->vertex(curve+1,0);
            const Vec3fa a0 = curves->vertex(curve+0,0);
            
            const Vec3fa b3 = curves->vertex(curve+3,1);
            //const Vec3fa b2 = curves->vertex(curve+2,1);
            //const Vec3fa b1 = curves->vertex(curve+1,1);
            const Vec3fa b0 = curves->vertex(curve+0,1);
            
            if (sqr_length(a3 - a0) > 1E-18f && sqr_length(b3 - b0) > 1E-18f)
            {
              axis0 = normalize(a3 - a0);
              axis1 = normalize(b3 - b0);
              break;
            }
          }

          Vec3fa axis01 = axis0+axis1;
          if (sqr_length(axis01) < 1E-18f) axis01 = axis0;
          axis01 = normalize(axis01);
          return frame(axis01).transposed();
        }
        
        const PrimInfo computePrimInfo(const range<size_t>& set, const LinearSpace3fa& space)
        {
          auto computeBounds = [&](const range<size_t>& r) -> CentGeomBBox3fa
            {
              CentGeomBBox3fa bounds(empty);
              for (size_t i=r.begin(); i<r.end(); i++)
                bounds.extend(prims[i].bounds(space));
              return bounds;
            };
          
          const CentGeomBBox3fa bounds = parallel_reduce(set.begin(), set.end(), size_t(1024), size_t(4096), 
                                                         CentGeomBBox3fa(empty), computeBounds, CentGeomBBox3fa::merge2);

          return PrimInfo(set.begin(),set.end(),bounds.geomBounds,bounds.centBounds);
        }
        
        const PrimInfoMB computePrimInfoMB(size_t timeSegment, size_t numTimeSteps, Scene* scene, const range<size_t>& pinfo, const AffineSpace3fa& space)
        {
          size_t N = 0;
          BBox3fa centBounds = empty;
          BBox3fa geomBounds = empty;
          BBox3fa s0t0 = empty, s1t1 = empty;
          for (size_t i=pinfo.begin(); i<pinfo.end(); i++)  // FIXME: parallelize
          {
            const BezierPrim& prim = prims[i];
            const size_t geomID = prim.geomID();
            const size_t primID = prim.primID();

            N++;
            const BBox3fa bounds = prim.bounds(space);
            geomBounds.extend(bounds);
            centBounds.extend(center2(bounds));

            const BezierCurves* curves = scene->getBezierCurves(geomID);
            const LBBox3fa linearBounds = curves->linearBounds(space,primID,timeSegment,numTimeSteps);
            s0t0.extend(linearBounds.bounds0);
            s1t1.extend(linearBounds.bounds1);
          }
          
          PrimInfoMB ret;
          ret.pinfo = PrimInfo(N,geomBounds,centBounds);
          ret.s0t0 = s0t0;
          ret.s1t1 = s1t1;
          return ret;
        }
        
        /*! finds the best split */
        __forceinline const Split find(const PrimInfoRange& pinfo, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          if (likely(pinfo.size() < 10000))
            return find_template<false>(pinfo,logBlockSize,space);
          else
            return find_template<true>(pinfo,logBlockSize,space);
        }

        /*! finds the best split */
        template<bool parallel>
        const Split find_template(const PrimInfoRange& set, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(set);
          bin_serial_or_parallel<parallel>(binner,prims,set.begin(),set.end(),size_t(4096),mapping,space);
          return binner.best(mapping,logBlockSize);
        }
        
        /*! array partitioning */
        __forceinline void split(const Split& split, const LinearSpace3fa& space, const Set& set, PrimInfoRange& lset, PrimInfoRange& rset)
        {
          if (likely(set.size() < 10000))
            split_template<false>(split,space,set,lset,rset);
          else
            split_template<true>(split,space,set,lset,rset);
        }

        /*! array partitioning */
        template<bool parallel>
        __forceinline void split_template(const Split& split, const LinearSpace3fa& space, const Set& set, PrimInfoRange& lset, PrimInfoRange& rset)
        {
          if (!split.valid()) {
            deterministic_order(set);
            return splitFallback(set,lset,rset);
          }
          
          const size_t begin = set.begin();
          const size_t end   = set.end();
          CentGeomBBox3fa local_left(empty);
          CentGeomBBox3fa local_right(empty);
          const int splitPos = split.pos;
          const int splitDim = split.dim;

          size_t center = 0;
          if (likely(set.size() < 10000))
            center = serial_partitioning(prims,begin,end,local_left,local_right,
                                         [&] (const PrimRef& ref) { return split.mapping.bin_unsafe(center2(ref.bounds(space)))[splitDim] < splitPos; },
                                         [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); });
          else
            center = parallel_partitioning(prims,begin,end,EmptyTy(),local_left,local_right,
                                           [&] (const PrimRef& ref) { return split.mapping.bin_unsafe(center2(ref.bounds(space)))[splitDim] < splitPos; },
                                           [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); },
                                           [] (CentGeomBBox3fa& pinfo0,const CentGeomBBox3fa& pinfo1) { pinfo0.merge(pinfo1); },
                                           128);
          
          new (&lset) PrimInfoRange(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&rset) PrimInfoRange(center,end,local_right.geomBounds,local_right.centBounds);
          assert(area(lset.geomBounds) >= 0.0f);
          assert(area(rset.geomBounds) >= 0.0f);
        }
        
        void deterministic_order(const range<size_t>& set) 
        {
          /* required as parallel partition destroys original primitive order */
          std::sort(&prims[set.begin()],&prims[set.end()]);
        }
        
        void splitFallback(const range<size_t>& set, PrimInfoRange& lset, PrimInfoRange& rset)
        {
          const size_t begin = set.begin();
          const size_t end   = set.end();
          const size_t center = (begin + end)/2;
          
          CentGeomBBox3fa left; left.reset();
          for (size_t i=begin; i<center; i++)
            left.extend(prims[i].bounds());
          new (&lset) PrimInfoRange(begin,center,left.geomBounds,left.centBounds);
          
          CentGeomBBox3fa right; right.reset();
          for (size_t i=center; i<end; i++)
            right.extend(prims[i].bounds());	
          new (&rset) PrimInfoRange(center,end,right.geomBounds,right.centBounds);
        }
        
      private:
        PrimRef* const prims;
      };
  }
}
