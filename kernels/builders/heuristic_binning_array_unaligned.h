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

        const LinearSpace3fa computeAlignedSpace(const PrimInfo& pinfo)
        {
          /*! find first curve that defines valid direction */
          Vec3fa axis(0,0,1);
          for (size_t i=pinfo.begin; i<pinfo.end; i++)
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

        const AffineSpace3fa computeAlignedSpaceMB(Scene* scene, const PrimInfo& pinfo)
        {
          /*! find first curve that defines valid directions */
          Vec3fa axis0(0,0,1);
          Vec3fa axis1(0,0,1);

          for (size_t i=pinfo.begin; i<pinfo.end; i++)
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
        
        const PrimInfoMB computePrimInfoMB(size_t timeSegment, size_t numTimeSteps, Scene* scene, const PrimInfo& pinfo, const AffineSpace3fa& space)
        {
          size_t N = 0;
          BBox3fa centBounds = empty;
          BBox3fa geomBounds = empty;
          BBox3fa s0t0 = empty, s1t1 = empty;
          for (size_t i=pinfo.begin; i<pinfo.end; i++)  // FIXME: parallelize
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
        const Split find(const PrimInfo& pinfo, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          Set set(pinfo.begin,pinfo.end);
          return find(set,pinfo,logBlockSize,space);
        }
        
        /*! finds the best split */
        __forceinline const Split find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          if (likely(pinfo.size() < 10000))
            return find_template<false>(set,pinfo,logBlockSize,space);
          else
            return find_template<true>(set,pinfo,logBlockSize,space);
        }

        /*! finds the best split */
        template<bool parallel>
        const Split find_template(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          binner.template bin_serial_or_parallel<parallel>(prims,set.begin(),set.end(),size_t(4096),mapping,space);
          return binner.best(mapping,logBlockSize);
        }
        
        /*! array partitioning */
        void split(const Split& spliti, const LinearSpace3fa& space, const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right) 
        {
          Set lset,rset;
          Set set(pinfo.begin,pinfo.end);
          split(spliti,space,set,left,lset,right,rset);
        }

        /*! array partitioning */
        __forceinline void split(const Split& split, const LinearSpace3fa& space, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
        {
          if (likely(set.size() < 10000))
            split_template<false>(split,space,set,left,lset,right,rset);
          else
            split_template<true>(split,space,set,left,lset,right,rset);
        }

        /*! array partitioning */
        template<bool parallel>
        __forceinline void split_template(const Split& split, const LinearSpace3fa& space, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
        {
          if (!split.valid()) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset);
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

    /*! Performs standard object binning */
    template<typename PrimRefMB, size_t BINS>
      struct UnalignedHeuristicArrayBinningMB
      {
        typedef BinSplit<BINS> Split;
        typedef typename PrimRefMB::BBox BBox;
        typedef BinInfoT<BINS,PrimRefMB,BBox> ObjectBinner;

        static const size_t PARALLEL_THRESHOLD = 3 * 1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 128;

        UnalignedHeuristicArrayBinningMB(Scene* scene)
        : scene(scene) {}

        const LinearSpace3fa computeAlignedSpaceMB(Scene* scene, const SetMB& set)
        {
          /*! find first curve that defines valid directions */
          Vec3fa axis0(0,0,1);
          Vec3fa axis1(0,0,1);

          for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++)
          {
            const PrimRefMB& prim = (*set.prims)[i];
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

        const SetMB computePrimInfoMB(Scene* scene, const SetMB& set, const AffineSpace3fa& space) // FIXME: required
        {
          PrimInfoMB ret(empty);
          for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++)  // FIXME: parallelize
          {
            const PrimRefMB& prim = (*set.prims)[i];
            const size_t geomID = prim.geomID();
            const size_t primID = prim.primID();
            const BezierCurves* mesh = (BezierCurves*)scene->get(geomID);
            const LBBox3fa lbounds = mesh->linearBounds(space, primID, set.time_range);
            const unsigned num_time_segments = mesh->numTimeSegments();
            const range<int> tbounds = getTimeSegmentRange(set.time_range, num_time_segments);
            assert(tbounds.size() > 0);
            const PrimRefMB prim2(lbounds, tbounds.size(), num_time_segments, geomID, primID);
            ret.add_primref(prim2);
          }
          return SetMB(ret,set.prims,set.object_range,set.time_range);
        }

        const LBBox3fa linearBounds(Scene* scene, const SetMB& set, const AffineSpace3fa& space)
        {
          LBBox3fa lbounds(empty);
          for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++)  // FIXME: parallelize
          {
            const PrimRefMB& prim = (*set.prims)[i];
            const size_t geomID = prim.geomID();
            const size_t primID = prim.primID();
            const BezierCurves* mesh = (BezierCurves*)scene->get(geomID);
            lbounds.extend(mesh->linearBounds(space, primID, set.time_range));
          }
          return lbounds;
        }

        /*! finds the best split */
        const Split find(const SetMB& set, const size_t logBlockSize, const LinearSpace3fa& space)
        {
          UserPrimRefData user(scene,set.time_range);
          ObjectBinner binner(empty); // FIXME: this clear can be optimized away
          const BinMapping<BINS> mapping(set.pinfo.centBounds,set.pinfo.size());
          binner.bin_parallel(set.prims->data(),set.object_range.begin(),set.object_range.end(),PARALLEL_FIND_BLOCK_SIZE,PARALLEL_THRESHOLD,mapping,space,&user);
          Split osplit = binner.best(mapping,logBlockSize);
          osplit.sah *= set.pinfo.time_range.size();
          if (!osplit.valid()) osplit.data = Split::SPLIT_FALLBACK; // use fallback split
          return osplit;
        }
        
        /*! array partitioning */
        __forceinline void split(const Split& split, const LinearSpace3fa& space, const SetMB& set, SetMB& lset, SetMB& rset)
        {
          UserPrimRefData user(scene,set.time_range);
          const size_t begin = set.object_range.begin();
          const size_t end   = set.object_range.end();
          PrimInfoMB left = empty;
          PrimInfoMB right = empty;
          const vint4 vSplitPos(split.pos);
          const vbool4 vSplitMask(1 << split.dim);
          auto isLeft = [&] (const PrimRefMB &ref) { return any(((vint4)split.mapping.bin_unsafe(ref,space,&user) < vSplitPos) & vSplitMask); };
          auto reduction = [] (PrimInfoMB& pinfo, const PrimRefMB& ref) { pinfo.add_primref(ref); };
          auto reduction2 = [] (PrimInfoMB& pinfo0,const PrimInfoMB& pinfo1) { pinfo0.merge(pinfo1); };
          size_t center = parallel_partitioning(set.prims->data(),begin,end,EmptyTy(),left,right,isLeft,reduction,reduction2,PARALLEL_PARTITION_BLOCK_SIZE,PARALLEL_THRESHOLD);
          new (&lset) SetMB(left,set.prims,range<size_t>(begin,center),set.time_range);
          new (&rset) SetMB(right,set.prims,range<size_t>(center,end ),set.time_range);
        }

      private:
        Scene* scene;
      };
  }
}
