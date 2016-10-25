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

//#define DBG_PRINT(x) PRINT(x)
#define DBG_PRINT(x) 

namespace embree
{
  namespace isa
  { 
    /*! Performs standard object binning */
    template<typename PrimRef>
      struct HeuristicArraySweepSAH
      {
        /*! stores all information to perform some split */
        struct Split
        {
          /*! construct an invalid split by default */
          __forceinline Split()
            : sah(inf), dim(-1), pos(0), data(0) {}
        
          /*! constructs specified split */
          __forceinline Split(float sah, int dim, int pos)
            : sah(sah), dim(dim), pos(pos), data(0) {}
        
          /*! tests if this split is valid */
          __forceinline bool valid() const { return dim != -1; }
        
          /*! calculates surface area heuristic for performing the split */
          __forceinline float splitSAH() const { return sah; }
        
          /*! stream output */
          friend std::ostream& operator<<(std::ostream& cout, const Split& split) {
            return cout << "Split { sah = " << split.sah << ", dim = " << split.dim << ", pos = " << split.pos << "}";
          }
        
        public:
          float sah;                //!< SAH cost of the split
          int dim;                  //!< split dimension
          int pos;                  //!< bin index for splitting
          unsigned int data;        //!< extra optional split data
        };

        typedef range<size_t> Set;

#if defined(__AVX512F__)
        static const size_t PARALLEL_THRESHOLD = 3*1024; 
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 768;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 128;
#else
        static const size_t PARALLEL_THRESHOLD = 3 * 1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 128;
#endif

        struct Centroid
        {
          float v;
          unsigned int id;
          __forceinline Centroid() {}
          __forceinline Centroid(const float v, const size_t id) : v(v), id((unsigned int)id) {}

          __forceinline bool operator<(const Centroid &m) const { return v < m.v; } 

        };

        __forceinline HeuristicArraySweepSAH ()
          : prims(nullptr) {}
        
        /*! remember prim array */
        __forceinline HeuristicArraySweepSAH (PrimRef* prims)
          : prims(prims) {}

        const LBBox3fa computePrimInfoMB(size_t timeSegment, size_t numTimeSteps, Scene* scene, const PrimInfo& pinfo)
        {
          LBBox3fa allBounds = empty;
          for (size_t i=pinfo.begin; i<pinfo.end; i++) // FIXME: parallelize
          {
            BezierPrim& prim = prims[i];
            const size_t geomID = prim.geomID();
            const BezierCurves* curves = scene->getBezierCurves(geomID);
            allBounds.extend(curves->linearBounds(prim.primID(),timeSegment,numTimeSteps));
          }
          return allBounds;
        }

        /*! finds the best split */
        const Split find(const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Set set(pinfo.begin,pinfo.end);
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) return sequential_find(set,pinfo,logBlockSize);
          else                                           return   parallel_find(set,pinfo,logBlockSize);
        }

        /*! finds the best split */
        const Split find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          /* std::cout << std::endl; */
          /* PING; */
          DBG_PRINT(pinfo);
          DBG_PRINT(area(pinfo.geomBounds));
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) return sequential_find(set,pinfo,logBlockSize);
          else                                           return   parallel_find(set,pinfo,logBlockSize);
        }
        
        /*! finds the best split */
        __noinline const Split sequential_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          assert(pinfo.size() == set.size());
          const size_t numPrims = pinfo.size();
          DBG_PRINT(numPrims);
          assert(numPrims);

          /* create temporary arrays */
          Centroid *centroid[3] = {
            new Centroid[numPrims],
            new Centroid[numPrims],
            new Centroid[numPrims]
          };          
          float *right_area = new float[numPrims];

          /* init & sort arrays */
          const size_t begin = set.begin();
          for (size_t i = 0;i<numPrims;i++)
          {
            const Vec3fa center = prims[begin+i].bounds().center();
            new (&centroid[0][i]) Centroid(center.x,begin+i);
            new (&centroid[1][i]) Centroid(center.y,begin+i);
            new (&centroid[2][i]) Centroid(center.z,begin+i);
          }          
          std::sort(&centroid[0][0],&centroid[0][numPrims]);
          std::sort(&centroid[1][0],&centroid[1][numPrims]);
          std::sort(&centroid[2][0],&centroid[2][numPrims]);

          /* scan and compute best sah split */
          const vfloat4 diag = (vfloat4)pinfo.centBounds.size();
          const vfloat4 scale = select(diag > vfloat4(1E-34f),diag,vfloat4(0.0f));

          float bestSAH = inf;
          int   bestDim = -1;
          int   bestPos = 0;
          for (size_t dim = 0;dim<3;dim++)
          {
            if (unlikely(scale[dim] == 0.0f)) continue;

            /* compute area from right to left */
            BBox3fa right_bounds(empty);
            for (size_t i = numPrims-1;i>0;i--)
            {
              right_bounds = right_bounds.extend(prims[centroid[dim][i].id].bounds());
              right_area[i] = halfArea(right_bounds);
            }
            right_area[0] = halfArea(right_bounds);

            /* compute sah */
            BBox3fa left_bounds(empty);
            for (size_t i = 1;i<numPrims;i++)
            {
              left_bounds = left_bounds.extend(prims[centroid[dim][i-1].id].bounds());
              const size_t numLeft = i;
              const size_t numRight = numPrims-numLeft;              
              const float lArea = halfArea(left_bounds);
              const float rArea = right_area[i];
              const size_t blocks_add = (1 << logBlockSize)-1;
              const size_t lCount = (numLeft +blocks_add) >> logBlockSize;
              const size_t rCount = (numRight+blocks_add) >> logBlockSize;
              const float sah =  lArea*lCount + rArea*rCount;

              if (unlikely(sah < bestSAH)) {
                bestDim = (int)dim;
                bestPos = (int)i;
                bestSAH = sah;
              }
            }

          }                    
                 
          /* delete temporary arrays */
          delete [] right_area;
          delete [] centroid[0];
          delete [] centroid[1];
          delete [] centroid[2];

          DBG_PRINT(Split(bestSAH,bestDim,bestPos));
          return Split(bestSAH,bestDim,bestPos);
        }
        
        /*! finds the best split */
        __noinline const Split parallel_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          return sequential_find(set,pinfo,logBlockSize);
        }
        
        /*! array partitioning */
        void split(const Split& spliti, const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right) 
        {
          Set lset,rset;
          Set set(pinfo.begin,pinfo.end);
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
            split_helper<false>(spliti,pinfo,set,left,lset,right,rset);
          else
            split_helper<true>(spliti,pinfo,set,left,lset,right,rset);
        }
        
        /*! array partitioning */
        __forceinline void split(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
            split_helper<false>(split,pinfo,set,left,lset,right,rset);
          else                                
            split_helper<true>(split,pinfo,set,left,lset,right,rset);
        }

        template<bool parallel>
        __forceinline void split_helper(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
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

          /* sort prims according to best split dimension */
          std::sort(&prims[begin],&prims[end], [&](const PrimRef &a, const PrimRef &b) -> bool { 
              const Vec3fa centerA = a.bounds().center();
              const Vec3fa centerB = b.bounds().center();
              return centerA[splitDim] < centerB[splitDim];
            });

          const size_t center = begin + splitPos;
          assert(begin < center);
          assert(center < end);

          /* compute left, right pinfos */
          for (size_t i = begin;i<center;i++)
            local_left.extend(prims[i].bounds());
          for (size_t i = center;i<end;i++)
            local_right.extend(prims[i].bounds());

          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) range<size_t>(begin,center);
          new (&rset) range<size_t>(center,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
          DBG_PRINT(center);
          DBG_PRINT(area(local_left.geomBounds));
          DBG_PRINT(left);
          DBG_PRINT(area(local_right.geomBounds));
          DBG_PRINT(right);

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
        PrimRef* const prims;
      };
  }
}
