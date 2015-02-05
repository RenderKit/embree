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
#include "builders/primrefalloc.h"
//#include "builders/heuristic_fallback.h"
#include "algorithms/range.h"
#include "algorithms/parallel_reduce.h"
#include "algorithms/parallel_partition.h"

namespace embree
{
  namespace isa
  { 
    /*! mapping into bins */
    template<size_t BINS>
      struct BinMapping
      {
      public:
        __forceinline BinMapping() {}
        
        /*! calculates the mapping */
        __forceinline BinMapping(const PrimInfo& pinfo) 
        {
          num = min(BINS,size_t(4.0f + 0.05f*pinfo.size()));
          const ssef diag = (ssef) pinfo.centBounds.size();
          scale = select(diag > ssef(1E-19f),rcp(diag) * ssef(0.99f*num),ssef(0.0f));
          ofs  = (ssef) pinfo.centBounds.lower;
        }
        
        /*! returns number of bins */
        __forceinline size_t size() const { return num; }
        
        /*! slower but safe binning */
        __forceinline Vec3ia bin(const Vec3fa& p) const 
        {
          const ssei i = floori((ssef(p)-ofs)*scale);
#if 1
          assert(i[0] >=0 && i[0] < num); 
          assert(i[1] >=0 && i[1] < num);
          assert(i[2] >=0 && i[2] < num);
          return Vec3ia(i);
#else
          return Vec3ia(clamp(i,ssei(0),ssei(num-1)));
#endif
        }
        
        /*! faster but unsafe binning */
        __forceinline Vec3ia bin_unsafe(const Vec3fa& p) const {
          return Vec3ia(floori((ssef(p)-ofs)*scale));
        }
        
        /*! returns true if the mapping is invalid in some dimension */
        __forceinline bool invalid(const int dim) const {
          return scale[dim] == 0.0f;
        }
        
        /*! stream output */
        friend std::ostream& operator<<(std::ostream& cout, const BinMapping& mapping) {
          return cout << "BinMapping { num = " << mapping.num << ", ofs = " << mapping.ofs << ", scale = " << mapping.scale << "}";
        }
        
      public:
        size_t num;
        ssef ofs,scale;        //!< linear function that maps to bin ID
      };
    
    /*! stores all information to perform some split */
    template<size_t BINS>
      struct BinSplit
      {
        /*! construct an invalid split by default */
        __forceinline BinSplit()
          : sah(inf), dim(-1), pos(0) {}
        
        /*! constructs specified split */
        __forceinline BinSplit(float sah, int dim, int pos, const BinMapping<BINS>& mapping)
          : sah(sah), dim(dim), pos(pos), mapping(mapping) {}
        
        /*! tests if this split is valid */
        __forceinline bool valid() const { return dim != -1; }
        
        /*! calculates surface area heuristic for performing the split */
        __forceinline float splitSAH() const { return sah; }
        
        /*! stream output */
        friend std::ostream& operator<<(std::ostream& cout, const BinSplit& split) {
          return cout << "BinSplit { sah = " << split.sah << ", dim = " << split.dim << ", pos = " << split.pos << "}";
        }
        
      public:
        float sah;       //!< SAH cost of the split
        int dim;         //!< split dimension
        int pos;         //!< bin index for splitting
        BinMapping<BINS> mapping; //!< mapping into bins
      };
    
    /*! stores extended information about the split */
    struct SplitInfo
    {
      __forceinline SplitInfo () {}
      
      __forceinline SplitInfo (size_t leftCount, const BBox3fa& leftBounds, size_t rightCount, const BBox3fa& rightBounds)
	: leftCount(leftCount), rightCount(rightCount), leftBounds(leftBounds), rightBounds(rightBounds) {}
      
    public:
      size_t leftCount,rightCount;
      BBox3fa leftBounds,rightBounds;
    };
    
    /*! stores all binning information */
    template<size_t BINS, typename PrimRef>
      struct __aligned(64) BinInfo
    {
      typedef BinSplit<BINS> Split;
      
      __forceinline BinInfo() {
      }
      
      __forceinline BinInfo(EmptyTy) {
	clear();
      }
      
      /*! clears the bin info */
      __forceinline void clear() 
      {
	for (size_t i=0; i<BINS; i++) {
	  bounds[i][0] = bounds[i][1] = bounds[i][2] = empty;
	  counts[i] = 0;
	}
      }
      
      /*! bins an array of primitives */
      __forceinline void bin (const PrimRef* prims, size_t N, const BinMapping<BINS>& mapping)
      {
	if (N == 0) return;
        
	size_t i; 
	for (i=0; i<N-1; i+=2)
        {
          /*! map even and odd primitive to bin */
          const BBox3fa prim0 = prims[i+0].bounds(); 
          const Vec3fa center0 = Vec3fa(center2(prim0)); 
          const Vec3ia bin0 = mapping.bin(center0); 
          
          const BBox3fa prim1 = prims[i+1].bounds(); 
          const Vec3fa center1 = Vec3fa(center2(prim1)); 
          const Vec3ia bin1 = mapping.bin(center1); 
          
          /*! increase bounds for bins for even primitive */
          const unsigned int b00 = bin0.x; bounds[b00][0].extend(prim0);
          const unsigned int b01 = bin0.y; bounds[b01][1].extend(prim0);
          const unsigned int b02 = bin0.z; bounds[b02][2].extend(prim0);
          
          counts[b00][0]++;
          counts[b01][1]++;
          counts[b02][2]++;

          /*! increase bounds of bins for odd primitive */
          const unsigned int b10 = bin1.x;  bounds[b10][0].extend(prim1);
          const unsigned int b11 = bin1.y;  bounds[b11][1].extend(prim1);
          const unsigned int b12 = bin1.z;  bounds[b12][2].extend(prim1);

          counts[b10][0]++;
          counts[b11][1]++;
          counts[b12][2]++;
        }
	
	/*! for uneven number of primitives */
	if (i < N)
        {
          /*! map primitive to bin */
          const BBox3fa prim0 = prims[i].bounds(); const Vec3fa center0 = Vec3fa(center2(prim0)); const Vec3ia bin0 = mapping.bin(center0); 
          
          /*! increase bounds of bins */
          const int b00 = bin0.x; counts[b00][0]++; bounds[b00][0].extend(prim0);
          const int b01 = bin0.y; counts[b01][1]++; bounds[b01][1].extend(prim0);
          const int b02 = bin0.z; counts[b02][2]++; bounds[b02][2].extend(prim0);
        }
      }

      /*! bins an array of primitives */
      __forceinline void bin (const PrimRef* prims, size_t N, const BinMapping<BINS>& mapping, const AffineSpace3fa& space)
      {
	if (N == 0) return;
        
	size_t i; 
	for (i=0; i<N-1; i+=2)
        {
          /*! map even and odd primitive to bin */
          const BBox3fa prim0 = prims[i+0].bounds(space); const Vec3fa center0 = Vec3fa(center2(prim0)); const Vec3ia bin0 = mapping.bin(center0); 
          const BBox3fa prim1 = prims[i+1].bounds(space); const Vec3fa center1 = Vec3fa(center2(prim1)); const Vec3ia bin1 = mapping.bin(center1); 
          
          /*! increase bounds for bins for even primitive */
          const int b00 = bin0.x; counts[b00][0]++; bounds[b00][0].extend(prim0);
          const int b01 = bin0.y; counts[b01][1]++; bounds[b01][1].extend(prim0);
          const int b02 = bin0.z; counts[b02][2]++; bounds[b02][2].extend(prim0);
          
          /*! increase bounds of bins for odd primitive */
          const int b10 = bin1.x; counts[b10][0]++; bounds[b10][0].extend(prim1);
          const int b11 = bin1.y; counts[b11][1]++; bounds[b11][1].extend(prim1);
          const int b12 = bin1.z; counts[b12][2]++; bounds[b12][2].extend(prim1);
        }
	
	/*! for uneven number of primitives */
	if (i < N)
        {
          /*! map primitive to bin */
          const BBox3fa prim0 = prims[i].bounds(space); const Vec3fa center0 = Vec3fa(center2(prim0)); const Vec3ia bin0 = mapping.bin(center0); 
          
          /*! increase bounds of bins */
          const int b00 = bin0.x; counts[b00][0]++; bounds[b00][0].extend(prim0);
          const int b01 = bin0.y; counts[b01][1]++; bounds[b01][1].extend(prim0);
          const int b02 = bin0.z; counts[b02][2]++; bounds[b02][2].extend(prim0);
        }
      }
      
      __forceinline void bin(const PrimRef* prims, size_t begin, size_t end, const BinMapping<BINS>& mapping) {
	bin(prims+begin,end-begin,mapping);
      }

      __forceinline void bin(const PrimRef* prims, size_t begin, size_t end, const BinMapping<BINS>& mapping, const AffineSpace3fa& space) {
	bin(prims+begin,end-begin,mapping,space);
      }
      
      /*! merges in other binning information */
      __forceinline void merge (const BinInfo& other, size_t numBins)
      {
	for (size_t i=0; i<numBins; i++) 
        {
          counts[i] += other.counts[i];
          bounds[i][0].extend(other.bounds[i][0]);
          bounds[i][1].extend(other.bounds[i][1]);
          bounds[i][2].extend(other.bounds[i][2]);
        }
      }
      
      /*! finds the best split by scanning binning information */
      __forceinline Split best(const BinMapping<BINS>& mapping, const size_t blocks_shift)
      {
	/* sweep from right to left and compute parallel prefix of merged bounds */
	ssef rAreas[BINS];
	ssei rCounts[BINS];
	ssei count = 0; BBox3fa bx = empty; BBox3fa by = empty; BBox3fa bz = empty;
	for (size_t i=mapping.size()-1; i>0; i--)
        {
          count += counts[i];
          rCounts[i] = count;
          bx.extend(bounds[i][0]); rAreas[i][0] = halfArea(bx);
          by.extend(bounds[i][1]); rAreas[i][1] = halfArea(by);
          bz.extend(bounds[i][2]); rAreas[i][2] = halfArea(bz);
        }
	
	/* sweep from left to right and compute SAH */
	ssei blocks_add = (1 << blocks_shift)-1;
	ssei ii = 1; ssef vbestSAH = pos_inf; ssei vbestPos = 0; 
	count = 0; bx = empty; by = empty; bz = empty;
	for (size_t i=1; i<mapping.size(); i++, ii+=1)
        {
          count += counts[i-1];
          bx.extend(bounds[i-1][0]); float Ax = halfArea(bx);
          by.extend(bounds[i-1][1]); float Ay = halfArea(by);
          bz.extend(bounds[i-1][2]); float Az = halfArea(bz);
          const ssef lArea = ssef(Ax,Ay,Az,Az);
          const ssef rArea = rAreas[i];
          const ssei lCount = (count     +blocks_add) >> blocks_shift;
          const ssei rCount = (rCounts[i]+blocks_add) >> blocks_shift;
          const ssef sah = lArea*ssef(lCount) + rArea*ssef(rCount);
          vbestPos = select(sah < vbestSAH,ii ,vbestPos);
          vbestSAH = select(sah < vbestSAH,sah,vbestSAH);
        }
	
	/* find best dimension */
	float bestSAH = inf;
	int   bestDim = -1;
	int   bestPos = 0;
	int   bestLeft = 0;
	for (size_t dim=0; dim<3; dim++) 
        {
          /* ignore zero sized dimensions */
          if (unlikely(mapping.invalid(dim)))
            continue;
          
          /* test if this is a better dimension */
          if (vbestSAH[dim] < bestSAH && vbestPos[dim] != 0) {
            bestDim = dim;
            bestPos = vbestPos[dim];
            bestSAH = vbestSAH[dim];
          }
        }
	
	return Split(bestSAH,bestDim,bestPos,mapping);
      }
      
      /*! calculates extended split information */
      __forceinline void getSplitInfo(const BinMapping<BINS>& mapping, const Split& split, SplitInfo& info) const
      {
	if (split.dim == -1) {
	  new (&info) SplitInfo(0,empty,0,empty);
	  return;
	}
	
	size_t leftCount = 0;
	BBox3fa leftBounds = empty;
	for (size_t i=0; i<split.pos; i++) {
	  leftCount += counts[i][split.dim];
	  leftBounds.extend(bounds[i][split.dim]);
	}
	size_t rightCount = 0;
	BBox3fa rightBounds = empty;
	for (size_t i=split.pos; i<mapping.size(); i++) {
	  rightCount += counts[i][split.dim];
	  rightBounds.extend(bounds[i][split.dim]);
	}
	new (&info) SplitInfo(leftCount,leftBounds,rightCount,rightBounds);
      }
      
    private:
      BBox3fa bounds[BINS][3]; //!< geometry bounds for each bin in each dimension
      ssei    counts[BINS];    //!< counts number of primitives that map into the bins
    };
    
    
    /*! Performs standard object binning */
    template<typename PrimRef, size_t BINS = 32>
      struct HeuristicArrayBinningSAH
      {
        typedef BinSplit<BINS> Split;
        typedef BinInfo<BINS,PrimRef> Binner;
        typedef range<size_t> Set;
        
        static const size_t PARALLEL_THRESHOLD = 10000;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 4096;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 64;

        __forceinline HeuristicArrayBinningSAH ()
          : prims(NULL) {}
        
        /*! remember prim array */
        __forceinline HeuristicArrayBinningSAH (PrimRef* prims)
          : prims(prims) {}

        const std::pair<BBox3fa,BBox3fa> computePrimInfoMB(Scene* scene, const PrimInfo& pinfo)
        {
          BBox3fa bounds0 = empty;
          BBox3fa bounds1 = empty;
          for (size_t i=pinfo.begin; i<pinfo.end; i++) // FIXME: parallelize
          {
            Bezier1v& prim = prims[i];
            const size_t geomID = prim.geomID<0>();
            const BezierCurves* curves = scene->getBezierCurves(geomID);
            bounds0.extend(curves->bounds(prim.primID<0>(),0));
            bounds1.extend(curves->bounds(prim.primID<0>(),1));
          }
          return std::pair<BBox3fa,BBox3fa>(bounds0,bounds1);
        }

        /*! finds the best split */
        const Split find(const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Set set(pinfo.begin,pinfo.end);
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) return sequential_find(set,pinfo,logBlockSize);
          else                              return   parallel_find(set,pinfo,logBlockSize);
        }

        /*! finds the best split */
        const Split find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) return sequential_find(set,pinfo,logBlockSize);
          else                              return   parallel_find(set,pinfo,logBlockSize);
        }
        
        /*! finds the best split */
        const Split sequential_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          binner.bin(prims,set.begin(),set.end(),mapping);
          return binner.best(mapping,logBlockSize);
        }
        
        /*! finds the best split */
        const Split parallel_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          Binner binner(empty);
          const BinMapping<BINS> mapping(pinfo);
          binner = parallel_reduce(set.begin(),set.end(),PARALLEL_FIND_BLOCK_SIZE,binner,
                                   [&] (const range<size_t>& r) { Binner binner(empty); binner.bin(prims+r.begin(),r.size(),mapping); return binner; },
                                   [&] (const Binner& b0, const Binner& b1) { Binner r = b0; r.merge(b1,mapping.size()); return r; });
          return binner.best(mapping,logBlockSize);
        }
        
        /*! array partitioning */
        void split(const Split& spliti, const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right) 
        {
          Set lset,rset;
          Set set(pinfo.begin,pinfo.end);
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
            sequential_split(spliti,set,left,lset,right,rset);
          else
            parallel_split(spliti,set,left,lset,right,rset);
        }
        
        /*! array partitioning */
        void split(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (likely(pinfo.size() < PARALLEL_THRESHOLD)) 
            sequential_split(split,set,left,lset,right,rset);
          else                                
            parallel_split(split,set,left,lset,right,rset);
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
          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          const unsigned int splitDimMask = (unsigned int)1 << splitDim;

          size_t center = serial_partitioning(prims,begin,end,local_left,local_right,
                                              [&] (const PrimRef& ref) { return split.mapping.bin_unsafe(center2(ref.bounds()))[splitDim] < splitPos; },
                                              [] (CentGeomBBox3fa& pinfo,const PrimRef& ref) { pinfo.extend(ref.bounds()); });
          
          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) range<size_t>(begin,center);
          new (&rset) range<size_t>(center,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
        }
        
        /*! array partitioning */
        void parallel_split(const Split& split, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset)
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
          
          const size_t mid = parallel_in_place_partitioning<PARALLEL_PARITION_BLOCK_SIZE,PrimRef,PrimInfo>
	  (&prims[begin],end-begin,init,left,right,
	   [&] (const PrimRef &ref) { return split.mapping.bin_unsafe(center2(ref.bounds()))[splitDim] < splitPos; },
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
          BBox3fa s0t1_s1t0;
          BBox3fa s1t1;
        };

        __forceinline UnalignedHeuristicArrayBinningSAH ()
          : prims(NULL) {}
        
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
        
        const PrimInfoMB computePrimInfoMB(Scene* scene, const PrimInfo& pinfo, const std::pair<AffineSpace3fa,AffineSpace3fa>& spaces)
        {
          size_t N = 0;
          BBox3fa centBounds = empty;
          BBox3fa geomBounds = empty;
          BBox3fa s0t0 = empty, s0t1_s1t0 = empty, s1t1 = empty;
          for (size_t i=pinfo.begin; i<pinfo.end; i++)  // FIXME: parallelize
          {
            const Bezier1v& prim = prims[i];
            const size_t geomID = prim.geomID<0>();
            const size_t primID = prim.primID<0>();

            N++;
            geomBounds.extend(prim.bounds(spaces.first));
            centBounds.extend(prim.center(spaces.first));

            const BezierCurves* curves = scene->getBezierCurves(prim.geomID<0>());
            s0t0.extend(curves->bounds(spaces.first,prim.primID<0>(),0));
            s0t1_s1t0.extend(curves->bounds(spaces.first,spaces.second,prim.primID<0>()));
            s1t1.extend(curves->bounds(spaces.second,prim.primID<0>(),1));
          }
          
          PrimInfoMB ret;
          ret.pinfo = PrimInfo(N,geomBounds,centBounds);
          ret.s0t0 = s0t0;
          ret.s0t1_s1t0 = s0t1_s1t0;
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
                                   [&] (const range<size_t>& r) { Binner binner(empty); binner.bin(prims+r.begin(),r.size(),mapping,space); return binner; },
                                   [&] (const Binner& b0, const Binner& b1) { Binner r = b0; r.merge(b1,mapping.size()); return r; });
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
          if (likely(pinfo.size() < 10000)) sequential_split(split,space,set,left,lset,right,rset);
          else                                parallel_split(split,space,set,left,lset,right,rset);
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
