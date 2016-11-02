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

namespace embree
{
  namespace isa
  { 
    /*! stores extended information about the split */
    struct SplitInfo2
    {
      __forceinline SplitInfo2 () {}
      
      __forceinline SplitInfo2 (size_t leftCount, const LBBox3fa& leftBounds, size_t rightCount, const LBBox3fa& rightBounds)
	: leftCount(leftCount), rightCount(rightCount), leftBounds(leftBounds), rightBounds(rightBounds) {}
      
    public:
      size_t leftCount,rightCount;
      LBBox3fa leftBounds,rightBounds;
    };
    
    /*! stores all binning information */
    template<size_t BINS, typename PrimRef>
      struct __aligned(64) BinInfo2
    {
		  
      typedef BinSplit<BINS> Split;
      
      __forceinline BinInfo2() {
      }
      
      __forceinline BinInfo2(EmptyTy) {
	clear();
      }

      /*! bin access function */
      __forceinline LBBox3fa &bounds(const size_t binID, const size_t dimID)             { return _bounds[binID][dimID]; }
      __forceinline const LBBox3fa &bounds(const size_t binID, const size_t dimID) const { return _bounds[binID][dimID]; }

      __forceinline int &counts(const size_t binID, const size_t dimID)             { return _counts[binID][dimID]; }
      __forceinline const int &counts(const size_t binID, const size_t dimID) const { return _counts[binID][dimID]; }

      __forceinline vint4 &counts(const size_t binID)             { return _counts[binID]; }
      __forceinline const vint4 &counts(const size_t binID) const { return _counts[binID]; }

      /*! clears the bin info */
      __forceinline void clear() 
      {
#if defined(__AVX__)
        const vfloat8 emptyBounds(pos_inf,pos_inf,pos_inf,0.0f,neg_inf,neg_inf,neg_inf,0.0f);

	for (size_t i=0; i<BINS; i++) {
          vfloat8::store(&bounds(i,0),emptyBounds);
          vfloat8::store(&bounds(i,1),emptyBounds);
          vfloat8::store(&bounds(i,2),emptyBounds);
	  counts(i) = vint4(zero);
	}
#else
	for (size_t i=0; i<BINS; i++) {
	  bounds(i,0) = bounds(i,1) = bounds(i,2) = empty;
	  counts(i) = vint4(zero);
	}
#endif
      }
      
      /*! bins an array of primitives */
      __forceinline void bin (const PrimRef* prims, size_t N, const BinMapping<BINS>& mapping)
      {
	if (unlikely(N == 0)) return;
	size_t i; 
	for (i=0; i<N-1; i+=2)
        {
          /*! map even and odd primitive to bin */
          const LBBox3fa prim0 = prims[i+0].bounds(); 
          const Vec3fa center0 = Vec3fa(center2(prim0)); 
          const vint4 bin0 = (vint4)mapping.bin(center0); 
          
          const LBBox3fa prim1 = prims[i+1].bounds(); 
          const Vec3fa center1 = Vec3fa(center2(prim1)); 
          const vint4 bin1 = (vint4)mapping.bin(center1); 
          
          /*! increase bounds for bins for even primitive */
          const unsigned int b00 = extract<0>(bin0); bounds(b00,0).extend(prim0); 
          const unsigned int b01 = extract<1>(bin0); bounds(b01,1).extend(prim0); 
          const unsigned int b02 = extract<2>(bin0); bounds(b02,2).extend(prim0); 

          counts(b00,0)++;
          counts(b01,1)++;
          counts(b02,2)++;

          /*! increase bounds of bins for odd primitive */
          const unsigned int b10 = extract<0>(bin1);  bounds(b10,0).extend(prim1); 
          const unsigned int b11 = extract<1>(bin1);  bounds(b11,1).extend(prim1); 
          const unsigned int b12 = extract<2>(bin1);  bounds(b12,2).extend(prim1); 

          counts(b10,0)++;
          counts(b11,1)++;
          counts(b12,2)++;
        }
	/*! for uneven number of primitives */
	if (i < N)
        {
          /*! map primitive to bin */
          const LBBox3fa prim0 = prims[i].bounds(); const Vec3fa center0 = Vec3fa(center2(prim0)); const vint4 bin0 = (vint4)mapping.bin(center0); 
          
          /*! increase bounds of bins */
          const int b00 = extract<0>(bin0); counts(b00,0)++; bounds(b00,0).extend(prim0);
          const int b01 = extract<1>(bin0); counts(b01,1)++; bounds(b01,1).extend(prim0);
          const int b02 = extract<2>(bin0); counts(b02,2)++; bounds(b02,2).extend(prim0);
        }
      }

      __forceinline void bin(const PrimRef* prims, size_t begin, size_t end, const BinMapping<BINS>& mapping) {
	bin(prims+begin,end-begin,mapping);
      }

      __forceinline void bin(const PrimRef* prims, size_t begin, size_t end, const BinMapping<BINS>& mapping, const AffineSpace3fa& space) {
	bin(prims+begin,end-begin,mapping,space);
      }   

      /*! merges in other binning information */
      __forceinline void merge (const BinInfo2& other, size_t numBins)
      {
	for (size_t i=0; i<numBins; i++) 
        {
          counts(i) += other.counts(i);
          bounds(i,0).extend(other.bounds(i,0));
          bounds(i,1).extend(other.bounds(i,1));
          bounds(i,2).extend(other.bounds(i,2));
        }
      }

      /*! reduces binning information */
      static __forceinline const BinInfo2 reduce (const BinInfo2& a, const BinInfo2& b, const size_t numBins = BINS)
      {
        BinInfo2 c;
	for (size_t i=0; i<numBins; i++) 
        {
          c.counts(i) = a.counts(i)+b.counts(i);
          c.bounds(i,0) = embree::merge(a.bounds(i,0),b.bounds(i,0));
          c.bounds(i,1) = embree::merge(a.bounds(i,1),b.bounds(i,1));
          c.bounds(i,2) = embree::merge(a.bounds(i,2),b.bounds(i,2));
        }
        return c;
      }
      
      /*! finds the best split by scanning binning information */
      __forceinline Split best(const BinMapping<BINS>& mapping, const size_t blocks_shift) const
      {
	/* sweep from right to left and compute parallel prefix of merged bounds */
	vfloat4 rAreas[BINS];
	vint4 rCounts[BINS];
	vint4 count = 0; LBBox3fa bx = empty; LBBox3fa by = empty; LBBox3fa bz = empty;
	for (size_t i=mapping.size()-1; i>0; i--)
        {
          count += counts(i);
          rCounts[i] = count;
          bx.extend(bounds(i,0)); rAreas[i][0] = bx.expectedApproxHalfArea();
          by.extend(bounds(i,1)); rAreas[i][1] = by.expectedApproxHalfArea();
          bz.extend(bounds(i,2)); rAreas[i][2] = bz.expectedApproxHalfArea();
          rAreas[i][3] = 0.0f;
        }
	/* sweep from left to right and compute SAH */
	vint4 blocks_add = (1 << blocks_shift)-1;
	vint4 ii = 1; vfloat4 vbestSAH = pos_inf; vint4 vbestPos = 0; 
	count = 0; bx = empty; by = empty; bz = empty;
	for (size_t i=1; i<mapping.size(); i++, ii+=1)
        {
          count += counts(i-1);
          bx.extend(bounds(i-1,0)); float Ax = bx.expectedApproxHalfArea();
          by.extend(bounds(i-1,1)); float Ay = by.expectedApproxHalfArea();
          bz.extend(bounds(i-1,2)); float Az = bz.expectedApproxHalfArea();
          const vfloat4 lArea = vfloat4(Ax,Ay,Az,Az);
          const vfloat4 rArea = rAreas[i];
          const vint4 lCount = (count     +blocks_add) >> int(blocks_shift);
          const vint4 rCount = (rCounts[i]+blocks_add) >> int(blocks_shift);
          const vfloat4 sah = lArea*vfloat4(lCount) + rArea*vfloat4(rCount);
          vbestPos = select(sah < vbestSAH,ii ,vbestPos);
          vbestSAH = select(sah < vbestSAH,sah,vbestSAH);
        }
	
	/* find best dimension */
	float bestSAH = inf;
	int   bestDim = -1;
	int   bestPos = 0;
	for (int dim=0; dim<3; dim++) 
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
      __forceinline void getSplitInfo(const BinMapping<BINS>& mapping, const Split& split, SplitInfo2& info) const 
      {
	if (split.dim == -1) {
	  new (&info) SplitInfo2(0,empty,0,empty);
	  return;
	}
	
	size_t leftCount = 0;
	LBBox3fa leftBounds = empty;
	for (size_t i=0; i<(size_t)split.pos; i++) {
	  leftCount += counts(i,split.dim);
	  leftBounds.extend(bounds(i,split.dim));
	}
	size_t rightCount = 0;
	LBBox3fa rightBounds = empty;
	for (size_t i=split.pos; i<mapping.size(); i++) {
	  rightCount += counts(i,split.dim);
	  rightBounds.extend(bounds(i,split.dim));
	}
	new (&info) SplitInfo2(leftCount,leftBounds,rightCount,rightBounds);
      }

      /*! gets the number of primitives left of the split */
      __forceinline size_t getLeftCount(const BinMapping<BINS>& mapping, const Split& split) const
      {
        if (unlikely(split.dim == -1)) return -1;

        size_t leftCount = 0;
        for (size_t i = 0; i < (size_t)split.pos; i++) {
          leftCount += counts(i, split.dim);
        }
        return leftCount;
      }

      /*! gets the number of primitives right of the split */
      __forceinline size_t getRightCount(const BinMapping<BINS>& mapping, const Split& split) const
      {
        if (unlikely(split.dim == -1)) return -1;

        size_t rightCount = 0;
        for (size_t i = (size_t)split.pos; i<mapping.size(); i++) {
          rightCount += counts(i, split.dim);
        }
        return rightCount;
      }

    private:
      LBBox3fa _bounds[BINS][3]; //!< geometry bounds for each bin in each dimension
      vint4   _counts[BINS];    //!< counts number of primitives that map into the bins
    };
  }
}
