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

#include "priminfo.h"
#include "../geometry/bezier1v.h"

#include "../../algorithms/parallel_reduce.h"
#include "../../algorithms/parallel_partition.h"

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
          const float4 diag = (float4) pinfo.centBounds.size();
          //scale = select(diag > float4(1E-19f),rcp(diag) * float4(0.99f*num),float4(0.0f));
          scale = select(diag > float4(1E-34f),float4(0.99f*num)/diag,float4(0.0f));
          ofs  = (float4) pinfo.centBounds.lower;
        }
        
        /*! returns number of bins */
        __forceinline size_t size() const { return num; }
        
        /*! slower but safe binning */
        __forceinline Vec3ia bin(const Vec3fa& p) const 
        {
          const int4 i = floori((float4(p)-ofs)*scale);
#if 1
          assert(i[0] >=0 && i[0] < num); 
          assert(i[1] >=0 && i[1] < num);
          assert(i[2] >=0 && i[2] < num);
          return Vec3ia(i);
#else
          return Vec3ia(clamp(i,int4(0),int4(num-1)));
#endif
        }
        
        /*! faster but unsafe binning */
        __forceinline Vec3ia bin_unsafe(const Vec3fa& p) const {
          return Vec3ia(floori((float4(p)-ofs)*scale));
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
        float4 ofs,scale;        //!< linear function that maps to bin ID
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
        float sah;                //!< SAH cost of the split
        int dim;                  //!< split dimension
        int pos;                  //!< bin index for splitting
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

      /*! reducesr binning information */
      static __forceinline const BinInfo reduce (const BinInfo& a, const BinInfo& b)
      {
        BinInfo c;
	for (size_t i=0; i<BINS; i++) 
        {
          c.counts[i] = a.counts[i]+b.counts[i];
          c.bounds[i][0] = embree::merge(a.bounds[i][0],b.bounds[i][0]);
          c.bounds[i][1] = embree::merge(a.bounds[i][1],b.bounds[i][1]);
          c.bounds[i][2] = embree::merge(a.bounds[i][2],b.bounds[i][2]);
        }
        return c;
      }
      
      /*! finds the best split by scanning binning information */
      __forceinline Split best(const BinMapping<BINS>& mapping, const size_t blocks_shift) const
      {
	/* sweep from right to left and compute parallel prefix of merged bounds */
	float4 rAreas[BINS];
	int4 rCounts[BINS];
	int4 count = 0; BBox3fa bx = empty; BBox3fa by = empty; BBox3fa bz = empty;
	for (size_t i=mapping.size()-1; i>0; i--)
        {
          count += counts[i];
          rCounts[i] = count;
          bx.extend(bounds[i][0]); rAreas[i][0] = halfArea(bx);
          by.extend(bounds[i][1]); rAreas[i][1] = halfArea(by);
          bz.extend(bounds[i][2]); rAreas[i][2] = halfArea(bz);
          rAreas[i][3] = 0.0f;
        }
	
	/* sweep from left to right and compute SAH */
	int4 blocks_add = (1 << blocks_shift)-1;
	int4 ii = 1; float4 vbestSAH = pos_inf; int4 vbestPos = 0; 
	count = 0; bx = empty; by = empty; bz = empty;
	for (size_t i=1; i<mapping.size(); i++, ii+=1)
        {
          count += counts[i-1];
          bx.extend(bounds[i-1][0]); float Ax = halfArea(bx);
          by.extend(bounds[i-1][1]); float Ay = halfArea(by);
          bz.extend(bounds[i-1][2]); float Az = halfArea(bz);
          const float4 lArea = float4(Ax,Ay,Az,Az);
          const float4 rArea = rAreas[i];
          const int4 lCount = (count     +blocks_add) >> blocks_shift;
          const int4 rCount = (rCounts[i]+blocks_add) >> blocks_shift;
          const float4 sah = lArea*float4(lCount) + rArea*float4(rCount);
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
      __forceinline void getSplitInfo(const BinMapping<BINS>& mapping, const Split& split, SplitInfo& info) const // FIXME: still required?
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
      int4    counts[BINS];    //!< counts number of primitives that map into the bins
    };
  }
}
