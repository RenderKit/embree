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

#include "geometry/bezier1v.h"
#include "builders/primrefalloc.h"
#include "heuristic_fallback.h"
#include "algorithms/parallel_reduce.h"
#include "algorithms/parallel_partition.h"

namespace embree
{
  namespace isa
  {
    /*! Performs standard object binning */
    struct ObjectPartitionNew
    {
      struct Split;
      struct SplitInfo;
      typedef atomic_set<PrimRefBlockT<PrimRef> > PrimRefList;   //!< list of primitives
      typedef atomic_set<PrimRefBlockT<BezierPrim> > BezierRefList; //!< list of bezier primitives
      
    public:
      
      /*! finds the best split */
      template<bool Parallel>
	static const Split find(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, BezierRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize);
      
      /*! finds the best split */
      template<bool Parallel>
	static const Split find(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize);

      /*! finds the best split and returns extended split information */
      template<bool Parallel>
      static const Split find(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo& sinfo_o);
      
      /*! finds the best split */
      static const Split find(PrimRef *__restrict__ const prims, const size_t begin, const size_t end, const PrimInfo& pinfo, const size_t logBlockSize)
      {
        BinInfo binner(empty);
        const Mapping mapping(pinfo);
        binner.bin(prims+begin,end-begin,mapping);
        return binner.best(mapping,logBlockSize);
      }

      /*! finds the best split */
      static const Split find_parallel(PrimRef *__restrict__ const prims, const size_t begin, const size_t end, const PrimInfo& pinfo, const size_t logBlockSize)
      {
        BinInfo binner(empty);
        const Mapping mapping(pinfo);
        //binner.bin(prims+begin,end-begin,mapping);
        
        binner = parallel_reduce(begin,end,binner,
                                 [&](const range<size_t>& r) { BinInfo binner(empty); binner.bin(prims+r.begin(),r.size(),mapping); return binner; },
                                 [] (const BinInfo& b0, const BinInfo& b1) { BinInfo r = b0; r.merge(b1); return r; });

        return binner.best(mapping,logBlockSize);
      }
      
    private:
      
      /*! number of bins */
      static const size_t maxBins = 32;
      
      /*! number of tasks */
      static const size_t maxTasks = 32;
      
      /*! mapping into bins */
      struct Mapping
      {
      public:
	__forceinline Mapping() {}
	
	/*! calculates the mapping */
        __forceinline Mapping(const PrimInfo& pinfo) 
        {
          num = min(maxBins,size_t(4.0f + 0.05f*pinfo.size()));
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
	friend std::ostream& operator<<(std::ostream& cout, const Mapping& mapping) {
	  return cout << "Mapping { num = " << mapping.num << ", ofs = " << mapping.ofs << ", scale = " << mapping.scale << "}";
	}
	
      public:
	size_t num;
	ssef ofs,scale;        //!< linear function that maps to bin ID
      };

      
    public:
      
      /*! stores all information to perform some split */
      struct Split
      {
	/*! construct an invalid split by default */
	__forceinline Split()
	  : sah(inf), dim(-1), pos(0) {}
	
	/*! constructs specified split */
	__forceinline Split(float sah, int dim, int pos, const Mapping& mapping)
	  : sah(sah), dim(dim), pos(pos), mapping(mapping) {}
	
	/*! tests if this split is valid */
	__forceinline bool valid() const { return dim != -1; }

	/*! calculates surface area heuristic for performing the split */
	__forceinline float splitSAH() const { return sah; }
	
	/*! splitting into two sets */
	template<bool Parallel>
	  void split(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, 
		     PrimRefBlockAlloc<BezierPrim>& alloc, 
		     BezierRefList& prims, 
		     BezierRefList& lprims_o, PrimInfo& linfo_o, 
		     BezierRefList& rprims_o, PrimInfo& rinfo_o) const;
	
	/*! splitting into two sets */
	template<bool Parallel>
	  void split(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, 
		     PrimRefBlockAlloc<PrimRef>& alloc, 
		     PrimRefList& prims, 
		     PrimRefList& lprims_o, PrimInfo& linfo_o, 
		     PrimRefList& rprims_o, PrimInfo& rinfo_o) const;
	
	/*! array partitioning */
        void partition(PrimRef *__restrict__ const prims, const size_t begin, const size_t end, PrimInfo& left, PrimInfo& right) const
        {
          assert(valid());
          CentGeomBBox3fa local_left(empty);
          CentGeomBBox3fa local_right(empty);
          
          assert(begin <= end);
          PrimRef* l = prims + begin;
          PrimRef* r = prims + end - 1;
          
          while(1)
          {
            while (likely(l <= r && mapping.bin_unsafe(center2(l->bounds()))[dim] < pos)) 
            {
              local_left.extend(l->bounds());
              ++l;
            }
            while (likely(l <= r && mapping.bin_unsafe(center2(r->bounds()))[dim] >= pos)) 
            {
              local_right.extend(r->bounds());
              --r;
            }
            if (r<l) break;
            
            const BBox3fa bl = l->bounds();
            const BBox3fa br = r->bounds();
            local_left.extend(br);
            local_right.extend(bl);
            *(BBox3fa*)l = br;
            *(BBox3fa*)r = bl;
            l++; r--;
          }
          
          unsigned int center = l - prims;
          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
        }

        /*! array partitioning */
        void partition_parallel(PrimRef *__restrict__ const prims, const size_t begin, const size_t end, PrimInfo& left, PrimInfo& right) const
        {
          left.reset(); 
          right.reset();
          PrimInfo init; init.reset();
          const unsigned int splitPos = pos;
          const unsigned int splitDim = dim;

          size_t mid = parallel_in_place_partitioning<128,PrimRef,PrimInfo>(&prims[begin],
                                                                            end-begin,
                                                                            init,
                                                                            left,
                                                                            right,
                                                                            [&] (const PrimRef &ref) { return mapping.bin_unsafe(center2(ref.bounds()))[splitDim] < splitPos; },
                                                                            [] (PrimInfo &pinfo,const PrimRef &ref) { pinfo.add(ref.bounds()); },
                                                                            [] (PrimInfo &pinfo0,const PrimInfo &pinfo1) { pinfo0.merge(pinfo1); }
                                                                            );

          left.begin = begin;
          left.end   = begin+mid;
          right.begin = begin+mid;
          right.end   = end;
        }

	/*! stream output */
	friend std::ostream& operator<<(std::ostream& cout, const Split& split) {
	  return cout << "Split { sah = " << split.sah << ", dim = " << split.dim << ", pos = " << split.pos << "}";
	}
	
      public:
	float sah;       //!< SAH cost of the split
	int dim;         //!< split dimension
	int pos;         //!< bin index for splitting
	Mapping mapping; //!< mapping into bins
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
      
    private:

      /*! stores all binning information */
      struct __aligned(64) BinInfo
      {
        __forceinline BinInfo() {
        }

        __forceinline BinInfo(EmptyTy) {
          clear();
        }
	
	/*! clears the bin info */
        __forceinline void clear() 
        {
          for (size_t i=0; i<maxBins; i++) {
            bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = empty;
            counts[i] = 0;
          }
        }
	
	/*! bins an array of bezier curves */
        __forceinline void bin (const BezierPrim* prims, size_t N, const Mapping& mapping)
        {
          for (size_t i=0; i<N; i++)
          {
            const BBox3fa cbounds = prims[i].bounds();
            const Vec3fa  center  = prims[i].center();
            const ssei bin = ssei(mapping.bin(center));
            const int b0 = bin[0]; counts[b0][0]++; bounds[b0][0].extend(cbounds);
            const int b1 = bin[1]; counts[b1][1]++; bounds[b1][1].extend(cbounds);
            const int b2 = bin[2]; counts[b2][2]++; bounds[b2][2].extend(cbounds);
          }
        }
	
	/*! bins an array of primitives */
        __forceinline void bin (const PrimRef* prims, size_t num, const Mapping& mapping)
        {
          if (num == 0) return;
          
          size_t i; 
          for (i=0; i<num-1; i+=2)
          {
            /*! map even and odd primitive to bin */
            const BBox3fa prim0 = prims[i+0].bounds(); const Vec3fa center0 = Vec3fa(center2(prim0)); const Vec3ia bin0 = mapping.bin(center0); 
            const BBox3fa prim1 = prims[i+1].bounds(); const Vec3fa center1 = Vec3fa(center2(prim1)); const Vec3ia bin1 = mapping.bin(center1); 
            
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
          if (i < num)
          {
            /*! map primitive to bin */
            const BBox3fa prim0 = prims[i].bounds(); const Vec3fa center0 = Vec3fa(center2(prim0)); const Vec3ia bin0 = mapping.bin(center0); 
            
            /*! increase bounds of bins */
            const int b00 = bin0.x; counts[b00][0]++; bounds[b00][0].extend(prim0);
            const int b01 = bin0.y; counts[b01][1]++; bounds[b01][1].extend(prim0);
            const int b02 = bin0.z; counts[b02][2]++; bounds[b02][2].extend(prim0);
          }
        }
	
        __forceinline void bin(const PrimRef* prims, size_t begin, size_t end, const Mapping& mapping) {
          bin(prims+begin,end-begin,mapping);
        }

	/*! bins a list of bezier curves */
        __forceinline void bin(BezierRefList& prims, const Mapping& mapping)
        {
          BezierRefList::iterator i=prims;
          while (BezierRefList::item* block = i.next())
            bin(block->base(),block->size(),mapping);
        }
	
	/*! bins a list of primitives */
        __forceinline void bin(PrimRefList& prims, const Mapping& mapping)
        {
          PrimRefList::iterator i=prims;
          while (PrimRefList::item* block = i.next())
            bin(block->base(),block->size(),mapping);
        }
	
	/*! merges in other binning information */
        __forceinline void merge (const BinInfo& other) 
        {
          for (size_t i=0; i<maxBins; i++) // FIXME: dont iterate over all bins
          {
            counts[i] += other.counts[i];
            bounds[i][0].extend(other.bounds[i][0]);
            bounds[i][1].extend(other.bounds[i][1]);
            bounds[i][2].extend(other.bounds[i][2]);
          }
        }

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
	
	/*! merge multiple binning infos into one */
        static void reduce(const BinInfo binners[], size_t num, BinInfo& binner_o)
        {
          binner_o = binners[0];
          for (size_t tid=1; tid<num; tid++) 
          {
            const BinInfo& binner = binners[tid];
            for (size_t bin=0; bin<maxBins; bin++) 
            {
              binner_o.bounds[bin][0].extend(binner.bounds[bin][0]);
              binner_o.bounds[bin][1].extend(binner.bounds[bin][1]);
              binner_o.bounds[bin][2].extend(binner.bounds[bin][2]);
              binner_o.counts[bin] += binner.counts[bin];
            }
          }
        }

	/*! finds the best split by scanning binning information */
        __forceinline Split best(const Mapping& mapping, const size_t blocks_shift)
        {
          /* sweep from right to left and compute parallel prefix of merged bounds */
          ssef rAreas[maxBins];
          ssei rCounts[maxBins];
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
	
	/*! calculates number of primitives on the left */
	__forceinline size_t getNumLeft(Split& split) 
	{
	  size_t N = 0;
	  for (size_t i=0; i<split.pos; i++)
	    N += counts[i][split.dim];
	  return N;
	}

	/*! calculates extended split information */
	__forceinline void getSplitInfo(const Mapping& mapping, const Split& split, SplitInfo& info) const
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
	
      public: // FIXME
      	BBox3fa bounds[maxBins][4]; //!< geometry bounds for each bin in each dimension
	ssei    counts[maxBins];    //!< counts number of primitives that map into the bins
      };
    };

    template<>
      inline const ObjectPartitionNew::Split ObjectPartitionNew::find<false>(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, BezierRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize)
    {
      BinInfo binner(empty);
      const Mapping mapping(pinfo);
      binner.bin(prims,mapping);
      return binner.best(mapping,logBlockSize);
    }

    template<>
      inline const ObjectPartitionNew::Split ObjectPartitionNew::find<false>(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize)
    {
      BinInfo binner(empty);
      const Mapping mapping(pinfo);
      binner.bin(prims,mapping);
      return binner.best(mapping,logBlockSize);
    }

    template<>
      inline const ObjectPartitionNew::Split ObjectPartitionNew::find<false>(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, PrimRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize, SplitInfo& sinfo_o)
    {
      BinInfo binner(empty);
      const Mapping mapping(pinfo);
      binner.bin(prims,mapping);
      const ObjectPartitionNew::Split split = binner.best(mapping,logBlockSize);
      binner.getSplitInfo(mapping,split,sinfo_o);
      return split;
    }


    template<>
    inline void ObjectPartitionNew::Split::split<false>(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, 
					      PrimRefBlockAlloc<BezierPrim>& alloc, 
					      BezierRefList& prims, 
					      BezierRefList& lprims_o, PrimInfo& linfo_o, 
					      BezierRefList& rprims_o, PrimInfo& rinfo_o) const
    {
      assert(valid());
      BezierRefList::item* lblock = lprims_o.insert(alloc.malloc(threadIndex));
      BezierRefList::item* rblock = rprims_o.insert(alloc.malloc(threadIndex));
      linfo_o.reset();
      rinfo_o.reset();

      while (BezierRefList::item* block = prims.take()) 
      {
	for (size_t i=0; i<block->size(); i++) 
	{
	  const BezierPrim& prim = block->at(i); 
	  const Vec3fa center = prim.center2();
	  const ssei bin = ssei(mapping.bin_unsafe(center));
	  
	  if (bin[dim] < pos) 
	  {
	    linfo_o.add(prim.bounds(),center);
	    if (likely(lblock->insert(prim))) continue; 
	    lblock = lprims_o.insert(alloc.malloc(threadIndex));
	    lblock->insert(prim);
	  } 
	  else 
	  {
	    rinfo_o.add(prim.bounds(),center);
	    if (likely(rblock->insert(prim))) continue;
	    rblock = rprims_o.insert(alloc.malloc(threadIndex));
	    rblock->insert(prim);
	  }
	}
	alloc.free(threadIndex,block);
      }
    }
    
    template<>
    inline void ObjectPartitionNew::Split::split<false>(size_t threadIndex, size_t threadCount, LockStepTaskScheduler* scheduler, 
					      PrimRefBlockAlloc<PrimRef>& alloc, 
					      PrimRefList& prims, 
					      PrimRefList& lprims_o, PrimInfo& linfo_o, 
					      PrimRefList& rprims_o, PrimInfo& rinfo_o) const
    {
      assert(valid());
      PrimRefList::item* lblock = lprims_o.insert(alloc.malloc(threadIndex));
      PrimRefList::item* rblock = rprims_o.insert(alloc.malloc(threadIndex));
      linfo_o.reset();
      rinfo_o.reset();

      size_t numLeft = 0; CentGeomBBox3fa leftBounds(empty);
      size_t numRight = 0; CentGeomBBox3fa rightBounds(empty);
      
      while (PrimRefList::item* block = prims.take()) 
      {
	for (size_t i=0; i<block->size(); i++) 
	{
	  const PrimRef& prim = block->at(i); 
	  const Vec3fa center = center2(prim.bounds());
	  const ssei bin = ssei(mapping.bin_unsafe(center));

	  if (bin[dim] < pos) 
	  {
	    leftBounds.extend(prim.bounds()); numLeft++;
	    //linfo_o.add(prim.bounds(),center);
	    //if (++lblock->num > PrimRefBlock::blockSize)
	    //lblock = lprims_o.insert(alloc.malloc(threadIndex));
	    if (likely(lblock->insert(prim))) continue; 
	    lblock = lprims_o.insert(alloc.malloc(threadIndex));
	    lblock->insert(prim);
	  } 
	  else 
	  {
	    rightBounds.extend(prim.bounds()); numRight++;
	    //rinfo_o.add(prim.bounds(),center);
	    //if (++rblock->num > PrimRefBlock::blockSize)
	    //rblock = rprims_o.insert(alloc.malloc(threadIndex));
	    if (likely(rblock->insert(prim))) continue;
	    rblock = rprims_o.insert(alloc.malloc(threadIndex));
	    rblock->insert(prim);
	  }
	}
	alloc.free(threadIndex,block);
      }

      linfo_o.add(leftBounds.geomBounds,leftBounds.centBounds,numLeft);
      rinfo_o.add(rightBounds.geomBounds,rightBounds.centBounds,numRight);
    }
  }
}
