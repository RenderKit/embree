// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "heuristic_object_partition.h"

namespace embree
{
  //////////////////////////////////////////////////////////////////////////////
  //                        Bin Mapping                                       //
  //////////////////////////////////////////////////////////////////////////////

  __forceinline ObjectPartition::Mapping::Mapping(const PrimInfo& pinfo) 
  {
    num = min(maxBins,size_t(4.0f + 0.05f*pinfo.size()));
    const ssef diag = (ssef) pinfo.centBounds.size();
    scale = select(diag != 0.0f,rcp(diag) * ssef(0.99f*num),ssef(0.0f));
    ofs  = (ssef) pinfo.centBounds.lower;
  }
  
  __forceinline Vec3ia ObjectPartition::Mapping::bin(const Vec3fa& p) const 
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

  __forceinline Vec3ia ObjectPartition::Mapping::bin_unsafe(const Vec3fa& p) const {
    return Vec3ia(floori((ssef(p)-ofs)*scale));
  }
    
  __forceinline bool ObjectPartition::Mapping::invalid(const int dim) const {
    return scale[dim] == 0.0f;
  }

  //////////////////////////////////////////////////////////////////////////////
  //                             Binning                                      //
  //////////////////////////////////////////////////////////////////////////////

  __forceinline ObjectPartition::BinInfo::BinInfo() 
  {
    for (size_t i=0; i<maxBins; i++) {
      bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = empty;
      counts[i] = 0;
    }
  }

  __forceinline void ObjectPartition::BinInfo::bin (const Bezier1* prims, size_t N, const Mapping& mapping)
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

  __forceinline void ObjectPartition::BinInfo::bin (const PrimRef* prims, size_t num, const Mapping& mapping)
  {
    if (num == 0) return;
    
    size_t i; for (i=0; i<num-1; i+=2)
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

  __forceinline void ObjectPartition::BinInfo::bin(BezierRefList& prims, const Mapping& mapping)
  {
    BezierRefList::iterator i=prims;
    while (BezierRefList::item* block = i.next())
      bin(block->base(),block->size(),mapping);
  }

  __forceinline void ObjectPartition::BinInfo::bin(PrimRefList& prims, const Mapping& mapping)
  {
    PrimRefList::iterator i=prims;
    while (PrimRefList::item* block = i.next())
      bin(block->base(),block->size(),mapping);
  }

  __forceinline void ObjectPartition::BinInfo::merge (const BinInfo& other)
  {
    for (size_t i=0; i<maxBins; i++) 
    {
      counts[i] += other.counts[i];
      bounds[i][0].extend(other.bounds[i][0]);
      bounds[i][1].extend(other.bounds[i][1]);
      bounds[i][2].extend(other.bounds[i][2]);
    }
  }
  
  __forceinline ObjectPartition::Split ObjectPartition::BinInfo::best(const Mapping& mapping, const size_t blocks_shift)
  {
    //PING;

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
      const ssei lCount = (count     +blocks_add) >> blocks_shift; //blocks(count);
      const ssei rCount = (rCounts[i]+blocks_add) >> blocks_shift; //blocks(rCounts[i]);
      const ssef sah = lArea*ssef(lCount) + rArea*ssef(rCount);
      vbestPos = select(sah < vbestSAH,ii ,vbestPos);
      vbestSAH = select(sah < vbestSAH,sah,vbestSAH);
    }
    
    /* find best dimension */
    float bestSAH = inf;
    int   bestDim = -1;
    int   bestPos = 0;
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

    return ObjectPartition::Split(bestSAH,bestDim,bestPos,mapping);
  }

  template<>
  const ObjectPartition::Split ObjectPartition::find<false>(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize)
  {
    BinInfo binner;
    const Mapping mapping(pinfo);
    binner.bin(prims,mapping);
    return binner.best(mapping,logBlockSize);
  }

  template<>
  const ObjectPartition::Split ObjectPartition::find<false>(size_t threadIndex, size_t threadCount, PrimRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize)
  {
    BinInfo binner;
    const Mapping mapping(pinfo);
    binner.bin(prims,mapping);
    return binner.best(mapping,logBlockSize);
  }

  const ObjectPartition::Split ObjectPartition::find(PrimRef *__restrict__ const prims, const size_t begin, const size_t end, const PrimInfo& pinfo, const size_t logBlockSize)
  {
   BinInfo binner;
   const Mapping mapping(pinfo);
   binner.bin(prims+begin,end-begin,mapping);
   return binner.best(mapping,logBlockSize);
  }

  //////////////////////////////////////////////////////////////////////////////
  //                         Parallel Binning                                 //
  //////////////////////////////////////////////////////////////////////////////

  template<typename List>
  ObjectPartition::TaskBinParallel<List>::TaskBinParallel(size_t threadIndex, size_t threadCount, List& prims, const PrimInfo& pinfo, const size_t logBlockSize) 
    : iter(prims)
  {
   /* parallel binning */			
    size_t numTasks = min(maxTasks,threadCount);
    new (&mapping) Mapping(pinfo);
    TaskScheduler::executeTask(threadIndex,numTasks,_task_bin_parallel,this,numTasks,"build::task_bin_parallel");

    /* reduction of bin informations */
    BinInfo bins = binners[0];
    for (size_t i=1; i<numTasks; i++)
      bins.merge(binners[i]);

    /* calculation of best split */
    split = bins.best(mapping,logBlockSize);
  }

  template<typename List>
  void ObjectPartition::TaskBinParallel<List>::task_bin_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    while (typename List::item* block = iter.next())
      binners[taskIndex].bin(block->base(),block->size(),mapping);
  }

  template<>
  const ObjectPartition::Split ObjectPartition::find<true>(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize) {
    return TaskBinParallel<BezierRefList>(threadIndex,threadCount,prims,pinfo,logBlockSize).split;
  }

  template<>
  const ObjectPartition::Split ObjectPartition::find<true>(size_t threadIndex, size_t threadCount, PrimRefList& prims, const PrimInfo& pinfo, const size_t logBlockSize) {
    return TaskBinParallel<PrimRefList>(threadIndex,threadCount,prims,pinfo,logBlockSize).split;
  }

  //////////////////////////////////////////////////////////////////////////////
  //                             Splitting                                    //
  //////////////////////////////////////////////////////////////////////////////

  template<>
  void ObjectPartition::Split::split<false>(size_t threadIndex, size_t threadCount, 
					    PrimRefBlockAlloc<Bezier1>& alloc, 
					    BezierRefList& prims, 
					    BezierRefList& lprims_o, PrimInfo& linfo_o, 
					    BezierRefList& rprims_o, PrimInfo& rinfo_o) const
  {
    BezierRefList::item* lblock = lprims_o.insert(alloc.malloc(threadIndex));
    BezierRefList::item* rblock = rprims_o.insert(alloc.malloc(threadIndex));
    
    while (BezierRefList::item* block = prims.take()) 
    {
      for (size_t i=0; i<block->size(); i++) 
      {
        const Bezier1& prim = block->at(i); 
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
  void ObjectPartition::Split::split<false>(size_t threadIndex, size_t threadCount, 
					    PrimRefBlockAlloc<PrimRef>& alloc, 
					    PrimRefList& prims, 
					    PrimRefList& lprims_o, PrimInfo& linfo_o, 
					    PrimRefList& rprims_o, PrimInfo& rinfo_o) const
  {
    PrimRefList::item* lblock = lprims_o.insert(alloc.malloc(threadIndex));
    PrimRefList::item* rblock = rprims_o.insert(alloc.malloc(threadIndex));
    
    while (PrimRefList::item* block = prims.take()) 
    {
      for (size_t i=0; i<block->size(); i++) 
      {
        const PrimRef& prim = block->at(i); 
	const Vec3fa center = center2(prim.bounds());
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

  template<typename PrimRef>
    __forceinline bool lt_split(const PrimRef *__restrict__ const aabb,
                                const unsigned int dim,
                                const float &c,
                                const float &s,
                                const int bestSplit) // FIXME: has to be singed int!!!!!!!!!
    {
      const ssef b_min(aabb->lower[dim]);
      const ssef b_max(aabb->upper[dim]);
      const ssef centroid_2 = b_min + b_max;
      const ssei binID = floori((centroid_2 - c)*s);
      return extract<0>(binID) < bestSplit;    
    }
    
    
    template<typename PrimRef>
    __forceinline bool ge_split(const PrimRef *__restrict__ const aabb,
                                const unsigned int dim,
                                const float &c,
                                const float &s,
                                const int bestSplit) // FIXME: has to be singed int!!!!!!!!!
    {
      const ssef b_min(aabb->lower[dim]);
      const ssef b_max(aabb->upper[dim]);
      const ssef centroid_2 = b_min + b_max;
      const ssei binID = floori((centroid_2 - c)*s);
      return extract<0>(binID) >= bestSplit;    
    }

  void ObjectPartition::Split::partition(PrimRef *__restrict__ const prims,
					 const size_t begin, const size_t end,
					 BuildRecord& left, BuildRecord& right)
  {
    Centroid_Scene_AABB local_left; local_left.reset();
    Centroid_Scene_AABB local_right; local_right.reset();
    
    assert(begin <= end);
    PrimRef* l = prims + begin;
    PrimRef* r = prims + end - 1;
    
    const float c = mapping.ofs[dim];
    const float s = mapping.scale[dim];
    const int bestSplitDim = dim;
    const int bestSplit = pos;
    
    ssef left_centroidMinAABB = (ssef) local_left.centroid2.lower;
    ssef left_centroidMaxAABB = (ssef) local_left.centroid2.upper;
    ssef left_sceneMinAABB    = (ssef) local_left.geometry.lower;
    ssef left_sceneMaxAABB    = (ssef) local_left.geometry.upper;
    
    ssef right_centroidMinAABB = (ssef) local_right.centroid2.lower;
    ssef right_centroidMaxAABB = (ssef) local_right.centroid2.upper;
    ssef right_sceneMinAABB    = (ssef) local_right.geometry.lower;
    ssef right_sceneMaxAABB    = (ssef) local_right.geometry.upper;
    
    while(1)
    {
      while (likely(l <= r && lt_split(l,bestSplitDim,c,s,bestSplit))) {
	const ssef b_min = load4f((float*)&l->lower);
	const ssef b_max = load4f((float*)&l->upper);
	const ssef centroid2 = b_min+b_max;
	left_centroidMinAABB = min(left_centroidMinAABB,centroid2);
	left_centroidMaxAABB = max(left_centroidMaxAABB,centroid2);
	left_sceneMinAABB    = min(left_sceneMinAABB,b_min);
	left_sceneMaxAABB    = max(left_sceneMaxAABB,b_max);
	++l;
      }
      while (likely(l <= r && ge_split(r,bestSplitDim,c,s,bestSplit))) {
	const ssef b_min = load4f((float*)&r->lower);
	const ssef b_max = load4f((float*)&r->upper);
	const ssef centroid2 = b_min+b_max;
	right_centroidMinAABB = min(right_centroidMinAABB,centroid2);
	right_centroidMaxAABB = max(right_centroidMaxAABB,centroid2);
	right_sceneMinAABB    = min(right_sceneMinAABB,b_min);
	right_sceneMaxAABB    = max(right_sceneMaxAABB,b_max);
	--r;
      }
      if (r<l) break;
      
      const ssef r_min = load4f((float*)&l->lower);
      const ssef r_max = load4f((float*)&l->upper);
      const ssef r_centroid2 = r_min+r_max;
      right_centroidMinAABB = min(right_centroidMinAABB,r_centroid2);
      right_centroidMaxAABB = max(right_centroidMaxAABB,r_centroid2);
      right_sceneMinAABB    = min(right_sceneMinAABB,r_min);
      right_sceneMaxAABB    = max(right_sceneMaxAABB,r_max);
      const ssef l_min = load4f((float*)&r->lower);
      const ssef l_max = load4f((float*)&r->upper);
      const ssef l_centroid2 = l_min+l_max;
      left_centroidMinAABB = min(left_centroidMinAABB,l_centroid2);
      left_centroidMaxAABB = max(left_centroidMaxAABB,l_centroid2);
      left_sceneMinAABB    = min(left_sceneMinAABB,l_min);
      left_sceneMaxAABB    = max(left_sceneMaxAABB,l_max);
      store4f((float*)&l->lower,l_min);
      store4f((float*)&l->upper,l_max);
      store4f((float*)&r->lower,r_min);
      store4f((float*)&r->upper,r_max);
      l++; r--;
    }
    
    local_left.centroid2.lower = (Vec3fa) left_centroidMinAABB;
    local_left.centroid2.upper = (Vec3fa) left_centroidMaxAABB;
    local_left.geometry.lower = (Vec3fa) left_sceneMinAABB;
    local_left.geometry.upper = (Vec3fa) left_sceneMaxAABB;
    
    local_right.centroid2.lower = (Vec3fa) right_centroidMinAABB;
    local_right.centroid2.upper = (Vec3fa) right_centroidMaxAABB;
    local_right.geometry.lower = (Vec3fa) right_sceneMinAABB;
    local_right.geometry.upper = (Vec3fa) right_sceneMaxAABB;
    
    unsigned int center = l - prims;
    left.init(local_left,begin,center);
    right.init(local_right,center,end);
    
    assert(area(left.bounds.geometry) >= 0.0f);
    assert(area(left.bounds.centroid2) >= 0.0f);
    assert(area(right.bounds.geometry) >= 0.0f);
    assert(area(right.bounds.centroid2) >= 0.0f);
    
    assert( prims + begin <= l && l <= prims + end);
    assert( prims + begin <= r && r <= prims + end);
    
    assert(l <= prims + end);
    assert(center == begin+split.numLeft);
  }
  
  template<typename Prim>
  ObjectPartition::TaskSplitParallel<Prim>::TaskSplitParallel(size_t threadIndex, size_t threadCount, const Split* split, PrimRefBlockAlloc<Prim>& alloc, List& prims, 
							      List& lprims_o, PrimInfo& linfo_o, List& rprims_o, PrimInfo& rinfo_o)
    : split(split), alloc(alloc), prims(prims), lprims_o(lprims_o), linfo_o(linfo_o), rprims_o(rprims_o), rinfo_o(rinfo_o)
  {
    /* parallel calculation of centroid bounds */
    size_t numTasks = min(maxTasks,threadCount);
    TaskScheduler::executeTask(threadIndex,numTasks,_task_split_parallel,this,numTasks,"build::task_split_parallel");

    /* reduction of bounding info */
    linfo_o = linfos[0];
    rinfo_o = rinfos[0];
    for (size_t i=1; i<numTasks; i++) {
      linfo_o.merge(linfos[i]);
      rinfo_o.merge(rinfos[i]);
    }
  }

  template<typename Prim>
  void ObjectPartition::TaskSplitParallel<Prim>::task_split_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    split->split(threadIndex,threadCount,alloc,prims,lprims_o,linfos[taskIndex],rprims_o,rinfos[taskIndex]);
  }

  template<>
  void ObjectPartition::Split::split<true>(size_t threadIndex, size_t threadCount, 
					   PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& prims, 
					   BezierRefList& lprims_o, PrimInfo& linfo_o, 
					   BezierRefList& rprims_o, PrimInfo& rinfo_o) const
  {
    TaskSplitParallel<Bezier1>(threadIndex,threadCount,this,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o);
  }

  template<>
  void ObjectPartition::Split::split<true>(size_t threadIndex, size_t threadCount, 
					   PrimRefBlockAlloc<PrimRef>& alloc, PrimRefList& prims, 
					   PrimRefList& lprims_o, PrimInfo& linfo_o, 
					   PrimRefList& rprims_o, PrimInfo& rinfo_o) const
  {
    TaskSplitParallel<PrimRef>(threadIndex,threadCount,this,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o);
  }
}
