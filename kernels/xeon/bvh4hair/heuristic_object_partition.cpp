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
  __forceinline ObjectPartition::Mapping::Mapping(const BBox3fa& centBounds) 
  {
    const ssef diag = (ssef) centBounds.size();
    scale = select(diag != 0.0f,rcp(diag) * ssef(BINS * 0.99f),ssef(0.0f));
    ofs  = (ssef) centBounds.lower;
  }
  
  __forceinline Vec3ia ObjectPartition::Mapping::bin(const Vec3fa& p) const 
  {
    const ssei i = floori((ssef(p)-ofs)*scale);
#if 0
    assert(i[0] >=0 && i[0] < BINS);
    assert(i[1] >=0 && i[1] < BINS);
    assert(i[2] >=0 && i[2] < BINS);
    return i;
#else
    return Vec3ia(clamp(i,ssei(0),ssei(BINS-1)));
#endif
  }

  __forceinline Vec3ia ObjectPartition::Mapping::bin_unsafe(const Vec3fa& p) const {
    return Vec3ia(floori((ssef(p)-ofs)*scale));
  }
    
  __forceinline bool ObjectPartition::Mapping::invalid(const int dim) const {
    return scale[dim] == 0.0f;
  }

  __forceinline ObjectPartition::BinInfo::BinInfo() 
  {
    for (size_t i=0; i<BINS; i++) {
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

  __forceinline void ObjectPartition::BinInfo::bin(BezierRefList& prims, const Mapping& mapping)
  {
    BezierRefList::iterator i=prims;
    while (BezierRefList::item* block = i.next())
      bin(block->base(),block->size(),mapping);
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

  __forceinline void ObjectPartition::BinInfo::bin(PrimRefList& prims, const Mapping& mapping)
  {
    PrimRefList::iterator i=prims;
    while (PrimRefList::item* block = i.next())
      bin(block->base(),block->size(),mapping);
  }

  __forceinline void ObjectPartition::BinInfo::merge (const BinInfo& other)
  {
    for (size_t i=0; i<BINS; i++) 
    {
      counts[i] += other.counts[i];
      bounds[i][0].extend(other.bounds[i][0]);
      bounds[i][1].extend(other.bounds[i][1]);
      bounds[i][2].extend(other.bounds[i][2]);
    }
  }
  
  __forceinline ObjectPartition::Split ObjectPartition::BinInfo::best(const Mapping& mapping)
  {
    /* sweep from right to left and compute parallel prefix of merged bounds */
    ssef rAreas[BINS];
    ssei rCounts[BINS];
    ssei count = 0; BBox3fa bx = empty; BBox3fa by = empty; BBox3fa bz = empty;
    for (size_t i=BINS-1; i>0; i--)
    {
      count += counts[i];
      rCounts[i] = count;
      bx.extend(bounds[i][0]); rAreas[i][0] = halfArea(bx);
      by.extend(bounds[i][1]); rAreas[i][1] = halfArea(by);
      bz.extend(bounds[i][2]); rAreas[i][2] = halfArea(bz);
    }
    
    /* sweep from left to right and compute SAH */
    ssei ii = 1; ssef vbestSAH = pos_inf; ssei vbestPos = 0;
    count = 0; bx = empty; by = empty; bz = empty;
    for (size_t i=1; i<BINS; i++, ii+=1)
    {
      count += counts[i-1];
      bx.extend(bounds[i-1][0]); float Ax = halfArea(bx);
      by.extend(bounds[i-1][1]); float Ay = halfArea(by);
      bz.extend(bounds[i-1][2]); float Az = halfArea(bz);
      const ssef lArea = ssef(Ax,Ay,Az,Az);
      const ssef rArea = rAreas[i];
      const ssei lCount = blocks(count);
      const ssei rCount = blocks(rCounts[i]);
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
  const ObjectPartition::Split ObjectPartition::find<false>(size_t threadIndex, size_t threadCount, BezierRefList& prims)
  {
    /* calculate geometry and centroid bounds */
    BBox3fa centBounds = empty;
    BBox3fa geomBounds = empty;
    for (BezierRefList::block_iterator_unsafe i = prims; i; i++) {
      geomBounds.extend(i->bounds());
      centBounds.extend(i->center());
    }

    BinInfo binner;
    const Mapping mapping(centBounds);
    binner.bin(prims,mapping);
    return binner.best(mapping);
  }

  ObjectPartition::TaskBinBezierParallel::TaskBinBezierParallel(size_t threadIndex, size_t threadCount, BezierRefList& prims) 
    : iter0(prims), iter1(prims), geomBounds(empty), centBounds(empty)
  {
    /* parallel calculation of centroid bounds */
    size_t numTasks = min(maxTasks,threadCount);
    TaskScheduler::executeTask(threadIndex,numTasks,_task_bound_parallel,this,numTasks,"build::task_bound_parallel");

    /* parallel binning */
    new (&mapping) Mapping(centBounds);
    TaskScheduler::executeTask(threadIndex,numTasks,_task_bin_parallel,this,numTasks,"build::task_bin_parallel");

    /* reduction of bin informations */
    BinInfo bins = binners[0];
    for (size_t i=1; i<numTasks; i++)
      bins.merge(binners[i]);

    /* calculation of best split */
    split = bins.best(mapping);
  }

  void ObjectPartition::TaskBinBezierParallel::task_bound_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    BBox3fa centBounds = empty;
    BBox3fa geomBounds = empty;
    while (BezierRefList::item* block = iter0.next()) 
    {
      for (size_t i=0; i<block->size(); i++) {
	geomBounds.extend(block->at(i).bounds());
	centBounds.extend(block->at(i).center());
      }
    }
    this->centBounds.extend_atomic(centBounds);
    this->geomBounds.extend_atomic(geomBounds);
  }

  void ObjectPartition::TaskBinBezierParallel::task_bin_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    while (BezierRefList::item* block = iter1.next())
      binners[taskIndex].bin(block->base(),block->size(),mapping);
  }

  template<>
  const ObjectPartition::Split ObjectPartition::find<true>(size_t threadIndex, size_t threadCount, BezierRefList& prims) {
    return TaskBinBezierParallel(threadIndex,threadCount,prims).split;
  }

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
	const Vec3fa center = prim.center();
	const ssei bin = ssei(mapping.bin_unsafe(center));

        if (bin[dim] < pos) 
        {
	  linfo_o.add(prim.bounds(),prim.center());
	  if (likely(lblock->insert(prim))) continue; 
          lblock = lprims_o.insert(alloc.malloc(threadIndex));
          lblock->insert(prim);
        } 
        else 
        {
          rinfo_o.add(prim.bounds(),prim.center());
          if (likely(rblock->insert(prim))) continue;
          rblock = rprims_o.insert(alloc.malloc(threadIndex));
          rblock->insert(prim);
        }
      }
      alloc.free(threadIndex,block);
    }
  }

  ObjectPartition::TaskSplitBezierParallel::TaskSplitBezierParallel(size_t threadIndex, size_t threadCount, const Split* split, PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& prims, 
							BezierRefList& lprims_o, PrimInfo& linfo_o, BezierRefList& rprims_o, PrimInfo& rinfo_o)
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

  void ObjectPartition::TaskSplitBezierParallel::task_split_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    split->split(threadIndex,threadCount,alloc,prims,lprims_o,linfos[taskIndex],rprims_o,rinfos[taskIndex]);
  }

  template<>
  void ObjectPartition::Split::split<true>(size_t threadIndex, size_t threadCount, 
					   PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& prims, 
					   BezierRefList& lprims_o, PrimInfo& linfo_o, 
					   BezierRefList& rprims_o, PrimInfo& rinfo_o) const
  {
    TaskSplitBezierParallel(threadIndex,threadCount,this,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o);
  }
}
