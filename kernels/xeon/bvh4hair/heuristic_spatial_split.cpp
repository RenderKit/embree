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

#include "heuristic_spatial_split.h"

namespace embree
{
  namespace isa
  {
    __forceinline SpatialSplit::Mapping::Mapping(const PrimInfo& pinfo) 
    {
      const ssef diag = (ssef) pinfo.geomBounds.size();
      scale = select(diag != 0.0f,rcp(diag) * ssef(BINS * 0.99f),ssef(0.0f));
      ofs  = (ssef) pinfo.geomBounds.lower;
    }
    
    __forceinline ssei SpatialSplit::Mapping::bin(const Vec3fa& p) const 
    {
      const ssei i = floori((ssef(p)-ofs)*scale);
#if 1
      assert(i[0] >=0 && i[0] < BINS);
      assert(i[1] >=0 && i[1] < BINS);
      assert(i[2] >=0 && i[2] < BINS);
      return i;
#else
      return clamp(i,ssei(0),ssei(BINS-1));
#endif
    }
    
    __forceinline float SpatialSplit::Mapping::pos(const int bin, const int dim) const {
      return float(bin)/scale[dim]+ofs[dim];
    }
    
    __forceinline bool SpatialSplit::Mapping::invalid(const int dim) const {
      return scale[dim] == 0.0f;
    }
    
    __forceinline SpatialSplit::BinInfo::BinInfo()
    {
      for (size_t i=0; i<BINS; i++) {
	bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = empty;
	numBegin[i] = numEnd[i] = 0;
      }
    }
    
    __forceinline void SpatialSplit::BinInfo::bin (const Bezier1* prims, size_t N, const PrimInfo& pinfo, const Mapping& mapping)
    {
      for (size_t i=0; i<N; i++)
      {
	const ssei bin0 = mapping.bin(min(prims[i].p0,prims[i].p3));
	const ssei bin1 = mapping.bin(max(prims[i].p0,prims[i].p3));
	
	for (size_t dim=0; dim<3; dim++) 
	{
	  size_t bin;
	  Bezier1 curve = prims[i];
	  for (bin=bin0[dim]; bin<bin1[dim]; bin++)
	  {
	    const float pos = mapping.pos(bin+1,dim);
	    Bezier1 bincurve,restcurve; 
	    if (curve.split(dim,pos,bincurve,restcurve)) {
	      bounds[bin][dim].extend(bincurve.bounds());
	      curve = restcurve;
	    }
	  }
	  numBegin[bin0[dim]][dim]++;
	  numEnd  [bin1[dim]][dim]++;
	  bounds  [bin][dim].extend(curve.bounds());
	}
      }
    }
    
    __forceinline void SpatialSplit::BinInfo::bin(BezierRefList& prims, const PrimInfo& pinfo, const Mapping& mapping)
    {
      BezierRefList::iterator i=prims;
      while (BezierRefList::item* block = i.next())
	bin(block->base(),block->size(),pinfo,mapping);
    }
    
    __forceinline void SpatialSplit::BinInfo::merge (const BinInfo& other)
    {
      for (size_t i=0; i<BINS; i++) 
      {
	numBegin[i] += other.numBegin[i];
	numEnd[i] += other.numEnd[i];
	bounds[i][0].extend(other.bounds[i][0]);
	bounds[i][1].extend(other.bounds[i][1]);
	bounds[i][2].extend(other.bounds[i][2]);
      }
    }
    
    __forceinline SpatialSplit::Split SpatialSplit::BinInfo::best(BezierRefList& prims, const PrimInfo& pinfo, const Mapping& mapping)
    {
      /* sweep from right to left and compute parallel prefix of merged bounds */
      ssef rAreas[BINS];
      ssei rCounts[BINS];
      ssei count = 0; BBox3fa bx = empty; BBox3fa by = empty; BBox3fa bz = empty;
      for (size_t i=BINS-1; i>0; i--)
      {
	count += numEnd[i];
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
	count += numBegin[i-1];
	bx.extend(bounds[i-1][0]); float Ax = halfArea(bx);
	by.extend(bounds[i-1][1]); float Ay = halfArea(by);
	bz.extend(bounds[i-1][2]); float Az = halfArea(bz);
	const ssef lArea = ssef(Ax,Ay,Az,Az);
	const ssef rArea = rAreas[i];
	const ssei lCount = blocks(count);
	const ssei rCount = blocks(rCounts[i]);
	const ssef sah = lArea*ssef(lCount) + rArea*ssef(rCount);
	vbestPos  = select(sah < vbestSAH,ii ,vbestPos);
	vbestSAH  = select(sah < vbestSAH,sah,vbestSAH);
      }
      
      /* find best dimension */
      float bestSAH = inf;
      int   bestDim = -1;
      int   bestPos = 0.0f;
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
      
      /* return invalid split if no split found */
      if (bestDim == -1) 
	return Split(inf,-1,0.0f,mapping);

      /* compute bounds of left and right side */
      size_t lnum = 0, rnum = 0;
      BBox3fa lbounds = empty, rbounds = empty;
      for (size_t i=0; i<bestPos; i++) { lnum+=numBegin[i][bestDim]; lbounds.extend(bounds[i][bestDim]); }
      for (size_t i=bestPos; i<BINS; i++) { rnum+=numEnd[i][bestDim]; rbounds.extend(bounds[i][bestDim]); }
      
      /* return invalid split if no progress made */
      if (lnum == 0 || rnum == 0) 
	return Split(inf,-1,0.0f,mapping);
      
      /* calculate SAH and return best found split */
      float sah = float(lnum)*halfArea(lbounds) + float(rnum)*halfArea(rbounds);
      return Split(sah,bestDim,bestPos,mapping);
    }
    
    template<>
    const SpatialSplit::Split SpatialSplit::find<false>(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo)
    {
      BinInfo binner;
      Mapping mapping(pinfo);
      binner.bin(prims,pinfo,mapping);
      return binner.best(prims,pinfo,mapping);
    }
    
    SpatialSplit::TaskBinParallel::TaskBinParallel(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo, const Mapping& mapping) 
      : iter(prims), pinfo(pinfo), mapping(mapping)
    {
      /* parallel binning */
      size_t numTasks = min(maxTasks,threadCount);
      TaskScheduler::executeTask(threadIndex,numTasks,_task_bin_parallel,this,numTasks,"build::task_bin_parallel");
      
      /* reduction of bin informations */
      BinInfo bins = binners[0];
      for (size_t i=1; i<numTasks; i++)
	bins.merge(binners[i]);
      
      /* calculation of best split */
      split = bins.best(prims,pinfo,mapping);
    }
    
    void SpatialSplit::TaskBinParallel::task_bin_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
    {
      while (BezierRefList::item* block = iter.next())
	binners[taskIndex].bin(block->base(),block->size(),pinfo,mapping);
    }
    
    template<>
    const SpatialSplit::Split SpatialSplit::find<true>(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo) 
    {
      const Mapping mapping(pinfo);
      return TaskBinParallel(threadIndex,threadCount,prims,pinfo,mapping).split;
    }
    
    template<>
    void SpatialSplit::Split::split<false>(size_t threadIndex,size_t threadCount,  PrimRefBlockAlloc<Bezier1>& alloc, 
					   BezierRefList& prims, 
					   BezierRefList& lprims_o, PrimInfo& linfo_o, 
					   BezierRefList& rprims_o, PrimInfo& rinfo_o) const
    {
      /* sort each curve to left, right, or left and right */
      BezierRefList::item* lblock = lprims_o.insert(alloc.malloc(threadIndex));
      BezierRefList::item* rblock = rprims_o.insert(alloc.malloc(threadIndex));
      
      while (BezierRefList::item* block = prims.take()) 
      {
	for (size_t i=0; i<block->size(); i++) 
	{
	  const Bezier1& prim = block->at(i);
	  const int bin0 = mapping.bin(min(prim.p0,prim.p3))[dim];
	  const int bin1 = mapping.bin(max(prim.p0,prim.p3))[dim];
	  
	  /* sort to the left side */
	  if (bin0 < pos && bin1 < pos)
	  {
	    linfo_o.add(prim.bounds(),prim.center());
	    if (likely(lblock->insert(prim))) continue; 
	    lblock = lprims_o.insert(alloc.malloc(threadIndex));
	    lblock->insert(prim);
	    continue;
	  }
	  
	  /* sort to the right side */
	  if (bin0 >= pos && bin1 >= pos)
	  {
	    rinfo_o.add(prim.bounds(),prim.center());
	    if (likely(rblock->insert(prim))) continue;
	    rblock = rprims_o.insert(alloc.malloc(threadIndex));
	    rblock->insert(prim);
	    continue;
	  }
	  
	  /* split and sort to left and right */
	  Bezier1 left,right;
	  float fpos = mapping.pos(pos,dim);
	  if (prim.split(dim,fpos,left,right)) 
	  {
	    linfo_o.add(left.bounds(),left.center());
	    if (!lblock->insert(left)) {
	      lblock = lprims_o.insert(alloc.malloc(threadIndex));
	      lblock->insert(left);
	    }
	    rinfo_o.add(right.bounds(),right.center());
	    if (!rblock->insert(right)) {
	      rblock = rprims_o.insert(alloc.malloc(threadIndex));
	      rblock->insert(right);
	    }
	    continue;
	  }
	  
	  /* insert to left side as fallback */
	  linfo_o.add(prim.bounds(),prim.center());
	  if (!lblock->insert(prim)) {
	    lblock = lprims_o.insert(alloc.malloc(threadIndex));
	    lblock->insert(prim);
	  }
	}
	alloc.free(threadIndex,block);
      }
    }
    
    SpatialSplit::TaskSplitParallel::TaskSplitParallel(size_t threadIndex, size_t threadCount, const Split* split, 
						       PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& prims, 
						       BezierRefList& lprims_o, PrimInfo& linfo_o, 
						       BezierRefList& rprims_o, PrimInfo& rinfo_o)
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
    
    void SpatialSplit::TaskSplitParallel::task_split_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
    {
      split->split<false>(threadIndex,threadCount,alloc,prims,lprims_o,linfos[taskIndex],rprims_o,rinfos[taskIndex]);
    }
    
    template<>
    void SpatialSplit::Split::split<true>(size_t threadIndex, size_t threadCount, 
					  PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& prims, 
					  BezierRefList& lprims_o, PrimInfo& linfo_o, 
					  BezierRefList& rprims_o, PrimInfo& rinfo_o) const
    {
      TaskSplitParallel(threadIndex,threadCount,this,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o);
    }
  }
}

  
