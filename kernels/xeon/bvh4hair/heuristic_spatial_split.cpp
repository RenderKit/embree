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
  const SpatialSplit SpatialSplit::find(size_t threadIndex, BezierRefList& prims, const PrimInfo& pinfo)
  {
    /* calculate binning function */
    const ssef ofs  = (ssef) pinfo.geomBounds.lower;
    const ssef diag = (ssef) pinfo.geomBounds.size();
    const ssef scale = select(diag != 0.0f,rcp(diag) * ssef(BINS * 0.99f),ssef(0.0f));

    /* initialize bins */
    BBox3fa bounds[BINS][4];
    ssei    numBegin[BINS];
    ssei    numEnd[BINS];
    for (size_t i=0; i<BINS; i++) {
      bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = empty;
      numBegin[i] = numEnd[i] = 0;
    }
 
    /* perform binning of curves */
    for (BezierRefList::block_iterator_unsafe i = prims; i; i++)
    {
      const ssei bin0 = clamp(floori((ssef(i->p0)-ofs)*scale),ssei(0),ssei(BINS-1));
      //const ssei bin0 = floori((ssef(v0)-ofs)*scale);
      assert(bin0[0] >=0 && bin0[0] < BINS);
      assert(bin0[1] >=0 && bin0[1] < BINS);
      assert(bin0[2] >=0 && bin0[2] < BINS);
      const ssei bin1 = clamp(floori((ssef(i->p3)-ofs)*scale),ssei(0),ssei(BINS-1));
      //const ssei bin1 = floori((ssef(v1)-ofs)*scale);
      assert(bin1[0] >=0 && bin1[0] < BINS);
      assert(bin1[1] >=0 && bin1[1] < BINS);
      assert(bin1[2] >=0 && bin1[2] < BINS);
      const ssei startbin = min(bin0,bin1);
      const ssei endbin   = max(bin0,bin1);

      for (size_t dim=0; dim<3; dim++) 
      {
        size_t bin;
        Bezier1 curve = *i;
        for (bin=startbin[dim]; bin<endbin[dim]; bin++)
        {
          const float pos = float(bin+1)/scale[dim]+ofs[dim];
          Bezier1 bincurve,restcurve; 
          if (curve.split(dim,pos,bincurve,restcurve)) {
            const BBox3fa cbounds = bincurve.bounds();
            bounds[bin][dim].extend(cbounds);
            curve = restcurve;
          }
        }
        numBegin[startbin[dim]][dim]++;
        numEnd  [endbin  [dim]][dim]++;
        const BBox3fa cbounds = curve.bounds();
        bounds[bin][dim].extend(cbounds);
      }
    }
    
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
    ssei ii = 1; ssef bestSAH = pos_inf; ssei bestPos = 0; ssei bestLeft = 0; ssei bestRight = 0;
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
      bestPos  = select(sah < bestSAH,ii ,bestPos);
      bestLeft = select(sah < bestSAH,count,bestLeft);
      bestRight= select(sah < bestSAH,rCounts[i],bestRight);
      bestSAH  = select(sah < bestSAH,sah,bestSAH);
    }
    
    /* find best dimension */
    SpatialSplit split;
    split.ofs = ofs;
    split.scale = scale;
    split.cost = inf;
    split.dim = -1;
    split.pos = 0.0f;

    float bestCost = inf;
    for (size_t dim=0; dim<3; dim++) 
    {
      /* ignore zero sized dimensions */
      if (unlikely(scale[dim] == 0.0f)) 
        continue;
      
      /* test if this is a better dimension */
      if (bestSAH[dim] < bestCost && bestPos[dim] != 0) {
        split.dim = dim;
        split.pos = bestPos[dim]/scale[dim]+ofs[dim];
        split.cost = bestSAH[dim];
        bestCost = bestSAH[dim];
      }
    }

    /* compute bounds of left and right side */
    if (split.dim == -1) {
      split.cost = inf;
      return split;
    }

    /* calculate bounding box of left and right side */
    BBox3fa lbounds = empty, rbounds = empty;
    size_t lnum = 0, rnum = 0;

    for (BezierRefList::block_iterator_unsafe i = prims; i; i++)
    {
      const float p0p = i->p0[split.dim];
      const float p3p = i->p3[split.dim];

      /* sort to the left side */
      if (p0p <= split.pos && p3p <= split.pos) {
        lbounds.extend(i->bounds()); lnum++;
        continue;
      }
      
      /* sort to the right side */
      if (p0p >= split.pos && p3p >= split.pos) {
        rbounds.extend(i->bounds()); rnum++;
        continue;
      }

      Bezier1 left,right; 
      if (i->split(split.dim,split.pos,left,right)) {
        lbounds.extend(left .bounds()); lnum++;
        rbounds.extend(right.bounds()); rnum++;
        continue;
      }
      
      lbounds.extend(i->bounds()); lnum++;
    }

    if (lnum == 0 || rnum == 0) {
      split.cost = inf;
      return split;
    }

    split.cost = float(lnum)*halfArea(lbounds) + float(rnum)*halfArea(rbounds);
    return split;

#if 0
    {
    size_t lnum = 0, rnum = 0;
    BBox3fa lbounds = empty, rbounds = empty;
    for (size_t i=0; i<split.pos; i++) { lnum+=numBegin[i][split.dim]; lbounds.extend(bounds[i][split.dim]); }
    for (size_t i=split.pos; i<BINS; i++) { rnum+=numEnd[i][split.dim]; rbounds.extend(bounds[i][split.dim]); }
    split.cost = float(lnum)*halfArea(lbounds) + float(rnum)*halfArea(rbounds);
    split.numReplications = split.num0 + split.num1 - pinfo.size();
    assert(split.numReplications >= 0);
    }
    return split;
#endif
  }
      
  void SpatialSplit::split(size_t threadIndex, PrimRefBlockAlloc<Bezier1>& alloc, 
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
        const float p0p = prim.p0[dim];
        const float p3p = prim.p3[dim];

        /* sort to the left side */
        if (p0p <= pos && p3p <= pos)
        {
	  linfo_o.add(prim.bounds(),prim.center());
          if (likely(lblock->insert(prim))) continue; 
          lblock = lprims_o.insert(alloc.malloc(threadIndex));
          lblock->insert(prim);
          continue;
        }

        /* sort to the right side */
        if (p0p >= pos && p3p >= pos)
        {
	  rinfo_o.add(prim.bounds(),prim.center());
          if (likely(rblock->insert(prim))) continue;
          rblock = rprims_o.insert(alloc.malloc(threadIndex));
          rblock->insert(prim);
          continue;
        }

        /* split and sort to left and right */
        Bezier1 left,right;
        if (prim.split(dim,pos,left,right)) 
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
}

