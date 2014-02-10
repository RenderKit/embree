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

#include "bvh2hair.h"
#include "bvh2hair_builder.h"
#include "common/scene_quadratic_bezier_curves.h"

namespace embree
{
  BVH2HairBuilder::BVH2HairBuilder (BVH2Hair* bvh, Scene* scene)
    : scene(scene), minLeafSize(1), maxLeafSize(inf), bvh(bvh)
  {
    if (BVH2Hair::maxLeafBlocks < this->maxLeafSize) 
      this->maxLeafSize = BVH2Hair::maxLeafBlocks;
  }

  BVH2HairBuilder::~BVH2HairBuilder () {
    delete [] curves;
  }

  void BVH2HairBuilder::build(size_t threadIndex, size_t threadCount) 
  {
    /* fast path for empty BVH */
    size_t numPrimitives = scene->numCurves;
    bvh->init(numPrimitives);
    if (numPrimitives == 0) return;
    
    double t0 = 0.0;
    if (g_verbose >= 2) {
      std::cout << "building BVH2Hair<Bezier1> ... " << std::flush;
      t0 = getSeconds();
    }

    /* allocate temporary curve array */
    curves = new Bezier1[2*numPrimitives];

    /* create initial curve list */
    BBox3fa bounds = empty;
    size_t begin = 0, end = 0;
    for (size_t i=0; i<scene->size(); i++) 
    {
      Geometry* geom = scene->get(i);
      if (geom->type != QUADRATIC_BEZIER_CURVES) continue;
      if (!geom->isEnabled()) continue;
      QuadraticBezierCurvesScene::QuadraticBezierCurves* set = (QuadraticBezierCurvesScene::QuadraticBezierCurves*) geom;

      for (size_t j=0; j<set->numCurves; j++) {
        int ofs = set->curve(j);
        const Vec3fa& p0 = set->vertex(ofs+0); bounds.extend(p0);
        const Vec3fa& p1 = set->vertex(ofs+1); bounds.extend(p1);
        const Vec3fa& p2 = set->vertex(ofs+2); bounds.extend(p2);
        const Vec3fa& p3 = set->vertex(ofs+3); bounds.extend(p3);
        curves[end++] = Bezier1 (p0,p1,p2,p3,0,1,i,j);
      }
    }

    /* find best bounds for hair */
    NAABBox3fa naabb = bestSpace(curves,begin,end);
    //bvh->rbounds = naabb; // FIXME: remove
    //bvh->rbounds = bestSpace(curves,0,1);

    /* start recursive build */
    bvh->root = recurse(threadIndex,0,begin,end,naabb);
    bvh->bounds = bounds;

    if (g_verbose >= 2) {
      double t1 = getSeconds();
      std::cout << "[DONE]" << std::endl;
      std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(numPrimitives)/(t1-t0) << " Mprim/s" << std::endl;
      //std::cout << BVH2HairStatistics(bvh).str();
    }
  }

  const BVH2Hair::NAABBox3fa BVH2HairBuilder::bestSpace(Bezier1* curves, size_t begin, size_t end)
  {
    if (end-begin == 0)
      return NAABBox3fa(empty);

    BBox3fa bestBounds = empty;
    Vec3fa bestAxis = one;
    float bestArea = inf;
    for (size_t i=0; i<16; i++)
    {
      size_t k = begin + rand() % (end-begin);
      const Vec3fa axis = normalize(curves[k].p3-curves[k].p0);
      //PRINT(axis);
      const AffineSpace3f space = rcp(frame(axis));
      
      BBox3fa bounds = empty;
      float area = 0.0f;
      for (size_t j=begin; j<end; j++) {
        BBox3fa cbounds = empty;
        const Vec3fa p0 = xfmPoint(space,curves[j].p0); cbounds.extend(p0);
        const Vec3fa p1 = xfmPoint(space,curves[j].p1); cbounds.extend(p1);
        const Vec3fa p2 = xfmPoint(space,curves[j].p2); cbounds.extend(p2);
        const Vec3fa p3 = xfmPoint(space,curves[j].p3); cbounds.extend(p3);
        //PRINT(halfArea(cbounds));
        area += halfArea(cbounds);
        bounds.extend(cbounds);
      }

      if (area <= bestArea) {
        bestBounds = bounds;
        bestAxis = axis;
        bestArea = area;
      }
    }
    
    NAABBox3fa bounds(rcp(frame(bestAxis)),bestBounds);
    bounds.bounds.lower.w = bestArea;
    return bounds;
  }

  __forceinline BVH2HairBuilder::StrandSplit::StrandSplit (const NAABBox3fa& bounds0, const Vec3fa& axis0, const size_t num0,
                                                           const NAABBox3fa& bounds1, const Vec3fa& axis1, const size_t num1)
    : bounds0(bounds0), bounds1(bounds1), axis0(axis0), axis1(axis1), num0(num0), num1(num1) {}
  
  __forceinline const BVH2HairBuilder::StrandSplit BVH2HairBuilder::StrandSplit::find(Bezier1* curves, size_t begin, size_t end)
  {
    /* first try to split two hair strands */
    Vec3fa axis0 = normalize(curves[begin].p3-curves[begin].p0);
    float bestCos = 1.0f;
    size_t bestI = begin;
    for (size_t i=begin; i<end; i++) {
      Vec3fa axisi = normalize(curves[i].p3-curves[i].p0);
      float cos = abs(dot(axisi,axis0));
      if (cos < bestCos) { bestCos = cos; bestI = i; }
    }
    Vec3fa axis1 = normalize(curves[bestI].p3-curves[bestI].p0);

    //PRINT(axis0);
    //PRINT(axis1);
    
    /* partition the two strands */
    ssize_t left = begin, right = end;
    while (left < right) {
      const Vec3fa axisi = normalize(curves[left].p3-curves[left].p0);
      const float cos0 = abs(dot(axisi,axis0));
      const float cos1 = abs(dot(axisi,axis1));
      //PRINT(axisi);
      //PRINT3(cos0,cos1,cos0 > cos1);
      if (cos0 > cos1) left++;
      else std::swap(curves[left],curves[--right]);
    }
    //PRINT(left-begin);
    //PRINT(end-left);
    //PING;
    const NAABBox3fa naabb0 = bestSpace(curves,begin,left);
    PING;
    const NAABBox3fa naabb1 = bestSpace(curves,left, end );
    PING;
    return StrandSplit(naabb0,axis0,left-begin,
                       naabb1,axis1,end-left);
  }

  __forceinline size_t BVH2HairBuilder::StrandSplit::split(Bezier1* curves, size_t begin, size_t end) const
  {
    ssize_t left = begin, right = end;
    while (left < right) {
      const Vec3fa axisi = normalize(curves[left].p3-curves[left].p0);
      const float cos0 = abs(dot(axisi,axis0));
      const float cos1 = abs(dot(axisi,axis1));
      if (cos0 < cos1) left++;
      else std::swap(curves[left],curves[--right]);
    }
    return left;
  }

  __forceinline const BVH2HairBuilder::ObjectSplit BVH2HairBuilder::ObjectSplit::find(Bezier1* curves, size_t begin, size_t end, const NAABBox3fa& space)
  {
    /* calculate geometry and centroid bounds */
    BBox3fa centBounds = empty;
    BBox3fa geomBounds = empty;
    for (size_t i=begin; i<end; i++)
    {
      const Vec3fa p0 = xfmPoint(space.space,curves[i].p0); geomBounds.extend(p0);
      const Vec3fa p1 = xfmPoint(space.space,curves[i].p1); geomBounds.extend(p1);
      const Vec3fa p2 = xfmPoint(space.space,curves[i].p2); geomBounds.extend(p2);
      const Vec3fa p3 = xfmPoint(space.space,curves[i].p3); geomBounds.extend(p3);
      centBounds.extend(p0+p3);
    }

    /* calculate binning function */
    const ssef ofs = (ssef) centBounds.lower;
    const ssef diag = (ssef) centBounds.size();
    const ssef scale = select(diag != 0.0f,rcp(diag) * ssef(BINS * 0.99f),ssef(0.0f));

    /* initialize bins */
    BBox3fa bounds[BINS][4];
    ssei    counts[BINS];
    for (size_t i=0; i<BINS; i++) {
      bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = empty;
      counts[i] = 0;
    }
 
    /* perform binning of curves */
    for (size_t i=begin; i<end; i++)
    {
      BBox3fa cbounds = empty;
      const Vec3fa p0 = xfmPoint(space.space,curves[i].p0); cbounds.extend(p0);
      const Vec3fa p1 = xfmPoint(space.space,curves[i].p1); cbounds.extend(p1);
      const Vec3fa p2 = xfmPoint(space.space,curves[i].p2); cbounds.extend(p2);
      const Vec3fa p3 = xfmPoint(space.space,curves[i].p3); cbounds.extend(p3);
      //const ssei bin = clamp(floori((ssef(p0+p3) - ofs)*scale),ssei(0),ssei(BINS-1));
      const ssei bin = floori((ssef(p0+p3) - ofs)*scale);
      assert(bin[0] >=0 && bin[0] < BINS);
      assert(bin[1] >=0 && bin[1] < BINS);
      assert(bin[2] >=0 && bin[2] < BINS);
      const int b0 = bin[0]; counts[b0][0]++; bounds[b0][0].extend(cbounds);
      const int b1 = bin[1]; counts[b1][1]++; bounds[b1][1].extend(cbounds);
      const int b2 = bin[2]; counts[b2][2]++; bounds[b2][2].extend(cbounds);
    }
    
    /* sweep from right to left and compute parallel prefix of merged bounds */
    ssef rAreas[BINS];
    ssei rCounts[BINS];
    ssei count = 0; BBox3fa bx = empty; BBox3fa by = empty; BBox3fa bz = empty;
    for (size_t i=BINS-1; i>0; i--)
    {
      count += counts[i];
      rCounts[i] = count;
      bx.extend(bounds[i][0]); rAreas[i][0] = area(bx);
      by.extend(bounds[i][1]); rAreas[i][1] = area(by);
      bz.extend(bounds[i][2]); rAreas[i][2] = area(bz);
    }
    
    /* sweep from left to right and compute SAH */
    ssei ii = 1; ssef bestSAH = pos_inf; ssei bestPos = 0; ssei bestLeft = 0;
    count = 0; bx = empty; by = empty; bz = empty;
    for (size_t i=1; i<BINS; i++, ii+=1)
    {
      count += counts[i-1];
      bx.extend(bounds[i-1][0]); float Ax = area(bx);
      by.extend(bounds[i-1][1]); float Ay = area(by);
      bz.extend(bounds[i-1][2]); float Az = area(bz);
      const ssef lArea = ssef(Ax,Ay,Az,Az);
      const ssef rArea = rAreas[i];
      //const ssei lCount = (count     +ssei(3)) >> 2;
      //const ssei rCount = (rCounts[i]+ssei(3)) >> 2;
      const ssei lCount = count;
      const ssei rCount = rCounts[i];
      const ssef sah = lArea*ssef(lCount) + rArea*ssef(rCount);
      //PRINT2(lArea,lCount);
      //PRINT2(rArea,rCount);
      //PRINT2(i,sah);
      bestPos = select(sah < bestSAH,ii ,bestPos);
      bestLeft= select(sah < bestSAH,count,bestLeft);
      bestSAH = select(sah < bestSAH,sah,bestSAH);
    }
    
    /* find best dimension */
    ObjectSplit split;
    for (size_t dim=0; dim<3; dim++) 
    {
      /* ignore zero sized dimensions */
      if (unlikely(scale[dim] == 0.0f)) 
        continue;
      
      /* test if this is a better dimension */
      if (bestSAH[dim] < split.cost && bestPos[dim] != 0) {
        split.dim = dim;
        split.pos = bestPos[dim];
        split.cost = bestSAH[dim];
        split.num0 = bestLeft[dim];
        split.num1 = end-begin-split.num0;
      }
    }
    
    /* partition curves */
    ssize_t left = begin, right = end;
    while (left < right) {
      const Vec3fa p0 = xfmPoint(space.space,curves[left].p0);
      const Vec3fa p3 = xfmPoint(space.space,curves[left].p3);
      //const ssei bin = clamp(floori((ssef(p0+p3) - ofs)*scale),ssei(0),ssei(BINS-1));
      const ssei bin = floori((ssef(p0+p3) - ofs)*scale);
      if (bin[split.dim] < split.pos) left++;
      else std::swap(curves[left],curves[--right]);
    }
    split.space = space.space;
    split.bounds0 = bestSpace(curves,begin,left);
    split.bounds1 = bestSpace(curves,left, end );
    split.ofs = ofs;
    split.scale = scale;
    return split;
  }
  
  __forceinline size_t BVH2HairBuilder::ObjectSplit::split(Bezier1* curves, size_t begin, size_t end) const
  {
    ssize_t left = begin, right = end;
    while (left < right) {
      const Vec3fa p0 = xfmPoint(space,curves[left].p0);
      const Vec3fa p3 = xfmPoint(space,curves[left].p3);
      //const ssei bin = clamp(floori((ssef(p0+p3) - ofs)*scale),ssei(0),ssei(BINS-1));
      const ssei bin = floori((ssef(p0+p3) - ofs)*scale);
      if (bin[dim] < pos) left++;
      else std::swap(curves[left],curves[--right]);
    }
    return left;
  }
  
  typename BVH2Hair::NodeRef BVH2HairBuilder::leaf(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds)
  {
    size_t N = end-begin;
    assert(N <= (size_t)BVH2Hair::maxLeafBlocks);
    Bezier1* leaf = (Bezier1*) bvh->allocPrimitiveBlocks(threadIndex,N);
    for (size_t i=0; i<N; i++) leaf[i] = curves[begin+i];
    return bvh->encodeLeaf((char*)leaf,N);
  }

  typename BVH2Hair::NodeRef BVH2HairBuilder::recurse(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds)
  {
    /*! compute leaf and split cost */
    size_t N = end-begin;
    //const float leafSAH  = float(N)*halfArea(bounds.bounds);
    const float leafSAH  = bounds.bounds.lower.w*halfArea(bounds.bounds);
    
    /* first split into two strands */
    const StrandSplit strandSplit = StrandSplit::find(curves,begin,end);
    const float strandSAH = BVH2Hair::travCost*halfArea(bounds.bounds) + strandSplit.sah();

    /* second perform standard binning */
    const ObjectSplit objectSplit = ObjectSplit::find(curves,begin,end,bounds);
    const float objectSAH = BVH2Hair::travCost*halfArea(bounds.bounds) + objectSplit.sah();

    /*if (depth == 0) {
      PRINT(strandSplit);
      PRINT(objectSplit);
      PRINT(leafSAH);
      PRINT(strandSAH);
      PRINT(objectSAH);
      }*/

    /* leaf test */
    const float bestSAH = min(leafSAH,strandSAH,objectSAH);
    if (N <= minLeafSize || depth > BVH2Hair::maxBuildDepth || (N <= maxLeafSize && bestSAH == leafSAH))
      return leaf(threadIndex,depth,begin,end,bounds);
    Node* node = bvh->allocNode(threadIndex);

    /* perform strand split */
    if (bestSAH == strandSAH) {
      size_t center = strandSplit.split(curves,begin,end);
      node->set(0,strandSplit.bounds0,recurse(threadIndex,depth+1,begin ,center,strandSplit.bounds0));
      node->set(1,strandSplit.bounds1,recurse(threadIndex,depth+1,center,end   ,strandSplit.bounds1));
    }
    /* or object split */
    else {
      const size_t center = objectSplit.split(curves,begin,end);
      node->set(0,objectSplit.bounds0,recurse(threadIndex,depth+1,begin ,center,objectSplit.bounds0));
      node->set(1,objectSplit.bounds1,recurse(threadIndex,depth+1,center,end   ,objectSplit.bounds1));
    }

    return bvh->encodeNode(node);
  }

  Builder* BVH2HairBuilder_ (BVH2Hair* accel, Scene* scene) {
    return new BVH2HairBuilder(accel,scene);
  }
}
