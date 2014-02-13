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
#include "bvh2hair_statistics.h"
#include "common/scene_bezier_curves.h"

namespace embree
{
  BVH2HairBuilder::BVH2HairBuilder (BVH2Hair* bvh, Scene* scene)
    : scene(scene), minLeafSize(1), maxLeafSize(inf), bvh(bvh)
  {
    if (BVH2Hair::maxLeafBlocks < this->maxLeafSize) 
      this->maxLeafSize = BVH2Hair::maxLeafBlocks;
  }

  BVH2HairBuilder::~BVH2HairBuilder () {
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

    /* create initial curve list */
    BBox3fa bounds = empty;
    curves.reserve(numPrimitives);
    for (size_t i=0; i<scene->size(); i++) 
    {
      Geometry* geom = scene->get(i);
      if (geom->type != BEZIER_CURVES) continue;
      if (!geom->isEnabled()) continue;
      BezierCurves* set = (BezierCurves*) geom;

      for (size_t j=0; j<set->numCurves; j++) {
        const int ofs = set->curve(j);
        const Vec3fa& p0 = set->vertex(ofs+0);
        const Vec3fa& p1 = set->vertex(ofs+1);
        const Vec3fa& p2 = set->vertex(ofs+2);
        const Vec3fa& p3 = set->vertex(ofs+3);
        const Bezier1 bezier(p0,p1,p2,p3,0,1,i,j);
        bounds.extend(bezier.bounds());
        curves.push_back(bezier);
      }
    }

    /* subdivide very curved hair segments */
    subdivide(0.1f);
    bvh->numPrimitives = curves.size();
    bvh->numVertices = 0;

    /* start recursive build */
    size_t begin = 0, end = curves.size();
    //bvh->root = recurse_aligned(threadIndex,0,begin,end,computeAlignedBounds(&curves[0],begin,end,AffineSpace3fa(one)));
    //bvh->root = recurse_unaligned(threadIndex,0,begin,end,computeUnalignedBounds(&curves[0],begin,end));
    bvh->root = recurse_aligned_unaligned(threadIndex,0,begin,end,computeAlignedBounds(&curves[0],begin,end,AffineSpace3fa(one)));
    bvh->bounds = bounds;

    if (g_verbose >= 2) {
      double t1 = getSeconds();
      std::cout << "[DONE]" << std::endl;
      std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(numPrimitives)/(t1-t0) << " Mprim/s" << std::endl;
      std::cout << BVH2HairStatistics(bvh).str();
    }
  }

  void BVH2HairBuilder::subdivide(float ratio)
  {
    if (g_verbose >= 2) 
      std::cout << std::endl << "  before subdivision: " << 1E-6*float(curves.size()) << " M curves" << std::endl;

    for (ssize_t i=0; i<curves.size(); i++)
    {
      /* calculate axis to align bounds */
      const Vec3fa axis = curves[i].p3-curves[i].p0;
      const float len = length(axis);
      if (len == 0.0f) continue; // FIXME: could still need subdivision
      
      /* test if we should subdivide */
      const AffineSpace3fa space = rcp(frame(axis/len)); // FIXME: use transpose
      BBox3fa bounds = empty;
      const Vec3fa p0 = xfmPoint(space,curves[i].p0); bounds.extend(p0);
      const Vec3fa p1 = xfmPoint(space,curves[i].p1); bounds.extend(p1);
      const Vec3fa p2 = xfmPoint(space,curves[i].p2); bounds.extend(p2);
      const Vec3fa p3 = xfmPoint(space,curves[i].p3); bounds.extend(p3);
      const Vec3fa diag = bounds.size();

      /* perform subdivision */
      if (max(diag.x,diag.y) > ratio*diag.z && curves[i].dt > 0.1f) {
        Bezier1 left,right; curves[i].subdivide(left,right);
        curves[i] = left; curves.push_back(right);
        i--;
      }
    }

    if (g_verbose >= 2) 
      std::cout << "  after  subdivision: " << 1E-6*float(curves.size()) << " M curves" << std::endl;
  }

  const BBox3fa BVH2HairBuilder::computeAlignedBounds(Bezier1* curves, size_t begin, size_t end)
  {
    float area = 0.0f;
    BBox3fa bounds = empty;
    for (size_t j=begin; j<end; j++) {
      const BBox3fa cbounds = curves[j].bounds();
      area += halfArea(cbounds);
      bounds.extend(cbounds);
    }
    bounds.upper.w = area;
    return bounds;
  }

  const BVH2Hair::NAABBox3fa BVH2HairBuilder::computeAlignedBounds(Bezier1* curves, size_t begin, size_t end, const AffineSpace3fa& space)
  {
    float area = 0.0f;
    BBox3fa bounds = empty;
    for (size_t j=begin; j<end; j++) {
      const BBox3fa cbounds = curves[j].bounds(space);
      area += halfArea(cbounds);
      bounds.extend(cbounds);
    }
    NAABBox3fa b(space,bounds);
    b.bounds.upper.w = area;
    return b;
  }

  const BVH2Hair::NAABBox3fa BVH2HairBuilder::computeUnalignedBounds(Bezier1* curves, size_t begin, size_t end)
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
      const AffineSpace3fa space = rcp(frame(axis)); // FIXME: use transpose
      
      BBox3fa bounds = empty;
      float area = 0.0f;
      for (size_t j=begin; j<end; j++) {
        const BBox3fa cbounds = curves[j].bounds(space);
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
    bounds.bounds.upper.w = bestArea;
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

    /* partition the two strands */
    ssize_t left = begin, right = end;
    while (left < right) {
      const Vec3fa axisi = normalize(curves[left].p3-curves[left].p0);
      const float cos0 = abs(dot(axisi,axis0));
      const float cos1 = abs(dot(axisi,axis1));
      if (cos0 > cos1) left++;
      else std::swap(curves[left],curves[--right]);
    }
    const size_t num0 = left-begin;
    const size_t num1 = end-left;
    if (num0 == 0 || num1 == 0)
      return StrandSplit(NAABBox3fa(one,inf),axis0,1,NAABBox3fa(one,inf),axis1,1);

    const NAABBox3fa naabb0 = computeUnalignedBounds(&curves[0],begin,left);
    const NAABBox3fa naabb1 = computeUnalignedBounds(&curves[0],left, end );
    return StrandSplit(naabb0,axis0,num0,naabb1,axis1,num1);
  }

  __forceinline size_t BVH2HairBuilder::StrandSplit::split(Bezier1* curves, size_t begin, size_t end) const
  {
    ssize_t left = begin, right = end;
    while (left < right) {
      const Vec3fa axisi = normalize(curves[left].p3-curves[left].p0);
      const float cos0 = abs(dot(axisi,axis0));
      const float cos1 = abs(dot(axisi,axis1));
      if (cos0 > cos1) left++;
      else std::swap(curves[left],curves[--right]);
    }
    assert(left-begin == num0);
    assert(end-left == num1);
    return left;
  }

  __forceinline BVH2HairBuilder::ObjectSplit BVH2HairBuilder::ObjectSplit::find(Bezier1* curves, size_t begin, size_t end, const AffineSpace3fa& space)
  {
    /* calculate geometry and centroid bounds */
    BBox3fa centBounds = empty;
    BBox3fa geomBounds = empty;
    for (size_t i=begin; i<end; i++) {
      const Vec3fa p0 = xfmPoint(space,curves[i].p0);
      const Vec3fa p3 = xfmPoint(space,curves[i].p3);
      geomBounds.extend(curves[i].bounds(space)); // FIXME: transforms points again
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
      const BBox3fa cbounds = curves[i].bounds(space); // FIXME: transforms again
      const Vec3fa p0 = xfmPoint(space,curves[i].p0);
      const Vec3fa p3 = xfmPoint(space,curves[i].p3);
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
      bestPos = select(sah < bestSAH,ii ,bestPos);
      bestLeft= select(sah < bestSAH,count,bestLeft);
      bestSAH = select(sah < bestSAH,sah,bestSAH);
    }
    
    /* find best dimension */
    ObjectSplit split;
    split.space = space;
    split.ofs = ofs;
    split.scale = scale;

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
    return split;
  }

  const BVH2HairBuilder::ObjectSplit BVH2HairBuilder::ObjectSplit::alignedBounds(Bezier1* curves, size_t begin, size_t end)
  {
    if (dim == -1) {
      num0 = num1 = 1;
      bounds0 = bounds1 = BBox3fa(inf);
      return *this;
    }

    const size_t center = split(&curves[0],begin,end);
    bounds0 = computeAlignedBounds(&curves[0],begin,center);
    bounds1 = computeAlignedBounds(&curves[0],center,end);
    return *this;
  }
  
  const BVH2HairBuilder::ObjectSplit  BVH2HairBuilder::ObjectSplit::unalignedBounds(Bezier1* curves, size_t begin, size_t end)
  {
    if (dim == -1) {
      num0 = num1 = 1;
      bounds0 = bounds1 = BBox3fa(inf);
      return *this;
    }

    /* partition curves */
    ssize_t left = begin, right = end;
    while (left < right) {
      const Vec3fa p0 = xfmPoint(space,curves[left].p0);
      const Vec3fa p3 = xfmPoint(space,curves[left].p3);
      //const ssei bin = clamp(floori((ssef(p0+p3) - ofs)*scale),ssei(0),ssei(BINS-1));
      const ssei bin = floori((ssef(p0+p3) - ofs)*scale);
      if (bin[dim] < pos) left++;
      else std::swap(curves[left],curves[--right]);
    }
    bounds0 = computeUnalignedBounds(&curves[0],begin,left);
    bounds1 = computeUnalignedBounds(&curves[0],left, end );
    return *this;
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
    assert(left-begin == num0);
    assert(end-left == num1);
    return left;
  }
  
  __forceinline BVH2HairBuilder::FallBackSplit BVH2HairBuilder::FallBackSplit::find(Bezier1* curves, size_t begin, size_t end)
  {
    const size_t center = (begin+end)/2;
    const BBox3fa bounds0 = computeAlignedBounds(&curves[0],begin,center);
    const BBox3fa bounds1 = computeAlignedBounds(&curves[0],center,end);
    return FallBackSplit(center,bounds0,bounds1);
  }

  typename BVH2Hair::NodeRef BVH2HairBuilder::leaf(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds)
  {
    size_t N = end-begin;
    if (N > (size_t)BVH2Hair::maxLeafBlocks) {
      std::cout << "WARNING: Loosing " << N-BVH2Hair::maxLeafBlocks << " primitives during build!" << std::endl;
      N = (size_t)BVH2Hair::maxLeafBlocks;
    }
    //assert(N <= (size_t)BVH2Hair::maxLeafBlocks);
    Bezier1* leaf = (Bezier1*) bvh->allocPrimitiveBlocks(threadIndex,N);
    for (size_t i=0; i<N; i++) leaf[i] = curves[begin+i];
    return bvh->encodeLeaf((char*)leaf,N);
  }

  typename BVH2Hair::NodeRef BVH2HairBuilder::recurse_aligned(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds)
  {
    /* create enforced leaf */
    const size_t N = end-begin;
    if (N <= minLeafSize || depth > BVH2Hair::maxBuildDepth)
      return leaf(threadIndex,depth,begin,end,bounds);

    /*! compute leaf cost */
    const float leafSAH = N <= maxLeafSize ? BVH2Hair::intCost*float(N)*halfArea(bounds.bounds) : inf;
    
    /* perform standard binning with aligned bounds */
    const ObjectSplit objectSplit = ObjectSplit::find(&curves[0],begin,end).alignedBounds(&curves[0],begin,end);
    const float objectSAH = BVH2Hair::travCostAligned*halfArea(bounds.bounds) + objectSplit.standardSAH();

    /* calculate best SAH */
    const float bestSAH = min(leafSAH,objectSAH);

    /* perform fallback split */
    if (bestSAH == float(inf)) 
    {
      AlignedNode* node = bvh->allocAlignedNode(threadIndex);
      const FallBackSplit split = FallBackSplit::find(&curves[0],begin,end);
      assert((split.center-begin > 0) && (end-split.center) > 0);
      node->set(0,split.bounds0,recurse_aligned(threadIndex,depth+1,begin,split.center,split.bounds0));
      node->set(1,split.bounds1,recurse_aligned(threadIndex,depth+1,split.center,end  ,split.bounds1));
      return bvh->encodeNode(node);
    }

    /* perform object split */
    else if (bestSAH == objectSAH) {
      AlignedNode* node = bvh->allocAlignedNode(threadIndex);
      const size_t center = objectSplit.split(&curves[0],begin,end);
      assert((center-begin > 0) && (end-center) > 0);
      node->set(0,objectSplit.bounds0.bounds,recurse_aligned(threadIndex,depth+1,begin ,center,objectSplit.bounds0));
      node->set(1,objectSplit.bounds1.bounds,recurse_aligned(threadIndex,depth+1,center,end   ,objectSplit.bounds1));
      return bvh->encodeNode(node);
    }

    /* else create leaf */
    else
      return leaf(threadIndex,depth,begin,end,bounds);
  }

  typename BVH2Hair::NodeRef BVH2HairBuilder::recurse_unaligned(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds)
  {
    /* create enforced leaf */
    const size_t N = end-begin;
    if (N <= minLeafSize || depth > BVH2Hair::maxBuildDepth)
      return leaf(threadIndex,depth,begin,end,bounds);

    /*! compute leaf and split cost */
    const float leafSAH = N <= maxLeafSize ? BVH2Hair::intCost*float(N)*halfArea(bounds.bounds) : inf;
    
    /* first split into two strands */
    const StrandSplit strandSplit = StrandSplit::find(&curves[0],begin,end);
    const float strandSAH = BVH2Hair::travCostUnaligned*halfArea(bounds.bounds) + strandSplit.modifiedSAH();

    /* second perform standard binning */
    const ObjectSplit objectSplit = ObjectSplit::find(&curves[0],begin,end,bounds.space).unalignedBounds(&curves[0],begin,end);
    const float objectSAH = BVH2Hair::travCostUnaligned*halfArea(bounds.bounds) + objectSplit.modifiedSAH();

    /* calculate best SAH */
    const float bestSAH = min(leafSAH,strandSAH,objectSAH);

    /* perform fallback split */
    if (bestSAH == float(inf)) {
      AlignedNode* node = bvh->allocAlignedNode(threadIndex);
      const FallBackSplit split = FallBackSplit::find(&curves[0],begin,end);
      assert((split.center-begin > 0) && (end-split.center) > 0);
      node->set(0,split.bounds0,recurse_unaligned(threadIndex,depth+1,begin,split.center,split.bounds0));
      node->set(1,split.bounds1,recurse_unaligned(threadIndex,depth+1,split.center,end  ,split.bounds1));
      return bvh->encodeNode(node);
    }

    /* perform strand split */
    else if (bestSAH == strandSAH) {
      UnalignedNode* node = bvh->allocUnalignedNode(threadIndex);
      const size_t center = strandSplit.split(&curves[0],begin,end);
      assert((center-begin > 0) && (end-center) > 0);
      node->set(0,strandSplit.bounds0,recurse_unaligned(threadIndex,depth+1,begin ,center,strandSplit.bounds0));
      node->set(1,strandSplit.bounds1,recurse_unaligned(threadIndex,depth+1,center,end   ,strandSplit.bounds1));
      return bvh->encodeNode(node);
    }
    
    /* perform object split */
    else if (bestSAH == objectSAH) {
      UnalignedNode* node = bvh->allocUnalignedNode(threadIndex);
      const size_t center = objectSplit.split(&curves[0],begin,end);
      assert((center-begin > 0) && (end-center) > 0);
      node->set(0,objectSplit.bounds0,recurse_unaligned(threadIndex,depth+1,begin ,center,objectSplit.bounds0));
      node->set(1,objectSplit.bounds1,recurse_unaligned(threadIndex,depth+1,center,end   ,objectSplit.bounds1));
      return bvh->encodeNode(node);
    }

    /* else create leaf */
    else
      return leaf(threadIndex,depth,begin,end,bounds);
  }

  typename BVH2Hair::NodeRef BVH2HairBuilder::recurse_aligned_unaligned(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds)
  {
    /* create enforced leaf */
    const size_t N = end-begin;
    if (N <= minLeafSize || depth > BVH2Hair::maxBuildDepth)
      return leaf(threadIndex,depth,begin,end,bounds);

    /*! compute leaf and split cost */
    const float leafSAH = N <= maxLeafSize ? BVH2Hair::intCost*float(N)*halfArea(bounds.bounds) : inf;

    /* first split into two strands */
    const StrandSplit strandSplit = StrandSplit::find(&curves[0],begin,end);
    const float strandSAH = BVH2Hair::travCostUnaligned*halfArea(bounds.bounds) + strandSplit.modifiedSAH();

    /* second perform standard binning */
    const ObjectSplit objectSplit = ObjectSplit::find(&curves[0],begin,end,bounds.space).unalignedBounds(&curves[0],begin,end);
    const float objectSAH = BVH2Hair::travCostUnaligned*halfArea(bounds.bounds) + objectSplit.modifiedSAH();

    /* third perform standard binning in aligned space */
    const ObjectSplit objectSplitAligned = ObjectSplit::find(&curves[0],begin,end).alignedBounds(&curves[0],begin,end);
    const float alignedObjectSAH = BVH2Hair::travCostAligned*halfArea(bounds.bounds) + objectSplitAligned.modifiedSAH();

    /* calculate best SAH */
    const float bestSAH = min(leafSAH,strandSAH,objectSAH,alignedObjectSAH);

    /* perform fallback split */
    if (bestSAH == float(inf)) {
      AlignedNode* node = bvh->allocAlignedNode(threadIndex);
      const FallBackSplit split = FallBackSplit::find(&curves[0],begin,end);
      assert((split.center-begin > 0) && (end-split.center) > 0);
      node->set(0,split.bounds0,recurse_aligned_unaligned(threadIndex,depth+1,begin,split.center,split.bounds0));
      node->set(1,split.bounds1,recurse_aligned_unaligned(threadIndex,depth+1,split.center,end  ,split.bounds1));
      return bvh->encodeNode(node);
    }

    /* perform aligned object split */
    else if (bestSAH == alignedObjectSAH) {
      AlignedNode* node = bvh->allocAlignedNode(threadIndex);
      const size_t center = objectSplitAligned.split(&curves[0],begin,end);
      assert((center-begin > 0) && (end-center) > 0);
      node->set(0,objectSplitAligned.bounds0.bounds,recurse_aligned_unaligned(threadIndex,depth+1,begin ,center,objectSplitAligned.bounds0));
      node->set(1,objectSplitAligned.bounds1.bounds,recurse_aligned_unaligned(threadIndex,depth+1,center,end   ,objectSplitAligned.bounds1));
      return bvh->encodeNode(node);
    }

    /* perform unaliged object split */
    else if (bestSAH == objectSAH) {
      UnalignedNode* node = bvh->allocUnalignedNode(threadIndex);
      const size_t center = objectSplit.split(&curves[0],begin,end);
      assert((center-begin > 0) && (end-center) > 0);
      node->set(0,objectSplit.bounds0,recurse_aligned_unaligned(threadIndex,depth+1,begin ,center,objectSplit.bounds0));
      node->set(1,objectSplit.bounds1,recurse_aligned_unaligned(threadIndex,depth+1,center,end   ,objectSplit.bounds1));
      return bvh->encodeNode(node);
    }

    /* perform strand split */
    else if (bestSAH == strandSAH) {
      UnalignedNode* node = bvh->allocUnalignedNode(threadIndex);
      const size_t center = strandSplit.split(&curves[0],begin,end);
      assert((center-begin > 0) && (end-center) > 0);
      node->set(0,strandSplit.bounds0,recurse_aligned_unaligned(threadIndex,depth+1,begin ,center,strandSplit.bounds0));
      node->set(1,strandSplit.bounds1,recurse_aligned_unaligned(threadIndex,depth+1,center,end   ,strandSplit.bounds1));
      return bvh->encodeNode(node);
    }

    /* else create leaf */
    else
      return leaf(threadIndex,depth,begin,end,bounds);
  }

  Builder* BVH2HairBuilder_ (BVH2Hair* accel, Scene* scene) {
    return new BVH2HairBuilder(accel,scene);
  }
}
