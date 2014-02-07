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
    if (scene->numCurves == 0) {
      bvh->init(0);
      return;
    }
    
    double t0 = 0.0;
    if (g_verbose >= 2) {
      std::cout << "building BVH2Hair<Bezier1> ... " << std::flush;
      t0 = getSeconds();
    }

    /* allocate temporary curve array */
    size_t numPrimitives = scene->numCurves;
    curves = new Bezier1[2*numPrimitives];

    /* create initial curve list */
    BBox3f bounds = empty;
    size_t begin = 0, end = 0;
    for (size_t i=0; i<scene->size(); i++) 
    {
      Geometry* geom = scene->get(i);
      if (geom->type != QUADRATIC_BEZIER_CURVES) continue;
      if (!geom->enabled()) continue;
      QuadraticBezierCurvesScene::QuadraticBezierCurves* set = (QuadraticBezierCurvesScene::QuadraticBezierCurves*) geom;

      for (size_t j=0; j<set.numCurves; i++) {
        int ofs = set.curve(j);
        const Vec3fa& p0 = set.vertex(ofs+0); bounds.extend(p0);
        const Vec3fa& p1 = set.vertex(ofs+1); bounds.extend(p1);
        const Vec3fa& p2 = set.vertex(ofs+2); bounds.extend(p2);
        const Vec3fa& p3 = set.vertex(ofs+3); bounds.extend(p3);
        curves[end++] = Bezier1 (p0,p1,p2,p3,0,1,set.geomID,j);
      }
    }

    /* find best bounds for hair */
    NAABBox3fa naabb = bestBounds(begin,end);

    /* start recursive build */
    bvh->root = recurse(threadIndex,0,begin,end,naabb);
    bvh->bounds = bounds;

    if (g_verbose >= 2) {
      double t1 = getSeconds();
      std::cout << "[DONE]" << std::endl;
      std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(source->size())/(t1-t0) << " Mprim/s" << std::endl;
      //std::cout << BVH2HairStatistics(bvh).str();
    }
  }

  struct StrandSplit
  {
  public:
    __forceinline StrandSplit (const NAABBox3fa& bounds0, const Vec3fa& axis0, const size_t num0,
                               const NAABBox3fa& bounds1, const Vec3fa& axis1, const size_t num1)
      : bounds0(bounds0), bounds1(bounds1), axis0(axis0), axis1(axis1), num0(num0), num1(num1) {}
    
    __forceinline float sah() const {
      return float(num0)*halfArea(bounds0.bounds) + float(num1)*halfArea(bounds1.bounds);
    }

    static __forceinline const StrandSplit find(size_t begin, size_t end)
    {
      /* first try to split two hair strands */
      Vec3fa axis0 = normalize(curves[begin].p3-curves[begin].p0);
      float bestCos = 1.0f;
      float bestI = begin;
      for (size_t i=begin; i<end; i++) {
        Vec3fa axisi = normalize(curves[i].p3-curves[i].p0);
        float cos = abs(dot(axisi,axis0));
        if (cos < bestCos) { bestCos = cos; bestI = i; }
      }
      Vec3fa axis1 = normalize(curves[bestI].p3-curves[bestI].p0);

      /* partition the two strands */
      ssize_t left = begin, right = end;
      while (left < right) {
        if (curves[left].isLeft()) left++;
        else std::swap(curves[left],curves[--right]);
      }
      const NAABBox3fa naabb0 = bestBounds(begin,left);
      const NAABBox3fa naabb1 = bestBounds(left, end );
      return StrandSplit(axis0,naabb0,axis1,naabb1);
    }

    __forceinline size_t split(size_t begin, size_t end)
    {
      ssize_t left = begin, right = end;
      while (left < right) {
        if (curves[left].isLeft()) left++;
        else std::swap(curves[left],curves[--right]);
      }
      return left;
    }

  public:
    NAABBox3fa bounds0, bounds1;
    Vec3fa axis0, axis1;
    size_t num0, num1;
  };

  const size_t BINS = 16;

  struct ObjectSplit
  {
  public:

    __forceinline ObjectSplit ()
      : dim(0), pos(0), cost(inf), num0(0), num1(0) {}

    __forceinline ObjectSplit (const NAABBox3fa& bounds,
                               const BBox3f& bounds0, const size_t num0,
                               const BBox3f& bounds1, const size_t num1)
      : bounds(bounds), bounds0(bounds0), bounds1(bounds1), num0(num0), num1(num1) {}
    
    __forceinline float sah() const {
      return float(num0)*halfArea(bounds0) + float(num1)*halfArea(bounds1);
    }

    static __forceinline const ObjectSplit find(size_t begin, size_t end, const NAABBox3fa& pbounds)
    {
      /* calculate binning function */
      const ssef geometryDiagonal = 2.0f * (ssef) bounds.bounds.size();
      const ssef scale = select(geometryDiagonal != 0.0f,rcp(geometryDiagonal) * ssef(BINS * 0.99f),ssef(0.0f));
      const ssef ofs = 2.0f * (ssef) bounds.bounds.lower;

      /* perform binning of curves */
      for (size_t i=begin; i<end; i++)
      {
        const BBox3fa bounds = empty;
        const Vec3fa p0 = xfmPoint(pbounds.space,curves[i].p0); bounds.extend(p0);
        const Vec3fa p1 = xfmPoint(pbounds.space,curves[i].p1); bounds.extend(p1);
        const Vec3fa p2 = xfmPoint(pbounds.space,curves[i].p2); bounds.extend(p2);
        const Vec3fa p3 = xfmPoint(pbounds.space,curves[i].p3); bounds.extend(p3);
        const ssei bin = clamp(floori((ssef(p0+p3) - ofs)*scale),0,BINS-1);
        const int b0 = bin[0]; counts[b0][0]++; bounds[b0][0].extend(bounds);
        const int b1 = bin[1]; counts[b1][1]++; bounds[b1][1].extend(bounds);
        const int b2 = bin[2]; counts[b2][2]++; bounds[b2][2].extend(bounds);
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
        const ssei lCount = (count     +ssei(3)) >> 2;
        const ssei rCount = (rCounts[i]+ssei(3)) >> 2;
        const ssef sah = lArea*ssef(lCount) + rArea*ssef(rCount);
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
          split.num1 = end-begin-split.nleft;
        }
      }

      /* partition curves */
      ssize_t left = begin, right = end;
      while (left < right) {
        const Vec3fa p0 = xfmPoint(pbounds.space,curves[left].p0);
        const Vec3fa p3 = xfmPoint(pbounds.space,curves[left].p3);
        const ssei bin = clamp(floori((ssef(p0+p3) - ofs)*scale),0,BINS-1);
        if (bin < pos) left++;
        else std::swap(curves[left],curves[--right]);
      }
      split.bounds = bounds;
      split.bounds0 = bestBounds(begin,left);
      split.bounds1 = bestBounds(left, end );
      split.ofs = ofs;
      split.scale = scale;
      return split;
    }

    __forceinline size_t split(size_t begin, size_t end)
    {
      ssize_t left = begin, right = end;
      while (left < right) {
        const Vec3fa p0 = xfmPoint(bounds.space,curves[left].p0);
        const Vec3fa p3 = xfmPoint(ounds.space,curves[left].p3);
        const ssei bin = clamp(floori((ssef(p0+p3) - ofs)*scale),0,BINS-1);
        if (bin < pos) left++;
        else std::swap(curves[left],curves[--right]);
      }
      return left;
    }

  public:
    NAABBox3fa bounds;
    NAABBox3fa bounds0, bounds1;
    size_t dim;
    size_t pos;
    float cost;
    size_t num0,num1;
    ssef ofs,scale;
  };

  typename BVH2Hair::NodeRef BVH2HairBuilder::leaf(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds)
  {
    size_t N = end-begin;
    assert(N <= (size_t)BVH2Hair::maxLeafBlocks);
    Bezier1* curves = (Bezier1*) bvh->allocPrimitiveBlocks(threadIndex,N);
    for (size_t i=0; i<N; i++) curves[i] = prims[begin+i];
    return bvh->encodeLeaf(curves,N);
  }

  typename BVH2Hair::NodeRef BVH2HairBuilder::recurse(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds)
  {
    /*! compute leaf and split cost */
    size_t N = end-begin;
    const float leafSAH  = float(N)*bounds.area();
    
    /* first split into two strands */
    StrandSplit strandSplit = StrandSplit::find(begin,end);
    const float strandSAH = strandsplit.sah();

    /* second perform standard binning */
    ObjectSplit objectSplit = ObjectSplit::find(begin,end,bounds.xfm);
    const float objectSAH = objectsplit.sah();
    
    /* leaf test */
    const float bestSAH = min(leafSAH,strandSAH,objectSAH);
    if (bestSAH == leafSAH) return leaf(threadIndex,depth,begin,end,bounds);
    Node* node = parent->bvh->allocNode(threadIndex);

    /* perform strand split */
    if (bestSAH == strandSAH) {
      size_t center = strandSplit.split(begin,end);
      node->set(0,strandSplit.bounds0,recurse(threadIndex,depth+1,begin ,center,strandSplit.bounds0));
      node->set(1,strandSplit.bounds1,recurse(threadIndex,depth+1,center,end   ,strandSplit.bounds1));
    }
    /* or object split */
    else {
      const size_t center = objectSplit.split(begin,end);
      node->set(0,strandSplit.bounds0,recurse(threadIndex,depth+1,begin ,center,strandSplit.bounds0));
      node->set(1,strandSplit.bounds1,recurse(threadIndex,depth+1,center,end   ,strandSplit.bounds1));
    }
    return parent->bvh->encodeNode(node);
  }

  Builder* BVH2HairBuilder_ (BVH2Hair* accel, Scene* scene) {
    return new BVH2HairBuilder(accel,scene);
  }
}
