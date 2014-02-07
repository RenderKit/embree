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
    NAABBox3f naabb = bestBounds(begin,end);

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

  typename BVH2Hair::NodeRef BVH2HairBuilder::createLeaf(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3f& bounds)
  {
    size_t N = end-begin;
    assert(N <= (size_t)BVH2Hair::maxLeafBlocks);
    Bezier1* curves = (Bezier1*) bvh->allocPrimitiveBlocks(threadIndex,N);
    for (size_t i=0; i<N; i++) curves[i] = prims[begin+i];
    return bvh->encodeLeaf(curves,N);
  }

  typename BVH2Hair::NodeRef BVH2HairBuilder::recurse(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3f& bounds)
  {
    /*! compute leaf and split cost */
    size_t N = end-begin;
    const float leafSAH  = float(N)*bounds.area();
    
    
    /*! create an inner node */
    Node* node = parent->bvh->allocNode(threadIndex);
    for (size_t i=0; i<numChildren; i++) 
      node->set(i,cinfo[i].geomBounds,recurse(threadIndex,depth+1,cbegin[i],cend[i],cbounds[i]));
    return parent->bvh->encodeNode(node);
  }

  Builder* BVH2HairBuilder_ (BVH2Hair* accel, Scene* scene) {
    return new BVH2HairBuilder(accel,scene);
  }
}
