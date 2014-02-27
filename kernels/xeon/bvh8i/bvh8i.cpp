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

#include "bvh8i.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle8.h"

#include "common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL(Accel::Intersector1,BVH8iTriangle8Intersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8iTriangle8Intersector8ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8iTriangle8Intersector8HybridMoeller);

  DECLARE_BUILDER(BVH8iTriangle8BuilderObjectSplit);

  void BVH8iRegister () 
  {
    int features = getCPUFeatures();
    
    SELECT_SYMBOL_AVX(features,BVH8iTriangle8Intersector1Moeller);
    SELECT_SYMBOL_AVX2(features,BVH8iTriangle8Intersector1Moeller);

    SELECT_SYMBOL_AVX(features,BVH8iTriangle8Intersector8ChunkMoeller);
    SELECT_SYMBOL_AVX2(features,BVH8iTriangle8Intersector8ChunkMoeller);

    SELECT_SYMBOL_AVX(features,BVH8iTriangle8Intersector8HybridMoeller);
    SELECT_SYMBOL_AVX2(features,BVH8iTriangle8Intersector8HybridMoeller);
    
    SELECT_SYMBOL_AVX(features,BVH8iTriangle8BuilderObjectSplit);
  }

  Accel* BVH8i::BVH8iTriangle8(Scene* scene)
  { 
    BVH8i* accel = new BVH8i(SceneTriangle8::type);
    Builder* builder = BVH8iTriangle8BuilderObjectSplit(accel,&scene->flat_triangle_source_1,scene,1,inf);


    Accel::Intersectors intersectors;
    intersectors.ptr = accel;
    intersectors.intersector1 = BVH8iTriangle8Intersector1Moeller;
    intersectors.intersector4 = NULL;
    intersectors.intersector8 = BVH8iTriangle8Intersector8HybridMoeller;

    return new AccelInstance(accel,builder,intersectors);
  }

#if defined (__AVX__) && 1

  float BVH8i::sah8 (Node * base, BVH4i::NodeRef& root, avxi &bvh8i_node_dist) {
    BVH8i::Node* n = (BVH8i::Node*)root.node(base);
    BBox3fa bounds = n->bounds();

    return sah8(base,root,bounds,bvh8i_node_dist)/area(bounds);
  }

  float BVH8i::sah8 (Node * base, NodeRef& node, const BBox3fa& bounds, avxi &bvh8i_node_dist)
  {
    float f = bounds.empty() ? 0.0f : area(bounds);

    if (node.isNode()) 
    {
      BVH8i::Node* n = (BVH8i::Node*)node.node(base);

      size_t children = n->numValidChildren();

      bvh8i_node_dist[children-1]++;

      for (size_t c=0; c<children; c++) 
        f += sah8(base,n->child(c),n->bounds(c),bvh8i_node_dist);
      return f;
    }
    else 
    {
      size_t num = node.items();
      return f*num;
    }
  }

  float BVH8i::sah8_quantized(Quantized8BitNode * base, BVH4i::NodeRef& root, avxi &bvh8i_node_dist) {
    BVH8i::Quantized8BitNode* n = (BVH8i::Quantized8BitNode*)root.node(base);
    BBox3fa bounds = n->bounds();

    return sah8_quantized(base,root,bounds,bvh8i_node_dist)/area(bounds);
  }

  float BVH8i::sah8_quantized(Quantized8BitNode * base, NodeRef& node, const BBox3fa& bounds, avxi &bvh8i_node_dist)
  {
    float f = bounds.empty() ? 0.0f : area(bounds);

    if (node.isNode()) 
    {
      BVH8i::Quantized8BitNode* n = (BVH8i::Quantized8BitNode*)node.node(base);

      size_t children = n->numValidChildren();
      assert(children > 0 && children <= 8);
      bvh8i_node_dist[children-1]++;

      for (size_t c=0; c<children; c++) 
        {
          f += sah8_quantized(base,n->child(c),n->bounds(c),bvh8i_node_dist);
        }
      return f;
    }
    else 
    {
      size_t num = node.items();
      return f*num;
    }

  }
#endif

}
