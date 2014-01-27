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

#include "common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL(Accel::Intersector1,BVH8iTriangle8Intersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8iTriangle8Intersector8ChunkMoeller);

  DECLARE_BUILDER(BVH8iTriangle8BuilderObjectSplit);

  void BVH8iRegister () 
  {
    int features = getCPUFeatures();
    
    SELECT_SYMBOL_AVX(features,BVH8iTriangle8Intersector1Moeller);
    SELECT_SYMBOL_AVX2(features,BVH8iTriangle8Intersector1Moeller);

    SELECT_SYMBOL_AVX(features,BVH8iTriangle8Intersector8ChunkMoeller);
    SELECT_SYMBOL_AVX2(features,BVH8iTriangle8Intersector8ChunkMoeller);
    
    SELECT_SYMBOL_AVX(features,BVH8iTriangle8BuilderObjectSplit);
  }

  // Accel* BVH8i::BVH8iTriangle1(Scene* scene)
  // { 
  //   BVH8i* accel = new BVH8i(SceneTriangle1::type);
  //   Builder* builder = BVH8iTriangle1BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);

  //   Accel::Intersectors intersectors;
  //   intersectors.ptr = accel;
  //   intersectors.intersector1 = BVH8iTriangle1Intersector1Moeller; //intersectors1.get("bvh8i.triangle1.moeller");
  //   //intersectors.intersector4 = intersectors4.get("bvh8i.triangle1.chunk.moeller");
  //   //intersectors.intersector8 = intersectors8.get("bvh8i.triangle1.chunk.moeller");
  //   intersectors.intersector8 = BVH8iTriangle1Intersector8ChunkMoeller; //intersectors8.get("bvh8i.triangle1.chunk");
  //   //intersectors.intersector16 = intersectors16.get("bvh4i.triangle4.chunk.moeller");

  //   return new AccelInstance(accel,builder,intersectors);
  // }


  Accel* BVH8i::BVH8iTriangle8(Scene* scene)
  { 
    BVH8i* accel = new BVH8i(SceneTriangle1::type);
    Builder* builder = BVH8iTriangle8BuilderObjectSplit(accel,&scene->flat_triangle_source_1,scene,1,inf);


    Accel::Intersectors intersectors;
    intersectors.ptr = accel;
    intersectors.intersector1 = BVH8iTriangle8Intersector1Moeller;
    intersectors.intersector4 = NULL;
    intersectors.intersector8 = BVH8iTriangle8Intersector8ChunkMoeller;

    return new AccelInstance(accel,builder,intersectors);
  }

#if defined (__AVX__)

  float BVH8i::sah8 () {
    return sah(bvh8i_base,bvh8i_root,bounds)/area(bounds);
  }

  float BVH8i::sah8 (BVH8iNode * base, NodeRef& node, const BBox3f& bounds)
  {
    float f = bounds.empty() ? 0.0f : area(bounds);

    if (node.isNode()) 
    {
      BVH8i::BVH8iNode* n = node.node(base);
      unsigned int children = n->numValidChildren();
      for (size_t c=0; c<n; c++) 
        f += sah(n->child(c),n->bounds(c));
      return f;
    }
    else 
    {
      size_t num; node.leaf(triPtr(),num);
      return f*num;
    }
  }
#endif

}
