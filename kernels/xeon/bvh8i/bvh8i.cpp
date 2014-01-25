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
  DECLARE_SYMBOL(Accel::Intersector1,BVH8iTriangle1Intersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8iTriangle1Intersector8ChunkMoeller);

  DECLARE_BUILDER(BVH8iTriangle8BuilderObjectSplit);

  void BVH8iRegister () 
  {
    int features = getCPUFeatures();
    
    SELECT_SYMBOL_AVX(features,BVH8iTriangle1Intersector1Moeller);
    SELECT_SYMBOL_AVX2(features,BVH8iTriangle1Intersector1Moeller);

    SELECT_SYMBOL_AVX(features,BVH8iTriangle1Intersector8ChunkMoeller);
    SELECT_SYMBOL_AVX2(features,BVH8iTriangle1Intersector8ChunkMoeller);
    
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
    FATAL("not implemented");

    return new AccelInstance(accel,builder,intersectors);
  }



}
