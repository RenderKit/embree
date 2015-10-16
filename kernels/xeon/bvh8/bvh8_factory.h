// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#pragma once

#include "../bvh/bvh.h"
#include "../../common/isa.h"
#include "../../common/accel.h"
#include "../../common/scene.h"

namespace embree
{
  /*! BVH8 instantiations */
  struct BVH8Factory
  {
  public:
    BVH8Factory();

  public:
    Accel* BVH8Triangle4(Scene* scene);
    Accel* BVH8Triangle4ObjectSplit(Scene* scene);
    Accel* BVH8Triangle4SpatialSplit(Scene* scene);

    Accel* BVH8Triangle8(Scene* scene);
    //Accel* BVH8Triangle8v(Scene* scene);
    Accel* BVH8TrianglePairs4ObjectSplit(Scene* scene);
    Accel* BVH8Triangle8ObjectSplit(Scene* scene);
    Accel* BVH8Triangle8SpatialSplit(Scene* scene);
    //Accel* BVH8Triangle8vObjectSplit(Scene* scene);
    //Accel* BVH8Triangle8vSpatialSplit(Scene* scene);
    
  private:
    Accel::Intersectors BVH8Triangle4Intersectors(BVH8* bvh);
    Accel::Intersectors BVH8Triangle8Intersectors(BVH8* bvh);
    Accel::Intersectors BVH8TrianglePairs4Intersectors(BVH8* bvh);

  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8ChunkMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);
    
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle8Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8ChunkMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoellerNoFilter);
    
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8TrianglePairs4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8TrianglePairs4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8TrianglePairs4Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8TrianglePairs4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8TrianglePairs4Intersector8HybridMoellerNoFilter);
    
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16ChunkMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16ChunkMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoellerNoFilter);
    
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8TrianglePairs4Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8TrianglePairs4Intersector16HybridMoellerNoFilter);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle8SceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8TrianglePairs4SceneBuilderSAH);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSpatialSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle8SceneBuilderSpatialSAH);
  };
}
