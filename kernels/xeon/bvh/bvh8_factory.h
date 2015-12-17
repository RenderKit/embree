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
  class BVH8Factory
  {
  public:
    BVH8Factory(int features);

  public:
    Accel* BVH8OBBBezier1v(Scene* scene, bool highQuality);
    Accel* BVH8OBBBezier1i(Scene* scene, bool highQuality);
    Accel* BVH8OBBBezier1iMB(Scene* scene, bool highQuality);

    Accel* BVH8Line4i(Scene* scene);
    Accel* BVH8Line4iMB(Scene* scene);

    Accel* BVH8Triangle4(Scene* scene);
    Accel* BVH8Triangle4ObjectSplit(Scene* scene);
    Accel* BVH8Triangle4SpatialSplit(Scene* scene);

    Accel* BVH8Triangle8(Scene* scene);
    //Accel* BVH8Triangle8v(Scene* scene);
    Accel* BVH8Triangle8ObjectSplit(Scene* scene);
    Accel* BVH8Triangle8SpatialSplit(Scene* scene);
    //Accel* BVH8Triangle8vObjectSplit(Scene* scene);
    //Accel* BVH8Triangle8vSpatialSplit(Scene* scene);
    Accel* BVH8Triangle4vMB(Scene* scene);
    Accel* BVH8SubdivGridEager(Scene* scene);
    Accel* BVH8Quad4v(Scene* scene);
    Accel* BVH8Quad4i(Scene* scene);
    Accel* BVH8Quad4iMB(Scene* scene);
    
  private:
    Accel::Intersectors BVH8Line4iIntersectors(BVH8* bvh);
    Accel::Intersectors BVH8Line4iMBIntersectors(BVH8* bvh);
    Accel::Intersectors BVH8Bezier1vIntersectors_OBB(BVH8* bvh);
    Accel::Intersectors BVH8Bezier1iIntersectors_OBB(BVH8* bvh);
    Accel::Intersectors BVH8Bezier1iMBIntersectors_OBB(BVH8* bvh);
    Accel::Intersectors BVH8Triangle4Intersectors(BVH8* bvh);
    Accel::Intersectors BVH8Triangle8Intersectors(BVH8* bvh);
    Accel::Intersectors BVH8Triangle4vMBIntersectors(BVH8* bvh);
    Accel::Intersectors BVH8Quad4vIntersectors(BVH8* bvh);
    Accel::Intersectors BVH8Quad4iIntersectors(BVH8* bvh);
    Accel::Intersectors BVH8Quad4iMBIntersectors(BVH8* bvh);

  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Line4iIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Line4iMBIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Bezier1vIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Bezier1iIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Bezier1iMBIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle8Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4vIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4iIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4iMBIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8GridAOSIntersector1);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Line4iIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Line4iMBIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Bezier1vIntersector4Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Bezier1iIntersector4Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Bezier1iMBIntersector4Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8GridAOSIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4iIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4iIntersector4HybridPlueckerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4iMBIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Line4iIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Line4iMBIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Bezier1vIntersector8Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Bezier1iIntersector8Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Bezier1iMBIntersector8Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8GridAOSIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4iIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4iIntersector8HybridPlueckerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4iMBIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Line4iIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Line4iMBIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Bezier1vIntersector16Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Bezier1iIntersector16Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Bezier1iMBIntersector16Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vMBIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8GridAOSIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridPlueckerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4iMBIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoeller2);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoellerNoFilter2);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller2);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter2);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Bezier1vBuilder_OBB_New);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Bezier1iBuilder_OBB_New);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Bezier1iMBBuilder_OBB_New);

    DEFINE_BUILDER2(void,Scene,size_t,BVH8Line4iSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Line4iMBSceneBuilderSAH);

    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle8SceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle4vMBSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Quad4vSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Quad4iSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Quad4iMBSceneBuilderSAH);
    DEFINE_BUILDER2(void,QuadMesh,size_t,BVH8Quad4iMBMeshBuilderSAH);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSpatialSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH8Triangle8SceneBuilderSpatialSAH);

    DEFINE_BUILDER2(void,Scene,size_t,BVH8SubdivGridEagerBuilderBinnedSAH);
  };
}
