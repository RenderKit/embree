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

#if defined (__TARGET_AVX__)

#include "bvh8_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"
#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/quadi_mb.h"
#include "../geometry/subdivpatch1cached.h"
#include "../../common/accelinstance.h"

#define EXPERIMENTAL_INTERSECTOR16 0

namespace embree
{
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Line4iIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Line4iMBIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Bezier1vIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Bezier1iIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Bezier1iMBIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle8Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vMBIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8GridAOSIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4vIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4iIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4iMBIntersector1Pluecker);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Line4iIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Line4iMBIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Bezier1vIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Bezier1iIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Bezier1iMBIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vMBIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8GridAOSIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4iIntersector4HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4iIntersector4HybridPlueckerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4iMBIntersector4HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Line4iIntersector8);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Line4iMBIntersector8);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Bezier1vIntersector8Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Bezier1iIntersector8Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Bezier1iMBIntersector8Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vMBIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8GridAOSIntersector8);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4iIntersector8HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4iIntersector8HybridPlueckerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4iMBIntersector8HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Line4iIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Line4iMBIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Bezier1vIntersector16Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Bezier1iIntersector16Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Bezier1iMBIntersector16Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller2);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter2);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vMBIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8GridAOSIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridPlueckerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iMBIntersector16HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoeller2);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoellerNoFilter2);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Bezier1vBuilder_OBB_New);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Bezier1iBuilder_OBB_New);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Bezier1iMBBuilder_OBB_New);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Line4iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Line4iMBSceneBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle8SceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4vMBSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Quad4vSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Quad4iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Quad4iMBSceneBuilderSAH);
  DECLARE_BUILDER2(void,QuadMesh,size_t,BVH8Quad4iMBMeshBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSpatialSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle8SceneBuilderSpatialSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8SubdivGridEagerBuilderBinnedSAH);

  BVH8Factory::BVH8Factory (int features)
  {
    /* select builders */
    SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1vBuilder_OBB_New);
    SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1iBuilder_OBB_New);
    SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1iMBBuilder_OBB_New);

    SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Line4iSceneBuilderSAH);
    SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Line4iMBSceneBuilderSAH);

    SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4SceneBuilderSAH);
    SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle8SceneBuilderSAH);
    SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vMBSceneBuilderSAH);
    SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vSceneBuilderSAH);
    SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4iSceneBuilderSAH);
    SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4iMBSceneBuilderSAH);

    SELECT_SYMBOL_INIT_AVX(features,BVH8Triangle4SceneBuilderSpatialSAH);
    SELECT_SYMBOL_INIT_AVX(features,BVH8Triangle8SceneBuilderSpatialSAH);

    SELECT_SYMBOL_INIT_AVX(features,BVH8SubdivGridEagerBuilderBinnedSAH);

    /* select intersectors1 */
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Line4iIntersector1);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Line4iMBIntersector1);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Bezier1vIntersector1_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Bezier1iIntersector1_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Bezier1iMBIntersector1_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle4Intersector1Moeller);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle8Intersector1Moeller);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle4vMBIntersector1Moeller);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Quad4vIntersector1Moeller);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Quad4iIntersector1Pluecker);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Quad4iMBIntersector1Pluecker);
    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8GridAOSIntersector1);

#if defined (RTCORE_RAY_PACKETS)

    /* select intersectors4 */
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Line4iIntersector4);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Line4iMBIntersector4);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1vIntersector4Single_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1iIntersector4Single_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1iMBIntersector4Single_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector4HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector4HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector4HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector4HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4vMBIntersector4HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8GridAOSIntersector4);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4vIntersector4HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4vIntersector4HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iIntersector4HybridPluecker);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iIntersector4HybridPlueckerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iMBIntersector4HybridPluecker);

    /* select intersectors8 */
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Line4iIntersector8);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Line4iMBIntersector8);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1vIntersector8Single_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1iIntersector8Single_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1iMBIntersector8Single_OBB);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector8HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector8HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector8HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector8HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4vMBIntersector8HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8GridAOSIntersector8);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4vIntersector8HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4vIntersector8HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iIntersector8HybridPluecker);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iIntersector8HybridPlueckerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iMBIntersector8HybridPluecker);

    /* select intersectors16 */
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Line4iIntersector16);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Line4iMBIntersector16);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Bezier1vIntersector16Single_OBB);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Bezier1iIntersector16Single_OBB);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Bezier1iMBIntersector16Single_OBB);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Triangle4Intersector16HybridMoeller);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Triangle4Intersector16HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Triangle4Intersector16HybridMoeller2);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Triangle4Intersector16HybridMoellerNoFilter2);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Triangle4vMBIntersector16HybridMoeller);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8GridAOSIntersector16);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Quad4vIntersector16HybridMoeller);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Quad4vIntersector16HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Quad4iIntersector16HybridPluecker);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Quad4iIntersector16HybridPlueckerNoFilter);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Quad4iMBIntersector16HybridPluecker);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Quad4vIntersector16HybridMoeller2);
    SELECT_SYMBOL_INIT_AVX512KNL(features,BVH8Quad4vIntersector16HybridMoellerNoFilter2);

#endif
  }

  Accel::Intersectors BVH8Factory::BVH8Bezier1vIntersectors_OBB(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Bezier1vIntersector1_OBB;
    intersectors.intersector4  = BVH8Bezier1vIntersector4Single_OBB;
    intersectors.intersector8  = BVH8Bezier1vIntersector8Single_OBB;
    intersectors.intersector16 = BVH8Bezier1vIntersector16Single_OBB;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Bezier1iIntersectors_OBB(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Bezier1iIntersector1_OBB;
    intersectors.intersector4  = BVH8Bezier1iIntersector4Single_OBB;
    intersectors.intersector8  = BVH8Bezier1iIntersector8Single_OBB;
    intersectors.intersector16 = BVH8Bezier1iIntersector16Single_OBB;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Bezier1iMBIntersectors_OBB(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Bezier1iMBIntersector1_OBB;
    intersectors.intersector4  = BVH8Bezier1iMBIntersector4Single_OBB;
    intersectors.intersector8  = BVH8Bezier1iMBIntersector8Single_OBB;
    intersectors.intersector16 = BVH8Bezier1iMBIntersector16Single_OBB;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Line4iIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Line4iIntersector1;
    intersectors.intersector4  = BVH8Line4iIntersector4;
    intersectors.intersector8  = BVH8Line4iIntersector8;
    intersectors.intersector16 = BVH8Line4iIntersector16;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Line4iMBIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Line4iMBIntersector1;
    intersectors.intersector4  = BVH8Line4iMBIntersector4;
    intersectors.intersector8  = BVH8Line4iMBIntersector8;
    intersectors.intersector16 = BVH8Line4iMBIntersector16;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle4Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle4Intersector1Moeller;
    intersectors.intersector4_filter    = BVH8Triangle4Intersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH8Triangle4Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH8Triangle4Intersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH8Triangle4Intersector8HybridMoellerNoFilter;
#if EXPERIMENTAL_INTERSECTOR16 == 0
    intersectors.intersector16_filter   = BVH8Triangle4Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Triangle4Intersector16HybridMoellerNoFilter;
#else
    intersectors.intersector16_filter   = BVH8Triangle4Intersector16HybridMoeller2;
    intersectors.intersector16_nofilter = BVH8Triangle4Intersector16HybridMoellerNoFilter2;
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle8Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle8Intersector1Moeller;
    intersectors.intersector4_filter    = BVH8Triangle8Intersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH8Triangle8Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH8Triangle8Intersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH8Triangle8Intersector8HybridMoellerNoFilter;
    intersectors.intersector16_filter   = BVH8Triangle8Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Triangle8Intersector16HybridMoellerNoFilter;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle4vMBIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Triangle4vMBIntersector1Moeller;
    intersectors.intersector4  = BVH8Triangle4vMBIntersector4HybridMoeller;
    intersectors.intersector8  = BVH8Triangle4vMBIntersector8HybridMoeller;
    intersectors.intersector16 = BVH8Triangle4vMBIntersector16HybridMoeller;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Quad4vIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Quad4vIntersector1Moeller;
    intersectors.intersector4_filter    = BVH8Quad4vIntersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH8Quad4vIntersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH8Quad4vIntersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH8Quad4vIntersector8HybridMoellerNoFilter;
#if EXPERIMENTAL_INTERSECTOR16 == 0
    intersectors.intersector16_filter   = BVH8Quad4vIntersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Quad4vIntersector16HybridMoellerNoFilter;
#else
    intersectors.intersector16_filter   = BVH8Quad4vIntersector16HybridMoeller2;
    intersectors.intersector16_nofilter = BVH8Quad4vIntersector16HybridMoellerNoFilter2;
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Quad4iIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Quad4iIntersector1Pluecker;
    intersectors.intersector4_filter    = BVH8Quad4iIntersector4HybridPluecker;
    intersectors.intersector4_nofilter  = BVH8Quad4iIntersector4HybridPlueckerNoFilter;
    intersectors.intersector8_filter    = BVH8Quad4iIntersector8HybridPluecker;
    intersectors.intersector8_nofilter  = BVH8Quad4iIntersector8HybridPlueckerNoFilter;
    intersectors.intersector16_filter   = BVH8Quad4iIntersector16HybridPluecker;
    intersectors.intersector16_nofilter = BVH8Quad4iIntersector16HybridPlueckerNoFilter;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Quad4iMBIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Quad4iMBIntersector1Pluecker;
    intersectors.intersector4  = BVH8Quad4iMBIntersector4HybridPluecker;
    intersectors.intersector8  = BVH8Quad4iMBIntersector8HybridPluecker;
    intersectors.intersector16 = BVH8Quad4iMBIntersector16HybridPluecker;
    return intersectors;
  }

  Accel* BVH8Factory::BVH8OBBBezier1v(Scene* scene, bool highQuality)
  {
    BVH8* accel = new BVH8(Bezier1v::type,scene);
    Accel::Intersectors intersectors = BVH8Bezier1vIntersectors_OBB(accel);
    Builder* builder = BVH8Bezier1vBuilder_OBB_New(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8OBBBezier1i(Scene* scene, bool highQuality)
  {
    BVH8* accel = new BVH8(Bezier1i::type,scene);
    Accel::Intersectors intersectors = BVH8Bezier1iIntersectors_OBB(accel);
    Builder* builder = BVH8Bezier1iBuilder_OBB_New(accel,scene,0);
    scene->needBezierVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8OBBBezier1iMB(Scene* scene, bool highQuality)
  {
    BVH8* accel = new BVH8(Bezier1i::type,scene);
    Accel::Intersectors intersectors = BVH8Bezier1iMBIntersectors_OBB(accel);
    Builder* builder = BVH8Bezier1iMBBuilder_OBB_New(accel,scene,0);
    scene->needBezierVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Line4i(Scene* scene)
  {
    BVH8* accel = new BVH8(Line4i::type,scene);
    Accel::Intersectors intersectors = BVH8Line4iIntersectors(accel);
    Builder* builder = nullptr;
    if      (scene->device->line_builder == "default"     ) builder = BVH8Line4iSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->line_builder+" for BVH8<Line4i>");
    scene->needLineVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Line4iMB(Scene* scene)
  {
    BVH8* accel = new BVH8(Line4i::type,scene);
    Accel::Intersectors intersectors = BVH8Line4iMBIntersectors(accel);
    Builder* builder = nullptr;
    if      (scene->device->line_builder_mb == "default"     ) builder = BVH8Line4iMBSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->line_builder_mb+" for BVH8<Line4i>");
    scene->needLineVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);

    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     )  builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah"         )  builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_spatial" )  builder = BVH8Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH8Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4ObjectSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);
    Builder* builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4SpatialSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);
    Builder* builder = BVH8Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle8(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);

    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     ) builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah"         ) builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_spatial" ) builder = BVH8Triangle8SceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH8Triangle8SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle8>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4vMB(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4vMB::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4vMBIntersectors(accel);

    Builder* builder = nullptr;
    if      (scene->device->tri_builder_mb == "default"     )  builder = BVH8Triangle4vMBSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder_mb == "sah"         )  builder = BVH8Triangle4vMBSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder_mb+" for BVH8<Triangle4vMB>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle8ObjectSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);
    Builder* builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle8SpatialSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);
    Builder* builder = BVH8Triangle8SceneBuilderSpatialSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Quad4v(Scene* scene)
  {
    BVH8* accel = new BVH8(Quad4v::type,scene);
    Accel::Intersectors intersectors = BVH8Quad4vIntersectors(accel);
    Builder* builder = nullptr;
    if      (scene->device->quad_builder == "default"     ) builder = BVH8Quad4vSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->quad_builder+" for BVH8<Quad4v>");
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Quad4i(Scene* scene)
  {
    BVH8* accel = new BVH8(Quad4i::type,scene);
    Accel::Intersectors intersectors = BVH8Quad4iIntersectors(accel);
    Builder* builder = nullptr;
    if      (scene->device->quad_builder == "default"     ) builder = BVH8Quad4iSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->quad_builder+" for BVH8<Quad4i>");
    scene->needQuadVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Quad4iMB(Scene* scene)
  {
    BVH8* accel = new BVH8(Quad4iMB::type,scene);
    Accel::Intersectors intersectors = BVH8Quad4iMBIntersectors(accel);
    Builder* builder = nullptr;
    if      (scene->device->quad_builder_mb == "default"     ) builder = BVH8Quad4iMBSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->quad_builder_mb+" for BVH8<Quad4i>");
    scene->needQuadVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }


  Accel* BVH8Factory::BVH8SubdivGridEager(Scene* scene)
  {
    BVH8* accel = new BVH8(SubdivPatch1Eager::type,scene);
    Accel::Intersectors intersectors;
    intersectors.ptr = accel;
    intersectors.intersector1  = BVH8GridAOSIntersector1;
    intersectors.intersector4  = BVH8GridAOSIntersector4;
    intersectors.intersector8  = BVH8GridAOSIntersector8;
    intersectors.intersector16 = BVH8GridAOSIntersector16;
    Builder* builder = BVH8SubdivGridEagerBuilderBinnedSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }
}

#endif
