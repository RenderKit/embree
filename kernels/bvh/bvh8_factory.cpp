// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
#include "../geometry/trianglei.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/quadi_mb.h"
#include "../geometry/subdivpatch1cached.h"
#include "../common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Line4iIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Line4iMBIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Bezier1vIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Bezier1iIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Bezier1iMBIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vMBIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8GridAOSIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4vIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4iIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4iMBIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4iIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Quad4iIntersector1Pluecker);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Line4iIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Line4iMBIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Bezier1vIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Bezier1iIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Bezier1iMBIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
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

  
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vMBIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8GridAOSIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridPlueckerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iMBIntersector16HybridPluecker);

  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Line4iStreamIntersector);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Line4iMBStreamIntersector);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Bezier1vStreamIntersector_OBB);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Bezier1iStreamIntersector_OBB);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Bezier1iMBStreamIntersector_OBB);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4StreamIntersectorMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4StreamIntersectorMoellerNoFilter);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4vMBStreamIntersectorMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4vStreamIntersectorMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4vStreamIntersectorMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4iStreamIntersectorPluecker);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4iMBStreamIntersectorPluecker);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8GridAOSStreamIntersector);
  //DECLARE_SYMBOL2(Accel::IntersectorN,QBVH8Triangle4StreamIntersectorMoeller);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Bezier1vBuilder_OBB_New);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Bezier1iBuilder_OBB_New);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Bezier1iMBBuilder_OBB_New);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Line4iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Line4iMBSceneBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4vMBSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Quad4vSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Quad4iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Quad4iMBSceneBuilderSAH);
  //DECLARE_BUILDER2(void,QuadMesh,size_t,BVH8Quad4iMBMeshBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSpatialSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderFastSpatialSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8SubdivGridEagerBuilderBinnedSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8QuantizedTriangle4iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8QuantizedQuad4iSceneBuilderSAH);

  BVH8Factory::BVH8Factory (int features)
  {
    /* select builders */
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1vBuilder_OBB_New));

    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1vBuilder_OBB_New));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1iBuilder_OBB_New));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1iMBBuilder_OBB_New));

    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,BVH8Line4iSceneBuilderSAH));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,BVH8Line4iMBSceneBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,BVH8Triangle4SceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,BVH8Triangle4vMBSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,BVH8Quad4vSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,BVH8Quad4iSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,BVH8Quad4iMBSceneBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX(features,BVH8QuantizedTriangle4iSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX(features,BVH8QuantizedQuad4iSceneBuilderSAH));
   
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX(features,BVH8Triangle4SceneBuilderSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,BVH8Triangle4SceneBuilderFastSpatialSAH));

    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX(features,BVH8SubdivGridEagerBuilderBinnedSAH));

    /* select intersectors1 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Line4iIntersector1));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Line4iMBIntersector1));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Bezier1vIntersector1_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Bezier1iIntersector1_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Bezier1iMBIntersector1_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4vMBIntersector1Moeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersector1Moeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersector1Pluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iMBIntersector1Pluecker));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8GridAOSIntersector1));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Triangle4iIntersector1Pluecker));    
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Quad4iIntersector1Pluecker));

#if defined (EMBREE_RAY_PACKETS)

    /* select intersectors4 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Line4iIntersector4));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Line4iMBIntersector4));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1vIntersector4Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1iIntersector4Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1iMBIntersector4Single_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector4HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector4HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4vMBIntersector4HybridMoeller));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8GridAOSIntersector4));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4vIntersector4HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4vIntersector4HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iIntersector4HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iIntersector4HybridPlueckerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iMBIntersector4HybridPluecker));

    /* select intersectors8 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Line4iIntersector8));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Line4iMBIntersector8));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1vIntersector8Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1iIntersector8Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Bezier1iMBIntersector8Single_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector8HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector8HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4vMBIntersector8HybridMoeller));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8GridAOSIntersector8));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4vIntersector8HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4vIntersector8HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iIntersector8HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iIntersector8HybridPlueckerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iMBIntersector8HybridPluecker));

    /* select intersectors16 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Line4iIntersector16));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Line4iMBIntersector16));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Bezier1vIntersector16Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Bezier1iIntersector16Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Bezier1iMBIntersector16Single_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector16HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector16HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4vMBIntersector16HybridMoeller));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8GridAOSIntersector16));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersector16HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersector16HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersector16HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersector16HybridPlueckerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4iMBIntersector16HybridPluecker));

    /* select stream intersectors */
    //IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Line4iStreamIntersector));
    //IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Line4iMBStreamIntersector));
    //IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Bezier1vStreamIntersector_OBB));
    //IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Bezier1iStreamIntersector_OBB));
    //IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Bezier1iMBStreamIntersector_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4StreamIntersectorMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4StreamIntersectorMoellerNoFilter));
    //IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4vMBStreamIntersectorMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4vStreamIntersectorMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4vStreamIntersectorMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iStreamIntersectorPluecker));
    //IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iMBStreamIntersectorPluecker));
    //IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8GridAOSStreamIntersector));
    //IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Triangle4StreamIntersectorMoeller));

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
    //intersectors.intersectorN  = BVH8Bezier1vStreamIntersector_OBB;
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
    //intersectors.intersectorN  = BVH8Bezier1iStreamIntersector_OBB;
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
    //intersectors.intersectorN  = BVH8Bezier1iMBStreamIntersector_OBB;
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
    //intersectors.intersectorN  = BVH8Line4iStreamIntersector;
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
    //intersectors.intersectorN  = BVH8Line4iMBStreamIntersector;
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
    intersectors.intersector16_filter   = BVH8Triangle4Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Triangle4Intersector16HybridMoellerNoFilter;
    intersectors.intersectorN_filter    = BVH8Triangle4StreamIntersectorMoeller;
    intersectors.intersectorN_nofilter  = BVH8Triangle4StreamIntersectorMoellerNoFilter;
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
    //intersectors.intersectorN  = BVH8Triangle4vMBStreamIntersectorMoeller;
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
    intersectors.intersector16_filter   = BVH8Quad4vIntersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Quad4vIntersector16HybridMoellerNoFilter;
    intersectors.intersectorN_filter    = BVH8Quad4vStreamIntersectorMoeller;
    intersectors.intersectorN_nofilter  = BVH8Quad4vStreamIntersectorMoellerNoFilter;
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
    intersectors.intersectorN           = BVH8Quad4iStreamIntersectorPluecker; 
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
    //intersectors.intersectorN  = BVH8Quad4iMBStreamIntersectorPluecker;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8SubdivGridEagerIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8GridAOSIntersector1;
    intersectors.intersector4  = BVH8GridAOSIntersector4;
    intersectors.intersector8  = BVH8GridAOSIntersector8;
    intersectors.intersector16 = BVH8GridAOSIntersector16;
    //intersectors.intersectorN  = BVH8GridAOSStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::QBVH8Triangle4iIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH8Triangle4iIntersector1Pluecker;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::QBVH8Quad4iIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH8Quad4iIntersector1Pluecker;
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
    else if (scene->device->tri_builder == "sah_presplit")     builder = BVH8Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }


  Accel* BVH8Factory::BVH8QuantizedTriangle4i(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4i::type,scene);
    Accel::Intersectors intersectors = QBVH8Triangle4iIntersectors(accel);
    Builder* builder = BVH8QuantizedTriangle4iSceneBuilderSAH(accel,scene,0);
    scene->needTriangleVertices = true;
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
    Builder *builder = NULL;
    if (scene->device->tri_builder == "sah_spatial" ) 
      builder = BVH8Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    else
      builder = BVH8Triangle4SceneBuilderFastSpatialSAH(accel,scene,0);
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

  Accel* BVH8Factory::BVH8QuantizedQuad4i(Scene* scene)
  {
    BVH8* accel = new BVH8(Quad4i::type,scene);
    Accel::Intersectors intersectors = QBVH8Quad4iIntersectors(accel);
    Builder* builder = nullptr;
    if      (scene->device->quad_builder == "default"     ) builder = BVH8QuantizedQuad4iSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->quad_builder+" for QBVH8<Quad4i>");
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
    Accel::Intersectors intersectors = BVH8SubdivGridEagerIntersectors(accel);
    Builder* builder = BVH8SubdivGridEagerBuilderBinnedSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }
}

#endif
