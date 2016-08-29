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

#include "bvh4_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/bezier1v.h"
#include "../geometry/bezier1i.h"
#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/quadi_mb.h"
#include "../geometry/subdivpatch1cached.h"
#include "../geometry/object.h"
#include "../common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Line4iIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Line4iMBIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Bezier1vIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Bezier1iIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Bezier1vIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Bezier1iIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Bezier1iMBIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4XfmTriangle4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vMBIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Subdivpatch1CachedIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4GridAOSIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4VirtualIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4VirtualMBIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Quad4vIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Quad4iIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH4Quad4iMBIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,QBVH4Triangle4iIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,QBVH4Quad4iIntersector1Pluecker);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Line4iIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Line4iMBIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Bezier1vIntersector4Single);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Bezier1iIntersector4Single);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Bezier1vIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Bezier1iIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Bezier1iMBIntersector4Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Triangle4vIntersector4HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Triangle4iIntersector4HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Triangle4vMBIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Quad4iIntersector4HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Quad4iMBIntersector4HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4Subdivpatch1CachedIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4GridAOSIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4VirtualIntersector4Chunk);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH4VirtualMBIntersector4Chunk);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Line4iIntersector8);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Line4iMBIntersector8);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Bezier1vIntersector8Single);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Bezier1iIntersector8Single);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Bezier1vIntersector8Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Bezier1iIntersector8Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Bezier1iMBIntersector8Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vIntersector8HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iIntersector8HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vMBIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Quad4iIntersector8HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Quad4iMBIntersector8HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4Subdivpatch1CachedIntersector8);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4GridAOSIntersector8);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4VirtualIntersector8Chunk);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH4VirtualMBIntersector8Chunk);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Line4iIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Line4iMBIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Bezier1vIntersector16Single);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Bezier1iIntersector16Single);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Bezier1vIntersector16Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Bezier1iIntersector16Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Bezier1iMBIntersector16Single_OBB);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Triangle4vIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Triangle4iIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Triangle4vMBIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Quad4vIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Quad4vIntersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Quad4iIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Quad4iMBIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4Subdivpatch1CachedIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4GridAOSIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4VirtualIntersector16Chunk);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH4VirtualMBIntersector16Chunk);

  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Line4iStreamIntersector);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Line4iMBStreamIntersector);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1vStreamIntersector);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1iStreamIntersector);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1vStreamIntersector_OBB);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1iStreamIntersector_OBB);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1iMBStreamIntersector_OBB);
  DECLARE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4StreamIntersectorMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4StreamIntersectorMoellerNoFilter);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4XfmTriangle4StreamIntersectorMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Triangle4vStreamIntersectorPluecker);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Triangle4iStreamIntersectorPluecker);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Triangle4vMBStreamIntersectorMoeller);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Subdivpatch1CachedStreamIntersector);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4GridAOSStreamIntersector);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4VirtualStreamIntersector);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4VirtualMBStreamIntersector);
  DECLARE_SYMBOL2(Accel::IntersectorN, BVH4Quad4vStreamIntersectorMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN, BVH4Quad4vStreamIntersectorMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Quad4iStreamIntersectorPluecker);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH4Quad4iMBStreamIntersectorPluecker);
  //DECLARE_SYMBOL2(Accel::IntersectorN,QBVH4Triangle4StreamIntersectorMoeller);

  DECLARE_BUILDER2(void,Scene,const createLineSegmentsAccelTy,BVH4BuilderTwoLevelLineSegmentsSAH);
  DECLARE_BUILDER2(void,Scene,const createTriangleMeshAccelTy,BVH4BuilderTwoLevelTriangleMeshSAH);
  DECLARE_BUILDER2(void,Scene,const createTriangleMeshAccelTy,BVH4BuilderInstancingTriangleMeshSAH);
  //DECLARE_BUILDER2(void,Scene,const createQuadMeshAccelTy,BVH4BuilderTwoLevelQuadMeshSAH);
  //DECLARE_BUILDER2(void,Scene,const createQuadMeshAccelTy,BVH4BuilderInstancingQuadMeshSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH4Bezier1vBuilder_OBB_New);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Bezier1iBuilder_OBB_New);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Bezier1iMBBuilder_OBB_New);

  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4SceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4vSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4vMBSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4QuantizedTriangle4iSceneBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH4Quad4vSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Quad4iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Quad4iMBSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4QuantizedQuad4iSceneBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4SceneBuilderSpatialSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4vSceneBuilderSpatialSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4iSceneBuilderSpatialSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4SceneBuilderFastSpatialSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4vSceneBuilderFastSpatialSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Triangle4iSceneBuilderFastSpatialSAH);

  DECLARE_BUILDER2(void,LineSegments,size_t,BVH4Line4iMeshBuilderSAH);
  //DECLARE_BUILDER2(void,LineSegments,size_t,BVH4Line4iMBMeshBuilderSAH);
  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4MeshBuilderSAH);
  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4vMeshBuilderSAH);
  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4iMeshBuilderSAH);
  //DECLARE_BUILDER2(void,QuadMesh,size_t,BVH4Quad4vMeshBuilderSAH);
  //DECLARE_BUILDER2(void,QuadMesh,size_t,BVH4Quad4iMeshBuilderSAH);
  DECLARE_BUILDER2(void,QuadMesh,size_t,BVH4Quad4iMBMeshBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH4Bezier1vSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Bezier1iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Line4iSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4Line4iMBSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4VirtualSceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4VirtualMBSceneBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH4SubdivPatch1CachedBuilderBinnedSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH4SubdivGridEagerBuilderBinnedSAH);

  DECLARE_BUILDER2(void,LineSegments,size_t,BVH4Line4iMeshRefitSAH);
  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4MeshRefitSAH);
  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4vMeshRefitSAH);
  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4iMeshRefitSAH);

  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4MeshBuilderMortonGeneral);
  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4vMeshBuilderMortonGeneral);
  DECLARE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4iMeshBuilderMortonGeneral);

  BVH4Factory::BVH4Factory (int features)
  {
    /* select builders */
    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4BuilderTwoLevelLineSegmentsSAH));
    IF_ENABLED_TRIS (SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4BuilderTwoLevelTriangleMeshSAH));
    IF_ENABLED_TRIS (SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4BuilderInstancingTriangleMeshSAH));

    //IF_ENABLED_QUADS (SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4BuilderTwoLevelQuadMeshSAH));
    //IF_ENABLED_QUADS (SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4BuilderInstancingQuadMeshSAH));

    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1vBuilder_OBB_New));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iBuilder_OBB_New));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iMBBuilder_OBB_New));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4SceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4vSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4iSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMBSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4QuantizedTriangle4iSceneBuilderSAH));

    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Quad4vSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Quad4iSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Quad4iMBSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4QuantizedQuad4iSceneBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4SceneBuilderSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vSceneBuilderSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iSceneBuilderSpatialSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4SceneBuilderFastSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vSceneBuilderFastSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iSceneBuilderFastSpatialSAH));

    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Line4iMeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4MeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4vMeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4iMeshBuilderSAH));
    //IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Quad4vMeshBuilderSAH));
    //IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Quad4iMeshBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Quad4iMBMeshBuilderSAH));
    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Line4iSceneBuilderSAH));
    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Line4iMBSceneBuilderSAH));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1vSceneBuilderSAH));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iSceneBuilderSAH));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4VirtualSceneBuilderSAH));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4VirtualMBSceneBuilderSAH));

    IF_ENABLED_SUBDIV(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4SubdivPatch1CachedBuilderBinnedSAH));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivGridEagerBuilderBinnedSAH));

    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Line4iMeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4MeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4vMeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4iMeshRefitSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4MeshBuilderMortonGeneral));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4vMeshBuilderMortonGeneral));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,BVH4Triangle4iMeshBuilderMortonGeneral));

    /* select intersectors1 */
    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Line4iIntersector1));
    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Line4iMBIntersector1));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1vIntersector1));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iIntersector1));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1vIntersector1_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iIntersector1_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iMBIntersector1_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH4Triangle4Intersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH4XfmTriangle4Intersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX     (features,BVH4Triangle4vIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX     (features,BVH4Triangle4iIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Triangle4vMBIntersector1Moeller));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Subdivpatch1CachedIntersector1));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4GridAOSIntersector1));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4VirtualIntersector1));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4VirtualMBIntersector1));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Quad4vIntersector1Moeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Quad4iIntersector1Pluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Quad4iMBIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX(features,QBVH4Triangle4iIntersector1Pluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_SSE42_AVX(features,QBVH4Quad4iIntersector1Pluecker));

#if defined (EMBREE_RAY_PACKETS)

    /* select intersectors4 */
    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Line4iIntersector4));
    IF_ENABLED_LINES(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Line4iMBIntersector4));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Bezier1vIntersector4Single));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Bezier1iIntersector4Single));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Bezier1vIntersector4Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Bezier1iIntersector4Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Bezier1iMBIntersector4Single_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Triangle4Intersector4HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Triangle4Intersector4HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX(features,BVH4Triangle4vIntersector4HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX(features,BVH4Triangle4iIntersector4HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Triangle4vMBIntersector4HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Quad4vIntersector4HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4Quad4vIntersector4HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX     (features,BVH4Quad4iIntersector4HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_AVX     (features,BVH4Quad4iMBIntersector4HybridPluecker));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Subdivpatch1CachedIntersector4));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4GridAOSIntersector4));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4VirtualIntersector4Chunk));
    IF_ENABLED_USER(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4VirtualMBIntersector4Chunk));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,BVH4Quad4vIntersector4HybridMoeller));

    /* select intersectors8 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Line4iIntersector8));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Line4iMBIntersector8));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Bezier1vIntersector8Single));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Bezier1iIntersector8Single));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Bezier1vIntersector8Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Bezier1iIntersector8Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Bezier1iMBIntersector8Single_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Triangle4Intersector8HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Triangle4Intersector8HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX     (features,BVH4Triangle4vIntersector8HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX     (features,BVH4Triangle4iIntersector8HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Triangle4vMBIntersector8HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Quad4vIntersector8HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Quad4vIntersector8HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX     (features,BVH4Quad4iIntersector8HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX     (features,BVH4Quad4iMBIntersector8HybridPluecker));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4Subdivpatch1CachedIntersector8));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4GridAOSIntersector8));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4VirtualIntersector8Chunk));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH4VirtualMBIntersector8Chunk));

    /* select intersectors16 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Line4iIntersector16));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Line4iMBIntersector16));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Bezier1vIntersector16Single));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Bezier1iIntersector16Single));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Bezier1vIntersector16Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Bezier1iIntersector16Single_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Bezier1iMBIntersector16Single_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Triangle4Intersector16HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Triangle4Intersector16HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Triangle4vIntersector16HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Triangle4iIntersector16HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Triangle4vMBIntersector16HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Quad4vIntersector16HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Quad4vIntersector16HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Quad4iIntersector16HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Quad4iMBIntersector16HybridPluecker));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4Subdivpatch1CachedIntersector16));
    IF_ENABLED_SUBDIV(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4GridAOSIntersector16));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4VirtualIntersector16Chunk));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH4VirtualMBIntersector16Chunk));

    /* select stream intersectors */
    IF_ENABLED_LINES(SELECT_SYMBOL_SSE42_AVX_AVX2    (features,BVH4Line4iStreamIntersector));
    //IF_ENABLED_LINES(SELECT_SYMBOL_SSE42_AVX_AVX2      (features,BVH4Line4iMBStreamIntersector));
    IF_ENABLED_HAIR(SELECT_SYMBOL_SSE42_AVX_AVX2      (features,BVH4Bezier1vStreamIntersector));
    IF_ENABLED_HAIR(SELECT_SYMBOL_SSE42_AVX_AVX2      (features,BVH4Bezier1iStreamIntersector));
    //IF_ENABLED_HAIR(SELECT_SYMBOL_SSE42_AVX_AVX2      (features,BVH4Bezier1vStreamIntersector_OBB));
    //IF_ENABLED_HAIR(SELECT_SYMBOL_SSE42_AVX_AVX2      (features,BVH4Bezier1iStreamIntersector_OBB));
    //IF_ENABLED_HAIR(SELECT_SYMBOL_SSE42_AVX_AVX2      (features,BVH4Bezier1iMBStreamIntersector_OBB));
    IF_ENABLED_TRIS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Triangle4StreamIntersectorMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Triangle4StreamIntersectorMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Triangle4vStreamIntersectorPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Triangle4iStreamIntersectorPluecker));
    //IF_ENABLED_TRIS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Triangle4vMBStreamIntersectorMoeller));
    //IF_ENABLED_SUBDIV(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Subdivpatch1CachedStreamIntersector));
    //IF_ENABLED_SUBDIV(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4GridAOSStreamIntersector));
    IF_ENABLED_USER(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4VirtualStreamIntersector));
    //IF_ENABLED_USER(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4VirtualStreamMBIntersector));
    IF_ENABLED_QUADS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Quad4vStreamIntersectorMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Quad4vStreamIntersectorMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Quad4iStreamIntersectorPluecker));
    //IF_ENABLED_QUADS(SELECT_SYMBOL_SSE42_AVX_AVX2(features,BVH4Quad4iStreamMBIntersectorPluecker));

#endif
  }

  Accel::Intersectors BVH4Factory::BVH4Bezier1vIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Bezier1vIntersector1;
    intersectors.intersector4  = BVH4Bezier1vIntersector4Single;
    intersectors.intersector8  = BVH4Bezier1vIntersector8Single;
    intersectors.intersector16 = BVH4Bezier1vIntersector16Single;
    intersectors.intersectorN  = BVH4Bezier1vStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Bezier1iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Bezier1iIntersector1;
    intersectors.intersector4  = BVH4Bezier1iIntersector4Single;
    intersectors.intersector8  = BVH4Bezier1iIntersector8Single;
    intersectors.intersector16 = BVH4Bezier1iIntersector16Single;
    intersectors.intersectorN  = BVH4Bezier1iStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Line4iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Line4iIntersector1;
    intersectors.intersector4  = BVH4Line4iIntersector4;
    intersectors.intersector8  = BVH4Line4iIntersector8;
    intersectors.intersector16 = BVH4Line4iIntersector16;
    intersectors.intersectorN  = BVH4Line4iStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Line4iMBIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Line4iMBIntersector1;
    intersectors.intersector4  = BVH4Line4iMBIntersector4;
    intersectors.intersector8  = BVH4Line4iMBIntersector8;
    intersectors.intersector16 = BVH4Line4iMBIntersector16;
    //intersectors.intersectorN  = BVH4Line4iMBStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Bezier1vIntersectors_OBB(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Bezier1vIntersector1_OBB;
    intersectors.intersector4  = BVH4Bezier1vIntersector4Single_OBB;
    intersectors.intersector8  = BVH4Bezier1vIntersector8Single_OBB;
    intersectors.intersector16 = BVH4Bezier1vIntersector16Single_OBB;
    //intersectors.intersectorN  = BVH4Bezier1vStreamIntersector_OBB;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Bezier1iIntersectors_OBB(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Bezier1iIntersector1_OBB;
    intersectors.intersector4  = BVH4Bezier1iIntersector4Single_OBB;
    intersectors.intersector8  = BVH4Bezier1iIntersector8Single_OBB;
    intersectors.intersector16 = BVH4Bezier1iIntersector16Single_OBB;
    //intersectors.intersectorN  = BVH4Bezier1iStreamIntersector_OBB;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Bezier1iMBIntersectors_OBB(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Bezier1iMBIntersector1_OBB;
    intersectors.intersector4  = BVH4Bezier1iMBIntersector4Single_OBB;
    intersectors.intersector8  = BVH4Bezier1iMBIntersector8Single_OBB;
    intersectors.intersector16 = BVH4Bezier1iMBIntersector16Single_OBB;
    //intersectors.intersectorN  = BVH4Bezier1iMBStreamIntersector_OBB;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Triangle4IntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH4Triangle4Intersector1Moeller;
    intersectors.intersector4_filter    = BVH4Triangle4Intersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH4Triangle4Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH4Triangle4Intersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH4Triangle4Intersector8HybridMoellerNoFilter;
    intersectors.intersector16_filter   = BVH4Triangle4Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH4Triangle4Intersector16HybridMoellerNoFilter;
    intersectors.intersectorN_filter    = BVH4Triangle4StreamIntersectorMoeller;
    intersectors.intersectorN_nofilter  = BVH4Triangle4StreamIntersectorMoellerNoFilter;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Triangle4IntersectorsInstancing(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4XfmTriangle4Intersector1Moeller;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Triangle4vIntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Triangle4vIntersector1Pluecker;
    intersectors.intersector4  = BVH4Triangle4vIntersector4HybridPluecker;
    intersectors.intersector8  = BVH4Triangle4vIntersector8HybridPluecker;
    intersectors.intersector16 = BVH4Triangle4vIntersector16HybridPluecker;
    intersectors.intersectorN  = BVH4Triangle4vStreamIntersectorPluecker;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Triangle4iIntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Triangle4iIntersector1Pluecker;
    intersectors.intersector4  = BVH4Triangle4iIntersector4HybridPluecker;
    intersectors.intersector8  = BVH4Triangle4iIntersector8HybridPluecker;
    intersectors.intersector16 = BVH4Triangle4iIntersector16HybridPluecker;
    intersectors.intersectorN  = BVH4Triangle4iStreamIntersectorPluecker;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Triangle4vMBIntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Triangle4vMBIntersector1Moeller;
    intersectors.intersector4  = BVH4Triangle4vMBIntersector4HybridMoeller;
    intersectors.intersector8  = BVH4Triangle4vMBIntersector8HybridMoeller;
    intersectors.intersector16 = BVH4Triangle4vMBIntersector16HybridMoeller;
    //intersectors.intersectorN  = BVH4Triangle4vMBStreamIntersectorMoeller;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Quad4vIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH4Quad4vIntersector1Moeller;
    intersectors.intersector4_filter    = BVH4Quad4vIntersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH4Quad4vIntersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH4Quad4vIntersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH4Quad4vIntersector8HybridMoellerNoFilter;
    intersectors.intersector16_filter   = BVH4Quad4vIntersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH4Quad4vIntersector16HybridMoellerNoFilter;
    intersectors.intersectorN_filter    = BVH4Quad4vStreamIntersectorMoeller;
    intersectors.intersectorN_nofilter  = BVH4Quad4vStreamIntersectorMoellerNoFilter;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Quad4iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Quad4iIntersector1Pluecker;
    intersectors.intersector4 = BVH4Quad4iIntersector4HybridPluecker;
    intersectors.intersector8 = BVH4Quad4iIntersector8HybridPluecker;
    intersectors.intersector16= BVH4Quad4iIntersector16HybridPluecker;
    intersectors.intersectorN = BVH4Quad4iStreamIntersectorPluecker; 
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4Quad4iMBIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Quad4iMBIntersector1Pluecker;
    intersectors.intersector4 = BVH4Quad4iMBIntersector4HybridPluecker;
    intersectors.intersector8 = BVH4Quad4iMBIntersector8HybridPluecker;
    intersectors.intersector16= BVH4Quad4iMBIntersector16HybridPluecker;
    //intersectors.intersectorN = BVH4Quad4iMBStreamIntersectorPluecker;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4UserGeometryIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4VirtualIntersector1;
    intersectors.intersector4  = BVH4VirtualIntersector4Chunk;
    intersectors.intersector8  = BVH4VirtualIntersector8Chunk;
    intersectors.intersector16 = BVH4VirtualIntersector16Chunk;
    intersectors.intersectorN  = BVH4VirtualStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4UserGeometryMBIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4VirtualMBIntersector1;
    intersectors.intersector4  = BVH4VirtualMBIntersector4Chunk;
    intersectors.intersector8  = BVH4VirtualMBIntersector8Chunk;
    intersectors.intersector16 = BVH4VirtualMBIntersector16Chunk;
    //intersectors.intersectorN  = BVH4VirtualMBStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4SubdivPatch1CachedIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4Subdivpatch1CachedIntersector1;
    intersectors.intersector4  = BVH4Subdivpatch1CachedIntersector4;
    intersectors.intersector8  = BVH4Subdivpatch1CachedIntersector8;
    intersectors.intersector16 = BVH4Subdivpatch1CachedIntersector16;
    //intersectors.intersectorN  = BVH4Subdivpatch1CachedStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::BVH4SubdivGridEagerIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4GridAOSIntersector1;
    intersectors.intersector4  = BVH4GridAOSIntersector4;
    intersectors.intersector8  = BVH4GridAOSIntersector8;
    intersectors.intersector16 = BVH4GridAOSIntersector16;
    //intersectors.intersectorN  = BVH4GridAOSStreamIntersector;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::QBVH4Triangle4iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH4Triangle4iIntersector1Pluecker;
    return intersectors;
  }

  Accel::Intersectors BVH4Factory::QBVH4Quad4iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH4Quad4iIntersector1Pluecker;
    return intersectors;
  }


  void BVH4Factory::createLineSegmentsLine4i(LineSegments* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->parent->device->bvh4_factory;
    accel = new BVH4(Line4i::type,mesh->parent);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH4Line4iMeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH4Line4iMeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH4Line4iMeshBuilderSAH(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  void BVH4Factory::createTriangleMeshTriangle4Morton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->parent->device->bvh4_factory;
    accel = new BVH4(Triangle4::type,mesh->parent);
    builder = factory->BVH4Triangle4MeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH4Factory::createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->parent->device->bvh4_factory;
    accel = new BVH4(Triangle4v::type,mesh->parent);
    builder = factory->BVH4Triangle4vMeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH4Factory::createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->parent->device->bvh4_factory;
    accel = new BVH4(Triangle4i::type,mesh->parent);
    builder = factory->BVH4Triangle4iMeshBuilderMortonGeneral(accel,mesh,0); 
  }

  void BVH4Factory::createTriangleMeshTriangle4(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->parent->device->bvh4_factory;
    accel = new BVH4(Triangle4::type,mesh->parent);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH4Triangle4MeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH4Triangle4MeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH4Triangle4MeshBuilderMortonGeneral(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  void BVH4Factory::createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->parent->device->bvh4_factory;
    accel = new BVH4(Triangle4v::type,mesh->parent);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH4Triangle4vMeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH4Triangle4vMeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH4Triangle4vMeshBuilderMortonGeneral(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  void BVH4Factory::createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH4Factory* factory = mesh->parent->device->bvh4_factory;
    accel = new BVH4(Triangle4i::type,mesh->parent);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH4Triangle4iMeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH4Triangle4iMeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH4Triangle4iMeshBuilderMortonGeneral(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  Accel* BVH4Factory::BVH4Bezier1v(Scene* scene)
  {
    BVH4* accel = new BVH4(Bezier1v::type,scene);
    Accel::Intersectors intersectors = BVH4Bezier1vIntersectors(accel);
    Builder* builder = BVH4Bezier1vSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Bezier1i(Scene* scene)
  {
    BVH4* accel = new BVH4(Bezier1i::type,scene);
    Accel::Intersectors intersectors = BVH4Bezier1iIntersectors(accel);
    Builder* builder = BVH4Bezier1iSceneBuilderSAH(accel,scene,0);
    scene->needBezierVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Line4i(Scene* scene)
  {
    BVH4* accel = new BVH4(Line4i::type,scene);
    Accel::Intersectors intersectors = BVH4Line4iIntersectors(accel);

    Builder* builder = nullptr;
    if      (scene->device->line_builder == "default"     ) builder = BVH4Line4iSceneBuilderSAH(accel,scene,0);
    else if (scene->device->line_builder == "sah"         ) builder = BVH4Line4iSceneBuilderSAH(accel,scene,0);
    else if (scene->device->line_builder == "dynamic"     ) builder = BVH4BuilderTwoLevelLineSegmentsSAH(accel,scene,&createLineSegmentsLine4i);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->line_builder+" for BVH4<Line4i>");

    scene->needLineVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Line4iMB(Scene* scene)
  {
    BVH4* accel = new BVH4(Line4i::type,scene);
    Accel::Intersectors intersectors = BVH4Line4iMBIntersectors(accel);
    Builder* builder = BVH4Line4iMBSceneBuilderSAH(accel,scene,0);
    scene->needLineVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Line4iTwolevel(Scene* scene)
  {
    BVH4* accel = new BVH4(Line4i::type,scene);
    Accel::Intersectors intersectors = BVH4Line4iIntersectors(accel);
    Builder* builder = BVH4BuilderTwoLevelLineSegmentsSAH(accel,scene,&createLineSegmentsLine4i);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4OBBBezier1v(Scene* scene, bool highQuality)
  {
    BVH4* accel = new BVH4(Bezier1v::type,scene);
    Accel::Intersectors intersectors = BVH4Bezier1vIntersectors_OBB(accel);
    Builder* builder = BVH4Bezier1vBuilder_OBB_New(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4OBBBezier1i(Scene* scene, bool highQuality)
  {
    BVH4* accel = new BVH4(Bezier1i::type,scene);
    Accel::Intersectors intersectors = BVH4Bezier1iIntersectors_OBB(accel);
    Builder* builder = BVH4Bezier1iBuilder_OBB_New(accel,scene,0);
    scene->needBezierVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

   Accel* BVH4Factory::BVH4OBBBezier1iMB(Scene* scene, bool highQuality)
  {
    BVH4* accel = new BVH4(Bezier1i::type,scene);
    Accel::Intersectors intersectors = BVH4Bezier1iMBIntersectors_OBB(accel);
    Builder* builder = BVH4Bezier1iMBBuilder_OBB_New(accel,scene,0);
    scene->needBezierVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4::type,scene);

    Accel::Intersectors intersectors;
    if      (scene->device->tri_traverser == "default") intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    else if (scene->device->tri_traverser == "hybrid" ) intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4>");

    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     ) builder = BVH4Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah"         ) builder = BVH4Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_spatial" ) builder = BVH4Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_fast_spatial" ) builder = BVH4Triangle4SceneBuilderFastSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH4Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (scene->device->tri_builder == "dynamic"     ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4);
    else if (scene->device->tri_builder == "morton"      ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4Morton);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH4<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4v(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4v::type,scene);

    Accel::Intersectors intersectors;
    if      (scene->device->tri_traverser == "default") intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    else if (scene->device->tri_traverser == "hybrid" ) intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4>");

    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     ) builder = BVH4Triangle4vSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah"         ) builder = BVH4Triangle4vSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_spatial" ) builder = BVH4Triangle4vSceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_fast_spatial" ) builder = BVH4Triangle4vSceneBuilderFastSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH4Triangle4vSceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (scene->device->tri_builder == "dynamic"     ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4v);
    else if (scene->device->tri_builder == "morton"      ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4vMorton);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH4<Triangle4v>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4i(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4i::type,scene);
    
    Accel::Intersectors intersectors;
    if      (scene->device->tri_traverser == "default") intersectors = BVH4Triangle4iIntersectorsHybrid(accel);
    else if (scene->device->tri_traverser == "hybrid" ) intersectors = BVH4Triangle4iIntersectorsHybrid(accel);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4i>");

    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     ) builder = BVH4Triangle4iSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah"         ) builder = BVH4Triangle4iSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_spatial" ) builder = BVH4Triangle4iSceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_fast_spatial" ) builder = BVH4Triangle4iSceneBuilderFastSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH4Triangle4iSceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (scene->device->tri_builder == "dynamic"     ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4i);
    else if (scene->device->tri_builder == "morton"      ) builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4iMorton);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH4<Triangle4i>");

    scene->needTriangleVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

   Accel* BVH4Factory::BVH4Triangle4vMB(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4vMB::type,scene);

     Accel::Intersectors intersectors;
    if      (scene->device->tri_traverser == "default") intersectors = BVH4Triangle4vMBIntersectorsHybrid(accel);
    else if (scene->device->tri_traverser == "hybrid" ) intersectors = BVH4Triangle4vMBIntersectorsHybrid(accel);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown traverser "+scene->device->tri_traverser+" for BVH4<Triangle4vMB>");

    Builder* builder = nullptr;
    if       (scene->device->tri_builder_mb == "default"    ) builder = BVH4Triangle4vMBSceneBuilderSAH(accel,scene,0);
    else  if (scene->device->tri_builder_mb == "sah") builder = BVH4Triangle4vMBSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder_mb+" for BVH4<Triangle4vMB>");
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4InstancedBVH4Triangle4ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4::type,scene);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsInstancing(accel);
    Builder* builder = BVH4BuilderInstancingTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4Twolevel(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4::type,scene);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    Builder* builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4vTwolevel(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4v::type,scene);
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    Builder* builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4v);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4iTwolevel(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4i::type,scene);
    Accel::Intersectors intersectors = BVH4Triangle4iIntersectorsHybrid(accel);
    Builder* builder = BVH4BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4i);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4SpatialSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4::type,scene);
    Builder* builder = NULL;
    if (scene->device->tri_builder == "sah_spatial" ) 
      builder = BVH4Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    else
      builder = BVH4Triangle4SceneBuilderFastSpatialSAH(accel,scene,0);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }


  Accel* BVH4Factory::BVH4Triangle4ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4::type,scene);
    Builder* builder = BVH4Triangle4SceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4v::type,scene);
    Builder* builder = BVH4Triangle4vSceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4iObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4i::type,scene);
    Builder* builder = BVH4Triangle4iSceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = BVH4Triangle4iIntersectorsHybrid(accel);
    scene->needTriangleVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4SubdivPatch1Cached(Scene* scene)
  {
    BVH4* accel = new BVH4(SubdivPatch1Cached::type,scene);
    Accel::Intersectors intersectors = BVH4SubdivPatch1CachedIntersectors(accel);
    Builder* builder = BVH4SubdivPatch1CachedBuilderBinnedSAH(accel,scene,0);
    scene->needSubdivIndices = false;
    scene->needSubdivVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4SubdivGridEager(Scene* scene)
  {
    BVH4* accel = new BVH4(SubdivPatch1Eager::type,scene);
    Accel::Intersectors intersectors = BVH4SubdivGridEagerIntersectors(accel);
    Builder* builder = BVH4SubdivGridEagerBuilderBinnedSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4UserGeometry(Scene* scene)
  {
    BVH4* accel = new BVH4(Object::type,scene);
    Accel::Intersectors intersectors = BVH4UserGeometryIntersectors(accel);
    Builder* builder = BVH4VirtualSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4UserGeometryMB(Scene* scene)
  {
    BVH4* accel = new BVH4(Object::type,scene);
    Accel::Intersectors intersectors = BVH4UserGeometryMBIntersectors(accel);
    Builder* builder = BVH4VirtualMBSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4ObjectSplit(TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(Triangle4::type,mesh->parent);
    Builder* builder = BVH4Triangle4MeshBuilderSAH(accel,mesh,0);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4vObjectSplit(TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(Triangle4v::type,mesh->parent);
    Builder* builder = BVH4Triangle4vMeshBuilderSAH(accel,mesh,0);
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Triangle4Refit(TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(Triangle4::type,mesh->parent);
    Builder* builder = BVH4Triangle4MeshRefitSAH(accel,mesh,0);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Quad4v(Scene* scene)
  {
    BVH4* accel = new BVH4(Quad4v::type,scene);
    Builder* builder = BVH4Quad4vSceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = BVH4Quad4vIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Quad4i(Scene* scene)
  {
    BVH4* accel = new BVH4(Quad4i::type,scene);
    Builder* builder = BVH4Quad4iSceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = BVH4Quad4iIntersectors(accel);
    scene->needQuadVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4Quad4iMB(Scene* scene)
  {
    BVH4* accel = new BVH4(Quad4iMB::type,scene);
    Builder* builder = BVH4Quad4iMBSceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = BVH4Quad4iMBIntersectors(accel);
    scene->needQuadVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4QuantizedQuad4i(Scene* scene)
  {
    BVH4* accel = new BVH4(Quad4i::type,scene);
    Builder* builder = BVH4QuantizedQuad4iSceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = QBVH4Quad4iIntersectors(accel);
    scene->needQuadVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Factory::BVH4QuantizedTriangle4i(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4i::type,scene);
    Builder* builder = BVH4QuantizedTriangle4iSceneBuilderSAH(accel,scene,0);
    Accel::Intersectors intersectors = QBVH4Triangle4iIntersectors(accel);
    scene->needTriangleVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }


}


