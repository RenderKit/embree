// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "../common/isa.h" // to define EMBREE_TARGET_SIMD8

#if defined (EMBREE_TARGET_SIMD8)

#include "bvh8_factory.h"
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
#include "../geometry/subdivpatch1cached.h"
#include "../geometry/object.h"
#include "../common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Line4iIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Line4iMBIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Bezier1vIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Bezier1iIntersector1_OBB);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8OBBBezier1iMBIntersector1_OBB);

  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iIntersector1Pluecker);

  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vMBIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iMBIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vMBIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iMBIntersector1Pluecker);

  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4vIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4iIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4vIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4iIntersector1Pluecker);

  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4iMBIntersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Quad4iMBIntersector1Pluecker);

  DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4iIntersector1Pluecker);
  DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector1,QBVH8Quad4iIntersector1Pluecker);

  DECLARE_SYMBOL2(Accel::Intersector1,BVH8VirtualIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8VirtualMBIntersector1);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Line4iIntersector4);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Line4iMBIntersector4);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Bezier1vIntersector4Hybrid_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Bezier1iIntersector4Hybrid_OBB);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8OBBBezier1iMBIntersector4Hybrid_OBB);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vIntersector4HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iIntersector4HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vMBIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iMBIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vMBIntersector4HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iMBIntersector4HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4iIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4iIntersector4HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4iMBIntersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Quad4iMBIntersector4HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector4,BVH8VirtualIntersector4Chunk);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8VirtualMBIntersector4Chunk);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Line4iIntersector8);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Line4iMBIntersector8);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Bezier1vIntersector8Hybrid_OBB);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Bezier1iIntersector8Hybrid_OBB);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8OBBBezier1iMBIntersector8Hybrid_OBB);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vIntersector8HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iIntersector8HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vMBIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iMBIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vMBIntersector8HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iMBIntersector8HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4iIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4iIntersector8HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4iMBIntersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Quad4iMBIntersector8HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector8,BVH8VirtualIntersector8Chunk);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8VirtualMBIntersector8Chunk);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Line4iIntersector16);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Line4iMBIntersector16);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Bezier1vIntersector16Hybrid_OBB);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Bezier1iIntersector16Hybrid_OBB);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8OBBBezier1iMBIntersector16Hybrid_OBB);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iIntersector16HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vMBIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iMBIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vMBIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iMBIntersector16HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridPluecker);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iMBIntersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Quad4iMBIntersector16HybridPluecker);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8VirtualIntersector16Chunk);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8VirtualMBIntersector16Chunk);

  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8IntersectorStreamPacketFallback);

  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Line4iIntersectorStream);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Bezier1vIntersectorStream_OBB);
  //DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Bezier1iIntersectorStream_OBB);

  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4IntersectorStreamMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4IntersectorStreamMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4iIntersectorStreamMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4vIntersectorStreamPluecker);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4iIntersectorStreamPluecker);

  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4vIntersectorStreamMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4vIntersectorStreamMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4iIntersectorStreamMoeller);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4vIntersectorStreamPluecker);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8Quad4iIntersectorStreamPluecker);

  DECLARE_SYMBOL2(Accel::IntersectorN,BVH8VirtualIntersectorStream);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Line4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Line4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Bezier1vBuilder_OBB_New,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Bezier1iBuilder_OBB_New,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8OBBBezier1iMBBuilder_OBB,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8QuantizedTriangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8QuantizedTriangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8QuantizedQuad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
  DECLARE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelQuadMeshSAH,void* COMMA Scene* COMMA const createQuadMeshAccelTy);
  DECLARE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelVirtualSAH,void* COMMA Scene* COMMA const createAccelSetAccelTy);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshBuilderSAH,void* COMMA QuadMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualMeshBuilderSAH,void* COMMA AccelSet* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshRefitSAH,void* COMMA QuadMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualMeshRefitSAH,void* COMMA AccelSet* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshBuilderMortonGeneral,void* COMMA QuadMesh* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVH8VirtualMeshBuilderMortonGeneral,void* COMMA AccelSet* COMMA size_t);

  BVH8Factory::BVH8Factory(int bfeatures, int ifeatures)
  {
    selectBuilders(bfeatures);
    selectIntersectors(ifeatures);
  }

  void BVH8Factory::selectBuilders(int features)
  {
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Line4iSceneBuilderSAH));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Line4iMBSceneBuilderSAH));

    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1vBuilder_OBB_New));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1vBuilder_OBB_New));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX(features,BVH8Bezier1iBuilder_OBB_New));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX(features,BVH8OBBBezier1iMBBuilder_OBB));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4SceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4iSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4iMBSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vMBSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX(features,BVH8QuantizedTriangle4iSceneBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX(features,BVH8QuantizedTriangle4SceneBuilderSAH));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4iSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4iMBSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX(features,BVH8QuantizedQuad4iSceneBuilderSAH));

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX(features,BVH8VirtualSceneBuilderSAH));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX(features,BVH8VirtualMBSceneBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4SceneBuilderFastSpatialSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vSceneBuilderFastSpatialSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vSceneBuilderFastSpatialSAH));

    IF_ENABLED_TRIS  (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelTriangleMeshSAH));
    IF_ENABLED_QUADS (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelQuadMeshSAH));
    IF_ENABLED_USER  (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8BuilderTwoLevelVirtualSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4MeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vMeshBuilderSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4iMeshBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vMeshBuilderSAH));
    IF_ENABLED_USER (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8VirtualMeshBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4MeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4vMeshRefitSAH));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Triangle4iMeshRefitSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8Quad4vMeshRefitSAH));
    IF_ENABLED_USER (SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,BVH8VirtualMeshRefitSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle4MeshBuilderMortonGeneral));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle4vMeshBuilderMortonGeneral));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Triangle4iMeshBuilderMortonGeneral));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8Quad4vMeshBuilderMortonGeneral));
    IF_ENABLED_USER (SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,BVH8VirtualMeshBuilderMortonGeneral));
  }

  void BVH8Factory::selectIntersectors(int features)
  {
    /* select intersectors1 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Line4iIntersector1));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Line4iMBIntersector1));

    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Bezier1vIntersector1_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Bezier1iIntersector1_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8OBBBezier1iMBIntersector1_OBB));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4iIntersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4vIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4iIntersector1Pluecker));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4vMBIntersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4iMBIntersector1Moeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4vMBIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4iMBIntersector1Pluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersector1Moeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersector1Moeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersector1Pluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersector1Pluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iMBIntersector1Moeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iMBIntersector1Pluecker));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Triangle4iIntersector1Pluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Triangle4Intersector1Moeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,QBVH8Quad4iIntersector1Pluecker));

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8VirtualIntersector1));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8VirtualMBIntersector1));

#if defined (EMBREE_RAY_PACKETS)

    /* select intersectors4 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Line4iIntersector4));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Line4iMBIntersector4));

    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Bezier1vIntersector4Hybrid_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Bezier1iIntersector4Hybrid_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8OBBBezier1iMBIntersector4Hybrid_OBB));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4Intersector4HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4Intersector4HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4iIntersector4HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4vIntersector4HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4iIntersector4HybridPluecker));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4vMBIntersector4HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4iMBIntersector4HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4vMBIntersector4HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4iMBIntersector4HybridPluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4vIntersector4HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4vIntersector4HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4iIntersector4HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4vIntersector4HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4iIntersector4HybridPluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iMBIntersector4HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iMBIntersector4HybridPluecker));

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8VirtualIntersector4Chunk));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8VirtualMBIntersector4Chunk));

    /* select intersectors8 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Line4iIntersector8));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Line4iMBIntersector8));

    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Bezier1vIntersector8Hybrid_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Bezier1iIntersector8Hybrid_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8OBBBezier1iMBIntersector8Hybrid_OBB));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4Intersector8HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4Intersector8HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4iIntersector8HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4vIntersector8HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4iIntersector8HybridPluecker));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4vMBIntersector8HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4iMBIntersector8HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4vMBIntersector8HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Triangle4iMBIntersector8HybridPluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4vIntersector8HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4vIntersector8HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4iIntersector8HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4vIntersector8HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8Quad4iIntersector8HybridPluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iMBIntersector8HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Quad4iMBIntersector8HybridPluecker));

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8VirtualIntersector8Chunk));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,BVH8VirtualMBIntersector8Chunk));

    /* select intersectors16 */
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Line4iIntersector16));
    IF_ENABLED_LINES(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Line4iMBIntersector16));

    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Bezier1vIntersector16Hybrid_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Bezier1iIntersector16Hybrid_OBB));
    IF_ENABLED_HAIR(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8OBBBezier1iMBIntersector16Hybrid_OBB));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector16HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4Intersector16HybridMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4iIntersector16HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4vIntersector16HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4iIntersector16HybridPluecker));

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4vMBIntersector16HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4iMBIntersector16HybridMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4vMBIntersector16HybridPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Triangle4iMBIntersector16HybridPluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersector16HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersector16HybridMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersector16HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersector16HybridPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersector16HybridPluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4iMBIntersector16HybridMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8Quad4iMBIntersector16HybridPluecker));

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8VirtualIntersector16Chunk));
    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,BVH8VirtualMBIntersector16Chunk));

    /* select stream intersectors */

    SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8IntersectorStreamPacketFallback);

    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4IntersectorStreamMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4IntersectorStreamMoellerNoFilter));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4iIntersectorStreamMoeller));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4vIntersectorStreamPluecker));
    IF_ENABLED_TRIS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Triangle4iIntersectorStreamPluecker));

    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersectorStreamMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersectorStreamMoellerNoFilter));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersectorStreamMoeller));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4vIntersectorStreamPluecker));
    IF_ENABLED_QUADS(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8Quad4iIntersectorStreamPluecker));

    IF_ENABLED_USER(SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVH8VirtualIntersectorStream));

#endif
  }

  void BVH8Factory::createTriangleMeshTriangle4Morton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4::type,mesh->scene);
    builder = factory->BVH8Triangle4MeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH8Factory::createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4v::type,mesh->scene);
    builder = factory->BVH8Triangle4vMeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH8Factory::createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4i::type,mesh->scene);
    builder = factory->BVH8Triangle4iMeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH8Factory::createTriangleMeshTriangle4(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4::type,mesh->scene);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH8Triangle4MeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH8Triangle4MeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH8Triangle4MeshBuilderMortonGeneral(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  void BVH8Factory::createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4v::type,mesh->scene);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH8Triangle4vMeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH8Triangle4vMeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH8Triangle4vMeshBuilderMortonGeneral(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  void BVH8Factory::createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Triangle4i::type,mesh->scene);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH8Triangle4iMeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH8Triangle4iMeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH8Triangle4iMeshBuilderMortonGeneral(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  void BVH8Factory::createQuadMeshQuad4v(QuadMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Quad4v::type,mesh->scene);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH8Quad4vMeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH8Quad4vMeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH8Quad4vMeshBuilderMortonGeneral(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  void BVH8Factory::createQuadMeshQuad4vMorton(QuadMesh* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Quad4v::type,mesh->scene);
    builder = factory->BVH8Quad4vMeshBuilderMortonGeneral(accel,mesh,0);
  }

  void BVH8Factory::createAccelSetMesh(AccelSet* mesh, AccelData*& accel, Builder*& builder)
  {
    BVH8Factory* factory = mesh->scene->device->bvh8_factory.get();
    accel = new BVH8(Object::type,mesh->scene);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = factory->BVH8VirtualMeshBuilderSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = factory->BVH8VirtualMeshRefitSAH(accel,mesh,0); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = factory->BVH8VirtualMeshBuilderMortonGeneral(accel,mesh,0); break;
    default: throw_RTCError(RTC_UNKNOWN_ERROR,"invalid geometry flag");
    }
  }

  Accel::Intersectors BVH8Factory::BVH8Bezier1vIntersectors_OBB(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Bezier1vIntersector1_OBB();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4  = BVH8Bezier1vIntersector4Hybrid_OBB();
    intersectors.intersector8  = BVH8Bezier1vIntersector8Hybrid_OBB();
    intersectors.intersector16 = BVH8Bezier1vIntersector16Hybrid_OBB();
    intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Bezier1iIntersectors_OBB(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Bezier1iIntersector1_OBB();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4  = BVH8Bezier1iIntersector4Hybrid_OBB();
    intersectors.intersector8  = BVH8Bezier1iIntersector8Hybrid_OBB();
    intersectors.intersector16 = BVH8Bezier1iIntersector16Hybrid_OBB();
    intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8OBBBezier1iMBIntersectors_OBB(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8OBBBezier1iMBIntersector1_OBB();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4  = BVH8OBBBezier1iMBIntersector4Hybrid_OBB();
    intersectors.intersector8  = BVH8OBBBezier1iMBIntersector8Hybrid_OBB();
    intersectors.intersector16 = BVH8OBBBezier1iMBIntersector16Hybrid_OBB();
    intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Line4iIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Line4iIntersector1();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4  = BVH8Line4iIntersector4();
    intersectors.intersector8  = BVH8Line4iIntersector8();
    intersectors.intersector16 = BVH8Line4iIntersector16();
    intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Line4iMBIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8Line4iMBIntersector1();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4  = BVH8Line4iMBIntersector4();
    intersectors.intersector8  = BVH8Line4iMBIntersector8();
    intersectors.intersector16 = BVH8Line4iMBIntersector16();
    intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    assert(ivariant == IntersectVariant::FAST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle4Intersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4_filter    = BVH8Triangle4Intersector4HybridMoeller();
    intersectors.intersector4_nofilter  = BVH8Triangle4Intersector4HybridMoellerNoFilter();
    intersectors.intersector8_filter    = BVH8Triangle4Intersector8HybridMoeller();
    intersectors.intersector8_nofilter  = BVH8Triangle4Intersector8HybridMoellerNoFilter();
    intersectors.intersector16_filter   = BVH8Triangle4Intersector16HybridMoeller();
    intersectors.intersector16_nofilter = BVH8Triangle4Intersector16HybridMoellerNoFilter();
    intersectors.intersectorN_filter    = BVH8Triangle4IntersectorStreamMoeller();
    intersectors.intersectorN_nofilter  = BVH8Triangle4IntersectorStreamMoellerNoFilter();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle4vIntersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    assert(ivariant == IntersectVariant::ROBUST);
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1    = BVH8Triangle4vIntersector1Pluecker();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4    = BVH8Triangle4vIntersector4HybridPluecker();
    intersectors.intersector8    = BVH8Triangle4vIntersector8HybridPluecker();
    intersectors.intersector16   = BVH8Triangle4vIntersector16HybridPluecker();
    intersectors.intersectorN    = BVH8Triangle4vIntersectorStreamPluecker();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle4iIntersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    switch (ivariant) {
    case IntersectVariant::FAST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Triangle4iIntersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Triangle4iIntersector4HybridMoeller();
      intersectors.intersector8  = BVH8Triangle4iIntersector8HybridMoeller();
      intersectors.intersector16 = BVH8Triangle4iIntersector16HybridMoeller();
      intersectors.intersectorN  = BVH8Triangle4iIntersectorStreamMoeller();
#endif
      return intersectors;
    }
    case IntersectVariant::ROBUST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Triangle4iIntersector1Pluecker();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Triangle4iIntersector4HybridPluecker();
      intersectors.intersector8  = BVH8Triangle4iIntersector8HybridPluecker();
      intersectors.intersector16 = BVH8Triangle4iIntersector16HybridPluecker();
      intersectors.intersectorN  = BVH8Triangle4iIntersectorStreamPluecker();
#endif
      return intersectors;
    }
    }
    return Accel::Intersectors();
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle4vMBIntersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    switch (ivariant) {
    case IntersectVariant::FAST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Triangle4vMBIntersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Triangle4vMBIntersector4HybridMoeller();
      intersectors.intersector8  = BVH8Triangle4vMBIntersector8HybridMoeller();
      intersectors.intersector16 = BVH8Triangle4vMBIntersector16HybridMoeller();
      intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
      return intersectors;
    }
    case IntersectVariant::ROBUST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Triangle4vMBIntersector1Pluecker();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Triangle4vMBIntersector4HybridPluecker();
      intersectors.intersector8  = BVH8Triangle4vMBIntersector8HybridPluecker();
      intersectors.intersector16 = BVH8Triangle4vMBIntersector16HybridPluecker();
      intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
      return intersectors;
    }
    }
    return Accel::Intersectors();
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle4iMBIntersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    switch (ivariant) {
    case IntersectVariant::FAST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Triangle4iMBIntersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Triangle4iMBIntersector4HybridMoeller();
      intersectors.intersector8  = BVH8Triangle4iMBIntersector8HybridMoeller();
      intersectors.intersector16 = BVH8Triangle4iMBIntersector16HybridMoeller();
      intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
      return intersectors;
    }
    case IntersectVariant::ROBUST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Triangle4iMBIntersector1Pluecker();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Triangle4iMBIntersector4HybridPluecker();
      intersectors.intersector8  = BVH8Triangle4iMBIntersector8HybridPluecker();
      intersectors.intersector16 = BVH8Triangle4iMBIntersector16HybridPluecker();
      intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
      return intersectors;
    }
    }
    return Accel::Intersectors();
  }

  Accel::Intersectors BVH8Factory::BVH8Quad4vIntersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    switch (ivariant) {
    case IntersectVariant::FAST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1           = BVH8Quad4vIntersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4_filter    = BVH8Quad4vIntersector4HybridMoeller();
      intersectors.intersector4_nofilter  = BVH8Quad4vIntersector4HybridMoellerNoFilter();
      intersectors.intersector8_filter    = BVH8Quad4vIntersector8HybridMoeller();
      intersectors.intersector8_nofilter  = BVH8Quad4vIntersector8HybridMoellerNoFilter();
      intersectors.intersector16_filter   = BVH8Quad4vIntersector16HybridMoeller();
      intersectors.intersector16_nofilter = BVH8Quad4vIntersector16HybridMoellerNoFilter();
      intersectors.intersectorN_filter    = BVH8Quad4vIntersectorStreamMoeller();
      intersectors.intersectorN_nofilter  = BVH8Quad4vIntersectorStreamMoellerNoFilter();
#endif
      return intersectors;
    }
    case IntersectVariant::ROBUST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Quad4vIntersector1Pluecker();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Quad4vIntersector4HybridPluecker();
      intersectors.intersector8  = BVH8Quad4vIntersector8HybridPluecker();
      intersectors.intersector16 = BVH8Quad4vIntersector16HybridPluecker();
      intersectors.intersectorN  = BVH8Quad4vIntersectorStreamPluecker();
#endif
      return intersectors;
    }
    }
    return Accel::Intersectors();
  }

  Accel::Intersectors BVH8Factory::BVH8Quad4iIntersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    switch (ivariant) {
    case IntersectVariant::FAST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Quad4iIntersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Quad4iIntersector4HybridMoeller();
      intersectors.intersector8  = BVH8Quad4iIntersector8HybridMoeller();
      intersectors.intersector16 = BVH8Quad4iIntersector16HybridMoeller();
      intersectors.intersectorN  = BVH8Quad4iIntersectorStreamMoeller();
#endif
      return intersectors;
    }
    case IntersectVariant::ROBUST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Quad4iIntersector1Pluecker();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Quad4iIntersector4HybridPluecker();
      intersectors.intersector8  = BVH8Quad4iIntersector8HybridPluecker();
      intersectors.intersector16 = BVH8Quad4iIntersector16HybridPluecker();
      intersectors.intersectorN  = BVH8Quad4iIntersectorStreamPluecker();
#endif
      return intersectors;
    }
    }
    return Accel::Intersectors();
  }

  Accel::Intersectors BVH8Factory::BVH8Quad4iMBIntersectors(BVH8* bvh, IntersectVariant ivariant)
  {
    switch (ivariant) {
    case IntersectVariant::FAST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Quad4iMBIntersector1Moeller();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Quad4iMBIntersector4HybridMoeller();
      intersectors.intersector8  = BVH8Quad4iMBIntersector8HybridMoeller();
      intersectors.intersector16 = BVH8Quad4iMBIntersector16HybridMoeller();
      intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
      return intersectors;
    }
    case IntersectVariant::ROBUST:
    {
      Accel::Intersectors intersectors;
      intersectors.ptr = bvh;
      intersectors.intersector1  = BVH8Quad4iMBIntersector1Pluecker();
#if defined (EMBREE_RAY_PACKETS)
      intersectors.intersector4  = BVH8Quad4iMBIntersector4HybridPluecker();
      intersectors.intersector8  = BVH8Quad4iMBIntersector8HybridPluecker();
      intersectors.intersector16 = BVH8Quad4iMBIntersector16HybridPluecker();
      intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
      return intersectors;
    }
    }
    return Accel::Intersectors();
  }

  Accel::Intersectors BVH8Factory::QBVH8Triangle4iIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH8Triangle4iIntersector1Pluecker();
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::QBVH8Triangle4Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH8Triangle4Intersector1Moeller();
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::QBVH8Quad4iIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = QBVH8Quad4iIntersector1Pluecker();
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8UserGeometryIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8VirtualIntersector1();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4  = BVH8VirtualIntersector4Chunk();
    intersectors.intersector8  = BVH8VirtualIntersector8Chunk();
    intersectors.intersector16 = BVH8VirtualIntersector16Chunk();
    intersectors.intersectorN  = BVH8VirtualIntersectorStream();
#endif
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8UserGeometryMBIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH8VirtualMBIntersector1();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4  = BVH8VirtualMBIntersector4Chunk();
    intersectors.intersector8  = BVH8VirtualMBIntersector8Chunk();
    intersectors.intersector16 = BVH8VirtualMBIntersector16Chunk();
    intersectors.intersectorN  = BVH8IntersectorStreamPacketFallback();
#endif
    return intersectors;
  }

  Accel* BVH8Factory::BVH8OBBBezier1v(Scene* scene)
  {
    BVH8* accel = new BVH8(Bezier1v::type,scene);
    Accel::Intersectors intersectors = BVH8Bezier1vIntersectors_OBB(accel);
    Builder* builder = BVH8Bezier1vBuilder_OBB_New(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8OBBBezier1i(Scene* scene)
  {
    BVH8* accel = new BVH8(Bezier1i::type,scene);
    Accel::Intersectors intersectors = BVH8Bezier1iIntersectors_OBB(accel);
    Builder* builder = BVH8Bezier1iBuilder_OBB_New(accel,scene,0);
    scene->needBezierVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8OBBBezier1iMB(Scene* scene)
  {
    BVH8* accel = new BVH8(Bezier1i::type,scene);
    Accel::Intersectors intersectors = BVH8OBBBezier1iMBIntersectors_OBB(accel);

    Builder* builder = nullptr;
    if      (scene->device->hair_builder_mb == "default"     ) builder = BVH8OBBBezier1iMBBuilder_OBB(accel,scene,0);
    else if (scene->device->hair_builder_mb == "sah"         ) builder = BVH8OBBBezier1iMBBuilder_OBB(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->hair_builder_mb+" for BVH8MBOBB<Bezier1iMB>");

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

  Accel* BVH8Factory::BVH8Triangle4(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel,ivariant);
    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default")  {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4); break;
      case BuildVariant::HIGH_QUALITY: builder = BVH8Triangle4SceneBuilderFastSpatialSAH(accel,scene,0); break;
      }
    }
    else if (scene->device->tri_builder == "sah"         )  builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_fast_spatial")  builder = BVH8Triangle4SceneBuilderFastSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit")     builder = BVH8Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (scene->device->tri_builder == "dynamic"     ) builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4);
    else if (scene->device->tri_builder == "morton"     ) builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4Morton);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4v(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Triangle4v::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4vIntersectors(accel,ivariant);
    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default")  {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Triangle4vSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4v); break;
      case BuildVariant::HIGH_QUALITY: builder = BVH8Triangle4vSceneBuilderFastSpatialSAH(accel,scene,0); break;
      }
    }
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4v>");
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4i(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Triangle4i::type,scene);
    Accel::Intersectors intersectors = BVH8Triangle4iIntersectors(accel,ivariant);

    Builder* builder = nullptr;
    if (scene->device->tri_builder == "default") {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Triangle4iSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : builder = BVH8BuilderTwoLevelTriangleMeshSAH(accel,scene,&createTriangleMeshTriangle4i); break;
      case BuildVariant::HIGH_QUALITY: assert(false); break; // FIXME: implement
      }
    }
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4i>");

    scene->needTriangleVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4iMB(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Triangle4i::type,scene);
    Accel::Intersectors intersectors = BVH8Triangle4iMBIntersectors(accel,ivariant);

    Builder* builder = nullptr;
    if (scene->device->tri_builder_mb == "default") { // FIXME: implement
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Triangle4iMBSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : assert(false); break; // FIXME: implement
      case BuildVariant::HIGH_QUALITY: assert(false); break;
      }
    }
    else if (scene->device->tri_builder_mb == "internal_time_splits")  builder = BVH8Triangle4iMBSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder_mb+" for BVH8<Triangle4iMB>");

    scene->needTriangleVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4vMB(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Triangle4vMB::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4vMBIntersectors(accel,ivariant);

    Builder* builder = nullptr;
    if (scene->device->tri_builder_mb == "default") {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Triangle4vMBSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : assert(false); break; // FIXME: implement
      case BuildVariant::HIGH_QUALITY: assert(false); break;
      }
    }
    else if (scene->device->tri_builder_mb == "internal_time_splits")  builder = BVH8Triangle4vMBSceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder_mb+" for BVH8<Triangle4vMB>");

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

  Accel* BVH8Factory::BVH8QuantizedTriangle4(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors = QBVH8Triangle4Intersectors(accel);
    Builder* builder = BVH8QuantizedTriangle4SceneBuilderSAH(accel,scene,0);
    scene->needTriangleVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Quad4v(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Quad4v::type,scene);
    Accel::Intersectors intersectors = BVH8Quad4vIntersectors(accel,ivariant);

    Builder* builder = nullptr;
    if (scene->device->quad_builder == "default") {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Quad4vSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : builder = BVH8BuilderTwoLevelQuadMeshSAH(accel,scene,&createQuadMeshQuad4v); break;
      case BuildVariant::HIGH_QUALITY: builder = BVH8Quad4vSceneBuilderFastSpatialSAH(accel,scene,0); break;
      }
    }
    else if (scene->device->quad_builder == "dynamic"      ) builder = BVH8BuilderTwoLevelQuadMeshSAH(accel,scene,&createQuadMeshQuad4v);
    else if (scene->device->quad_builder == "morton"       ) builder = BVH8BuilderTwoLevelQuadMeshSAH(accel,scene,&createQuadMeshQuad4vMorton);
    else if (scene->device->quad_builder == "sah_fast_spatial" ) builder = BVH8Quad4vSceneBuilderFastSpatialSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->quad_builder+" for BVH8<Quad4v>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Quad4i(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Quad4i::type,scene);
    Accel::Intersectors intersectors = BVH8Quad4iIntersectors(accel,ivariant);

    Builder* builder = nullptr;
    if (scene->device->quad_builder == "default") {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Quad4iSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : assert(false); break; // FIXME: implement
      case BuildVariant::HIGH_QUALITY: assert(false); break; // FIXME: implement
      }
    }
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->quad_builder+" for BVH8<Quad4i>");

    scene->needQuadVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Quad4iMB(Scene* scene, BuildVariant bvariant, IntersectVariant ivariant)
  {
    BVH8* accel = new BVH8(Quad4i::type,scene);
    Accel::Intersectors intersectors = BVH8Quad4iMBIntersectors(accel,ivariant);

    Builder* builder = nullptr;
    if (scene->device->quad_builder_mb == "default") {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8Quad4iMBSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : assert(false); break; // FIXME: implement
      case BuildVariant::HIGH_QUALITY: assert(false); break;
      }
    }
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->quad_builder_mb+" for BVH8<Quad4i>");

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

  Accel* BVH8Factory::BVH8UserGeometry(Scene* scene, BuildVariant bvariant)
  {
    BVH8* accel = new BVH8(Object::type,scene);
    Accel::Intersectors intersectors = BVH8UserGeometryIntersectors(accel);

    Builder* builder = nullptr;
    if (scene->device->object_builder == "default") {
      switch (bvariant) {
      case BuildVariant::STATIC      : builder = BVH8VirtualSceneBuilderSAH(accel,scene,0); break;
      case BuildVariant::DYNAMIC     : builder = BVH8BuilderTwoLevelVirtualSAH(accel,scene,&createAccelSetMesh); break;
      case BuildVariant::HIGH_QUALITY: assert(false); break;
      }
    }
    else if (scene->device->object_builder == "sah") builder = BVH8VirtualSceneBuilderSAH(accel,scene,0);
    else if (scene->device->object_builder == "dynamic") builder = BVH8BuilderTwoLevelVirtualSAH(accel,scene,&createAccelSetMesh);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->object_builder+" for BVH8<Object>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8UserGeometryMB(Scene* scene)
  {
    BVH8* accel = new BVH8(Object::type,scene);
    Accel::Intersectors intersectors = BVH8UserGeometryMBIntersectors(accel);
    Builder* builder = BVH8VirtualMBSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }
}

#endif
