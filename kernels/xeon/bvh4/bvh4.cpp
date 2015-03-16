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

#include "bvh4.h"
#include "bvh4_statistics.h"

#include "geometry/bezier1v.h"
#include "geometry/bezier1i.h"
#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4v_mb.h"
#include "geometry/triangle4i.h"
#include "geometry/subdivpatch1.h"
#include "geometry/subdivpatch1cached.h"
#include "geometry/virtual_accel.h"

#include "common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Bezier1vIntersector1);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Bezier1iIntersector1);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Bezier1vIntersector1_OBB);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Bezier1iIntersector1_OBB);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Bezier1iMBIntersector1_OBB);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Triangle1Intersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Triangle8Intersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Triangle1vIntersector1Pluecker);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Triangle4vIntersector1Pluecker);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Triangle4iIntersector1Pluecker);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Triangle1vMBIntersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Triangle4vMBIntersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Subdivpatch1Intersector1);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4Subdivpatch1CachedIntersector1);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4GridIntersector1);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4GridLazyIntersector1);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4VirtualIntersector1);

  DECLARE_SYMBOL(Accel::Intersector4,BVH4Bezier1vIntersector4Chunk);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Bezier1iIntersector4Chunk);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Bezier1vIntersector4Single_OBB);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Bezier1iIntersector4Single_OBB);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Bezier1iMBIntersector4Single_OBB);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle1Intersector4ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle4Intersector4ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle4Intersector4ChunkMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle8Intersector4ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle8Intersector4ChunkMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle8Intersector4HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle8Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle1vIntersector4ChunkPluecker);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle4vIntersector4ChunkPluecker);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle4vIntersector4HybridPluecker);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle4iIntersector4ChunkPluecker);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle1vMBIntersector4ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Triangle4vMBIntersector4ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Subdivpatch1Intersector4);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4Subdivpatch1CachedIntersector4);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4GridIntersector4);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4GridLazyIntersector4);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4VirtualIntersector4Chunk);
  
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Bezier1vIntersector8Chunk);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Bezier1iIntersector8Chunk);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Bezier1vIntersector8Single_OBB);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Bezier1iIntersector8Single_OBB);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Bezier1iMBIntersector8Single_OBB);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle1Intersector8ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle4Intersector8ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle4Intersector8ChunkMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle8Intersector8ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle8Intersector8ChunkMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle8Intersector8HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle8Intersector8HybridMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle1vIntersector8ChunkPluecker);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle4vIntersector8ChunkPluecker);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle4vIntersector8HybridPluecker);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle4iIntersector8ChunkPluecker);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle1vMBIntersector8ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Triangle4vMBIntersector8ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Subdivpatch1Intersector8);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4Subdivpatch1CachedIntersector8);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4GridIntersector8);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4GridLazyIntersector8);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4VirtualIntersector8Chunk);

  DECLARE_TOPLEVEL_BUILDER(BVH4BuilderTopLevelFast);

  DECLARE_SCENE_BUILDER(BVH4Bezier1vBuilder_OBB);
  DECLARE_SCENE_BUILDER(BVH4Bezier1iBuilder_OBB);
  DECLARE_SCENE_BUILDER(BVH4Bezier1iMBBuilder_OBB);
  DECLARE_SCENE_BUILDER(BVH4Triangle1Builder);
  DECLARE_SCENE_BUILDER(BVH4Triangle4Builder);
  DECLARE_SCENE_BUILDER(BVH4Triangle8Builder);
  DECLARE_SCENE_BUILDER(BVH4Triangle1vBuilder);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vBuilder);
  DECLARE_SCENE_BUILDER(BVH4Triangle4iBuilder);
  DECLARE_SCENE_BUILDER(BVH4Triangle1vMBBuilder);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vMBBuilder);

  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1MeshBuilder);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4MeshBuilder);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle8MeshBuilder);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1vMeshBuilder);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4vMeshBuilder);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4iMeshBuilder);

  DECLARE_SCENE_BUILDER(BVH4Bezier1vBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4Bezier1iBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4Triangle1BuilderFast);
  DECLARE_SCENE_BUILDER(BVH4Triangle4BuilderFast);
  DECLARE_SCENE_BUILDER(BVH4Triangle8BuilderFast);
  DECLARE_SCENE_BUILDER(BVH4Triangle1vBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4Triangle4iBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4SubdivPatch1BuilderFast);
  DECLARE_SCENE_BUILDER(BVH4SubdivPatch1CachedBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4SubdivGridBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4SubdivGridEagerBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4SubdivGridLazyBuilderFast);
  DECLARE_SCENE_BUILDER(BVH4UserGeometryBuilderFast);

  //DECLARE_SCENE_BUILDER(BVH4Triangle4BuilderFastSweep);

  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1MeshBuilderFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4MeshBuilderFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle8MeshBuilderFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1vMeshBuilderFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4vMeshBuilderFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4iMeshBuilderFast);
  DECLARE_USERGEOMETRY_BUILDER(BVH4UserGeometryMeshBuilderFast);

  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1MeshRefitFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4MeshRefitFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle8MeshRefitFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1vMeshRefitFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4vMeshRefitFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4iMeshRefitFast);

  DECLARE_SCENE_BUILDER(BVH4Triangle1BuilderMorton);
  DECLARE_SCENE_BUILDER(BVH4Triangle4BuilderMorton);
  DECLARE_SCENE_BUILDER(BVH4Triangle8BuilderMorton);
  DECLARE_SCENE_BUILDER(BVH4Triangle1vBuilderMorton);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vBuilderMorton);
  DECLARE_SCENE_BUILDER(BVH4Triangle4iBuilderMorton);

  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1MeshBuilderMorton);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4MeshBuilderMorton);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle8MeshBuilderMorton);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1vMeshBuilderMorton);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4vMeshBuilderMorton);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4iMeshBuilderMorton);

  /************************* new builders ********************/
  
  DECLARE_TOPLEVEL_BUILDER(BVH4BuilderTopLevelBinnedSAH);

  DECLARE_SCENE_BUILDER(BVH4Bezier1vBuilder_OBB_New);
  DECLARE_SCENE_BUILDER(BVH4Bezier1iBuilder_OBB_New);
  DECLARE_SCENE_BUILDER(BVH4Bezier1iMBBuilder_OBB_New);

  DECLARE_SCENE_BUILDER(BVH4Triangle1SceneBuilderBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle4SceneBuilderBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle8SceneBuilderBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle1vSceneBuilderBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vSceneBuilderBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle4iSceneBuilderBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vMBSceneBuilderBinnedSAH2);

  DECLARE_SCENE_BUILDER(BVH4Triangle1SceneBuilderSpatialBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle4SceneBuilderSpatialBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle8SceneBuilderSpatialBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle1vSceneBuilderSpatialBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vSceneBuilderSpatialBinnedSAH2);
  DECLARE_SCENE_BUILDER(BVH4Triangle4iSceneBuilderSpatialBinnedSAH2);

  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1MeshBuilderBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4MeshBuilderBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle8MeshBuilderBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1vMeshBuilderBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4vMeshBuilderBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4iMeshBuilderBinnedSAH2);

  DECLARE_SCENE_BUILDER(BVH4Bezier1vSceneBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4Bezier1iSceneBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4Triangle1SceneBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4Triangle4SceneBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4Triangle8SceneBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4Triangle1vSceneBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vSceneBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4Triangle4iSceneBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4SubdivPatch1BuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4SubdivPatch1CachedBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4SubdivGridBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4SubdivGridEagerBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4SubdivGridLazyBuilderBinnedSAH);
  DECLARE_SCENE_BUILDER(BVH4VirtualSceneBuilderBinnedSAH);

  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1MeshRefitBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4MeshRefitBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle8MeshRefitBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1vMeshRefitBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4vMeshRefitBinnedSAH2);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4iMeshRefitBinnedSAH2);

  DECLARE_SCENE_BUILDER(BVH4Triangle1SceneBuilderMortonGeneral);
  DECLARE_SCENE_BUILDER(BVH4Triangle4SceneBuilderMortonGeneral);
  DECLARE_SCENE_BUILDER(BVH4Triangle8SceneBuilderMortonGeneral);
  DECLARE_SCENE_BUILDER(BVH4Triangle1vSceneBuilderMortonGeneral);
  DECLARE_SCENE_BUILDER(BVH4Triangle4vSceneBuilderMortonGeneral);
  DECLARE_SCENE_BUILDER(BVH4Triangle4iSceneBuilderMortonGeneral);

  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1MeshBuilderMortonGeneral);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4MeshBuilderMortonGeneral);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle8MeshBuilderMortonGeneral);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle1vMeshBuilderMortonGeneral);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4vMeshBuilderMortonGeneral);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4Triangle4iMeshBuilderMortonGeneral);

  void BVH4Register () 
  {
    int features = getCPUFeatures();

    /* select builders */
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4BuilderTopLevelFast);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1vBuilder_OBB);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iBuilder_OBB);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iMBBuilder_OBB);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1Builder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4Builder);
    SELECT_SYMBOL_AVX(features,BVH4Triangle8Builder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vBuilder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vBuilder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iBuilder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vMBBuilder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMBBuilder);
    
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1MeshBuilder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4MeshBuilder);
    SELECT_SYMBOL_AVX(features,BVH4Triangle8MeshBuilder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vMeshBuilder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMeshBuilder);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iMeshBuilder);
  
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1vBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1BuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4BuilderFast);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8BuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4UserGeometryBuilderFast);

    //SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4BuilderFastSweep);
    
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1MeshBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4MeshBuilderFast);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8MeshBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vMeshBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMeshBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iMeshBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivPatch1BuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivPatch1CachedBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivGridBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivGridEagerBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivGridLazyBuilderFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4UserGeometryMeshBuilderFast);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1MeshRefitFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4MeshRefitFast);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8MeshRefitFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vMeshRefitFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMeshRefitFast);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iMeshRefitFast);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1BuilderMorton);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4BuilderMorton);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8BuilderMorton);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vBuilderMorton);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vBuilderMorton);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iBuilderMorton);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1MeshBuilderMorton);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4MeshBuilderMorton);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8MeshBuilderMorton);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vMeshBuilderMorton);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMeshBuilderMorton);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iMeshBuilderMorton);

    /* select new builders */
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4BuilderTopLevelBinnedSAH);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1vBuilder_OBB_New);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iBuilder_OBB_New);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iMBBuilder_OBB_New);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1SceneBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4SceneBuilderBinnedSAH2);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8SceneBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vSceneBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vSceneBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iSceneBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMBSceneBuilderBinnedSAH2);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1SceneBuilderSpatialBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4SceneBuilderSpatialBinnedSAH2);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8SceneBuilderSpatialBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vSceneBuilderSpatialBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vSceneBuilderSpatialBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iSceneBuilderSpatialBinnedSAH2);
    
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1MeshBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4MeshBuilderBinnedSAH2);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8MeshBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vMeshBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMeshBuilderBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iMeshBuilderBinnedSAH2);
  
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1vSceneBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Bezier1iSceneBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1SceneBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4SceneBuilderBinnedSAH);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8SceneBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vSceneBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vSceneBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iSceneBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4VirtualSceneBuilderBinnedSAH);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivPatch1BuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivPatch1CachedBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivGridBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivGridEagerBuilderBinnedSAH);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4SubdivGridLazyBuilderBinnedSAH);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1MeshRefitBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4MeshRefitBinnedSAH2);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8MeshRefitBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vMeshRefitBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMeshRefitBinnedSAH2);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iMeshRefitBinnedSAH2);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1SceneBuilderMortonGeneral);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4SceneBuilderMortonGeneral);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8SceneBuilderMortonGeneral);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vSceneBuilderMortonGeneral);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vSceneBuilderMortonGeneral);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iSceneBuilderMortonGeneral);

    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1MeshBuilderMortonGeneral);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4MeshBuilderMortonGeneral);
    SELECT_SYMBOL_AVX        (features,BVH4Triangle8MeshBuilderMortonGeneral);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle1vMeshBuilderMortonGeneral);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4vMeshBuilderMortonGeneral);
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4Triangle4iMeshBuilderMortonGeneral);
#endif

    /* select intersectors1 */
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1vIntersector1);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iIntersector1);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1vIntersector1_OBB);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iIntersector1_OBB);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iMBIntersector1_OBB);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle1Intersector1Moeller);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector1Moeller);
    SELECT_SYMBOL_AVX_AVX2              (features,BVH4Triangle8Intersector1Moeller);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX     (features,BVH4Triangle1vIntersector1Pluecker);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector1Pluecker);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX     (features,BVH4Triangle4iIntersector1Pluecker);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle1vMBIntersector1Moeller);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4vMBIntersector1Moeller);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Subdivpatch1Intersector1);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Subdivpatch1CachedIntersector1);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4GridIntersector1);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4GridLazyIntersector1);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4VirtualIntersector1);

    /* select intersectors4 */
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1vIntersector4Chunk);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iIntersector4Chunk);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1vIntersector4Single_OBB);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iIntersector4Single_OBB);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Bezier1iMBIntersector4Single_OBB);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle1Intersector4ChunkMoeller);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector4ChunkMoeller);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector4ChunkMoellerNoFilter);
    SELECT_SYMBOL_AVX_AVX2              (features,BVH4Triangle8Intersector4ChunkMoeller);
    SELECT_SYMBOL_DEFAULT2              (features,BVH4Triangle4Intersector4HybridMoeller,BVH4Triangle4Intersector4ChunkMoeller); // hybrid not supported below SSE4.2
    SELECT_SYMBOL_AVX_AVX2              (features,BVH4Triangle8Intersector4ChunkMoellerNoFilter);
    SELECT_SYMBOL_DEFAULT2              (features,BVH4Triangle4Intersector4HybridMoellerNoFilter,BVH4Triangle4Intersector4ChunkMoellerNoFilter); // hybrid not supported below SSE4.2
    SELECT_SYMBOL_SSE42_AVX_AVX2        (features,BVH4Triangle4Intersector4HybridMoeller);
    SELECT_SYMBOL_SSE42_AVX_AVX2        (features,BVH4Triangle4Intersector4HybridMoellerNoFilter);
    SELECT_SYMBOL_AVX_AVX2              (features,BVH4Triangle8Intersector4HybridMoeller);
    SELECT_SYMBOL_AVX_AVX2              (features,BVH4Triangle8Intersector4HybridMoellerNoFilter);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX     (features,BVH4Triangle1vIntersector4ChunkPluecker);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector4ChunkPluecker);
    SELECT_SYMBOL_DEFAULT2              (features,BVH4Triangle4vIntersector4HybridPluecker,BVH4Triangle4vIntersector4ChunkPluecker); // hybrid not supported below SSE4.2
    SELECT_SYMBOL_SSE42_AVX             (features,BVH4Triangle4vIntersector4HybridPluecker);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX     (features,BVH4Triangle4iIntersector4ChunkPluecker);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle1vMBIntersector4ChunkMoeller);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4vMBIntersector4ChunkMoeller);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Subdivpatch1Intersector4);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4Subdivpatch1CachedIntersector4);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4GridIntersector4);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2      (features,BVH4GridLazyIntersector4);
    SELECT_SYMBOL_DEFAULT_SSE41_AVX_AVX2(features,BVH4VirtualIntersector4Chunk);
   
    /* select intersectors8 */
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Bezier1vIntersector8Chunk);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Bezier1iIntersector8Chunk);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Bezier1vIntersector8Single_OBB);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Bezier1iIntersector8Single_OBB);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Bezier1iMBIntersector8Single_OBB);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle1Intersector8ChunkMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle4Intersector8ChunkMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle4Intersector8ChunkMoellerNoFilter);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle8Intersector8ChunkMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle8Intersector8ChunkMoellerNoFilter);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle4Intersector8HybridMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle4Intersector8HybridMoellerNoFilter);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle8Intersector8HybridMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle8Intersector8HybridMoellerNoFilter);
    SELECT_SYMBOL_AVX     (features,BVH4Triangle1vIntersector8ChunkPluecker);
    SELECT_SYMBOL_AVX     (features,BVH4Triangle4vIntersector8ChunkPluecker);
    SELECT_SYMBOL_AVX     (features,BVH4Triangle4vIntersector8HybridPluecker);
    SELECT_SYMBOL_AVX     (features,BVH4Triangle4iIntersector8ChunkPluecker);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle1vMBIntersector8ChunkMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Triangle4vMBIntersector8ChunkMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Subdivpatch1Intersector8);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4Subdivpatch1CachedIntersector8);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4GridIntersector8);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4GridLazyIntersector8);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4VirtualIntersector8Chunk);
  }

  BVH4::BVH4 (const PrimitiveType& primTy, Scene* scene, bool listMode)
    : primTy(primTy), scene(scene), listMode(listMode),
      root(emptyNode), numPrimitives(0), numVertices(0), data_mem(NULL), size_data_mem(0) {}

  BVH4::~BVH4 () {
    for (size_t i=0; i<objects.size(); i++) 
      delete objects[i];
  }

  void BVH4::init(size_t nodeSize, size_t numPrimitives, size_t numThreads)
  {
    /* allocate as much memory as likely needed and reserve conservative amounts of memory */
    size_t blockSize = LinearAllocatorPerThread::allocBlockSize;
    
    size_t numPrimBlocks = primTy.blocks(numPrimitives);
    size_t numAllocatedNodes      = min(size_t(0.6*numPrimBlocks),numPrimitives);
    size_t numAllocatedPrimitives = min(size_t(1.2*numPrimBlocks),numPrimitives);
#if defined(__X86_64__)
    size_t numReservedNodes = 2*numPrimitives;
    size_t numReservedPrimitives = 2*numPrimitives;
#else
    size_t numReservedNodes = 1.5*numAllocatedNodes;
    size_t numReservedPrimitives = 1.5*numAllocatedPrimitives;
#endif
    
    size_t bytesAllocated = numAllocatedNodes * nodeSize + numAllocatedPrimitives * primTy.bytes; 
    bytesAllocated = max(bytesAllocated,numPrimitives*sizeof(PrimRef)); // required also for parallel split stage in BVH4BuilderFast
    size_t bytesReserved  = numReservedNodes  * nodeSize + numReservedPrimitives  * primTy.bytes;
    if (numPrimitives) bytesReserved = (bytesReserved+blockSize-1)/blockSize*blockSize + numThreads*blockSize*2;
    alloc.init(bytesAllocated,bytesReserved);
    
    init();
  }

  void BVH4::init()
  {
    root = emptyNode;
    bounds = empty;
  }

  void BVH4::clear() 
  {
    set(BVH4::emptyNode,empty,0);
    alloc2.clear();
  }

  void BVH4::set (NodeRef root, const BBox3fa& bounds, size_t numPrimitives)
  {
    this->root = root;
    this->bounds = bounds;
    this->numPrimitives = numPrimitives;
  }

  void BVH4::printStatistics()
  {
    std::cout << BVH4Statistics(this).str();
    std::cout << "  "; alloc.print_statistics();
    std::cout << "  "; alloc2.print_statistics();
  }	

  void BVH4::clearBarrier(NodeRef& node)
  {
    if (node.isBarrier())
      node.clearBarrier();
    else if (!node.isLeaf()) {
      Node* n = node.node();
      for (size_t c=0; c<N; c++)
        clearBarrier(n->child(c));
    }
  }

  void BVH4::layoutLargeNodes(size_t N)
  {
    struct NodeArea 
    {
      __forceinline NodeArea() {}

      __forceinline NodeArea(NodeRef& node, const BBox3fa& bounds)
        : node(&node), A(node.isLeaf() ? float(neg_inf) : area(bounds)) {}

      __forceinline bool operator< (const NodeArea& other) const {
        return this->A < other.A;
      }

      NodeRef* node;
      float A;
    };
    std::vector<NodeArea> lst;
    lst.reserve(N);
    lst.push_back(NodeArea(root,empty));

    while (lst.size() < N)
    {
      std::pop_heap(lst.begin(), lst.end());
      NodeArea n = lst.back(); lst.pop_back();
      if (!n.node->isNode()) break;
      Node* node = n.node->node();
      for (size_t i=0; i<BVH4::N; i++) {
        if (node->child(i) == BVH4::emptyNode) continue;
        lst.push_back(NodeArea(node->child(i),node->bounds(i)));
        std::push_heap(lst.begin(), lst.end());
      }
    }

    for (size_t i=0; i<lst.size(); i++)
      lst[i].node->setBarrier();
      
    root = layoutLargeNodesRecursion(root);
  }
  
  BVH4::NodeRef BVH4::layoutLargeNodesRecursion(NodeRef& node)
  {
    if (node.isBarrier()) {
      node.clearBarrier();
      return node;
    }
    else if (node.isNode()) 
    {
      Node* oldnode = node.node();
      Node* newnode = (BVH4::Node*) alloc2.threadLocal2()->alloc0.malloc(sizeof(BVH4::Node)); // FIXME: optimize access to threadLocal2 
      *newnode = *oldnode;
      for (size_t c=0; c<BVH4::N; c++)
        newnode->child(c) = layoutLargeNodesRecursion(oldnode->child(c));
      return encodeNode(newnode);
    }
    else return node;
  }

  std::pair<BBox3fa,BBox3fa> BVH4::refit(Scene* scene, NodeRef node)
  {
    /*! merge bounds of triangles for both time steps */
    if (node.isLeaf()) 
    {
      size_t num; char* tri = node.leaf(num);
      if (node == BVH4::emptyNode) return std::pair<BBox3fa,BBox3fa>(empty,empty);
      return primTy.update2(tri,listMode ? -1 : num,scene);
    }
    /*! set and propagate merged bounds for both time steps */
    else
    {
      NodeMB* n = node.nodeMB();
      if (!n->hasBounds()) {
        for (size_t i=0; i<4; i++) {
          std::pair<BBox3fa,BBox3fa> bounds = refit(scene,n->child(i));
          n->set(i,bounds.first,bounds.second);
        }
      }
      BBox3fa bounds0 = merge(n->bounds0(0),n->bounds0(1),n->bounds0(2),n->bounds0(3));
      BBox3fa bounds1 = merge(n->bounds1(0),n->bounds1(1),n->bounds1(2),n->bounds1(3));
      return std::pair<BBox3fa,BBox3fa>(bounds0,bounds1);
    }
  }

  Accel::Intersectors BVH4Bezier1vIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Bezier1vIntersector1;
    intersectors.intersector4 = BVH4Bezier1vIntersector4Chunk;
    intersectors.intersector8 = BVH4Bezier1vIntersector8Chunk;
    intersectors.intersector16 = NULL;
    return intersectors;
  }
  
  Accel::Intersectors BVH4Bezier1iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Bezier1iIntersector1;
    intersectors.intersector4 = BVH4Bezier1iIntersector4Chunk;
    intersectors.intersector8 = BVH4Bezier1iIntersector8Chunk;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Bezier1vIntersectors_OBB(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Bezier1vIntersector1_OBB;
    intersectors.intersector4 = BVH4Bezier1vIntersector4Single_OBB;
    intersectors.intersector8 = BVH4Bezier1vIntersector8Single_OBB;
    intersectors.intersector16 = NULL;
    return intersectors;
  }
  
  Accel::Intersectors BVH4Bezier1iIntersectors_OBB(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Bezier1iIntersector1_OBB;
    intersectors.intersector4 = BVH4Bezier1iIntersector4Single_OBB;
    intersectors.intersector8 = BVH4Bezier1iIntersector8Single_OBB;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Bezier1iMBIntersectors_OBB(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Bezier1iMBIntersector1_OBB;
    intersectors.intersector4 = BVH4Bezier1iMBIntersector4Single_OBB;
    intersectors.intersector8 = BVH4Bezier1iMBIntersector8Single_OBB;
    intersectors.intersector16 = NULL;
    return intersectors;
  }
  
  Accel::Intersectors BVH4Triangle1Intersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle1Intersector1Moeller;
    intersectors.intersector4 = BVH4Triangle1Intersector4ChunkMoeller;
    intersectors.intersector8 = BVH4Triangle1Intersector8ChunkMoeller;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4IntersectorsChunk(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4Intersector1Moeller;
    intersectors.intersector4_filter   = BVH4Triangle4Intersector4ChunkMoeller;
    intersectors.intersector4_nofilter = BVH4Triangle4Intersector4ChunkMoellerNoFilter;
    intersectors.intersector8_filter   = BVH4Triangle4Intersector8ChunkMoeller;
    intersectors.intersector8_nofilter = BVH4Triangle4Intersector8ChunkMoellerNoFilter;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4IntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4Intersector1Moeller;
    intersectors.intersector4_filter   = BVH4Triangle4Intersector4HybridMoeller;
    intersectors.intersector4_nofilter = BVH4Triangle4Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter = BVH4Triangle4Intersector8HybridMoeller;
    intersectors.intersector8_nofilter = BVH4Triangle4Intersector8HybridMoellerNoFilter;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle8IntersectorsChunk(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle8Intersector1Moeller;
    intersectors.intersector4_filter   = BVH4Triangle8Intersector4ChunkMoeller;
    intersectors.intersector4_nofilter = BVH4Triangle8Intersector4ChunkMoellerNoFilter;
    intersectors.intersector8 = BVH4Triangle8Intersector8ChunkMoeller;
    intersectors.intersector8_filter   = BVH4Triangle8Intersector8ChunkMoeller;
    intersectors.intersector8_nofilter = BVH4Triangle8Intersector8ChunkMoellerNoFilter;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle8IntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle8Intersector1Moeller;
    intersectors.intersector4_filter   = BVH4Triangle8Intersector4HybridMoeller;
    intersectors.intersector4_nofilter = BVH4Triangle8Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter   = BVH4Triangle8Intersector8HybridMoeller;
    intersectors.intersector8_nofilter = BVH4Triangle8Intersector8HybridMoellerNoFilter;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle1vIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle1vIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle1vIntersector4ChunkPluecker;
    intersectors.intersector8 = BVH4Triangle1vIntersector8ChunkPluecker;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle1vMBIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle1vMBIntersector1Moeller;
    intersectors.intersector4 = BVH4Triangle1vMBIntersector4ChunkMoeller;
    intersectors.intersector8 = BVH4Triangle1vMBIntersector8ChunkMoeller;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4vMBIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4vMBIntersector1Moeller;
    intersectors.intersector4 = BVH4Triangle4vMBIntersector4ChunkMoeller;
    intersectors.intersector8 = BVH4Triangle4vMBIntersector8ChunkMoeller;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4vIntersectorsChunk(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4vIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle4vIntersector4ChunkPluecker;
    intersectors.intersector8 = BVH4Triangle4vIntersector8HybridPluecker;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4vIntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4vIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle4vIntersector4HybridPluecker;
    intersectors.intersector8 = BVH4Triangle4vIntersector8HybridPluecker;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4iIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle4iIntersector4ChunkPluecker;
    intersectors.intersector8 = BVH4Triangle4iIntersector8ChunkPluecker;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel* BVH4::BVH4Bezier1v(Scene* scene)
  { 
    BVH4* accel = new BVH4(Bezier1vType::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Bezier1vIntersectors(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Bezier1vSceneBuilderBinnedSAH(accel,scene,LeafMode);
#else
    Builder* builder = BVH4Bezier1vBuilderFast(accel,scene,LeafMode);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Bezier1i(Scene* scene)
  { 
    BVH4* accel = new BVH4(SceneBezier1i::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Bezier1iIntersectors(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Bezier1iSceneBuilderBinnedSAH(accel,scene,LeafMode);
#else
    Builder* builder = BVH4Bezier1iBuilderFast(accel,scene,LeafMode);
#endif
    scene->needVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4OBBBezier1v(Scene* scene, bool highQuality)
  { 
    BVH4* accel = new BVH4(Bezier1vType::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Bezier1vIntersectors_OBB(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Bezier1vBuilder_OBB_New(accel,scene,MODE_HIGH_QUALITY); // FIXME: enable high quality mode
#else
    Builder* builder = BVH4Bezier1vBuilder_OBB(accel,scene,LeafMode | (highQuality ? MODE_HIGH_QUALITY : 0));
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4OBBBezier1i(Scene* scene, bool highQuality)
  { 
    BVH4* accel = new BVH4(SceneBezier1i::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Bezier1iIntersectors_OBB(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Bezier1iBuilder_OBB_New(accel,scene,MODE_HIGH_QUALITY); // FIXME: enable high quality mode
#else
    Builder* builder = BVH4Bezier1iBuilder_OBB(accel,scene,LeafMode | (highQuality ? MODE_HIGH_QUALITY : 0));
#endif
    scene->needVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

   Accel* BVH4::BVH4OBBBezier1iMB(Scene* scene, bool highQuality)
  { 
    scene->needVertices = true;
    BVH4* accel = new BVH4(Bezier1iMBType::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Bezier1iMBIntersectors_OBB(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Bezier1iMBBuilder_OBB_New(accel,scene,MODE_HIGH_QUALITY); // FIXME: support high quality mode
#else
    Builder* builder = BVH4Bezier1iMBBuilder_OBB(accel,scene,LeafMode | (highQuality ? MODE_HIGH_QUALITY : 0));
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle1(Scene* scene)
  { 
    BVH4* accel = new BVH4(Triangle1Type::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    
    Builder* builder = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    if      (g_tri_builder == "default"             ) builder = BVH4Triangle1SceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah"          ) builder = BVH4Triangle1SceneBuilderBinnedSAH(accel,scene,0);
    else if (g_tri_builder == "binned_sah2"         ) builder = BVH4Triangle1SceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah_presplit" ) builder = BVH4Triangle1SceneBuilderBinnedSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "binned_sah2_presplit") builder = BVH4Triangle1SceneBuilderBinnedSAH2(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle1SceneBuilderMortonGeneral(accel,scene,0);
#else
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle1Builder(accel,scene,LeafMode);
    else if (g_tri_builder == "spatialsplit") builder = BVH4Triangle1Builder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
    else if (g_tri_builder == "objectsplit" ) builder = BVH4Triangle1Builder(accel,scene,LeafMode);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle1BuilderMorton(accel,scene,LeafMode);
    else if (g_tri_builder == "fast"        ) builder = BVH4Triangle1BuilderFast(accel,scene,LeafMode);
#endif
    else THROW_RUNTIME_ERROR("unknown builder "+g_tri_builder+" for BVH4<Triangle1>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4(Scene* scene)
  { 
    BVH4* accel = new BVH4(Triangle4Type::type,scene,LeafMode);

    Accel::Intersectors intersectors;
    if      (g_tri_traverser == "default") intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    else if (g_tri_traverser == "chunk"  ) intersectors = BVH4Triangle4IntersectorsChunk(accel);
    else if (g_tri_traverser == "hybrid" ) intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    else THROW_RUNTIME_ERROR("unknown traverser "+g_tri_traverser+" for BVH4<Triangle4>");
   
    Builder* builder = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle4SceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah"  ) builder = BVH4Triangle4SceneBuilderBinnedSAH(accel,scene,0);
    else if (g_tri_builder == "binned_sah2" ) builder = BVH4Triangle4SceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah2_spatial" ) builder = BVH4Triangle4SceneBuilderSpatialBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah_presplit" ) builder = BVH4Triangle4SceneBuilderBinnedSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "binned_sah2_presplit") builder = BVH4Triangle4SceneBuilderBinnedSAH2(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle4SceneBuilderMortonGeneral(accel,scene,0);
#else
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle4Builder(accel,scene,LeafMode);
    else if (g_tri_builder == "spatialsplit") builder = BVH4Triangle4Builder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
    else if (g_tri_builder == "objectsplit" ) builder = BVH4Triangle4Builder(accel,scene,LeafMode);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle4BuilderMorton(accel,scene,LeafMode); 
    else if (g_tri_builder == "fast"        ) builder = BVH4Triangle4BuilderFast(accel,scene,LeafMode);
    //else if (g_tri_builder == "sweep"       ) builder = BVH4Triangle4BuilderFastSweep(accel,scene,LeafMode);
#endif
    else THROW_RUNTIME_ERROR("unknown builder "+g_tri_builder+" for BVH4<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

#if defined (__TARGET_AVX__)
  Accel* BVH4::BVH4Triangle8(Scene* scene)
  { 
    BVH4* accel = new BVH4(Triangle8Type::type,scene,LeafMode);

    Accel::Intersectors intersectors;
    if      (g_tri_traverser == "default") intersectors = BVH4Triangle8IntersectorsHybrid(accel);
    else if (g_tri_traverser == "chunk"  ) intersectors = BVH4Triangle8IntersectorsChunk(accel);
    else if (g_tri_traverser == "hybrid" ) intersectors = BVH4Triangle8IntersectorsHybrid(accel);
    else THROW_RUNTIME_ERROR("unknown traverser "+g_tri_traverser+" for BVH4<Triangle8>");
   
    Builder* builder = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle8SceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah"  ) builder = BVH4Triangle8SceneBuilderBinnedSAH(accel,scene,0);
    else if (g_tri_builder == "binned_sah2" ) builder = BVH4Triangle8SceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah2_spatial" ) builder = BVH4Triangle8SceneBuilderSpatialBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah_presplit"  ) builder = BVH4Triangle8SceneBuilderBinnedSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "binned_sah2_presplit" ) builder = BVH4Triangle8SceneBuilderBinnedSAH2(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle8SceneBuilderMortonGeneral(accel,scene,0);
#else
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle8Builder(accel,scene,LeafMode);
    else if (g_tri_builder == "spatialsplit") builder = BVH4Triangle8Builder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
    else if (g_tri_builder == "objectsplit" ) builder = BVH4Triangle8Builder(accel,scene,LeafMode);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle8BuilderMorton(accel,scene,LeafMode);
    else if (g_tri_builder == "fast"        ) builder = BVH4Triangle8BuilderFast(accel,scene,LeafMode);
#endif
    else THROW_RUNTIME_ERROR("unknown builder "+g_tri_builder+" for BVH4<Triangle8>");

    return new AccelInstance(accel,builder,intersectors);
  }
#endif

  Accel* BVH4::BVH4Triangle1v(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle1vType::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle1vIntersectors(accel);

    Builder* builder = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle1vSceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah"  ) builder = BVH4Triangle1vSceneBuilderBinnedSAH(accel,scene,0);
    else if (g_tri_builder == "binned_sah2" ) builder = BVH4Triangle1vSceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah2_spatial" ) builder = BVH4Triangle1vSceneBuilderSpatialBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah_presplit"  ) builder = BVH4Triangle1vSceneBuilderBinnedSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "binned_sah2_presplit" ) builder = BVH4Triangle1vSceneBuilderBinnedSAH2(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle1vSceneBuilderMortonGeneral(accel,scene,0);
#else
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle1vBuilder(accel,scene,LeafMode);
    else if (g_tri_builder == "spatialsplit") builder = BVH4Triangle1vBuilder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
    else if (g_tri_builder == "objectsplit" ) builder = BVH4Triangle1vBuilder(accel,scene,LeafMode);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle1vBuilderMorton(accel,scene,LeafMode);
    else if (g_tri_builder == "fast"        ) builder = BVH4Triangle1vBuilderFast(accel,scene,LeafMode);
#endif
    else THROW_RUNTIME_ERROR("unknown builder "+g_tri_builder+" for BVH4<Triangle1v>");
        
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4vMB(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4vMB::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle4vMBIntersectors(accel);
    Builder* builder = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    if       (g_tri_builder_mb == "default"    ) builder = BVH4Triangle4vMBSceneBuilderBinnedSAH2(accel,scene,0);
    else  if (g_tri_builder_mb == "binned_sah2") builder = BVH4Triangle4vMBSceneBuilderBinnedSAH2(accel,scene,0);
    else THROW_RUNTIME_ERROR("unknown builder "+g_tri_builder_mb+" for BVH4<Triangle4vMB>");
#else
    if       (g_tri_builder_mb == "default"    ) builder = BVH4Triangle4vMBBuilder(accel,scene,LeafMode);
    else THROW_RUNTIME_ERROR("unknown builder "+g_tri_builder_mb+" for BVH4<Triangle4vMB>");
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4v(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4vType::type,scene,LeafMode);

    Accel::Intersectors intersectors;
    if      (g_tri_traverser == "default") intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    else if (g_tri_traverser == "chunk"  ) intersectors = BVH4Triangle4vIntersectorsChunk(accel);
    else if (g_tri_traverser == "hybrid" ) intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    else THROW_RUNTIME_ERROR("unknown traverser "+g_tri_traverser+" for BVH4<Triangle4>");

    Builder* builder = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle4vSceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah"  ) builder = BVH4Triangle4vSceneBuilderBinnedSAH(accel,scene,0);
    else if (g_tri_builder == "binned_sah2" ) builder = BVH4Triangle4vSceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah2_spatial" ) builder = BVH4Triangle4vSceneBuilderSpatialBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah_presplit"  ) builder = BVH4Triangle4vSceneBuilderBinnedSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "binned_sah2_presplit" ) builder = BVH4Triangle4vSceneBuilderBinnedSAH2(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle4vSceneBuilderMortonGeneral(accel,scene,0);
#else
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle4vBuilder(accel,scene,LeafMode);
    else if (g_tri_builder == "spatialsplit") builder = BVH4Triangle4vBuilder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
    else if (g_tri_builder == "objectsplit" ) builder = BVH4Triangle4vBuilder(accel,scene,LeafMode);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle4vBuilderMorton(accel,scene,LeafMode);
    else if (g_tri_builder == "fast"        ) builder = BVH4Triangle4vBuilderFast(accel,scene,LeafMode);
#endif
    else THROW_RUNTIME_ERROR("unknown builder "+g_tri_builder+" for BVH4<Triangle4v>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4i(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4iType::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle4iIntersectors(accel);

    Builder* builder = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle4iSceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah"  ) builder = BVH4Triangle4iSceneBuilderBinnedSAH(accel,scene,0);
    else if (g_tri_builder == "binned_sah2" ) builder = BVH4Triangle4iSceneBuilderBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah2_spatial" ) builder = BVH4Triangle4iSceneBuilderSpatialBinnedSAH2(accel,scene,0);
    else if (g_tri_builder == "binned_sah_presplit"  ) builder = BVH4Triangle4iSceneBuilderBinnedSAH(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "binned_sah2_presplit" ) builder = BVH4Triangle4iSceneBuilderBinnedSAH2(accel,scene,MODE_HIGH_QUALITY);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle4iSceneBuilderMortonGeneral(accel,scene,0);
#else
    if      (g_tri_builder == "default"     ) builder = BVH4Triangle4iBuilder(accel,scene,LeafMode);
    else if (g_tri_builder == "spatialsplit") builder = BVH4Triangle4iBuilder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
    else if (g_tri_builder == "objectsplit" ) builder = BVH4Triangle4iBuilder(accel,scene,LeafMode);
    else if (g_tri_builder == "fast"        ) builder = BVH4Triangle4iBuilderFast(accel,scene,LeafMode);
    else if (g_tri_builder == "morton"      ) builder = BVH4Triangle4iBuilderMorton(accel,scene,LeafMode);
#endif
    else THROW_RUNTIME_ERROR("unknown builder "+g_tri_builder+" for BVH4<Triangle4i>");

    scene->needVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  void createTriangleMeshTriangle1Morton(TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) THROW_RUNTIME_ERROR("internal error");
    accel = new BVH4(TriangleMeshTriangle1::type,mesh->parent,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    builder = BVH4Triangle1MeshBuilderMortonGeneral(accel,mesh,LeafMode);
#else
    builder = BVH4Triangle1MeshBuilderMorton(accel,mesh,LeafMode);
#endif
  } 

  void createTriangleMeshTriangle1(TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) THROW_RUNTIME_ERROR("internal error");
    accel = new BVH4(TriangleMeshTriangle1::type,mesh->parent,LeafMode);
    switch (mesh->flags) {
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle1MeshBuilderBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle1MeshRefitBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle1MeshBuilderMortonGeneral(accel,mesh,LeafMode); break;
#else
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle1MeshBuilderFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle1MeshRefitFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle1MeshBuilderMorton(accel,mesh,LeafMode); break;
#endif
    default: THROW_RUNTIME_ERROR("internal error"); 
    }
  } 

  void createTriangleMeshTriangle4(TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) THROW_RUNTIME_ERROR("internal error");
    accel = new BVH4(TriangleMeshTriangle4::type,mesh->parent,LeafMode);
    switch (mesh->flags) {
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle4MeshBuilderBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle4MeshRefitBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle4MeshBuilderMortonGeneral(accel,mesh,LeafMode); break;
    //default:                      builder = BVH4Triangle4MeshBuilderMortonGeneral(accel,mesh,LeafMode); break;
#else
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle4MeshBuilderFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle4MeshRefitFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle4MeshBuilderMorton(accel,mesh,LeafMode); break;
#endif
    default: THROW_RUNTIME_ERROR("internal error"); 
    }
  } 

  void createTriangleMeshTriangle1v(TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) THROW_RUNTIME_ERROR("internal error");
    accel = new BVH4(TriangleMeshTriangle1v::type,mesh->parent,LeafMode);
    switch (mesh->flags) {
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle1vMeshBuilderBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle1vMeshRefitBinnedSAH2  (accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle1vMeshBuilderMortonGeneral(accel,mesh,LeafMode); break;
#else
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle1vMeshBuilderFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle1vMeshRefitFast  (accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle1vMeshBuilderMorton(accel,mesh,LeafMode); break;
#endif
    default: THROW_RUNTIME_ERROR("internal error"); 
    }
  } 

  void createTriangleMeshTriangle4v(TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) THROW_RUNTIME_ERROR("internal error");
    accel = new BVH4(TriangleMeshTriangle4v::type,mesh->parent,LeafMode);
    switch (mesh->flags) {
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle4vMeshBuilderBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle4vMeshRefitBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle4vMeshBuilderMortonGeneral(accel,mesh,LeafMode); break;
#else
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle4vMeshBuilderFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle4vMeshRefitFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle4vMeshBuilderMorton(accel,mesh,LeafMode); break;
#endif
    default: THROW_RUNTIME_ERROR("internal error"); 
    }
  } 

  void createTriangleMeshTriangle4i(TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) THROW_RUNTIME_ERROR("internal error");
    accel = new BVH4(TriangleMeshTriangle4i::type,mesh->parent,LeafMode);
    switch (mesh->flags) {
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle4iMeshBuilderBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle4iMeshRefitBinnedSAH2(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle4iMeshBuilderMortonGeneral(accel,mesh,LeafMode); break;
#else
    case RTC_GEOMETRY_STATIC:     builder = BVH4Triangle4iMeshBuilderFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4Triangle4iMeshRefitFast(accel,mesh,LeafMode); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4Triangle4iMeshBuilderMorton(accel,mesh,LeafMode); break;
#endif
    default: THROW_RUNTIME_ERROR("internal error"); 
    }
  } 

  Accel* BVH4::BVH4BVH4Triangle1Morton(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4BuilderTopLevelBinnedSAH(accel,scene,&createTriangleMeshTriangle1Morton);
#else
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle1Morton);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle1ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4BuilderTopLevelBinnedSAH(accel,scene,&createTriangleMeshTriangle1);
#else
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle1);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle4ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4BuilderTopLevelBinnedSAH(accel,scene,&createTriangleMeshTriangle4);
#else
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle4);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle1vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1v::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle1vIntersectors(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
  Builder* builder = BVH4BuilderTopLevelBinnedSAH(accel,scene,&createTriangleMeshTriangle1v);
#else
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle1v);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle4vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4v::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4BuilderTopLevelBinnedSAH(accel,scene,&createTriangleMeshTriangle4v);
#else
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle4v);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle4iObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4i::type,scene,LeafMode);
    Accel::Intersectors intersectors = BVH4Triangle4iIntersectors(accel);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
     Builder* builder = BVH4BuilderTopLevelBinnedSAH(accel,scene,&createTriangleMeshTriangle4i);
#else
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle4i);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle1SpatialSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle1Type::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle1SceneBuilderSpatialBinnedSAH2(accel,scene,0); 
#else
    Builder* builder = BVH4Triangle1Builder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
#endif
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }
  
  Accel* BVH4::BVH4Triangle4SpatialSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4Type::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle4SceneBuilderSpatialBinnedSAH2(accel,scene,0); 
#else
    Builder* builder = BVH4Triangle4Builder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
#endif
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

#if defined (__TARGET_AVX__)
  Accel* BVH4::BVH4Triangle8SpatialSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle8Type::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle8SceneBuilderSpatialBinnedSAH2(accel,scene,0);
#else
    Builder* builder = BVH4Triangle8Builder(accel,scene,LeafMode | MODE_HIGH_QUALITY);
#endif
    Accel::Intersectors intersectors = BVH4Triangle8IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }
#endif

  Accel* BVH4::BVH4Triangle1ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle1Type::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle1SceneBuilderBinnedSAH2(accel,scene,LeafMode);
#else
    Builder* builder = BVH4Triangle1Builder(accel,scene,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }
  
  Accel* BVH4::BVH4Triangle4ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4Type::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle4SceneBuilderBinnedSAH2(accel,scene,LeafMode);
#else
    Builder* builder = BVH4Triangle4Builder(accel,scene,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

#if defined (__TARGET_AVX__)
  Accel* BVH4::BVH4Triangle8ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle8Type::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle8SceneBuilderBinnedSAH2(accel,scene,LeafMode);
#else
    Builder* builder = BVH4Triangle8Builder(accel,scene,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle8IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }
#endif

  Accel* BVH4::BVH4Triangle1vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle1vType::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle1vSceneBuilderBinnedSAH2(accel,scene,LeafMode);
#else
    Builder* builder = BVH4Triangle1vBuilder(accel,scene,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle1vIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4vType::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle4vSceneBuilderBinnedSAH2(accel,scene,LeafMode);
#else
    Builder* builder = BVH4Triangle4vBuilder(accel,scene,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4iObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4iType::type,scene,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle4iSceneBuilderBinnedSAH2(accel,scene,LeafMode);
#else
    Builder* builder = BVH4Triangle4iBuilder(accel,scene,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle4iIntersectors(accel);
    scene->needVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4SubdivPatch1(Scene* scene)
  {
    BVH4* accel = new BVH4(SubdivPatch1::type,scene,LeafMode);
    Accel::Intersectors intersectors;
    intersectors.ptr = accel; 
    intersectors.intersector1 = BVH4Subdivpatch1Intersector1;
    intersectors.intersector4 = BVH4Subdivpatch1Intersector4;
    intersectors.intersector8 = BVH4Subdivpatch1Intersector8;
    intersectors.intersector16 = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4SubdivPatch1BuilderBinnedSAH(accel,scene,LeafMode);
#else
    Builder* builder = BVH4SubdivPatch1BuilderFast(accel,scene,LeafMode);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4SubdivPatch1Cached(Scene* scene)
  {
    BVH4* accel = new BVH4(SubdivPatch1Cached::type,scene,LeafMode);
    Accel::Intersectors intersectors;
    intersectors.ptr = accel; 
    intersectors.intersector1 = BVH4Subdivpatch1CachedIntersector1;
    intersectors.intersector4 = BVH4Subdivpatch1CachedIntersector4;
    intersectors.intersector8 = BVH4Subdivpatch1CachedIntersector8;
    intersectors.intersector16 = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4SubdivPatch1CachedBuilderBinnedSAH(accel,scene,LeafMode);
#else
    Builder* builder = BVH4SubdivPatch1CachedBuilderFast(accel,scene,LeafMode);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4SubdivGrid(Scene* scene)
  {
    BVH4* accel = new BVH4(PrimitiveType2::type,scene,LeafMode); // FIXME: type
    Accel::Intersectors intersectors;
    intersectors.ptr = accel; 
    intersectors.intersector1 = BVH4GridIntersector1;
    intersectors.intersector4 = BVH4GridIntersector4;
    intersectors.intersector8 = BVH4GridIntersector8;
    intersectors.intersector16 = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4SubdivGridBuilderBinnedSAH(accel,scene,LeafMode);
#else
    Builder* builder = BVH4SubdivGridBuilderFast(accel,scene,LeafMode);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4SubdivGridEager(Scene* scene)
  {
    BVH4* accel = new BVH4(PrimitiveType2::type,scene,LeafMode); // FIXME: type
    Accel::Intersectors intersectors;
    intersectors.ptr = accel; 
    intersectors.intersector1 = BVH4GridIntersector1;
    intersectors.intersector4 = BVH4GridIntersector4;
    intersectors.intersector8 = BVH4GridIntersector8;
    intersectors.intersector16 = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4SubdivGridEagerBuilderBinnedSAH(accel,scene,LeafMode);
#else
    Builder* builder = BVH4SubdivGridEagerBuilderFast(accel,scene,LeafMode);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4SubdivGridLazy(Scene* scene)
  {
    BVH4* accel = new BVH4(PrimitiveType2::type,scene,LeafMode); // FIXME: type
    Accel::Intersectors intersectors;
    intersectors.ptr = accel; 
    intersectors.intersector1 = BVH4GridLazyIntersector1;
    intersectors.intersector4 = BVH4GridLazyIntersector4;
    intersectors.intersector8 = BVH4GridLazyIntersector8;
    intersectors.intersector16 = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4SubdivGridLazyBuilderBinnedSAH(accel,scene,LeafMode);
#else
    Builder* builder = BVH4SubdivGridLazyBuilderFast(accel,scene,LeafMode);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4UserGeometry(Scene* scene)
  {
    BVH4* accel = new BVH4(VirtualAccelObjectType::type,scene,LeafMode);
    Accel::Intersectors intersectors;
    intersectors.ptr = accel; 
    intersectors.intersector1 = BVH4VirtualIntersector1;
    intersectors.intersector4 = BVH4VirtualIntersector4Chunk;
    intersectors.intersector8 = BVH4VirtualIntersector8Chunk;
    intersectors.intersector16 = NULL;
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4VirtualSceneBuilderBinnedSAH(accel,scene,LeafMode);
#else
    Builder* builder = BVH4UserGeometryBuilderFast(accel,scene,LeafMode);
#endif
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle1ObjectSplit(TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1::type,mesh->parent,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle1MeshBuilderBinnedSAH2(accel,mesh,LeafMode);
#else
    Builder* builder = BVH4Triangle1MeshBuilderFast(accel,mesh,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4ObjectSplit(TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4::type,mesh->parent,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle4MeshBuilderBinnedSAH2(accel,mesh,LeafMode);
#else
    Builder* builder = BVH4Triangle4MeshBuilderFast(accel,mesh,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle1vObjectSplit(TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1v::type,mesh->parent,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle1vMeshBuilderBinnedSAH2(accel,mesh,LeafMode);
#else
    Builder* builder = BVH4Triangle1vMeshBuilderFast(accel,mesh,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle1vIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4vObjectSplit(TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4v::type,mesh->parent,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle4vMeshBuilderBinnedSAH2(accel,mesh,LeafMode);
#else
    Builder* builder = BVH4Triangle4vMeshBuilderFast(accel,mesh,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4Refit(TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4::type,mesh->parent,LeafMode);
#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)
    Builder* builder = BVH4Triangle4MeshRefitBinnedSAH2(accel,mesh,LeafMode);
#else
    Builder* builder = BVH4Triangle4MeshRefitFast(accel,mesh,LeafMode);
#endif
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }
}

