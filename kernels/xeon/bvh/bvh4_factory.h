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
  /*! BVH4 instantiations */
  class BVH4Factory
  {
  public:
    BVH4Factory(int features);

  public:
    Accel* BVH4Triangle4vMB(Scene* scene);

    Accel* BVH4Bezier1v(Scene* scene);
    Accel* BVH4Bezier1i(Scene* scene);
    Accel* BVH4Line4i(Scene* scene);
    Accel* BVH4Line4iMB(Scene* scene);

    Accel* BVH4OBBBezier1v(Scene* scene, bool highQuality);
    Accel* BVH4OBBBezier1i(Scene* scene, bool highQuality);
    Accel* BVH4OBBBezier1iMB(Scene* scene, bool highQuality);

    Accel* BVH4Triangle4(Scene* scene);
    Accel* BVH4Triangle8(Scene* scene);
    Accel* BVH4Triangle4v(Scene* scene);
    Accel* BVH4Triangle4i(Scene* scene);
    Accel* BVH4SubdivPatch1Cached(Scene* scene);
    Accel* BVH4SubdivGridEager(Scene* scene);
    Accel* BVH4UserGeometry(Scene* scene);
    Accel* BVH4UserGeometryMB(Scene* scene);
    Accel* BVH4InstancedBVH4Triangle4ObjectSplit(Scene* scene);
    Accel* BVH4Quad4v(Scene* scene);
    Accel* BVH4Quad4i(Scene* scene);
    Accel* BVH4Quad4iMB(Scene* scene);


    Accel* BVH4Line4iTwolevel(Scene* scene);
    Accel* BVH4Triangle4Twolevel(Scene* scene);
    Accel* BVH4Triangle8Twolevel(Scene* scene);
    Accel* BVH4Triangle4vTwolevel(Scene* scene);
    Accel* BVH4Triangle4iTwolevel(Scene* scene);

    Accel* BVH4Triangle4SpatialSplit(Scene* scene);
    Accel* BVH4Triangle8SpatialSplit(Scene* scene);
    Accel* BVH4Triangle4ObjectSplit(Scene* scene);
    Accel* BVH4Triangle8ObjectSplit(Scene* scene);
    Accel* BVH4Triangle4vObjectSplit(Scene* scene);
    Accel* BVH4Triangle4iObjectSplit(Scene* scene);

    Accel* BVH4Triangle4ObjectSplit(TriangleMesh* mesh);
    Accel* BVH4Triangle4vObjectSplit(TriangleMesh* mesh);
    Accel* BVH4Triangle4Refit(TriangleMesh* mesh);

  private:
    Accel::Intersectors BVH4Line4iIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Line4iMBIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1vIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1iIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1vIntersectors_OBB(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1iIntersectors_OBB(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1iMBIntersectors_OBB(BVH4* bvh);
    Accel::Intersectors BVH4Triangle4IntersectorsHybrid(BVH4* bvh);
    Accel::Intersectors BVH4Triangle4IntersectorsInstancing(BVH4* bvh);
    Accel::Intersectors BVH4Triangle8IntersectorsHybrid(BVH4* bvh);
    Accel::Intersectors BVH4Triangle4vIntersectorsHybrid(BVH4* bvh);
    Accel::Intersectors BVH4Triangle4iIntersectorsHybrid(BVH4* bvh);
    Accel::Intersectors BVH4Triangle4vMBIntersectorsHybrid(BVH4* bvh);
    Accel::Intersectors BVH4Quad4vIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Quad4iIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Quad4iMBIntersectors(BVH4* bvh);

    static void createLineSegmentsLine4i(LineSegments* mesh, AccelData*& accel, Builder*& builder);

    static void createTriangleMeshTriangle4Morton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
#if defined (__TARGET_AVX__)
    static void createTriangleMeshTriangle8Morton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
#endif
    static void createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);

    static void createTriangleMeshTriangle4(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
#if defined (__TARGET_AVX__)
    static void createTriangleMeshTriangle8(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
#endif

    static void createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);

  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Line4iIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Line4iMBIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1vIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1iIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1vIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1iIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1iMBIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4XfmTriangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle8Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Subdivpatch1CachedIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4GridAOSIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4VirtualIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4VirtualMBIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4vIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4iIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4iMBIntersector1Pluecker);
    
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Line4iIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Line4iMBIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1vIntersector4Single);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1iIntersector4Single);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1vIntersector4Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1iIntersector4Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1iMBIntersector4Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle8Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle8Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4vIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4iIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4vMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4iIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4iMBIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Subdivpatch1CachedIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4GridAOSIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4VirtualIntersector4Chunk);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4VirtualMBIntersector4Chunk);
    
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Line4iIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Line4iMBIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1vIntersector8Single);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1iIntersector8Single);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1vIntersector8Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1iIntersector8Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1iMBIntersector8Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle8Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle8Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iMBIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Subdivpatch1CachedIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4GridAOSIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4VirtualIntersector8Chunk);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4VirtualMBIntersector8Chunk);
    
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Line4iIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Line4iMBIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1vIntersector16Single);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1iIntersector16Single);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1vIntersector16Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1iIntersector16Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1iMBIntersector16Single_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle8Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle8Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4vIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4iIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4vMBIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4vIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4vIntersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4iIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4iMBIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Subdivpatch1CachedIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4GridAOSIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4VirtualIntersector16Chunk);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4VirtualMBIntersector16Chunk);
    
    DEFINE_BUILDER2(void,Scene,const createLineSegmentsAccelTy,BVH4BuilderTwoLevelLineSegmentsSAH);
    DEFINE_BUILDER2(void,Scene,const createTriangleMeshAccelTy,BVH4BuilderTwoLevelTriangleMeshSAH);
    DEFINE_BUILDER2(void,Scene,const createTriangleMeshAccelTy,BVH4BuilderInstancingTriangleMeshSAH);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Bezier1vBuilder_OBB_New);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Bezier1iBuilder_OBB_New);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Bezier1iMBBuilder_OBB_New);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4SceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle8SceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4vSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4iSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4vMBSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Quad4vSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Quad4iSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Quad4iMBSceneBuilderSAH);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4SceneBuilderSpatialSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle8SceneBuilderSpatialSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4vSceneBuilderSpatialSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4iSceneBuilderSpatialSAH);
    
    DEFINE_BUILDER2(void,LineSegments,size_t,BVH4Line4iMeshBuilderSAH);
    DEFINE_BUILDER2(void,LineSegments,size_t,BVH4Line4iMBMeshBuilderSAH);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4MeshBuilderSAH);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle8MeshBuilderSAH);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4vMeshBuilderSAH);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4iMeshBuilderSAH);
    DEFINE_BUILDER2(void,QuadMesh,size_t,BVH4Quad4vMeshBuilderSAH);
    DEFINE_BUILDER2(void,QuadMesh,size_t,BVH4Quad4iMeshBuilderSAH);
    DEFINE_BUILDER2(void,QuadMesh,size_t,BVH4Quad4iMBMeshBuilderSAH);

    DEFINE_BUILDER2(void,Scene,size_t,BVH4Bezier1vSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Bezier1iSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Line4iSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Line4iMBSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4VirtualSceneBuilderSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4VirtualMBSceneBuilderSAH);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH4SubdivPatch1CachedBuilderBinnedSAH);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4SubdivGridEagerBuilderBinnedSAH);
    
    DEFINE_BUILDER2(void,LineSegments,size_t,BVH4Line4iMeshRefitSAH);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4MeshRefitSAH);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle8MeshRefitSAH);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4vMeshRefitSAH);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4iMeshRefitSAH);
    
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4SceneBuilderMortonGeneral);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle8SceneBuilderMortonGeneral);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4vSceneBuilderMortonGeneral);
    DEFINE_BUILDER2(void,Scene,size_t,BVH4Triangle4iSceneBuilderMortonGeneral);
    
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4MeshBuilderMortonGeneral);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle8MeshBuilderMortonGeneral);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4vMeshBuilderMortonGeneral);
    DEFINE_BUILDER2(void,TriangleMesh,size_t,BVH4Triangle4iMeshBuilderMortonGeneral);
  };
}
