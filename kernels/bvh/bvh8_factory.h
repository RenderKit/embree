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

#pragma once

#include "bvh_factory.h"

namespace embree
{
  /*! BVH8 instantiations */
  class BVH8Factory : public BVHFactory
  {
  public:
    BVH8Factory(int bfeatures, int ifeatures);

  public:
    Accel* BVH8OBBBezier1v(Scene* scene);
    Accel* BVH8OBBBezier1i(Scene* scene);
    Accel* BVH8OBBBezier1iMB(Scene* scene);

    Accel* BVH8Line4i(Scene* scene);
    Accel* BVH8Line4iMB(Scene* scene);

    Accel* BVH8Triangle4   (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4vMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Triangle4iMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH8Quad4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Quad4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH8Quad4iMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH8QuantizedTriangle4i(Scene* scene);
    Accel* BVH8QuantizedTriangle4(Scene* scene);
    Accel* BVH8QuantizedQuad4i(Scene* scene);

    Accel* BVH8UserGeometry(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC);
    Accel* BVH8UserGeometryMB(Scene* scene);
  
    static void createTriangleMeshTriangle4Morton (TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4 (TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);

    static void createQuadMeshQuad4vMorton(QuadMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createQuadMeshQuad4v(QuadMesh* mesh, AccelData*& accel, Builder*& builder);

    static void createAccelSetMesh(AccelSet* mesh, AccelData*& accel, Builder*& builder);

  private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

  private:
    Accel::Intersectors BVH8Line4iIntersectors(BVH8* bvh);
    Accel::Intersectors BVH8Line4iMBIntersectors(BVH8* bvh);
    Accel::Intersectors BVH8Bezier1vIntersectors_OBB(BVH8* bvh);
    Accel::Intersectors BVH8Bezier1iIntersectors_OBB(BVH8* bvh);
    Accel::Intersectors BVH8OBBBezier1iMBIntersectors_OBB(BVH8* bvh);

    Accel::Intersectors BVH8Triangle4Intersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH8Triangle4vIntersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH8Triangle4iIntersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH8Triangle4iMBIntersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH8Triangle4vMBIntersectors(BVH8* bvh, IntersectVariant ivariant);

    Accel::Intersectors BVH8Quad4vIntersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH8Quad4iIntersectors(BVH8* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH8Quad4iMBIntersectors(BVH8* bvh, IntersectVariant ivariant);

    Accel::Intersectors QBVH8Triangle4iIntersectors(BVH8* bvh);
    Accel::Intersectors QBVH8Triangle4Intersectors(BVH8* bvh);
    Accel::Intersectors QBVH8Quad4iIntersectors(BVH8* bvh);

    Accel::Intersectors BVH8UserGeometryIntersectors(BVH8* bvh);
    Accel::Intersectors BVH8UserGeometryMBIntersectors(BVH8* bvh);

  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Line4iIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Line4iMBIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Bezier1vIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Bezier1iIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8OBBBezier1iMBIntersector1_OBB);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4vMBIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Triangle4iMBIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4vIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4iIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4vIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4iIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4iMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8Quad4iMBIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4iIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,QBVH8Triangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,QBVH8Quad4iIntersector1Pluecker);
    
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8VirtualIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH8VirtualMBIntersector1);
    
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Line4iIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Line4iMBIntersector4);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Bezier1vIntersector4Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Bezier1iIntersector4Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8OBBBezier1iMBIntersector4Hybrid_OBB);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4vMBIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Triangle4iMBIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4iIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4vIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4iIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4iMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8Quad4iMBIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH8VirtualIntersector4Chunk);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH8VirtualMBIntersector4Chunk);
    
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Line4iIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Line4iMBIntersector8);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Bezier1vIntersector8Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Bezier1iIntersector8Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8OBBBezier1iMBIntersector8Hybrid_OBB);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4vMBIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Triangle4iMBIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4iIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4iIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4iMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8Quad4iMBIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH8VirtualIntersector8Chunk);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH8VirtualMBIntersector8Chunk);
   
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Line4iIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Line4iMBIntersector16);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Bezier1vIntersector16Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Bezier1iIntersector16Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8OBBBezier1iMBIntersector16Hybrid_OBB);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vMBIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iMBIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4vMBIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Triangle4iMBIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4vIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4iIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4iMBIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8Quad4iMBIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH8VirtualIntersector16Chunk);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH8VirtualMBIntersector16Chunk);
   
    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Line4iIntersectorStream);
    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Bezier1vIntersectorStream_OBB);
    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Bezier1iIntersectorStream_OBB);

    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8IntersectorStreamPacketFallback);

    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4IntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4IntersectorStreamMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4iIntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4vIntersectorStreamPluecker);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Triangle4iIntersectorStreamPluecker);

    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Quad4vIntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Quad4vIntersectorStreamMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Quad4iIntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Quad4vIntersectorStreamPluecker);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8Quad4iIntersectorStreamPluecker);

    DEFINE_SYMBOL2(Accel::IntersectorN,BVH8VirtualIntersectorStream);

    // SAH scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Line4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Line4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    DEFINE_ISA_FUNCTION(Builder*,BVH8Bezier1vBuilder_OBB_New,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Bezier1iBuilder_OBB_New,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8OBBBezier1iMBBuilder_OBB,void* COMMA Scene* COMMA size_t);
 
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8QuantizedTriangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8QuantizedTriangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
 
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8QuantizedQuad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    
    DEFINE_ISA_FUNCTION(Builder*,BVH8VirtualSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8VirtualMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    // SAH spatial scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);

    // twolevel scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelQuadMeshSAH,void* COMMA Scene* COMMA const createQuadMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH8BuilderTwoLevelVirtualSAH,void* COMMA Scene* COMMA const createAccelSetAccelTy);
 
    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshBuilderSAH,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8VirtualMeshBuilderSAH,void* COMMA AccelSet* COMMA size_t);

    // mesh refitters
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshRefitSAH,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8VirtualMeshRefitSAH,void* COMMA AccelSet* COMMA size_t);
 
    // morton mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4MeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4vMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Triangle4iMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8Quad4vMeshBuilderMortonGeneral,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH8VirtualMeshBuilderMortonGeneral,void* COMMA AccelSet* COMMA size_t);
  };
}
