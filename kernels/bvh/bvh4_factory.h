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
  /*! BVH4 instantiations */
  class BVH4Factory : public BVHFactory
  {
  public:
    BVH4Factory(int bfeatures, int ifeatures);

  public:
    Accel* BVH4Bezier1v(Scene* scene);
    Accel* BVH4Bezier1i(Scene* scene);
    Accel* BVH4Line4i(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC);
    Accel* BVH4Line4iMB(Scene* scene);

    Accel* BVH4OBBBezier1v(Scene* scene);
    Accel* BVH4OBBBezier1i(Scene* scene);
    Accel* BVH4OBBBezier1iMB(Scene* scene);
    
    Accel* BVH4Triangle4   (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Triangle4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::ROBUST);
    Accel* BVH4Triangle4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Triangle4vMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Triangle4iMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH4Quad4v  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Quad4i  (Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);
    Accel* BVH4Quad4iMB(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC, IntersectVariant ivariant = IntersectVariant::FAST);

    Accel* BVH4QuantizedTriangle4i(Scene* scene);
    Accel* BVH4QuantizedQuad4i(Scene* scene);
 
    Accel* BVH4SubdivPatch1Eager(Scene* scene);
    Accel* BVH4SubdivPatch1(Scene* scene, bool cached);
    Accel* BVH4SubdivPatch1MB(Scene* scene, bool cached);

    Accel* BVH4UserGeometry(Scene* scene, BuildVariant bvariant = BuildVariant::STATIC);
    Accel* BVH4UserGeometryMB(Scene* scene);

    Accel* BVH4InstancedBVH4Triangle4ObjectSplit(Scene* scene);

  private:
    void selectBuilders(int features);
    void selectIntersectors(int features);
    
  private:
    Accel::Intersectors BVH4Line4iIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Line4iMBIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1vIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1iIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1vIntersectors_OBB(BVH4* bvh);
    Accel::Intersectors BVH4Bezier1iIntersectors_OBB(BVH4* bvh);
    Accel::Intersectors BVH4OBBBezier1iMBIntersectors_OBB(BVH4* bvh);
    Accel::Intersectors BVH4Triangle4IntersectorsInstancing(BVH4* bvh);

    Accel::Intersectors BVH4Triangle4Intersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Triangle4vIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Triangle4iIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Triangle4iMBIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Triangle4vMBIntersectors(BVH4* bvh, IntersectVariant ivariant);

    Accel::Intersectors BVH4Quad4vIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Quad4iIntersectors(BVH4* bvh, IntersectVariant ivariant);
    Accel::Intersectors BVH4Quad4iMBIntersectors(BVH4* bvh, IntersectVariant ivariant);

    Accel::Intersectors QBVH4Quad4iIntersectors(BVH4* bvh);
    Accel::Intersectors QBVH4Triangle4iIntersectors(BVH4* bvh);

    Accel::Intersectors BVH4UserGeometryIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4UserGeometryMBIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4SubdivPatch1Intersectors(BVH4* bvh);
    Accel::Intersectors BVH4SubdivPatch1EagerIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4SubdivPatch1CachedIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4SubdivPatch1MBIntersectors(BVH4* bvh);
    Accel::Intersectors BVH4SubdivPatch1CachedMBIntersectors(BVH4* bvh);
    
    static void createLineSegmentsLine4i(LineSegments* mesh, AccelData*& accel, Builder*& builder);

    static void createTriangleMeshTriangle4Morton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4vMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4iMorton(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4v(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createTriangleMeshTriangle4i(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);

    static void createQuadMeshQuad4v(QuadMesh* mesh, AccelData*& accel, Builder*& builder);
    static void createQuadMeshQuad4vMorton(QuadMesh* mesh, AccelData*& accel, Builder*& builder);

    static void createAccelSetMesh(AccelSet* mesh, AccelData*& accel, Builder*& builder);
    
  private:
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Line4iIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Line4iMBIntersector1);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1vIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1iIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1vIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Bezier1iIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4OBBBezier1iMBIntersector1_OBB);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4XfmTriangle4Intersector1Moeller);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4Intersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4vMBIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Triangle4iMBIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4vIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4iIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4vIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4iIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4iMBIntersector1Moeller);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4Quad4iMBIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,QBVH4Triangle4iIntersector1Pluecker);
    DEFINE_SYMBOL2(Accel::Intersector1,QBVH4Quad4iIntersector1Pluecker);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4SubdivPatch1Intersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4SubdivPatch1EagerIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4SubdivPatch1CachedIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4SubdivPatch1MBIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4SubdivPatch1CachedMBIntersector1);

    DEFINE_SYMBOL2(Accel::Intersector1,BVH4VirtualIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector1,BVH4VirtualMBIntersector1);
        
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Line4iIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Line4iMBIntersector4);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1vIntersector4Hybrid);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1iIntersector4Hybrid);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1vIntersector4Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Bezier1iIntersector4Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4OBBBezier1iMBIntersector4Hybrid_OBB);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4Intersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4iIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4vIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4iIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4vMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4iMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4vMBIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Triangle4iMBIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4iIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4vIntersector4HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4iIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4iMBIntersector4HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4Quad4iMBIntersector4HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4SubdivPatch1Intersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4SubdivPatch1EagerIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4SubdivPatch1CachedIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4SubdivPatch1MBIntersector4);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4SubdivPatch1CachedMBIntersector4);

    DEFINE_SYMBOL2(Accel::Intersector4,BVH4VirtualIntersector4Chunk);
    DEFINE_SYMBOL2(Accel::Intersector4,BVH4VirtualMBIntersector4Chunk);
    
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Line4iIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Line4iMBIntersector8);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1vIntersector8Hybrid);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1iIntersector8Hybrid);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1vIntersector8Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Bezier1iIntersector8Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4OBBBezier1iMBIntersector8Hybrid_OBB);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4Intersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4vMBIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Triangle4iMBIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4vIntersector8HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iMBIntersector8HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4Quad4iMBIntersector8HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4SubdivPatch1Intersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4SubdivPatch1EagerIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4SubdivPatch1CachedIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4SubdivPatch1MBIntersector8);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4SubdivPatch1CachedMBIntersector8);

    DEFINE_SYMBOL2(Accel::Intersector8,BVH4VirtualIntersector8Chunk);
    DEFINE_SYMBOL2(Accel::Intersector8,BVH4VirtualMBIntersector8Chunk);
    
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Line4iIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Line4iMBIntersector16);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1vIntersector16Hybrid);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1iIntersector16Hybrid);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1vIntersector16Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Bezier1iIntersector16Hybrid_OBB);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4OBBBezier1iMBIntersector16Hybrid_OBB);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4Intersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4iIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4vIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4iIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4vMBIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4iMBIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4vMBIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Triangle4iMBIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4vIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4vIntersector16HybridMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4iIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4vIntersector16HybridPluecker);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4iIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4iMBIntersector16HybridMoeller);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4Quad4iMBIntersector16HybridPluecker);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH4SubdivPatch1Intersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4SubdivPatch1EagerIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4SubdivPatch1CachedIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4SubdivPatch1MBIntersector16);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4SubdivPatch1CachedMBIntersector16);

    DEFINE_SYMBOL2(Accel::Intersector16,BVH4VirtualIntersector16Chunk);
    DEFINE_SYMBOL2(Accel::Intersector16,BVH4VirtualMBIntersector16Chunk);

    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH4Line4iIntersectorStream);
    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1vIntersectorStream);
    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1iIntersectorStream);
    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1vIntersectorStream_OBB);
    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH4Bezier1iIntersectorStream_OBB);

    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4IntersectorStreamPacketFallback);

    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4IntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4IntersectorStreamMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4iIntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4vIntersectorStreamPluecker);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Triangle4iIntersectorStreamPluecker);

    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Quad4vIntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Quad4vIntersectorStreamMoellerNoFilter);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Quad4iIntersectorStreamMoeller);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Quad4vIntersectorStreamPluecker);
    DEFINE_SYMBOL2(Accel::IntersectorN, BVH4Quad4iIntersectorStreamPluecker);

    //DEFINE_SYMBOL2(Accel::IntersectorN,BVH4SubdivPatch1CachedIntersectorStream);
    DEFINE_SYMBOL2(Accel::IntersectorN,BVH4VirtualIntersectorStream);
    //DEFINE_SYMBOL2(Accel::IntersectorN,QBVH4Triangle4IntersectorStreamMoeller);
       
    // SAH scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Line4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Line4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  
    DEFINE_ISA_FUNCTION(Builder*,BVH4Bezier1vBuilder_OBB_New,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Bezier1iBuilder_OBB_New,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4OBBBezier1iMBBuilder_OBB,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Bezier1vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Bezier1iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4QuantizedTriangle4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4iMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4QuantizedQuad4iSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    
    DEFINE_ISA_FUNCTION(Builder*,BVH4SubdivPatch1EagerBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4SubdivPatch1CachedBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4SubdivPatch1CachedMBBuilderSAH,void* COMMA Scene* COMMA size_t);
    
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualMBSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

    // spatial scene builder
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4SceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vSceneBuilderFastSpatialSAH,void* COMMA Scene* COMMA size_t);
    
    // twolevel scene builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelLineSegmentsSAH,void* COMMA Scene* COMMA const createLineSegmentsAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderInstancingTriangleMeshSAH,void* COMMA Scene* COMMA const createTriangleMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelQuadMeshSAH,void* COMMA Scene* COMMA const createQuadMeshAccelTy);
    DEFINE_ISA_FUNCTION(Builder*,BVH4BuilderTwoLevelVirtualSAH,void* COMMA Scene* COMMA const createAccelSetAccelTy);
 
    // SAH mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Line4iMeshBuilderSAH,void* COMMA LineSegments* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshBuilderSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshBuilderSAH,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualMeshBuilderSAH,void* COMMA AccelSet* COMMA size_t);

    // mesh refitters
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Line4iMeshRefitSAH,void* COMMA LineSegments* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshRefitSAH,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshRefitSAH,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualMeshRefitSAH,void* COMMA AccelSet* COMMA size_t);
    
    // morton mesh builders
  private:
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4MeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4vMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Triangle4iMeshBuilderMortonGeneral,void* COMMA TriangleMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4Quad4vMeshBuilderMortonGeneral,void* COMMA QuadMesh* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVH4VirtualMeshBuilderMortonGeneral,void* COMMA AccelSet* COMMA size_t);
  };
}
