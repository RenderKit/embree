// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"

namespace embree
{
  DECLARE_INTERSECTOR1(BVH4Triangle1Intersector1Moeller);
  DECLARE_INTERSECTOR1(BVH4Triangle4Intersector1Moeller);
  DECLARE_INTERSECTOR1(BVH4Triangle8Intersector1Moeller);
  DECLARE_INTERSECTOR1(BVH4Triangle1vIntersector1Pluecker);
  DECLARE_INTERSECTOR1(BVH4Triangle4vIntersector1Pluecker);
  DECLARE_INTERSECTOR1(BVH4Triangle4iIntersector1Pluecker);
  DECLARE_INTERSECTOR1(BVH4VirtualIntersector1);

  DECLARE_INTERSECTOR4(BVH4Triangle1Intersector4ChunkMoeller);
  DECLARE_INTERSECTOR4(BVH4Triangle4Intersector4ChunkMoeller);
  DECLARE_INTERSECTOR4(BVH4Triangle8Intersector4ChunkMoeller);
  DECLARE_INTERSECTOR4(BVH4Triangle4Intersector4HybridMoeller);
  DECLARE_INTERSECTOR4(BVH4Triangle8Intersector4HybridMoeller);
  DECLARE_INTERSECTOR4(BVH4Triangle1vIntersector4ChunkPluecker);
  DECLARE_INTERSECTOR4(BVH4Triangle4vIntersector4ChunkPluecker);
  DECLARE_INTERSECTOR4(BVH4Triangle4vIntersector4HybridPluecker);
  DECLARE_INTERSECTOR4(BVH4Triangle4iIntersector4ChunkPluecker);
  DECLARE_INTERSECTOR4(BVH4VirtualIntersector4Chunk);

  DECLARE_INTERSECTOR8(BVH4Triangle1Intersector8ChunkMoeller);
  DECLARE_INTERSECTOR8(BVH4Triangle4Intersector8ChunkMoeller);
  DECLARE_INTERSECTOR8(BVH4Triangle8Intersector8ChunkMoeller);
  DECLARE_INTERSECTOR8(BVH4Triangle4Intersector8HybridMoeller);
  DECLARE_INTERSECTOR8(BVH4Triangle8Intersector8HybridMoeller);
  DECLARE_INTERSECTOR8(BVH4Triangle1vIntersector8ChunkPluecker);
  DECLARE_INTERSECTOR8(BVH4Triangle4vIntersector8ChunkPluecker);
  DECLARE_INTERSECTOR8(BVH4Triangle4vIntersector8HybridPluecker);
  DECLARE_INTERSECTOR8(BVH4Triangle4iIntersector8ChunkPluecker);
  DECLARE_INTERSECTOR8(BVH4VirtualIntersector8Chunk);

  DECLARE_INTERSECTOR16(BVH4Triangle1Intersector16ChunkMoeller);
  DECLARE_INTERSECTOR16(BVH4Triangle4Intersector16ChunkMoeller);
  DECLARE_INTERSECTOR16(BVH4Triangle8Intersector16ChunkMoeller);
  DECLARE_INTERSECTOR16(BVH4Triangle4Intersector16HybridMoeller);
  DECLARE_INTERSECTOR16(BVH4Triangle8Intersector16HybridMoeller);
  DECLARE_INTERSECTOR16(BVH4Triangle1vIntersector16ChunkPluecker);
  DECLARE_INTERSECTOR16(BVH4Triangle4vIntersector16ChunkPluecker);
  DECLARE_INTERSECTOR16(BVH4Triangle4vIntersector16HybridPluecker);
  DECLARE_INTERSECTOR16(BVH4Triangle4iIntersector16ChunkPluecker);
  DECLARE_INTERSECTOR16(BVH4VirtualIntersector16Chunk);

  DECLARE_TOPLEVEL_BUILDER(BVH4BuilderTopLevelFast);

  DECLARE_BUILDER(BVH4BuilderObjectSplit4Fast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4BuilderObjectSplit4TriangleMeshFast);

  DECLARE_BUILDER(BVH4BuilderMortonFast);
  DECLARE_TRIANGLEMESH_BUILDER(BVH4BuilderMortonTriangleMeshFast);

  DECLARE_TRIANGLEMESH_BUILDER(BVH4BuilderRefitObjectSplit4TriangleMeshFast);

  Builder* BVH4BuilderObjectSplit1 (void* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize);
  Builder* BVH4BuilderObjectSplit4 (void* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize);
  Builder* BVH4BuilderObjectSplit8 (void* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize);
  Builder* BVH4BuilderSpatialSplit1 (void* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize);
  Builder* BVH4BuilderSpatialSplit4 (void* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize);
  Builder* BVH4BuilderSpatialSplit8 (void* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize);
  
  void BVH4Register () 
  {
    int features = getCPUFeatures();

    SELECT_DEFAULT_SSE41(features,BVH4BuilderTopLevelFast);

    SELECT_DEFAULT(features,BVH4BuilderObjectSplit4Fast);
    SELECT_DEFAULT(features,BVH4BuilderObjectSplit4TriangleMeshFast);

    SELECT_DEFAULT_SSE41(features,BVH4BuilderMortonFast);
    SELECT_DEFAULT_SSE41(features,BVH4BuilderMortonTriangleMeshFast);
    
    SELECT_DEFAULT(features,BVH4BuilderRefitObjectSplit4TriangleMeshFast);

    /* select intersectors1 */
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle1Intersector1Moeller);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector1Moeller);
    SELECT_AVX_AVX2              (features,BVH4Triangle8Intersector1Moeller);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle1vIntersector1Pluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector1Pluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4iIntersector1Pluecker);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4VirtualIntersector1);

    /* select intersectors4 */
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle1Intersector4ChunkMoeller);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector4ChunkMoeller);
    SELECT_AVX_AVX2              (features,BVH4Triangle8Intersector4ChunkMoeller);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector4HybridMoeller);
    SELECT_AVX_AVX2              (features,BVH4Triangle8Intersector4HybridMoeller);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle1vIntersector4ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector4ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector4HybridPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4iIntersector4ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4VirtualIntersector4Chunk);

    /* select intersectors8 */
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle1Intersector8ChunkMoeller);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector8ChunkMoeller);
    SELECT_AVX_AVX2              (features,BVH4Triangle8Intersector8ChunkMoeller);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector8HybridMoeller);
    SELECT_AVX_AVX2              (features,BVH4Triangle8Intersector8HybridMoeller);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle1vIntersector8ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector8ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector8HybridPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4iIntersector8ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4VirtualIntersector8Chunk);

    /* select intersectors16 */
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle1Intersector16ChunkMoeller);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector16ChunkMoeller);
    SELECT_AVX_AVX2              (features,BVH4Triangle8Intersector16ChunkMoeller);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4Triangle4Intersector16HybridMoeller);
    SELECT_AVX_AVX2              (features,BVH4Triangle8Intersector16HybridMoeller);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle1vIntersector16ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector16ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4vIntersector16HybridPluecker);
    SELECT_DEFAULT_SSE41_AVX     (features,BVH4Triangle4iIntersector16ChunkPluecker);
    SELECT_DEFAULT_SSE41_AVX_AVX2(features,BVH4VirtualIntersector16Chunk);
  }

  BVH4::BVH4 (const PrimitiveType& primTy, void* geometry)
  : primTy(primTy), geometry(geometry), root(emptyNode),
    numPrimitives(0), numVertices(0),
    nodes(NULL), bytesNodes(0), primitives(NULL), bytesPrimitives(0) {}

  BVH4::~BVH4 () {
    if (nodes) os_free(nodes, bytesNodes);
    if (primitives) os_free(primitives, bytesPrimitives);
    for (size_t i=0; i<objects.size(); i++) delete objects[i];
  }
  
  Accel::Intersectors BVH4Triangle1Intersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle1Intersector1Moeller;
    intersectors.intersector4 = BVH4Triangle1Intersector4ChunkMoeller;
    intersectors.intersector8 = BVH4Triangle1Intersector8ChunkMoeller;
    intersectors.intersector16 = BVH4Triangle1Intersector16ChunkMoeller;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4IntersectorsChunk(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4Intersector1Moeller;
    intersectors.intersector4 = BVH4Triangle4Intersector4ChunkMoeller;
    intersectors.intersector8 = BVH4Triangle4Intersector8ChunkMoeller;
    intersectors.intersector16 = BVH4Triangle4Intersector16ChunkMoeller;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4IntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
#if !defined(__MIC__)
    intersectors.intersector1 = BVH4Triangle4Intersector1Moeller;
    intersectors.intersector4 = BVH4Triangle4Intersector4HybridMoeller;
    intersectors.intersector8 = BVH4Triangle4Intersector8HybridMoeller;
    intersectors.intersector16 = BVH4Triangle4Intersector16HybridMoeller;
#else
    intersectors.intersector1 = BVH4Triangle4Intersector1Moeller;
    intersectors.intersector4 = BVH4Triangle4Intersector4ChunkMoeller;
    intersectors.intersector8 = BVH4Triangle4Intersector8ChunkMoeller;
    intersectors.intersector16 = BVH4Triangle4Intersector16ChunkMoeller;
#endif
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle8IntersectorsChunk(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle8Intersector1Moeller;
    intersectors.intersector4 = BVH4Triangle8Intersector4ChunkMoeller;
    intersectors.intersector8 = BVH4Triangle8Intersector8ChunkMoeller;
    intersectors.intersector16 = BVH4Triangle8Intersector16ChunkMoeller;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle8IntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
#if !defined(__MIC__)
    intersectors.intersector1 = BVH4Triangle8Intersector1Moeller;
    intersectors.intersector4 = BVH4Triangle8Intersector4HybridMoeller;
    intersectors.intersector8 = BVH4Triangle8Intersector8HybridMoeller;
    intersectors.intersector16 = BVH4Triangle8Intersector16HybridMoeller;
#else
    intersectors.intersector1 = BVH4Triangle8Intersector1Moeller;
    intersectors.intersector4 = BVH4Triangle8Intersector4ChunkMoeller;
    intersectors.intersector8 = BVH4Triangle8Intersector8ChunkMoeller;
    intersectors.intersector16 = BVH4Triangle8Intersector16ChunkMoeller;
#endif
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle1vIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle1vIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle1vIntersector4ChunkPluecker;
    intersectors.intersector8 = BVH4Triangle1vIntersector8ChunkPluecker;
    intersectors.intersector16 = BVH4Triangle1vIntersector16ChunkPluecker;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4vIntersectorsChunk(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4vIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle4vIntersector4ChunkPluecker;
    intersectors.intersector8 = BVH4Triangle4vIntersector8ChunkPluecker;
    intersectors.intersector16 = BVH4Triangle4vIntersector16ChunkPluecker;
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4vIntersectorsHybrid(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
#if !defined(__MIC__)
    intersectors.intersector1 = BVH4Triangle4vIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle4vIntersector4HybridPluecker;
    intersectors.intersector8 = BVH4Triangle4vIntersector8HybridPluecker;
    intersectors.intersector16 = BVH4Triangle4vIntersector16HybridPluecker;
#else
    intersectors.intersector1 = BVH4Triangle4vIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle4vIntersector4ChunkPluecker;
    intersectors.intersector8 = BVH4Triangle4vIntersector8ChunkPluecker;
    intersectors.intersector16 = BVH4Triangle4vIntersector16ChunkPluecker;
#endif
    return intersectors;
  }

  Accel::Intersectors BVH4Triangle4iIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4Triangle4iIntersector1Pluecker;
    intersectors.intersector4 = BVH4Triangle4iIntersector4ChunkPluecker;
    intersectors.intersector8 = BVH4Triangle4iIntersector8ChunkPluecker;
    intersectors.intersector16 = BVH4Triangle4iIntersector16ChunkPluecker;
    return intersectors;
  }

  Accel* BVH4::BVH4Triangle1(Scene* scene)
  { 
    BVH4* accel = new BVH4(SceneTriangle1::type);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    
    Builder* builder = NULL;
    if      (g_builder == "default"     ) builder = BVH4BuilderObjectSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "spatialsplit") builder = BVH4BuilderSpatialSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit" ) builder = BVH4BuilderObjectSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "morton"      ) builder = BVH4BuilderMortonFast(accel,&scene->flat_triangle_source_1,scene,4,inf);
    else if (g_builder == "fast"        ) builder = BVH4BuilderObjectSplit4Fast(accel,&scene->flat_triangle_source_1,scene,4,inf);
    else throw std::runtime_error("unknown builder "+g_builder+" for BVH4<Triangle1>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4(Scene* scene)
  { 
    BVH4* accel = new BVH4(SceneTriangle4::type);

    Accel::Intersectors intersectors;
    if      (g_traverser == "default") intersectors = BVH4Triangle4IntersectorsChunk(accel);
    else if (g_traverser == "chunk"  ) intersectors = BVH4Triangle4IntersectorsChunk(accel);
    else if (g_traverser == "hybrid" ) intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    else throw std::runtime_error("unknown traverser "+g_traverser+" for BVH4<Triangle4>");
   
    Builder* builder = NULL;
    if      (g_builder == "default"     ) builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "spatialsplit") builder = BVH4BuilderSpatialSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit" ) builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit1") builder = BVH4BuilderObjectSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit4") builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "morton"      ) builder = BVH4BuilderMortonFast(accel,&scene->flat_triangle_source_1,scene,4,inf);
    else if (g_builder == "fast"        ) builder = BVH4BuilderObjectSplit4Fast(accel,&scene->flat_triangle_source_1,scene,4,inf);
    else throw std::runtime_error("unknown builder "+g_builder+" for BVH4<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

#if defined (__TARGET_AVX__)

  Accel* BVH4::BVH4Triangle8(Scene* scene)
  { 
    BVH4* accel = new BVH4(SceneTriangle8::type);

    Accel::Intersectors intersectors;
    if      (g_traverser == "default") intersectors = BVH4Triangle8IntersectorsChunk(accel);
    else if (g_traverser == "chunk"  ) intersectors = BVH4Triangle8IntersectorsChunk(accel);
    else if (g_traverser == "hybrid" ) intersectors = BVH4Triangle8IntersectorsHybrid(accel);
    else throw std::runtime_error("unknown traverser "+g_traverser+" for BVH4<Triangle8>");
   
    Builder* builder = NULL;
    if      (g_builder == "default"     ) builder = BVH4BuilderObjectSplit8(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "spatialsplit") builder = BVH4BuilderSpatialSplit8(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit" ) builder = BVH4BuilderObjectSplit8(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else throw std::runtime_error("unknown builder "+g_builder+" for BVH4<Triangle8>");

    return new AccelInstance(accel,builder,intersectors);
  }
#endif

  Accel* BVH4::BVH4Triangle1v(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle1v::type);
    Accel::Intersectors intersectors = BVH4Triangle1vIntersectors(accel);

    Builder* builder = NULL;
    if      (g_builder == "default"     ) builder = BVH4BuilderObjectSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "spatialsplit") builder = BVH4BuilderSpatialSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit" ) builder = BVH4BuilderObjectSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "morton"      ) builder = BVH4BuilderMortonFast(accel,&scene->flat_triangle_source_1,scene,4,inf);
    else if (g_builder == "fast"        ) builder = BVH4BuilderObjectSplit4Fast(accel,&scene->flat_triangle_source_1,scene,4,inf);
    else throw std::runtime_error("unknown builder "+g_builder+" for BVH4<Triangle1v>");
        
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4v(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle4v::type);

    Accel::Intersectors intersectors;
    if      (g_traverser == "default") intersectors = BVH4Triangle4vIntersectorsChunk(accel);
    else if (g_traverser == "chunk"  ) intersectors = BVH4Triangle4vIntersectorsChunk(accel);
    else if (g_traverser == "hybrid" ) intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    else throw std::runtime_error("unknown traverser "+g_traverser+" for BVH4<Triangle4>");

    Builder* builder = NULL;
    if      (g_builder == "default"     ) builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "spatialsplit") builder = BVH4BuilderSpatialSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit" ) builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "morton"      ) builder = BVH4BuilderMortonFast(accel,&scene->flat_triangle_source_1,scene,4,inf);
    else if (g_builder == "fast"        ) builder = BVH4BuilderObjectSplit4Fast(accel,&scene->flat_triangle_source_1,scene,4,inf);
    else throw std::runtime_error("unknown builder "+g_builder+" for BVH4<Triangle4v>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4i(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4iType::type,scene);
    Accel::Intersectors intersectors = BVH4Triangle4iIntersectors(accel);

    Builder* builder = NULL;
    if      (g_builder == "default"     ) builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "spatialsplit") builder = BVH4BuilderSpatialSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit" ) builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else throw std::runtime_error("unknown builder "+g_builder+" for BVH4<Triangle4i>");

    scene->needVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  void createTriangleMeshTriangle1Morton(TriangleMeshScene::TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) throw std::runtime_error("internal error");
    accel = new BVH4(TriangleMeshTriangle1::type);
    builder = BVH4BuilderMortonTriangleMeshFast(accel,mesh,4,inf);
  } 

  void createTriangleMeshTriangle1(TriangleMeshScene::TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) throw std::runtime_error("internal error");
    accel = new BVH4(TriangleMeshTriangle1::type);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = BVH4BuilderObjectSplit4TriangleMeshFast(accel,mesh,4,inf); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4BuilderRefitObjectSplit4TriangleMeshFast(accel,mesh,4,inf); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4BuilderMortonTriangleMeshFast(accel,mesh,4,inf); break;
    default: throw std::runtime_error("internal error"); 
    }
  } 

  void createTriangleMeshTriangle4(TriangleMeshScene::TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) throw std::runtime_error("internal error");
    accel = new BVH4(TriangleMeshTriangle4::type);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = BVH4BuilderObjectSplit4TriangleMeshFast(accel,mesh,4,inf); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4BuilderRefitObjectSplit4TriangleMeshFast(accel,mesh,4,inf); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4BuilderMortonTriangleMeshFast(accel,mesh,4,inf); break;
    default: throw std::runtime_error("internal error"); 
    }
  } 

  void createTriangleMeshTriangle1v(TriangleMeshScene::TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) throw std::runtime_error("internal error");
    accel = new BVH4(TriangleMeshTriangle1v::type);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = BVH4BuilderObjectSplit4TriangleMeshFast(accel,mesh,4,inf); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4BuilderRefitObjectSplit4TriangleMeshFast(accel,mesh,4,inf); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4BuilderMortonTriangleMeshFast(accel,mesh,4,inf); break;
    default: throw std::runtime_error("internal error"); 
    }
  } 

  void createTriangleMeshTriangle4v(TriangleMeshScene::TriangleMesh* mesh, BVH4*& accel, Builder*& builder)
  {
    if (mesh->numTimeSteps != 1) throw std::runtime_error("internal error");
    accel = new BVH4(TriangleMeshTriangle4v::type);
    switch (mesh->flags) {
    case RTC_GEOMETRY_STATIC:     builder = BVH4BuilderObjectSplit4TriangleMeshFast(accel,mesh,4,inf); break;
    case RTC_GEOMETRY_DEFORMABLE: builder = BVH4BuilderRefitObjectSplit4TriangleMeshFast(accel,mesh,4,inf); break;
    case RTC_GEOMETRY_DYNAMIC:    builder = BVH4BuilderMortonTriangleMeshFast(accel,mesh,4,inf); break;
    default: throw std::runtime_error("internal error"); 
    }
  } 

#if !defined(__MIC__)
  Accel* BVH4::BVH4BVH4Triangle1Morton(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1::type);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle1Morton);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle1ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1::type);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle1);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle4ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4::type);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle4);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle1vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1v::type);
    Accel::Intersectors intersectors = BVH4Triangle1vIntersectors(accel);
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle1v);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4BVH4Triangle4vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4v::type);
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    Builder* builder = BVH4BuilderTopLevelFast(accel,scene,&createTriangleMeshTriangle4v);
    return new AccelInstance(accel,builder,intersectors);
  }
#endif

  Accel* BVH4::BVH4Triangle1SpatialSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle1::type);
    Builder* builder = BVH4BuilderSpatialSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }
  
  Accel* BVH4::BVH4Triangle4SpatialSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle4::type);
    Builder* builder = BVH4BuilderSpatialSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

#if defined (__TARGET_AVX__)

  Accel* BVH4::BVH4Triangle8SpatialSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle8::type);
    Builder* builder = BVH4BuilderSpatialSplit8(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle8IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

#endif

  Accel* BVH4::BVH4Triangle1ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle1::type);
    Builder* builder = BVH4BuilderObjectSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }
  
  Accel* BVH4::BVH4Triangle4ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle4::type);
    Builder* builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

#if defined (__TARGET_AVX__)

  Accel* BVH4::BVH4Triangle8ObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle8::type);
    Builder* builder = BVH4BuilderObjectSplit8(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle8IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

#endif

  Accel* BVH4::BVH4Triangle1vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle1v::type);
    Builder* builder = BVH4BuilderObjectSplit1(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle1vIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4vObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(SceneTriangle4v::type);
    Builder* builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4iObjectSplit(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle4iType::type,scene);
    Builder* builder = BVH4BuilderObjectSplit4(accel,&scene->flat_triangle_source_1,scene,1,inf);
    Accel::Intersectors intersectors = BVH4Triangle4iIntersectors(accel);
    scene->needVertices = true;
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle1ObjectSplit(TriangleMeshScene::TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1::type);
    Builder* builder = BVH4BuilderObjectSplit4TriangleMeshFast(accel,mesh,4,inf);
    Accel::Intersectors intersectors = BVH4Triangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4ObjectSplit(TriangleMeshScene::TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4::type);
    Builder* builder = BVH4BuilderObjectSplit4TriangleMeshFast(accel,mesh,4,inf);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle1vObjectSplit(TriangleMeshScene::TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle1v::type);
    Builder* builder = BVH4BuilderObjectSplit4TriangleMeshFast(accel,mesh,4,inf);
    Accel::Intersectors intersectors = BVH4Triangle1vIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4vObjectSplit(TriangleMeshScene::TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4v::type);
    Builder* builder = BVH4BuilderObjectSplit4TriangleMeshFast(accel,mesh,4,inf);
    Accel::Intersectors intersectors = BVH4Triangle4vIntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4::BVH4Triangle4Refit(TriangleMeshScene::TriangleMesh* mesh)
  {
    BVH4* accel = new BVH4(TriangleMeshTriangle4::type);
    Builder* builder = BVH4BuilderRefitObjectSplit4TriangleMeshFast(accel,mesh,4,inf);
    Accel::Intersectors intersectors = BVH4Triangle4IntersectorsHybrid(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  void BVH4::clear () 
  {
    root = emptyNode;
    AllocatorPerThread::clear();
  }

  void BVH4::clearBarrier(NodeRef& node)
  {
    if (node.isBarrier())
      node.clearBarrier();
    else if (!node.isLeaf()) {
      Node* n = node.node();
      for (size_t c=0; c<4; c++)
        clearBarrier(n->child(c));
    }
  }
}

