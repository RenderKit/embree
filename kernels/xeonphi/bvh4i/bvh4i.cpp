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

#include "bvh4i.h"
#include "bvh4i_builder.h"
#include "bvh4i_builder_morton.h"
#include "bvh4i_builder_morton_enhanced.h"

#include "common/accelinstance.h"
#include "geometry/triangle1.h"

namespace embree
{

#if 1
  __aligned(64) BVH4i::Helper BVH4i::initQBVHNode[4] = { 
    { pos_inf, pos_inf, pos_inf,BVH4i::invalidNode},
    { neg_inf, neg_inf, neg_inf,BVH4i::invalidNode},
    { pos_inf, pos_inf, pos_inf,BVH4i::invalidNode},
    { neg_inf, neg_inf, neg_inf,BVH4i::invalidNode}
   };
#else
  __aligned(64) BVH4i::Helper BVH4i::initQBVHNode[4] = { 
    {1E38f,1E38f,1E38f,BVH_LEAF_MASK},
    {1E38f,1E38f,1E38f,BVH_LEAF_MASK},
    {1E38f,1E38f,1E38f,BVH_LEAF_MASK},
    {1E38f,1E38f,1E38f,BVH_LEAF_MASK}
  };
#endif

  /*! intersector registration functions */
  DECLARE_SYMBOL(Accel::Intersector1 ,BVH4iTriangle1Intersector1);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4iTriangle1Intersector16ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4iTriangle1Intersector16SingleMoeller);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4iTriangle1Intersector16HybridMoeller);

  DECLARE_SYMBOL(Accel::Intersector1 ,BVH4iVirtualGeometryIntersector1);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4iVirtualGeometryIntersector16);
  DECLARE_SYMBOL(Accel::Intersector1 ,BVH4iBezierCurvesIntersector1);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4iBezierCurvesIntersector16);

  DECLARE_SYMBOL(Accel::Intersector1 ,BVH4iTriangle1mcIntersector1);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4iTriangle1mcIntersector16SingleMoeller);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4iTriangle1mcIntersector16ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4iTriangle1mcIntersector16HybridMoeller);

  void BVH4iRegister () 
  {
    int features = getCPUFeatures();

    /* default target */
    SELECT_SYMBOL_KNC(features,BVH4iTriangle1Intersector1);
    SELECT_SYMBOL_KNC(features,BVH4iTriangle1Intersector16ChunkMoeller);
    SELECT_SYMBOL_KNC(features,BVH4iTriangle1Intersector16SingleMoeller);
    SELECT_SYMBOL_KNC(features,BVH4iTriangle1Intersector16HybridMoeller);
    SELECT_SYMBOL_KNC(features,BVH4iVirtualGeometryIntersector1);
    SELECT_SYMBOL_KNC(features,BVH4iVirtualGeometryIntersector16);
    SELECT_SYMBOL_KNC(features,BVH4iBezierCurvesIntersector1);
    SELECT_SYMBOL_KNC(features,BVH4iBezierCurvesIntersector16);

    SELECT_SYMBOL_KNC(features,BVH4iTriangle1mcIntersector1);
    SELECT_SYMBOL_KNC(features,BVH4iTriangle1mcIntersector16SingleMoeller);
    SELECT_SYMBOL_KNC(features,BVH4iTriangle1mcIntersector16ChunkMoeller);
    SELECT_SYMBOL_KNC(features,BVH4iTriangle1mcIntersector16HybridMoeller);

  }


  Accel::Intersectors BVH4iTriangle1Intersectors(BVH4i* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4iTriangle1Intersector1;
    if      (g_traverser == "default") intersectors.intersector16 = BVH4iTriangle1Intersector16HybridMoeller;
    else if (g_traverser == "hybrid" ) intersectors.intersector16 = BVH4iTriangle1Intersector16HybridMoeller;
    else if (g_traverser == "chunk"  ) intersectors.intersector16 = BVH4iTriangle1Intersector16ChunkMoeller;
    else if (g_traverser == "single" ) intersectors.intersector16 = BVH4iTriangle1Intersector16SingleMoeller;
    else throw std::runtime_error("unknown traverser "+g_traverser+" for BVH4i<Triangle1>");      
    return intersectors;
  }

  Accel::Intersectors BVH4iTriangle1mcIntersectors(BVH4i* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4iTriangle1mcIntersector1; 
    if      (g_traverser == "default") intersectors.intersector16 = BVH4iTriangle1mcIntersector16HybridMoeller;
    else if (g_traverser == "hybrid" ) intersectors.intersector16 = BVH4iTriangle1mcIntersector16HybridMoeller;
    else if (g_traverser == "chunk"  ) intersectors.intersector16 = BVH4iTriangle1mcIntersector16ChunkMoeller;
    else if (g_traverser == "single" ) intersectors.intersector16 = BVH4iTriangle1mcIntersector16SingleMoeller;
    else throw std::runtime_error("unknown traverser "+g_traverser+" for BVH4i<Triangle1>");      
    return intersectors;
  }


  Accel::Intersectors BVH4iVirtualGeometryIntersectors(BVH4i* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4iVirtualGeometryIntersector1;
    intersectors.intersector16 = BVH4iVirtualGeometryIntersector16;
    return intersectors;
  }

  Accel::Intersectors BVH4iBezierCurvesIntersectors(BVH4i* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4iBezierCurvesIntersector1;
    intersectors.intersector16 = BVH4iBezierCurvesIntersector16;
    return intersectors;
  }

  Accel* BVH4i::BVH4iTriangle1ObjectSplitBinnedSAH(Scene* scene)
  { 
    BVH4i* accel = new BVH4i(SceneTriangle1::type,scene);   
    Builder* builder = BVH4iBuilder::create(accel,&scene->flat_triangle_source_1,scene,BVH4iBuilder::BVH4I_BUILDER_DEFAULT);    
    Accel::Intersectors intersectors = BVH4iTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4i::BVH4iTriangle1ObjectSplitMorton(Scene* scene)
  { 
    BVH4i* accel = new BVH4i(SceneTriangle1::type,scene);   
    Builder* builder = BVH4iBuilderMorton::create(accel,&scene->flat_triangle_source_1,scene);  
    Accel::Intersectors intersectors = BVH4iTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4i::BVH4iTriangle1ObjectSplitEnhancedMorton(Scene* scene)
  { 
    BVH4i* accel = new BVH4i(SceneTriangle1::type,scene);
    
    Builder* builder = BVH4iBuilderMortonEnhanced::create(accel,&scene->flat_triangle_source_1,scene);
    
    Accel::Intersectors intersectors = BVH4iTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4i::BVH4iTriangle1PreSplitsBinnedSAH(Scene* scene)
  {
    BVH4i* accel = new BVH4i(SceneTriangle1::type,scene);
    
    Builder* builder = BVH4iBuilder::create(accel,&scene->flat_triangle_source_1,scene,BVH4iBuilder::BVH4I_BUILDER_PRESPLITS);
    
    Accel::Intersectors intersectors = BVH4iTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);    
  }


  Accel* BVH4i::BVH4iTriangle1MemoryConservativeBinnedSAH(Scene* scene)
  {
    BVH4i* accel = new BVH4i(SceneTriangle1::type,scene);
    
    Builder* builder = BVH4iBuilder::create(accel,&scene->flat_triangle_source_1,scene,BVH4iBuilder::BVH4I_BUILDER_MEMORY_CONSERVATIVE);       
    Accel::Intersectors intersectors = BVH4iTriangle1mcIntersectors(accel);
    scene->needVertices = true;

    return new AccelInstance(accel,builder,intersectors);    
  }

  Accel* BVH4i::BVH4iVirtualGeometryBinnedSAH(Scene* scene)
  {
    BVH4i* accel = new BVH4i(SceneTriangle1::type,scene);    
    Builder* builder = BVH4iBuilder::create(accel,NULL,scene,BVH4iBuilder::BVH4I_BUILDER_VIRTUAL_GEOMETRY);   
    Accel::Intersectors intersectors = BVH4iVirtualGeometryIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);    
  }

  Accel* BVH4i::BVH4iBezierCurvesBinnedSAH(Scene* scene)
  {
    BVH4i* accel = new BVH4i(SceneTriangle1::type,scene);    
    Builder* builder = BVH4iBuilder::create(accel,NULL,scene,BVH4iBuilder::BVH4I_BUILDER_BEZIER_CURVES);   
    Accel::Intersectors intersectors = BVH4iBezierCurvesIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);    
  }

  BVH4i::~BVH4i()
  {
    if (qbvh)  os_free(qbvh,size_node);
    if (accel) os_free(accel,size_accel);
  }


  float BVH4i::sah () {
    return sah(root,bounds)/area(bounds);
  }

  float BVH4i::sah (NodeRef& node, BBox3fa bounds)
  {
    float f = bounds.empty() ? 0.0f : area(bounds);

    if (node.isNode()) 
    {
      Node* n = node.node(nodePtr());
      for (size_t c=0; c<4; c++) 
        f += sah(n->child(c),n->bounds(c));
      return f;
    }
    else 
    {
      unsigned int num; node.leaf(triPtr(),num);
      return f*num;
    }
  }

}
