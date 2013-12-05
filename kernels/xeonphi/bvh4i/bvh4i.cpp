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

#include "kernels/xeonphi/bvh4i/bvh4i.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"

#include "bvh4i_builder.h"
#include "bvh4i_builder_morton.h"
#include "bvh4i_builder_morton_enhanced.h"

namespace embree
{
  /*! intersector registration functions */
  DECLARE_INTERSECTOR1(BVH4iTriangle1Intersector1);
  DECLARE_INTERSECTOR16(BVH4iTriangle1Intersector16ChunkMoeller);
  DECLARE_INTERSECTOR16(BVH4iTriangle1Intersector16SingleMoeller);
  DECLARE_INTERSECTOR16(BVH4iTriangle1Intersector16HybridMoeller);

  void BVH4iRegister () 
  {
    int features = getCPUFeatures();

    /* default target */
    SELECT_KNC(features,BVH4iTriangle1Intersector1);
    SELECT_KNC(features,BVH4iTriangle1Intersector16ChunkMoeller);
    SELECT_KNC(features,BVH4iTriangle1Intersector16SingleMoeller);
    SELECT_KNC(features,BVH4iTriangle1Intersector16HybridMoeller);
  }


  Accel::Intersectors BVH4iTriangle1Intersectors(BVH4i* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4iTriangle1Intersector1;
    intersectors.intersector16 = BVH4iTriangle1Intersector16HybridMoeller;
    //intersectors.intersector16 = BVH4iTriangle1Intersector16ChunkMoeller;
    //intersectors.intersector16 = BVH4iTriangle1Intersector16SingleMoeller;
    return intersectors;
  }

  Accel* BVH4i::BVH4iTriangle1(Scene* scene)
  { 
    BVH4i* accel = new BVH4i(SceneTriangle1::type);
    
    Builder* builder = NULL;
    if      (g_builder == "default"         ) builder = BVH4iBuilder::create(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "objectsplit"     ) builder = BVH4iBuilder::create(accel,&scene->flat_triangle_source_1,scene,1,inf);
    else if (g_builder == "morton"          ) builder = BVH4iBuilderMorton::create(accel,&scene->flat_triangle_source_1,scene);
    else if (g_builder == "morton.enhanced" ) builder = BVH4iBuilderMortonEnhanced::create(accel,&scene->flat_triangle_source_1,scene);
    else throw std::runtime_error("unknown builder "+g_builder+" for BVH4i<Triangle1>");
    
    Accel::Intersectors intersectors = BVH4iTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }
    
  Accel* BVH4i::BVH4iTriangle1(TriangleMeshScene::TriangleMesh* mesh)
  {
    BVH4i* accel = new BVH4i(TriangleMeshTriangle1::type);

    Builder* builder = NULL;
    if      (g_builder == "default"     ) builder = BVH4iBuilder::create(accel,mesh,mesh,1,inf);
    else if (g_builder == "objectsplit" ) builder = BVH4iBuilder::create(accel,mesh,mesh,1,inf);
    else throw std::runtime_error("unknown builder "+g_builder+" for BVH4i<Triangle1>");

    Accel::Intersectors intersectors = BVH4iTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  void BVH4i::init(size_t numNodes, size_t numPrimitives)
  {
  }

  BVH4i::~BVH4i()
  {
    if (qbvh)  os_free(qbvh,size_node);
    if (accel) os_free(accel,size_accel);
  }


  float BVH4i::sah () {
    return sah(root,bounds)/area(bounds);
  }

  float BVH4i::sah (NodeRef& node, BBox3f bounds)
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
