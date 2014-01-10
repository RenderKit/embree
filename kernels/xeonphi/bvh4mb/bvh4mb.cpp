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

#include "kernels/xeonphi/bvh4mb/bvh4mb.h"
#include "geometry/triangle1.h"
#include "common/accelinstance.h"

namespace embree
{

  //DECLARE_SYMBOL(Accel::Intersector1,BVH4mbTriangle1Intersector1);
  //DECLARE_SYMBOL(Accel::Intersector1,BVH4mbTriangle1Intersector16HybridMoeller);

  void BVH4mbRegister () 
  {
    int features = getCPUFeatures();

    /* default target */
    //SELECT_SYMBOL_KNC(features,BVH4mbTriangle1Intersector1);
    //SELECT_SYMBOL_KNC(features,BVH4mbTriangle1Intersector16HybridMoeller);
    //SELECT_SYMBOL_KNC(features,BVH4mbVirtualGeometryIntersector1);
    //SELECT_SYMBOL_KNC(features,BVH4mbVirtualGeometryIntersector16);
  }


#if 0
  Accel::Intersectors BVH4mbTriangle1Intersectors(BVH4mb* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4mbTriangle1Intersector1;
    if      (g_traverser == "default") intersectors.intersector16 = BVH4mbTriangle1Intersector16HybridMoeller;
    else if (g_traverser == "hybrid" ) intersectors.intersector16 = BVH4mbTriangle1Intersector16HybridMoeller;
    else if (g_traverser == "chunk"  ) intersectors.intersector16 = BVH4mbTriangle1Intersector16ChunkMoeller;
    else if (g_traverser == "single" ) intersectors.intersector16 = BVH4mbTriangle1Intersector16SingleMoeller;
    else if (g_traverser == "scalar" ) 
      {
	intersectors.intersector1  = BVH4mbTriangle1Intersector1Scalar;
	intersectors.intersector16 = BVH4mbTriangle1Intersector16SingleMoeller;
      }
    else throw std::runtime_error("unknown traverser "+g_traverser+" for BVH4mb<Triangle1>");      
    return intersectors;
  }


  Accel::Intersectors BVH4mbVirtualGeometryIntersectors(BVH4mb* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4mbVirtualGeometryIntersector1;
    intersectors.intersector16 = BVH4mbVirtualGeometryIntersector16;
    return intersectors;
  }

  Accel* BVH4mb::BVH4mbTriangle1ObjectSplitBinnedSAH(Scene* scene)
  { 
    BVH4mb* accel = new BVH4mb(SceneTriangle1::type);   
    Builder* builder = BVH4mbBuilder::create(accel,&scene->flat_triangle_source_1,scene,BVH4mbBuilder::BVH4MB_BUILDER_DEFAULT);    
    Accel::Intersectors intersectors = BVH4mbTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4mb::BVH4mbTriangle1ObjectSplitMorton(Scene* scene)
  { 
    BVH4mb* accel = new BVH4mb(SceneTriangle1::type);   
    Builder* builder = BVH4mbBuilderMorton::create(accel,&scene->flat_triangle_source_1,scene);  
    Accel::Intersectors intersectors = BVH4mbTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4mb::BVH4mbTriangle1ObjectSplitEnhancedMorton(Scene* scene)
  { 
    BVH4mb* accel = new BVH4mb(SceneTriangle1::type);
    
    Builder* builder = BVH4mbBuilderMortonEnhanced::create(accel,&scene->flat_triangle_source_1,scene);
    
    Accel::Intersectors intersectors = BVH4mbTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4mb::BVH4mbTriangle1PreSplitsBinnedSAH(Scene* scene)
  {
    BVH4mb* accel = new BVH4mb(SceneTriangle1::type);
    
    Builder* builder = BVH4mbBuilder::create(accel,&scene->flat_triangle_source_1,scene,BVH4mbBuilder::BVH4MB_BUILDER_PRESPLITS);
    
    Accel::Intersectors intersectors = BVH4mbTriangle1Intersectors(accel);
    return new AccelInstance(accel,builder,intersectors);    
  }

  Accel* BVH4mb::BVH4mbVirtualGeometryBinnedSAH(Scene* scene)
  {
    BVH4mb* accel = new BVH4mb(SceneTriangle1::type);    
    Builder* builder = BVH4mbBuilder::create(accel,NULL,scene,BVH4mbBuilder::BVH4MB_BUILDER_VIRTUAL_GEOMETRY);   
    Accel::Intersectors intersectors = BVH4mbVirtualGeometryIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);    
  }
#endif

  BVH4mb::~BVH4mb()
  {
    if (qbvh)  os_free(qbvh,size_node);
    if (accel) os_free(accel,size_accel);
  }


  float BVH4mb::sah () {
    return sah(root,bounds)/area(bounds);
  }

  float BVH4mb::sah (NodeRef& node, BBox3f bounds)
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
