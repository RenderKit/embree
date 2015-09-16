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

#include "bvh8.h"
#include "bvh8_statistics.h"
#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/triangle8v.h"
#include "../geometry/trianglepairs8.h"
#include "../../common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8Triangle4Intersector8ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);

  DECLARE_SYMBOL(Accel::Intersector1,BVH8Triangle8Intersector1Moeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8Triangle8Intersector8ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoellerNoFilter);

  //DECLARE_SYMBOL(Accel::Intersector1,BVH8Triangle8vIntersector1Pluecker);
  //DECLARE_SYMBOL(Accel::Intersector4,BVH8Triangle8vIntersector4HybridPluecker);
  //DECLARE_SYMBOL(Accel::Intersector4,BVH8Triangle8vIntersector4HybridPlueckerNoFilter);
  //DECLARE_SYMBOL(Accel::Intersector8,BVH8Triangle8vIntersector8HybridPluecker);
  //DECLARE_SYMBOL(Accel::Intersector8,BVH8Triangle8vIntersector8HybridPlueckerNoFilter);

  DECLARE_SYMBOL(Accel::Intersector1,BVH8TrianglePairs8Intersector1Moeller);

  DECLARE_SYMBOL(Accel::Intersector16,BVH8Triangle4Intersector16ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL(Accel::Intersector16,BVH8Triangle8Intersector16ChunkMoeller);
  DECLARE_SYMBOL(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoeller);
  DECLARE_SYMBOL(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoellerNoFilter);
  //DECLARE_SYMBOL(Accel::Intersector16,BVH8Triangle8vIntersector16HybridPluecker);
  //DECLARE_SYMBOL(Accel::Intersector16,BVH8Triangle8vIntersector16HybridPlueckerNoFilter);

  DECLARE_BUILDER(void,Scene,size_t,BVH8Triangle4SceneBuilderSAH);
  DECLARE_BUILDER(void,Scene,size_t,BVH8Triangle8SceneBuilderSAH);
  DECLARE_BUILDER(void,Scene,size_t,BVH8TrianglePairs8SceneBuilderSAH);
  //DECLARE_BUILDER(void,Scene,size_t,BVH8Triangle8vSceneBuilderSAH);

  DECLARE_BUILDER(void,Scene,size_t,BVH8Triangle4SceneBuilderSpatialSAH);
  DECLARE_BUILDER(void,Scene,size_t,BVH8Triangle8SceneBuilderSpatialSAH);
  //DECLARE_BUILDER(void,Scene,size_t,BVH8Triangle8vSceneBuilderSpatialSAH);

  void BVH8Register () 
  {
    int features = getCPUFeatures();

    /* select builders */
    SELECT_SYMBOL_AVX(features,BVH8Triangle4SceneBuilderSAH);
    SELECT_SYMBOL_AVX(features,BVH8Triangle8SceneBuilderSAH);
    //SELECT_SYMBOL_AVX(features,BVH8Triangle8vSceneBuilderSAH);
    SELECT_SYMBOL_AVX(features,BVH8TrianglePairs8SceneBuilderSAH);
    
    SELECT_SYMBOL_AVX(features,BVH8Triangle4SceneBuilderSpatialSAH);
    SELECT_SYMBOL_AVX(features,BVH8Triangle8SceneBuilderSpatialSAH);
    //SELECT_SYMBOL_AVX(features,BVH8Triangle8vSceneBuilderSpatialSAH);
 
    /* select intersectors1 */
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle4Intersector1Moeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8Intersector1Moeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8TrianglePairs8Intersector1Moeller);
    //SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8vIntersector1Pluecker);

#if defined (RTCORE_RAY_PACKETS)

    /* select intersectors4 */
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle4Intersector4HybridMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle4Intersector4HybridMoellerNoFilter);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8Intersector4HybridMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8Intersector4HybridMoellerNoFilter);
    //SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8vIntersector4HybridPluecker);
    //SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8vIntersector4HybridPlueckerNoFilter);

    /* select intersectors8 */
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle4Intersector8ChunkMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle4Intersector8HybridMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle4Intersector8HybridMoellerNoFilter);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8Intersector8ChunkMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8Intersector8HybridMoeller);
    SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8Intersector8HybridMoellerNoFilter);
    //SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8vIntersector8HybridPluecker);
    //SELECT_SYMBOL_AVX_AVX2(features,BVH8Triangle8vIntersector8HybridPlueckerNoFilter);

    /* select intersectors16 */
    SELECT_SYMBOL_AVX512(features,BVH8Triangle4Intersector16ChunkMoeller);
    SELECT_SYMBOL_AVX512(features,BVH8Triangle4Intersector16HybridMoeller);
    SELECT_SYMBOL_AVX512(features,BVH8Triangle4Intersector16HybridMoellerNoFilter);

    SELECT_SYMBOL_AVX512(features,BVH8Triangle8Intersector16ChunkMoeller);
    SELECT_SYMBOL_AVX512(features,BVH8Triangle8Intersector16HybridMoeller);
    SELECT_SYMBOL_AVX512(features,BVH8Triangle8Intersector16HybridMoellerNoFilter);
    //SELECT_SYMBOL_AVX512(features,BVH8Triangle8vIntersector16HybridPluecker);
    //SELECT_SYMBOL_AVX512(features,BVH8Triangle8vIntersector16HybridPlueckerNoFilter);
#endif
  }

  BVH8::BVH8 (const PrimitiveType& primTy, Scene* scene)
    : AccelData(AccelData::TY_BVH8), alloc2(scene->device), primTy(primTy), device(scene->device), scene(scene), root(emptyNode),
      numPrimitives(0), numVertices(0) {}

  BVH8::~BVH8 () {
    for (size_t i=0; i<objects.size(); i++) 
      delete objects[i];
  }

  void BVH8::clear() 
  {
    set(BVH8::emptyNode,empty,0);
    alloc2.clear();
  }
  
  void BVH8::set (NodeRef root, const BBox3fa& bounds, size_t numPrimitives)
  {
    this->root = root;
    this->bounds = bounds;
    this->numPrimitives = numPrimitives;
  }
  
   void BVH8::printStatistics()
   {
     std::cout << BVH8Statistics(this).str();
     std::cout << "  "; alloc2.print_statistics();
   }	
  
  void BVH8::clearBarrier(NodeRef& node)
  {
    if (node.isBarrier())
      node.clearBarrier();
    else if (!node.isLeaf()) {
      Node* n = node.node();
      for (size_t c=0; c<N; c++)
        clearBarrier(n->child(c));
    }
  }

  void BVH8::layoutLargeNodes(size_t N)
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
      for (size_t i=0; i<BVH8::N; i++) {
        if (node->child(i) == BVH8::emptyNode) continue;
        lst.push_back(NodeArea(node->child(i),node->bounds(i)));
        std::push_heap(lst.begin(), lst.end());
      }
    }

    for (size_t i=0; i<lst.size(); i++)
      lst[i].node->setBarrier();
      
    root = layoutLargeNodesRecursion(root);
  }
  
  BVH8::NodeRef BVH8::layoutLargeNodesRecursion(NodeRef& node)
  {
    if (node.isBarrier()) {
      node.clearBarrier();
      return node;
    }
    else if (node.isNode()) 
    {
      Node* oldnode = node.node();
      Node* newnode = (BVH8::Node*) alloc2.threadLocal2()->alloc0.malloc(sizeof(BVH8::Node)); // FIXME: optimize access to threadLocal2 
      *newnode = *oldnode;
      for (size_t c=0; c<BVH8::N; c++)
        newnode->child(c) = layoutLargeNodesRecursion(oldnode->child(c));
      return encodeNode(newnode);
    }
    else return node;
  }

  Accel::Intersectors BVH8Triangle4Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle4Intersector1Moeller;
    intersectors.intersector4_filter    = BVH8Triangle4Intersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH8Triangle4Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH8Triangle4Intersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH8Triangle4Intersector8HybridMoellerNoFilter;
    intersectors.intersector16_filter   = BVH8Triangle4Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Triangle4Intersector16HybridMoellerNoFilter;
    return intersectors;
  }

  Accel::Intersectors BVH8Triangle8Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle8Intersector1Moeller;
    intersectors.intersector4_filter    = BVH8Triangle8Intersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH8Triangle8Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH8Triangle8Intersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH8Triangle8Intersector8HybridMoellerNoFilter;    
    intersectors.intersector16_filter   = BVH8Triangle8Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Triangle8Intersector16HybridMoellerNoFilter;
    return intersectors;
  }

  /*Accel::Intersectors BVH8Triangle8vIntersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle8vIntersector1Pluecker;
    intersectors.intersector4_filter    = BVH8Triangle8vIntersector4HybridPluecker;
    intersectors.intersector4_nofilter  = BVH8Triangle8vIntersector4HybridPlueckerNoFilter;
    intersectors.intersector8_filter    = BVH8Triangle8vIntersector8HybridPluecker;
    intersectors.intersector8_nofilter  = BVH8Triangle8vIntersector8HybridPlueckerNoFilter;
    intersectors.intersector16_filter   = BVH8Triangle8vIntersector16HybridPluecker;
    intersectors.intersector16_nofilter = BVH8Triangle8vIntersector16HybridPlueckerNoFilter;
    return intersectors;
    }*/

  Accel::Intersectors BVH8TrianglePairs8Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle8Intersector1Moeller;
    intersectors.intersector4_filter    = nullptr;
    intersectors.intersector4_nofilter  = nullptr;
    intersectors.intersector8_filter    = nullptr;
    intersectors.intersector8_nofilter  = nullptr;
    intersectors.intersector16_filter   = nullptr;
    intersectors.intersector16_nofilter = nullptr;
    return intersectors;
  }

  Accel* BVH8::BVH8Triangle4(Scene* scene)
  { 
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);
    
    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     )          builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2" )          builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2_spatial" )  builder = BVH8Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2_presplit" ) builder = BVH8Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else THROW_RUNTIME_ERROR("unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4>");
    
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8::BVH8Triangle4ObjectSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);
    Builder* builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8::BVH8Triangle4SpatialSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);
    Builder* builder = BVH8Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8::BVH8Triangle8(Scene* scene)
  { 
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);
    
    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     )          builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2" )          builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2_spatial" )  builder = BVH8Triangle8SceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2_presplit" ) builder = BVH8Triangle8SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else THROW_RUNTIME_ERROR("unknown builder "+scene->device->tri_builder+" for BVH8<Triangle8>");
    
    return new AccelInstance(accel,builder,intersectors);
  }


  /*Accel* BVH8::BVH8Triangle8v(Scene* scene)
  { 
    BVH8* accel = new BVH8(Triangle8v::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8vIntersectors(accel);
    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     )          builder = BVH8Triangle8vSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2" )          builder = BVH8Triangle8vSceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2_spatial" )  builder = BVH8Triangle8vSceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "binned_sah2_presplit" ) builder = BVH8Triangle8vSceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else 
      THROW_RUNTIME_ERROR("unknown builder "+scene->device->tri_builder+" for BVH8<Triangle8v>");    
    return new AccelInstance(accel,builder,intersectors);
    }*/

  Accel* BVH8::BVH8TrianglePairs8(Scene* scene)
  { 
    BVH8* accel = new BVH8(TrianglePairs8::type,scene);
    Accel::Intersectors intersectors = BVH8TrianglePairs8Intersectors(accel);
    
    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     ) builder = BVH8TrianglePairs8SceneBuilderSAH(accel,scene,0);
    else THROW_RUNTIME_ERROR("unknown builder "+scene->device->tri_builder+" for BVH8<Triangle8>");
    
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8::BVH8Triangle8ObjectSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);
    Builder* builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8::BVH8Triangle8SpatialSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);
    Builder* builder = BVH8Triangle8SceneBuilderSpatialSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  /*Accel* BVH8::BVH8Triangle8vObjectSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8v::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8vIntersectors(accel);
    Builder* builder = BVH8Triangle8vSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
    }*/

  /*Accel* BVH8::BVH8Triangle8vSpatialSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8v::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8vIntersectors(accel);
    Builder* builder = BVH8Triangle8vSceneBuilderSpatialSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
    }*/

}

