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

#include "bvh4hair.h"
#include "common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL(Accel::Intersector1,BVH4HairBezier1Intersector1);
  DECLARE_SYMBOL(Accel::Intersector1,BVH4HairBezier1iIntersector1);

  Builder* BVH4HairBuilder_ (BVH4Hair* bvh, Scene* scene);

  DECLARE_FUNCTION_SYMBOL(Builder*   BVH4HairBuilder2_  (BVH4Hair* bvh, Scene* scene));
  static                  Builder* (*BVH4HairBuilder2_) (BVH4Hair* bvh, Scene* scene);

  void BVH4HairRegister () 
  {
    int features = getCPUFeatures();
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4HairBuilder2_);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4HairBezier1Intersector1);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4HairBezier1iIntersector1);
  }

  BVH4Hair::BVH4Hair (const PrimitiveType& primTy, Scene* scene) 
    : primTy(primTy), scene(scene), root(emptyNode), numPrimitives(0), numVertices(0) {}

  BVH4Hair::~BVH4Hair () {
  }

  Accel::Intersectors BVH4HairBezier1Intersectors(BVH4Hair* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4HairBezier1Intersector1;
    intersectors.intersector4 = NULL;
    intersectors.intersector8 = NULL;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4HairBezier1iIntersectors(BVH4Hair* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4HairBezier1iIntersector1;
    intersectors.intersector4 = NULL;
    intersectors.intersector8 = NULL;
    intersectors.intersector16 = NULL;
    return intersectors;
  }
  
  Accel* BVH4Hair::BVH4HairBezier1(Scene* scene)
  { 
    BVH4Hair* accel = new BVH4Hair(Bezier1Type::type,scene);
    Accel::Intersectors intersectors = BVH4HairBezier1Intersectors(accel);

    Builder* builder = NULL;
    if      (g_hair_builder == "builder1") builder = BVH4HairBuilder_(accel,scene);
    else if (g_hair_builder == "builder2" ) builder = BVH4HairBuilder2_(accel,scene);
    else if (g_hair_builder == "default" ) builder = BVH4HairBuilder2_(accel,scene);
    else throw std::runtime_error("unknown hair builder");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Hair::BVH4HairBezier1i(Scene* scene)
  { 
    BVH4Hair* accel = new BVH4Hair(SceneBezier1i::type,scene);
    Accel::Intersectors intersectors = BVH4HairBezier1iIntersectors(accel);

    Builder* builder = NULL;
    if      (g_hair_builder == "builder1") builder = BVH4HairBuilder_(accel,scene);
    else if (g_hair_builder == "builder2" ) builder = BVH4HairBuilder2_(accel,scene);
    else if (g_hair_builder == "default" ) builder = BVH4HairBuilder2_(accel,scene);
    else throw std::runtime_error("unknown hair builder");

    return new AccelInstance(accel,builder,intersectors);
  }

  void BVH4Hair::init(size_t numPrimitivesMin, size_t numPrimitivesMax)
  {
#if BVH4HAIR_COMPRESS_ALIGNED_NODES 
    std::cout << "BVH4HAIR_COMPRESS_ALIGNED_NODES ENABLED" << std::endl;
#endif

#if BVH4HAIR_COMPRESS_UNALIGNED_NODES 
    std::cout << "BVH4HAIR_COMPRESS_UNALIGNED_NODES ENABLED" << std::endl;
#endif

    if (numPrimitivesMax == 0) numPrimitivesMax = numPrimitivesMin;

    size_t numAllocatedNodes = numPrimitivesMin;
    size_t numAllocatedPrimitives = numPrimitivesMin;
#if defined(__X86_64__)
    size_t numReservedNodes = 2*numPrimitivesMax;
    size_t numReservedPrimitives = 2*numPrimitivesMax;
#else
    size_t numReservedNodes = 1.5*numAllocatedNodes;
    size_t numReservedPrimitives = 1.5*numAllocatedPrimitives;
#endif
    
    size_t bytesAllocated = 0; //numAllocatedNodes * sizeof(UnalignedNode) + numAllocatedPrimitives * sizeof(primTy.bytes);
    size_t bytesReserved  = numReservedNodes  * sizeof(UnalignedNode) + numReservedPrimitives  * sizeof(primTy.bytes);

    size_t blockSize = LinearAllocatorPerThread::allocBlockSize;
    bytesReserved    = (bytesReserved+blockSize-1)/blockSize*blockSize;

    root = emptyNode;
    alloc.init(bytesAllocated,bytesReserved);
  }
}
