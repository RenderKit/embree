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
  DECLARE_SYMBOL(Accel::Intersector4,BVH4HairBezier1Intersector4);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4HairBezier1Intersector8);

  DECLARE_SYMBOL(Accel::Intersector1,BVH4HairBezier1iIntersector1);
  DECLARE_SYMBOL(Accel::Intersector4,BVH4HairBezier1iIntersector4);
  DECLARE_SYMBOL(Accel::Intersector8,BVH4HairBezier1iIntersector8);

  DECLARE_FUNCTION_SYMBOL(Builder*   BVH4HairBuilder_  (BVH4Hair* bvh, Scene* scene));
  static                  Builder* (*BVH4HairBuilder_) (BVH4Hair* bvh, Scene* scene);

  void BVH4HairRegister () 
  {
    int features = getCPUFeatures();
    SELECT_SYMBOL_DEFAULT_AVX(features,BVH4HairBuilder_);

    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4HairBezier1Intersector1);
    SELECT_SYMBOL_DEFAULT(features,BVH4HairBezier1Intersector4);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4HairBezier1Intersector8);

    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,BVH4HairBezier1iIntersector1);
    SELECT_SYMBOL_DEFAULT(features,BVH4HairBezier1iIntersector4);
    SELECT_SYMBOL_AVX_AVX2(features,BVH4HairBezier1iIntersector8);
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
    intersectors.intersector4 = BVH4HairBezier1Intersector4;
    intersectors.intersector8 = BVH4HairBezier1Intersector8;
    intersectors.intersector16 = NULL;
    return intersectors;
  }

  Accel::Intersectors BVH4HairBezier1iIntersectors(BVH4Hair* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4HairBezier1iIntersector1;
    intersectors.intersector4 = BVH4HairBezier1iIntersector4;
    intersectors.intersector8 = BVH4HairBezier1iIntersector8;
    intersectors.intersector16 = NULL;
    return intersectors;
  }
  
  Accel* BVH4Hair::BVH4HairBezier1(Scene* scene)
  { 
    BVH4Hair* accel = new BVH4Hair(Bezier1Type::type,scene);
    Accel::Intersectors intersectors = BVH4HairBezier1Intersectors(accel);
    Builder* builder = BVH4HairBuilder_(accel,scene);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH4Hair::BVH4HairBezier1i(Scene* scene)
  { 
    scene->needVertices = true;
    BVH4Hair* accel = new BVH4Hair(SceneBezier1i::type,scene);
    Accel::Intersectors intersectors = BVH4HairBezier1iIntersectors(accel);
    Builder* builder = BVH4HairBuilder_(accel,scene);
    return new AccelInstance(accel,builder,intersectors);
  }

  void BVH4Hair::init(size_t numPrimitivesMin, size_t numPrimitivesMax)
  {
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
