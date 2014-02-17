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
  DECLARE_SYMBOL(Accel::Intersector1,BVH4HairIntersector1_);

  Builder* BVH4HairBuilder_ (BVH4Hair* bvh, Scene* scene);
  
  void BVH4HairRegister () 
  {
    int features = getCPUFeatures();
    SELECT_SYMBOL_AVX_AVX2(features,BVH4HairIntersector1_);
  }

  BVH4Hair::BVH4Hair () 
    : root(emptyNode), numPrimitives(0), numVertices(0) {}

  BVH4Hair::~BVH4Hair () {
  }

  Accel::Intersectors BVH4HairIntersectors(BVH4Hair* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH4HairIntersector1_;
    intersectors.intersector4 = NULL;
    intersectors.intersector8 = NULL;
    intersectors.intersector16 = NULL;
    return intersectors;
  }
  
  Accel* BVH4Hair::BVH4HairBezier1(Scene* scene)
  { 
    BVH4Hair* accel = new BVH4Hair;
    Accel::Intersectors intersectors = BVH4HairIntersectors(accel);
    Builder* builder = BVH4HairBuilder_(accel,scene);
    return new AccelInstance(accel,builder,intersectors);
  }

  void BVH4Hair::init(size_t numPrimitives)
  {
    size_t numAllocatedNodes = numPrimitives;
    size_t numAllocatedPrimitives = numPrimitives;
#if defined(__X86_64__)
    size_t numReservedNodes = 2*numPrimitives;
    size_t numReservedPrimitives = 2*numPrimitives;
#else
    size_t numReservedNodes = 1.5*numAllocatedNodes;
    size_t numReservedPrimitives = 1.5*numAllocatedPrimitives;
#endif
    
    size_t bytesAllocated = numAllocatedNodes * sizeof(UnalignedNode) + numAllocatedPrimitives * sizeof(Bezier1);
    size_t bytesReserved  = numReservedNodes  * sizeof(UnalignedNode) + numReservedPrimitives  * sizeof(Bezier1);

    size_t blockSize = LinearAllocatorPerThread::allocBlockSize;
    bytesReserved    = (bytesReserved+blockSize-1)/blockSize*blockSize;

    root = emptyNode;
    alloc.init(bytesAllocated,bytesReserved);
  }
}
