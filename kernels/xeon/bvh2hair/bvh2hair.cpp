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

#include "bvh2hair.h"
#include "common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL(Accel::Intersector1,BVH2HairIntersector1_);

  Builder* BVH2HairBuilder_ (BVH2Hair* bvh, Scene* scene);
  
  void BVH2HairRegister () 
  {
    int features = getCPUFeatures();
    SELECT_SYMBOL_AVX_AVX2(features,BVH2HairIntersector1_);
  }

  BVH2Hair::BVH2Hair (Scene* scene) 
    : scene(scene), root(emptyNode), numPrimitives(0), numVertices(0) {}

  BVH2Hair::~BVH2Hair () {
  }

  Accel::Intersectors BVH2HairIntersectors(BVH2Hair* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1 = BVH2HairIntersector1_;
    intersectors.intersector4 = NULL;
    intersectors.intersector8 = NULL;
    intersectors.intersector16 = NULL;
    return intersectors;
  }
  
  Accel* BVH2Hair::BVH2HairBezier1(Scene* scene)
  { 
    BVH2Hair* accel = new BVH2Hair(scene);
    Accel::Intersectors intersectors = BVH2HairIntersectors(accel);
    Builder* builder = BVH2HairBuilder_(accel,scene);
    return new AccelInstance(accel,builder,intersectors);
  }

  void BVH2Hair::init(size_t numPrimitives)
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
