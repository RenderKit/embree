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

#include "../common/isa.h" // to define EMBREE_TARGET_SIMD8


#include "bvh_gpu_factory.h"
#include "../bvh/bvh.h"
#include "../geometry/triangle1v.h"
#include "../common/accelinstance.h"

namespace embree
{
  DECLARE_ISA_FUNCTION(Builder*,BVHGPUTriangle1vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  BVHGPUFactory::BVHGPUFactory()
  {
    int features = 0;
    selectBuilders(features);
    selectIntersectors(features);
  }

  void BVHGPUFactory::selectBuilders(int features)
  {
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT(features,BVHGPUTriangle1vSceneBuilderSAH));
  }

  void BVHGPUFactory::selectIntersectors(int features)
  {
    
  }

  Accel::Intersectors BVHGPUFactory::BVHGPUTriangle1vIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1    = NULL;
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4    = NULL;
    intersectors.intersector8    = NULL;
    intersectors.intersector16   = NULL;
    intersectors.intersectorN    = NULL;
#endif
    return intersectors;
  
  }

  Accel* BVHGPUFactory::BVHGPUTriangle1v(Scene* scene)
  {
    PING;
    BVH4* accel = new BVH4(Triangle1v::type,scene);
    Accel::Intersectors intersectors = BVHGPUTriangle1vIntersectors(accel); 
    Builder* builder = BVHGPUTriangle1vSceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }


}

