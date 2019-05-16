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
#include "../common/accelinstance.h"

namespace embree
{

  BVHGPUFactory::BVHGPUFactory()
  {
    int features = 0;
    selectBuilders(features);
    selectIntersectors(features);
  }

  void BVHGPUFactory::selectBuilders(int features)
  {
    
  }

  void BVHGPUFactory::selectIntersectors(int features)
  {
    
  }

  Accel* BVHGPUFactory::BVHGPUTriangle1v(Scene* scene)
  {
    BVH8* accel = nullptr;
    Accel::Intersectors intersectors; 
    Builder* builder = nullptr;
    return new AccelInstance(accel,builder,intersectors);
  }

}

