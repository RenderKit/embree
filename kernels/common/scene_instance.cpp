// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "scene_instance.h"
#include "scene.h"

namespace embree
{
  DECLARE_SYMBOL2(RTCBoundsFunction,InstanceBoundsFunc);
  DECLARE_SYMBOL2(AccelSet::IntersectorN,InstanceIntersectorN);

  InstanceFactory::InstanceFactory(int features)
  {
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceBoundsFunc);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,InstanceIntersectorN);
  }

  Instance::Instance (Device* device, Scene* object, unsigned int numTimeSteps) 
    : AccelSet(device,1,numTimeSteps), object(object)
  {
    object->refInc();
    world2local0 = one;
    for (unsigned int i=0; i<numTimeSteps; i++) local2world[i] = one;
    intersectors.ptr = this;
    boundsFunc = device->instance_factory->InstanceBoundsFunc();
    //boundsFuncUserPtr = nullptr;
    intersectors.intersectorN = device->instance_factory->InstanceIntersectorN();
  }

  Instance::~Instance() {
    object->refDec();
  }
  
  void Instance::setTransform(const AffineSpace3fa& xfm, unsigned int timeStep)
  {
    if (timeStep >= numTimeSteps)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid timestep");

    local2world[timeStep] = xfm;
    if (timeStep == 0) world2local0 = rcp(xfm);
  }

  void Instance::setMask (unsigned mask) 
  {
    this->mask = mask; 
    Geometry::update();
  }
}
