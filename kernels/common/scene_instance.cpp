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

#include "scene_instance.h"
#include "scene.h"

namespace embree
{
  DECLARE_SYMBOL2(RTCBoundsFunc3,InstanceBoundsFunc);
  DECLARE_SYMBOL2(AccelSet::IntersectorN,InstanceIntersectorN);

  InstanceFactory::InstanceFactory(int features)
  {
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceBoundsFunc);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,InstanceIntersectorN);
  }

  Instance::Instance (Scene* scene, Scene* object, size_t numTimeSteps) 
    : AccelSet(scene,RTC_GEOMETRY_STATIC,1,numTimeSteps), object(object)
  {
    world2local0 = one;
    for (size_t i=0; i<numTimeSteps; i++) local2world[i] = one;
    intersectors.ptr = this;
    boundsFunc3 = scene->device->instance_factory->InstanceBoundsFunc();
    boundsFuncUserPtr = nullptr;
    intersectors.intersectorN = scene->device->instance_factory->InstanceIntersectorN();
  }
  
  void Instance::setTransform(const AffineSpace3fa& xfm, size_t timeStep)
  {
    if (scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (timeStep >= numTimeSteps)
      throw_RTCError(RTC_INVALID_OPERATION,"invalid timestep");

    local2world[timeStep] = xfm;
    if (timeStep == 0) world2local0 = rcp(xfm);
  }

  void Instance::setMask (unsigned mask) 
  {
    if (scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
    Geometry::update();
  }
}
