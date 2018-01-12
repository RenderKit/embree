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
  DECLARE_SYMBOL2(RTCBoundsFunction,InstanceBoundsFunc);
  DECLARE_SYMBOL2(AccelSet::IntersectorN,InstanceIntersectorN);

  InstanceFactory::InstanceFactory(int features)
  {
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceBoundsFunc);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,InstanceIntersectorN);
  }

  Instance::Instance (Device* device, Scene* object, unsigned int numTimeSteps) 
    : AccelSet(device,1,numTimeSteps), object(object), local2world(nullptr)
  {
    if (object) object->refInc();
    world2local0 = one;
    local2world = (AffineSpace3fa*) alignedMalloc(numTimeSteps*sizeof(AffineSpace3fa));
    for (size_t i = 0; i < numTimeSteps; i++)
      local2world[i] = one;
    intersectors.ptr = this;
    boundsFunc = device->instance_factory->InstanceBoundsFunc();
    intersectors.intersectorN = device->instance_factory->InstanceIntersectorN();
  }

  Instance::~Instance()
  {
    alignedFree(local2world);
    if (object) object->refDec();
  }

  void Instance::setNumTimeSteps (unsigned int numTimeSteps_in)
  {
    if (numTimeSteps_in == numTimeSteps)
      return;
    
    AffineSpace3fa* local2world2 = (AffineSpace3fa*) alignedMalloc(numTimeSteps_in*sizeof(AffineSpace3fa));
     
    for (size_t i = 0; i < min(numTimeSteps, numTimeSteps_in); i++)
      local2world2[i] = local2world[i];

    for (size_t i = numTimeSteps; i < numTimeSteps_in; i++)
      local2world2[i] = one;
        
    alignedFree(local2world);
    local2world = local2world2;
    
    Geometry::setNumTimeSteps(numTimeSteps_in);
  }

  void Instance::setInstancedScene(const Ref<Scene>& scene)
  {
    if (object) object->refDec();
    object = scene.ptr;
    if (object) object->refInc();
    Geometry::update();
  }
  
  void Instance::setTransform(const AffineSpace3fa& xfm, unsigned int timeStep)
  {
    if (timeStep >= numTimeSteps)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid timestep");

    local2world[timeStep] = xfm;
    if (timeStep == 0)
      world2local0 = rcp(xfm);
  }

  AffineSpace3fa Instance::getTransform(float time)
  {
    return getWorld2Local(time);
  }
  
  void Instance::setMask (unsigned mask) 
  {
    this->mask = mask; 
    Geometry::update();
  }
}
