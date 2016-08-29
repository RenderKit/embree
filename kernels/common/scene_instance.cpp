// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
  DECLARE_SYMBOL2(RTCBoundsFunc2,InstanceBoundsFunc);
  DECLARE_SYMBOL2(AccelSet::Intersector1,InstanceIntersector1);
  DECLARE_SYMBOL2(AccelSet::Intersector4,InstanceIntersector4);
  DECLARE_SYMBOL2(AccelSet::Intersector8,InstanceIntersector8);
  DECLARE_SYMBOL2(AccelSet::Intersector16,InstanceIntersector16);
  DECLARE_SYMBOL2(AccelSet::Intersector1M,InstanceIntersector1M);

  InstanceFactory::InstanceFactory(int features)
  {
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceBoundsFunc);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceIntersector1);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceIntersector1M);
#if defined (EMBREE_RAY_PACKETS)
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceIntersector4);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,InstanceIntersector8);
    SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,InstanceIntersector16);
#endif
  }

  Instance::Instance (Scene* parent, Scene* object, size_t numTimeSteps) 
    : AccelSet(parent,1,numTimeSteps), object(object)
  {
    local2world[0] = local2world[1] = one;
    world2local[0] = world2local[1] = one;
    intersectors.ptr = this;
    boundsFunc2 = parent->device->instance_factory->InstanceBoundsFunc;
    intersectors.intersector1 = parent->device->instance_factory->InstanceIntersector1;
    intersectors.intersector4 = parent->device->instance_factory->InstanceIntersector4; 
    intersectors.intersector8 = parent->device->instance_factory->InstanceIntersector8; 
    intersectors.intersector16 = parent->device->instance_factory->InstanceIntersector16;
    intersectors.intersector1M = parent->device->instance_factory->InstanceIntersector1M;
  }
  
  void Instance::setTransform(const AffineSpace3fa& xfm, size_t timeStep)
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (timeStep >= numTimeSteps)
      throw_RTCError(RTC_INVALID_OPERATION,"invalid timestep");

    local2world[timeStep] = xfm;
    world2local[timeStep] = rcp(xfm);
  }

  void Instance::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
    Geometry::update();
  }
}
