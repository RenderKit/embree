// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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
  extern RTCBoundsFunc InstanceBoundsFunc;
  extern AccelSet::Intersector1 InstanceIntersector1;
  extern AccelSet::Intersector4 InstanceIntersector4;
  extern AccelSet::Intersector8 InstanceIntersector8;
  extern AccelSet::Intersector16 InstanceIntersector16;

  Instance::Instance (Scene* parent, Accel* object) 
    : AccelSet(parent,1), local2world(one), world2local(one), object(object)
  {
    intersectors.ptr = this;
    boundsFunc = InstanceBoundsFunc;
    intersectors.intersector1 = InstanceIntersector1;
    intersectors.intersector4 = InstanceIntersector4; 
    intersectors.intersector8 = InstanceIntersector8; 
    intersectors.intersector16 = InstanceIntersector16;
  }
  
  void Instance::setTransform(const AffineSpace3fa& xfm)
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    local2world = xfm;
    world2local = rcp(xfm);
  }
}
