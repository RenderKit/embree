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

#include "scene_user_geometry.h"
#include "scene.h"

namespace embree
{
  UserGeometry::UserGeometry (Device* device, RTCGeometryFlags gflags, size_t items, size_t numTimeSteps) 
    : AccelSet(device,gflags,items,numTimeSteps) {}
  
  void UserGeometry::setUserData (void* ptr) {
    intersectors.ptr = ptr;
    Geometry::setUserData(ptr);
  }

  void UserGeometry::setMask (unsigned mask) 
  {
    if (scene && scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
    Geometry::update();
  }

  void UserGeometry::setBoundsFunction (RTCBoundsFunc bounds, void* userPtr) 
  {
    if (scene && scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->boundsFunc = bounds;
    this->boundsFuncUserPtr = userPtr;
  }

  void UserGeometry::setIntersectFunctionN (RTCIntersectFuncN intersect) 
  {
    if (scene && scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersectorN.intersect = intersect;
  }

  void UserGeometry::setOccludedFunctionN (RTCOccludedFuncN occluded) 
  {
    if (scene && scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersectorN.occluded = occluded;
  }
}
