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

#include "scene_user_geometry.h"
#include "scene.h"

namespace embree
{
  UserGeometry::UserGeometry (Scene* parent, size_t items) 
    : AccelSet(parent,items) {}
  
  void UserGeometry::setUserData (void* ptr) {
    intersectors.ptr = ptr;
    Geometry::setUserData(ptr);
  }

  void UserGeometry::setBoundsFunction (RTCBoundsFunc bounds) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->boundsFunc = bounds;
  }

  void UserGeometry::setIntersectFunction (RTCIntersectFunc intersect1, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersector1.intersect = intersect1;
  }

  void UserGeometry::setIntersectFunction4 (RTCIntersectFunc4 intersect4, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersector4.intersect = (void*)intersect4;
    intersectors.intersector4.ispc = ispc;
  }

  void UserGeometry::setIntersectFunction8 (RTCIntersectFunc8 intersect8, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersector8.intersect = (void*)intersect8;
    intersectors.intersector8.ispc = ispc;
  }

  void UserGeometry::setIntersectFunction16 (RTCIntersectFunc16 intersect16, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersector16.intersect = (void*)intersect16;
    intersectors.intersector16.ispc = ispc;
  }

  void UserGeometry::setOccludedFunction (RTCOccludedFunc occluded1, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersector1.occluded = occluded1;
  }

  void UserGeometry::setOccludedFunction4 (RTCOccludedFunc4 occluded4, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersector4.occluded = (void*)occluded4;
    intersectors.intersector4.ispc = ispc;
  }

  void UserGeometry::setOccludedFunction8 (RTCOccludedFunc8 occluded8, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersector8.occluded = (void*)occluded8;
    intersectors.intersector8.ispc = ispc;
  }

  void UserGeometry::setOccludedFunction16 (RTCOccludedFunc16 occluded16, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    intersectors.intersector16.occluded = (void*)occluded16;
    intersectors.intersector16.ispc = ispc;
  }
}
