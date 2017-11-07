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

#pragma once

#include "accelset.h"

namespace embree
{
  /*! User geometry with user defined intersection functions */
  struct UserGeometry : public AccelSet
  {
  public:
    UserGeometry (Device* device, RTCGeometryFlags gflags, unsigned int items, unsigned int numTimeSteps); 
    virtual void setUserData (void* ptr);
    virtual void setMask (unsigned mask);
    virtual void setBoundsFunction (RTCBoundsFunction bounds, void* userPtr);
    virtual void setIntersectFunctionN (RTCIntersectFunctionN intersect);
    virtual void setOccludedFunctionN (RTCOccludedFunctionN occluded);
    virtual void build() {}
  };
}
