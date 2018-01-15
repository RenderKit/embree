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

#pragma once

#include "default.h"
#include "rtcore.h"

namespace embree
{
  class Scene;

  struct IntersectContext
  {
  public:
    __forceinline IntersectContext(Scene* scene, const RTCIntersectContext* user_context)
      : scene(scene), user(user_context), flags(0), geomID_to_instID(nullptr) {}

  public:
    Scene* scene;
    const RTCIntersectContext* user;
    size_t flags;
    const unsigned* geomID_to_instID; // required for xfm node handling
    unsigned instID; // required for xfm node handling
    unsigned geomID; // required for xfm node handling
  };
}
