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

#pragma once

#include "object.h"
#include "../../common/ray16.h"

namespace embree
{
  namespace isa
  {
    struct ObjectIntersector16
    {
      typedef Object Primitive;
      
      struct Precalculations {
        __forceinline Precalculations (const bool16& valid, const Ray16& ray) {}
      };
      
      static __forceinline void intersect(const bool16& valid_i, const Precalculations& pre, Ray16& ray, const Primitive& prim, Scene* scene) {
        // FIXME: add ray mask test
        prim.accel->intersect16(&valid_i,(RTCRay16&)ray,prim.item);
      }
      
      static __forceinline bool16 occluded(const bool16& valid_i, const Precalculations& pre, const Ray16& ray, const Primitive& prim, Scene* scene) 
      {
        // FIXME: add ray mask test
        prim.accel->occluded16(&valid_i,(RTCRay16&)ray,prim.item);
        return ray.geomID == 0;
      }
    };
  }
}
