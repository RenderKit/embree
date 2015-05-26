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
#include "../../common/ray4.h"

namespace embree
{
  namespace isa
  {
    struct ObjectIntersector4
    {
      typedef Object Primitive;
      
      struct Precalculations {
        __forceinline Precalculations (const bool4& valid, const Ray4& ray) {}
      };
      
      static __forceinline void intersect(const bool4& valid_i, const Precalculations& pre, Ray4& ray, const Primitive& prim, Scene* scene) 
      {
        AVX_ZERO_UPPER();
        // FIXME: add ray mask test
        prim.accel->intersect4(&valid_i,(RTCRay4&)ray,prim.item);
      }
      
      static __forceinline bool4 occluded(const bool4& valid_i, const Precalculations& pre, const Ray4& ray, const Primitive& prim, Scene* scene) 
      {
        AVX_ZERO_UPPER();
        // FIXME: add ray mask test
        prim.accel->occluded4(&valid_i,(RTCRay4&)ray,prim.item);
        return ray.geomID == 0;
      }
    };
  }
}
