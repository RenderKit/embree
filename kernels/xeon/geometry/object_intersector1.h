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
#include "../../common/ray.h"

namespace embree
{
  namespace isa
  {
    struct ObjectIntersector1
    {
      typedef Object Primitive;
      
      struct Precalculations {
        __forceinline Precalculations (const Ray& ray, const void *ptr) {}
      };
      
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID) 
      {
        AVX_ZERO_UPPER();
        AccelSet* accel = (AccelSet*) scene->get(prim.geomID);

        /* perform ray mask test */
#if defined(RTCORE_RAY_MASK)
        if ((ray.mask & accel->mask) == 0) 
          return;
#endif

        accel->intersect((RTCRay&)ray,prim.primID);
      }
      
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID) 
      {
        AVX_ZERO_UPPER();
        AccelSet* accel = (AccelSet*) scene->get(prim.geomID);

        /* perform ray mask test */
#if defined(RTCORE_RAY_MASK)
        if ((ray.mask & accel->mask) == 0) 
          return false;
#endif

        accel->occluded((RTCRay&)ray,prim.primID);
        return ray.geomID == 0;
      }
    };
  }
}
