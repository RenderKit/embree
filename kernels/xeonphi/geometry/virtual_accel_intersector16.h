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

#include "../../common/accel.h"
#include "../../common/ray16.h"

namespace embree
{
  struct VirtualAccelIntersector16
  {
    typedef AccelSetItem Primitive;

    static __forceinline void intersect(const bool16& valid_i, Ray16& ray, const Primitive& prim, const void* geom) 
    {
      int16 maski = select(valid_i,int16(-1),int16(0));
      prim.accel->intersect16(&maski,(RTCRay16&)ray,prim.item);
    }

    static __forceinline void intersect(const bool16& valid, Ray16& ray, const Primitive* tri, size_t num, const void* geom)
    {
      for (size_t i=0; i<num; i++)
        intersect(valid,ray,tri[i],geom);
    }

    static __forceinline bool16 occluded(const bool16& valid_i, const Ray16& ray, const Primitive& prim, const void* geom) 
    {
      int16 maski = select(valid_i,int16(-1),int16(0));
      prim.accel->occluded16(&maski,(RTCRay16&)ray,prim.item);
      return ray.geomID == 0;
    }

    static __forceinline bool16 occluded(const bool16& valid, const Ray16& ray, const Primitive* tri, size_t num, const void* geom)
    {
      bool16 terminated = !valid;
      for (size_t i=0; i<num; i++) {
        terminated |= occluded(!terminated,ray,tri[i],geom);
        if (all(terminated)) return terminated;
      }
      return terminated;
    }
  };
}
