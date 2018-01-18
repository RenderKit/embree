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

#include "../common/ray.h"
#include "bezier_curve_intersector.h"

namespace embree
{
  namespace isa
  {
    struct OrientedBezierCurve1Intersector1
    {
      __forceinline OrientedBezierCurve1Intersector1() {}

      __forceinline OrientedBezierCurve1Intersector1(const Ray& ray, const void* ptr) {}

      template<typename Epilog>
      __noinline bool intersect(Ray& ray,
                                const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3,
                                const Vec3fa& n0, const Vec3fa& n1, const Vec3fa& n2, const Vec3fa& n3,
                                const Epilog& epilog) const
      {
        STAT3(normal.trav_prims,1,1,1);
        return false;
      }
    };
  }
}
