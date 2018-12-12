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

#include "../common/geometry.h"
#include "../common/ray.h"

namespace embree
{
  namespace isa
  {
    struct SpherePrecalculations1
    {
      float one_over_raydir2;

      __forceinline SpherePrecalculations1() {}

      __forceinline SpherePrecalculations1(const Ray& ray, const void* ptr)
      {
        one_over_raydir2 = rcp(dot(ray.dir, ray.dir));
      }
    };

    template<int K>
    struct SpherePrecalculationsK
    {
      vfloat<K> one_over_raydir2;

      __forceinline SpherePrecalculationsK(const vbool<K>& valid, const RayK<K>& ray)
      {
        one_over_raydir2 = rsqrt(dot(ray.dir, ray.dir));
      }
    };

    struct DiscPrecalculations1
    {
      __forceinline DiscPrecalculations1() {}

      __forceinline DiscPrecalculations1(const Ray& ray, const void* ptr) {}
    };

    template<int K>
    struct DiscPrecalculationsK
    {
      __forceinline DiscPrecalculationsK(const vbool<K>& valid, const RayK<K>& ray) {}
    };
  }  // namespace isa
}  // namespace embree
