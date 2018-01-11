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

#include "../../../common/math/vec2.h"
#include "../../../common/math/vec3.h"
#include "../../../common/math/vec4.h"

namespace embree {

__forceinline Vec3f  neg(const Vec3f& a ) { return -a; }
__forceinline Vec3fa neg(const Vec3fa& a) { return -a; }
__forceinline bool   eq (const Vec3fa& a, const Vec3fa& b) { return a == b; }
__forceinline bool   ne (const Vec3fa& a, const Vec3fa& b) { return a != b; }

// FIXME: change order of lerp arguments, then remove this function
template<typename V>
__forceinline V lerpr(float t, const V& v0, const V& v1) {
  return (1.0f-t)*v0 + t*v1;
}

// -------------------------------------------------------
// sRGB conversion functions
// -------------------------------------------------------
#define APPROXIMATE_SRGB

inline float linear_to_srgb(const float f)
{
  const float c = max(f, 0.f);
#ifdef APPROXIMATE_SRGB
  return pow(c, 1.f/2.2f);
#else
  return c <= 0.0031308f ? 12.92f*c : pow(c, 1.f/2.4f)*1.055f - 0.055f;
#endif
}

inline Vec4f linear_to_srgba(const Vec4f c)
{
  return Vec4f(linear_to_srgb(c.x),
               linear_to_srgb(c.y),
               linear_to_srgb(c.z),
               max(c.w, 0.f)); // alpha is never gamma-corrected
}

inline uint32_t linear_to_srgba8(const Vec4f c)
{
#if 1
  Vec4f l = 255.f * min(linear_to_srgba(c), Vec4f(1.f));
  return
    ((uint32_t)l.x << 0)  |
    ((uint32_t)l.y << 8)  |
    ((uint32_t)l.z << 16) |
    ((uint32_t)l.w << 24);
#else
//  TODO use ISPC's float_to_srgb8 once it is fixed (issue #1198)
  return
    (float_to_srgb8(c.x) << 0)  |
    (float_to_srgb8(c.y) << 8)  |
    (float_to_srgb8(c.z) << 16) |
    ((uint32_t)clamp(c.w, 0.f, 1.f) << 24); // alpha is never gamma-corrected
#endif
}

inline float srgb_to_linear(const float f)
{
  const float c = max(f, 0.f);
#ifdef APPROXIMATE_SRGB
  return pow(c, 2.2f);
#else
  return c <= 0.04045f ? c/12.92f : pow((c + 0.055f)/1.055f, 2.4f);
#endif
}

inline Vec4f srgba_to_linear(const Vec4f c)
{
  return Vec4f(srgb_to_linear(c.x),
               srgb_to_linear(c.y),
               srgb_to_linear(c.z),
               max(c.w, 0.f));  // alpha is never gamma-corrected
}

// TODO implement srgba8_to_linear with a 256 entry LUT

#undef APPROXIMATE_SRGB

} // namespace embree
