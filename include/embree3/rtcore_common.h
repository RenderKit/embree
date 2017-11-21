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

#include <stddef.h>
#include <sys/types.h>
#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(_WIN32)
#if defined(_M_X64)
typedef long long ssize_t;
#else
typedef int ssize_t;
#endif
#endif

#ifndef RTCORE_API
#if defined(_WIN32) && !defined(EMBREE_STATIC_LIB)
#  define RTCORE_API __declspec(dllimport) 
#else
#  define RTCORE_API 
#endif
#endif

#ifdef _WIN32
#  define RTCORE_ALIGN(...) __declspec(align(__VA_ARGS__))
#else
#  define RTCORE_ALIGN(...) __attribute__((aligned(__VA_ARGS__)))
#endif

#if !defined (RTCORE_DEPRECATED)
#ifdef __GNUC__
  #define RTCORE_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
  #define RTCORE_DEPRECATED __declspec(deprecated)
#else
  #define RTCORE_DEPRECATED
#endif
#endif

#if defined(_WIN32) 
#  define RTCORE_FORCEINLINE __forceinline
#else
#  define RTCORE_FORCEINLINE inline __attribute__((always_inline))
#endif

/*! Axis aligned bounding box representation */
struct RTCORE_ALIGN(16) RTCBounds
{
  float lower_x, lower_y, lower_z, align0;
  float upper_x, upper_y, upper_z, align1;
};

#if defined(__cplusplus)
}
#endif

