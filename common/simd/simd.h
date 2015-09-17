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

#include "../math/math.h"

/* include SSE wrapper classes */
#if defined(__SSE__)
#  include "sse.h"
#endif

/* include AVX wrapper classes */
#if defined(__AVX__)
#include "avx.h"
#endif

/* include AVX512 wrapper classes */
#if defined (__MIC__)
#  include "avx512.h"
#endif

#if defined (__AVX__)
#if defined(__AVX512F__)
#define AVX_ZERO_UPPER()
#else
#define AVX_ZERO_UPPER() _mm256_zeroupper()
#endif
#else
#define AVX_ZERO_UPPER()
#endif

namespace embree
{
#if defined (__AVX512F__) || defined (__MIC__)
  typedef bool16 vbool;
  typedef int16 vint;
  typedef float16 vfloat;
#elif defined(__AVX__)
  typedef bool8 vbool;
  typedef int8 vint;
  typedef float8 vfloat;
#else
  typedef bool4 vbool;
  typedef int4 vint;
  typedef float4 vfloat;
#endif

  /* foreach unique */
  template<typename vbool, typename vint, typename Closure>
    __forceinline void foreach_unique(const vbool& valid0, const vint& vi, const Closure& closure) 
  {
    vbool valid1 = valid0;
    while (any(valid1)) {
      const int j = __bsf(movemask(valid1));
      const int i = vi[j];
      const vbool valid2 = valid1 & (i == vi);
      valid1 = valid1 & !valid2;
      closure(valid2,i);
    }
  }

  /* foreach unique */
  template<typename vbool, typename vint, typename Closure>
    __forceinline void foreach_unique_index(const vbool& valid0, const vint& vi, const Closure& closure) 
  {
    vbool valid1 = valid0;
    while (any(valid1)) {
      const int j = __bsf(movemask(valid1));
      const int i = vi[j];
      const vbool valid2 = valid1 & (i == vi);
      valid1 = valid1 & !valid2;
      closure(valid2,i,j);
    }
  }

  template<typename Closure>
    __forceinline void foreach2(int x0, int x1, int y0, int y1, const Closure& closure) 
  {
    __aligned(64) int U[128];
    __aligned(64) int V[128];
    int index = 0;
    for (int y=y0; y<y1; y++) {
      const bool lasty = y+1>=y1;
      const vint vy = y;
      for (int x=x0; x<x1; ) { //x+=vfloat::size) {
        const bool lastx = x+vfloat::size >= x1;
        vint vx = x+vint(step);
        vint::storeu(&U[index],vx);
        vint::storeu(&V[index],vy);
        const int dx = min(x1-x,vfloat::size);
        index += dx;
        x += dx;
        if (index >= vfloat::size || (lastx && lasty)) {
          const vbool valid = vint(step) < vint(index);
          closure(valid,vint::load(U),vint::load(V));
          x-= max(0,index-vfloat::size);
          index = 0;
        }
      }
    }
  }
}
