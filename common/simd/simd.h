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
#define AVX_ZERO_UPPER() _mm256_zeroupper()
#else
#define AVX_ZERO_UPPER()
#endif

/* foreach unique */
namespace embree
{
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
}
