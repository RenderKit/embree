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

#include "sse.h"

namespace embree 
{
  struct bool8;
  struct int8;
  struct float8;
}

#include "bool8_avx.h"
#if defined (__AVX2__)
#include "int8_avx2.h"
#else
#include "int8_avx.h"
#endif
#include "float8_avx.h"

#if defined (__AVX512F__)
#include "avx512.h"
#endif

