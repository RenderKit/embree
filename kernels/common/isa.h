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

namespace embree
{
namespace isa
{
#define DEFINE_SYMBOL2(type,name)               \
  typedef type (*name##Func)();                 \
  name##Func name;
  
#define DECLARE_SYMBOL2(type,name)                                       \
  extern type name();                                                   \
  void name##_error2() { throw_RTCError(RTC_UNKNOWN_ERROR,"internal error in ISA selection for " TOSTRING(name)); } \
  type name##_error() { return type(name##_error2); }                 \
  type name##_zero() { return type(nullptr); }
  
#define DEFINE_BUILDER2(Accel,Mesh,Args,symbol)                         \
  typedef Builder* (*symbol##Func)(Accel* accel, Mesh* mesh, Args args); \
  symbol##Func symbol;

#define DECLARE_BUILDER2(Accel,Mesh,Args,symbol)                         \
  extern Builder* symbol(Accel* accel, Mesh* scene, Args args);         \
  void symbol##_error() { throw_RTCError(RTC_UNSUPPORTED_CPU,"builder " TOSTRING(symbol) " not supported by your CPU"); } \

#define ZERO_SYMBOL(features,intersector)                      \
  intersector = intersector##_zero;

#define INIT_SYMBOL(features,intersector)                      \
  intersector = decltype(intersector)(intersector##_error);

#define SELECT_SYMBOL_DEFAULT(features,intersector) \
  intersector = embree::isa::intersector;

#if defined(__SSE42__)
#define SELECT_SYMBOL_SSE42(features,intersector) \
  intersector = embree::isa::intersector;
#else
#define SELECT_SYMBOL_SSE42(features,intersector)
#endif

#if defined(__AVX__)
#define SELECT_SYMBOL_AVX(features,intersector) \
  intersector = embree::isa::intersector;
#else
#define SELECT_SYMBOL_AVX(features,intersector)
#endif

#if defined(__AVX512F__)
#define SELECT_SYMBOL_AVX512(features,intersector) \
  intersector = embree::isa::intersector;
#else
#define SELECT_SYMBOL_AVX512(features,intersector)
#endif

#if defined(__SSE__)
#if !defined(__TARGET_SIMD4__)
#define __TARGET_SIMD4__
#endif
#endif

#if defined(__TARGET_AVX__)
#if !defined(__TARGET_SIMD8__)
#define __TARGET_SIMD8__
#endif
#endif

#if defined(__TARGET_AVX2__)
#if !defined(__TARGET_SIMD8__)
#define __TARGET_SIMD8__
#endif
#endif

#if defined(__TARGET_AVX512KNL__)
#if !defined(__TARGET_SIMD16__)
#define __TARGET_SIMD16__
#endif
#endif

#if defined(__TARGET_AVX512SKX__)
#if !defined(__TARGET_SIMD16__)
#define __TARGET_SIMD16__
#endif
#endif

#define SELECT_SYMBOL_DEFAULT_SSE42(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);
  
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);
  
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);

#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX512SKX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);

#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);

#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512SKX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);

#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);

#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512SKX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);

#define SELECT_SYMBOL_DEFAULT_AVX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);          \
  
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);
  
#define SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);

#define SELECT_SYMBOL_DEFAULT_AVX_AVX512KNL_AVX512SKX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);

#define SELECT_SYMBOL_DEFAULT_AVX_AVX512SKX(features,intersector) \
  SELECT_SYMBOL_DEFAULT(features,intersector);
  
#define SELECT_SYMBOL_INIT_AVX(features,intersector) \
  INIT_SYMBOL(features,intersector);                 \
  SELECT_SYMBOL_AVX(features,intersector);                                
  
#define SELECT_SYMBOL_INIT_AVX_AVX2(features,intersector) \
  INIT_SYMBOL(features,intersector);                      \
  SELECT_SYMBOL_AVX(features,intersector);

#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX512SKX(features,intersector) \
  INIT_SYMBOL(features,intersector);                                \
  SELECT_SYMBOL_AVX(features,intersector);

#define SELECT_SYMBOL_INIT_SSE42_AVX_AVX2(features,intersector) \
  INIT_SYMBOL(features,intersector);                            \
  SELECT_SYMBOL_SSE42(features,intersector);
  
#define SELECT_SYMBOL_INIT_AVX_AVX512KNL(features,intersector) \
  INIT_SYMBOL(features,intersector);                           \
  SELECT_SYMBOL_AVX(features,intersector);

#define SELECT_SYMBOL_INIT_AVX_AVX512KNL_AVX512SKX(features,intersector) \
  INIT_SYMBOL(features,intersector);                                     \
  SELECT_SYMBOL_AVX(features,intersector);

#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL(features,intersector) \
  INIT_SYMBOL(features,intersector);                                \
  SELECT_SYMBOL_AVX(features,intersector);

#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX512KNL_AVX512SKX(features,intersector) \
  INIT_SYMBOL(features,intersector);                                          \
  SELECT_SYMBOL_AVX(features,intersector);

#define SELECT_SYMBOL_INIT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,intersector) \
  INIT_SYMBOL(features,intersector);                                                \
  SELECT_SYMBOL_SSE42(features,intersector);

#define SELECT_SYMBOL_ZERO_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,intersector) \
  ZERO_SYMBOL(features,intersector);                                    \
  SELECT_SYMBOL_SSE42(features,intersector);

#define SELECT_SYMBOL_INIT_AVX512KNL_AVX512SKX(features,intersector) \
  INIT_SYMBOL(features,intersector);                                 \
  SELECT_SYMBOL_AVX512(features,intersector);

}
}
