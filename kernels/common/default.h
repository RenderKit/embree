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

#include "sys/platform.h"
#include "sys/sysinfo.h"
#include "sys/thread.h"
#include "sys/alloc.h"
#include "sys/ref.h"
#include "sys/intrinsics.h"
#include "sys/atomic.h"
#include "sys/mutex.h"
#include "sys/vector.h"
#include "sys/array.h"
#include "sys/string.h"
#include "sys/regression.h"

#if defined(TASKING_LOCKSTEP)
#include "tasking/taskscheduler_mic.h"
#else // if defined(TASKING_TBB_INTERNAL) // FIXME
#include "tasking/taskscheduler_tbb.h"
#endif

#include "config.h"
#include "isa.h"

#include "math/math.h"
#include "math/vec2.h"
#include "math/vec3.h"
#include "math/vec4.h"
#include "math/bbox.h"
#include "math/obbox.h"
#include "math/affinespace.h"

#include "simd/simd.h"
#include "embree2/rtcore.h"
#include "stat.h"
#include "monitor.h"
#include "profile.h"

#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <array>

namespace embree
{

  /* we consider floating point numbers in that range as valid input numbers */
#define VALID_FLOAT_RANGE  1.844E18f

  __forceinline bool inFloatRange(const float v) {
    return (v > -VALID_FLOAT_RANGE) && (v < +VALID_FLOAT_RANGE);
  };
  __forceinline bool inFloatRange(const Vec3fa& v) {
    return all(gt_mask(v,Vec3fa_t(-VALID_FLOAT_RANGE)) & lt_mask(v,Vec3fa_t(+VALID_FLOAT_RANGE)));
  };
#if defined(__SSE2__)
  __forceinline bool inFloatRange(const ssef& v) {
    return all((v > ssef(-VALID_FLOAT_RANGE)) & (v < ssef(+VALID_FLOAT_RANGE)));
  };
#endif
  __forceinline bool inFloatRange(const BBox3fa& v) {
    return all(gt_mask(v.lower,Vec3fa_t(-VALID_FLOAT_RANGE)) & lt_mask(v.upper,Vec3fa_t(+VALID_FLOAT_RANGE)));
  };

#define MODE_HIGH_QUALITY (1<<8)
#define LIST_MODE_BITS 0xFF

#if 0
#define LeafMode 1
#define LeafIterator1 ListIntersector1
#define LeafIterator4 ListIntersector4
#define LeafIterator4_1 ListIntersector4_1
#define LeafIterator8 ListIntersector8
#define LeafIterator8_1 ListIntersector8_1
#else
#define LeafMode 0
#define LeafIterator1 ArrayIntersector1
#define LeafIterator4 ArrayIntersector4
#define LeafIterator4_1 ArrayIntersector4_1
#define LeafIterator8 ArrayIntersector8
#define LeafIterator8_1 ArrayIntersector8_1
#endif

  /* global settings */
  extern size_t g_numThreads;
  extern size_t g_verbose;

  extern std::string g_tri_accel;
  extern std::string g_tri_builder;
  extern std::string g_tri_traverser;
  extern double g_tri_builder_replication_factor;

  extern std::string g_tri_accel_mb;
  extern std::string g_tri_builder_mb;
  extern std::string g_tri_traverser_mb;

  extern std::string g_hair_accel;
  extern std::string g_hair_builder;
  extern std::string g_hair_traverser;
  extern double g_hair_builder_replication_factor;

  extern std::string g_subdiv_accel;

  extern int g_scene_flags;
  extern size_t g_benchmark;
  extern float g_memory_preallocation_factor;

  /*! processes an error */
  void process_error(RTCError error, const char* code);

#if defined (__SSE__) // || defined (__MIC__)
  typedef Vec2<sseb> sse2b;
  typedef Vec3<sseb> sse3b;
  typedef Vec2<ssei> sse2i;
  typedef Vec3<ssei> sse3i;
  typedef Vec2<ssef> sse2f;
  typedef Vec3<ssef> sse3f;
  typedef Vec4<ssef> sse4f;
  typedef LinearSpace3<sse3f> LinearSpaceSSE3f;
  typedef AffineSpaceT<LinearSpace3<sse3f > > AffineSpaceSSE3f;
  typedef BBox<sse3f > BBoxSSE3f;
#endif

#if defined (__AVX__)
  typedef Vec2<avxb> avx2b;
  typedef Vec3<avxb> avx3b;
  typedef Vec2<avxi> avx2i; 
  typedef Vec3<avxi> avx3i;
  typedef Vec2<avxf> avx2f;
  typedef Vec3<avxf> avx3f;
  typedef Vec4<avxf> avx4f;
#endif

#if defined (__MIC__)
  typedef Vec2<mic_m> mic2b;
  typedef Vec3<mic_m> mic3b;
  typedef Vec2<mic_i> mic2i;
  typedef Vec3<mic_i> mic3i;
  typedef Vec2<mic_f> mic2f;
  typedef Vec3<mic_f> mic3f;
  typedef Vec4<mic_f> mic4f;
  typedef Vec4<mic_i> mic4i;
#endif

typedef void (*ErrorFunc) ();


void memoryMonitor(ssize_t bytes, bool post);

struct my_runtime_error : public std::exception
{
  __forceinline my_runtime_error(RTCError error, const std::string& str)
    : error(error), str(str) {}

  ~my_runtime_error() throw() {}
  
  const char* what () const throw () {
    return str.c_str();
  }

  RTCError error;
  std::string str;
};

#define THROW_MY_RUNTIME_ERROR(error,str)                                \
  throw my_runtime_error(error,std::string(__FILE__) + " (" + std::to_string((long long)__LINE__) + "): " + std::string(str));


}
