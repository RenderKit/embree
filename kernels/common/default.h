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

#include "../../common/sys/platform.h"
#include "../../common/sys/sysinfo.h"
#include "../../common/sys/thread.h"
#include "../../common/sys/alloc.h"
#include "../../common/sys/ref.h"
#include "../../common/sys/intrinsics.h"
#include "../../common/sys/atomic.h"
#include "../../common/sys/mutex.h"
#include "../../common/sys/vector.h"
#include "../../common/sys/array.h"
#include "../../common/sys/string.h"
#include "../../common/sys/regression.h"

#include "../../common/math/math.h"
#include "../../common/math/vec2.h"
#include "../../common/math/vec3.h"
#include "../../common/math/vec4.h"
#include "../../common/math/bbox.h"
#include "../../common/math/obbox.h"
#include "../../common/math/affinespace.h"
#include "../../common/simd/simd.h"
#include "../../common/lexers/tokenstream.h"

#if defined(TASKING_LOCKSTEP)
#include "../../common/tasking/taskscheduler_mic.h"
#else // if defined(TASKING_TBB_INTERNAL) // FIXME
#include "../../common/tasking/taskscheduler_tbb.h"
#endif

#define COMMA ,

#include "config.h"
#include "isa.h"
#include "stat.h"
#include "profile.h"
#include "rtcore.h"
#include "vector.h"
#include "state.h"

#include <vector>
#include <algorithm>
#include <functional>

namespace embree
{
#if defined (__SSE__)
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
}
