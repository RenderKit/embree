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
#include "../../common/simd/simd.h"
#include "../../common/math/vec2.h"
#include "../../common/math/vec3.h"
#include "../../common/math/vec4.h"
#include "../../common/math/bbox.h"
#include "../../common/math/obbox.h"
#include "../../common/math/linearspace2.h"
#include "../../common/math/linearspace3.h"
#include "../../common/math/affinespace.h"
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
#include <map>
#include <algorithm>
#include <functional>

namespace embree
{
#if defined (__SSE__)
  typedef Vec2<bool4> Vec2b4;
  typedef Vec3<bool4> Vec3b4;
  typedef Vec2<int4> Vec2i4;
  typedef Vec3<int4> Vec3i4;
  typedef Vec2<float4> Vec2f4;
  typedef Vec3<float4> Vec3f4;
  typedef Vec4<float4> sse4f; // FIXME: rename
  typedef LinearSpace3<Vec3f4> LinearSpaceSSE3f;
  typedef AffineSpaceT<LinearSpace3<Vec3f4 > > AffineSpaceSSE3f;
  typedef BBox<Vec3f4 > BBoxSSE3f;
#endif

#if defined (__AVX__)
  typedef Vec2<bool8> Vec2b8;
  typedef Vec3<bool8> Vec3b8;
  typedef Vec2<int8> Vec2i8; 
  typedef Vec3<int8> Vec3i8;
  typedef Vec2<float8> Vec2f8;
  typedef Vec3<float8> Vec3f8;
  typedef Vec4<float8> avx4f; // FIXME: rename
#endif

#if defined (__AVX512F__) || defined (__MIC__)
  typedef Vec2<bool16> Vec2b16;
  typedef Vec3<bool16> Vec3b16;
  typedef Vec2<int16> Vec2i16; 
  typedef Vec3<int16> Vec3i16;
  typedef Vec2<float16> Vec2f16;
  typedef Vec3<float16> Vec3f16;
  typedef Vec4<float16> Vec4f16;
#endif
}
