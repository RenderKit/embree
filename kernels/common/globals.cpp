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

#include "../../common/sys/platform.h"

#if defined(__MIC__)
#include "../../common/simd/avx512.h"
#else
#include "../../common/simd/sse.h"
#endif

namespace embree
{
#if !defined(__MIC__)

  template<>
  struct __aligned(32) vfloat<8>
  {
    __forceinline vfloat() { }
    __forceinline vfloat(float a) { for (size_t i=0; i<8; i++) v[i] = a; }
    __forceinline vfloat(StepTy ) { for (size_t i=0; i<8; i++) v[i] = i; }

    friend __forceinline const vfloat8 operator-(const vfloat8& a) { vfloat8 r; for (size_t i=0; i<8; i++) r.v[i] = -a.v[i]; return r; }

    friend __forceinline const vfloat8 operator+(const vfloat8& a, const vfloat8& b) { vfloat8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] + b.v[i]; return r; }
    friend __forceinline const vfloat8 operator-(const vfloat8& a, const vfloat8& b) { vfloat8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] - b.v[i]; return r; }

    friend __forceinline const vfloat8 operator*(const vfloat8& a, const vfloat8& b) { vfloat8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] * b.v[i]; return r; }
    friend __forceinline const vfloat8 operator*(const float a, const vfloat8& b) { vfloat8 r; for (size_t i=0; i<8; i++) r.v[i] = a      * b.v[i]; return r; }
    friend __forceinline const vfloat8 operator*(const vfloat8& a, const float b) { vfloat8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] * b     ; return r; }

    friend __forceinline const vfloat8 operator/(const vfloat8& a, const vfloat8& b) { vfloat8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] / b.v[i]; return r; }

   float v[8];
  };

  vfloat8 coeff0[4];
  vfloat8 coeff1[4];

  vfloat8 coeff_P0[4];
  vfloat8 coeff_P1[4];
  vfloat8 coeff_P2[4];
  vfloat8 coeff_P3[4];

  vfloat4 sse_coeff0[4];
  vfloat4 sse_coeff1[4];

  vfloat4 sse_coeff_P0[4];
  vfloat4 sse_coeff_P1[4];
  vfloat4 sse_coeff_P2[4];
  vfloat4 sse_coeff_P3[4];

#else

  vfloat16 coeff0[4];
  vfloat16 coeff1[4];

  vfloat16 coeff01[4];

  vfloat16 coeff_P0[4];
  vfloat16 coeff_P1[4];
  vfloat16 coeff_P2[4];
  vfloat16 coeff_P3[4];

#endif

  void init_globals()
  {

#if defined(__MIC__)

    {
      __aligned(64) float STEP[16] = {0/16.0f,1/16.0f,2/16.0f,3/16.0f,4/16.0f,5/16.0f,6/16.0f,7/16.0f,8/16.0f,9/16.0f,10/16.0f,11/16.0f,12/16.0f,13/16.0f,14/16.0f,15/16.0f};
      vfloat16 step16 = vfloat16::load(STEP);
      const vfloat16 t1 = step16;
      const vfloat16 t0 = 1.0f-t1;
      coeff01[0] = t0 * t0 * t0;
      coeff01[1] = 3.0f * t1 * t0 * t0;
      coeff01[2] = 3.0f * t1 * t1 * t0;
      coeff01[3] = t1 * t1 * t1;
    }
    vfloat16 step16 = vfloat16(step);
    vfloat16 dt   = 1.0f/16.0f;
    {
      const vfloat16 t1 = step16*dt;
      const vfloat16 t0 = 1.0f-t1;
      coeff0[0] = t0 * t0 * t0;
      coeff0[1] = 3.0f * t1 * t0 * t0;
      coeff0[2] = 3.0f * t1 * t1 * t0;
      coeff0[3] = t1 * t1 * t1;
    }
    {
      const vfloat16 t1 = step16*dt+vfloat16(dt);
      const vfloat16 t0 = 1.0f-t1;
      coeff1[0] = t0 * t0 * t0;
      coeff1[1] = 3.0f * t1 * t0 * t0;
      coeff1[2] = 3.0f * t1 * t1 * t0;
      coeff1[3] = t1 * t1 * t1;
    }

    {
      const vfloat16 t1 = step16*dt;
      const vfloat16 t0 = 1.0f-t1;
      coeff_P0[0] = t0 * t0 * t0;
      coeff_P0[1] = 3.0f * t1 * t0 * t0;
      coeff_P0[2] = 3.0f * t1 * t1 * t0;
      coeff_P0[3] = t1 * t1 * t1;
    }

    {
      const vfloat16 t1 = step16*dt;
      const vfloat16 t0 = 1.0f-t1;
      const vfloat16 d0 = -t0*t0;
      const vfloat16 d1 = t0*t0 - 2.0f*t0*t1;
      const vfloat16 d2 = 2.0f*t1*t0 - t1*t1;
      const vfloat16 d3 = t1*t1;
      coeff_P1[0] = coeff_P0[0] + dt*d0;
      coeff_P1[1] = coeff_P0[1] + dt*d1;
      coeff_P1[2] = coeff_P0[2] + dt*d2;
      coeff_P1[3] = coeff_P0[3] + dt*d3;
    }

    {
      const vfloat16 t1 = step16*dt+vfloat16(dt);
      const vfloat16 t0 = 1.0f-t1;
      coeff_P3[0] = t0 * t0 * t0;
      coeff_P3[1] = 3.0f * t1 * t0 * t0;
      coeff_P3[2] = 3.0f * t1 * t1 * t0;
      coeff_P3[3] = t1 * t1 * t1;
    }

    {
      const vfloat16 t1 = step16*dt+vfloat16(dt);
      const vfloat16 t0 = 1.0f-t1;
      const vfloat16 d0 = -t0*t0;
      const vfloat16 d1 = t0*t0 - 2.0f*t0*t1;
      const vfloat16 d2 = 2.0f*t1*t0 - t1*t1;
      const vfloat16 d3 = t1*t1;
      coeff_P2[0] = coeff_P3[0] - dt*d0;
      coeff_P2[1] = coeff_P3[1] - dt*d1;
      coeff_P2[2] = coeff_P3[2] - dt*d2;
      coeff_P2[3] = coeff_P3[3] - dt*d3;
    }

#else

    {
      const float dt = 1.0f/8.0f;
      const vfloat8 t1 = vfloat8(step)*dt;
      const vfloat8 t0 = 1.0f-t1;
      coeff0[0] = t0 * t0 * t0;
      coeff0[1] = 3.0f * t1 * t0 * t0;
      coeff0[2] = 3.0f * t1 * t1 * t0;
      coeff0[3] = t1 * t1 * t1;
    }
    {
      const float dt = 1.0f/8.0f;
      const vfloat8 t1 = vfloat8(step)*dt+vfloat8(dt);
      const vfloat8 t0 = 1.0f-t1;
      coeff1[0] = t0 * t0 * t0;
      coeff1[1] = 3.0f * t1 * t0 * t0;
      coeff1[2] = 3.0f * t1 * t1 * t0;
      coeff1[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const vfloat8 t1 = vfloat8(step)*dt;
      const vfloat8 t0 = 1.0f-t1;
      coeff_P0[0] = t0 * t0 * t0;
      coeff_P0[1] = 3.0f * t1 * t0 * t0;
      coeff_P0[2] = 3.0f * t1 * t1 * t0;
      coeff_P0[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const vfloat8 t1 = vfloat8(step)*dt;
      const vfloat8 t0 = 1.0f-t1;
      const vfloat8 d0 = -t0*t0;
      const vfloat8 d1 = t0*t0 - 2.0f*t0*t1;
      const vfloat8 d2 = 2.0f*t1*t0 - t1*t1;
      const vfloat8 d3 = t1*t1;
      coeff_P1[0] = coeff_P0[0] + dt*d0;
      coeff_P1[1] = coeff_P0[1] + dt*d1;
      coeff_P1[2] = coeff_P0[2] + dt*d2;
      coeff_P1[3] = coeff_P0[3] + dt*d3;
    }

    {
      const float dt = 1.0f/8.0f;
      const vfloat8 t1 = vfloat8(step)*dt+vfloat8(dt);
      const vfloat8 t0 = 1.0f-t1;
      coeff_P3[0] = t0 * t0 * t0;
      coeff_P3[1] = 3.0f * t1 * t0 * t0;
      coeff_P3[2] = 3.0f * t1 * t1 * t0;
      coeff_P3[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const vfloat8 t1 = vfloat8(step)*dt+vfloat8(dt);
      const vfloat8 t0 = 1.0f-t1;
      const vfloat8 d0 = -t0*t0;
      const vfloat8 d1 = t0*t0 - 2.0f*t0*t1;
      const vfloat8 d2 = 2.0f*t1*t0 - t1*t1;
      const vfloat8 d3 = t1*t1;
      coeff_P2[0] = coeff_P3[0] - dt*d0;
      coeff_P2[1] = coeff_P3[1] - dt*d1;
      coeff_P2[2] = coeff_P3[2] - dt*d2;
      coeff_P2[3] = coeff_P3[3] - dt*d3;
    }

    ////////////////////////// SSE ////////////////////////

    {
      const float dt = 1.0f/4.0f;
      const vfloat4 t1 = vfloat4(step)*dt;
      const vfloat4 t0 = 1.0f-t1;
      sse_coeff0[0] = t0 * t0 * t0;
      sse_coeff0[1] = 3.0f * t1 * t0 * t0;
      sse_coeff0[2] = 3.0f * t1 * t1 * t0;
      sse_coeff0[3] = t1 * t1 * t1;
    }
    {
      const float dt = 1.0f/4.0f;
      const vfloat4 t1 = vfloat4(step)*dt+vfloat4(dt);
      const vfloat4 t0 = 1.0f-t1;
      sse_coeff1[0] = t0 * t0 * t0;
      sse_coeff1[1] = 3.0f * t1 * t0 * t0;
      sse_coeff1[2] = 3.0f * t1 * t1 * t0;
      sse_coeff1[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/4.0f;
      const vfloat4 t1 = vfloat4(step)*dt;
      const vfloat4 t0 = 1.0f-t1;
      sse_coeff_P0[0] = t0 * t0 * t0;
      sse_coeff_P0[1] = 3.0f * t1 * t0 * t0;
      sse_coeff_P0[2] = 3.0f * t1 * t1 * t0;
      sse_coeff_P0[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/4.0f;
      const vfloat4 t1 = vfloat4(step)*dt;
      const vfloat4 t0 = 1.0f-t1;
      const vfloat4 d0 = -t0*t0;
      const vfloat4 d1 = t0*t0 - 2.0f*t0*t1;
      const vfloat4 d2 = 2.0f*t1*t0 - t1*t1;
      const vfloat4 d3 = t1*t1;
      sse_coeff_P1[0] = sse_coeff_P0[0] + dt*d0;
      sse_coeff_P1[1] = sse_coeff_P0[1] + dt*d1;
      sse_coeff_P1[2] = sse_coeff_P0[2] + dt*d2;
      sse_coeff_P1[3] = sse_coeff_P0[3] + dt*d3;
    }

    {
      const float dt = 1.0f/4.0f;
      const vfloat4 t1 = vfloat4(step)*dt+vfloat4(dt);
      const vfloat4 t0 = 1.0f-t1;
      sse_coeff_P3[0] = t0 * t0 * t0;
      sse_coeff_P3[1] = 3.0f * t1 * t0 * t0;
      sse_coeff_P3[2] = 3.0f * t1 * t1 * t0;
      sse_coeff_P3[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/4.0f;
      const vfloat4 t1 = vfloat4(step)*dt+vfloat4(dt);
      const vfloat4 t0 = 1.0f-t1;
      const vfloat4 d0 = -t0*t0;
      const vfloat4 d1 = t0*t0 - 2.0f*t0*t1;
      const vfloat4 d2 = 2.0f*t1*t0 - t1*t1;
      const vfloat4 d3 = t1*t1;
      sse_coeff_P2[0] = sse_coeff_P3[0] - dt*d0;
      sse_coeff_P2[1] = sse_coeff_P3[1] - dt*d1;
      sse_coeff_P2[2] = sse_coeff_P3[2] - dt*d2;
      sse_coeff_P2[3] = sse_coeff_P3[3] - dt*d3;
    }
#endif
  }

}
