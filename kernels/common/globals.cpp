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

  struct __aligned(32) float8 
  {
    __forceinline float8() { }
    __forceinline float8(float a) { for (size_t i=0; i<8; i++) v[i] = a; }
    __forceinline float8(StepTy ) { for (size_t i=0; i<8; i++) v[i] = i; }

    friend __forceinline const float8 operator-(const float8& a) { float8 r; for (size_t i=0; i<8; i++) r.v[i] = -a.v[i]; return r; }

    friend __forceinline const float8 operator+(const float8& a, const float8& b) { float8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] + b.v[i]; return r; }
    friend __forceinline const float8 operator-(const float8& a, const float8& b) { float8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] - b.v[i]; return r; }

    friend __forceinline const float8 operator*(const float8& a, const float8& b) { float8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] * b.v[i]; return r; }
    friend __forceinline const float8 operator*(const float a, const float8& b) { float8 r; for (size_t i=0; i<8; i++) r.v[i] = a      * b.v[i]; return r; }
    friend __forceinline const float8 operator*(const float8& a, const float b) { float8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] * b     ; return r; }

    friend __forceinline const float8 operator/(const float8& a, const float8& b) { float8 r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] / b.v[i]; return r; }

    float v[8];
  };

  float8 coeff0[4];
  float8 coeff1[4];

  float8 coeff_P0[4];
  float8 coeff_P1[4];
  float8 coeff_P2[4];
  float8 coeff_P3[4];

  float4 sse_coeff0[4];
  float4 sse_coeff1[4];

  float4 sse_coeff_P0[4];
  float4 sse_coeff_P1[4];
  float4 sse_coeff_P2[4];
  float4 sse_coeff_P3[4];

#else

  float16 coeff0[4];
  float16 coeff1[4];

  float16 coeff01[4]; 

  float16 coeff_P0[4];
  float16 coeff_P1[4];
  float16 coeff_P2[4];
  float16 coeff_P3[4];

#endif

  void init_globals()
  {

#if defined(__MIC__)

    {
      __aligned(64) float STEP[16] = {0/16.0f,1/16.0f,2/16.0f,3/16.0f,4/16.0f,5/16.0f,6/16.0f,7/16.0f,8/16.0f,9/16.0f,10/16.0f,11/16.0f,12/16.0f,13/16.0f,14/16.0f,15/16.0f};
      float16 step16 = load16f(STEP); 
      const float16 t1 = step16;
      const float16 t0 = 1.0f-t1;
      coeff01[0] = t0 * t0 * t0;
      coeff01[1] = 3.0f * t1 * t0 * t0;
      coeff01[2] = 3.0f * t1 * t1 * t0;
      coeff01[3] = t1 * t1 * t1;
    }
    float16 step16 = float16(step); 
    float16 dt   = 1.0f/16.0f;
    {
      const float16 t1 = step16*dt;
      const float16 t0 = 1.0f-t1;
      coeff0[0] = t0 * t0 * t0;
      coeff0[1] = 3.0f * t1 * t0 * t0;
      coeff0[2] = 3.0f * t1 * t1 * t0;
      coeff0[3] = t1 * t1 * t1;
    }
    {
      const float16 t1 = step16*dt+float16(dt);
      const float16 t0 = 1.0f-t1;
      coeff1[0] = t0 * t0 * t0;
      coeff1[1] = 3.0f * t1 * t0 * t0;
      coeff1[2] = 3.0f * t1 * t1 * t0;
      coeff1[3] = t1 * t1 * t1;
    }

    {
      const float16 t1 = step16*dt;
      const float16 t0 = 1.0f-t1;
      coeff_P0[0] = t0 * t0 * t0;
      coeff_P0[1] = 3.0f * t1 * t0 * t0;
      coeff_P0[2] = 3.0f * t1 * t1 * t0;
      coeff_P0[3] = t1 * t1 * t1;
    }

    {
      const float16 t1 = step16*dt;
      const float16 t0 = 1.0f-t1;
      const float16 d0 = -t0*t0;
      const float16 d1 = t0*t0 - 2.0f*t0*t1;
      const float16 d2 = 2.0f*t1*t0 - t1*t1;
      const float16 d3 = t1*t1;
      coeff_P1[0] = coeff_P0[0] + dt*d0;
      coeff_P1[1] = coeff_P0[1] + dt*d1;
      coeff_P1[2] = coeff_P0[2] + dt*d2;
      coeff_P1[3] = coeff_P0[3] + dt*d3;
    }

    {
      const float16 t1 = step16*dt+float16(dt);
      const float16 t0 = 1.0f-t1;
      coeff_P3[0] = t0 * t0 * t0;
      coeff_P3[1] = 3.0f * t1 * t0 * t0;
      coeff_P3[2] = 3.0f * t1 * t1 * t0;
      coeff_P3[3] = t1 * t1 * t1;
    }

    {
      const float16 t1 = step16*dt+float16(dt);
      const float16 t0 = 1.0f-t1;
      const float16 d0 = -t0*t0;
      const float16 d1 = t0*t0 - 2.0f*t0*t1;
      const float16 d2 = 2.0f*t1*t0 - t1*t1;
      const float16 d3 = t1*t1;
      coeff_P2[0] = coeff_P3[0] - dt*d0;
      coeff_P2[1] = coeff_P3[1] - dt*d1;
      coeff_P2[2] = coeff_P3[2] - dt*d2;
      coeff_P2[3] = coeff_P3[3] - dt*d3;
    }

#else

    {
      const float dt = 1.0f/8.0f;
      const float8 t1 = float8(step)*dt;
      const float8 t0 = 1.0f-t1;
      coeff0[0] = t0 * t0 * t0;
      coeff0[1] = 3.0f * t1 * t0 * t0;
      coeff0[2] = 3.0f * t1 * t1 * t0;
      coeff0[3] = t1 * t1 * t1;
    }
    {
      const float dt = 1.0f/8.0f;
      const float8 t1 = float8(step)*dt+float8(dt);
      const float8 t0 = 1.0f-t1;
      coeff1[0] = t0 * t0 * t0;
      coeff1[1] = 3.0f * t1 * t0 * t0;
      coeff1[2] = 3.0f * t1 * t1 * t0;
      coeff1[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const float8 t1 = float8(step)*dt;
      const float8 t0 = 1.0f-t1;
      coeff_P0[0] = t0 * t0 * t0;
      coeff_P0[1] = 3.0f * t1 * t0 * t0;
      coeff_P0[2] = 3.0f * t1 * t1 * t0;
      coeff_P0[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const float8 t1 = float8(step)*dt;
      const float8 t0 = 1.0f-t1;
      const float8 d0 = -t0*t0;
      const float8 d1 = t0*t0 - 2.0f*t0*t1;
      const float8 d2 = 2.0f*t1*t0 - t1*t1;
      const float8 d3 = t1*t1;
      coeff_P1[0] = coeff_P0[0] + dt*d0;
      coeff_P1[1] = coeff_P0[1] + dt*d1;
      coeff_P1[2] = coeff_P0[2] + dt*d2;
      coeff_P1[3] = coeff_P0[3] + dt*d3;
    }

    {
      const float dt = 1.0f/8.0f;
      const float8 t1 = float8(step)*dt+float8(dt);
      const float8 t0 = 1.0f-t1;
      coeff_P3[0] = t0 * t0 * t0;
      coeff_P3[1] = 3.0f * t1 * t0 * t0;
      coeff_P3[2] = 3.0f * t1 * t1 * t0;
      coeff_P3[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const float8 t1 = float8(step)*dt+float8(dt);
      const float8 t0 = 1.0f-t1;
      const float8 d0 = -t0*t0;
      const float8 d1 = t0*t0 - 2.0f*t0*t1;
      const float8 d2 = 2.0f*t1*t0 - t1*t1;
      const float8 d3 = t1*t1;
      coeff_P2[0] = coeff_P3[0] - dt*d0;
      coeff_P2[1] = coeff_P3[1] - dt*d1;
      coeff_P2[2] = coeff_P3[2] - dt*d2;
      coeff_P2[3] = coeff_P3[3] - dt*d3;
    }

    ////////////////////////// SSE ////////////////////////

    {
      const float dt = 1.0f/4.0f;
      const float4 t1 = float4(step)*dt;
      const float4 t0 = 1.0f-t1;
      sse_coeff0[0] = t0 * t0 * t0;
      sse_coeff0[1] = 3.0f * t1 * t0 * t0;
      sse_coeff0[2] = 3.0f * t1 * t1 * t0;
      sse_coeff0[3] = t1 * t1 * t1;
    }
    {
      const float dt = 1.0f/4.0f;
      const float4 t1 = float4(step)*dt+float4(dt);
      const float4 t0 = 1.0f-t1;
      sse_coeff1[0] = t0 * t0 * t0;
      sse_coeff1[1] = 3.0f * t1 * t0 * t0;
      sse_coeff1[2] = 3.0f * t1 * t1 * t0;
      sse_coeff1[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/4.0f;
      const float4 t1 = float4(step)*dt;
      const float4 t0 = 1.0f-t1;
      sse_coeff_P0[0] = t0 * t0 * t0;
      sse_coeff_P0[1] = 3.0f * t1 * t0 * t0;
      sse_coeff_P0[2] = 3.0f * t1 * t1 * t0;
      sse_coeff_P0[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/4.0f;
      const float4 t1 = float4(step)*dt;
      const float4 t0 = 1.0f-t1;
      const float4 d0 = -t0*t0;
      const float4 d1 = t0*t0 - 2.0f*t0*t1;
      const float4 d2 = 2.0f*t1*t0 - t1*t1;
      const float4 d3 = t1*t1;
      sse_coeff_P1[0] = sse_coeff_P0[0] + dt*d0;
      sse_coeff_P1[1] = sse_coeff_P0[1] + dt*d1;
      sse_coeff_P1[2] = sse_coeff_P0[2] + dt*d2;
      sse_coeff_P1[3] = sse_coeff_P0[3] + dt*d3;
    }

    {
      const float dt = 1.0f/4.0f;
      const float4 t1 = float4(step)*dt+float4(dt);
      const float4 t0 = 1.0f-t1;
      sse_coeff_P3[0] = t0 * t0 * t0;
      sse_coeff_P3[1] = 3.0f * t1 * t0 * t0;
      sse_coeff_P3[2] = 3.0f * t1 * t1 * t0;
      sse_coeff_P3[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/4.0f;
      const float4 t1 = float4(step)*dt+float4(dt);
      const float4 t0 = 1.0f-t1;
      const float4 d0 = -t0*t0;
      const float4 d1 = t0*t0 - 2.0f*t0*t1;
      const float4 d2 = 2.0f*t1*t0 - t1*t1;
      const float4 d3 = t1*t1;
      sse_coeff_P2[0] = sse_coeff_P3[0] - dt*d0;
      sse_coeff_P2[1] = sse_coeff_P3[1] - dt*d1;
      sse_coeff_P2[2] = sse_coeff_P3[2] - dt*d2;
      sse_coeff_P2[3] = sse_coeff_P3[3] - dt*d3;
    }
#endif
  }

}
