// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "common/default.h"

namespace embree
{
  struct __aligned(32) avxf 
  {
    __forceinline avxf() { }
    __forceinline avxf(float a) { for (size_t i=0; i<8; i++) v[i] = a; }
    __forceinline avxf(StepTy ) { for (size_t i=0; i<8; i++) v[i] = i; }

    friend __forceinline const avxf operator-(const avxf& a) { avxf r; for (size_t i=0; i<8; i++) r.v[i] = -a.v[i]; return r; }

    friend __forceinline const avxf operator+(const avxf& a, const avxf& b) { avxf r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] + b.v[i]; return r; }
    friend __forceinline const avxf operator-(const avxf& a, const avxf& b) { avxf r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] - b.v[i]; return r; }

    friend __forceinline const avxf operator*(const avxf& a, const avxf& b) { avxf r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] * b.v[i]; return r; }
    friend __forceinline const avxf operator*(const float a, const avxf& b) { avxf r; for (size_t i=0; i<8; i++) r.v[i] = a      * b.v[i]; return r; }
    friend __forceinline const avxf operator*(const avxf& a, const float b) { avxf r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] * b     ; return r; }

    friend __forceinline const avxf operator/(const avxf& a, const avxf& b) { avxf r; for (size_t i=0; i<8; i++) r.v[i] = a.v[i] / b.v[i]; return r; }

    float v[8];
  };

  avxf coeff0[4];
  avxf coeff1[4];

  avxf coeff_P0[4];
  avxf coeff_P1[4];
  avxf coeff_P2[4];
  avxf coeff_P3[4];

  void init_globals()
  {
    {
      const float dt = 1.0f/8.0f;
      const avxf t1 = avxf(step)*dt;
      const avxf t0 = 1.0f-t1;
      coeff0[0] = t0 * t0 * t0;
      coeff0[1] = 3.0f * t1 * t0 * t0;
      coeff0[2] = 3.0f * t1 * t1 * t0;
      coeff0[3] = t1 * t1 * t1;
    }
    {
      const float dt = 1.0f/8.0f;
      const avxf t1 = avxf(step)*dt+avxf(dt);
      const avxf t0 = 1.0f-t1;
      coeff1[0] = t0 * t0 * t0;
      coeff1[1] = 3.0f * t1 * t0 * t0;
      coeff1[2] = 3.0f * t1 * t1 * t0;
      coeff1[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const avxf t1 = avxf(step)*dt;
      const avxf t0 = 1.0f-t1;
      coeff_P0[0] = t0 * t0 * t0;
      coeff_P0[1] = 3.0f * t1 * t0 * t0;
      coeff_P0[2] = 3.0f * t1 * t1 * t0;
      coeff_P0[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const avxf t1 = avxf(step)*dt;
      const avxf t0 = 1.0f-t1;
      const avxf d0 = -t0*t0;
      const avxf d1 = t0*t0 - 2.0f*t0*t1;
      const avxf d2 = 2.0f*t1*t0 - t1*t1;
      const avxf d3 = t1*t1;
      coeff_P1[0] = coeff_P0[0] + dt*d0;
      coeff_P1[1] = coeff_P0[1] + dt*d1;
      coeff_P1[2] = coeff_P0[2] + dt*d2;
      coeff_P1[3] = coeff_P0[3] + dt*d3;
    }

    {
      const float dt = 1.0f/8.0f;
      const avxf t1 = avxf(step)*dt+avxf(dt);
      const avxf t0 = 1.0f-t1;
      coeff_P3[0] = t0 * t0 * t0;
      coeff_P3[1] = 3.0f * t1 * t0 * t0;
      coeff_P3[2] = 3.0f * t1 * t1 * t0;
      coeff_P3[3] = t1 * t1 * t1;
    }

    {
      const float dt = 1.0f/8.0f;
      const avxf t1 = avxf(step)*dt+avxf(dt);
      const avxf t0 = 1.0f-t1;
      const avxf d0 = -t0*t0;
      const avxf d1 = t0*t0 - 2.0f*t0*t1;
      const avxf d2 = 2.0f*t1*t0 - t1*t1;
      const avxf d3 = t1*t1;
      coeff_P2[0] = coeff_P3[0] - dt*d0;
      coeff_P2[1] = coeff_P3[1] - dt*d1;
      coeff_P2[2] = coeff_P3[2] - dt*d2;
      coeff_P2[3] = coeff_P3[3] - dt*d3;
    }
  }
}
