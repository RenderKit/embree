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

#include "bezier_curve.h"

namespace embree
{
  BezierCoefficients::BezierCoefficients(int dj)
  {
    for (size_t i=0; i<=N; i++) 
    {
      for (size_t j=0; j<=N; j++) 
      {
        const float t1 = float(j+dj)/float(i);
        const float t0 = 1.0f-t1;

        c0[i][j] = t0 * t0 * t0;
        c1[i][j] = 3.0f * t1 * t0 * t0;
        c2[i][j] = 3.0f * t1 * t1 * t0;
        c3[i][j] = t1 * t1 * t1;

        d0[i][j] = -3.0f*(t0*t0);
        d1[i][j] = -6.0f*(t0*t1) + 3.0f*(t0*t0);
        d2[i][j] = +6.0f*(t0*t1) - 3.0f*(t1*t1);
        d3[i][j] = +3.0f*(t1*t1);
      }
    }
  }
  BezierCoefficients bezier_coeff0(0);
  BezierCoefficients bezier_coeff1(1);
}
