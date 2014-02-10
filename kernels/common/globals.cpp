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
#if defined(__AVX__)  
  avxf coeff0[4];
  avxf coeff1[4];

  namespace isa
  {
    void init_globals()
    {
      {
        const float dt = 1.0f/8.0f;
        const avxf t1 = avxf(step)*dt;
        const avxf t0 = 1.0f-t1;
        coeff0[0] = t0 * t0 * t0;
        coeff0[1] = 3.0f * t1* t0 * t0;
        coeff0[2] = 3.0f * t1* t1 * t0;
        coeff0[3] = t1 * t1 * t1;
      }
      {
        const float dt = 1.0f/8.0f;
        const avxf t1 = avxf(step)*dt+avxf(dt);
        const avxf t0 = 1.0f-t1;
        coeff1[0] = t0 * t0 * t0;
        coeff1[1] = 3.0f * t1* t0 * t0;
        coeff1[2] = 3.0f * t1* t1 * t0;
        coeff1[3] = t1 * t1 * t1;
      }
    }
  }
#endif
}
