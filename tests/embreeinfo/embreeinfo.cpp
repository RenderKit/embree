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

#include <embree2/rtcore.h>
#include <xmmintrin.h>

int main(int argc, char* argv[])
{
  /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
  _mm_setcsr(_mm_getcsr() | /* FTZ */ (1<<15) | /* DAZ */ (1<<6));

  /* initialize Embree */
  rtcInit("verbose=1");

  /* cleanup Embree again */
  rtcExit();
  
  return 0;
}
