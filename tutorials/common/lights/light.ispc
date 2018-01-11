// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "light.isph"

Light_EvalRes Light_eval(const uniform Light* uniform,
                         const DifferentialGeometry&,
                         const Vec3f&)
{
  Light_EvalRes res;
  res.value = make_Vec3f(0.f);
  res.dist = inf;
  res.pdf = 0.f;
  return res;
}

export void Light_destroy(Light* uniform light)
{
  delete light;
}

export void dummy() {} // just to avoid linker warning under MacOSX
