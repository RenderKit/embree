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

#include "../common/tutorial/tutorial.h"

namespace embree
{
  extern "C" {
    float g_time = -1.0f;
    unsigned g_num_time_steps = 8;
    unsigned g_num_time_steps2 = 30;
  }

  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("motion_blur_geometry",FEATURE_RTCORE) 
    {
      registerOption("time", [] (Ref<ParseStream> cin, const FileName& path) {
        g_time = cin->getFloat();
      }, "--time <float>: time to render image at");

      registerOption("time-steps", [] (Ref<ParseStream> cin, const FileName& path) {
        g_num_time_steps = cin->getInt();
        if (g_num_time_steps < 2) throw std::runtime_error("at least 2 time steps have to be used");
      }, "--time-steps <int>: number of time steps to use");

      registerOption("time-steps2", [] (Ref<ParseStream> cin, const FileName& path) {
        g_num_time_steps2 = cin->getInt();
        if (g_num_time_steps2 < 2) throw std::runtime_error("at least 2 time steps have to be used");
      }, "--time-steps2 <int>: number of time steps to use");
    
      /* set default camera */
      camera.from = Vec3fa(6,11,0);
      camera.to   = Vec3fa(0,0,0);
    }
  };
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
