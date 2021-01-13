// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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
      camera.from = Vec3fa(8,13,2.5);
      camera.to   = Vec3fa(0,0,2.5);
    }
  };
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
