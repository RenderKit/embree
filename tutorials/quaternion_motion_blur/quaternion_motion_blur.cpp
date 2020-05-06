// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

namespace embree
{
  typedef void (*DrawGUI)(void);

  extern "C" {
    int g_spp = 8;
    int g_numTimeSteps = 5;
    float g_time = 0.f;
    float g_shutter_close = 1.f;
    bool g_animate = true;
    bool g_accumulate = true;
    bool g_motion_blur = true;
    bool g_reset = false;
    DrawGUI g_drawGUI = nullptr;
  }

  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("quaternion motion blur",FEATURE_RTCORE | FEATURE_STREAM)
    {
      registerOption("spp", [] (Ref<ParseStream> cin, const FileName& path) {
          g_spp = cin->getInt();
        }, "--spp <int>: sets number of samples per pixel");

      camera.from = Vec3fa(-30.0f,30.0f,0.0f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
      camera.up   = Vec3fa(1.0f,0.0f,0.0f);
      camera.fov  = 30.f;
    }

#if defined(USE_GLFW)
    
    void drawGUI() override
    {
      if (ImGui::SliderInt ("samples per pixel",  &g_spp, 1, 32))  g_reset = true;
      if (ImGui::SliderInt ("time steps",  &g_numTimeSteps, 3, 10))  g_reset = true;
      if (ImGui::Checkbox ("animate", &g_animate)) g_reset = true;
      if (ImGui::Checkbox ("motion blur", &g_motion_blur)) g_reset = true;
      if (!g_motion_blur) {
        if (ImGui::SliderFloat ("time", &g_time, 0.f, 1.f)) g_reset = true;
      } else {
        if (ImGui::SliderFloat ("shutter close", &g_shutter_close, 0.f, 1.f)) g_reset = true;
      }
      if (g_drawGUI)
        g_drawGUI();
    }
#endif
  };
}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "quaternion_motion_blur");
  }
  return embree::Tutorial().main(argc,argv);
}
