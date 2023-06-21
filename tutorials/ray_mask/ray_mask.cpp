// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define NAME "ray_mask_sycl"
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define NAME "ray_mask"
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  typedef void (*DrawGUI)(void);

  extern "C" {
    bool g_ray_mask = true;
    DrawGUI g_drawGUI = nullptr;
  }

  struct Tutorial : public TutorialApplication
  {
    Tutorial()
      : TutorialApplication(NAME,FEATURES)
    {
      /* set default camera */
      camera.from = Vec3fa(3.5f,3.0f,-5.0f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
    }
#if defined(USE_GLFW)
    
    void drawGUI() override
    {
      ImGui::Checkbox ("enable ray mask", &g_ray_mask);
      if (g_drawGUI)
        g_drawGUI();
    }
#endif
  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "ray_mask");
  }
  return embree::Tutorial().main(argc,argv);
}
