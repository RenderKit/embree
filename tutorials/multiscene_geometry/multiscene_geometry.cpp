// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define NAME "multiscene_geometry_sycl"
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define NAME "multiscene_geometry"
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  extern int g_scene_id;
  
  struct Tutorial : public TutorialApplication
  {
    Tutorial()
      : TutorialApplication(NAME, FEATURES)
    {
      /* set start camera */
      camera.from = Vec3f(2, 2, 2);
      camera.to = Vec3f(0, 0, 0);
    }

#if defined(USE_GLFW)
    
    void drawGUI() override
    {
      static const char* items[] = { "full scene", "right half", "left half" };
      ImGui::Combo("##scene_id",&g_scene_id,items,IM_ARRAYSIZE(items));
    }
   
    void keypressed(int key) override
    {
      if (key == ' ')
        g_scene_id = (g_scene_id+1)%3;
      else
        TutorialApplication::keypressed(key);
    }
#endif
    
  };
  
}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv);
  }
  return embree::Tutorial().main(argc, argv);
}
