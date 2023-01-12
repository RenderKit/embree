// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/benchmark_render.h"
#include "nanite_geometry_device.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{

  extern "C" RenderMode user_rendering_mode;
  extern "C" uint user_spp;
  
  
  struct Tutorial : public SceneLoadingTutorialApplication 
  {
    Tutorial()
      : SceneLoadingTutorialApplication("nanite_geometry", FEATURES) 
    {
      /* set default camera */
      camera.from = Vec3fa(2.5f,2.5f,2.5f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
    }


#if defined(USE_GLFW)
    
    /* called when a key is pressed */
    void keypressed(int key) override
    {
      if (key == GLFW_KEY_F1) {
        user_rendering_mode = RENDER_PRIMARY;
      }
      else if (key == GLFW_KEY_F2) {
        user_rendering_mode = RENDER_DEBUG_GRIDS;
      }
      else if (key == GLFW_KEY_F3) {
        user_rendering_mode = RENDER_DEBUG_SUBGRIDS;
      }
      else if (key == GLFW_KEY_F4) {
        user_rendering_mode = RENDER_DEBUG_QUADS;
      }
      else if (key == GLFW_KEY_F5) {
        user_rendering_mode = RENDER_DEBUG_LOD;
      }
      else if (key == GLFW_KEY_F6) {
        user_rendering_mode = RENDER_DEBUG_CRACK_FIXING;
      }
      else if (key == GLFW_KEY_KP_SUBTRACT) {
        user_spp -= user_spp > 0 ? 1 : 0;
      }            
      else if (key == GLFW_KEY_KP_ADD) {
        user_spp += 1;
      }            
      
      else
        TutorialApplication::keypressed(key);
    }
#endif
    

   void postParseCommandLine() override
    {
      /* load default scene if none specified */
      if (scene_empty_post_parse()) {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }
    
  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "nanite_geometry");
  }
  return embree::Tutorial().main(argc,argv);
}
