// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/benchmark_render.h"
#include "micropoly_device.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{

  extern "C" RenderMode user_rendering_mode;
  extern "C" uint32_t user_spp;
  extern "C" uint32_t g_lod_threshold;
  extern "C" char* camera_file;
  extern "C" unsigned int camera_mode;
  extern "C" unsigned int frameIndex;
  extern "C" char* patches_file;

  FileName cameraFilename;
  FileName patchesFilename;
  
  struct Tutorial : public SceneLoadingTutorialApplication 
  {    
    Tutorial()
      : SceneLoadingTutorialApplication("Micro-Poly HW RT", FEATURES) 
    {
      registerOption("spp", [] (Ref<ParseStream> cin, const FileName& path) {
        user_spp = cin->getInt();
      }, "--spp <int>: sets number of samples per pixel");

      registerOption("rendering_mode", [] (Ref<ParseStream> cin, const FileName& path) {
        user_rendering_mode = (RenderMode)cin->getInt();
      }, "--rendering_mode <int>: sets rendering mode");

      registerOption("lod_threshold", [] (Ref<ParseStream> cin, const FileName& path) {
        g_lod_threshold = (RenderMode)cin->getInt();
      }, "--lod_threshold <uint>: sets lod threshold");

      registerOption("camera", [] (Ref<ParseStream> cin, const FileName& path) {
        cameraFilename = cin->getFileName();
        camera_file = (char*)cameraFilename.c_str();
      }, "--camera file");

      registerOption("patches", [] (Ref<ParseStream> cin, const FileName& path) {
        patchesFilename = cin->getFileName();
        patches_file = (char*)patchesFilename.c_str();
      }, "--patches file");
      

      registerOption("camera_record", [] (Ref<ParseStream> cin, const FileName& path) {
        camera_mode = 1;
      }, "--camera_record");

      registerOption("camera_replay", [] (Ref<ParseStream> cin, const FileName& path) {
        camera_mode = 2;
      }, "--camera_replay");
      
      
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
      else if (key == GLFW_KEY_F7) {
        user_rendering_mode = RENDER_DEBUG_CLOD;
      }
      else if (key == GLFW_KEY_F8) {
        user_rendering_mode = RENDER_DEBUG_TEXTURE;
      }      
      else if (key == GLFW_KEY_F9) {
        user_rendering_mode = RENDER_DEBUG_CLUSTER_ID;
      }
      else if (key == GLFW_KEY_F10) {
        user_rendering_mode = RENDER_DEBUG_LOD_LEVEL;
      }
      else if (key == GLFW_KEY_F11) {
        user_rendering_mode = RENDER_PATH_TRACER;
      }
#if defined(ENABLE_OIDN)
      else if (key == GLFW_KEY_F12) {
        user_rendering_mode = RENDER_PATH_TRACER_DENOISE;
      }                 
#endif                   
      else if (key == GLFW_KEY_KP_SUBTRACT) {
        user_spp -= user_spp > 0 ? 1 : 0;
      }            
      else if (key == GLFW_KEY_KP_ADD) {
        user_spp += 1;
      }            
      else if (key == GLFW_KEY_R) {
        frameIndex = 0;
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
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "MicroPoly Hardware RayTracing");
  }
  return embree::Tutorial().main(argc,argv);
}
