// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define NAME "viewer_sycl"
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define NAME "viewer"
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  extern "C" float g_min_width = 0.0f;
  extern "C" float g_min_width_max_radius_scale;
  extern "C" bool g_use_scene_features = true;
  extern "C" RTCFeatureFlags g_feature_mask = RTC_FEATURE_FLAG_ALL;
  extern "C" float scale;
  extern "C" bool g_changed;
  extern "C" Shader shader = SHADER_DEFAULT;

  typedef void (* renderFrameFunc)(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern renderFrameFunc renderFrame;
  
  extern "C" void renderFrameStandard(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameDebugShader(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);
  extern "C" void renderFrameAOShader(int* pixels, const unsigned int width, const unsigned int height, const float time, const ISPCCamera& camera);

  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication(NAME,FEATURES)
    {
#if RTC_MIN_WIDTH
      registerOption("min-width", [] (Ref<ParseStream> cin, const FileName& path) {
          g_min_width = cin->getFloat();
          g_min_width_max_radius_scale = cin->getFloat();
        }, "--min-width <float> <float>: first value sets number of pixel to enlarge curve and point geometry to, but maximally scales hair radii by second value");
#endif

      registerOption("shader", [] (Ref<ParseStream> cin, const FileName& path) {
        std::string mode = cin->getString();
        if      (mode == "default" ) shader = SHADER_DEFAULT;
        else if (mode == "eyelight") shader = SHADER_EYELIGHT;
        else if (mode == "occlusion") shader = SHADER_OCCLUSION;
        else if (mode == "uv"      ) shader = SHADER_UV;
        else if (mode == "texcoords") shader = SHADER_TEXCOORDS;
        else if (mode == "texcoords-grid") shader = SHADER_TEXCOORDS_GRID;
        else if (mode == "Ng"      ) shader = SHADER_NG;
        else if (mode == "cycles"  ) { shader = SHADER_CYCLES; scale = cin->getFloat(); }
        else if (mode == "geomID"  ) shader = SHADER_GEOMID;
        else if (mode == "primID"  ) shader = SHADER_GEOMID_PRIMID;
        else if (mode == "ao" ) shader = SHADER_AO;
        else throw std::runtime_error("invalid shader:" +mode);
      },
      "--shader <string>: sets shader to use at startup\n"
      "  default: default tutorial shader\n"
      "  eyelight: eyelight shading\n"
      "  occlusion: occlusion shading\n"
      "  uv: uv debug shader\n"
      "  texcoords: texture coordinate debug shader\n"
      "  texcoords-grid: grid texture debug shader\n"
      "  Ng: visualization of shading normal\n"
      "  cycles <float>: CPU cycle visualization\n"
      "  ao: ambient occlusion\n"      
      "  geomID: visualization of geometry ID\n"
      "  primID: visualization of geometry and primitive ID");

#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
      registerOption("features", [] (Ref<ParseStream> cin, const FileName& path) {
          g_use_scene_features = false;
          unsigned int feature_mask = RTC_FEATURE_FLAG_NONE;
          while (cin->peek() != "" && cin->peek()[0] != '-') {
            std::string feature = cin->getString();
            std::transform(feature.begin(), feature.end(), feature.begin(), [](unsigned char c){ return toupper(c); });
            if      (feature == "MOTION_BLUR") feature_mask |= RTC_FEATURE_FLAG_MOTION_BLUR;
            else if (feature == "TRIANGLE") feature_mask |= RTC_FEATURE_FLAG_TRIANGLE;
            else if (feature == "QUAD") feature_mask |= RTC_FEATURE_FLAG_QUAD;
            else if (feature == "GRID") feature_mask |= RTC_FEATURE_FLAG_GRID;
            else if (feature == "SUBDIVISION") feature_mask |= RTC_FEATURE_FLAG_SUBDIVISION;
            else if (feature == "CONE_LINEAR_CURVE" ) feature_mask |= RTC_FEATURE_FLAG_CONE_LINEAR_CURVE;
            else if (feature == "ROUND_LINEAR_CURVE") feature_mask |= RTC_FEATURE_FLAG_ROUND_LINEAR_CURVE;
            else if (feature == "FLAT_LINEAR_CURVE" ) feature_mask |= RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE;
            else if (feature == "ROUND_BEZIER_CURVE") feature_mask |= RTC_FEATURE_FLAG_ROUND_BEZIER_CURVE;
            else if (feature == "FLAT_BEZIER_CURVE") feature_mask |= RTC_FEATURE_FLAG_FLAT_BEZIER_CURVE;
            else if (feature == "NORMAL_ORIENTED_BEZIER_CURVE") feature_mask |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_BEZIER_CURVE;
            else if (feature == "ROUND_BSPLINE_CURVE") feature_mask |= RTC_FEATURE_FLAG_ROUND_BSPLINE_CURVE;
            else if (feature == "FLAT_BSPLINE_CURVE") feature_mask |= RTC_FEATURE_FLAG_FLAT_BSPLINE_CURVE;
            else if (feature == "NORMAL_ORIENTED_BSPLINE_CURVE") feature_mask |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_BSPLINE_CURVE;
            else if (feature == "ROUND_HERMITE_CURVE") feature_mask |= RTC_FEATURE_FLAG_ROUND_HERMITE_CURVE;
            else if (feature == "FLAT_HERMITE_CURVE") feature_mask |= RTC_FEATURE_FLAG_FLAT_HERMITE_CURVE;
            else if (feature == "NORMAL_ORIENTED_HERMITE_CURVE") feature_mask |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_HERMITE_CURVE;
            else if (feature == "SPHERE_POINT") feature_mask |= RTC_FEATURE_FLAG_SPHERE_POINT;
            else if (feature == "DISC_POINT") feature_mask |= RTC_FEATURE_FLAG_DISC_POINT;
            else if (feature == "ORIENTED_DISC_POINT") feature_mask |= RTC_FEATURE_FLAG_ORIENTED_DISC_POINT;
            else if (feature == "POINT") feature_mask |= RTC_FEATURE_FLAG_POINT;
            else if (feature == "ROUND_CATMULL_ROM_CURVE") feature_mask |= RTC_FEATURE_FLAG_ROUND_CATMULL_ROM_CURVE;
            else if (feature == "FLAT_CATMULL_ROM_CURVE") feature_mask |= RTC_FEATURE_FLAG_FLAT_CATMULL_ROM_CURVE;
            else if (feature == "NORMAL_ORIENTED_CATMULL_ROM_CURVE") feature_mask |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_CATMULL_ROM_CURVE;
            else if (feature == "ROUND_CURVES") feature_mask |= RTC_FEATURE_FLAG_ROUND_CURVES;
            else if (feature == "FLAT_CURVES") feature_mask |= RTC_FEATURE_FLAG_FLAT_CURVES;
            else if (feature == "NORMAL_ORIENTED_CURVES") feature_mask |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_CURVES;
            else if (feature == "INSTANCE") feature_mask |= RTC_FEATURE_FLAG_INSTANCE;
            else if (feature == "INSTANCE_ARRAY") feature_mask |= RTC_FEATURE_FLAG_INSTANCE_ARRAY;
            else if (feature == "FILTER_FUNCTION_IN_ARGUMENTS") feature_mask |= RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS;
            else if (feature == "FILTER_FUNCTION_IN_GEOMETRY") feature_mask |= RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY;
            else if (feature == "FILTER_FUNCTION") feature_mask |= RTC_FEATURE_FLAG_FILTER_FUNCTION;
            else if (feature == "USER_GEOMETRY_CALLBACK_IN_ARGUMENTS") feature_mask |= RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS;
            else if (feature == "USER_GEOMETRY_CALLBACK_IN_GEOMETRY") feature_mask |= RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY;
            else if (feature == "USER_GEOMETRY") feature_mask |= RTC_FEATURE_FLAG_USER_GEOMETRY;
            else if (feature == "ALL") feature_mask |= RTC_FEATURE_FLAG_ALL;
            else throw std::runtime_error("unknown feature \"" + feature + "\"");
          }
          g_feature_mask = (RTCFeatureFlags) feature_mask;

        }, "--features feature1,feature2: sets feature mask for JIT compilation, e.g. TRIANGLE,QUAD");
#endif
    }

#if defined(USE_GLFW)
    
    /* called when a key is pressed */
    void keypressed(int key) override
    {
      if (key == GLFW_KEY_F1) {
        renderFrame = renderFrameStandard;
        shader = SHADER_DEFAULT;
        g_changed = true;
      }
      else if (key == GLFW_KEY_F2) {
        renderFrame = renderFrameDebugShader;
        shader = SHADER_EYELIGHT;
        g_changed = true;
      }
      else if (key == GLFW_KEY_F3) {
        renderFrame = renderFrameDebugShader;
        shader = SHADER_OCCLUSION;
        g_changed = true;
      }
      else if (key == GLFW_KEY_F4) {
        renderFrame = renderFrameDebugShader;
        shader = SHADER_UV;
        g_changed = true;
      }
      else if (key == GLFW_KEY_F5) {
        renderFrame = renderFrameDebugShader;
        shader = SHADER_NG;
        g_changed = true;
      }
      else if (key == GLFW_KEY_F6) {
        renderFrame = renderFrameDebugShader;
        shader = SHADER_GEOMID;
        g_changed = true;
      }
      else if (key == GLFW_KEY_F7) {
        renderFrame = renderFrameDebugShader;
        shader = SHADER_GEOMID_PRIMID;
        g_changed = true;
      }
      else if (key == GLFW_KEY_F8) {
        renderFrame = renderFrameDebugShader;
        if (shader == SHADER_TEXCOORDS) shader = SHADER_TEXCOORDS_GRID;
        else                            shader = SHADER_TEXCOORDS;
        g_changed = true;
      }
      else if (key == GLFW_KEY_F9) {
        if (shader == SHADER_CYCLES) scale *= 2.0f;
        renderFrame = renderFrameDebugShader;
        shader = SHADER_CYCLES; 
        g_changed = true;
      }
      else if (key == GLFW_KEY_F10) {
        if (shader == SHADER_CYCLES) scale *= 0.5f;
        renderFrame = renderFrameDebugShader;
        shader = SHADER_CYCLES; 
        g_changed = true;
      }
      else
        TutorialApplication::keypressed(key);
    }
#endif
    
    void postParseCommandLine() override
    {
      /* set shader mode */
      switch (shader) {
      case SHADER_DEFAULT  : renderFrame = renderFrameStandard; break;
      case SHADER_EYELIGHT : renderFrame = renderFrameDebugShader; break;
      case SHADER_OCCLUSION: renderFrame = renderFrameDebugShader; break;
      case SHADER_UV       : renderFrame = renderFrameDebugShader; break;
      case SHADER_TEXCOORDS: renderFrame = renderFrameDebugShader; break;
      case SHADER_TEXCOORDS_GRID: renderFrame = renderFrameDebugShader; break;
      case SHADER_NG       : renderFrame = renderFrameDebugShader; break;
      case SHADER_CYCLES   : renderFrame = renderFrameDebugShader; break;
      case SHADER_GEOMID   : renderFrame = renderFrameDebugShader; break;
      case SHADER_GEOMID_PRIMID: renderFrame = renderFrameDebugShader; break;
      case SHADER_AO: renderFrame = renderFrameAOShader; break;      
      };
      
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
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv);
  }
  return embree::Tutorial().main(argc, argv);
}
