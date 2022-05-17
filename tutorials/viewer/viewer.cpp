// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  extern "C" float g_min_width = 0.0f;
  extern "C" float g_min_width_max_radius_scale;
  extern "C" RTCFeatureFlags g_feature_mask = RTC_FEATURE_ALL;
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("viewer",FEATURES)
    {
#if RTC_MIN_WIDTH
      registerOption("min-width", [] (Ref<ParseStream> cin, const FileName& path) {
          g_min_width = cin->getFloat();
          g_min_width_max_radius_scale = cin->getFloat();
        }, "--min-width <float> <float>: first value sets number of pixel to enlarge curve and point geometry to, but maximally scales hair radii by second value");
#endif

#if defined(EMBREE_SYCL_TUTORIAL)
      registerOption("features", [] (Ref<ParseStream> cin, const FileName& path) {

          unsigned int feature_mask = RTC_FEATURE_NONE;
          while (cin->peek() != "" && cin->peek()[0] != '-') {
            std::string feature = cin->getString();
            std::transform(feature.begin(), feature.end(), feature.begin(), [](unsigned char c){ return std::toupper(c); });
            if      (feature == "MOTION_BLUR") feature_mask |= RTC_FEATURE_MOTION_BLUR;
            else if (feature == "TRIANGLE") feature_mask |= RTC_FEATURE_TRIANGLE;
            else if (feature == "QUAD") feature_mask |= RTC_FEATURE_QUAD;
            else if (feature == "GRID") feature_mask |= RTC_FEATURE_GRID;
            else if (feature == "SUBDIVISION") feature_mask |= RTC_FEATURE_SUBDIVISION;
            else if (feature == "CONE_LINEAR_CURVE" ) feature_mask |= RTC_FEATURE_CONE_LINEAR_CURVE;
            else if (feature == "ROUND_LINEAR_CURVE") feature_mask |= RTC_FEATURE_ROUND_LINEAR_CURVE;
            else if (feature == "FLAT_LINEAR_CURVE" ) feature_mask |= RTC_FEATURE_FLAT_LINEAR_CURVE;
            else if (feature == "ROUND_BEZIER_CURVE") feature_mask |= RTC_FEATURE_ROUND_BEZIER_CURVE;
            else if (feature == "FLAT_BEZIER_CURVE") feature_mask |= RTC_FEATURE_FLAT_BEZIER_CURVE;
            else if (feature == "NORMAL_ORIENTED_BEZIER_CURVE") feature_mask |= RTC_FEATURE_NORMAL_ORIENTED_BEZIER_CURVE;
            else if (feature == "ROUND_BSPLINE_CURVE") feature_mask |= RTC_FEATURE_ROUND_BSPLINE_CURVE;
            else if (feature == "FLAT_BSPLINE_CURVE") feature_mask |= RTC_FEATURE_FLAT_BSPLINE_CURVE;
            else if (feature == "NORMAL_ORIENTED_BSPLINE_CURVE") feature_mask |= RTC_FEATURE_NORMAL_ORIENTED_BSPLINE_CURVE;
            else if (feature == "ROUND_HERMITE_CURVE") feature_mask |= RTC_FEATURE_ROUND_HERMITE_CURVE;
            else if (feature == "FLAT_HERMITE_CURVE") feature_mask |= RTC_FEATURE_FLAT_HERMITE_CURVE;
            else if (feature == "NORMAL_ORIENTED_HERMITE_CURVE") feature_mask |= RTC_FEATURE_NORMAL_ORIENTED_HERMITE_CURVE;
            else if (feature == "SPHERE_POINT") feature_mask |= RTC_FEATURE_SPHERE_POINT;
            else if (feature == "DISC_POINT") feature_mask |= RTC_FEATURE_DISC_POINT;
            else if (feature == "ORIENTED_DISC_POINT") feature_mask |= RTC_FEATURE_ORIENTED_DISC_POINT;
            else if (feature == "POINT") feature_mask |= RTC_FEATURE_POINT;
            else if (feature == "ROUND_CATMULL_ROM_CURVE") feature_mask |= RTC_FEATURE_ROUND_CATMULL_ROM_CURVE;
            else if (feature == "FLAT_CATMULL_ROM_CURVE") feature_mask |= RTC_FEATURE_FLAT_CATMULL_ROM_CURVE;
            else if (feature == "NORMAL_ORIENTED_CATMULL_ROM_CURVE") feature_mask |= RTC_FEATURE_NORMAL_ORIENTED_CATMULL_ROM_CURVE;
            else if (feature == "ROUND_CURVES") feature_mask |= RTC_FEATURE_ROUND_CURVES;
            else if (feature == "FLAT_CURVES") feature_mask |= RTC_FEATURE_FLAT_CURVES;
            else if (feature == "NORMAL_ORIENTED_CURVES") feature_mask |= RTC_FEATURE_NORMAL_ORIENTED_CURVES;
            else if (feature == "INSTANCE") feature_mask |= RTC_FEATURE_INSTANCE;
            else if (feature == "FILTER_FUNCTION_IN_CONTEXT") feature_mask |= RTC_FEATURE_FILTER_FUNCTION_IN_CONTEXT;
            else if (feature == "FILTER_FUNCTION_IN_GEOMETRY") feature_mask |= RTC_FEATURE_FILTER_FUNCTION_IN_GEOMETRY;
            else if (feature == "FILTER_FUNCTION") feature_mask |= RTC_FEATURE_FILTER_FUNCTION;
            else if (feature == "USER_GEOMETRY_CALLBACK_IN_CONTEXT") feature_mask |= RTC_FEATURE_USER_GEOMETRY_CALLBACK_IN_CONTEXT;
            else if (feature == "USER_GEOMETRY_CALLBACK_IN_GEOMETRY") feature_mask |= RTC_FEATURE_USER_GEOMETRY_CALLBACK_IN_GEOMETRY;
            else if (feature == "USER_GEOMETRY") feature_mask |= RTC_FEATURE_USER_GEOMETRY;
            else if (feature == "ALL") feature_mask |= RTC_FEATURE_ALL;
            else throw std::runtime_error("unknown feature \"" + feature + "\"");
          }
          g_feature_mask = (RTCFeatureFlags) feature_mask;

        }, "--features feature1,feature2: sets feature mask for JIT compilation, e.g. TRIANGLE,QUAD");
#endif
    }
    
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
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv);
  }
  return embree::Tutorial().main(argc, argv);
}
