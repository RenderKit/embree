// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

namespace embree
{
  extern "C" float g_min_width = 0.0f;
  extern "C" float g_min_width_max_radius_scale;
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("viewer",FEATURE_RTCORE)
    {
#if RTC_MIN_WIDTH
      registerOption("min-width", [] (Ref<ParseStream> cin, const FileName& path) {
          g_min_width = cin->getFloat();
          g_min_width_max_radius_scale = cin->getFloat();
        }, "--min-width <float> <float>: first value sets number of pixel to enlarge curve and point geometry to, but maximally scales hair radii by second value");
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
