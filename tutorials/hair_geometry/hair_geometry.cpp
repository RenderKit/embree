// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define NAME "hair_geometry_sycl"
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#define NAME "hair_geometry"
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  extern "C" {
    embree::Vec3fa g_dirlight_direction = embree::normalize(embree::Vec3fa(1,-1,1));
    embree::Vec3fa g_dirlight_intensity = embree::Vec3fa(5.0f);
    embree::Vec3fa g_ambient_intensity = embree::Vec3fa(3.0f);
  }

  struct Tutorial : public SceneLoadingTutorialApplication 
  {
    Tutorial()
      : SceneLoadingTutorialApplication(NAME,FEATURES) {}

    void postParseCommandLine() override
    {
      /* load default scene if none specified */
      if (scene_empty_post_parse()) {
        FileName file = FileName::executableFolder() + FileName("models/furBall_A.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }

  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "hair_geometry");
  }
  return embree::Tutorial().main(argc,argv);
}
