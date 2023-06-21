// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define NAME "intersection_filter_sycl"
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define NAME "intersection_filter"
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication(NAME,FEATURES) 
    {
      /* set default camera */
      camera.from = Vec3fa(-1.27f,1.75f,-6.75f);
      camera.to   = Vec3fa(0.0f,-2.0f,-3.5f);
    }
  };
}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "intersection_filter");
  }
  return embree::Tutorial().main(argc,argv);
}
