// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define NAME "host_device_memory_sycl"
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define NAME "host_device_memory"
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
      camera.from = Vec3fa(5.5f,5.5f,-5.5f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
    }
  };

}

int main(int argc, char** argv) {
  try {
    if (embree::TutorialBenchmark::benchmark(argc, argv)) {
      return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "host_device_memory");
    }
    return embree::Tutorial().main(argc,argv);
  } catch (std::exception& e ) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
  }
}
