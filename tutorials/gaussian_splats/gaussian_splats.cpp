// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#define NAME "gaussian_splats"
#define FEATURES FEATURE_RTCORE

namespace embree
{
  struct Tutorial : public TutorialApplication
  {
    Tutorial()
      : TutorialApplication(NAME, FEATURES)
    {
      camera.from = Vec3fa(0.0f, 2.5f, 10.0f);
      camera.to   = Vec3fa(0.0f, 0.5f, 0.0f);
    }
  };
}

int main(int argc, char** argv)
{
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "gaussian_splats");
  }
  return embree::Tutorial().main(argc, argv);
}
