// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

namespace embree
{
  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("instanced_geometry", FEATURE_RTCORE | FEATURE_STREAM) 
    {
      /* set default camera */
      camera.from = Vec3fa(2.5f,2.5f,2.5f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
    }
  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "multi_instanced_geometry");
  }
  return embree::Tutorial().main(argc,argv);
}
