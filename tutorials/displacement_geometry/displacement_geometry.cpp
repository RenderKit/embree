// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

namespace embree
{
  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("displacement_geometry",FEATURE_RTCORE) 
    {
      /* set default camera */
      camera.from = Vec3fa(1.5f,1.5f,-1.5f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
    }
  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "displacement_geometry");
  }
  return embree::Tutorial().main(argc,argv);
}
