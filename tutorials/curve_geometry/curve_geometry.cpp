// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

namespace embree
{
  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("curve_geometry",FEATURE_RTCORE) 
    {
      /* set default camera */
      camera.from = Vec3fa(-0.1188741848f, 6.87527132f, 7.228342533f);
      camera.to   = Vec3fa(-0.1268435568f, -1.961063862f, -0.5809717178f);
    }
  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "curve_geometry");
  }
  return embree::Tutorial().main(argc,argv);
}
