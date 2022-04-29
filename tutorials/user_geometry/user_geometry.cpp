// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define FEATURES FEATURE_RTCORE | FEATURE_STREAM
#endif

namespace embree
{
  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("user_geometry",FEATURES) 
    {
#if defined(EMBREE_SYCL_TUTORIAL)
      std::cout << "WARNING: setting CFEFusedEUDispatch=1 as workaround to get stack calls working!" << std::endl;
#if defined(__WIN32__)
      _putenv_s("CFEFusedEUDispatch","1");
#else
      setenv("CFEFusedEUDispatch","1",1);
#endif
#endif
            
      /* set default camera */
      //camera.from = Vec3fa(0.8846014142,2.204180241,-0.6649734378);
      camera.from = Vec3fa(2.5f,2.5f,2.5f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
    }
  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "user_geometry");
  }
  return embree::Tutorial().main(argc,argv);
}
