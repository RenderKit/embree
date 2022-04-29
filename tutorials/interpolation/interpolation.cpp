// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("interpolation",FEATURES)
    {
      /* set default camera */
      camera.from = Vec3fa(9.0f,4.0f,1.0f);
      camera.to   = Vec3fa(0.0f,0.0f,1.0f);
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
