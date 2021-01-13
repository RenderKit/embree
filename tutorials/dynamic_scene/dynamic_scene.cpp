// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

namespace embree
{
  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("dynamic_scene",FEATURE_RTCORE) 
    {
      /* set start camera */
      camera.from = Vec3f(2,2,2);
      camera.to = Vec3f(0,0,0);
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
