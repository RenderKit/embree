// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

namespace embree
{
  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("multi_instanced_geometry", FEATURE_RTCORE | FEATURE_STREAM) 
    {
      camera.from = Vec3fa(130.f, 50.f, 130.f);
      camera.to   = Vec3fa(0.0f, 0.0f, 0.0f);
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
