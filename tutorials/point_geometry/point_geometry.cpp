// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

namespace embree
{
  struct Tutorial : public TutorialApplication
  {
    Tutorial()
      : TutorialApplication("point_geometry",FEATURE_RTCORE)
    {
      /* set default camera */
      camera.from = Vec3fa(0.0f, 2.0f, 7.0f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
