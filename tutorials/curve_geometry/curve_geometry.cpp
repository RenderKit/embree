// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

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
  return embree::Tutorial().main(argc,argv);
}
