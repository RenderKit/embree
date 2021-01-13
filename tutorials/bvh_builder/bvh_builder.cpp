// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

namespace embree
{
  struct Tutorial : public TutorialApplication
  {
    Tutorial()
      : TutorialApplication("bvh_builder",FEATURE_RTCORE)
    {
      interactive = false;
    }
  };

}

int main(int argc, char** argv)
{
  int code = embree::Tutorial().main(argc,argv);

  /* wait for user input under Windows when opened in separate window */
  embree::waitForKeyPressedUnderWindows();

  return code;
}
