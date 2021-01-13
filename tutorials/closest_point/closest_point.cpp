// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

namespace embree
{
  extern "C" {
    bool g_animate = false;
    bool g_userDefinedInstancing = false;
  }

  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("closest_point",FEATURE_RTCORE | FEATURE_STREAM) 
    {
      camera.from = Vec3fa(8.74064f, 8.84506f, 7.48329f);
      camera.to = Vec3fa(-0.106665f, -1.8421f, -6.5347f);
      camera.fov  = 60;
    }

#if defined(USE_GLFW)
    
    void drawGUI() override
    {
      ImGui::Checkbox  ("Animate", &g_animate);
      ImGui::Checkbox  ("User Defined Instancing", &g_userDefinedInstancing);
    }
    
#endif
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
