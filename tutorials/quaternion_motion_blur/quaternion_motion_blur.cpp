// ======================================================================== //
// Copyright 2009-2020 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "../common/tutorial/tutorial.h"

namespace embree
{
  typedef void (*DrawGUI)(void);

  extern "C" {
    int g_spp = 8;
    int g_numTimeSteps = 5;
    float g_time = 0.f;
    float g_shutter_close = 1.f;
    bool g_animate = true;
    bool g_accumulate = true;
    bool g_motion_blur = true;
    bool g_reset = false;
    DrawGUI g_drawGUI = nullptr;
  }

  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("quaternion motion blur",FEATURE_RTCORE | FEATURE_STREAM)
    {
      registerOption("spp", [] (Ref<ParseStream> cin, const FileName& path) {
          g_spp = cin->getInt();
        }, "--spp <int>: sets number of samples per pixel");

      camera.from = Vec3fa(-30.0f,30.0f,0.0f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
      camera.up   = Vec3fa(1.0f,0.0f,0.0f);
      camera.fov  = 30.f;
    }

    void drawGUI() override
    {
      if (ImGui::SliderInt ("samples per pixel",  &g_spp, 1, 32))  g_reset = true;
      if (ImGui::SliderInt ("time steps",  &g_numTimeSteps, 3, 10))  g_reset = true;
      if (ImGui::Checkbox ("animate", &g_animate)) g_reset = true;
      if (ImGui::Checkbox ("motion blur", &g_motion_blur)) g_reset = true;
      if (!g_motion_blur) {
        if (ImGui::SliderFloat ("time", &g_time, 0.f, 1.f)) g_reset = true;
      } else {
        if (ImGui::SliderFloat ("shutter close", &g_shutter_close, 0.f, 1.f)) g_reset = true;
      }
      if (g_drawGUI)
        g_drawGUI();
    }
  };
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
