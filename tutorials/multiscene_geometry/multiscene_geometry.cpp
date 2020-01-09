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
  extern int g_scene_id;
  
  struct Tutorial : public TutorialApplication
  {
    Tutorial()
      : TutorialApplication("dynamic_scene", FEATURE_RTCORE)
    {
      /* set start camera */
      camera.from = Vec3f(2, 2, 2);
      camera.to = Vec3f(0, 0, 0);
    }

    void drawGUI()
    {
      static const char* items[] = { "full scene", "right half", "left half" };
      ImGui::Combo("",&g_scene_id,items,IM_ARRAYSIZE(items));
    }
  };
  
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc, argv);
}
