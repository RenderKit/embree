// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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
  extern "C" {
    bool g_animate = false;
    bool g_userDefinedInstancing = false;
  }

  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("closest_point",FEATURE_RTCORE | FEATURE_STREAM) 
    {
      camera.from = Vec3fa(8.74064, 8.84506, 7.48329);
      camera.to = Vec3fa(-0.106665, -1.8421, -6.5347);
      camera.fov  = 60;
    }
    
    void drawGUI() override
    {
      ImGui::Checkbox  ("Animate", &g_animate);
      ImGui::Checkbox  ("User Defined Instancing", &g_userDefinedInstancing);
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
