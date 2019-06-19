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
  typedef void (*DrawGUI)(void);

  extern "C" {
    int g_num_points = 128;
    int g_num_knn = 4;
    bool g_show_voronoi = true;
    bool g_point_repulsion = false;
    float g_tmax = inf;
    DrawGUI g_drawGUI = nullptr;
  }

  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("voronoi", FEATURE_RTCORE | FEATURE_STREAM) 
    { }

    void drawGUI() override
    {
      ImGui::SliderInt ("Number of points", &g_num_points, 4, 4096);
      if (g_drawGUI)
        g_drawGUI();
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
