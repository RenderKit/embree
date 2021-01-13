// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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
    Vec3fa g_query_point(0.7f, 0.0f, 0.3f);
  }

  struct Tutorial : public TutorialApplication 
  {
    Tutorial()
      : TutorialApplication("voronoi", FEATURE_RTCORE | FEATURE_STREAM) {}

#if defined(USE_GLFW)
    
    void drawGUI() override
    {
      ImGui::SliderInt ("Number of points", &g_num_points, 4, 4096);
      ImGui::Checkbox  ("Show Voronoi", &g_show_voronoi);
      if (!g_show_voronoi)
      {
        ImGui::SliderInt ("Number of neighbours", &g_num_knn, 1, 128);
        ImGui::Checkbox  ("Simulate: Point repulsion", &g_point_repulsion);
        ImGui::InputFloat("Max dist for NN query", &g_tmax);
      }
    }

    void keypressed(int key) override
    {
      if (key == GLFW_KEY_RIGHT) g_query_point.x += 0.01f;
      if (key == GLFW_KEY_LEFT)  g_query_point.x -= 0.01f;
      if (key == GLFW_KEY_UP)    g_query_point.z += 0.01f;
      if (key == GLFW_KEY_DOWN)  g_query_point.z -= 0.01f;
      g_query_point = max(g_query_point, Vec3fa(0.f));
      g_query_point = min(g_query_point, Vec3fa(1.f));
      
      TutorialApplication::keypressed(key);
    }
#endif
  };
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
