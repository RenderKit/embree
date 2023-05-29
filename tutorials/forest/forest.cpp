// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define NAME "forest_sycl"
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define NAME "forest"
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  typedef void (*DrawGUI)(void);

  extern "C" {
    #if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    bool g_use_instance_array = true;
    #else
    bool g_use_instance_array = false;
    #endif
    int  g_complexity = 2;
    bool g_rebuild = true;
    bool g_animate = true;
    bool g_trees_changed = false;
    bool g_trees_moved = false;
    size_t g_memory_consumed = 0;
    size_t g_cycles_cleanup = 0;
    size_t g_cycles_objects = 0;
    size_t g_cycles_embree_objects = 0;
    size_t g_cycles_embree_bvh_build = 0;
    size_t g_cycles_total = 0;
    int g_build_quality = g_animate ? 0 : 1; // low = 0, med = 1, high = 2, refit = 3
    int g_spp = 2; // will be squared in device
    int mode = g_animate ? 1 : 0; // rebuild = 0, update = 1
    int g_trees[] = { 0, 1, 2, 3, 4, 5 };
    int focal_length = 24;

    /*
     * a very simple time stamp with microsecond resolution that can also be
     * used from the ISPC side
     */
    int64_t get_clock()
    {
#if defined(__MACOSX__)
      // oh apple
      return 0;
#else
      struct timespec now;
      timespec_get(&now, TIME_UTC);
      return ((int64_t) now.tv_sec) * 1000000 + ((int64_t) now.tv_nsec) / 1000;
#endif
    }

    /*
     * returns the elapsed time in full milliseconds for a time stamp delta has
     * enough accuracy for a couple of seconds
     */
    uint32_t get_milliseconds(int64_t diff) {
      return uint32_t(((double)diff)/1000);
    }
  }

  struct Tutorial : public TutorialApplication
  {
    Tutorial()
      : TutorialApplication(NAME,FEATURES,1280,720)
    {
      /* set default camera */
      camera.from = Vec3fa(507.72, 109.37, 1173.20);
      camera.to   = Vec3fa(504.62, 108.63, 1161.37);
    }

#if defined(USE_GLFW)

    void drawGUI() override
    {
      if (ImGui::Button("Rebuild")) {
        mode = 0;
        g_rebuild = true;
      }
    #if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
      if (ImGui::Checkbox("Use instance arrays", &g_use_instance_array)) {
        mode = 0;
        g_rebuild = true;
      }
    #endif
      mode = g_animate ? 1 : 0;
      ImGui::Checkbox("Animate", &g_animate);

      if (ImGui::SliderInt("Zoom", &focal_length, 24, 120)) {
        camera.fov = 90.f / std::pow(((float)focal_length) / 24.f, 2.f);
      }

      ImGui::Text("Build quality:");
      if (ImGui::RadioButton("Low",    &g_build_quality, 0)) { g_build_quality = 0; g_rebuild = true; mode = 0; };
      ImGui::SameLine();
      if (ImGui::RadioButton("Medium", &g_build_quality, 1)) { g_build_quality = 1; g_rebuild = true; mode = 0; };
      ImGui::SameLine();
      if (ImGui::RadioButton("High",   &g_build_quality, 2)) { g_build_quality = 2; g_rebuild = true; mode = 0; };

      ImGui::Text("Samples per pixel:");
      if (ImGui::RadioButton("1", &g_spp, 1)) { g_spp = 1; };
      ImGui::SameLine();
      if (ImGui::RadioButton("4", &g_spp, 2)) { g_spp = 2; };
      ImGui::SameLine();
      if (ImGui::RadioButton("9", &g_spp, 3)) { g_spp = 3; };

      ImGui::Text("Number of trees/instances:");
      if (ImGui::RadioButton("250k",  &g_complexity, 0)) { g_complexity = 0; g_rebuild = true; mode = 0; };
      ImGui::SameLine();
      if (ImGui::RadioButton("500k",  &g_complexity, 1)) { g_complexity = 1; g_rebuild = true; mode = 0; };
      ImGui::SameLine();
      if (ImGui::RadioButton("1000k", &g_complexity, 2)) { g_complexity = 2; g_rebuild = true; mode = 0; };
      ImGui::SameLine();
      if (ImGui::RadioButton("4000k", &g_complexity, 3)) { g_complexity = 3; g_rebuild = true; mode = 0; };

      const char* types[] = { "Tree1", "Tree2", "Tree3", "Tree4", "Tree5", "Tree6" };
      ImGui::Text("Trees:");
      if (ImGui::Combo("Slot 1", &g_trees[0], types, IM_ARRAYSIZE(types))) { g_trees_changed = true; mode = 1; };
      if (ImGui::Combo("Slot 2", &g_trees[1], types, IM_ARRAYSIZE(types))) { g_trees_changed = true; mode = 1; };
      if (ImGui::Combo("Slot 3", &g_trees[2], types, IM_ARRAYSIZE(types))) { g_trees_changed = true; mode = 1; };
      if (ImGui::Combo("Slot 4", &g_trees[3], types, IM_ARRAYSIZE(types))) { g_trees_changed = true; mode = 1; };
      if (ImGui::Combo("Slot 5", &g_trees[4], types, IM_ARRAYSIZE(types))) { g_trees_changed = true; mode = 1; };
      if (ImGui::Combo("Slot 6", &g_trees[5], types, IM_ARRAYSIZE(types))) { g_trees_changed = true; mode = 1; };

      ImGui::Text("%s", ("Memory consumption: " + std::to_string(g_memory_consumed/1024/1024) + " MB").c_str());
      if (mode == 0) ImGui::Text("Build timings:");
      else           ImGui::Text("Update timings:");
      ImGui::Text("  Scene Data %d ms", get_milliseconds(g_cycles_objects));
      ImGui::Text("  Embree Data %d ms", get_milliseconds(g_cycles_embree_objects));
      ImGui::Text("  Embree BVH %d ms", get_milliseconds(g_cycles_embree_bvh_build));
      if (mode == 0) ImGui::Text("  Cleanup old scene data %d ms", get_milliseconds(g_cycles_cleanup));
      ImGui::Text("  Total %d ms", get_milliseconds(g_cycles_total));
    }

#endif

  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "forest");
  }
  return embree::Tutorial().main(argc,argv);
}
