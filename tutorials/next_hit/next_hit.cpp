// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

namespace embree
{
  #define SINGLE_PASS 0
  #define MULTI_PASS_FIXED_NEXT_HITS 1
  #define MULTI_PASS_OPTIMAL_NEXT_HITS 2
  #define MULTI_PASS_ESTIMATED_NEXT_HITS 3

  const char* next_hit_mode_names[] = {
    "single pass",
    "multi_pass_fixed_next_hits",
    "multi_pass_optimal_next_hits",
    "multi_pass_estimated_next_hits"
  };
  
  extern "C" {
    int g_next_hit_mode = MULTI_PASS_FIXED_NEXT_HITS;
    unsigned g_max_next_hits = 4;
    unsigned g_max_total_hits = 128;
    bool g_verify = false;
    bool g_visualize_errors = false;
    bool g_enable_opacity = false;
    float g_curve_opacity = 0.5f;
  }
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("next_hit",FEATURE_RTCORE)
    {
      registerOption("single_pass", [] (Ref<ParseStream> cin, const FileName& path) {
          g_next_hit_mode = SINGLE_PASS;
        }, "--single_pass: use special all hits kernel to gather all hits along ray");

      registerOption("multi_pass_fixed_next_hits", [] (Ref<ParseStream> cin, const FileName& path) {
          g_next_hit_mode = MULTI_PASS_FIXED_NEXT_HITS;
        }, "--multi_pass_fixed_next_hits: use multiple passes and gather a fixed number of hits each pass");

      registerOption("multi_pass_optimal_next_hits", [] (Ref<ParseStream> cin, const FileName& path) {
          g_next_hit_mode = MULTI_PASS_OPTIMAL_NEXT_HITS;
        }, "--multi_pass_optimal_next_hits: use multiple passes and gather the optimal number of hits (known from previous frame)");

      registerOption("multi_pass_estimate_next_hits", [] (Ref<ParseStream> cin, const FileName& path) {
          g_next_hit_mode = MULTI_PASS_ESTIMATED_NEXT_HITS;
        }, "--multi_pass_estimate_next_hits: use multiple passes and estimate the number of hits to gather from opacity");

      registerOption("max_next_hits", [] (Ref<ParseStream> cin, const FileName& path) {
          g_max_next_hits = cin->getInt();
        }, "--max_next_hits <int>: sets maximal number of hits to accumulate in each pass");

      registerOption("max_total_hits", [] (Ref<ParseStream> cin, const FileName& path) {
          g_max_total_hits = cin->getInt();
          if (g_max_total_hits > 16*1024)
            throw std::runtime_error("max_total_hits too large");
        }, "--max_total_hits <int>: sets maximal number of hits to accumulate in total");

      registerOption("curve_opacity", [] (Ref<ParseStream> cin, const FileName& path) {
          g_enable_opacity = true;
          g_curve_opacity = cin->getFloat();
        }, "--curve_opacity <float>: specifies opacity for curves, make surfaces opaque, enables stop at opaque surface, and enables roussian roulette ray termination");
      
      registerOption("verify", [] (Ref<ParseStream> cin, const FileName& path) {
          g_verify = true;
        }, "--verify: verifies result of collecting all hits");

      registerOption("visualize_errors", [] (Ref<ParseStream> cin, const FileName& path) {
          g_visualize_errors = true;
        }, "--visualize_errors: visualizes pixels where collected hits are wrong");
    }
    
    void postParseCommandLine() override
    {
      /* load default scene if none specified */
      if (scene->size() == 0 && sceneFilename.size() == 0) {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }

#if defined(USE_GLFW)
    
    void drawGUI() override
    {
      
      ImGui::Combo("mode",&g_next_hit_mode,next_hit_mode_names,4);
      if (g_next_hit_mode != 0)
        ImGui::DragInt("max_next_hits",(int*)&g_max_next_hits,1.0f,1,16);
      ImGui::DragInt("max_total_hits",(int*)&g_max_total_hits,1.0f,1,128);
      ImGui::Checkbox("enable_opacity",&g_enable_opacity);
      if (g_enable_opacity)
        ImGui::DragFloat("curve_opacity",&g_curve_opacity,0.01f,0.0f,1.0f);
      if (g_next_hit_mode != 0)
        ImGui::Checkbox("verify",&g_verify);
    }
#endif   
  };
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
