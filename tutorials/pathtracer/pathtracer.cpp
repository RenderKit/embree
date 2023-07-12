// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"

#if defined(EMBREE_SYCL_TUTORIAL)
#  define NAME "pathtracer_sycl"
#  define FEATURES FEATURE_RTCORE | FEATURE_SYCL
#else
#  define NAME "pathtracer"
#  define FEATURES FEATURE_RTCORE
#endif

namespace embree
{
  extern "C" {
    int g_spp = 1;
    int g_max_path_length = 8;
    bool g_accumulate = 1;
  }
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication(NAME,FEATURES)
    {
      registerOption("spp", [] (Ref<ParseStream> cin, const FileName& path) {
          g_spp = cin->getInt();
        }, "--spp <int>: sets number of samples per pixel");

      registerOption("max-path-length", [] (Ref<ParseStream> cin, const FileName& path) {
          g_max_path_length = cin->getInt();
        }, "--max-path-length <int>: sets maximal path length (1=primary+shadow)");

      registerOption("accumulate", [] (Ref<ParseStream> cin, const FileName& path) {
          g_accumulate = cin->getInt();
        }, "--accumulate <bool>: accumulate samples (on by default)");
    }
    
    void postParseCommandLine() override
    {
      /* load default scene if none specified */
      if (scene_empty_post_parse()) {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }

#if defined(USE_GLFW)
    void drawGUI() override
    {
      ImGui::Checkbox("accumulate",&g_accumulate);
      ImGui::Text("max path length");
      ImGui::DragInt("##max_path_length",&g_max_path_length,1.0f,1,16);
      ImGui::Text("samples per pixel");
      ImGui::DragInt("##samples per pixel",&g_spp,1.0f,1,16);
    }
#endif
  };

}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv);
  }
  return embree::Tutorial().main(argc,argv);
}
