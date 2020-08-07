// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

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
      : SceneLoadingTutorialApplication("pathtracer",FEATURE_RTCORE)
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
      if (scene->size() == 0 && sceneFilename.size() == 0) {
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
      ImGui::DragInt("##spp",&g_spp,1.0f,1,16);
    }
#endif
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
