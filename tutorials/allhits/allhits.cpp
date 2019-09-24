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
    bool g_single_pass = false;
    unsigned g_max_next_hits = 4;
    unsigned g_max_total_hits = 128;
    bool g_verify = false;
    bool g_visualize_errors = false;
    float g_curve_opacity = -1.0f;
  }
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("allhits",FEATURE_RTCORE)
    {
      registerOption("single_pass", [] (Ref<ParseStream> cin, const FileName& path) {
          g_single_pass = true;
        }, "--single_pass: use special all hits kernel to gather all hits along ray");

      registerOption("max_next_hits", [] (Ref<ParseStream> cin, const FileName& path) {
          g_max_next_hits = cin->getInt();
        }, "--max_next_hits <int>: sets maximal number of hits to accumulate in each pass");

      registerOption("max_total_hits", [] (Ref<ParseStream> cin, const FileName& path) {
          g_max_total_hits = cin->getInt();
          if (g_max_total_hits > 16*1024)
            throw std::runtime_error("max_total_hits too large");
        }, "--max_total_hits <int>: sets maximal number of hits to accumulate in total");

       registerOption("curve_opacity", [] (Ref<ParseStream> cin, const FileName& path) {
           g_curve_opacity = cin->getFloat();
        }, "--curve_opacity <float>: sets the opacity to use for curves to terminate the ray");

      registerOption("verify", [] (Ref<ParseStream> cin, const FileName& path) {
          g_verify = true;
        }, "--verify: verifies result of collecting all hits");

      registerOption("visualize_errors", [] (Ref<ParseStream> cin, const FileName& path) {
          g_visualize_errors = true;
        }, "--visualize_errors: visualizes pixels where collected hits are wrong");
    }
    
    void postParseCommandLine() 
    {
      /* load default scene if none specified */
      if (scene->size() == 0 && sceneFilename.size() == 0) {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }

    void drawGUI()
    {
      ImGui::DragInt("max_next_hits",(int*)&g_max_next_hits,1.0f,0,16);
      ImGui::DragInt("max_total_hits",(int*)&g_max_total_hits,1.0f,0,128);
      ImGui::Checkbox("verify",&g_verify);
    }
  };
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
