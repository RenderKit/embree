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
    int g_num_hits = 4;
    bool g_verify = false;
  }
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("allhits",FEATURE_RTCORE)
    {
      registerOption("num_hits", [] (Ref<ParseStream> cin, const FileName& path) {
          g_num_hits = cin->getInt();
        }, "--num_hits <int>: sets number of hits to accumulate maximally in each trace ray call");

      registerOption("verify", [] (Ref<ParseStream> cin, const FileName& path) {
          g_verify = true;
        }, "--verify: verifies result of collecting all hits");
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
      ImGui::DragInt("num_hits",&g_num_hits,1.0f,0,16);
      ImGui::Checkbox("verify",&g_verify);
    }
  };
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
