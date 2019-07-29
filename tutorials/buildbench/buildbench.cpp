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
  uint32_t g_num_user_threads = 0;
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("build_bench",FEATURE_RTCORE) 
    {
      interactive = false;

      registerOption("user_threads", [this] (Ref<ParseStream> cin, const FileName& path) {
          g_num_user_threads = cin->getInt();
          rtcore += ",threads=" + toString(g_num_user_threads);
          rtcore += ",user_threads=" + toString(g_num_user_threads);
          rtcore += ",start_threads=0,set_affinity=0";
        }, "--user_threads <int>: invokes user thread benchmark with specified number of application provided build threads");
    }
    
    void postParseCommandLine() 
    {
      /* load default scene if none specified */
      if (sceneFilename.ext() == "") {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
