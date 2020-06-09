// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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
    
    void postParseCommandLine() override
    {
      /* load default scene if none specified */
      if (sceneFilename.size() == 0) {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
