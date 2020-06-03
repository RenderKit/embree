// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

namespace embree
{
  extern "C" bool g_min_width_enabled;
  extern "C" float g_min_width = 0.0f;
  
  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("viewer",FEATURE_RTCORE)
    {
#if RTC_CURVE_MINWIDTH
      registerOption("min-width", [] (Ref<ParseStream> cin, const FileName& path) {
          g_min_width = cin->getFloat();
          g_min_width_enabled = true;
        }, "--min-width <float>: sets number of pixel to enlarge hair geometry to");
#endif
    }
    
    void postParseCommandLine() 
    {
      /* load default scene if none specified */
      if (scene->size() == 0 && sceneFilename.size() == 0) {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }
  };
}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
