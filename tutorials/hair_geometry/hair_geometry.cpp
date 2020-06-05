// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"

namespace embree
{
  extern "C" {
    embree::Vec3fa g_dirlight_direction = embree::normalize(embree::Vec3fa(1,-1,1));
    embree::Vec3fa g_dirlight_intensity = embree::Vec3fa(5.0f);
    embree::Vec3fa g_ambient_intensity = embree::Vec3fa(3.0f);
  }

  struct Tutorial : public SceneLoadingTutorialApplication 
  {
    Tutorial()
      : SceneLoadingTutorialApplication("hair_geometry",FEATURE_RTCORE) {}

    void postParseCommandLine() override
    {
      /* load default scene if none specified */
      if (scene->size() == 0 && sceneFilename.size() == 0) {
        FileName file = FileName::executableFolder() + FileName("models/furBall_A.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }

  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
