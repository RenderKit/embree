// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#pragma once

#include "../../../common/sys/platform.h"
#include "../../../common/sys/filename.h"
#include "../../../common/sys/vector.h"
#include "../../../common/math/vec2.h"
#include "../../../common/math/vec3.h"
#include "../../../common/math/affinespace.h"
#include "scene.h"

#include <vector>
#include <memory>

namespace embree
{
  struct Texture {
    
    enum {
      RGBA8        = 1,
      RGB8         = 2,
      FLOAT32      = 3,
      PTEX_RGBA8   = 4,
      PTEX_FLOAT32 = 5
    };
    
    int width;
    int height;    
    int format;
    union {
      int bytesPerTexel;
      int faceTextures;
    };
    int width_mask;
    int height_mask;
    
    void *data;
    
  Texture() 
  : width(-1), height(-1), format(-1), bytesPerTexel(0), data(nullptr), width_mask(0), height_mask(0) {}

  };

  /*! read texture from disk */
  Texture *loadTexture(const FileName& fileName);
  Texture *loadPtexTexture(const FileName& fileName);   
}
