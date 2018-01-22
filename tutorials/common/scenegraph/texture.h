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

#pragma once

#if defined(ISPC)

enum TEXTURE_FORMAT {
    Texture_RGBA8        = 1,
    Texture_RGB8         = 2,
    Texture_FLOAT32      = 3,
  };

struct Texture {
  int width;
  int height;
  int format;
  int bytesPerTexel;
  int width_mask;
  int height_mask;
  void* data;
};

#else

#include "../default.h"
#include "../image/image.h"

namespace embree
{
  struct Texture // FIXME: should be derived from SceneGraph::Node
  {
    enum Format {
      INVALID = 0,
      RGBA8   = 1,
      RGB8    = 2,
      FLOAT32 = 3,
    };
    
  public:
    Texture (); 
    Texture (Ref<Image> image, const std::string fileName); 
    Texture (unsigned width, unsigned height, const Format format, const char* in = nullptr);
    ~Texture ();

  private:
    Texture (const Texture& other) DELETED; // do not implement
    Texture& operator= (const Texture& other) DELETED; // do not implement

  public:
    static const char* format_to_string(const Format format);
    static Format string_to_format(const std::string& str);
    static unsigned getFormatBytesPerTexel(const Format format);

    static std::shared_ptr<Texture> load(const FileName& fileName);
    static void clearTextureCache();
    
  public:
    unsigned width;
    unsigned height;    
    Format format;
    unsigned bytesPerTexel;
    unsigned width_mask;
    unsigned height_mask;
    void* data;
    std::string fileName;
  };
}
#endif
