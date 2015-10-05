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

#include "texture.h"

namespace embree
{
  bool isPowerOf2 (unsigned int x)
  {
    while (((x % 2) == 0) && x > 1)
      x /= 2;
    return (x == 1);
  }

  std::map<std::string,Texture*> texture_cache;

  Texture::Texture () 
    : width(-1), height(-1), format(-1), bytesPerTexel(0), data(nullptr), width_mask(0), height_mask(0) {}
  
  Texture::Texture(Ref<Image> img)
    : width(img->width), height(img->height), format(Texture::RGBA8), bytesPerTexel(4), data(nullptr), width_mask(0), height_mask(0) 
  {
    width_mask  = isPowerOf2(width) ? width-1 : 0;
    height_mask = isPowerOf2(height) ? height-1 : 0;

    data = _mm_malloc(width*height*sizeof(int32_t),64);
    img->convertToRGBA8((unsigned char*)data);
  }

  const char* Texture::strFormat() const 
  {
    switch (format) {
    case RGBA8  : return "RGBA8";
    case RGB8   : return "RGB8";
    case FLOAT32: return "FLOAT32";
    }
    THROW_RUNTIME_ERROR("invalid texture format");
    return nullptr;
  }

  /*! read png texture from disk */
  Texture* Texture::load(const FileName& fileName)
  {
    if (texture_cache.find(fileName.str()) != texture_cache.end())
      return texture_cache[fileName.str()];

    return texture_cache[fileName.str()] = new Texture(loadImage(fileName));
  }
}
