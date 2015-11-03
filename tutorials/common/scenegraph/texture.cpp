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
    : width(-1), height(-1), format(INVALID), bytesPerTexel(0), data(nullptr), width_mask(0), height_mask(0) {}
  
  Texture::Texture(Ref<Image> img, const std::string fileName)
    : width(img->width), height(img->height), format(RGBA8), bytesPerTexel(4), data(nullptr), width_mask(0), height_mask(0), fileName(fileName)
  {
    width_mask  = isPowerOf2(width) ? width-1 : 0;
    height_mask = isPowerOf2(height) ? height-1 : 0;

    data = _mm_malloc(4*width*height,64);
    img->convertToRGBA8((unsigned char*)data);
  }

  Texture::Texture (size_t width, size_t height, const Format format, const char* in)
    : width(width), height(height), format(format), bytesPerTexel(getFormatBytesPerTexel(format)), data(nullptr), width_mask(0), height_mask(0)
  {
    width_mask  = isPowerOf2(width) ? width-1 : 0;
    height_mask = isPowerOf2(height) ? height-1 : 0;

    data = _mm_malloc(bytesPerTexel*width*height,64);
    if (in) memcpy(data,in,bytesPerTexel*width*height);
    else    memset(data,0 ,bytesPerTexel*width*height);
  }

  Texture::~Texture () {
    _mm_free(data);
  }

  const char* Texture::format_to_string(const Format format)
  {
    switch (format) {
    case RGBA8  : return "RGBA8";
    case RGB8   : return "RGB8";
    case FLOAT32: return "FLOAT32";
    default     : THROW_RUNTIME_ERROR("invalid texture format");
    }
  }

  Texture::Format Texture::string_to_format(const std::string& str)
  {
    if      (str == "RGBA8") return RGBA8;
    else if (str == "RGB8")  return RGB8;
    else if (str == "FLOAT32") return FLOAT32;
    else THROW_RUNTIME_ERROR("invalid texture format string");
  }

  int Texture::getFormatBytesPerTexel(const Format format)
  {
    switch (format) {
    case RGBA8  : return 4;
    case RGB8   : return 3;
    case FLOAT32: return 4;
    default     : THROW_RUNTIME_ERROR("invalid texture format");
    }
  }

  /*! read png texture from disk */
  Texture* Texture::load(const FileName& fileName)
  {
    if (texture_cache.find(fileName.str()) != texture_cache.end())
      return texture_cache[fileName.str()];

    return texture_cache[fileName.str()] = new Texture(loadImage(fileName),fileName);
  }
}
