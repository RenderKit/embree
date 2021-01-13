// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "texture.h"

namespace embree
{
  bool isPowerOf2 (unsigned int x)
  {
    while (((x % 2) == 0) && x > 1)
      x /= 2;
    return (x == 1);
  }

  static std::map<std::string,std::shared_ptr<Texture>> texture_cache;

  void Texture::clearTextureCache() {
    texture_cache.clear();
  }

  Texture::Texture () 
    : width(-1), height(-1), format(INVALID), bytesPerTexel(0), width_mask(0), height_mask(0), data(nullptr) {}
  
  Texture::Texture(Ref<Image> img, const std::string fileName)
    : width(unsigned(img->width)), height(unsigned(img->height)), format(RGBA8), bytesPerTexel(4), width_mask(0), height_mask(0), data(nullptr), fileName(fileName)
  {
    width_mask  = isPowerOf2(width) ? width-1 : 0;
    height_mask = isPowerOf2(height) ? height-1 : 0;

    data = alignedMalloc(4*width*height,16);
    img->convertToRGBA8((unsigned char*)data);
  }

  Texture::Texture (unsigned width, unsigned height, const Format format, const char* in)
    : width(width), height(height), format(format), bytesPerTexel(getFormatBytesPerTexel(format)), width_mask(0), height_mask(0), data(nullptr)
  {
    width_mask  = isPowerOf2(width) ? width-1 : 0;
    height_mask = isPowerOf2(height) ? height-1 : 0;

    data = alignedMalloc(bytesPerTexel*width*height,16);
    if (in) {
      for (size_t i=0; i<bytesPerTexel*width*height; i++)
	((char*)data)[i] = in[i];
    }
    else {
      memset(data,0 ,bytesPerTexel*width*height);
    }   
  }

  Texture::~Texture () {
    alignedFree(data);
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

  unsigned Texture::getFormatBytesPerTexel(const Format format)
  {
    switch (format) {
    case RGBA8  : return 4;
    case RGB8   : return 3;
    case FLOAT32: return 4;
    default     : THROW_RUNTIME_ERROR("invalid texture format");
    }
  }

  /*! read png texture from disk */
  std::shared_ptr<Texture> Texture::load(const FileName& fileName)
  {
    if (texture_cache.find(fileName.str()) != texture_cache.end())
      return texture_cache[fileName.str()];
    
    std::shared_ptr<Texture> tex(new Texture(loadImage(fileName),fileName));
    return texture_cache[fileName.str()] = tex;
  }
}
