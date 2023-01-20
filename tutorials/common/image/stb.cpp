// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "image.h"
#include "../../../common/sys/estring.h"

#include <iostream>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace embree
{
  Ref<Image> loadSTB(const FileName& fileName)
  {
    int width, height, channels;
    unsigned char *pixels = stbi_load(fileName.str().c_str(), &width, &height, &channels, 0);
    if(pixels == nullptr) {
      THROW_RUNTIME_ERROR("Could not load " + fileName.str());
    }

    Image* out = new Image4uc(width, height, fileName);
    const float rcpMaxColor = 1.f/255.f;
    for (size_t y = 0; y < height; y++)
    {
      for (size_t x = 0; x < width; x++)
      {
        float r = (channels > 0) ? (float(pixels[(y*width+x)*channels+0]) * rcpMaxColor) : 0.f;
        float g = (channels > 1) ? (float(pixels[(y*width+x)*channels+1]) * rcpMaxColor) : r;
        float b = (channels > 2) ? (float(pixels[(y*width+x)*channels+2]) * rcpMaxColor) : ((channels == 1) ? r : 0.f);
        float a = (channels > 3) ? (float(pixels[(y*width+x)*channels+3]) * rcpMaxColor) : 1.f;
        out->set(x, y, Color4(r,g,b,a));
      }
    }

    stbi_image_free(pixels);

    return out;
  }

  void storeSTB(const Ref<Image>& img, const FileName& fileName)
  {
    std::string ext = toLowerCase(fileName.ext());
    if (ext != "bmp" && ext != "png" && ext != "jpg") {
        THROW_RUNTIME_ERROR("Could not store image")
        return;
    }

    int width = int(img->width);
    int height = int(img->height);
    int channels = 3;
    std::vector<unsigned char> pixels(width*height*channels);
    const float maxColor = 255.f;
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        Color4 c = img->get(x, y);
        pixels[(y*img->width+x)*channels+0] = (unsigned char)(clamp(c.r) * maxColor);
        pixels[(y*img->width+x)*channels+1] = (unsigned char)(clamp(c.g) * maxColor);
        pixels[(y*img->width+x)*channels+2] = (unsigned char)(clamp(c.b) * maxColor);
      }
    }

    if (ext == "bmp" ) { stbi_write_bmp(fileName.str().c_str(), width, height, channels, pixels.data());                    return; }
    if (ext == "png" ) { stbi_write_png(fileName.str().c_str(), width, height, channels, pixels.data(), width * channels);  return; }
    if (ext == "jpg" ) { stbi_write_jpg(fileName.str().c_str(), width, height, channels, pixels.data(), 100);               return; }
  }
}
