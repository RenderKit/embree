// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "image.h"

#include <iostream>
#include <cstring>
#include <cstdio>

#define TINYEXR_USE_MINIZ 0
//#include "zlib.h"
//Or, if your project uses `stb_image[_write].h`, use their
//zlib implementation:
#define TINYEXR_USE_STB_ZLIB 1
#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"

namespace embree
{
  /*! read PFM file from disk */
  Ref<Image> loadEXR(const FileName& fileName)
  {
    float* rgba; // width * height * RGBA
    int width;
    int height;
    const char* err = NULL; // or nullptr in C++11

    int ret = LoadEXR(&rgba, &width, &height, fileName.str().c_str(), &err);
    if (ret != TINYEXR_SUCCESS) {
      if (err) {
        std::cerr << "ERR: " << err;
        FreeEXRErrorMessage(err);
      }
      THROW_RUNTIME_ERROR("Could not load image " + fileName.str())
    }

    /* create image and fill with data */
    Ref<Image> img = new Image4f(width,height,fileName);

    for (ssize_t y=0; y<height; y++) {
      for (ssize_t x=0; x<width; x++) {
        float* pix = rgba + (y * width + x) * 4;
        img->set(x,y,Color4(pix[0],pix[1],pix[2],1.0f));
      }
    }
    free(rgba);
    return img;
  }

  /*! store PFM file to disk */
  void storeEXR(const Ref<Image>& img, const FileName& fileName)
  {
    std::vector<Col3f> rgb(img->width * img->height);
    for (size_t y=0; y<img->height; ++y) {
        for (size_t x=0; x<img->width; ++x) {
            Color4 c = img->get(x, y);
            rgb[y * img->width + x] = Col3f(c.r, c.g, c.b);
        }
    }
    const char* err = NULL;
    int ret = SaveEXR((float*)rgb.data(), img->width, img->height, 3, 0, fileName.str().c_str(), &err);
    if (ret != TINYEXR_SUCCESS) {
      if (err) {
        std::cerr << "ERR: " << err;
        FreeEXRErrorMessage(err);
      }
      THROW_RUNTIME_ERROR("Could not save image " + fileName.str())
    }
  }
}
