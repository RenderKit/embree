// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#ifdef USE_OPENIMAGEIO

#include "image.h"
#include <iostream>
#include <vector>

/*! include OpenImageIO headers */
#include <OpenImageIO/imageio.h>
OIIO_NAMESPACE_USING

namespace embree
{
  Ref<Image> loadOIIO(const FileName& fileName)
  {
    std::unique_ptr<ImageInput> in = ImageInput::open(fileName.str().c_str());
    if (!in)
      THROW_RUNTIME_ERROR("error opening file " + fileName.str());

    const ImageSpec& spec = in->spec();
    size_t width = spec.width;
    size_t height = spec.height;
    size_t channels = spec.nchannels;
    std::vector<unsigned char> pixels(width*height*channels);
    in->read_image(TypeDesc::UINT8, pixels.data());
    in->close();

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
        out->set(x, height-y-1, Color4(r,g,b,a)); // flip image
      }
    }

    return out;
  }

  void storeOIIO(const Ref<Image>& img, const FileName& fileName)
  {
    std::unique_ptr<ImageOutput> out = ImageOutput::create(fileName.c_str());
    if (!out) THROW_RUNTIME_ERROR("unsupported output file format " + fileName.str());

    std::vector<unsigned char> pixels(img->width*img->height*3);
    const float maxColor = 255.f;
    for (size_t y = 0; y < img->height; y++)
    {
      for (size_t x = 0; x < img->width; x++)
      {
        Color4 c = img->get(x, y);
        pixels[(y*img->width+x)*3+0] = (unsigned char)(clamp(c.r) * maxColor);
        pixels[(y*img->width+x)*3+1] = (unsigned char)(clamp(c.g) * maxColor);
        pixels[(y*img->width+x)*3+2] = (unsigned char)(clamp(c.b) * maxColor);
      }
    }

    ImageSpec spec(int(img->width), int(img->height), 3, TypeDesc::UINT8);
    if (!out->open(fileName.c_str(), spec))
    {
      THROW_RUNTIME_ERROR("error opening file " + fileName.str());
    }
    out->write_image(TypeDesc::UINT8, pixels.data());
    out->close();
  }
}

#endif // USE_OPENIMAGEIO

