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
    ImageInput* in = ImageInput::open(fileName.str().c_str());
    if (!in)
      THROW_RUNTIME_ERROR("error opening file " + fileName.str());

    const ImageSpec& spec = in->spec();
    size_t width = spec.width;
    size_t height = spec.height;
    size_t channels = spec.nchannels;
    std::vector<unsigned char> pixels(width*height*channels);
    in->read_image(TypeDesc::UINT8, pixels.data());
    in->close();
    delete in;
    //ImageInput::destroy(in);

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
    ImageOutput* out = ImageOutput::create(fileName.c_str());
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
      delete out;
      //ImageOutput::destroy(out);
      THROW_RUNTIME_ERROR("error opening file " + fileName.str());
    }
    out->write_image(TypeDesc::UINT8, pixels.data());
    out->close();
    //ImageOutput::destroy(out);
    delete out;
  }
}

#endif // USE_OPENIMAGEIO

