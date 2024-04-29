// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "image.h"
#include "../../../common/sys/estring.h"

#include <map>
#include <iostream>


namespace embree
{
  double compareImages(Ref<Image> image0, Ref<Image> image1)
  {
    /* compare image size */
    const size_t width = image0->width;
    const size_t height = image0->height;
    if (image1->width != width) return inf;
    if (image1->height != height) return inf;

    /* compare both images */
    double diff = 0.0;
    for (size_t y=0; y<height; y++)
    {
      for (size_t x=0; x<width; x++)
      {
        const Color c0 = image0->get(x,y);
        const Color c1 = image1->get(x,y);
        diff += sqr(fabs(c0.r - c1.r))/3.0f;
        diff += sqr(fabs(c0.g - c1.g))/3.0f;
        diff += sqr(fabs(c0.b - c1.b))/3.0f;
      }
    }

    return diff;
  }
  
  /*! loads an image from a file with auto-detection of format */
  Ref<Image> loadImageFromDisk(const FileName& fileName)
  {
    std::string ext = toLowerCase(fileName.ext());

    if (ext == "bmp" ) return loadSTB(fileName);
    if (ext == "png" ) return loadSTB(fileName);
    if (ext == "jpg" ) return loadSTB(fileName);

    if (ext == "exr" ) return loadEXR(fileName);
    
    if (ext == "pfm" ) return loadPFM(fileName);
    if (ext == "ppm" ) return loadPPM(fileName);
    if (ext == "tga" ) return loadTGA(fileName);
    THROW_RUNTIME_ERROR("image format " + ext + " not supported");
  }

  static std::map<std::string,Ref<Image> > image_cache;
  
  /*! loads an image from a file with auto-detection of format */
  Ref<Image> loadImage(const FileName& fileName, bool cache)
  {
    if (!cache)
      return loadImageFromDisk(fileName);

    if (image_cache.find(fileName) == image_cache.end())
      image_cache[fileName] = loadImageFromDisk(fileName);

    return image_cache[fileName];
  }

  /*! stores an image to file with auto-detection of format */
  void storeImage(const Ref<Image>& img, const FileName& fileName)
  {
    std::string ext = toLowerCase(fileName.ext());

    if (ext == "bmp" ) { storeSTB(img, fileName);  return; }
    if (ext == "png" ) { storeSTB(img, fileName);  return; }
    if (ext == "jpg" ) { storeSTB(img, fileName);  return; }

    if (ext == "exr" ) { storeEXR(img, fileName);  return; }

    if (ext == "pfm" ) { storePFM(img, fileName);  return; }
    if (ext == "ppm" ) { storePPM(img, fileName);  return; }
    if (ext == "tga" ) { storeTga(img, fileName);  return; }
    THROW_RUNTIME_ERROR("image format " + ext + " not supported");
  }

  /*! template instantiations */
  template class ImageT<Col3uc>;
  template class ImageT<Col3f>;

}
