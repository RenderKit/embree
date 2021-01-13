// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "image.h"
#include "../../../common/sys/string.h"

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

#ifdef EMBREE_TUTORIALS_LIBPNG
    if (ext == "png" ) return loadPNG(fileName);
#endif

#ifdef EMBREE_TUTORIALS_LIBJPEG
    if (ext == "jpg" ) return loadJPEG(fileName);
#endif

#ifdef USE_OPENIMAGEIO
    if (ext == "bmp" ) return loadOIIO(fileName);
    if (ext == "gif" ) return loadOIIO(fileName);
    if (ext == "tga" ) return loadOIIO(fileName);
    if (ext == "tif" ) return loadOIIO(fileName);
    if (ext == "tiff") return loadOIIO(fileName);
    if (ext == "png" ) return loadOIIO(fileName);
    if (ext == "jpg" ) return loadOIIO(fileName);
#endif
    
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

#ifdef EMBREE_TUTORIALS_LIBJPEG
    if (ext == "jpg" ) { storeJPEG(img, fileName);  return; }
#endif

#ifdef USE_OPENIMAGEIO
    if (ext == "bmp" ) { storeOIIO(img, fileName);  return; }
    if (ext == "gif" ) { storeOIIO(img, fileName);  return; }
    if (ext == "tga" ) { storeOIIO(img, fileName);  return; }
    if (ext == "tif" ) { storeOIIO(img, fileName);  return; }
    if (ext == "tiff") { storeOIIO(img, fileName);  return; }
    if (ext == "png" ) { storeOIIO(img, fileName);  return; }
    if (ext == "jpg" ) { storeOIIO(img, fileName);  return; }
#endif

    if (ext == "pfm" ) { storePFM(img, fileName);  return; }
    if (ext == "ppm" ) { storePPM(img, fileName);  return; }
    if (ext == "tga" ) { storeTga(img, fileName);  return; }
    THROW_RUNTIME_ERROR("image format " + ext + " not supported");
  }

  /*! template instantiations */
  template class ImageT<Col3uc>;
  template class ImageT<Col3f>;

}
