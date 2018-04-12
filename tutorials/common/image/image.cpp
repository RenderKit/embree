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

#include "image.h"
#include "../../../common/sys/string.h"

#include <map>
#include <iostream>

namespace embree
{
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
