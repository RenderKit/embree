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
#include "../image/image.h"

namespace embree
{
  bool isPowerOf2 (unsigned int x)
  {
    while (((x % 2) == 0) && x > 1)
      x /= 2;
    return (x == 1);
  }

  std::map<std::string,Texture*> texture_cache;

  /*! read png texture from disk */
  Texture* Texture::load(const FileName& fileName)
  {
    if (texture_cache.find(fileName.str()) != texture_cache.end())
      return texture_cache[fileName.str()];

    Ref<Image> img = loadImage(fileName);

    Texture* texture = new Texture();
    texture->width         = img.ptr->width;
    texture->height        = img.ptr->height;    
    texture->format        = Texture::RGBA8;
    texture->bytesPerTexel = 4;
    texture->data          = _mm_malloc(sizeof(int)*texture->width*texture->height,64);
    texture->width_mask    = isPowerOf2(texture->width) ? texture->width-1 : 0;
    texture->height_mask   = isPowerOf2(texture->height) ? texture->height-1 : 0;
    img.ptr->convertToRGBA8((unsigned char*)texture->data);
    return texture;
  }
}
