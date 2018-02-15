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

#include <iostream>
#include <cstring>
#include <cstdio>

namespace embree
{
  static void skipSpacesAndComments(std::fstream& file)
  {
    while (true)
    {
      if  (isspace(file.peek())) {
        file.ignore();
      } else  if (file.peek() == '#') {
        std::string line; std::getline(file,line);
      } else break;
    }
  }

  /*! read PFM file from disk */
  Ref<Image> loadPFM(const FileName& fileName)
  {
    /* open file for reading */
    std::fstream file;
    file.exceptions (std::fstream::failbit | std::fstream::badbit);
    file.open (fileName.c_str(), std::fstream::in | std::fstream::binary);

    /* read file type */
    char cty[2]; file.read(cty,2);
    skipSpacesAndComments(file);
    std::string type(cty,2);

    /* read width, height, and maximum color value */
    int width; file >> width;
    skipSpacesAndComments(file);
    int height; file >> height;
    skipSpacesAndComments(file);
    float maxColor; file >> maxColor;
    if (maxColor > 0) THROW_RUNTIME_ERROR("Big endian PFM files not supported");
    float rcpMaxColor = -1.0f/float(maxColor);
    file.ignore(); // skip space or return

    /* create image and fill with data */
    Ref<Image> img = new Image4f(width,height,fileName);

    /* image in binary format 16 bit */
    if (type == "PF")
    {
      float rgb[3];
      for (ssize_t y=height-1; y>=0; y--) {
        for (ssize_t x=0; x<width; x++) {
          file.read((char*)rgb,sizeof(rgb));
          img->set(x,y,Color4(rgb[0]*rcpMaxColor,rgb[1]*rcpMaxColor,rgb[2]*rcpMaxColor,1.0f));
        }
      }
    }

    /* invalid magic value */
    else {
      THROW_RUNTIME_ERROR("Invalid magic value in PFM file");
    }
    return img;
  }

  /*! store PFM file to disk */
  void storePFM(const Ref<Image>& img, const FileName& fileName)
  {
    /* open file for writing */
    std::fstream file;
    file.exceptions (std::fstream::failbit | std::fstream::badbit);
    file.open (fileName.c_str(), std::fstream::out | std::fstream::binary);

    /* write file header */
    file << "PF" << std::endl;
    file << img->width << " " << img->height << std::endl;
    file << -1.0f << std::endl;

    /* write image */
    for (ssize_t y=img->height-1; y>=0; y--) {
      for (ssize_t x=0; x<(ssize_t)img->width; x++) {
        const Color4 c = img->get(x,y);
        file.write((char*)&c,3*sizeof(float));
      }
    }
  }
}
