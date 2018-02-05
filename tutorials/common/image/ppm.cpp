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
#include <fstream>

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

  /*! read PPM file from disk */
  Ref<Image> loadPPM(const FileName& fileName)
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
    int maxColor; file >> maxColor;
    if (maxColor <= 0) THROW_RUNTIME_ERROR("Invalid maxColor value in PPM file");
    float rcpMaxColor = 1.0f/float(maxColor);
    file.ignore(); // skip space or return

    /* create image and fill with data */
    Ref<Image> img = new Image4uc(width,height,fileName);

    /* image in text format */
    if (type == "P3")
    {
      int r, g, b;
      for (ssize_t y=0; y<height; y++) {
        for (ssize_t x=0; x<width; x++) {
          file >> r; file >> g; file >> b;
          img->set(x,y,Color4(float(r)*rcpMaxColor,float(g)*rcpMaxColor,float(b)*rcpMaxColor,1.0f));
        }
      }
    }

    /* image in binary format 8 bit */
    else if (type == "P6" && maxColor <= 255)
    {
      unsigned char rgb[3];
      for (ssize_t y=0; y<height; y++) {
        for (ssize_t x=0; x<width; x++) {
          file.read((char*)rgb,sizeof(rgb));
          img->set(x,y,Color4(float(rgb[0])*rcpMaxColor,float(rgb[1])*rcpMaxColor,float(rgb[2])*rcpMaxColor,1.0f));
        }
      }
    }

    /* image in binary format 16 bit */
    else if (type == "P6" && maxColor <= 65535)
    {
      unsigned short rgb[3];
      for (ssize_t y=0; y<height; y++) {
        for (ssize_t x=0; x<width; x++) {
          file.read((char*)rgb,sizeof(rgb));
          img->set(x,y,Color4(float(rgb[0])*rcpMaxColor,float(rgb[1])*rcpMaxColor,float(rgb[2])*rcpMaxColor,1.0f));
        }
      }
    }

    /* invalid magic value */
    else {
      THROW_RUNTIME_ERROR("Invalid magic value in PPM file");
    }
    return img;
  }

  /*! store PPM file to disk */
  void storePPM(const Ref<Image>& img, const FileName& fileName)
  {
    /* open file for writing */
    std::fstream file;
    file.exceptions (std::fstream::failbit | std::fstream::badbit);
    file.open (fileName.c_str(), std::fstream::out | std::fstream::binary);

    /* write file header */
    file << "P6" << std::endl;
    file << img->width << " " << img->height << std::endl;
    file << 255 << std::endl;

    /* write image */
    for (size_t y=0; y<img->height; y++) {
      for (size_t x=0; x<img->width; x++) {
        const Color4 c = img->get(x,y);
        file << (unsigned char)(clamp(c.r)*255.0f);
        file << (unsigned char)(clamp(c.g)*255.0f);
        file << (unsigned char)(clamp(c.b)*255.0f);
      }
    }
  }
}
