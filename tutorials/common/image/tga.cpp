// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "image.h"

#include <fstream>

namespace embree
{
  inline void fwrite_uchar (unsigned char  v, std::fstream& file) { file.write((const char*)&v,sizeof(v)); }
  inline void fwrite_ushort(unsigned short v, std::fstream& file) { file.write((const char*)&v,sizeof(v)); }

  void storeTga(const Ref<Image>& img, const FileName& fileName)
  {
    /* open file for reading */
    std::fstream file;
    file.exceptions (std::fstream::failbit | std::fstream::badbit);
    file.open (fileName.c_str(), std::fstream::out | std::fstream::binary);

    fwrite_uchar(0x00, file);
    fwrite_uchar(0x00, file);
    fwrite_uchar(0x02, file);
    fwrite_ushort(0x0000, file);
    fwrite_ushort(0x0000, file);
    fwrite_uchar(0x00, file);
    fwrite_ushort(0x0000, file);
    fwrite_ushort(0x0000, file);
    fwrite_ushort((unsigned short)img->width , file);
    fwrite_ushort((unsigned short)img->height, file);
    fwrite_uchar(0x18, file);
    fwrite_uchar(0x20, file);

    for (size_t y=0; y<img->height; y++) {
      for (size_t x=0; x<img->width; x++) {
        Color c = img->get(x,y);
        fwrite_uchar((unsigned char)(clamp(c.b)*255.0f), file);
        fwrite_uchar((unsigned char)(clamp(c.g)*255.0f), file);
        fwrite_uchar((unsigned char)(clamp(c.r)*255.0f), file);
      }
    }
  }

  inline unsigned char  fread_uchar (std::fstream& file) { unsigned char  v; file.read((char*)&v,sizeof(v)); return v; }
  inline unsigned short fread_ushort(std::fstream& file) { unsigned short v; file.read((char*)&v,sizeof(v)); return v; }
  
  /*! read TGA file from disk */
  Ref<Image> loadTGA(const FileName& fileName)
  {
    /* open file for reading */
    std::fstream file;
    file.exceptions (std::fstream::failbit | std::fstream::badbit);
    file.open (fileName.c_str(), std::fstream::in | std::fstream::binary);

    unsigned char idlength = fread_uchar(file);
    if (idlength != 0) THROW_RUNTIME_ERROR("unsupported TGA file");

    unsigned char colormaptype = fread_uchar(file);
    if (colormaptype != 0) THROW_RUNTIME_ERROR("unsupported TGA file");

    unsigned char datatype = fread_uchar(file);
    if (datatype != 2) THROW_RUNTIME_ERROR("unsupported TGA file");

    unsigned short cmo = fread_ushort(file);
    unsigned short cml = fread_ushort(file);
    unsigned char  cmd = fread_uchar(file);
    unsigned short xorg = fread_ushort(file);
    unsigned short yorg = fread_ushort(file);
    if (cmo != 0 || cml != 0 || cmd != 0 || xorg != 0 || yorg != 0)
      THROW_RUNTIME_ERROR("unsupported TGA file");

    unsigned short width = fread_ushort(file);
    unsigned short height = fread_ushort(file);
    
    unsigned char bits = fread_uchar(file);
    if (bits != 3*8) THROW_RUNTIME_ERROR("unsupported TGA file bits per pixel");
    
    unsigned char desc = fread_uchar(file);
    if (desc != 0x20) THROW_RUNTIME_ERROR("unsupported TGA file");

    /* create image and fill with data */
    Ref<Image> img = new Image4f(width,height,fileName);

    /* load image data */
    for (size_t y=0; y<height; y++) {
      for (size_t x=0; x<width; x++) {
        const unsigned char b = fread_uchar(file);
        const unsigned char g = fread_uchar(file);
        const unsigned char r = fread_uchar(file);
        img->set(x,y,Color4(r/255.0f,g/255.0f,b/255.0f,1.0f));
      }
    }

    return img;
  }
}
