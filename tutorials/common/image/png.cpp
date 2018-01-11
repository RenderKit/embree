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

#ifdef EMBREE_TUTORIALS_LIBPNG

#include "image.h"

#include <iostream>
#include <cstring>
#include <cstdio>
#include <png.h>

namespace embree
{
  struct AutoCloseFile
  {
    FILE* file;
    AutoCloseFile (FILE* file) : file(file) {}
    ~AutoCloseFile () { if (file) fclose(file); }
  };

  /*! read PNG file from disk */
  Ref<Image> loadPNG(const FileName& fileName)
  {
    size_t width, height;

    //header for testing if it is a png
    png_byte header[8];
 
    //open file as binary
    FILE* fp = fopen(fileName.c_str(), "rb");
    if (!fp) THROW_RUNTIME_ERROR("cannot open file "+fileName.str());
    //ON_SCOPE_EXIT(fclose(fp));
    AutoCloseFile close_file(fp);

    //read the header
    if (fread(header, 1, 8, fp) != 8)
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());

    //test if png
    int is_png = !png_sig_cmp(header, 0, 8);
    if (!is_png)
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());

    //create png struct
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
    ON_SCOPE_EXIT(png_destroy_read_struct(&png_ptr, (png_infopp) nullptr, (png_infopp) nullptr));
    
    //create png info struct
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
 
    //create png info struct
    png_infop end_info = png_create_info_struct(png_ptr);
    if (!end_info) 
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
 
    //png error stuff, not sure libpng man suggests this.
    //if (setjmp(png_jmpbuf(png_ptr))) {
    ///  return (TEXTURE_LOAD_ERROR);
    //}
 
    //init png reading
    png_init_io(png_ptr, fp);
 
    //let libpng know you already read the first 8 bytes
    png_set_sig_bytes(png_ptr, 8);
 
    // read all the info up to the image data
    png_read_info(png_ptr, info_ptr);
 
    //variables to pass to get info
    int bit_depth, color_type;
    png_uint_32 twidth, theight;
 
    // get info about png
    png_get_IHDR(png_ptr, info_ptr, &twidth, &theight, &bit_depth, &color_type,
                 nullptr, nullptr, nullptr);
 
    //update width and height based on png info
    width = twidth;
    height = theight;
    
    Ref<Image> img = new Image4uc(width,height,fileName);

    // Update the png info struct.
    png_read_update_info(png_ptr, info_ptr);
 
    // Row size in bytes.
    int rowbytes = png_get_rowbytes(png_ptr, info_ptr);

    // Allocate the image_data as a big block, to be given to opengl
    std::vector<png_byte> data(rowbytes * height);
 
    // row_pointers is for pointing to image_data for reading the png with libpng
    std::vector<png_bytep> row_pointers(height);

    // set the individual row_pointers to point at the correct offsets of image_data
    for (size_t i = 0; i < height; ++i)
      row_pointers[i] = (unsigned char*) &data[i * rowbytes];
 
    // read the png into image_data through row_pointers
    png_read_image(png_ptr, row_pointers.data());
  
    if (color_type == PNG_COLOR_TYPE_RGB && bit_depth == 8)
    {
      for (size_t y=0;y<height;y++)
        for (size_t x=0;x<width;x++)
        {
          unsigned char* texel = data.data() + (y * width + x) * 3;
          Color4 c( (float)texel[0] * 1.0f/255.0f, (float)texel[1] * 1.0f/255.0f, (float)texel[2] * 1.0f/255.0f, 0.0f );
          img.ptr->set(x,y,c);
        }
    }
    else if (color_type == PNG_COLOR_TYPE_RGBA && bit_depth == 8)
    {
      for (size_t y=0;y<height;y++)
        for (size_t x=0;x<width;x++)
        {
          unsigned char *texel = data.data() + (y * width + x) * 4;
          Color4 c( (float)texel[0] * 1.0f/255.0f, (float)texel[1] * 1.0f/255.0f, (float)texel[2] * 1.0f/255.0f, (float)texel[3] * 1.0f/255.0f );
          img.ptr->set(x,y,c);
        }
    }
    else
      THROW_RUNTIME_ERROR("invalid color type in PNG file "+fileName.str());
      
    return img;
  }
}

#endif

