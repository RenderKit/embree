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

#ifdef USE_LIBPNG

#include "image.h"

#include <iostream>
#include <cstring>
#include <cstdio>
#include <png.h>

namespace embree
{
  /*! read PNG file from disk */
  Ref<Image> loadPNG(const FileName& fileName)
  {
    size_t width, height;

    //header for testing if it is a png
    png_byte header[8];
 
    //open file as binary
    FILE* fp = fopen(fileName.c_str(), "rb");
    if (!fp) 
      THROW_RUNTIME_ERROR("cannot open file "+fileName.str());

    //read the header
    fread(header, 1, 8, fp);
 
    //test if png
    int is_png = !png_sig_cmp(header, 0, 8);
    if (!is_png) {
      fclose(fp);
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
    }

    //create png struct
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                                 nullptr, nullptr);
    if (!png_ptr) {
      fclose(fp);
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
    }
 
    //create png info struct
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
      png_destroy_read_struct(&png_ptr, (png_infopp) nullptr, (png_infopp) nullptr);
      fclose(fp);
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
    }
 
    //create png info struct
    png_infop end_info = png_create_info_struct(png_ptr);
    if (!end_info) {
      png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp) nullptr);
      fclose(fp);
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
    }
 
    //png error stuff, not sure libpng man suggests this.
    //if (setjmp(png_jmpbuf(png_ptr))) {
    //  png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
    //  fclose(fp);
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
    unsigned char *data = new png_byte[rowbytes * height];
    if (!data) {
      //clean up memory and close stuff
      png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
      fclose(fp);
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
      return img;
    }
 
    // row_pointers is for pointing to image_data for reading the png with libpng
    png_bytep *row_pointers = new png_bytep[height];
    if (!row_pointers) {
      //clean up memory and close stuff
      png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
      delete[] data;
      fclose(fp);
      THROW_RUNTIME_ERROR("invalid PNG file "+fileName.str());
    }

    // set the individual row_pointers to point at the correct offsets of image_data
    for (int i = 0; i < height; ++i)
      row_pointers[height - 1 - i] = (unsigned char*)data + i * rowbytes;
 
    // read the png into image_data through row_pointers
    png_read_image(png_ptr, row_pointers);
  
    // clean up memory and close stuff
    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
    delete[] row_pointers;
    fclose(fp);


    if (color_type == PNG_COLOR_TYPE_RGB && bit_depth == 8)
      {
	for (size_t y=0;y<height;y++)
	  for (size_t x=0;x<width;x++)
	    {
	      unsigned char *texel = data + (y * width + x) * 3;
	      Color4 c( (float)texel[0] * 1.0f/255.0f, (float)texel[1] * 1.0f/255.0f, (float)texel[2] * 1.0f/255.0f, 0.0f );
	      img.ptr->set(x,y,c);
	    }
      }
    else if (color_type == PNG_COLOR_TYPE_RGBA && bit_depth == 8)
      {
	for (size_t y=0;y<height;y++)
	  for (size_t x=0;x<width;x++)
	    {
	      unsigned char *texel = data + (y * width + x) * 4;
	      Color4 c( (float)texel[0] * 1.0f/255.0f, (float)texel[1] * 1.0f/255.0f, (float)texel[2] * 1.0f/255.0f, (float)texel[3] * 1.0f/255.0f );
	      img.ptr->set(x,y,c);
	    }
      }
    else
      THROW_RUNTIME_ERROR("invalid color type in PNG file "+fileName.str());

    delete[] data;
      
    return img;
  }
}

#endif

