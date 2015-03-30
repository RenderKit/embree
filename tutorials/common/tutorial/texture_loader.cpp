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

#include "texture_loader.h"
#include <png.h>

namespace embree
{
  /*! read png texture from disk */
  OBJScene::Texture *loadTextureFromPNG(const FileName& fileName)
  {
    //Texture texture;
    PRINT(fileName);

    OBJScene::Texture *texture = new OBJScene::Texture();
    
    //header for testing if it is a png
    png_byte header[8];
 
    //open file as binary
    FILE *fp = fopen(fileName.c_str(), "rb");
    if (!fp) {
      FATAL("can't load texture, code 0");
      return NULL;
    }

    //read the header
    fread(header, 1, 8, fp);
 
    //test if png
    int is_png = !png_sig_cmp(header, 0, 8);
    if (!is_png) {
      fclose(fp);
       FATAL("can't load texture, code 1");
      return NULL;
    }

    //create png struct
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
                                                 NULL, NULL);
    if (!png_ptr) {
      fclose(fp);
      FATAL("can't load texture, code 2");      
      return NULL;
    }
 
    //create png info struct
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
      png_destroy_read_struct(&png_ptr, (png_infopp) NULL, (png_infopp) NULL);
      fclose(fp);
      FATAL("can't load texture, code 3");
      return NULL;
    }
 
    //create png info struct
    png_infop end_info = png_create_info_struct(png_ptr);
    if (!end_info) {
      png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp) NULL);
      fclose(fp);
      FATAL("can't load texture, code 4");     
      return NULL;
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
                 NULL, NULL, NULL);
 
    //update width and height based on png info
    texture->width = twidth;
    texture->height = theight;

    PRINT(texture->width);
    PRINT(texture->height);
    
 
    // Update the png info struct.
    png_read_update_info(png_ptr, info_ptr);
 
    // Row size in bytes.
    int rowbytes = png_get_rowbytes(png_ptr, info_ptr);
 
    // Allocate the image_data as a big block, to be given to opengl
    texture->data = new png_byte[rowbytes * texture->height];
    if (!texture->data) {
      //clean up memory and close stuff
      png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
      fclose(fp);
      FATAL("can't load texture, code 5");
      return texture;
    }
 
    //row_pointers is for pointing to image_data for reading the png with libpng
    png_bytep *row_pointers = new png_bytep[texture->height];
    if (!row_pointers) {
      //clean up memory and close stuff
      png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
      delete[] texture->data;
      fclose(fp);
      FATAL("can't load texture, code 6");
      return NULL;
    }
    // set the individual row_pointers to point at the correct offsets of image_data
    for (int i = 0; i < texture->height; ++i)
      row_pointers[texture->height - 1 - i] = (unsigned char*)texture->data + i * rowbytes;
 
    //read the png into image_data through row_pointers
    png_read_image(png_ptr, row_pointers);
  
    //clean up memory and close stuff
    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
    delete[] row_pointers;
    fclose(fp);
    
    return texture;
  }

};
