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
#include "../image/image.h"

#if defined(USE_PTEX)
#include "Ptexture.h"
#endif

#include <map>
#include <string>
#include "../../../common/sys/string.h"

namespace embree
{
  bool isPowerOf2 (unsigned int x)
  {
    while (((x % 2) == 0) && x > 1)
      x /= 2;
    return (x == 1);
  }

  /*! read png texture from disk */
  Texture *loadTexture(const FileName& fileName)
  {
    Texture *texture = new Texture();
    
    std::string ext = strlwr(fileName.ext());
    if (ext == "ptx" ) return loadPtexTexture(fileName);

    Ref<Image> img = loadImage(fileName);

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
  
  Texture *loadPtexTexture(const FileName& filename)
  {
#if defined(USE_PTEX)
    std::string fn = filename.str();
    Ptex::String error;
    std::cout << "opening " << fn << " ... " << std::flush;
    PtexTexture* tex = PtexTexture::open(fn.c_str(),error);
    if (!tex) {
      std::cout << "[FAILED]" << std::endl;
      THROW_RUNTIME_ERROR("cannot open ptex file: "+fn);
    }
    
    PtexMetaData *metadata = tex->getMetaData();
    const int32_t *vertices_per_face = nullptr;
    int geom_faces = 0;
    metadata->getValue("PtexFaceVertCounts", vertices_per_face, geom_faces);

    Texture **face_textures = new Texture *[geom_faces];
    for (size_t i=0;i<geom_faces;i++)
      face_textures[i] = nullptr;

    Texture *texture = new Texture();
    texture->width         = 0;
    texture->height        = 0;    
    texture->format        = Texture::PTEX_RGBA8;
    texture->faceTextures  = geom_faces;
    texture->data          = face_textures;
    texture->width_mask    = 0;
    texture->height_mask   = 0;

    int nchan = tex->numChannels();
    if (nchan != 3 && nchan != 1) 
      {
	std::cout << "[FAILED]" << std::endl;
	THROW_RUNTIME_ERROR(fn+": ptex file with other than 1 or 3 channels found!");
      }

    if (nchan == 1)
      texture->format = Texture::PTEX_FLOAT32;

    float px[3];
    int ptex_face_id = 0;

    for (size_t geom_face_id=0;geom_face_id<geom_faces;geom_face_id++)
      {
	face_textures[geom_face_id] = nullptr;
	const Ptex::FaceInfo &fi = tex->getFaceInfo(ptex_face_id);

	int n = vertices_per_face[geom_face_id];
	if (n == 4) /* ptex data only for quads */
	  {
	    Ptex::Res res = fi.res;
			  
	    Texture *face_txt = new Texture();
	    face_txt->width         = res.u();
	    face_txt->height        = res.v();    
	    face_txt->width_mask    =  0;
	    face_txt->height_mask   =  0;
	    face_txt->data          = nullptr;
	    face_textures[geom_face_id] = face_txt;
	  
	    if (nchan == 3) /* rgb color data */
	      {
		face_txt->format        = Texture::RGBA8;
		face_txt->bytesPerTexel = 4;
		unsigned char *data     = new unsigned char[face_txt->bytesPerTexel*face_txt->width*face_txt->height];
		face_txt->data          = data;

		for (int vi = 0; vi < face_txt->height; vi++) {
		  for (int ui = 0; ui < face_txt->width; ui++) {
		    tex->getPixel(ptex_face_id, ui, vi, px, 0, nchan, res);
		    data[(vi*face_txt->width+ui)*4+0] = (unsigned char)(px[0]*255.0f);
		    data[(vi*face_txt->width+ui)*4+1] = (unsigned char)(px[1]*255.0f);
		    data[(vi*face_txt->width+ui)*4+2] = (unsigned char)(px[2]*255.0f);
		  }
		}
	      }
	    else if (nchan == 1) /* displacement data */
	      {
		face_txt->format        = Texture::FLOAT32;
		face_txt->bytesPerTexel = 4;
		float*data              = new float[face_txt->width*face_txt->height];
		face_txt->data          = data;

		for (int vi = 0; vi < face_txt->height; vi++) {
		  for (int ui = 0; ui < face_txt->width; ui++) {
		    tex->getPixel(ptex_face_id, ui, vi, px, 0, nchan, res);
		    if (!isfinite(px[0]))
		      px[0] = 0.0f;
		    data[vi*face_txt->width+ui] = px[0];
		  }
		}
	      }
	    ptex_face_id++;
	  }
	else 
	    ptex_face_id += 3;
      }
    std::cout << "done" << std::endl << std::flush;
    return texture;	
#else
    return nullptr;
#endif
  }

}

