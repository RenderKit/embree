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
#include "image/image.h"

#if defined(USE_PTEX)
#include "Ptexture.h"
#else
#warning no ptex
#endif

#include <map>
#include <string>

namespace embree
{
  bool isPowerOf2 (unsigned int x)
  {
    while (((x % 2) == 0) && x > 1)
      x /= 2;
    return (x == 1);
  }

  /*! read png texture from disk */
  OBJScene::Texture *loadTexture(const FileName& fileName)
  {
    OBJScene::Texture *texture = new OBJScene::Texture();
    
    Ref<Image> img = loadImage(fileName);

    texture->width         = img.ptr->width;
    texture->height        = img.ptr->height;    
    texture->format        = OBJScene::Texture::RGBA8;
    texture->bytesPerTexel = 4;
    texture->data          = _mm_malloc(sizeof(int)*texture->width*texture->height,64);
    texture->width_mask    = isPowerOf2(texture->width) ? texture->width-1 : 0;
    texture->height_mask   = isPowerOf2(texture->height) ? texture->height-1 : 0;
    img.ptr->convertToRGBA8((unsigned char*)texture->data);
    return texture;
  }
  
#if defined(USE_PTEX)

  static std::map<std::string, ptex_file*> ptex_files;

  ptex_file::ptex_file(FileName filename, int faces) 
  : faces(faces), diffuse(0)
  {
    diffuse = new face_texture<uchar3>[faces];
    memset(diffuse,0,sizeof(face_texture<uchar3>)*faces);
  }

  ptex_file::~ptex_file() {
	  delete [] diffuse; diffuse = 0;
  }
      
  ptex_file* loadPtexFile(const FileName &filename)
  {
    std::string fn = filename.str();
    ptex_file *data = ptex_files[fn];
    if (data) 
      {
	PRINT("FOUND");
	return data;
      }

    Ptex::String error;
    std::cout << "opening " << fn << " ... " << std::flush;
    PtexTexture* tex = PtexTexture::open(fn.c_str(),error);
    if (!tex) {
      std::cout << "[FAILED]" << std::endl;
      THROW_RUNTIME_ERROR("cannot open ptex file: "+fn);
    }

    PtexMetaData *metadata = tex->getMetaData();
    const int32_t *vertices_per_face = 0;
    int geom_faces = 0;
    metadata->getValue("PtexFaceVertCounts", vertices_per_face, geom_faces);
    //PRINT(geom_faces);
    //PRINT(vertices_per_face);
    //PRINT(tex->numFaces());

    data = new ptex_file(filename, tex->numFaces());
    //PRINT(data);
    assert(sizeof(uchar3) == 4);
    float px[3];
    int ptex_face_id = 0;
    int obj_face_id = 0;
    int t = 0;
    for (int geom_face_id = 0; geom_face_id < geom_faces; ++geom_face_id) {
      const Ptex::FaceInfo &fi = tex->getFaceInfo(ptex_face_id);
      int nchan = tex->numChannels();
      if (nchan != 3 && nchan != 1) {
	std::cout << "[FAILED]" << std::endl;
	THROW_RUNTIME_ERROR(fn+": ptex file with other than 1 or 3 channels found!");
      }
      int n = vertices_per_face[geom_face_id];
      if (n == 4) {
	Ptex::Res res = fi.res;
			  
	face_texture<uchar3> subtex(res.u(), res.v());
	//PRINT(subtex.w);
	//PRINT(subtex.h);

	if (nchan == 3)
	  {
	    for (int vi = 0; vi < subtex.h; vi++) {
	      for (int ui = 0; ui < subtex.w; ui++) {
		tex->getPixel(ptex_face_id, ui, vi, px, 0, nchan, res);
		subtex.data[vi*subtex.w+ui].x = (unsigned char)(px[0]*255.0f);
		subtex.data[vi*subtex.w+ui].y = (unsigned char)(px[1]*255.0f);
		subtex.data[vi*subtex.w+ui].z = (unsigned char)(px[2]*255.0f);
	      }
	    }
	  }
	else if (nchan == 1)
	  {
	    for (int vi = 0; vi < subtex.h; vi++) {
	      for (int ui = 0; ui < subtex.w; ui++) {
		tex->getPixel(ptex_face_id, ui, vi, px, 0, nchan, res);
		if (!isfinite(px[0]))
		  {
		    px[0] = 0.0f;
		  }
		assert( isfinite(px[0]));
		//px[0] = 1.0f;
		
		*(float*)&subtex.data[vi*subtex.w+ui] = px[0];
	      }
	    }
	  }

	assert(obj_face_id < tex->numFaces());
	data->diffuse[obj_face_id].move_from(subtex);
	ptex_face_id++;
	obj_face_id++;
      }
      else {
	ptex_face_id += 3;
	++t;
      }
    }
    std::cout << "[DONE]" << std::endl;

    /*
      PtexFilter::Options opts(PtexFilter::f_point, 0, 1.0);
      //PtexFilter::Options opts(PtexFilter::f_bicubic, 0, 1.0);
      return PtexFilter::getFilter(tex, opts);
    */

    return data;
  }
#endif

}

