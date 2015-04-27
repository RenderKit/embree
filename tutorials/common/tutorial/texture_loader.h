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

#pragma once

#include "sys/platform.h"
#include "sys/filename.h"
#include "sys/vector.h"
#include "math/vec2.h"
#include "math/vec3.h"
#include "math/affinespace.h"
#include "scene.h"

#include <vector>
#include <memory>

namespace embree
{
  /*! read texture from disk */
  OBJScene::Texture *loadTexture(const FileName& fileName);
   
  //! maybe we don't want this type.
  struct uchar3 { unsigned char x, y, z, w; };

  template<typename T> struct face_texture {
  face_texture() : w(0), h(0), data(0) {}
  face_texture(int w, int h) : w(w), h(h), data(0) { data = new T[w*h]; }
    ~face_texture() { delete [] data; data = 0; }
    void move_from(face_texture<T> &src) {
      w = src.w;
      h = src.h;
      data = src.data;
      src.data = 0;
      src.w = src.h = 0;
    }
    int w, h;
    T *data;
  };

  //! representation of a single ptex file. can be shared between different meshes.
  struct ptex_file {
    ptex_file(FileName filename, int faces);
    ~ptex_file();

    __forceinline float getNearest1f(int faceid, float u, float v) {
      assert(diffuse);
      face_texture<uchar3> *tex = diffuse+faceid;
      u = min(max(u,0.0f),1.0f);
      v = min(max(v,0.0f),1.0f);

      int ui = min((int)floorf((tex->w-1)*u),tex->w-1);
      int vi = min((int)floorf((tex->h-1)*v),tex->h-1);

      float d = *(float*)&tex->data[vi*tex->w + ui];
      assert( isfinite(d));

      return d;
    }


    __forceinline Vec3fa getTexel(const face_texture<uchar3> * const tex,
				  const int ui0,
				  const int vi0)
    {
      const int offset = vi0*tex->w + ui0;
      return Vec3fa((float)tex->data[offset].x,
		    (float)tex->data[offset].y,
		    (float)tex->data[offset].z,
		    0.0f) * 1.0f/255.0f;      
    }

    __forceinline Vec3fa getNearest3f(const int faceid, float u, float v) 
    {
      const face_texture<uchar3> * const tex = diffuse+faceid;
      u = min(max(u,0.0f),1.0f);
      v = min(max(v,0.0f),1.0f);
      u = (tex->w-1)*u;
      v = (tex->h-1)*v;
      const float u_f = floorf(u);
      const float v_f = floorf(v);
      const float u_ratio = u - u_f;
      const float v_ratio = v - v_f;
      
      const float u_ratio_opp = 1.0f - u_ratio;
      const float v_ratio_opp = 1.0f - v_ratio;

      int ui0 = (int)u_f; 
      int vi0 = (int)v_f; 

      int ui1 = min(ui0+1,tex->w-1);
      int vi1 = min(vi0+1,tex->h-1);

      return \
	(getTexel(tex,ui0,vi0) * u_ratio_opp + getTexel(tex,ui1,vi0) * u_ratio) * v_ratio_opp +
	(getTexel(tex,ui0,vi1) * u_ratio_opp + getTexel(tex,ui1,vi1) * u_ratio) * v_ratio;
    }

    face_texture<uchar3> *diffuse;
    int faces;
  };

  ptex_file* loadPtexFile(const FileName &filename);
}
