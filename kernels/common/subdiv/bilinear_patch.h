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

#include "catmullclark_patch.h"

namespace embree
{
  template<typename Vertex, typename Vertex_t = Vertex>
    class __aligned(64) BilinearPatchT
    {
      typedef CatmullClark1RingT<Vertex,Vertex_t> CatmullClarkRing;
      typedef CatmullClarkPatchT<Vertex,Vertex_t> CatmullClarkPatch;
      
    public:
      Vertex v[4];
      
    public:
      
      __forceinline BilinearPatchT () {}

      __forceinline BilinearPatchT (const CatmullClarkPatch& patch) 
      {
        v[0] = patch.ring[0].getLimitVertex();
        v[1] = patch.ring[1].getLimitVertex();
        v[2] = patch.ring[2].getLimitVertex();
        v[3] = patch.ring[3].getLimitVertex();
      }

      __forceinline BBox<Vertex> bounds() const
      {
        
        BBox<Vertex> bounds (v[0]);
        bounds.extend(v[1]);
        bounds.extend(v[2]);
        bounds.extend(v[3]);
        return bounds;
      }
      
      __forceinline Vertex eval(const float uu, const float vv) const
      {
        const float sx1 = uu, sx0 = 1.0f-sx1;
        const float sy1 = vv, sy0 = 1.0f-sy1;
        return sy0*(sx0*v[0]+sx1*v[1]) + sy1*(sx0*v[3]+sx1*v[2]);
      }

      __forceinline Vertex tangentU(const float uu, const float vv) const
      {
        const float sx1 = uu, sx0 = 1.0f-sx1;
        const float sy1 = vv, sy0 = 1.0f-sy1;
        return sy0*(v[1]-v[0]) + sy1*(v[2]-v[3]); 
      }

      __forceinline Vertex tangentV(const float uu, const float vv) const
      {
        const float sx1 = uu, sx0 = 1.0f-sx1;
        const float sy1 = vv, sy0 = 1.0f-sy1;
        return sx0*(v[3]-v[0]) + sx1*(v[2]-v[1]);
      }
      
      __forceinline void eval(const float u, const float v, Vertex* P, Vertex* dPdu, Vertex* dPdv, const float dscale = 1.0f) const
      {
        if (P)    *P    = eval(u,v); 
        if (dPdu) *dPdu = tangentU(u,v)*dscale; 
        if (dPdv) *dPdv = tangentV(u,v)*dscale; 
      }

       template<class vfloat>
      __forceinline vfloat eval(const size_t i, const vfloat& uu, const vfloat& vv) const
      {
        const vfloat sx1 = uu, sx0 = 1.0f-sx1;
        const vfloat sy1 = vv, sy0 = 1.0f-sy1;
        return sy0*(sx0*v[0][i]+sx1*v[1][i]) + sy1*(sx0*v[3][i]+sx1*v[2][i]);
      }

      template<class vfloat>
      __forceinline vfloat tangentU(const size_t i, const vfloat uu, const vfloat& vv) const
      {
        const vfloat sx1 = uu, sx0 = 1.0f-sx1;
        const vfloat sy1 = vv, sy0 = 1.0f-sy1;
        return sy0*(v[1]-v[0]) + sy1*(v[2]-v[3]); 
      }

      template<class vfloat>
      __forceinline vfloat tangentV(const size_t i, const vfloat& uu, const vfloat& vv) const
      {
        const vfloat sx1 = uu, sx0 = 1.0f-sx1;
        const vfloat sy1 = vv, sy0 = 1.0f-sy1;
        return sx0*(v[3][i]-v[0][i]) + sx1*(v[2][i]-v[1][i]);
      }

      template<class vfloat>
      __forceinline void eval(const size_t N, const vfloat uu, const vfloat vv, vfloat* P, vfloat* dPdu, vfloat* dPdv, const float dscale = 1.0f) const
      {
        if (P) {
          for (size_t i=0; i<N; i++) P[i] = eval(i,uu,vv);
        }
        if (dPdu) {
          for (size_t i=0; i<N; i++) P[i] = tangentU(i,uu,vv)*dscale;
        }
        if (dPdv) {
          for (size_t i=0; i<N; i++) P[i] = tangentV(i,uu,vv)*dscale;
        }
      }

    };
  
  typedef BilinearPatchT<Vec3fa,Vec3fa_t> BilinearPatch3fa;
}
