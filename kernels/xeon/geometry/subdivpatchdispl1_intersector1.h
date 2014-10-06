// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "subdivpatchdispl1.h"
#include "common/ray.h"
#include "geometry/filter.h"

namespace embree
{
  template<bool list>
  struct SubdivPatchDispl1Intersector1
  {
    typedef SubdivPatchDispl1::QuadQuad4x4 Primitive;

    struct Precalculations 
    {
      __forceinline Precalculations (const Ray& ray) 
      {
#if defined (__AVX__)

      /*! load the ray into SIMD registers */
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      org = avx3f(ray.org.x,ray.org.y,ray.org.z);
      const avx3f dir(ray.dir.x,ray.dir.y,ray.dir.z);
      rdir = avx3f(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      org_rdir = avx3f(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      ray_tnear = ray.tnear;
      //const avxf  ray_tfar(ray.tfar);
      
      /*! offsets to select the side that becomes the lower or upper bound */
      nearX = ray_rdir.x >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      nearY = ray_rdir.y >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);

#else

      /*! load the ray into SIMD registers */
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      org = sse3f(ray.org.x,ray.org.y,ray.org.z);
      const sse3f dir(ray.dir.x,ray.dir.y,ray.dir.z);
      rdir = sse3f(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      org_rdir = sse3f(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      ray_tnear = ray.tnear;
      //const ssef  ray_tfar(ray.tfar);
      
      /*! offsets to select the side that becomes the lower or upper bound */
      nearX = ray_rdir.x >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      nearY = ray_rdir.y >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);

#endif
      }

#if defined (__AVX__)
      avx3f org;
      avx3f rdir;
      avx3f org_rdir;
      avxf ray_tnear;
      size_t nearX;
      size_t nearY;
      size_t nearZ;
#else
      sse3f org;
      sse3f rdir;
      sse3f org_rdir;
      ssef ray_tnear;
      size_t nearX;
      size_t nearY;
      size_t nearZ;
#endif
    };

    /*! Intersect a ray with the triangle and updates the hit. */
    static __forceinline void intersectTriangle(Ray& ray, const Vec3fa& tri_v0, const Vec3fa& tri_v1, const Vec3fa& tri_v2, const unsigned geomID, const unsigned primID)
    {
      /* calculate vertices relative to ray origin */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      const Vec3fa v0 = tri_v0-O;
      const Vec3fa v1 = tri_v1-O;
      const Vec3fa v2 = tri_v2-O;
        
      /* calculate triangle edges */
      const Vec3fa e0 = v2-v0;
      const Vec3fa e1 = v0-v1;
      const Vec3fa e2 = v1-v2;
      
      /* calculate geometry normal and denominator */
      const Vec3fa Ng1 = cross(e1,e0);
      const Vec3fa Ng = Ng1+Ng1;
      const float den = dot(Ng,D);
      const float absDen = abs(den);
      const float sgnDen = signmsk(den);
      
      /* perform edge tests */
      const float U = xorf(dot(cross(v2+v0,e0),D),sgnDen);
      if (unlikely(U < 0.0f)) return;
      const float V = xorf(dot(cross(v0+v1,e1),D),sgnDen);
      if (unlikely(V < 0.0f)) return;
      const float W = xorf(dot(cross(v1+v2,e2),D),sgnDen);
      if (unlikely(W < 0.0f)) return;
      
      /* perform depth test */
      const float T = xorf(dot(v0,Ng),sgnDen);
      if (unlikely(absDen*float(ray.tfar) < T)) return;
      if (unlikely(T < absDen*float(ray.tnear))) return;

      /* calculate hit information */
      const float rcpAbsDen = rcp(absDen);
      const float u = U * rcpAbsDen;
      const float v = V * rcpAbsDen;
      const float t = T * rcpAbsDen;
      
      /* update hit information */
      ray.u = u;
      ray.v = v;
      ray.tfar = t;
      ray.Ng  = Ng;
      ray.geomID = geomID;
      ray.primID = primID;
    }

    static __forceinline void intersectFinish (Ray& ray, 
                                               const Vec3fa& q, const Vec3fa& e0, const Vec3fa& e1, const Vec3fa& e2, 
                                               const float U0, const float U1, const float U2,
                                               const unsigned geomID, const unsigned primID)
      {
        const Vec3fa Ng0 = cross(e1,e0);
        const Vec3fa Ng = Ng0+Ng0;
        const float det = dot(ray.dir,Ng);
        const float T   = dot(q,Ng);
        if (unlikely(det*ray.tnear <= T && T <= det*ray.tfar)) {
          const float rcpDet = rcp(det);
          ray.u    = U0 * rcpDet;
          ray.v    = U1 * rcpDet;
          ray.tfar = T * rcpDet;
          ray.Ng   = Ng;
          ray.geomID  = geomID;
          ray.primID  = primID;
        }
      }

    static __forceinline void intersectQuadQuad (Ray& ray, 
                                                 const Vec3fa& v00, const Vec3fa& v10, const Vec3fa& v20,
                                                 const Vec3fa& v01, const Vec3fa& v11, const Vec3fa& v21,
                                                 const Vec3fa& v02, const Vec3fa& v12, const Vec3fa& v22,
                                                 const unsigned geomID, const unsigned primID)
    {
      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      
      const Vec3fa q00 = v00-O, q10 = v10-O, q20 = v20-O;
      const Vec3fa q01 = v01-O, q11 = v11-O, q21 = v21-O;
      const Vec3fa q02 = v02-O, q12 = v12-O, q22 = v22-O;

      const Vec3fa e0001 = q01-q00; const float u0001 = dot(cross(q01+q00,e0001),D);
      const Vec3fa e0102 = q02-q01; const float u0102 = dot(cross(q02+q01,e0102),D);
      const Vec3fa e1011 = q11-q10; const float u1011 = dot(cross(q11+q10,e1011),D);
      const Vec3fa e1112 = q12-q11; const float u1112 = dot(cross(q12+q11,e1112),D);
      const Vec3fa e2021 = q21-q20; const float u2021 = dot(cross(q21+q20,e2021),D);
      const Vec3fa e2122 = q22-q21; const float u2122 = dot(cross(q22+q21,e2122),D);
      
      const Vec3fa e0010 = q10-q00; const float u0010 = dot(cross(q10+q00,e0010),D);
      const Vec3fa e1020 = q20-q10; const float u1020 = dot(cross(q20+q10,e1020),D);
      const Vec3fa e0111 = q11-q01; const float u0111 = dot(cross(q11+q01,e0111),D);
      const Vec3fa e1121 = q21-q11; const float u1121 = dot(cross(q21+q11,e1121),D);
      const Vec3fa e0212 = q12-q02; const float u0212 = dot(cross(q12+q02,e0212),D);
      const Vec3fa e1222 = q22-q12; const float u1222 = dot(cross(q22+q12,e1222),D);
      
      const Vec3fa e0011 = q11-q00; const float u0011 = dot(cross(q11+q00,e0011),D);
      const Vec3fa e0112 = q12-q01; const float u0112 = dot(cross(q12+q01,e0112),D);
      const Vec3fa e1021 = q21-q10; const float u1021 = dot(cross(q21+q10,e1021),D);
      const Vec3fa e1122 = q22-q11; const float u1122 = dot(cross(q22+q11,e1122),D);

      if (u0011 >= 0.0f) {
        if (u0001 <= 0.0f && u0111 <= 0.0f) intersectFinish(ray,q00,-e0001,-e0111,+e0011,-u0001,-u0111,+u0011,geomID,primID);
      } else {
        if (u1011 >= 0.0f && u0010 >= 0.0f) intersectFinish(ray,q00,-e0011,+e1011,+e0010,-u0011,+u1011,+u0010,geomID,primID);
      }
      
      if (u0112 >= 0.0f) {
        if (u0102 <= 0.0f && u0212 <= 0.0f) intersectFinish(ray,q01,-e0102,-e0212,+e0112,-u0102,-u0212,+u0112,geomID,primID);
      } else {
        if (u1112 >= 0.0f && u0111 >= 0.0f) intersectFinish(ray,q01,-e0112,+e1112,+e0111,-u0112,+u1112,+u0111,geomID,primID);
      }
      
      if (u1021 >= 0.0f) {
        if (u1011 <= 0.0f && u1121 <= 0.0f) intersectFinish(ray,q10,-e1011,-e1121,+e1021,-u1011,-u1121,+u1021,geomID,primID);
      } else {
        if (u2021 >= 0.0f && u1020 >= 0.0f) intersectFinish(ray,q10,-e1021,+e2021,+e1020,-u1021,+u2021,+u1020,geomID,primID);
      }
      
      if (u1122 >= 0.0f) {
        if (u1112 <= 0.0f && u1222 <= 0.0f) intersectFinish(ray,q11,-e1112,-e1222,+e1122,-u1112,-u1222,+u1122,geomID,primID);
      } else {
        if (u2122 >= 0.0f && u1121 >= 0.0f) intersectFinish(ray,q11,-e1122,+e2122,+e1121,-u1122,+u2122,+u1121,geomID,primID);
      }
    }
    
    /*! Intersect a ray with the triangle and updates the hit. */
    static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& prim, const void* geom)
    {
      STAT3(normal.trav_prims,1,1,1);
      const size_t bx = prim.bx;
      const size_t by = prim.by;

#if defined (__AVX__)

      /* perform box tests */
      const avxf ray_tfar(ray.tfar);
      size_t mask = prim.n.intersect<false>(pre.nearX, pre.nearY, pre.nearZ, pre.org, pre.rdir, pre.org_rdir, pre.ray_tnear, ray_tfar);

#else

      /* perform box tests */
      const ssef  ray_tfar(ray.tfar);
      size_t mask = prim.n.intersect<false>(pre.nearX, pre.nearY, pre.nearZ, pre.org, pre.rdir, pre.org_rdir, pre.ray_tnear, ray_tfar);

#endif

      /* intersect quad-quads */
      while (mask) {
        size_t i = __bscf(mask);
        const size_t x = 2*(i&1) + (i&4);
        const size_t y = (i&2) + ((i&8) >> 1); 
        const Vec3fa& v00 = prim.vertices(bx+x+0,by+y+0);
        const Vec3fa& v10 = (&v00)[1]; //prim.vertices(bx+x+1,by+y+0);
        const Vec3fa& v20 = (&v00)[2]; //prim.vertices(bx+x+2,by+y+0);
        const Vec3fa& v01 = prim.vertices(bx+x+0,by+y+1);
        const Vec3fa& v11 = (&v01)[1]; //prim.vertices(bx+x+1,by+y+1);
        const Vec3fa& v21 = (&v01)[2]; //prim.vertices(bx+x+2,by+y+1);
        const Vec3fa& v02 = prim.vertices(bx+x+0,by+y+2);
        const Vec3fa& v12 = (&v02)[1]; //prim.vertices(bx+x+1,by+y+2);
        const Vec3fa& v22 = (&v02)[2]; //prim.vertices(bx+x+2,by+y+2);
        intersectQuadQuad(ray,v00,v10,v20,v01,v11,v21,v02,v12,v22,prim.geomID,prim.primID);
      }
    }
    
    /*! Test if the ray is occluded by the primitive */
    static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& subdiv_patch, const void* geom)
    {
      STAT3(shadow.trav_prims,1,1,1);
      return false;
    }
  };
}
