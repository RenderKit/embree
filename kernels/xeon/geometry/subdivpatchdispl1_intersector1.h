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
#if 1 

#if defined(__AVX__)

      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      const avx3f D8(D.x,D.y,D.z);
      
      const Vec3fa q00 = v00-O, q10 = v10-O, q20 = v20-O;
      const Vec3fa q01 = v01-O, q11 = v11-O, q21 = v21-O;
      const Vec3fa q02 = v02-O, q12 = v12-O, q22 = v22-O;
      
      sse3f p00; transpose((ssef)q00,(ssef)q01,(ssef)q11,(ssef)q10,p00.x,p00.y,p00.z);
      sse3f p10; transpose((ssef)q10,(ssef)q11,(ssef)q21,(ssef)q20,p10.x,p10.y,p10.z);
      const avx3f p00_p10(avxf(p00.x,p10.x),avxf(p00.y,p10.y),avxf(p00.z,p10.z));
      
      const avx3f t000_t100_start = shuffle<0,1,2,0>(p00_p10), t000_t100_end = shuffle<1,2,0,0>(p00_p10);
      const avx3f e000_e100 = t000_t100_end - t000_t100_start;
      const avx3f s000_s100 = t000_t100_end + t000_t100_start;
      const avxf  u000_u100 = dot(cross(e000_e100,s000_s100),D8);
      if (all(ge_mask(Vec3fa(extract<0>(u000_u100)),Vec3fa(0.0f)))) intersectFinish(ray,q00,q01-q00,q11-q01,q00-q11,u000_u100[0],u000_u100[1],u000_u100[2],geomID,primID);
      if (all(ge_mask(Vec3fa(extract<1>(u000_u100)),Vec3fa(0.0f)))) intersectFinish(ray,q10,q11-q10,q21-q11,q10-q21,u000_u100[3],u000_u100[4],u000_u100[5],geomID,primID);

      const avx3f t001_t101_start = shuffle<0,2,3,0>(p00_p10), t001_t101_end = shuffle<2,3,0,0>(p00_p10);
      const avx3f e001_e101 = t001_t101_end - t001_t101_start;
      const avx3f s001_s101 = t001_t101_end + t001_t101_start;
      const avxf  u001_u101 = dot(cross(e001_e101,s001_s101),D8);
      if (all(ge_mask(Vec3fa(extract<0>(u001_u101)),Vec3fa(0.0f)))) intersectFinish(ray,q00,q11-q00,q10-q11,q00-q10,u000_u100[0],u000_u100[1],u000_u100[2],geomID,primID);
      if (all(ge_mask(Vec3fa(extract<1>(u001_u101)),Vec3fa(0.0f)))) intersectFinish(ray,q10,q21-q10,q20-q21,q10-q20,u000_u100[3],u000_u100[4],u000_u100[5],geomID,primID);


      sse3f p01; transpose((ssef)q01,(ssef)q02,(ssef)q12,(ssef)q11,p01.x,p01.y,p01.z);
      sse3f p11; transpose((ssef)q11,(ssef)q12,(ssef)q22,(ssef)q21,p11.x,p11.y,p11.z);
      const avx3f p01_p11(avxf(p01.x,p11.x),avxf(p01.y,p11.y),avxf(p01.z,p11.z));
      
      const avx3f t010_t110_start = shuffle<0,1,2,0>(p01_p11), t010_t110_end = shuffle<1,2,0,0>(p01_p11);
      const avx3f e010_e110 = t010_t110_end - t010_t110_start;
      const avx3f s010_s110 = t010_t110_end + t010_t110_start;
      const avxf  u010_u110 = dot(cross(e010_e110,s010_s110),D8);
      if (all(ge_mask(Vec3fa(extract<0>(u010_u110)),Vec3fa(0.0f)))) intersectFinish(ray,q01,q02-q01,q12-q02,q01-q12,u010_u110[0],u010_u110[1],u010_u110[2],geomID,primID);
      if (all(ge_mask(Vec3fa(extract<1>(u010_u110)),Vec3fa(0.0f)))) intersectFinish(ray,q11,q12-q11,q22-q12,q11-q22,u010_u110[3],u010_u110[4],u010_u110[5],geomID,primID);

      const avx3f t011_t111_start = shuffle<0,2,3,0>(p01_p11), t011_t111_end = shuffle<2,3,0,0>(p01_p11);
      const avx3f e011_e111 = t011_t111_end - t011_t111_start;
      const avx3f s011_s111 = t011_t111_end + t011_t111_start;
      const avxf  u011_u111 = dot(cross(e011_e111,s011_s111),D8);
      if (all(ge_mask(Vec3fa(extract<0>(u011_u111)),Vec3fa(0.0f)))) intersectFinish(ray,q01,q12-q01,q11-q12,q01-q11,u010_u110[0],u010_u110[1],u010_u110[2],geomID,primID);
      if (all(ge_mask(Vec3fa(extract<1>(u011_u111)),Vec3fa(0.0f)))) intersectFinish(ray,q11,q22-q11,q21-q22,q11-q21,u010_u110[3],u010_u110[4],u010_u110[5],geomID,primID);

#else

      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      const sse3f DDDD(D.x,D.y,D.z);
      
      const Vec3fa q00 = v00-O, q10 = v10-O, q20 = v20-O;
      const Vec3fa q01 = v01-O, q11 = v11-O, q21 = v21-O;
      const Vec3fa q02 = v02-O, q12 = v12-O, q22 = v22-O;
      
      sse3f p00; transpose((ssef)q00,(ssef)q01,(ssef)q11,(ssef)q10,p00.x,p00.y,p00.z);
      
      sse3f t000_start = shuffle<0,1,2,0>(p00), t000_end = shuffle<1,2,0,0>(p00);
      sse3f e000 = t000_end - t000_start;
      sse3f s000 = t000_end + t000_start;
      ssef  u000 = dot(cross(e000,s000),DDDD);
      if (all(ge_mask(Vec3fa(u000),Vec3fa(0.0f)))) intersectFinish(ray,q00,q01-q00,q11-q01,q00-q11,u000[0],u000[1],u000[2],geomID,primID);

      sse3f t001_start = shuffle<0,2,3,0>(p00), t001_end = shuffle<2,3,0,0>(p00);
      sse3f e001 = t001_end - t001_start;
      sse3f s001 = t001_end + t001_start;
      ssef  u001 = dot(cross(e001,s001),DDDD);
      if (all(ge_mask(Vec3fa(u001),Vec3fa(0.0f)))) intersectFinish(ray,q00,q11-q00,q10-q11,q00-q10,u001[0],u001[1],u001[2],geomID,primID);

      sse3f p10; transpose((ssef)q10,(ssef)q11,(ssef)q21,(ssef)q20,p10.x,p10.y,p10.z);
      
      sse3f t100_start = shuffle<0,1,2,0>(p10), t100_end = shuffle<1,2,0,0>(p10);
      sse3f e100 = t100_end - t100_start;
      sse3f s100 = t100_end + t100_start;
      ssef  u100 = dot(cross(e100,s100),DDDD);
      if (all(ge_mask(Vec3fa(u100),Vec3fa(0.0f)))) intersectFinish(ray,q10,q11-q10,q21-q11,q10-q21,u100[0],u100[1],u100[2],geomID,primID);

      sse3f t101_start = shuffle<0,2,3,0>(p10), t101_end = shuffle<2,3,0,0>(p10);
      sse3f e101 = t101_end - t101_start;
      sse3f s101 = t101_end + t101_start;
      ssef  u101 = dot(cross(e101,s101),DDDD);
      if (all(ge_mask(Vec3fa(u101),Vec3fa(0.0f)))) intersectFinish(ray,q10,q21-q10,q20-q21,q10-q20,u101[0],u101[1],u101[2],geomID,primID);

      sse3f p01; transpose((ssef)q01,(ssef)q02,(ssef)q12,(ssef)q11,p01.x,p01.y,p01.z);
      
      sse3f t010_start = shuffle<0,1,2,0>(p01), t010_end = shuffle<1,2,0,0>(p01);
      sse3f e010 = t010_end - t010_start;
      sse3f s010 = t010_end + t010_start;
      ssef  u010 = dot(cross(e010,s010),DDDD);
      if (all(ge_mask(Vec3fa(u010),Vec3fa(0.0f)))) intersectFinish(ray,q01,q02-q01,q12-q02,q01-q12,u010[0],u010[1],u010[2],geomID,primID);

      sse3f t011_start = shuffle<0,2,3,0>(p01), t011_end = shuffle<2,3,0,0>(p01);
      sse3f e011 = t011_end - t011_start;
      sse3f s011 = t011_end + t011_start;
      ssef  u011 = dot(cross(e011,s011),DDDD);
      if (all(ge_mask(Vec3fa(u011),Vec3fa(0.0f)))) intersectFinish(ray,q01,q12-q01,q11-q12,q01-q11,u011[0],u011[1],u011[2],geomID,primID);

      sse3f p11; transpose((ssef)q11,(ssef)q12,(ssef)q22,(ssef)q21,p11.x,p11.y,p11.z);
      
      sse3f t110_start = shuffle<0,1,2,0>(p11), t110_end = shuffle<1,2,0,0>(p11);
      sse3f e110 = t110_end - t110_start;
      sse3f s110 = t110_end + t110_start;
      ssef  u110 = dot(cross(e110,s110),DDDD);
      if (all(ge_mask(Vec3fa(u110),Vec3fa(0.0f)))) intersectFinish(ray,q11,q12-q11,q22-q11,q12-q22,u110[0],u110[1],u110[2],geomID,primID);

      sse3f t111_start = shuffle<0,2,3,0>(p11), t111_end = shuffle<2,3,0,0>(p11);
      sse3f e111 = t111_end - t111_start;
      sse3f s111 = t111_end + t111_start;
      ssef  u111 = dot(cross(e111,s111),DDDD);
      if (all(ge_mask(Vec3fa(u111),Vec3fa(0.0f)))) intersectFinish(ray,q11,q22-q11,q21-q22,q11-q21,u111[0],u111[1],u111[2],geomID,primID);

#endif

#endif

#if 0
      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      
      const Vec3fa q00 = v00-O, q10 = v10-O, q20 = v20-O;
      const Vec3fa q01 = v01-O, q11 = v11-O, q21 = v21-O;
      const Vec3fa q02 = v02-O, q12 = v12-O, q22 = v22-O;

#if defined (__AVX__)

      const avxf DD = avxf(ssef(D),ssef(D));

      const avxf q00_q01((ssef)q00,(ssef)q01);
      const avxf q01_q02((ssef)q01,(ssef)q02);
      const avxf e0001_e0102 = q01_q02 - q00_q01; 
      const avxf _u0001_u0102 = dot(cross(q01_q02 + q00_q01, e0001_e0102), DD);
      const Vec3fa e0001 = (Vec3fa)  extract<0>( e0001_e0102),  e0102 = (Vec3fa)   extract<1>( e0001_e0102);
      const float u0001 = extract<0>(extract<0>(_u0001_u0102)), u0102 = extract<0>(extract<1>(_u0001_u0102));

      const avxf q10_q11((ssef)q10,(ssef)q11);
      const avxf q11_q12((ssef)q11,(ssef)q12);
      const avxf e1011_e1112 = q11_q12-q10_q11;
      const avxf _u1011_u1112 = dot(cross(q10_q11 + q11_q12, e1011_e1112), DD);
      const Vec3fa e1011 = (Vec3fa)  extract<0>( e1011_e1112),  e1112 = (Vec3fa)   extract<1>( e1011_e1112);
      const float u1011 = extract<0>(extract<0>(_u1011_u1112)), u1112 = extract<0>(extract<1>(_u1011_u1112));
      
      const avxf q20_q21((ssef)q20,(ssef)q21);
      const avxf q21_q22((ssef)q21,(ssef)q22);
      const avxf e2021_e2122 = q21_q22 - q20_q21;
      const avxf _u2021_u2122 = dot(cross(q20_q21 + q21_q22, e2021_e2122), DD);
      const Vec3fa e2021 = (Vec3fa)  extract<0>( e2021_e2122),  e2122 = (Vec3fa)   extract<1>( e2021_e2122);
      const float u2021 = extract<0>(extract<0>(_u2021_u2122)), u2122 = extract<0>(extract<1>(_u2021_u2122));

      const avxf q00_q10((ssef)q00,(ssef)q10);
      const avxf q10_q20((ssef)q10,(ssef)q20);
      const avxf e0010_e1020 = q10_q20 - q00_q10;
      const avxf _u0010_u1020 = dot(cross(q10_q20 + q00_q10, e0010_e1020), DD);
      const Vec3fa e0010 = (Vec3fa)  extract<0>( e0010_e1020),  e1020 = (Vec3fa)   extract<1>( e0010_e1020);
      const float u0010 = extract<0>(extract<0>(_u0010_u1020)), u1020 = extract<0>(extract<1>(_u0010_u1020));

      const avxf q01_q11((ssef)q01,(ssef)q11);
      const avxf q11_q21((ssef)q11,(ssef)q21);
      const avxf e0111_e1121 = q11_q21 - q01_q11;
      const avxf _u0111_u1121 = dot(cross(q11_q21 + q01_q11, e0111_e1121), DD);
      const Vec3fa e0111 = (Vec3fa)  extract<0>( e0111_e1121),  e1121 = (Vec3fa)   extract<1>( e0111_e1121);
      const float u0111 = extract<0>(extract<0>(_u0111_u1121)), u1121 = extract<0>(extract<1>(_u0111_u1121));

      const avxf q02_q12((ssef)q02,(ssef)q12);
      const avxf q12_q22((ssef)q12,(ssef)q22);
      const avxf e0212_e1222 = q12_q22 - q02_q12;
      const avxf _u0212_u1222 = dot(cross(q12_q22 + q02_q12, e0212_e1222), DD);
      const Vec3fa e0212 = (Vec3fa)  extract<0>( e0212_e1222),  e1222 = (Vec3fa)   extract<1>( e0212_e1222);
      const float u0212 = extract<0>(extract<0>(_u0212_u1222)), u1222 = extract<0>(extract<1>(_u0212_u1222));
      
      //const avxf q00_q01((ssef)q00,(ssef)q01);
      //const avxf q11_q12((ssef)q11,(ssef)q12);
      const avxf e0011_e0112 = q11_q12 - q00_q01;
      const avxf _u0011_u0112 = dot(cross(q11_q12 + q00_q01, e0011_e0112), DD);
      const Vec3fa e0011 = (Vec3fa)  extract<0>( e0011_e0112),  e0112 = (Vec3fa)   extract<1>( e0011_e0112);
      const float u0011 = extract<0>(extract<0>(_u0011_u0112)), u0112 = extract<0>(extract<1>(_u0011_u0112));

      //const avxf q10_q11((ssef)q10,(ssef)q11);
      //const avxf q21_q22((ssef)q21,(ssef)q22);
      const avxf e1021_e1122 = q21_q22 - q10_q11;
      const avxf _u1021_u1122 = dot(cross(q21_q22 + q10_q11, e1021_e1122), DD);
      const Vec3fa e1021 = (Vec3fa)  extract<0>( e1021_e1122),  e1122 = (Vec3fa)   extract<1>( e1021_e1122);
      const float u1021 = extract<0>(extract<0>(_u1021_u1122)), u1122 = extract<0>(extract<1>(_u1021_u1122));

#else
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
#endif

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
#endif
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
