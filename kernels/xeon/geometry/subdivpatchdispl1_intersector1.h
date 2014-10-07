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
#if 0 && defined(__AVX__)

      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      
      const Vec3fa q00 = v00-O, q10 = v10-O, q20 = v20-O;
      const Vec3fa q01 = v01-O, q11 = v11-O, q21 = v21-O;
      const Vec3fa q02 = v02-O, q12 = v12-O, q22 = v22-O;

      sse3f q10_q11_q01_q22; transpose((ssef)q10,(ssef)q11,(ssef)q01,(ssef)q22,q10_q11_q01_q22.x,q10_q11_q01_q22.y,q10_q11_q01_q22.z);
      sse3f q20_q21_q11_q21; transpose((ssef)q20,(ssef)q21,(ssef)q11,(ssef)q21,q20_q21_q11_q21.x,q20_q21_q11_q21.y,q20_q21_q11_q21.z);
      sse3f q11_q12_q02_q12; transpose((ssef)q11,(ssef)q12,(ssef)q02,(ssef)q12,q11_q12_q02_q12.x,q11_q12_q02_q12.y,q11_q12_q02_q12.z);
      sse3f q21_q22_q12_q22; transpose((ssef)q21,(ssef)q22,(ssef)q12,(ssef)q22,q21_q22_q12_q22.x,q21_q22_q12_q22.y,q21_q22_q12_q22.z);

      //sse3f q00_q00_q00_q12(shuffle<0,0,0,4>((ssef)q00,(ssef)q12), shuffle<1,1,1,5>((ssef)q00,(ssef)q12), shuffle<2,2,2,6>((ssef)q00,(ssef)q12));
      //sse3f q10_q10_q10_q20(shuffle<0,0,0,4>((ssef)q10,(ssef)q20), shuffle<1,1,1,5>((ssef)q10,(ssef)q20), shuffle<2,2,2,6>((ssef)q10,(ssef)q20));
      //sse3f q01_q01_q01_q02(shuffle<0,0,0,4>((ssef)q01,(ssef)q02), shuffle<1,1,1,5>((ssef)q01,(ssef)q02), shuffle<2,2,2,6>((ssef)q01,(ssef)q02));
      //sse3f q11_q11_q11_q21(shuffle<0,0,0,4>((ssef)q11,(ssef)q21), shuffle<1,1,1,5>((ssef)q11,(ssef)q21), shuffle<2,2,2,6>((ssef)q11,(ssef)q21));

      sse3f q00_q00_q00_q12(insert<3,0>(shuffle<0>((ssef)q00),(ssef)q12), insert<3,1>(shuffle<1>((ssef)q00),(ssef)q12), insert<3,2>(shuffle<2>((ssef)q00),(ssef)q12));
      sse3f q10_q10_q10_q20(insert<3,0>(shuffle<0>((ssef)q10),(ssef)q20), insert<3,1>(shuffle<1>((ssef)q10),(ssef)q20), insert<3,2>(shuffle<2>((ssef)q10),(ssef)q20));
      sse3f q01_q01_q01_q02(insert<3,0>(shuffle<0>((ssef)q01),(ssef)q02), insert<3,1>(shuffle<1>((ssef)q01),(ssef)q02), insert<3,2>(shuffle<2>((ssef)q01),(ssef)q02));
      sse3f q11_q11_q11_q21(insert<3,0>(shuffle<0>((ssef)q11),(ssef)q21), insert<3,1>(shuffle<1>((ssef)q11),(ssef)q21), insert<3,2>(shuffle<2>((ssef)q11),(ssef)q21));

      const sse3f DDDD(D.x,D.y,D.z);

      const sse3f e0010_e0011_e0001_e1222 = q10_q11_q01_q22 - q00_q00_q00_q12;
      const sse3f s0010_s0011_s0001_s1222 = q10_q11_q01_q22 + q00_q00_q00_q12;
      const ssef  u0010_u0011_u0001_u1222 = dot(cross(s0010_s0011_s0001_s1222,e0010_e0011_e0001_e1222),DDDD);

      const sse3f e1020_e1021_e1011_e2021 = q20_q21_q11_q21 - q10_q10_q10_q20;
      const sse3f s1020_s1021_s1011_s2021 = q20_q21_q11_q21 + q10_q10_q10_q20;
      const ssef  u1020_u1021_u1011_u2021 = dot(cross(s1020_s1021_s1011_s2021,e1020_e1021_e1011_e2021),DDDD);

      const sse3f e0111_e0112_e0102_e0212 = q11_q12_q02_q12 - q01_q01_q01_q02;
      const sse3f s0111_s0112_s0102_s0212 = q11_q12_q02_q12 + q01_q01_q01_q02;
      const ssef  u0111_u0112_u0102_u0212 = dot(cross(s0111_s0112_s0102_s0212,e0111_e0112_e0102_e0212),DDDD);

      const sse3f e1121_e1122_e1112_e2122 = q21_q22_q12_q22 - q11_q11_q11_q21;
      const sse3f s1121_s1122_s1112_s2122 = q21_q22_q12_q22 + q11_q11_q11_q21;
      const ssef  u1121_u1122_u1112_u2122 = dot(cross(s1121_s1122_s1112_s2122,e1121_e1122_e1112_e2122),DDDD);

      const avxf mp = avxf(-1,1,-1,1,-1,1,-1,1);

#if 0
      const float u0010 = extract<0>(u0010_u0011_u0001_u1222);
      const float u0011 = extract<1>(u0010_u0011_u0001_u1222);
      const float u0001 = extract<2>(u0010_u0011_u0001_u1222);
      const float u1222 = extract<3>(u0010_u0011_u0001_u1222);

      const float u1020 = extract<0>(u1020_u1021_u1011_u2021);
      const float u1021 = extract<1>(u1020_u1021_u1011_u2021);
      const float u1011 = extract<2>(u1020_u1021_u1011_u2021);
      const float u2021 = extract<3>(u1020_u1021_u1011_u2021);

      const float u0111 = extract<0>(u0111_u0112_u0102_u0212);
      const float u0112 = extract<1>(u0111_u0112_u0102_u0212);
      const float u0102 = extract<2>(u0111_u0112_u0102_u0212);
      const float u0212 = extract<3>(u0111_u0112_u0102_u0212);

      const float u1121 = extract<0>(u1121_u1122_u1112_u2122);
      const float u1122 = extract<1>(u1121_u1122_u1112_u2122);
      const float u1112 = extract<2>(u1121_u1122_u1112_u2122);
      const float u2122 = extract<3>(u1121_u1122_u1112_u2122);

      const avxf U0 = avxf(-u0001,-u0011,-u0102,-u0112,-u1011,-u1021,-u1112,-u1122);
      const avxf U1 = avxf(-u0111,+u1011,-u0212,+u1112, -u1121,+u2021,-u1222,+u2122);
      const avxf U2 = avxf(+u0011,+u0010,+u0112,+u0111,+u1021,+u1020,+u1122,+u1121);
#else

      const avxf U0 = -avxf(shuffle<2,1,2,1>(u0010_u0011_u0001_u1222,u0111_u0112_u0102_u0212),shuffle<2,1,2,1>(u1020_u1021_u1011_u2021,u1121_u1122_u1112_u2122));
      const avxf U1 = mp*avxf(insert<3,2>(insert<1,2>(shuffle<0,1,3,2>(u0111_u0112_u0102_u0212),u1020_u1021_u1011_u2021),u1121_u1122_u1112_u2122),
                              insert<2,3>(insert<1,3>(u1121_u1122_u1112_u2122,u1020_u1021_u1011_u2021),u0010_u0011_u0001_u1222));
      const avxf U2 = avxf(shuffle<1,0,1,0>(u0010_u0011_u0001_u1222,u0111_u0112_u0102_u0212),shuffle<1,0,1,0>(u1020_u1021_u1011_u2021,u1121_u1122_u1112_u2122));

#endif

      const avx3f q(avxf(q00.x,q00.x,q01.x,q01.x,q10.x,q10.x,q11.x,q11.x),
                    avxf(q00.y,q00.y,q01.y,q01.y,q10.y,q10.y,q11.y,q11.y),
                    avxf(q00.z,q00.z,q01.z,q01.z,q10.z,q10.z,q11.z,q11.z));

      const avx3f D8(avxf(D.x),avxf(D.y),avxf(D.z));

#if 0
      const float e0010_x = extract<0>(e0010_e0011_e0001_e1222.x), e0010_y = extract<0>(e0010_e0011_e0001_e1222.y), e0010_z = extract<0>(e0010_e0011_e0001_e1222.z);
      const float e0011_x = extract<1>(e0010_e0011_e0001_e1222.x), e0011_y = extract<1>(e0010_e0011_e0001_e1222.y), e0011_z = extract<1>(e0010_e0011_e0001_e1222.z);
      const float e0001_x = extract<2>(e0010_e0011_e0001_e1222.x), e0001_y = extract<2>(e0010_e0011_e0001_e1222.y), e0001_z = extract<2>(e0010_e0011_e0001_e1222.z);
      const float e1222_x = extract<3>(e0010_e0011_e0001_e1222.x), e1222_y = extract<3>(e0010_e0011_e0001_e1222.y), e1222_z = extract<3>(e0010_e0011_e0001_e1222.z);

      const float e1020_x = extract<0>(e1020_e1021_e1011_e2021.x), e1020_y = extract<0>(e1020_e1021_e1011_e2021.y), e1020_z = extract<0>(e1020_e1021_e1011_e2021.z);
      const float e1021_x = extract<1>(e1020_e1021_e1011_e2021.x), e1021_y = extract<1>(e1020_e1021_e1011_e2021.y), e1021_z = extract<1>(e1020_e1021_e1011_e2021.z);
      const float e1011_x = extract<2>(e1020_e1021_e1011_e2021.x), e1011_y = extract<2>(e1020_e1021_e1011_e2021.y), e1011_z = extract<2>(e1020_e1021_e1011_e2021.z);
      const float e2021_x = extract<3>(e1020_e1021_e1011_e2021.x), e2021_y = extract<3>(e1020_e1021_e1011_e2021.y), e2021_z = extract<3>(e1020_e1021_e1011_e2021.z);

      const float e0111_x = extract<0>(e0111_e0112_e0102_e0212.x), e0111_y = extract<0>(e0111_e0112_e0102_e0212.y), e0111_z = extract<0>(e0111_e0112_e0102_e0212.z);
      const float e0112_x = extract<1>(e0111_e0112_e0102_e0212.x), e0112_y = extract<1>(e0111_e0112_e0102_e0212.y), e0112_z = extract<1>(e0111_e0112_e0102_e0212.z);
      const float e0102_x = extract<2>(e0111_e0112_e0102_e0212.x), e0102_y = extract<2>(e0111_e0112_e0102_e0212.y), e0102_z = extract<2>(e0111_e0112_e0102_e0212.z);
      const float e0212_x = extract<3>(e0111_e0112_e0102_e0212.x), e0212_y = extract<3>(e0111_e0112_e0102_e0212.y), e0212_z = extract<3>(e0111_e0112_e0102_e0212.z);

      const float e1121_x = extract<0>(e1121_e1122_e1112_e2122.x), e1121_y = extract<0>(e1121_e1122_e1112_e2122.y), e1121_z = extract<0>(e1121_e1122_e1112_e2122.z);
      const float e1122_x = extract<1>(e1121_e1122_e1112_e2122.x), e1122_y = extract<1>(e1121_e1122_e1112_e2122.y), e1122_z = extract<1>(e1121_e1122_e1112_e2122.z);
      const float e1112_x = extract<2>(e1121_e1122_e1112_e2122.x), e1112_y = extract<2>(e1121_e1122_e1112_e2122.y), e1112_z = extract<2>(e1121_e1122_e1112_e2122.z);
      const float e2122_x = extract<3>(e1121_e1122_e1112_e2122.x), e2122_y = extract<3>(e1121_e1122_e1112_e2122.y), e2122_z = extract<3>(e1121_e1122_e1112_e2122.z);
      
      const avx3f e0(avxf(-e0001_x,-e0011_x,-e0102_x,-e0112_x,-e1011_x,-e1021_x,-e1112_x,-e1122_x),
                     avxf(-e0001_y,-e0011_y,-e0102_y,-e0112_y,-e1011_y,-e1021_y,-e1112_y,-e1122_y),
                     avxf(-e0001_z,-e0011_z,-e0102_z,-e0112_z,-e1011_z,-e1021_z,-e1112_z,-e1122_z));

      const avx3f e1(avxf(-e0111_x,+e1011_x,-e0212_x,+e1112_x,-e1121_x,+e2021_x,-e1222_x,+e2122_x),
                     avxf(-e0111_y,+e1011_y,-e0212_y,+e1112_y,-e1121_y,+e2021_y,-e1222_y,+e2122_y),
                     avxf(-e0111_z,+e1011_z,-e0212_z,+e1112_z,-e1121_z,+e2021_z,-e1222_z,+e2122_z));

      const avx3f Ng0 = cross(e1,e0);

#else

      const avx3f e0(-avxf(shuffle<2,1,2,1>(e0010_e0011_e0001_e1222.x,e0111_e0112_e0102_e0212.x),shuffle<2,1,2,1>(e1020_e1021_e1011_e2021.x,e1121_e1122_e1112_e2122.x)),
                     -avxf(shuffle<2,1,2,1>(e0010_e0011_e0001_e1222.y,e0111_e0112_e0102_e0212.y),shuffle<2,1,2,1>(e1020_e1021_e1011_e2021.y,e1121_e1122_e1112_e2122.y)),
                     -avxf(shuffle<2,1,2,1>(e0010_e0011_e0001_e1222.z,e0111_e0112_e0102_e0212.z),shuffle<2,1,2,1>(e1020_e1021_e1011_e2021.z,e1121_e1122_e1112_e2122.z)));
        
      const avx3f e2(avxf(shuffle<1,0,1,0>(e0010_e0011_e0001_e1222.x,e0111_e0112_e0102_e0212.x),shuffle<1,0,1,0>(e1020_e1021_e1011_e2021.x,e1121_e1122_e1112_e2122.x)),
                     avxf(shuffle<1,0,1,0>(e0010_e0011_e0001_e1222.y,e0111_e0112_e0102_e0212.y),shuffle<1,0,1,0>(e1020_e1021_e1011_e2021.y,e1121_e1122_e1112_e2122.y)),
                     avxf(shuffle<1,0,1,0>(e0010_e0011_e0001_e1222.z,e0111_e0112_e0102_e0212.z),shuffle<1,0,1,0>(e1020_e1021_e1011_e2021.z,e1121_e1122_e1112_e2122.z)));

      const avx3f Ng0 = cross(e0,e2);

#endif

      const avx3f Ng = Ng0+Ng0;
      const avxf det = dot(D8,Ng);
      const avxf T   = dot(q,Ng);
      
      avxb maskb = (U0 >= 0.0f) & (U1 >= 0.0f) & (U2 >= 0.0f) & (det*ray.tnear <= T) & (T <= det*ray.tfar);
      size_t mask = movemask(maskb);

      while (mask) {
        size_t i = __bscf(mask);
        const float rcpDet = rcp(det[i]);
        ray.u    = U0[i] * rcpDet;
        ray.v    = U1[i] * rcpDet;
        ray.tfar = T[i] * rcpDet;
        ray.Ng   = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
        ray.geomID  = geomID;
        ray.primID  = primID;
      }

#else
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
