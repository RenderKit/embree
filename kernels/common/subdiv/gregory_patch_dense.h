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

#include "gregory_patch.h"

namespace embree
{  
  class __aligned(64) DenseGregoryPatch3fa
  {
  public:
    Vec3fa v[4][4]; // f_p/m points are stored in 4th component
    
    static __forceinline float extract_f_m(const Vec3fa matrix[4][4], const size_t y, const size_t x) {
      return matrix[y][x].w;
    }
    
    static __forceinline Vec3fa extract_f_m_Vec3fa(const Vec3fa matrix[4][4], const size_t n) {
      return Vec3fa( extract_f_m(matrix,n,0), extract_f_m(matrix,n,1), extract_f_m(matrix,n,2) );
    }
    
     static __forceinline Vec3fa eval(const Vec3fa matrix[4][4], const float &uu, const float &vv) 
    {
      Vec3fa f[2][2];
      f[0][0] = extract_f_m_Vec3fa( matrix, 0 );
      f[0][1] = extract_f_m_Vec3fa( matrix, 1 );
      f[1][1] = extract_f_m_Vec3fa( matrix, 2 );
      f[1][0] = extract_f_m_Vec3fa( matrix, 3 );
      return GregoryPatch3fa::eval(matrix,f,uu,vv);
    }

    template<class M, class T>
      static __forceinline Vec3<T> eval_t(const Vec3fa matrix[4][4], const T &uu, const T &vv) 
    {
      Vec3<T> f[2][2];
      f[0][0] = Vec3<T>( extract_f_m(matrix,0,0), extract_f_m(matrix,0,1), extract_f_m(matrix,0,2) );
      f[0][1] = Vec3<T>( extract_f_m(matrix,1,0), extract_f_m(matrix,1,1), extract_f_m(matrix,1,2) );
      f[1][1] = Vec3<T>( extract_f_m(matrix,2,0), extract_f_m(matrix,2,1), extract_f_m(matrix,2,2) );
      f[1][0] = Vec3<T>( extract_f_m(matrix,3,0), extract_f_m(matrix,3,1), extract_f_m(matrix,3,2) );
      return GregoryPatch3fa::eval_t<M>(matrix,f,uu,vv);
    }
    
     template<class M, class T>
       static __forceinline Vec3<T> normal_t(const Vec3fa matrix[4][4], const T &uu, const T &vv) 
     {
       Vec3<T> f[2][2];
       f[0][0] = Vec3<T>( extract_f_m(matrix,0,0), extract_f_m(matrix,0,1), extract_f_m(matrix,0,2) );
       f[0][1] = Vec3<T>( extract_f_m(matrix,1,0), extract_f_m(matrix,1,1), extract_f_m(matrix,1,2) );
       f[1][1] = Vec3<T>( extract_f_m(matrix,2,0), extract_f_m(matrix,2,1), extract_f_m(matrix,2,2) );
       f[1][0] = Vec3<T>( extract_f_m(matrix,3,0), extract_f_m(matrix,3,1), extract_f_m(matrix,3,2) );
       return GregoryPatch3fa::normal_t<M>(matrix,f,uu,vv);
    }
     
#if defined(__MIC__)
     
     static __forceinline float16 extract_f_m_float16(const Vec3fa matrix[4][4], const size_t n)
     {
       const float16 row = load16f(&matrix[n][0]);
       __aligned(64) float xyzw[16];
      compactustore16f_low(0x8888,xyzw,row);
      return broadcast4to16f(xyzw);
    }
     
     static __forceinline Vec3f16 extract_f_m_Vec3f16(const Vec3fa matrix[4][4], const size_t n) {
       return Vec3f16( extract_f_m(matrix,n,0), extract_f_m(matrix,n,1), extract_f_m(matrix,n,2) );
    }
    
    static __forceinline float16 eval4(const Vec3fa matrix[4][4],
				     const float16 uu,
				     const float16 vv) 
    {
      const bool16 m_border = (uu == 0.0f) | (uu == 1.0f) | (vv == 0.0f) | (vv == 1.0f);
      
      const float16 f0_p = (Vec3fa_t)matrix[1][1];
      const float16 f1_p = (Vec3fa_t)matrix[1][2];
      const float16 f2_p = (Vec3fa_t)matrix[2][2];
      const float16 f3_p = (Vec3fa_t)matrix[2][1];
      
      const float16 f0_m = extract_f_m_float16(matrix,0);
      const float16 f1_m = extract_f_m_float16(matrix,1);
      const float16 f2_m = extract_f_m_float16(matrix,2);
      const float16 f3_m = extract_f_m_float16(matrix,3);
      
      const float16 one_minus_uu = float16(1.0f) - uu;
      const float16 one_minus_vv = float16(1.0f) - vv;      
      
#if 1
      const float16 inv0 = rcp(uu+vv);
      const float16 inv1 = rcp(one_minus_uu+vv);
      const float16 inv2 = rcp(one_minus_uu+one_minus_vv);
      const float16 inv3 = rcp(uu+one_minus_vv);
#else
      const float16 inv0 = 1.0f/(uu+vv);
      const float16 inv1 = 1.0f/(one_minus_uu+vv);
      const float16 inv2 = 1.0f/(one_minus_uu+one_minus_vv);
      const float16 inv3 = 1.0f/(uu+one_minus_vv);
#endif
      
      const float16 F0 = select(m_border,f0_p, (          uu * f0_p +           vv * f0_m) * inv0);
      const float16 F1 = select(m_border,f1_p, (one_minus_uu * f1_m +           vv * f1_p) * inv1);
      const float16 F2 = select(m_border,f2_p, (one_minus_uu * f2_p + one_minus_vv * f2_m) * inv2);
      const float16 F3 = select(m_border,f3_p, (          uu * f3_m + one_minus_vv * f3_p) * inv3);
      
      const float16 B0_u = one_minus_uu * one_minus_uu * one_minus_uu;
      const float16 B0_v = one_minus_vv * one_minus_vv * one_minus_vv;
      const float16 B1_u = 3.0f * (one_minus_uu * uu * one_minus_uu);
      const float16 B1_v = 3.0f * (one_minus_vv * vv * one_minus_vv);
      const float16 B2_u = 3.0f * (uu * one_minus_uu * uu);
      const float16 B2_v = 3.0f * (vv * one_minus_vv * vv);
      const float16 B3_u = uu * uu * uu;
      const float16 B3_v = vv * vv * vv;
      
      const float16 res = 
	(B0_u * (Vec3fa_t)matrix[0][0] + B1_u * (Vec3fa_t)matrix[0][1] + B2_u * (Vec3fa_t)matrix[0][2] + B3_u * (Vec3fa_t)matrix[0][3]) * B0_v + 
	(B0_u * (Vec3fa_t)matrix[1][0] + B1_u *                     F0 + B2_u *                     F1 + B3_u * (Vec3fa_t)matrix[1][3]) * B1_v + 
	(B0_u * (Vec3fa_t)matrix[2][0] + B1_u *                     F3 + B2_u *                     F2 + B3_u * (Vec3fa_t)matrix[2][3]) * B2_v + 
	(B0_u * (Vec3fa_t)matrix[3][0] + B1_u * (Vec3fa_t)matrix[3][1] + B2_u * (Vec3fa_t)matrix[3][2] + B3_u * (Vec3fa_t)matrix[3][3]) * B3_v; 
      return res;
    }
    
    
    static __forceinline Vec3f16 eval16(const Vec3fa matrix[4][4], const float16 &uu, const float16 &vv) {
      return eval_t<bool16,float16>(matrix,uu,vv);
    }
        
    static __forceinline Vec3f16 normal16(const Vec3fa matrix[4][4], const float16 &uu, const float16 &vv) {
      return normal_t<bool16,float16>(matrix,uu,vv);
    }
    
#endif

    static __forceinline Vec3fa normal(const Vec3fa matrix[4][4],
				       const float uu,
				       const float vv) 
    {
#if defined(__MIC__)
      const float16 row0 = load16f(&matrix[0][0]);
      const float16 row1 = load16f(&matrix[1][0]);
      const float16 row2 = load16f(&matrix[2][0]);
      const float16 row3 = load16f(&matrix[3][0]);
      
      __aligned(64) Vec3fa f_m[2][2];
      compactustore16f_low(0x8888,(float*)&f_m[0][0],row0);
      compactustore16f_low(0x8888,(float*)&f_m[0][1],row1);
      compactustore16f_low(0x8888,(float*)&f_m[1][1],row2);
      compactustore16f_low(0x8888,(float*)&f_m[1][0],row3);
#else
      __aligned(64) Vec3fa f_m[2][2];
      f_m[0][0] = extract_f_m_Vec3fa(matrix,0);
      f_m[0][1] = extract_f_m_Vec3fa(matrix,1);
      f_m[1][1] = extract_f_m_Vec3fa(matrix,2);
      f_m[1][0] = extract_f_m_Vec3fa(matrix,3);      
#endif      
      return GregoryPatch3fa::normal(matrix,f_m,uu,vv);
    }
  };
}
