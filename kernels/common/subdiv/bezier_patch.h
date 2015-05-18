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
  class __aligned(64) BezierPatch
  {
  public:

    template<class T, class S>
      static __forceinline T deCasteljau_t(const S &uu, const T &v0, const T &v1, const T &v2, const T &v3)
    {
      const S one_minus_uu = 1.0f - uu;      
      const T v0_1 = one_minus_uu * v0   + uu * v1;
      const T v1_1 = one_minus_uu * v1   + uu * v2;
      const T v2_1 = one_minus_uu * v2   + uu * v3;      
      const T v0_2 = one_minus_uu * v0_1 + uu * v1_1;
      const T v1_2 = one_minus_uu * v1_1 + uu * v2_1;      
      const T v0_3 = one_minus_uu * v0_2 + uu * v1_2;
      return v0_3;
    }
    
    template<class T, class S>
      static __forceinline T deCasteljau_tangent_t(const S &uu, const T &v0, const T &v1, const T &v2, const T &v3)
    {
      const S one_minus_uu = 1.0f - uu;      
      const T v0_1         = one_minus_uu * v0   + uu * v1;
      const T v1_1         = one_minus_uu * v1   + uu * v2;
      const T v2_1         = one_minus_uu * v2   + uu * v3;      
      const T v0_2         = one_minus_uu * v0_1 + uu * v1_1;
      const T v1_2         = one_minus_uu * v1_1 + uu * v2_1;      
      return v1_2 - v0_2;      
    }

    static __forceinline Vec3fa eval_bezier(const Vec3fa matrix[4][4], const float &uu, const float &vv) 
    {      
      const float one_minus_uu = 1.0f - uu;
      const float one_minus_vv = 1.0f - vv;      
      
      const float B0_u = one_minus_uu * one_minus_uu * one_minus_uu;
      const float B0_v = one_minus_vv * one_minus_vv * one_minus_vv;
      const float B1_u = 3.0f * (one_minus_uu * uu * one_minus_uu);
      const float B1_v = 3.0f * (one_minus_vv * vv * one_minus_vv);
      const float B2_u = 3.0f * (uu * one_minus_uu * uu);
      const float B2_v = 3.0f * (vv * one_minus_vv * vv);
      const float B3_u = uu * uu * uu;
      const float B3_v = vv * vv * vv;


      const Vec3fa_t res = 
       (B0_u * matrix[0][0] + B1_u * matrix[0][1] + B2_u * matrix[0][2] + B3_u * matrix[0][3]) * B0_v + 
       (B0_u * matrix[1][0] + B1_u * matrix[1][1] + B2_u * matrix[1][2] + B3_u * matrix[1][3]) * B1_v + 
       (B0_u * matrix[2][0] + B1_u * matrix[2][1] + B2_u * matrix[2][2] + B3_u * matrix[2][3]) * B2_v + 
       (B0_u * matrix[3][0] + B1_u * matrix[3][1] + B2_u * matrix[3][2] + B3_u * matrix[3][3]) * B3_v; 
      
      return res;
    }

    static __forceinline Vec3fa normal_bezier(const Vec3fa matrix[4][4],
                                              const float uu,
                                              const float vv) 
    {
      
      /* tangentU */
      const Vec3fa_t col0 = deCasteljau_t(vv, (Vec3fa_t)matrix[0][0], (Vec3fa_t)matrix[1][0], (Vec3fa_t)matrix[2][0], (Vec3fa_t)matrix[3][0]);
      const Vec3fa_t col1 = deCasteljau_t(vv, (Vec3fa_t)matrix[0][1], (Vec3fa_t)matrix[1][1], (Vec3fa_t)matrix[2][1], (Vec3fa_t)matrix[3][1]);
      const Vec3fa_t col2 = deCasteljau_t(vv, (Vec3fa_t)matrix[0][2], (Vec3fa_t)matrix[1][2], (Vec3fa_t)matrix[2][2], (Vec3fa_t)matrix[3][2]);
      const Vec3fa_t col3 = deCasteljau_t(vv, (Vec3fa_t)matrix[0][3], (Vec3fa_t)matrix[1][3], (Vec3fa_t)matrix[2][3], (Vec3fa_t)matrix[3][3]);
      
      const Vec3fa_t tangentU = deCasteljau_tangent_t(uu, col0, col1, col2, col3);
      
      /* tangentV */
      const Vec3fa_t row0 = deCasteljau_t(uu, (Vec3fa_t)matrix[0][0], (Vec3fa_t)matrix[0][1], (Vec3fa_t)matrix[0][2], (Vec3fa_t)matrix[0][3]);
      const Vec3fa_t row1 = deCasteljau_t(uu, (Vec3fa_t)matrix[1][0], (Vec3fa_t)matrix[1][1], (Vec3fa_t)matrix[1][2], (Vec3fa_t)matrix[1][3]);
      const Vec3fa_t row2 = deCasteljau_t(uu, (Vec3fa_t)matrix[2][0], (Vec3fa_t)matrix[2][1], (Vec3fa_t)matrix[2][2], (Vec3fa_t)matrix[2][3]);
      const Vec3fa_t row3 = deCasteljau_t(uu, (Vec3fa_t)matrix[3][0], (Vec3fa_t)matrix[3][1], (Vec3fa_t)matrix[3][2], (Vec3fa_t)matrix[3][3]);
      
      const Vec3fa_t tangentV = deCasteljau_tangent_t(vv, row0, row1, row2, row3);
      
      /* normal = tangentU x tangentV */
      const Vec3fa_t n = cross(tangentV,tangentU);
      
      return n;     
    }
    
    template<class M, class T>
      static __forceinline Vec3<T> eval_t_bezier(const Vec3fa matrix[4][4],
						 const T &uu,
						 const T &vv) 
    {      
      const T one_minus_uu = 1.0f - uu;
      const T one_minus_vv = 1.0f - vv;      

      const T B0_u = one_minus_uu * one_minus_uu * one_minus_uu;
      const T B0_v = one_minus_vv * one_minus_vv * one_minus_vv;
      const T B1_u = 3.0f * (one_minus_uu * uu * one_minus_uu);
      const T B1_v = 3.0f * (one_minus_vv * vv * one_minus_vv);
      const T B2_u = 3.0f * (uu * one_minus_uu * uu);
      const T B2_v = 3.0f * (vv * one_minus_vv * vv);
      const T B3_u = uu * uu * uu;
      const T B3_v = vv * vv * vv;
      
      const T x = 
	(B0_u * matrix[0][0].x + B1_u * matrix[0][1].x + B2_u * matrix[0][2].x + B3_u * matrix[0][3].x) * B0_v + 
	(B0_u * matrix[1][0].x + B1_u * matrix[1][1].x + B2_u * matrix[1][2].x + B3_u * matrix[1][3].x) * B1_v + 
	(B0_u * matrix[2][0].x + B1_u * matrix[2][1].x + B2_u * matrix[2][2].x + B3_u * matrix[2][3].x) * B2_v + 
	(B0_u * matrix[3][0].x + B1_u * matrix[3][1].x + B2_u * matrix[3][2].x + B3_u * matrix[3][3].x) * B3_v; 
      
      const T y = 
	(B0_u * matrix[0][0].y + B1_u * matrix[0][1].y + B2_u * matrix[0][2].y + B3_u * matrix[0][3].y) * B0_v + 
	(B0_u * matrix[1][0].y + B1_u * matrix[1][1].y + B2_u * matrix[1][2].y + B3_u * matrix[1][3].y) * B1_v + 
	(B0_u * matrix[2][0].y + B1_u * matrix[2][1].y + B2_u * matrix[2][2].y + B3_u * matrix[2][3].y) * B2_v + 
	(B0_u * matrix[3][0].y + B1_u * matrix[3][1].y + B2_u * matrix[3][2].y + B3_u * matrix[3][3].y) * B3_v; 
      
      const T z = 
	(B0_u * matrix[0][0].z + B1_u * matrix[0][1].z + B2_u * matrix[0][2].z + B3_u * matrix[0][3].z) * B0_v + 
	(B0_u * matrix[1][0].z + B1_u * matrix[1][1].z + B2_u * matrix[1][2].z + B3_u * matrix[1][3].z) * B1_v + 
	(B0_u * matrix[2][0].z + B1_u * matrix[2][1].z + B2_u * matrix[2][2].z + B3_u * matrix[2][3].z) * B2_v + 
	(B0_u * matrix[3][0].z + B1_u * matrix[3][1].z + B2_u * matrix[3][2].z + B3_u * matrix[3][3].z) * B3_v; 
            
      return Vec3<T>(x,y,z);
    }

    template<class M, class T>
      static __forceinline Vec3<T> normal_t_bezier(const Vec3fa matrix[4][4],
						   const T &uu,
						   const T &vv) 
    {
      
      const Vec3<T> matrix_00 = Vec3<T>(matrix[0][0].x,matrix[0][0].y,matrix[0][0].z);
      const Vec3<T> matrix_01 = Vec3<T>(matrix[0][1].x,matrix[0][1].y,matrix[0][1].z);
      const Vec3<T> matrix_02 = Vec3<T>(matrix[0][2].x,matrix[0][2].y,matrix[0][2].z);
      const Vec3<T> matrix_03 = Vec3<T>(matrix[0][3].x,matrix[0][3].y,matrix[0][3].z);

      const Vec3<T> matrix_10 = Vec3<T>(matrix[1][0].x,matrix[1][0].y,matrix[1][0].z);
      const Vec3<T> matrix_11 = Vec3<T>(matrix[1][1].x,matrix[1][1].y,matrix[1][1].z);
      const Vec3<T> matrix_12 = Vec3<T>(matrix[1][2].x,matrix[1][2].y,matrix[1][2].z);
      const Vec3<T> matrix_13 = Vec3<T>(matrix[1][3].x,matrix[1][3].y,matrix[1][3].z);

      const Vec3<T> matrix_20 = Vec3<T>(matrix[2][0].x,matrix[2][0].y,matrix[2][0].z);
      const Vec3<T> matrix_21 = Vec3<T>(matrix[2][1].x,matrix[2][1].y,matrix[2][1].z);
      const Vec3<T> matrix_22 = Vec3<T>(matrix[2][2].x,matrix[2][2].y,matrix[2][2].z);
      const Vec3<T> matrix_23 = Vec3<T>(matrix[2][3].x,matrix[2][3].y,matrix[2][3].z);

      const Vec3<T> matrix_30 = Vec3<T>(matrix[3][0].x,matrix[3][0].y,matrix[3][0].z);
      const Vec3<T> matrix_31 = Vec3<T>(matrix[3][1].x,matrix[3][1].y,matrix[3][1].z);
      const Vec3<T> matrix_32 = Vec3<T>(matrix[3][2].x,matrix[3][2].y,matrix[3][2].z);
      const Vec3<T> matrix_33 = Vec3<T>(matrix[3][3].x,matrix[3][3].y,matrix[3][3].z);
            
      /* tangentU */
      const Vec3<T> col0 = deCasteljau_t(vv, matrix_00, matrix_10, matrix_20, matrix_30);
      const Vec3<T> col1 = deCasteljau_t(vv, matrix_01, matrix_11, matrix_21, matrix_31);
      const Vec3<T> col2 = deCasteljau_t(vv, matrix_02, matrix_12, matrix_22, matrix_32);
      const Vec3<T> col3 = deCasteljau_t(vv, matrix_03, matrix_13, matrix_23, matrix_33);
      
      const Vec3<T> tangentU = deCasteljau_tangent_t(uu, col0, col1, col2, col3);
      
      /* tangentV */
      const Vec3<T> row0 = deCasteljau_t(uu, matrix_00, matrix_01, matrix_02, matrix_03);
      const Vec3<T> row1 = deCasteljau_t(uu, matrix_10, matrix_11, matrix_12, matrix_13);
      const Vec3<T> row2 = deCasteljau_t(uu, matrix_20, matrix_21, matrix_22, matrix_23);
      const Vec3<T> row3 = deCasteljau_t(uu, matrix_30, matrix_31, matrix_32, matrix_33);
      
      const Vec3<T> tangentV = deCasteljau_tangent_t(vv, row0, row1, row2, row3);
      
      /* normal = tangentU x tangentV */
      const Vec3<T> n = cross(tangentV,tangentU);
      return n;
    }
    
    
#if !defined(__MIC__)
    
#if defined(__AVX__)    
    
    static __forceinline avx3f eval8_bezier  (const Vec3fa matrix[4][4], const avxf &uu, const avxf &vv) { return eval_t_bezier<avxb,avxf>(matrix,uu,vv); }
    static __forceinline avx3f normal8_bezier(const Vec3fa matrix[4][4], const avxf &uu, const avxf &vv) { return normal_t_bezier<avxb,avxf>(matrix,uu,vv); }
    
#endif
    
    static __forceinline sse3f eval4_bezier  (const Vec3fa matrix[4][4], const ssef &uu, const ssef &vv) { return eval_t_bezier<sseb,ssef>(matrix,uu,vv); }
    static __forceinline sse3f normal4_bezier(const Vec3fa matrix[4][4], const ssef &uu, const ssef &vv) { return normal_t_bezier<sseb,ssef>(matrix,uu,vv); }
    
#endif
    
  };
}
