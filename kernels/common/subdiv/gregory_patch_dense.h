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
    Vec3fa matrix[4][4]; // f_p/m points are stored in 4th component
    
    static __forceinline float extract_f_m(const Vec3fa matrix[4][4], const size_t y, const size_t x) {
      return matrix[y][x].w;
    }
    
    static __forceinline Vec3fa extract_f_m_Vec3fa(const Vec3fa matrix[4][4], const size_t n) {
      return Vec3fa( extract_f_m(matrix,n,0), extract_f_m(matrix,n,1), extract_f_m(matrix,n,2) );
    }
    
    __forceinline Vec3fa eval(const float uu, const float vv) const
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
      return GregoryPatch3fa::eval(matrix,f_m,uu,vv);
    }

    __forceinline Vec3fa normal(const float uu, const float vv) const
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

    template<class T>
      static __forceinline Vec3<T> eval_t(const Vec3fa matrix[4][4], const T &uu, const T &vv) 
    {
      Vec3<T> f[2][2];
      f[0][0] = Vec3<T>( extract_f_m(matrix,0,0), extract_f_m(matrix,0,1), extract_f_m(matrix,0,2) );
      f[0][1] = Vec3<T>( extract_f_m(matrix,1,0), extract_f_m(matrix,1,1), extract_f_m(matrix,1,2) );
      f[1][1] = Vec3<T>( extract_f_m(matrix,2,0), extract_f_m(matrix,2,1), extract_f_m(matrix,2,2) );
      f[1][0] = Vec3<T>( extract_f_m(matrix,3,0), extract_f_m(matrix,3,1), extract_f_m(matrix,3,2) );
      return GregoryPatch3fa::eval_t(matrix,f,uu,vv);
    }

    template<class T>
      __forceinline Vec3<T> eval(const T &uu, const T &vv) const {
      return eval_t(matrix,uu,vv);
    }
    
     template<class T>
       static __forceinline Vec3<T> normal_t(const Vec3fa matrix[4][4], const T &uu, const T &vv) 
     {
       Vec3<T> f[2][2];
       f[0][0] = Vec3<T>( extract_f_m(matrix,0,0), extract_f_m(matrix,0,1), extract_f_m(matrix,0,2) );
       f[0][1] = Vec3<T>( extract_f_m(matrix,1,0), extract_f_m(matrix,1,1), extract_f_m(matrix,1,2) );
       f[1][1] = Vec3<T>( extract_f_m(matrix,2,0), extract_f_m(matrix,2,1), extract_f_m(matrix,2,2) );
       f[1][0] = Vec3<T>( extract_f_m(matrix,3,0), extract_f_m(matrix,3,1), extract_f_m(matrix,3,2) );
       return GregoryPatch3fa::normal_t(matrix,f,uu,vv);
    }

     template<class T>
      __forceinline Vec3<T> normal(const T &uu, const T &vv) const {
      return normal_t(matrix,uu,vv);
    }
  };
}
