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

#include "../../common/default.h"

namespace embree
{
  /* 3x3 point grid => 2x2 quad grid */


  struct __aligned(64) Quad2x2
  {

    /*  v00 - v01 - v02 */
    /*  v10 - v11 - v12 */
    /*  v20 - v21 - v22 */
  
    /* v00 - v10 - v01 - v11 - v02 - v12 */
    /* v10 - v20 - v11 - v21 - v12 - v22 */
   
    Quad2x2() {}

    float vtx_x[12];
    float vtx_y[12];
    float vtx_z[12];
#if DISCRETIZED_UV == 1
    unsigned short vtx_u[12];
    unsigned short vtx_v[12];
#else
    float vtx_u[12];
    float vtx_v[12];
#endif
    
    static __forceinline float u16_to_float(const unsigned short source) { return (float)source * (1.0f/65535.0f); } 
    static __forceinline unsigned short float_to_u16(const float f) { return (unsigned short)(f*65535.0f); }
    static __forceinline int4 float_to_u16(const float4 &f) { return (int4)(f*65535.0f); }

    __forceinline void initFrom3x3Grid( const float *const source,
                                        float *const dest,
                                        const size_t offset_line0,
                                        const size_t offset_line1,
                                        const size_t offset_line2)
    {
      const float v00 = source[offset_line0 + 0];
      const float v01 = source[offset_line0 + 1];
      const float v02 = source[offset_line0 + 2];
      const float v10 = source[offset_line1 + 0];
      const float v11 = source[offset_line1 + 1];
      const float v12 = source[offset_line1 + 2];
      const float v20 = source[offset_line2 + 0];
      const float v21 = source[offset_line2 + 1];
      const float v22 = source[offset_line2 + 2];

      /* v00 - v10 - v01 - v11 - v02 - v12 */
      dest[ 0] = v00;
      dest[ 1] = v10;
      dest[ 2] = v01;
      dest[ 3] = v11;
      dest[ 4] = v02;
      dest[ 5] = v12;
      /* v10 - v20 - v11 - v21 - v12 - v22 */
      dest[ 6] = v10;
      dest[ 7] = v20;
      dest[ 8] = v11;
      dest[ 9] = v21;
      dest[10] = v12;
      dest[11] = v22;
    }

    __forceinline void initFrom3x3Grid( const float4 source[3], float *const dest)
    {
      const float4 row0_a = unpacklo(source[0],source[1]); 
      const float4 row0_b = shuffle<0,1,0,1>(unpackhi(source[0],source[1]));
      const float4 row1_a = unpacklo(source[1],source[2]);
      const float4 row1_b = shuffle<0,1,0,1>(unpackhi(source[1],source[2]));

      storeu4f(&dest[2], row0_b);
      storeu4f(&dest[8], row1_b);
      storeu4f(&dest[0], row0_a);
      storeu4f(&dest[6], row1_a);
    }

    __forceinline void initFrom3x3Grid_discretized( const float4 source[3], unsigned short *const dest)
    {
      const float4 row0_a = unpacklo(source[0],source[1]); 
      const float4 row0_b = shuffle<0,1,0,1>(unpackhi(source[0],source[1]));
      const float4 row1_a = unpacklo(source[1],source[2]);
      const float4 row1_b = shuffle<0,1,0,1>(unpackhi(source[1],source[2]));

      //FIXME: use intrinsics for conversion
      for (size_t i=0;i<4;i++)
        dest[2+i] = (unsigned short)(row0_b[i]*65535.0f);
      for (size_t i=0;i<4;i++)
        dest[8+i] = (unsigned short)(row1_b[i]*65535.0f);
      for (size_t i=0;i<4;i++)
        dest[0+i] = (unsigned short)(row0_a[i]*65535.0f);
      for (size_t i=0;i<4;i++)
        dest[6+i] = (unsigned short)(row1_a[i]*65535.0f);

    }

    __forceinline void initFrom3x3Grid_discretized( const float *const source, unsigned short *const dest,
                                                    const size_t offset_line0, const size_t offset_line1, const size_t offset_line2)
    {
      const float v00 = source[offset_line0 + 0];
      const float v01 = source[offset_line0 + 1];
      const float v02 = source[offset_line0 + 2];
      const float v10 = source[offset_line1 + 0];
      const float v11 = source[offset_line1 + 1];
      const float v12 = source[offset_line1 + 2];
      const float v20 = source[offset_line2 + 0];
      const float v21 = source[offset_line2 + 1];
      const float v22 = source[offset_line2 + 2];

      /* v00 - v10 - v01 - v11 - v02 - v12 */
      dest[ 0] = float_to_u16(v00);
      dest[ 1] = float_to_u16(v10);
      dest[ 2] = float_to_u16(v01);
      dest[ 3] = float_to_u16(v11);
      dest[ 4] = float_to_u16(v02);
      dest[ 5] = float_to_u16(v12);
      /* v10 - v20 - v11 - v21 - v12 - v22 */
      dest[ 6] = float_to_u16(v10);
      dest[ 7] = float_to_u16(v20);
      dest[ 8] = float_to_u16(v11);
      dest[ 9] = float_to_u16(v21);
      dest[10] = float_to_u16(v12);
      dest[11] = float_to_u16(v22);
    }

    /* init from 3x3 point grid */
    void init( const float * const grid_x,
               const float * const grid_y,
               const float * const grid_z,
               const float * const grid_u,
               const float * const grid_v,
               const size_t offset_line0,
               const size_t offset_line1,
               const size_t offset_line2)
    {
      initFrom3x3Grid( grid_x, vtx_x, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_y, vtx_y, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_z, vtx_z, offset_line0, offset_line1, offset_line2 );
#if DISCRETIZED_UV == 1
      initFrom3x3Grid_discretized( grid_u, vtx_u, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid_discretized( grid_v, vtx_v, offset_line0, offset_line1, offset_line2 );
#else
      initFrom3x3Grid( grid_u, vtx_u, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_v, vtx_v, offset_line0, offset_line1, offset_line2 );
#endif
    }

    /* init from 3x3 point grid */
    void init( const float4 grid_x[3], const float4 grid_y[3], const float4 grid_z[3], const float4 grid_u[3], const float4 grid_v[3])
    {
      initFrom3x3Grid( grid_x, vtx_x);
      initFrom3x3Grid( grid_y, vtx_y);
      initFrom3x3Grid( grid_z, vtx_z);
#if DISCRETIZED_UV == 1
      initFrom3x3Grid_discretized( grid_u, vtx_u);
      initFrom3x3Grid_discretized( grid_v, vtx_v);
#else
      initFrom3x3Grid( grid_u, vtx_u );
      initFrom3x3Grid( grid_v, vtx_v );
#endif
    }

    /* init from 3x3 point grid */
    void init_xyz( const float4 grid_x[3], const float4 grid_y[3], const float4 grid_z[3])
    {
      initFrom3x3Grid( grid_x, vtx_x);
      initFrom3x3Grid( grid_y, vtx_y);
      initFrom3x3Grid( grid_z, vtx_z);
    }


#if defined(__AVX__)

    __forceinline float8 combine( const float *const source, const size_t offset) const {
      return float8( float4::loadu(&source[0+offset]), float4::loadu(&source[6+offset]) ); // FIXME: unaligned loads
    }

    __forceinline int8 combine_discretized( const unsigned short *const source, const size_t offset) const {  
      return int8( int4::load(&source[0+offset]), int4::load(&source[6+offset]) );            
    }

    __forceinline Vec3f8 getVtx( const size_t offset, const size_t delta = 0 ) const {
      return Vec3f8(  combine(vtx_x,offset), combine(vtx_y,offset), combine(vtx_z,offset) );
    }

    __forceinline Vec2f8 getUV( const size_t offset, const size_t delta = 0 ) const {
#if DISCRETIZED_UV == 1
      return Vec2f8(  float8(combine_discretized(vtx_u,offset)) * (1.0f/65535.0f), float8(combine_discretized(vtx_v,offset)) * (1.0f/65535.0f) )  ;
#else
      return Vec2f8(  combine(vtx_u,offset), combine(vtx_v,offset) );
#endif
    }

#else

    __forceinline Vec3f4 getVtx( const size_t offset, const size_t delta = 0 ) const {
      return Vec3f4( loadu4f(&vtx_x[offset+delta]), loadu4f(&vtx_y[offset+delta]), loadu4f(&vtx_z[offset+delta]) );
    }

    __forceinline Vec2f4 getUV( const size_t offset, const size_t delta = 0 ) const {
#if DISCRETIZED_UV == 1
      return Vec2f4( float4::load(&vtx_u[offset+delta]), float4::load(&vtx_v[offset+delta]) );
#else
      return Vec2f4(  loadu4f(&vtx_u[offset+delta]), loadu4f(&vtx_v[offset+delta])  );
#endif
    }
    
#endif

    
    __forceinline BBox3fa bounds() const 
    {
      BBox3fa b( empty );
      for (size_t i=0;i<12;i++)
        b.extend( Vec3fa(vtx_x[i],vtx_y[i],vtx_z[i]) );
      return b;
    }
    
    __forceinline Vec3fa getVec3fa_xyz(const size_t i) const {
      return Vec3fa( vtx_x[i], vtx_y[i], vtx_z[i] );
    }

    __forceinline Vec2f getVec2f_uv(const size_t i) const {
      return Vec2f( vtx_u[i], vtx_v[i] );
    }

  };

  inline std::ostream& operator<<(std::ostream& cout, const Quad2x2& qquad) {
    for (size_t i=0;i<12;i++)
      cout << "i = " << i << " -> xyz = " << qquad.getVec3fa_xyz(i) << " uv = " << qquad.getVec2f_uv(i) << std::endl;
    return cout;
  }
}
