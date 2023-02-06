// Copyright 2009-2023 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../common/math/vec3.h"

#define HIGH_PRECISION_OFFSETS 0

namespace embree {

  enum {
    NO_BORDER     = 0,
    TOP_BORDER    = 1 << 0,
    BOTTOM_BORDER = 1 << 1,
    LEFT_BORDER   = 1 << 2,
    RIGHT_BORDER  = 1 << 3,
    FULL_BORDER   = TOP_BORDER|BOTTOM_BORDER|LEFT_BORDER|RIGHT_BORDER,
    CRACK_FIXED_BORDER = 1 << 31    
  };

  struct LODEdgeLevel {
    uchar top,right,bottom,left; // top,right,bottom,left
    __forceinline LODEdgeLevel() {}
    __forceinline LODEdgeLevel(const uchar top, const uchar right, const uchar bottom, const uchar left) : top(top),right(right),bottom(bottom),left(left) {}
    __forceinline LODEdgeLevel(const uchar all) : top(all),right(all),bottom(all),left(all) {}
    
    __forceinline uint level() const { return max(max(top,right),max(bottom,left)); }

    __forceinline bool needsCrackFixing() const {
      const uint l = level();
      if ( l != top || l != right || l != bottom || l != left) return true;
      return false;
    }
  };

  struct BilinearPatch
  {
    const Vec3f v0,v1,v2,v3;
    
    __forceinline BilinearPatch(const Vec3f &v0,const Vec3f &v1,const Vec3f &v2,const Vec3f &v3) : v0(v0),v1(v1),v2(v2),v3(v3) {}                                    
    __forceinline Vec3f eval(const float u, const float v) const
    {
      return lerp(lerp(v0,v1,u),lerp(v3,v2,u),v);
    }    
  };    
  
  struct __aligned(64) LCGBP
  {
#if HIGH_PRECISION_OFFSETS == 0    
    static const uint SIGN_BIT      = 1;
    static const uint EXPONENT_BITS = 5;    
    static const uint MANTISSA_BITS = 4;
#else
    static const uint SIGN_BIT      = 1;
    static const uint EXPONENT_BITS = 7;    
    static const uint MANTISSA_BITS = 8;    
#endif    
    static const uint RP_BITS       = SIGN_BIT + EXPONENT_BITS + MANTISSA_BITS;
    static const uint EXPONENT_MASK = ((uint)1 << EXPONENT_BITS)-1;    
    static const uint MANTISSA_MASK = ((uint)1 << MANTISSA_BITS)-1;
    static const uint RP_MASK       = ((uint)1 << RP_BITS)-1;
    static const uint FLOAT_EXPONENT_BIAS = 127; // 127-1 due to zigzag encoding
    static const uint FLOAT_MANTISSA_BITS = 23;

    static const uint GRID_RES_VERTEX = 33;
    static const uint GRID_RES_QUAD   = GRID_RES_VERTEX-1;
  
    template<typename T>
      static __forceinline uint as_uint(T t)
    {
      return __builtin_bit_cast(uint,t);
    }

    template<typename T>
      static __forceinline float as_float(T t)
    {
      return __builtin_bit_cast(float,t);
    }
  
    static const uint zigzagEncode(const int i)
    {
      return (i >> 31) ^ (i << 1);
    }
    
    static const int zigzagDecode(const uint i)
    {
      return (i >> 1) ^ -(i & 1);
    }
        
    static __forceinline uint encodeFloat(const float diff) 
    {
      int exponent = (int)((as_uint(diff) >> 23) & 0xff) - FLOAT_EXPONENT_BIAS;
      uint mantissa = as_uint(diff) & (((uint)1<<23)-1);
      const bool small = exponent < -(int)((1 << (EXPONENT_BITS-1))-1);      
      /*--- clamp exponent---*/
      exponent = max(exponent, -(int)((1 << (EXPONENT_BITS-1))-1) );
      exponent = min(exponent,  (int)((1 << (EXPONENT_BITS-1))-1) );      
      uint exp = zigzagEncode(exponent);
      if (diff == 0.0f || diff == -0.0f || small) exp = EXPONENT_MASK;      
      const uint mantissa_rest = mantissa & (((uint)1<<(FLOAT_MANTISSA_BITS-MANTISSA_BITS-1)));
      /*--- clamp mantissa---*/            
      mantissa >>= FLOAT_MANTISSA_BITS-MANTISSA_BITS;
      /*--- round mantissa---*/            
      mantissa += mantissa_rest ? 1 : 0;
      mantissa = min(mantissa,MANTISSA_MASK);
      mantissa &= MANTISSA_MASK;
      const uint sign = diff < 0.0f ? ((uint)1<<(RP_BITS-1)) : 0;
      return sign | (exp << MANTISSA_BITS) | mantissa;      
    }    

    static __forceinline float decodeFloat(const uint input)
    {
      const uint sign = (input >> (RP_BITS-1)) << 31;
      const uint exp_bits = (input >> MANTISSA_BITS) & EXPONENT_MASK;
      const uint exp  = ( ((uint)zigzagDecode(exp_bits)+FLOAT_EXPONENT_BIAS)<<FLOAT_MANTISSA_BITS);
      const uint mant = ((input & MANTISSA_MASK) << (FLOAT_MANTISSA_BITS-MANTISSA_BITS));
      const uint output = exp_bits != EXPONENT_MASK ? (sign|exp|mant) : 0;
      return as_float(output);
    }
    
    struct LCType
    {
#if HIGH_PRECISION_OFFSETS == 0          
      uint data;
#else
      ushort data_x,data_y,data_z;
#endif      

      __forceinline LCType() {}
      
      __forceinline LCType(const Vec3f &diff)
      {
        const uint x = encodeFloat(diff.x);
        const uint y = encodeFloat(diff.y);
        const uint z = encodeFloat(diff.z);
#if HIGH_PRECISION_OFFSETS == 0                  
        data = x | (y << (1*RP_BITS)) | (z << (2*RP_BITS));
#else
        data_x = x; data_y = y; data_z = z;
#endif        
      }
      
      __forceinline Vec3f decode() const
      {
#if HIGH_PRECISION_OFFSETS == 0                          
        const float px = decodeFloat((data>>0*RP_BITS) & RP_MASK);
        const float py = decodeFloat((data>>1*RP_BITS) & RP_MASK);
        const float pz = decodeFloat((data>>2*RP_BITS) & RP_MASK);
#else
        const float px = decodeFloat(data_x);
        const float py = decodeFloat(data_y);
        const float pz = decodeFloat(data_z);        
#endif        
        return Vec3f(px,py,pz);
      }
      
    };


    BilinearPatch patch;
    int neighbor_top,neighbor_right,neighbor_bottom,neighbor_left;
    Vec2f u_range;
    Vec2f v_range;        
    uint ID;
    LCType lc_offsets[GRID_RES_VERTEX*GRID_RES_VERTEX];    
            
    __forceinline LCType encode(const Vec3f &p, const uint x, const uint y, const uint gridResX, const uint gridResY)
    {
      const float u = (float)x / (gridResX-1);
      const float v = (float)y / (gridResY-1);
      const Vec3f bp_p = patch.eval(u,v);      
      const Vec3f diff = p - bp_p;
      return LCType(diff);
    }

    __forceinline Vec3f decode(const LCType &input, const uint x, const uint y, const uint gridResX, const uint gridResY) const
    {
      const float u = (float)x / (gridResX-1);
      const float v = (float)y / (gridResY-1);      
      const Vec3f bp_p = patch.eval(u,v);
      const Vec3f offset = input.decode();
      return bp_p + offset;
    }

    __forceinline Vec3f decode(const uint x, const uint y) const
    {
      return decode(lc_offsets[y*GRID_RES_VERTEX+x],x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
    }

    __forceinline Vec3f decode(const LCType &input, const uint x, const uint y) const
    {
      return decode(input,x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
    }
    
    
    __forceinline Vec3fa getVertexGrid9x9(const uint x, const uint y, const uint step, const uint start_x, const uint start_y) const
    {
      return decode(start_x + x*step,start_y + y*step);
    }

    
    __forceinline Vec3fa getVertex(const uint x, const uint y, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY)
    {
      const uint px = min(x,grid_resX-1);
      const uint py = min(y,grid_resY-1);    
      return vtx[py*grid_resX + px];
    }

    __forceinline void encode(const uint start_x, const uint start_y, const Vec3fa *const vtx, const uint gridResX, const uint gridResY)
    {
      uint index = 0;
      for (int y=0;y<GRID_RES_VERTEX;y++)
        for (int x=0;x<GRID_RES_VERTEX;x++)
        {
          const Vec3f vertex = getVertex(start_x+x,start_y+y,vtx,gridResX,gridResY);
          lc_offsets[index] = encode(vertex,x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
          index++;
        }      
    }
    
    __forceinline LCGBP(const Vec3f &v0,const Vec3f &v1,const Vec3f &v2,const Vec3f &v3,const uint ID, const Vec2f& u_range,const Vec2f& v_range, const uint neighbor_top, const int neighbor_right, const int neighbor_bottom, const int neighbor_left) : patch(v0,v1,v2,v3),neighbor_top(neighbor_top),neighbor_right(neighbor_right),neighbor_bottom(neighbor_bottom),neighbor_left(neighbor_left),u_range(u_range),v_range(v_range),ID(ID)  {}
    
  };


  struct LCGBP_State
  {
    LCGBP *lcgbp;
    uchar start_x, start_y, step, localID;
    uchar blend, lod_diff_levels, lod_level, flags;

    LCGBP_State() {}

    LCGBP_State(LCGBP *lcgbp, const uint start_x, const uint start_y, const uint step, const uint localID, const uint lod_level, const LODEdgeLevel &lod_levels, const uint blend) : lcgbp(lcgbp), start_x(start_x), start_y(start_y), step(step), localID(localID), blend(blend), lod_diff_levels(0), lod_level(lod_level)
    {
      uint border_flags = 0;
      border_flags |= start_x == 0 ? LEFT_BORDER : 0;
      border_flags |= start_y == 0 ? TOP_BORDER  : 0;
      border_flags |= start_y+step*8 == LCGBP::GRID_RES_QUAD ? BOTTOM_BORDER : 0;
      border_flags |= start_x+step*8 == LCGBP::GRID_RES_QUAD ? RIGHT_BORDER : 0;

      //lod_level = lod_levels.level();

      if (border_flags & TOP_BORDER)
        lod_diff_levels |= (lod_level - lod_levels.top) << 0;

      if (border_flags & RIGHT_BORDER)
        lod_diff_levels |= (lod_level - lod_levels.right) << 2;

      if (border_flags & BOTTOM_BORDER)
        lod_diff_levels |= (lod_level - lod_levels.bottom) << 4;

      if (border_flags & LEFT_BORDER)
        lod_diff_levels |= (lod_level - lod_levels.left) << 6;

      flags = border_flags;
    }

    __forceinline bool cracksFixed()
    {
      return lod_diff_levels != 0;
    }

    __forceinline uchar get_lod_diff_level(const uint index) const { return (lod_diff_levels >> (index*2)) & 3; }
    
    
  };
  
};
