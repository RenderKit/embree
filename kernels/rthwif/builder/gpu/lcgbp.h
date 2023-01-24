// Copyright 2009-2023 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../common/math/vec3.h"

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
    __forceinline uint level() const { return max(max(top,right),max(bottom,left)); }

    __forceinline bool needsCrackFixing() const {
      const uint l = level();
      if ( l != top || l != right || l != bottom || l != left) return true;
      return false;
    }
  };
  
  struct __aligned(64) LCGBP
  {
    static const uint SIGN_BIT      = 1;
    static const uint EXPONENT_BITS = 5;    
    static const uint MANTISSA_BITS = 4;
    static const uint RP_BITS       = SIGN_BIT + EXPONENT_BITS + MANTISSA_BITS;
    static const uint EXPONENT_MASK = ((uint)1 << EXPONENT_BITS)-1;    
    static const uint MANTISSA_MASK = ((uint)1 << MANTISSA_BITS)-1;
    static const uint RP_MASK       = ((uint)1 << RP_BITS)-1;

    static const uint FLOAT_EXPONENT_BIAS = 127; // 127-1 due to zigzag encoding
    static const uint FLOAT_MANTISSA_BITS = 23;
    

    static const uint GRID_RES_VERTEX = 33;
    static const uint GRID_RES_QUAD   = GRID_RES_VERTEX-1;

    const Vec3f v0,v1,v2,v3;
    uint flags;
    uint lc_offsets[GRID_RES_VERTEX*GRID_RES_VERTEX];

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
    
    __forceinline Vec3f evalBilinearPatch(const float u, const float v) const
    {
      return lerp(lerp(v0,v1,u),lerp(v3,v2,u),v);
    }
    
    
    __forceinline uint encode(const Vec3f &p, const uint x, const uint y, const uint gridResX, const uint gridResY)
    {
      const float u = (float)x / (gridResX-1);
      const float v = (float)y / (gridResY-1);
      //PRINT4(as_uint(u),as_uint(v),as_uint(1.0f-u),as_uint(1.0f-v));
      const Vec3f bp_p = evalBilinearPatch(u,v);
      
      const Vec3f diff = p - bp_p;
      const uint rp_x = encodeFloat(diff.x);
      const uint rp_y = encodeFloat(diff.y);
      const uint rp_z = encodeFloat(diff.z);
      //PRINT5(diff,as_uint(diff.x),rp_x,rp_y,rp_z);      
      return rp_x | (rp_y << (1*RP_BITS)) | (rp_z << (2*RP_BITS));
    }

    __forceinline Vec3f decode(const uint input, const uint x, const uint y, const uint gridResX, const uint gridResY) const
    {
      const float u = (float)x / (gridResX-1);
      const float v = (float)y / (gridResY-1);      
      const Vec3f bp_p = evalBilinearPatch(u,v);
      //PRINT3(u,v,bp_p);
      const float px = bp_p.x + decodeFloat((input>>0*RP_BITS) & RP_MASK);
      const float py = bp_p.y + decodeFloat((input>>1*RP_BITS) & RP_MASK);
      const float pz = bp_p.z + decodeFloat((input>>2*RP_BITS) & RP_MASK);
      return Vec3f(px,py,pz);
    }

    __forceinline Vec3f decode(const uint x, const uint y) const
    {
      return decode(lc_offsets[y*GRID_RES_VERTEX+x],x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
    }

    __forceinline Vec3f getOffset(const uint x, const uint y) const
    {
      return decode(lc_offsets[y*GRID_RES_VERTEX+x],x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
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
           const Vec3f new_vertex = decode(x,y); 
           const float error = length(vertex-new_vertex);             
           //PRINT6(x,y,vertex,new_vertex,error,as_uint(error));
           //PRINT2(as_uint(vertex.x),as_uint(new_vertex.x));
           //if (error > 0.01) exit(0); 
          index++;
        }      
    }
    
    __forceinline LCGBP() {}
    __forceinline LCGBP(const Vec3f &v0,const Vec3f &v1,const Vec3f &v2,const Vec3f &v3) : v0(v0),v1(v1),v2(v2),v3(v3),flags(0) {}
    
  };


  struct LCGBP_State
  {
    LCGBP *lcgbp;
    uchar start_x, start_y, step, flags;
    LODEdgeLevel lod_levels;

    LCGBP_State() {}

  LCGBP_State(LCGBP *lcgbp, const uint start_x, const uint start_y, const uint step, const uint lod_top, const uint lod_right, const uint lod_bottom, const uint lod_left) : lcgbp(lcgbp), start_x(start_x), start_y(start_y), step(step), flags(0), lod_levels(lod_top,lod_right,lod_bottom,lod_left) {}
    
  };
  
};
