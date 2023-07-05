// Copyright 2009-2023 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../common/math/vec3.h"

#define HIGH_PRECISION_OFFSETS 0

namespace embree {

  __forceinline float3 to_float3(const Vec3f &v)
  {
    return float3(v.x,v.y,v.z);
  }
  
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
    unsigned char top,right,bottom,left; // top,right,bottom,left
    __forceinline LODEdgeLevel() {}
    __forceinline LODEdgeLevel(const unsigned char top, const unsigned char right, const unsigned char bottom, const unsigned char left) : top(top),right(right),bottom(bottom),left(left) {}
    __forceinline LODEdgeLevel(const unsigned char all) : top(all),right(all),bottom(all),left(all) {}
    
    __forceinline unsigned int level() const { return max(max(top,right),max(bottom,left)); }

    __forceinline bool needsCrackFixing() const {
      const unsigned int l = level();
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

    __forceinline BBox3f bounds() const
    {
      BBox3f bounds(empty);
      bounds.extend(v0);      
      bounds.extend(v1);
      bounds.extend(v2);
      bounds.extend(v3);
      return bounds;
    }
  };    
  
  struct __aligned(64) LCGBP
  {
#if HIGH_PRECISION_OFFSETS == 0    
    static const unsigned int SIGN_BIT      = 1;
    static const unsigned int EXPONENT_BITS = 5;    
    static const unsigned int MANTISSA_BITS = 4;
#else
    static const unsigned int SIGN_BIT      = 1;
    static const unsigned int EXPONENT_BITS = 7;    
    static const unsigned int MANTISSA_BITS = 8;    
#endif    
    static const unsigned int RP_BITS       = SIGN_BIT + EXPONENT_BITS + MANTISSA_BITS;
    static const unsigned int EXPONENT_MASK = ((unsigned int)1 << EXPONENT_BITS)-1;    
    static const unsigned int MANTISSA_MASK = ((unsigned int)1 << MANTISSA_BITS)-1;
    static const unsigned int RP_MASK       = ((unsigned int)1 << RP_BITS)-1;
    static const unsigned int FLOAT_EXPONENT_BIAS = 127; // 127-1 due to zigzag encoding
    static const unsigned int FLOAT_MANTISSA_BITS = 23;

    static const unsigned int GRID_RES_VERTEX = 33;
    static const unsigned int GRID_RES_QUAD   = GRID_RES_VERTEX-1;
  
    template<typename T>
      static __forceinline unsigned int as_uint(T t)
    {
      return __builtin_bit_cast(unsigned int,t);
    }

    template<typename T>
      static __forceinline float as_float(T t)
    {
      return __builtin_bit_cast(float,t);
    }
  
    static const unsigned int zigzagEncode(const int i)
    {
      return (i >> 31) ^ (i << 1);
    }
    
    static const int zigzagDecode(const unsigned int i)
    {
      return (i >> 1) ^ -(i & 1);
    }
        
    static __forceinline unsigned int encodeFloat(const float diff) 
    {
      int exponent = (int)((as_uint(diff) >> 23) & 0xff) - FLOAT_EXPONENT_BIAS;
      unsigned int mantissa = as_uint(diff) & (((unsigned int)1<<23)-1);
      const bool small = exponent < -(int)((1 << (EXPONENT_BITS-1))-1);      
      /*--- clamp exponent---*/
      exponent = max(exponent, -(int)((1 << (EXPONENT_BITS-1))-1) );
      exponent = min(exponent,  (int)((1 << (EXPONENT_BITS-1))-1) );      
      unsigned int exp = zigzagEncode(exponent);
      if (diff == 0.0f || diff == -0.0f || small) exp = EXPONENT_MASK;      
      const unsigned int mantissa_rest = mantissa & (((unsigned int)1<<(FLOAT_MANTISSA_BITS-MANTISSA_BITS-1)));
      /*--- clamp mantissa---*/            
      mantissa >>= FLOAT_MANTISSA_BITS-MANTISSA_BITS;
      /*--- round mantissa---*/            
      mantissa += mantissa_rest ? 1 : 0;
      mantissa = min(mantissa,MANTISSA_MASK);
      mantissa &= MANTISSA_MASK;
      const unsigned int sign = diff < 0.0f ? ((unsigned int)1<<(RP_BITS-1)) : 0;
      return sign | (exp << MANTISSA_BITS) | mantissa;      
    }    

    static __forceinline float decodeFloat(const unsigned int input)
    {
      const unsigned int sign = (input >> (RP_BITS-1)) << 31;
      const unsigned int exp_bits = (input >> MANTISSA_BITS) & EXPONENT_MASK;
      const unsigned int exp  = ( ((unsigned int)zigzagDecode(exp_bits)+FLOAT_EXPONENT_BIAS)<<FLOAT_MANTISSA_BITS);
      const unsigned int mant = ((input & MANTISSA_MASK) << (FLOAT_MANTISSA_BITS-MANTISSA_BITS));
      const unsigned int output = exp_bits != EXPONENT_MASK ? (sign|exp|mant) : 0;
      return as_float(output);
    }
    
    struct LCType
    {
#if HIGH_PRECISION_OFFSETS == 0          
      unsigned int data;
#else
      unsigned short data_x,data_y,data_z;
#endif      

      __forceinline LCType() {}
      
      __forceinline LCType(const Vec3f &diff)
      {
        const unsigned int x = encodeFloat(diff.x);
        const unsigned int y = encodeFloat(diff.y);
        const unsigned int z = encodeFloat(diff.z);
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
    unsigned int ID;
    LCType lc_offsets[GRID_RES_VERTEX*GRID_RES_VERTEX];    
            
    __forceinline LCType encode(const Vec3f &p, const unsigned int x, const unsigned int y, const unsigned int gridResX, const unsigned int gridResY)
    {
      const float u = (float)x / (gridResX-1);
      const float v = (float)y / (gridResY-1);
      const Vec3f bp_p = patch.eval(u,v);      
      const Vec3f diff = p - bp_p;
      return LCType(diff);
    }

    __forceinline Vec3f decode(const LCType &input, const unsigned int x, const unsigned int y, const unsigned int gridResX, const unsigned int gridResY) const
    {
      const float u = (float)x / (gridResX-1);
      const float v = (float)y / (gridResY-1);      
      const Vec3f bp_p = patch.eval(u,v);
      const Vec3f offset = input.decode();
      return bp_p + offset;
    }

    __forceinline Vec3f decode(const unsigned int x, const unsigned int y) const
    {
      return decode(lc_offsets[y*GRID_RES_VERTEX+x],x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
    }

    __forceinline Vec3f decode(const LCType &input, const unsigned int x, const unsigned int y) const
    {
      return decode(input,x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
    }
    
    
    __forceinline Vec3fa getVertexGrid9x9(const unsigned int x, const unsigned int y, const unsigned int step, const unsigned int start_x, const unsigned int start_y) const
    {
      return decode(start_x + x*step,start_y + y*step);
    }

    
    __forceinline Vec3fa getVertex(const unsigned int x, const unsigned int y, const Vec3fa *const vtx, const unsigned int grid_resX, const unsigned int grid_resY)
    {
      const unsigned int px = min(x,grid_resX-1);
      const unsigned int py = min(y,grid_resY-1);    
      return vtx[py*grid_resX + px];
    }

    __forceinline void encode(const unsigned int start_x, const unsigned int start_y, const Vec3fa *const vtx, const unsigned int gridResX, const unsigned int gridResY)
    {
      unsigned int index = 0;
      for (int y=0;y<GRID_RES_VERTEX;y++)
        for (int x=0;x<GRID_RES_VERTEX;x++)
        {
          const Vec3f vertex = getVertex(start_x+x,start_y+y,vtx,gridResX,gridResY);
          lc_offsets[index] = encode(vertex,x,y,GRID_RES_VERTEX,GRID_RES_VERTEX);
          index++;
        }      
    }
    
    __forceinline LCGBP(const Vec3f &v0,const Vec3f &v1,const Vec3f &v2,const Vec3f &v3,const unsigned int ID, const Vec2f& u_range,const Vec2f& v_range, const unsigned int neighbor_top, const int neighbor_right, const int neighbor_bottom, const int neighbor_left) : patch(v0,v1,v2,v3),neighbor_top(neighbor_top),neighbor_right(neighbor_right),neighbor_bottom(neighbor_bottom),neighbor_left(neighbor_left),u_range(u_range),v_range(v_range),ID(ID)  {}
    
  };


  struct LCGBP_State
  {
    LCGBP *lcgbp;
    unsigned char start_x, start_y, step, localID;
    unsigned char blend, lod_diff_levels, lod_level, flags;

    LCGBP_State() {}

    LCGBP_State(LCGBP *lcgbp, const unsigned int start_x, const unsigned int start_y, const unsigned int step, const unsigned int localID, const unsigned int lod_level, const LODEdgeLevel &lod_levels, const unsigned int blend) : lcgbp(lcgbp), start_x(start_x), start_y(start_y), step(step), localID(localID), blend(blend), lod_diff_levels(0), lod_level(lod_level)
    {
      unsigned int border_flags = 0;
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

    __forceinline unsigned char get_lod_diff_level(const unsigned int index) const { return (lod_diff_levels >> (index*2)) & 3; }        
  };


  // =====================================================================================================================================================================
  // =====================================================================================================================================================================
  // =====================================================================================================================================================================

  struct CompressedVertex
  {
    unsigned short x,y,z;
    
    static const unsigned int BITS_PER_DIM = 16;
    static const unsigned int BITS_MASK = ((unsigned int)1<<BITS_PER_DIM)-1;
    static const unsigned int RES_PER_DIM = BITS_MASK;
    
    static const unsigned short MIN_VALUE = 0;
    static const unsigned short MAX_VALUE = BITS_MASK;
    
    __forceinline CompressedVertex() {}

    __forceinline CompressedVertex(const unsigned short x, const unsigned short y, const unsigned short z) : x(x), y(y), z(z) {}

    __forceinline CompressedVertex(const unsigned short v) : x(v), y(v), z(v) {}
    
    
    __forceinline CompressedVertex(const Vec3f &org, const Vec3f &lower, const Vec3f &inv_diag)
    {
#if 0      
      const Vec3f norm_v = (org - lower)*inv_diag;
      x = (unsigned int)(floorf(norm_v.x * (float)RES_PER_DIM));
      y = (unsigned int)(floorf(norm_v.y * (float)RES_PER_DIM));
      z = (unsigned int)(floorf(norm_v.z * (float)RES_PER_DIM));
#else
      x = (unsigned int)(floorf((org.x - lower.x)*inv_diag.x * (float)RES_PER_DIM));
      y = (unsigned int)(floorf((org.y - lower.y)*inv_diag.y * (float)RES_PER_DIM));
      z = (unsigned int)(floorf((org.z - lower.z)*inv_diag.z * (float)RES_PER_DIM));
      
#endif      
    }    

    __forceinline Vec3f decompress(const Vec3f &lower, const Vec3f &diag) const
    {
      const Vec3f vv((float)x,(float)y,(float)z);
      return madd(vv,diag,lower);
    }

    __forceinline CompressedVertex sub_group_reduce_min() const
    {
      CompressedVertex result;
      result.x = embree::sub_group_reduce(x, SYCL_EXT_ONEAPI::minimum<unsigned short>());
      result.y = embree::sub_group_reduce(y, SYCL_EXT_ONEAPI::minimum<unsigned short>());
      result.z = embree::sub_group_reduce(z, SYCL_EXT_ONEAPI::minimum<unsigned short>());
      return result;	
    }

    __forceinline CompressedVertex sub_group_reduce_max() const
    {
      CompressedVertex result;
      result.x = embree::sub_group_reduce(x, SYCL_EXT_ONEAPI::maximum<unsigned short>());
      result.y = embree::sub_group_reduce(y, SYCL_EXT_ONEAPI::maximum<unsigned short>());
      result.z = embree::sub_group_reduce(z, SYCL_EXT_ONEAPI::maximum<unsigned short>());
      return result;	
    }
    
  };

  __forceinline bool operator ==( const CompressedVertex& a, const CompressedVertex& b ) { return a.x == b.x && a.y == b.y && a.z == b.z; }
  __forceinline bool operator !=( const CompressedVertex& a, const CompressedVertex& b ) { return a.x != b.x || a.y == b.y || a.z == b.z; }
  

  
  __forceinline CompressedVertex min(const CompressedVertex &v0,  const CompressedVertex &v1)
  {
    return CompressedVertex(min(v0.x,v1.x),min(v0.y,v1.y),min(v0.z,v1.z));
  }

  __forceinline CompressedVertex max(const CompressedVertex &v0,  const CompressedVertex &v1)
  {
    return CompressedVertex(max(v0.x,v1.x),max(v0.y,v1.y),max(v0.z,v1.z));
  }
  
  
  struct CompressedAABB3f
  {
    CompressedVertex lower,upper;

    __forceinline CompressedAABB3f() {}
    
    __forceinline CompressedAABB3f(const CompressedVertex &lower, const CompressedVertex &upper) : lower(lower), upper(upper) {}

    __forceinline CompressedAABB3f(const CompressedVertex &v) : lower(v),upper(v) {}

    __forceinline CompressedAABB3f(const CompressedAABB3f &v) : lower(v.lower),upper(v.upper) {}

    __forceinline void init()
    {
      lower = CompressedVertex::MAX_VALUE;
      upper = CompressedVertex::MIN_VALUE;      
    }
    
    __forceinline void extend(const CompressedAABB3f &v)
    {
      lower = min(lower,v.lower);
      upper = max(upper,v.upper);      
    }

    __forceinline void extend(const CompressedVertex &v)
    {
      lower = min(lower,v);
      upper = max(upper,v);      
    }
    
    __forceinline BBox3f decompress(const Vec3f &start, const Vec3f &diag) const
    {
      const Vec3f lower_f = lower.decompress(start,diag);
      const Vec3f upper_f = upper.decompress(start,diag);
      return BBox3f(lower_f,upper_f);
    }

    // __forceinline gpu::AABB3f decompressAABB3f(const Vec3f &start, const Vec3f &diag) const
    // {
    //   const Vec3f lower_f = lower.decompress(start,diag);
    //   const Vec3f upper_f = upper.decompress(start,diag);
    //   return gpu::AABB3f(to_float3(lower_f),to_float3(upper_f));
    // }
    
   
    
    __forceinline CompressedAABB3f sub_group_reduce() const
    {
      const CompressedVertex l = lower.sub_group_reduce_min();
      const CompressedVertex u = upper.sub_group_reduce_max();      
      return CompressedAABB3f(l,u);
    }
 
    
  };  

  struct CompressedQuadIndices
  {
    unsigned char _v0,_v1,_v2,_v3; 

    __forceinline unsigned int v0() const { return _v0; }
    __forceinline unsigned int v1() const { return _v1; }
    __forceinline unsigned int v2() const { return _v2; }
    __forceinline unsigned int v3() const { return _v3; }
    
    __forceinline CompressedQuadIndices() {}
    __forceinline CompressedQuadIndices(const unsigned int v0,const unsigned int v1,const unsigned int v2,const unsigned int v3) : _v0(v0), _v1(v1), _v2(v2), _v3(v3) {}
    
  };

  struct __aligned(32) LossyCompressedMesh
  {
    BBox3f bounds;
    unsigned int numQuads;
    unsigned int numVertices;
    unsigned int geomID;
    unsigned int tmp[3];
    
    CompressedVertex *compressedVertices;
    CompressedQuadIndices *compressedIndices;
    
    __forceinline LossyCompressedMesh() {}

    __forceinline BBox3f decompressQuadBounds(const CompressedQuadIndices &indices,const Vec3f &lower, const Vec3f &diag) const
    {
      const Vec3f vtx0 = compressedVertices[indices.v0()].decompress(lower,diag);
      const Vec3f vtx1 = compressedVertices[indices.v1()].decompress(lower,diag);
      const Vec3f vtx2 = compressedVertices[indices.v2()].decompress(lower,diag);
      const Vec3f vtx3 = compressedVertices[indices.v3()].decompress(lower,diag);
      BBox3f quadBounds(empty);
      quadBounds.extend(vtx0);
      quadBounds.extend(vtx1);
      quadBounds.extend(vtx2);
      quadBounds.extend(vtx3);
      return quadBounds;
    }
  };
  

  struct __aligned(8) LossyCompressedMeshCluster
  {
    //static const unsigned int MAX_QUADS_PER_CLUSTER = 16*6; //128;
    static const unsigned int MAX_QUADS_PER_CLUSTER = 128;    
    unsigned char numQuads;            
    unsigned char numBlocks;
    unsigned char lod_level;
    unsigned char tmp;                 // 4 bytes
    CompressedAABB3f bounds;   // 12 bytes
    LossyCompressedMesh *mesh; // 8 bytes    
    
    uint32_t leftID;            
    uint32_t rightID; 
    uint32_t neighborID;
    unsigned int offsetIndices;
    unsigned int offsetVertices;       // 16 bytes
    
    __forceinline LossyCompressedMeshCluster() {}

    __forceinline bool hasChildren() const { return leftID != -1; }
    __forceinline bool hasNeighbor() const { return neighborID != -1; }
    
    static __forceinline unsigned int getDecompressedInnerNodesSizeInBytes(const unsigned int numQuads) 
    {
      unsigned int numCacheLines = 0;
      unsigned int numPrims = numQuads;
      while(1)
      {
        unsigned int numNodes = (numPrims+5)/6;
        if (numNodes == 1) break;
        numCacheLines += numNodes;
        numPrims = numNodes;        
      }
      numCacheLines++;
      return numCacheLines*64;        
    }

    static __forceinline unsigned int getDecompressedLeavesSizeInBytes(const unsigned int numQuads)
    {
      return numQuads * 64;
    }
    
    static __forceinline unsigned int getDecompressedSizeInBytes(const unsigned int numQuads)
    {
      return getDecompressedInnerNodesSizeInBytes(numQuads) + getDecompressedLeavesSizeInBytes(numQuads);
    }
    
  };
  
};
