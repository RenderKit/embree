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

#include <cmath>
#include "bspline_patch.h"
#include "bezier_patch.h"
#include "gregory_patch.h"
#include "gregory_patch_dense.h"
#include "gregory_triangle_patch.h"
#include "tessellation.h"
#include "tessellation_cache.h"

#define FORCE_TESSELLATION_BOUNDS 1
#define USE_DISPLACEMENT_FOR_TESSELLATION_BOUNDS 1

#if defined(__MIC__)
#define USE_RANGE_EVAL 0
#else
#define USE_RANGE_EVAL RTCORE_USE_RANGE_EVAL
#endif

#if USE_RANGE_EVAL
#include "feature_adaptive_eval2.h"
#endif

namespace embree
{
  struct __aligned(16) GridRange
  {
    unsigned int u_start;
    unsigned int u_end;
    unsigned int v_start;
    unsigned int v_end;

    __forceinline GridRange() {}

#if defined(__MIC__)    
    __forceinline void operator=(const GridRange& v) {
      store4f((float*)this,broadcast4to16f((float*)&v));
    };
#endif

    __forceinline GridRange(unsigned int u_start, unsigned int u_end, unsigned int v_start, unsigned int v_end) 
      : u_start(u_start), u_end(u_end), v_start(v_start), v_end(v_end) {}

    __forceinline bool hasLeafSize() const
    {
#if defined(__MIC__)
      return u_end-u_start <= 1 && v_end-v_start <= 1; // FIXME: what does this ???
#else
      const unsigned int u_size = u_end-u_start+1;
      const unsigned int v_size = v_end-v_start+1;
      assert(u_size >= 1);
      assert(v_size >= 1);
      return u_size <= 3 && v_size <= 3;
#endif
    }

    static __forceinline unsigned int split(unsigned int start,unsigned int end)
    {
      const unsigned int center = (start+end)/2;
#if defined(__MIC__)
      assert(center<end);
#else
      assert (center > start);
      assert (center < end);
#endif
      return center;
    }

    __forceinline void split(GridRange& r0, GridRange& r1) const
    {
      assert( hasLeafSize() == false );
      const unsigned int u_size = u_end-u_start+1;
      const unsigned int v_size = v_end-v_start+1;
      r0 = *this;
      r1 = *this;

      if (u_size >= v_size)
      {
        const unsigned int u_mid = split(u_start,u_end);
        r0.u_end   = u_mid;
        r1.u_start = u_mid;
      }
      else
      {
        const unsigned int v_mid = split(v_start,v_end);
        r0.v_end   = v_mid;
        r1.v_start = v_mid;
      }
    }

    __forceinline unsigned int splitIntoSubRanges(GridRange r[4]) const
    {
      assert( !hasLeafSize() );
      size_t children = 0;
      GridRange first,second;
      split(first,second);

      if (first.hasLeafSize()) {
        r[0] = first;
        children++;
      } 
      else {
        first.split(r[0],r[1]);
        children += 2;
      }

      if (second.hasLeafSize())	{
        r[children] = second;
        children++;
      }
      else {
        second.split(r[children+0],r[children+1]);
        children += 2;
      }
      return children;      
    }
  };

  struct __aligned(64) SubdivPatch1Base
  {
  public:

    enum Type {
      BSPLINE_PATCH          = 1,  
      BEZIER_PATCH           = 2,  
      GREGORY_PATCH          = 4,
      GREGORY_TRIANGLE_PATCH = 8
    };

    enum Flags {
      TRANSITION_PATCH       = 16,  // needs stiching?
      HAS_DISPLACEMENT       = 32   // 0 => no displacments
    };

    /*! Default constructor. */
    __forceinline SubdivPatch1Base () {}

#if USE_RANGE_EVAL
    SubdivPatch1Base (const unsigned int gID,
                      const unsigned int pID,
                      const unsigned int subPatch,
                      const SubdivMesh *const mesh,
                      const Vec2f uv[4],
                      const float edge_level[4],
                      const int subdiv[4]);
#else
    /*! Construction from vertices and IDs. */
    SubdivPatch1Base (const CatmullClarkPatch3fa& ipatch,
                      const int fas_depth,
                      const unsigned int gID,
                      const unsigned int pID,
                      const SubdivMesh *const mesh,
                      const Vec2f uv[4],
                      const float edge_level[4],
                      const int subdiv[4],
                      const BezierCurve3fa *border, 
                      const int border_flags);
#endif

#if !USE_RANGE_EVAL
    __forceinline Vec3fa eval(const float uu, const float vv) const
    {
      if (likely(type == BEZIER_PATCH))
        return BezierPatch3fa::eval( patch.v, uu, vv );
      else if (likely(type == BSPLINE_PATCH))
        return patch.eval(uu,vv);
      else if (likely(type == GREGORY_PATCH))
	return DenseGregoryPatch3fa::eval( patch.v, uu, vv );
      else if (likely(type == GREGORY_TRIANGLE_PATCH))
	return GregoryTrianglePatch3fa::eval( patch.v, uu, vv );
      return Vec3fa( zero );
    }

    __forceinline Vec3fa normal(const float& uu, const float& vv) const
    {
      if (likely(type == BEZIER_PATCH))
        return BezierPatch3fa::normal( patch.v, uu, vv );
      else if (likely(type == BSPLINE_PATCH))
        return patch.normal(uu,vv);
      else if (likely(type == GREGORY_PATCH))
	return DenseGregoryPatch3fa::normal( patch.v, uu, vv );
      else if (likely(type == GREGORY_TRIANGLE_PATCH))
	return GregoryTrianglePatch3fa::normal( patch.v, uu, vv );
      return Vec3fa( zero );
    }

    template<typename simdf>
      __forceinline Vec3<simdf> eval(const simdf& uu, const simdf& vv) const
    {
      typedef typename simdf::Mask simdb;
      if (likely(type == BEZIER_PATCH))
        return BezierPatch3fa::eval( patch.v, uu, vv );
      else if (likely(type == BSPLINE_PATCH))
        return patch.eval(uu,vv);
      else if (likely(type == GREGORY_PATCH))
	return DenseGregoryPatch3fa::eval_t<simdb>( patch.v, uu, vv );
      else if (likely(type == GREGORY_TRIANGLE_PATCH))
        return GregoryTrianglePatch3fa::eval<simdb,simdf>( patch.v, uu * (1.0f - vv), vv );
      return Vec3<simdf>( zero );
    }

    template<typename simdf>
      __forceinline Vec3<simdf> normal(const simdf& uu, const simdf& vv) const
    {
      typedef typename simdf::Mask simdb;
      if (likely(type == BEZIER_PATCH))
        return BezierPatch3fa::normal( patch.v, uu, vv );
      else if (likely(type == BSPLINE_PATCH))
        return patch.normal(uu,vv);
      else if (likely(type == GREGORY_PATCH))
	return DenseGregoryPatch3fa::normal_t<simdb>( patch.v, uu, vv );
      else if (likely(type == GREGORY_TRIANGLE_PATCH))
	return GregoryTrianglePatch3fa::normal<simdb,simdf>( patch.v, uu, vv );
      return Vec3<simdf>( zero );
    }

#if defined(__MIC__)

    __forceinline Vec3f16 eval16(const float16& uu, const float16& vv) const
    {
      if (likely(type == BEZIER_PATCH))
        return BezierPatch3fa::eval( patch.v, uu, vv );
      else if (likely(type == BSPLINE_PATCH))
        return patch.eval(uu,vv);
      else 
        return DenseGregoryPatch3fa::eval16( patch.v, uu, vv );
    }

    __forceinline Vec3f16 normal16(const float16& uu, const float16& vv) const
    {
      if (likely(type == BEZIER_PATCH))
        return BezierPatch3fa::normal( patch.v, uu, vv );
      else if (likely(type == BSPLINE_PATCH))
	return patch.normal(uu,vv);
      else
        return DenseGregoryPatch3fa::normal16( patch.v, uu, vv );
    }
#endif
#endif

    __forceinline bool hasDisplacement() const {
      return (flags & HAS_DISPLACEMENT) == HAS_DISPLACEMENT;
    }

    __forceinline bool needsStitching() const {
      return (flags & TRANSITION_PATCH) == TRANSITION_PATCH;      
    }

    __forceinline void prefetchData() const
    {
      const char *const t = (char*)this;
      prefetchL1(t + 0*64);
      prefetchL1(t + 1*64);
      prefetchL1(t + 2*64);
      prefetchL1(t + 3*64);
      prefetchL1(t + 4*64);
    }

    __forceinline Vec2f getUV(const size_t i) const
    {
      return Vec2f((float)u[i],(float)v[i]) * (1.0f/65535.0f);
    }


#if defined(__MIC__)
    __forceinline void store(void *mem)
    {
      const float16 *const src = (float16*)this;
      assert(sizeof(SubdivPatch1Base) % 64 == 0);
      float16 *const dst = (float16*)mem;
#pragma unroll
      for (size_t i=0;i<sizeof(SubdivPatch1Base) / 64;i++)
	store16f_ngo(&dst[i],src[i]);
    }
#endif

    void updateEdgeLevels(const float edge_level[4],const SubdivMesh *const mesh);

    __forceinline size_t gridOffset(const size_t y, const size_t x) const
    {
      return grid_u_res*y+x;
    }

  private:

    size_t get64BytesBlocksForGridSubTree(const GridRange& range,
                                          const unsigned int leafBlocks)
    {
      if (range.hasLeafSize()) return leafBlocks;

      __aligned(64) GridRange r[4];

      const unsigned int children = range.splitIntoSubRanges(r);

      size_t blocks = 2; /* 128 bytes bvh4 node layout */

      for (unsigned int i=0;i<children;i++)
	blocks += get64BytesBlocksForGridSubTree(r[i],
						 leafBlocks);
      return blocks;    
    }




  public:
    __forceinline unsigned int getSubTreeSize64bBlocks(const unsigned int leafBlocks = 2)
    {
#if defined(__MIC__)
      const unsigned int U_BLOCK_SIZE = 5;
      const unsigned int V_BLOCK_SIZE = 3;

      const unsigned int grid_u_blocks = (grid_u_res + U_BLOCK_SIZE-2) / (U_BLOCK_SIZE-1);
      const unsigned int grid_v_blocks = (grid_v_res + V_BLOCK_SIZE-2) / (V_BLOCK_SIZE-1);

      return get64BytesBlocksForGridSubTree(GridRange(0,grid_u_blocks,0,grid_v_blocks),leafBlocks);
#else
      return get64BytesBlocksForGridSubTree(GridRange(0,grid_u_res-1,0,grid_v_res-1),leafBlocks);
#endif
    }

    __forceinline void read_lock()      { mtx.read_lock();    }
    __forceinline void read_unlock()    { mtx.read_unlock();  }
    __forceinline void write_lock()     { mtx.write_lock();   }
    __forceinline void write_unlock()   { mtx.write_unlock(); }
    
    __forceinline bool try_write_lock() { return mtx.try_write_lock(); }
    __forceinline bool try_read_lock()  { return mtx.try_read_lock(); }
    
    __forceinline void upgrade_read_to_write_lock() { mtx.upgrade_read_to_write_lock(); }
    __forceinline void upgrade_write_to_read_lock() { mtx.upgrade_write_to_read_lock(); }

    __forceinline void resetRootRef()
    {
      assert( mtx.hasInitialState() );
      root_ref = SharedLazyTessellationCache::Tag();
    }


    // 16bit discretized u,v coordinates

    unsigned short u[4]; 
    unsigned short v[4];
    float level[4];

    unsigned char flags;
    unsigned char type;
    unsigned short grid_bvh_size_64b_blocks;
    unsigned int geom;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int prim;                          //!< primitive ID of this subdivision patch
    unsigned short grid_u_res;
    unsigned short grid_v_res;

    unsigned short grid_size_simd_blocks;
    unsigned short grid_subtree_size_64b_blocks;

    RWMutex mtx;
    SharedLazyTessellationCache::Tag root_ref;

#if USE_RANGE_EVAL
    const SubdivMesh::HalfEdge* edge;
    size_t subPatch;
#else
    __aligned(64) BSplinePatch3fa patch;
#endif
  };

#if USE_RANGE_EVAL

  /* eval grid over patch and stich edges when required */      
  static __forceinline void evalGrid(const SubdivPatch1Base& patch,
                                     float *__restrict__ const grid_x,
                                     float *__restrict__ const grid_y,
                                     float *__restrict__ const grid_z,
                                     float *__restrict__ const grid_u,
                                     float *__restrict__ const grid_v,
                                     const SubdivMesh* const geom)
  {
    feature_adaptive_eval2 (patch.edge, patch.subPatch, geom->getVertexBuffer(0),
                            0,patch.grid_u_res-1,0,patch.grid_v_res-1,patch.grid_u_res,patch.grid_v_res,
                            grid_x,grid_y,grid_z,grid_u,grid_v,patch.grid_u_res,patch.grid_v_res);

#if defined(__MIC__)
    const size_t SIMD_WIDTH = 16;
#else
    const size_t SIMD_WIDTH = 8;
#endif
    /* set last elements in u,v array to 1.0f */
    const float last_u = grid_u[patch.grid_u_res*patch.grid_v_res-1];
    const float last_v = grid_v[patch.grid_u_res*patch.grid_v_res-1];
    const float last_x = grid_x[patch.grid_u_res*patch.grid_v_res-1];
    const float last_y = grid_y[patch.grid_u_res*patch.grid_v_res-1];
    const float last_z = grid_z[patch.grid_u_res*patch.grid_v_res-1];
    for (size_t i=patch.grid_u_res*patch.grid_v_res;i<patch.grid_size_simd_blocks*SIMD_WIDTH;i++)
      {
	grid_u[i] = last_u;
	grid_v[i] = last_v;
        grid_x[i] = last_x;
        grid_y[i] = last_y;
        grid_z[i] = last_z;
      }
  }

#else

  /* eval grid over patch and stich edges when required */      
  static __forceinline void evalGrid(const SubdivPatch1Base& patch,
                                     float *__restrict__ const grid_x,
                                     float *__restrict__ const grid_y,
                                     float *__restrict__ const grid_z,
                                     float *__restrict__ const grid_u,
                                     float *__restrict__ const grid_v,
                                     const SubdivMesh* const geom)
  {
    /* grid_u, grid_v need to be padded as we write with SIMD granularity */
    gridUVTessellator(patch.level,patch.grid_u_res,patch.grid_v_res,grid_u,grid_v);

#if defined(__MIC__)
    const size_t SIMD_WIDTH = 16;
#else
    const size_t SIMD_WIDTH = 8; // FIXME: why always 8???
#endif

    /* set last elements in u,v array to 1.0f */
    for (size_t i=patch.grid_u_res*patch.grid_v_res;i<patch.grid_size_simd_blocks*SIMD_WIDTH;i++)
      {
	grid_u[i] = 1.0f;
	grid_v[i] = 1.0f;
      }

    /* stitch edges if necessary */
    if (unlikely(patch.needsStitching()))
      stitchUVGrid(patch.level,patch.grid_u_res,patch.grid_v_res,grid_u,grid_v);
        
#if defined(__MIC__)

       for (size_t i = 0; i<patch.grid_size_simd_blocks; i++)
        {
          const float16 u = load16f(&grid_u[i * 16]);
          const float16 v = load16f(&grid_v[i * 16]);

	  //prefetch<PFHINT_L2EX>(&grid_x[16*i]);
	  //prefetch<PFHINT_L2EX>(&grid_y[16*i]);
	  //prefetch<PFHINT_L2EX>(&grid_z[16*i]);

          Vec3f16 vtx = patch.eval16(u, v);

          /* eval displacement function */
	  if (unlikely(((SubdivMesh*)geom)->displFunc != nullptr))
            {
              Vec3f16 normal = patch.normal16(u, v);
              normal = normalize(normal);

              const Vec2f uv0 = patch.getUV(0);
              const Vec2f uv1 = patch.getUV(1);
              const Vec2f uv2 = patch.getUV(2);
              const Vec2f uv3 = patch.getUV(3);

              const float16 patch_uu = lerp2(uv0.x, uv1.x, uv3.x, uv2.x, u, v);
              const float16 patch_vv = lerp2(uv0.y, uv1.y, uv3.y, uv2.y, u, v);

              ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
					     patch.geom,
					     patch.prim,
					     (const float*)&patch_uu,
					     (const float*)&patch_vv,
					     (const float*)&normal.x,
					     (const float*)&normal.y,
					     (const float*)&normal.z,
					     (float*)&vtx.x,
					     (float*)&vtx.y,
					     (float*)&vtx.z,
					     16);

            }
	  //prefetch<PFHINT_L1EX>(&grid_x[16*i]);
	  //prefetch<PFHINT_L1EX>(&grid_y[16*i]);
	  //prefetch<PFHINT_L1EX>(&grid_z[16*i]);

	  store16f_ngo(&grid_x[16*i],vtx.x);
	  store16f_ngo(&grid_y[16*i],vtx.y);
	  store16f_ngo(&grid_z[16*i],vtx.z);
        }
   
#else        
#if defined(__AVX__)
    for (size_t i=0;i<patch.grid_size_simd_blocks;i++)
      {
        float8 uu = load8f(&grid_u[8*i]);
        float8 vv = load8f(&grid_v[8*i]);
        Vec3f8 vtx = patch.eval(uu,vv);
                 
        if (unlikely(((SubdivMesh*)geom)->displFunc != nullptr))
          {
	    const Vec2f uv0 = patch.getUV(0);
	    const Vec2f uv1 = patch.getUV(1);
	    const Vec2f uv2 = patch.getUV(2);
	    const Vec2f uv3 = patch.getUV(3);

            Vec3f8 normal = patch.normal(uu,vv);
            normal = normalize_safe(normal) ;
            
            const float8 patch_uu = lerp2(uv0.x,uv1.x,uv3.x,uv2.x,uu,vv);
            const float8 patch_vv = lerp2(uv0.y,uv1.y,uv3.y,uv2.y,uu,vv);
            
            ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                           patch.geom,
                                           patch.prim,
                                           (const float*)&patch_uu,
                                           (const float*)&patch_vv,
                                           (const float*)&normal.x,
                                           (const float*)&normal.y,
                                           (const float*)&normal.z,
                                           (float*)&vtx.x,
                                           (float*)&vtx.y,
                                           (float*)&vtx.z,
                                           8);
          }
        *(float8*)&grid_x[8*i] = vtx.x;
        *(float8*)&grid_y[8*i] = vtx.y;
        *(float8*)&grid_z[8*i] = vtx.z;        
      }
#else
    for (size_t i=0;i<patch.grid_size_simd_blocks*2;i++) // 4-wide blocks for SSE
      {
        float4 uu = load4f(&grid_u[4*i]);
        float4 vv = load4f(&grid_v[4*i]);
        Vec3f4 vtx = patch.eval(uu,vv);
          
          
        if (unlikely(((SubdivMesh*)geom)->displFunc != nullptr))
          {
	    const Vec2f uv0 = patch.getUV(0);
	    const Vec2f uv1 = patch.getUV(1);
	    const Vec2f uv2 = patch.getUV(2);
	    const Vec2f uv3 = patch.getUV(3);

            Vec3f4 normal = patch.normal(uu,vv);
            normal = normalize_safe(normal);

            const float4 patch_uu = lerp2(uv0.x,uv1.x,uv3.x,uv2.x,uu,vv);
            const float4 patch_vv = lerp2(uv0.y,uv1.y,uv3.y,uv2.y,uu,vv);
            
            ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                           patch.geom,
                                           patch.prim,
                                           (const float*)&patch_uu,
                                           (const float*)&patch_vv,
                                           (const float*)&normal.x,
                                           (const float*)&normal.y,
                                           (const float*)&normal.z,
                                           (float*)&vtx.x,
                                           (float*)&vtx.y,
                                           (float*)&vtx.z,
                                           4);
          }
          
        *(float4*)&grid_x[4*i] = vtx.x;
        *(float4*)&grid_y[4*i] = vtx.y;
        *(float4*)&grid_z[4*i] = vtx.z;        
      }
#endif
#endif        
  }
#endif

}
