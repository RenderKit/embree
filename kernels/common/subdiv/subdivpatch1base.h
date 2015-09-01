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

#include "bspline_patch.h"
#include "bezier_patch.h"
#include "gregory_patch.h"
#include "gregory_patch_dense.h"
#include "gregory_triangle_patch.h"
#include "tessellation.h"
#include "tessellation_cache.h"
#include "gridrange.h"
#include "patch_eval_grid.h"
#include "feature_adaptive_eval_grid.h"
#include "../scene_subdiv_mesh.h"

namespace embree
{
  struct __aligned(64) SubdivPatch1Base
  {
  public:

    enum Type {
      INVALID_PATCH          = 0,
      BSPLINE_PATCH          = 1,  
      BEZIER_PATCH           = 2,  
      GREGORY_PATCH          = 3,
      GREGORY_TRIANGLE_PATCH = 4,
      EVAL_PATCH             = 5,
    };

    enum Flags {
      TRANSITION_PATCH       = 16, 
    };

    /*! Default constructor. */
    __forceinline SubdivPatch1Base () {}

    SubdivPatch1Base (const unsigned int gID,
                      const unsigned int pID,
                      const unsigned int subPatch,
                      const SubdivMesh *const mesh,
                      const Vec2f uv[4],
                      const float edge_level[4],
                      const int subdiv[4],
                      const int simd_width);

    __forceinline Vec3fa eval(const float uu, const float vv) const
    {
      if (likely(type == BEZIER_PATCH))
        return ((BezierPatch3fa*)patch_v)->eval(uu,vv);
      else if (likely(type == BSPLINE_PATCH))
        return ((BSplinePatch3fa*)patch_v)->eval(uu,vv);
      else if (likely(type == GREGORY_PATCH))
	return ((DenseGregoryPatch3fa*)patch_v)->eval(uu,vv);
      else if (likely(type == GREGORY_TRIANGLE_PATCH))
        return ((GregoryTrianglePatch3fa*)patch_v)->eval(uu * (1.0f - vv), vv);
      return Vec3fa( zero );
    }

    __forceinline Vec3fa normal(const float uu, const float vv) const
    {
      if (likely(type == BEZIER_PATCH))
        return ((BezierPatch3fa*)patch_v)->normal(uu,vv);
      else if (likely(type == BSPLINE_PATCH))
        return ((BSplinePatch3fa*)patch_v)->normal(uu,vv);
      else if (likely(type == GREGORY_PATCH))
	return ((DenseGregoryPatch3fa*)patch_v)->normal(uu,vv);
      else if (likely(type == GREGORY_TRIANGLE_PATCH))
	return ((GregoryTrianglePatch3fa*)patch_v)->normal(uu * (1.0f - vv), vv);
      return Vec3fa( zero );
    }

    template<typename simdf>
      __forceinline Vec3<simdf> eval(const simdf& uu, const simdf& vv) const
    {
      typedef typename simdf::Mask simdb;
      if (likely(type == BEZIER_PATCH))
        return ((BezierPatch3fa*)patch_v)->eval(uu,vv);
      else if (likely(type == BSPLINE_PATCH))
        return ((BSplinePatch3fa*)patch_v)->eval(uu,vv);
      else if (likely(type == GREGORY_PATCH))
	return ((DenseGregoryPatch3fa*)patch_v)->eval(uu,vv);
      else if (likely(type == GREGORY_TRIANGLE_PATCH))
        return ((GregoryTrianglePatch3fa*)patch_v)->eval(uu * (1.0f - vv), vv);
      return Vec3<simdf>( zero );
    }

    template<typename simdf>
      __forceinline Vec3<simdf> normal(const simdf& uu, const simdf& vv) const
    {
      typedef typename simdf::Mask simdb;
      if (likely(type == BEZIER_PATCH))
        return ((BezierPatch3fa*)patch_v)->normal(uu,vv);
      else if (likely(type == BSPLINE_PATCH))
        return ((BSplinePatch3fa*)patch_v)->normal(uu,vv);
      else if (likely(type == GREGORY_PATCH))
	return ((DenseGregoryPatch3fa*)patch_v)->normal(uu,vv);
      else if (likely(type == GREGORY_TRIANGLE_PATCH))
         return ((GregoryTrianglePatch3fa*)patch_v)->normal(uu * (1.0f - vv), vv);
      return Vec3<simdf>( zero );
    }

    __forceinline bool needsStitching() const {
      return flags & TRANSITION_PATCH;      
    }

    __forceinline Vec2f getUV(const size_t i) const {
      return Vec2f((float)u[i],(float)v[i]) * (1.0f/65535.0f);
    }

    static void computeEdgeLevels(const float edge_level[4], const int subdiv[4], float level[4]);
    static Vec2i computeGridSize(const float level[4]);
    bool updateEdgeLevels(const float edge_level[4], const int subdiv[4], const SubdivMesh *const mesh, const int simd_width);

  private:
    size_t get64BytesBlocksForGridSubTree(const GridRange& range, const unsigned int leafBlocks);

  public:
    size_t getSubTreeSize64bBlocks(const unsigned int leafBlocks = 2);

    __forceinline size_t getGridBytes() const {
      const size_t grid_size_xyzuv = (grid_size_simd_blocks * vfloat::size) * 4;
      return 64*((grid_size_xyzuv+15) / 16);
    }

    __forceinline void write_lock()     { mtx.write_lock();   }
    __forceinline void write_unlock()   { mtx.write_unlock(); }
    __forceinline bool try_write_lock() { return mtx.try_write_lock(); }
    __forceinline bool try_read_lock()  { return mtx.try_read_lock(); }

    __forceinline void resetRootRef() {
      assert( mtx.hasInitialState() );
      root_ref = SharedLazyTessellationCache::Tag();
    }

    __forceinline SharedLazyTessellationCache::CacheEntry& entry() {
      return (SharedLazyTessellationCache::CacheEntry&) root_ref;
    }

  public:    
    SharedLazyTessellationCache::Tag root_ref;
    RWMutex mtx;

    unsigned short u[4];                        //!< 16bit discretized u,v coordinates
    unsigned short v[4];
    float level[4];

    unsigned char flags;
    unsigned char type;
    unsigned short grid_u_res;
    unsigned int geom;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int prim;                          //!< primitive ID of this subdivision patch
    unsigned short grid_v_res;

    unsigned short grid_size_simd_blocks;
#if defined (__MIC__)
    unsigned short grid_bvh_size_64b_blocks;
    unsigned short grid_subtree_size_64b_blocks;
#endif

    struct PatchHalfEdge {
      const HalfEdge* edge;
      size_t subPatch;
    };

      Vec3fa patch_v[4][4];

      const HalfEdge *edge() const {
        return ((PatchHalfEdge*)patch_v)->edge;
      }

      size_t subPatch() const {
        return ((PatchHalfEdge*)patch_v)->subPatch;
      }

      void set_edge(const HalfEdge *h) const {
        ((PatchHalfEdge*)patch_v)->edge = h;
      }

      void set_subPatch(const size_t s) const {
        ((PatchHalfEdge*)patch_v)->subPatch = s;
      }

  };

  namespace isa
  {
    /* eval grid over patch and stich edges when required */      
    static __noinline void evalGrid(const SubdivPatch1Base& patch,
                                    const size_t x0, const size_t x1,
                                    const size_t y0, const size_t y1,
                                    const size_t swidth, const size_t sheight,
                                    float *__restrict__ const grid_x,
                                    float *__restrict__ const grid_y,
                                    float *__restrict__ const grid_z,
                                    float *__restrict__ const grid_u,
                                    float *__restrict__ const grid_v,
                                    const SubdivMesh* const geom)
    {
      const size_t dwidth  = x1-x0+1;
      const size_t dheight = y1-y0+1;
      const size_t M = dwidth*dheight+vfloat::size;
      const size_t grid_size_simd_blocks = (M-1)/vfloat::size;

      if (unlikely(patch.type == SubdivPatch1Base::EVAL_PATCH))
      {
        const bool displ = geom->displFunc;
        const size_t N = displ ? M : 0;
        dynamic_large_stack_array(float,grid_Ng_x,N,64*64);
        dynamic_large_stack_array(float,grid_Ng_y,N,64*64);
        dynamic_large_stack_array(float,grid_Ng_z,N,64*64);
        
        if (geom->patch_eval_trees.size())
        {
          feature_adaptive_eval_grid<PatchEvalGrid> 
            (geom->patch_eval_trees[patch.prim], patch.subPatch(), patch.needsStitching() ? patch.level : nullptr,
             x0,x1,y0,y1,swidth,sheight,
             grid_x,grid_y,grid_z,grid_u,grid_v,
             displ ? (float*)grid_Ng_x : nullptr, displ ? (float*)grid_Ng_y : nullptr, displ ? (float*)grid_Ng_z : nullptr,
             dwidth,dheight);
        }
        else 
        {
          GeneralCatmullClarkPatch3fa ccpatch(patch.edge(),geom->getVertexBuffer(0));
          
          feature_adaptive_eval_grid<FeatureAdaptiveEvalGrid,GeneralCatmullClarkPatch3fa> 
            (ccpatch, patch.subPatch(), patch.needsStitching() ? patch.level : nullptr,
            x0,x1,y0,y1,swidth,sheight,
            grid_x,grid_y,grid_z,grid_u,grid_v,
            displ ? (float*)grid_Ng_x : nullptr, displ ? (float*)grid_Ng_y : nullptr, displ ? (float*)grid_Ng_z : nullptr,
            dwidth,dheight);
        }

        /* convert sub-patch UVs to patch UVs*/
        const Vec2f uv0 = patch.getUV(0);
        const Vec2f uv1 = patch.getUV(1);
        const Vec2f uv2 = patch.getUV(2);
        const Vec2f uv3 = patch.getUV(3);
        for (size_t i=0; i<grid_size_simd_blocks; i++)
        {
          const vfloat u = vfloat::load(&grid_u[i*vfloat::size]);
          const vfloat v = vfloat::load(&grid_v[i*vfloat::size]);
          const vfloat patch_u = lerp2(uv0.x,uv1.x,uv3.x,uv2.x,u,v);
          const vfloat patch_v = lerp2(uv0.y,uv1.y,uv3.y,uv2.y,u,v);
          vfloat::store(&grid_u[i*vfloat::size],patch_u);
          vfloat::store(&grid_v[i*vfloat::size],patch_v);
        }

        /* call displacement shader */
        if (geom->displFunc) 
          geom->displFunc(geom->userPtr,patch.geom,patch.prim,grid_u,grid_v,grid_Ng_x,grid_Ng_y,grid_Ng_z,grid_x,grid_y,grid_z,dwidth*dheight);

        /* set last elements in u,v array to 1.0f */
        const float last_u = grid_u[dwidth*dheight-1];
        const float last_v = grid_v[dwidth*dheight-1];
        const float last_x = grid_x[dwidth*dheight-1];
        const float last_y = grid_y[dwidth*dheight-1];
        const float last_z = grid_z[dwidth*dheight-1];
        for (size_t i=dwidth*dheight;i<grid_size_simd_blocks*vfloat::size;i++)
        {
          grid_u[i] = last_u;
          grid_v[i] = last_v;
          grid_x[i] = last_x;
          grid_y[i] = last_y;
          grid_z[i] = last_z;
        }
      }
      else
      {
        /* grid_u, grid_v need to be padded as we write with SIMD granularity */
        gridUVTessellator(patch.level,swidth,sheight,x0,y0,dwidth,dheight,grid_u,grid_v);
      
        /* set last elements in u,v array to last valid point */
        const float last_u = grid_u[dwidth*dheight-1];
        const float last_v = grid_v[dwidth*dheight-1];
        for (size_t i=dwidth*dheight;i<grid_size_simd_blocks*vfloat::size;i++) {
          grid_u[i] = last_u;
          grid_v[i] = last_v;
        }

        /* stitch edges if necessary */
        if (unlikely(patch.needsStitching()))
          stitchUVGrid(patch.level,swidth,sheight,x0,y0,dwidth,dheight,grid_u,grid_v);
      
        /* iterates over all grid points */
        for (size_t i=0; i<grid_size_simd_blocks; i++)
        {
          const vfloat u = vfloat::load(&grid_u[i*vfloat::size]);
          const vfloat v = vfloat::load(&grid_v[i*vfloat::size]);
          Vec3<vfloat> vtx = patch.eval(u,v);
        
          /* evaluate displacement function */
          if (unlikely(geom->displFunc != nullptr))
          {
            const Vec3<vfloat> normal = normalize_safe(patch.normal(u, v)); 
            geom->displFunc(geom->userPtr,patch.geom,patch.prim,
                            &u[0],&v[0],&normal.x[0],&normal.y[0],&normal.z[0],
                            &vtx.x[0],&vtx.y[0],&vtx.z[0],vfloat::size);
          
          }
          vfloat::store(&grid_x[i*vfloat::size],vtx.x);
          vfloat::store(&grid_y[i*vfloat::size],vtx.y);
          vfloat::store(&grid_z[i*vfloat::size],vtx.z);
        }
      }
    }


    /* eval grid over patch and stich edges when required */      
    static __noinline BBox3fa evalGridBounds(const SubdivPatch1Base& patch,
                                                const size_t x0, const size_t x1,
                                                const size_t y0, const size_t y1,
                                                const size_t swidth, const size_t sheight,
                                                const SubdivMesh* const geom)
    {
      BBox3fa b(empty);
      const size_t dwidth  = x1-x0+1;
      const size_t dheight = y1-y0+1;
      const size_t M = dwidth*dheight+vfloat::size;
      const size_t grid_size_simd_blocks = (M-1)/vfloat::size;
      dynamic_large_stack_array(float,grid_u,M,64*64);
      dynamic_large_stack_array(float,grid_v,M,64*64);

      if (unlikely(patch.type == SubdivPatch1Base::EVAL_PATCH))
      {
        const bool displ = geom->displFunc;
        dynamic_large_stack_array(float,grid_x,M,64*64);
        dynamic_large_stack_array(float,grid_y,M,64*64);
        dynamic_large_stack_array(float,grid_z,M,64*64);
        dynamic_large_stack_array(float,grid_Ng_x,displ ? M : 0,64*64);
        dynamic_large_stack_array(float,grid_Ng_y,displ ? M : 0,64*64);
        dynamic_large_stack_array(float,grid_Ng_z,displ ? M : 0,64*64);

        if (geom->patch_eval_trees.size())
        {
          feature_adaptive_eval_grid<PatchEvalGrid> 
            (geom->patch_eval_trees[patch.prim], patch.subPatch(), patch.needsStitching() ? patch.level : nullptr,
             x0,x1,y0,y1,swidth,sheight,
             grid_x,grid_y,grid_z,grid_u,grid_v,
             displ ? (float*)grid_Ng_x : nullptr, displ ? (float*)grid_Ng_y : nullptr, displ ? (float*)grid_Ng_z : nullptr,
             dwidth,dheight);
        } 
        else 
        {
          GeneralCatmullClarkPatch3fa ccpatch(patch.edge(),geom->getVertexBuffer(0));
          
          feature_adaptive_eval_grid <FeatureAdaptiveEvalGrid,GeneralCatmullClarkPatch3fa>
            (ccpatch, patch.subPatch(), patch.needsStitching() ? patch.level : nullptr,
            x0,x1,y0,y1,swidth,sheight,
            grid_x,grid_y,grid_z,grid_u,grid_v,
            displ ? (float*)grid_Ng_x : nullptr, displ ? (float*)grid_Ng_y : nullptr, displ ? (float*)grid_Ng_z : nullptr,
            dwidth,dheight);
        }

        /* call displacement shader */
        if (geom->displFunc) 
          geom->displFunc(geom->userPtr,patch.geom,patch.prim,grid_u,grid_v,grid_Ng_x,grid_Ng_y,grid_Ng_z,grid_x,grid_y,grid_z,dwidth*dheight);

        /* set last elements in u,v array to 1.0f */
        const float last_u = grid_u[dwidth*dheight-1];
        const float last_v = grid_v[dwidth*dheight-1];
        const float last_x = grid_x[dwidth*dheight-1];
        const float last_y = grid_y[dwidth*dheight-1];
        const float last_z = grid_z[dwidth*dheight-1];
        for (size_t i=dwidth*dheight;i<grid_size_simd_blocks*vfloat::size;i++)
        {
          grid_u[i] = last_u;
          grid_v[i] = last_v;
          grid_x[i] = last_x;
          grid_y[i] = last_y;
          grid_z[i] = last_z;
        }

        vfloat bounds_min_x = pos_inf;
        vfloat bounds_min_y = pos_inf;
        vfloat bounds_min_z = pos_inf;
        vfloat bounds_max_x = neg_inf;
        vfloat bounds_max_y = neg_inf;
        vfloat bounds_max_z = neg_inf;
        for (size_t i = 0; i<grid_size_simd_blocks; i++)
        {
          vfloat x = vfloat::loadu(&grid_x[i * vfloat::size]);
          vfloat y = vfloat::loadu(&grid_y[i * vfloat::size]);
          vfloat z = vfloat::loadu(&grid_z[i * vfloat::size]);

	  bounds_min_x = min(bounds_min_x,x);
	  bounds_min_y = min(bounds_min_y,y);
	  bounds_min_z = min(bounds_min_z,z);

	  bounds_max_x = max(bounds_max_x,x);
	  bounds_max_y = max(bounds_max_y,y);
	  bounds_max_z = max(bounds_max_z,z);
        }

        b.lower.x = reduce_min(bounds_min_x);
        b.lower.y = reduce_min(bounds_min_y);
        b.lower.z = reduce_min(bounds_min_z);
        b.upper.x = reduce_max(bounds_max_x);
        b.upper.y = reduce_max(bounds_max_y);
        b.upper.z = reduce_max(bounds_max_z);
        b.lower.a = 0.0f;
        b.upper.a = 0.0f;
      }
      else
      {
        /* grid_u, grid_v need to be padded as we write with SIMD granularity */
        gridUVTessellator(patch.level,swidth,sheight,x0,y0,dwidth,dheight,grid_u,grid_v);
      
        /* set last elements in u,v array to last valid point */
        const float last_u = grid_u[dwidth*dheight-1];
        const float last_v = grid_v[dwidth*dheight-1];
        for (size_t i=dwidth*dheight;i<grid_size_simd_blocks*vfloat::size;i++) {
          grid_u[i] = last_u;
          grid_v[i] = last_v;
        }

        /* stitch edges if necessary */
        if (unlikely(patch.needsStitching()))
          stitchUVGrid(patch.level,swidth,sheight,x0,y0,dwidth,dheight,grid_u,grid_v);
      
        /* iterates over all grid points */
        Vec3<vfloat> bounds_min;
        bounds_min[0] = pos_inf;
        bounds_min[1] = pos_inf;
        bounds_min[2] = pos_inf;

        Vec3<vfloat> bounds_max;
        bounds_max[0] = neg_inf;
        bounds_max[1] = neg_inf;
        bounds_max[2] = neg_inf;

        for (size_t i=0; i<grid_size_simd_blocks; i++)
        {
          const vfloat u = vfloat::load(&grid_u[i*vfloat::size]);
          const vfloat v = vfloat::load(&grid_v[i*vfloat::size]);
          Vec3<vfloat> vtx = patch.eval(u,v);
        
          /* evaluate displacement function */
          if (unlikely(geom->displFunc != nullptr))
          {
            const Vec3<vfloat> normal = normalize_safe(patch.normal(u,v));
            geom->displFunc(geom->userPtr,patch.geom,patch.prim,
                            &u[0],&v[0],&normal.x[0],&normal.y[0],&normal.z[0],
                            &vtx.x[0],&vtx.y[0],&vtx.z[0],vfloat::size);
          
          }
          bounds_min[0] = min(bounds_min[0],vtx.x);
          bounds_max[0] = max(bounds_max[0],vtx.x);
          bounds_min[1] = min(bounds_min[1],vtx.y);
          bounds_max[1] = max(bounds_max[1],vtx.y);
          bounds_min[2] = min(bounds_min[2],vtx.z);
          bounds_max[2] = max(bounds_max[2],vtx.z);      
        }

        b.lower.x = reduce_min(bounds_min[0]);
        b.lower.y = reduce_min(bounds_min[1]);
        b.lower.z = reduce_min(bounds_min[2]);
        b.upper.x = reduce_max(bounds_max[0]);
        b.upper.y = reduce_max(bounds_max[1]);
        b.upper.z = reduce_max(bounds_max[2]);
        b.lower.a = 0.0f;
        b.upper.a = 0.0f;
      }

      assert( std::isfinite(b.lower.x) );
      assert( std::isfinite(b.lower.y) );
      assert( std::isfinite(b.lower.z) );

      assert( std::isfinite(b.upper.x) );
      assert( std::isfinite(b.upper.y) );
      assert( std::isfinite(b.upper.z) );


      assert(b.lower.x <= b.upper.x);
      assert(b.lower.y <= b.upper.y);
      assert(b.lower.z <= b.upper.z);

      return b;
    }
  }
}
