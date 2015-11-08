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
      const size_t grid_size_xyzuv = (grid_size_simd_blocks * VSIZEX) * 4;
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
    Vec3fa patchEval(const SubdivPatch1Base& patch, const float uu, const float vv);
    Vec3fa patchNormal(const SubdivPatch1Base& patch, const float uu, const float vv);
    
    template<typename simdf>
      Vec3<simdf> patchEval(const SubdivPatch1Base& patch, const simdf& uu, const simdf& vv); 

    template<typename simdf>
      Vec3<simdf> patchNormal(const SubdivPatch1Base& patch, const simdf& uu, const simdf& vv); 
   

    /* eval grid over patch and stich edges when required */      
    void evalGrid(const SubdivPatch1Base& patch,
                  const size_t x0, const size_t x1,
                  const size_t y0, const size_t y1,
                  const size_t swidth, const size_t sheight,
                  float *__restrict__ const grid_x,
                  float *__restrict__ const grid_y,
                  float *__restrict__ const grid_z,
                  float *__restrict__ const grid_u,
                  float *__restrict__ const grid_v,
                  const SubdivMesh* const geom);

    /* eval grid over patch and stich edges when required */      
    BBox3fa evalGridBounds(const SubdivPatch1Base& patch,
                           const size_t x0, const size_t x1,
                           const size_t y0, const size_t y1,
                           const size_t swidth, const size_t sheight,
                           const SubdivMesh* const geom);
  }
}
