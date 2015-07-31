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

#include "../../common/ray.h"
#include "../../common/scene_subdiv_mesh.h"
#include "filter.h"
#include "../bvh4/bvh4.h"
#include "../../common/subdiv/tessellation.h"
#include "../../common/subdiv/tessellation_cache.h"
#include "subdivpatch1cached.h"
#include "grid_soa.h"

namespace embree
{
  namespace isa
  {
    class SubdivPatch1CachedIntersector1
    {
    public:
      typedef SubdivPatch1Cached Primitive;
      typedef GridSOA::Precalculations Precalculations;
      
      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        STAT3(normal.trav_prims,1,1,1);
        
        if (likely(ty == 2))
        {
          const size_t dim_offset    = pre.patch->grid_size_simd_blocks * vfloat::size;
          const size_t line_offset   = pre.patch->grid_u_res;
          const size_t offset_bytes  = ((size_t)prim  - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr()) >> 2;   
          const float *const grid_x  = (float*)(offset_bytes + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr());
          const float *const grid_y  = grid_x + 1 * dim_offset;
          const float *const grid_z  = grid_x + 2 * dim_offset;
          const float *const grid_uv = grid_x + 3 * dim_offset;
#if defined(__AVX__)
          GridSOA::intersect1_precise_3x3( ray, grid_x,grid_y,grid_z,grid_uv, line_offset, pre, scene);
#else
	  GridSOA::intersect1_precise_2x3( ray, grid_x            ,grid_y            ,grid_z            ,grid_uv            , line_offset, pre, scene);
	  GridSOA::intersect1_precise_2x3( ray, grid_x+line_offset,grid_y+line_offset,grid_z+line_offset,grid_uv+line_offset, line_offset, pre, scene);
#endif
        }
        else 
        {
	  lazy_node = GridSOA::lazyBuildPatch(pre,(SubdivPatch1Cached*)prim, scene);
	  assert(lazy_node);
          pre.patch = (SubdivPatch1Cached*)prim;
        }             
      }
      
      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        STAT3(shadow.trav_prims,1,1,1);
        
        if (likely(ty == 2))
        {
          const size_t dim_offset    = pre.patch->grid_size_simd_blocks * vfloat::size;
          const size_t line_offset   = pre.patch->grid_u_res;
          const size_t offset_bytes  = ((size_t)prim  - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr()) >> 2;   
          const float *const grid_x  = (float*)(offset_bytes + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr());
          const float *const grid_y  = grid_x + 1 * dim_offset;
          const float *const grid_z  = grid_x + 2 * dim_offset;
          const float *const grid_uv = grid_x + 3 * dim_offset;

#if defined(__AVX__)
	  return GridSOA::occluded1_precise_3x3( ray, grid_x,grid_y,grid_z,grid_uv, line_offset, pre, scene);
#else
	  if (GridSOA::occluded1_precise_2x3( ray, grid_x            ,grid_y            ,grid_z            ,grid_uv            , line_offset, pre, scene)) return true;
	  if (GridSOA::occluded1_precise_2x3( ray, grid_x+line_offset,grid_y+line_offset,grid_z+line_offset,grid_uv+line_offset, line_offset, pre, scene)) return true;
#endif
        }
        else 
        {
	  lazy_node = GridSOA::lazyBuildPatch(pre,(SubdivPatch1Cached*)prim, scene);
          pre.patch = (SubdivPatch1Cached*)prim;
        }             
        return false;
      }      
    };
  }
}
