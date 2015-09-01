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
// WITHOUT WARRANTIES OR CONDI &TIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "grid_soa.h"
#include "../../common/ray.h"
#include "triangle_intersector_pluecker.h"

namespace embree
{
  namespace isa
  {
    template<typename RayN>
    class GridSOAIntersectorN
    {
    public:
      typedef SubdivPatch1Cached Primitive;
      typedef typename RayN::simdb rsimdb;
      typedef typename RayN::simdi rsimdi;
      typedef typename RayN::simdf rsimdf;
      typedef Vec3<rsimdf> rsimd3f;
      
      class Precalculations 
      { 
      public:
        __forceinline Precalculations (const rsimdb& valid, RayN& ray) 
          : grid(nullptr) {}
        
        __forceinline ~Precalculations() 
        {
	  if (grid)
            SharedLazyTessellationCache::sharedLazyTessellationCache.unlock();
        }
        
      public:
        GridSOA* grid;
      };     

      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(const rsimdb& valid_i, Precalculations& pre, RayN& ray, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        const size_t dim_offset    = pre.grid->dim_offset;
        const size_t line_offset   = pre.grid->width;
        const float* const grid_x  = pre.grid->gridData() + ((size_t) (prim) >> 4) - 1;
        const float* const grid_y  = grid_x + 1 * dim_offset;
        const float* const grid_z  = grid_x + 2 * dim_offset;
        const float* const grid_uv = grid_x + 3 * dim_offset;
        
        for (size_t y=0; y<2; y++) 
        {
          for (size_t x=0; x<2; x++) 
          {
            const size_t ofs00 = (y+0)*line_offset+(x+0);
            const size_t ofs01 = (y+0)*line_offset+(x+1);
            const size_t ofs10 = (y+1)*line_offset+(x+0);
            const size_t ofs11 = (y+1)*line_offset+(x+1);
            const rsimd3f p00(grid_x[ofs00],grid_y[ofs00],grid_z[ofs00]);
            const rsimd3f p01(grid_x[ofs01],grid_y[ofs01],grid_z[ofs01]);
            const rsimd3f p10(grid_x[ofs10],grid_y[ofs10],grid_z[ofs10]);
            const rsimd3f p11(grid_x[ofs11],grid_y[ofs11],grid_z[ofs11]);

            triangle_intersect_pluecker<true>(valid_i,ray,p00,p01,p10,pre.grid->geomID,pre.grid->primID,scene,[&](rsimdf& u, rsimdf& v) { // FIXME: use quad intersector
                const rsimdf uv00(grid_uv[ofs00]);
                const rsimdf uv01(grid_uv[ofs01]);
                const rsimdf uv10(grid_uv[ofs10]);
                const rsimdf uv11(grid_uv[ofs11]);
                const Vec2<rsimdf> uv0 = GridSOA::decodeUV(uv00);
                const Vec2<rsimdf> uv1 = GridSOA::decodeUV(uv01);
                const Vec2<rsimdf> uv2 = GridSOA::decodeUV(uv10);
                const Vec2<rsimdf> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
                u = uv[0];v = uv[1]; 
              });

            triangle_intersect_pluecker<true>(valid_i,ray,p10,p01,p11,pre.grid->geomID,pre.grid->primID,scene,[&](rsimdf& u, rsimdf& v) {
                const rsimdf uv00(grid_uv[ofs00]);
                const rsimdf uv01(grid_uv[ofs01]);
                const rsimdf uv10(grid_uv[ofs10]);
                const rsimdf uv11(grid_uv[ofs11]);
                const Vec2<rsimdf> uv0 = GridSOA::decodeUV(uv10);
                const Vec2<rsimdf> uv1 = GridSOA::decodeUV(uv01);
                const Vec2<rsimdf> uv2 = GridSOA::decodeUV(uv11);
                const Vec2<rsimdf> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
                u = uv[0];v = uv[1]; 
              });
          }
        }
      }

      /*! Test if the ray is occluded by the primitive */
      static __forceinline rsimdb occluded(const rsimdb& valid_i, Precalculations& pre, RayN& ray, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        const size_t dim_offset    = pre.grid->dim_offset;
        const size_t line_offset   = pre.grid->width;
        const float* const grid_x  = pre.grid->gridData() + ((size_t) (prim) >> 4) - 1;
        const float* const grid_y  = grid_x + 1 * dim_offset;
        const float* const grid_z  = grid_x + 2 * dim_offset;
        const float* const grid_uv = grid_x + 3 * dim_offset;
        
        rsimdb valid = valid_i;
        for (size_t y=0; y<2; y++) 
        {
          for (size_t x=0; x<2; x++) 
          {
            const size_t ofs00 = (y+0)*line_offset+(x+0);
            const size_t ofs01 = (y+0)*line_offset+(x+1);
            const size_t ofs10 = (y+1)*line_offset+(x+0);
            const size_t ofs11 = (y+1)*line_offset+(x+1);
            const rsimd3f p00(grid_x[ofs00],grid_y[ofs00],grid_z[ofs00]);
            const rsimd3f p01(grid_x[ofs01],grid_y[ofs01],grid_z[ofs01]);
            const rsimd3f p10(grid_x[ofs10],grid_y[ofs10],grid_z[ofs10]);
            const rsimd3f p11(grid_x[ofs11],grid_y[ofs11],grid_z[ofs11]);

            triangle_occluded_pluecker<true>(valid,ray,p00,p01,p10,pre.grid->geomID,pre.grid->primID,scene,[&](rsimdf& u, rsimdf& v) {
                const rsimdf uv00(grid_uv[ofs00]);
                const rsimdf uv01(grid_uv[ofs01]);
                const rsimdf uv10(grid_uv[ofs10]);
                const rsimdf uv11(grid_uv[ofs11]);
                const Vec2<rsimdf> uv0 = GridSOA::decodeUV(uv00);
                const Vec2<rsimdf> uv1 = GridSOA::decodeUV(uv01);
                const Vec2<rsimdf> uv2 = GridSOA::decodeUV(uv10);
                const Vec2<rsimdf> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
                u = uv[0];v = uv[1]; 
              });
            if (none(valid)) break;

            triangle_occluded_pluecker<true>(valid,ray,p10,p01,p11,pre.grid->geomID,pre.grid->primID,scene,[&](rsimdf& u, rsimdf& v) {
                const rsimdf uv00(grid_uv[ofs00]);
                const rsimdf uv01(grid_uv[ofs01]);
                const rsimdf uv10(grid_uv[ofs10]);
                const rsimdf uv11(grid_uv[ofs11]);
                const Vec2<rsimdf> uv0 = GridSOA::decodeUV(uv10);
                const Vec2<rsimdf> uv1 = GridSOA::decodeUV(uv01);
                const Vec2<rsimdf> uv2 = GridSOA::decodeUV(uv11);
                const Vec2<rsimdf> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
                u = uv[0];v = uv[1]; 
              });
            if (none(valid)) break;
          }
        }
        return !valid;
      }

         template<typename Loader>
        static __forceinline void intersect(RayN& ray, size_t k,
                                            const float* const grid_x,
                                            const float* const grid_y,
                                            const float* const grid_z,
                                            const float* const grid_uv,
                                            const size_t line_offset,
                                            Precalculations& pre,
                                            Scene* scene)
      {
        typedef typename Loader::vbool vbool;
        typedef typename Loader::vfloat vfloat;
	const Vec3<vfloat> tri_v012_x = Loader::gather(grid_x,line_offset);
	const Vec3<vfloat> tri_v012_y = Loader::gather(grid_y,line_offset);
	const Vec3<vfloat> tri_v012_z = Loader::gather(grid_z,line_offset);
        
	const Vec3<vfloat> v0(tri_v012_x[0],tri_v012_y[0],tri_v012_z[0]);
	const Vec3<vfloat> v1(tri_v012_x[1],tri_v012_y[1],tri_v012_z[1]);
	const Vec3<vfloat> v2(tri_v012_x[2],tri_v012_y[2],tri_v012_z[2]);
        
        triangle_intersect_pluecker<true>(ray,k,v0,v1,v2,pre.grid->geomID,pre.grid->primID,scene,[&](vfloat& u, vfloat& v) {
            const Vec3<vfloat> tri_v012_uv = Loader::gather(grid_uv,line_offset);	
            const Vec2<vfloat> uv0 = GridSOA::decodeUV(tri_v012_uv[0]);
            const Vec2<vfloat> uv1 = GridSOA::decodeUV(tri_v012_uv[1]);
            const Vec2<vfloat> uv2 = GridSOA::decodeUV(tri_v012_uv[2]);        
            const Vec2<vfloat> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
            u = uv[0];v = uv[1]; 
          });
      };
      
      template<typename Loader>
        static __forceinline bool occluded(RayN& ray, size_t k,
                                           const float* const grid_x,
                                           const float* const grid_y,
                                           const float* const grid_z,
                                           const float* const grid_uv,
                                           const size_t line_offset,
                                           Precalculations& pre,
                                           Scene* scene)
      {
        typedef typename Loader::vbool vbool;
        typedef typename Loader::vfloat vfloat;
	const Vec3<vfloat> tri_v012_x = Loader::gather(grid_x,line_offset);
	const Vec3<vfloat> tri_v012_y = Loader::gather(grid_y,line_offset);
	const Vec3<vfloat> tri_v012_z = Loader::gather(grid_z,line_offset);
        
	const Vec3<vfloat> v0(tri_v012_x[0],tri_v012_y[0],tri_v012_z[0]);
	const Vec3<vfloat> v1(tri_v012_x[1],tri_v012_y[1],tri_v012_z[1]);
	const Vec3<vfloat> v2(tri_v012_x[2],tri_v012_y[2],tri_v012_z[2]);
        
        return triangle_occluded_pluecker<true>(ray,k,v0,v1,v2,pre.grid->geomID,pre.grid->primID,scene,[&](vfloat& u, vfloat& v) {
            const Vec3<vfloat> tri_v012_uv = Loader::gather(grid_uv,line_offset);	
            const Vec2<vfloat> uv0 = GridSOA::decodeUV(tri_v012_uv[0]);
            const Vec2<vfloat> uv1 = GridSOA::decodeUV(tri_v012_uv[1]);
            const Vec2<vfloat> uv2 = GridSOA::decodeUV(tri_v012_uv[2]);        
            const Vec2<vfloat> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
            u = uv[0];v = uv[1]; 
          });
      }

      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, RayN& ray, size_t k, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        const size_t dim_offset    = pre.grid->dim_offset;
        const size_t line_offset   = pre.grid->width;
        const float* const grid_x  = pre.grid->gridData() + ((size_t) (prim) >> 4) - 1;
        const float* const grid_y  = grid_x + 1 * dim_offset;
        const float* const grid_z  = grid_x + 2 * dim_offset;
        const float* const grid_uv = grid_x + 3 * dim_offset;
        
#if defined(__AVX__)
        intersect<GridSOA::Gather3x3>( ray, k, grid_x,grid_y,grid_z,grid_uv, line_offset, pre, scene);
#else
        intersect<GridSOA::Gather2x3>(ray, k, grid_x            ,grid_y            ,grid_z            ,grid_uv            , line_offset, pre, scene);
        intersect<GridSOA::Gather2x3>(ray, k, grid_x+line_offset,grid_y+line_offset,grid_z+line_offset,grid_uv+line_offset, line_offset, pre, scene);
#endif
      }
      
      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(Precalculations& pre, RayN& ray, size_t k, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        const size_t dim_offset    = pre.grid->dim_offset;
        const size_t line_offset   = pre.grid->width;
        const float* const grid_x  = pre.grid->gridData() + ((size_t) (prim) >> 4) - 1;
        const float* const grid_y  = grid_x + 1 * dim_offset;
        const float* const grid_z  = grid_x + 2 * dim_offset;
        const float* const grid_uv = grid_x + 3 * dim_offset;
        
#if defined(__AVX__)
        return occluded<GridSOA::Gather3x3>( ray, k, grid_x,grid_y,grid_z,grid_uv, line_offset, pre, scene);
#else
        if (occluded<GridSOA::Gather2x3>(ray, k, grid_x            ,grid_y            ,grid_z            ,grid_uv            , line_offset, pre, scene)) return true;
        if (occluded<GridSOA::Gather2x3>(ray, k, grid_x+line_offset,grid_y+line_offset,grid_z+line_offset,grid_uv+line_offset, line_offset, pre, scene)) return true;
#endif
        return false;
      } 
    };
  }
}
