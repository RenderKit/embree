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

#include "grid_soa.h"
#include "../../common/ray.h"
#include "triangle_intersector_pluecker.h"

namespace embree
{
  namespace isa
  {
    template<int K>
    class GridSOAIntersectorK
    {
    public:
      typedef SubdivPatch1Cached Primitive;
      typedef Vec3<vfloat<K>> Vec3vfK;
      
      class Precalculations 
      { 
#if defined(__AVX__)
        static const int M = 8;
#else
        static const int M = 4;
#endif

      public:
        __forceinline Precalculations (const vbool<K>& valid, RayK<K>& ray)
          : grid(nullptr), intersector(valid,ray) {}
        
        __forceinline ~Precalculations() 
        {
	  if (grid)
            SharedLazyTessellationCache::sharedLazyTessellationCache.unlock();
        }
        
      public:
        GridSOA* grid;
        PlueckerIntersectorK<M,K> intersector; // FIXME: use quad intersector
      };     

      struct MapUV0
      {
        const float* const grid_uv;
        size_t ofs00, ofs01, ofs10, ofs11;

        __forceinline MapUV0(const float* const grid_uv, size_t ofs00, size_t ofs01, size_t ofs10, size_t ofs11)
          : grid_uv(grid_uv), ofs00(ofs00), ofs01(ofs01), ofs10(ofs10), ofs11(ofs11) {}

        __forceinline void operator() (vfloat<K>& u, vfloat<K>& v) const { 
          const vfloat<K> uv00(grid_uv[ofs00]);
          const vfloat<K> uv01(grid_uv[ofs01]);
          const vfloat<K> uv10(grid_uv[ofs10]);
          const vfloat<K> uv11(grid_uv[ofs11]);
          const Vec2<vfloat<K>> uv0 = GridSOA::decodeUV(uv00);
          const Vec2<vfloat<K>> uv1 = GridSOA::decodeUV(uv01);
          const Vec2<vfloat<K>> uv2 = GridSOA::decodeUV(uv10);
          const Vec2<vfloat<K>> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
          u = uv[0];v = uv[1]; 
        }
      };

      struct MapUV1
      {
        const float* const grid_uv;
        size_t ofs00, ofs01, ofs10, ofs11;

        __forceinline MapUV1(const float* const grid_uv, size_t ofs00, size_t ofs01, size_t ofs10, size_t ofs11)
          : grid_uv(grid_uv), ofs00(ofs00), ofs01(ofs01), ofs10(ofs10), ofs11(ofs11) {}

        __forceinline void operator() (vfloat<K>& u, vfloat<K>& v) const { 
          const vfloat<K> uv00(grid_uv[ofs00]);
          const vfloat<K> uv01(grid_uv[ofs01]);
          const vfloat<K> uv10(grid_uv[ofs10]);
          const vfloat<K> uv11(grid_uv[ofs11]);
          const Vec2<vfloat<K>> uv0 = GridSOA::decodeUV(uv10);
          const Vec2<vfloat<K>> uv1 = GridSOA::decodeUV(uv01);
          const Vec2<vfloat<K>> uv2 = GridSOA::decodeUV(uv11);
          const Vec2<vfloat<K>> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
          u = uv[0];v = uv[1]; 
        }
      };

       /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node)
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
            const Vec3vfK p00(grid_x[ofs00],grid_y[ofs00],grid_z[ofs00]);
            const Vec3vfK p01(grid_x[ofs01],grid_y[ofs01],grid_z[ofs01]);
            const Vec3vfK p10(grid_x[ofs10],grid_y[ofs10],grid_z[ofs10]);
            const Vec3vfK p11(grid_x[ofs11],grid_y[ofs11],grid_z[ofs11]);
            pre.intersector.intersectK(valid_i,ray,p00,p01,p10,MapUV0(grid_uv,ofs00,ofs01,ofs10,ofs11),IntersectKEpilogU<1,K,true>(ray,pre.grid->geomID,pre.grid->primID,scene));
            pre.intersector.intersectK(valid_i,ray,p10,p01,p11,MapUV1(grid_uv,ofs00,ofs01,ofs10,ofs11),IntersectKEpilogU<1,K,true>(ray,pre.grid->geomID,pre.grid->primID,scene));
          }
        }
      }

      /*! Test if the ray is occluded by the primitive */
      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node)
      {
        const size_t dim_offset    = pre.grid->dim_offset;
        const size_t line_offset   = pre.grid->width;
        const float* const grid_x  = pre.grid->gridData() + ((size_t) (prim) >> 4) - 1;
        const float* const grid_y  = grid_x + 1 * dim_offset;
        const float* const grid_z  = grid_x + 2 * dim_offset;
        const float* const grid_uv = grid_x + 3 * dim_offset;
        
        vbool<K> valid = valid_i;
        for (size_t y=0; y<2; y++) 
        {
          for (size_t x=0; x<2; x++) 
          {
            const size_t ofs00 = (y+0)*line_offset+(x+0);
            const size_t ofs01 = (y+0)*line_offset+(x+1);
            const size_t ofs10 = (y+1)*line_offset+(x+0);
            const size_t ofs11 = (y+1)*line_offset+(x+1);
            const Vec3vfK p00(grid_x[ofs00],grid_y[ofs00],grid_z[ofs00]);
            const Vec3vfK p01(grid_x[ofs01],grid_y[ofs01],grid_z[ofs01]);
            const Vec3vfK p10(grid_x[ofs10],grid_y[ofs10],grid_z[ofs10]);
            const Vec3vfK p11(grid_x[ofs11],grid_y[ofs11],grid_z[ofs11]);

            pre.intersector.intersectK(valid,ray,p00,p01,p10,MapUV0(grid_uv,ofs00,ofs01,ofs10,ofs11),OccludedKEpilogU<1,K,true>(valid,ray,pre.grid->geomID,pre.grid->primID,scene));
            if (none(valid)) break;
            pre.intersector.intersectK(valid,ray,p10,p01,p11,MapUV1(grid_uv,ofs00,ofs01,ofs10,ofs11),OccludedKEpilogU<1,K,true>(valid,ray,pre.grid->geomID,pre.grid->primID,scene));
            if (none(valid)) break;
          }
        }
        return !valid;
      }

      template<typename Loader>
      struct MapUV2
      {
        enum { M = Loader::M };
        const float* const grid_uv;
        size_t line_offset;

        __forceinline MapUV2(const float* const grid_uv, size_t line_offset)
          : grid_uv(grid_uv), line_offset(line_offset) {}

        __forceinline void operator() (vfloat<M>& u, vfloat<M>& v) const {
          const Vec3<vfloat<M>> tri_v012_uv = Loader::gather(grid_uv,line_offset);	
          const Vec2<vfloat<M>> uv0 = GridSOA::decodeUV(tri_v012_uv[0]);
          const Vec2<vfloat<M>> uv1 = GridSOA::decodeUV(tri_v012_uv[1]);
          const Vec2<vfloat<M>> uv2 = GridSOA::decodeUV(tri_v012_uv[2]);        
          const Vec2<vfloat<M>> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
          u = uv[0];v = uv[1]; 
        }
      };

      template<typename Loader>
        static __forceinline void intersect(RayK<K>& ray, size_t k,
                                            const float* const grid_x,
                                            const float* const grid_y,
                                            const float* const grid_z,
                                            const float* const grid_uv,
                                            const size_t line_offset,
                                            Precalculations& pre,
                                            Scene* scene)
      {
        enum { M = Loader::M };
        typedef typename Loader::vbool vbool;
        typedef typename Loader::vfloat vfloat;
	const Vec3<vfloat> tri_v012_x = Loader::gather(grid_x,line_offset);
	const Vec3<vfloat> tri_v012_y = Loader::gather(grid_y,line_offset);
	const Vec3<vfloat> tri_v012_z = Loader::gather(grid_z,line_offset);
	const Vec3<vfloat> v0(tri_v012_x[0],tri_v012_y[0],tri_v012_z[0]);
	const Vec3<vfloat> v1(tri_v012_x[1],tri_v012_y[1],tri_v012_z[1]);
	const Vec3<vfloat> v2(tri_v012_x[2],tri_v012_y[2],tri_v012_z[2]);
        pre.intersector.intersect(ray,k,v0,v1,v2,MapUV2<Loader>(grid_uv,line_offset),Intersect1KEpilogU<M,K,true>(ray,k,pre.grid->geomID,pre.grid->primID,scene));
      };
      
      template<typename Loader>
        static __forceinline bool occluded(RayK<K>& ray, size_t k,
                                           const float* const grid_x,
                                           const float* const grid_y,
                                           const float* const grid_z,
                                           const float* const grid_uv,
                                           const size_t line_offset,
                                           Precalculations& pre,
                                           Scene* scene)
      {
        enum { M = Loader::M };
        typedef typename Loader::vbool vbool;
        typedef typename Loader::vfloat vfloat;
	const Vec3<vfloat> tri_v012_x = Loader::gather(grid_x,line_offset);
	const Vec3<vfloat> tri_v012_y = Loader::gather(grid_y,line_offset);
	const Vec3<vfloat> tri_v012_z = Loader::gather(grid_z,line_offset);
	const Vec3<vfloat> v0(tri_v012_x[0],tri_v012_y[0],tri_v012_z[0]);
	const Vec3<vfloat> v1(tri_v012_x[1],tri_v012_y[1],tri_v012_z[1]);
	const Vec3<vfloat> v2(tri_v012_x[2],tri_v012_y[2],tri_v012_z[2]);
        return pre.intersector.intersect(ray,k,v0,v1,v2,MapUV2<Loader>(grid_uv,line_offset),Occluded1KEpilogU<M,K,true>(ray,k,pre.grid->geomID,pre.grid->primID,scene));
      }

      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node)
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
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node)
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
