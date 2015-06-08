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

/* returns u,v based on individual triangles instead relative to original patch */
#define FORCE_TRIANGLE_UV 0

#define ENABLE_NORMALIZED_INTERSECTION 0

#if FORCE_TRIANGLE_UV
#  pragma message("WARNING: FORCE_TRIANGLE_UV is enabled")
#endif

namespace embree
{
  namespace isa
  {
    class SubdivPatch1CachedIntersector1
    {
    public:
      typedef SubdivPatch1Cached Primitive;
      
      /*! Precalculations for subdiv patch intersection */
      class Precalculations {
      public:
        Vec3fa ray_rdir;
        Vec3fa ray_org_rdir;
#if ENABLE_NORMALIZED_INTERSECTION == 1
	Vec3fa ray_dir_scale;
#endif
        SubdivPatch1Cached* current_patch;
        SubdivPatch1Cached* hit_patch;
	unsigned int threadID;
        Ray& r;
#if _DEBUG
	size_t numPrimitives;
	SubdivPatch1Cached* array;
#endif
        
        __forceinline Precalculations (Ray& ray, const void *ptr) : r(ray) 
        {
          ray_rdir      = rcp_safe(ray.dir);
          ray_org_rdir  = ray.org*ray_rdir;
#if ENABLE_NORMALIZED_INTERSECTION == 1
	  ray_dir_scale = Vec3fa(ray.dir.y*ray.dir.z,ray.dir.z*ray.dir.x,ray.dir.x*ray.dir.y);
#endif

          current_patch = nullptr;
          hit_patch     = nullptr;
          threadID = SharedLazyTessellationCache::threadIndex();

#if _DEBUG
	  numPrimitives = ((BVH4*)ptr)->numPrimitives;
	  array         = (SubdivPatch1Cached*)(((BVH4*)ptr)->data_mem);
#endif
        }

          /*! Final per ray computations like smooth normal, patch u,v, etc. */        
        __forceinline ~Precalculations() 
        {
	  if (current_patch)
            SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadID);
          
          if (unlikely(hit_patch != nullptr))
          {

#if defined(RTCORE_RETURN_SUBDIV_NORMAL)
	    if (likely(!hit_patch->hasDisplacement()))
	      {		 
		Vec3fa normal = hit_patch->normal(r.v,r.u);
		r.Ng = normal;
	      }
#endif

#if FORCE_TRIANGLE_UV == 0
	    const Vec2f uv0 = hit_patch->getUV(0);
	    const Vec2f uv1 = hit_patch->getUV(1);
	    const Vec2f uv2 = hit_patch->getUV(2);
	    const Vec2f uv3 = hit_patch->getUV(3);
	    
	    const float patch_u = bilinear_interpolate(uv0.x,uv1.x,uv2.x,uv3.x,r.v,r.u);
	    const float patch_v = bilinear_interpolate(uv0.y,uv1.y,uv2.y,uv3.y,r.v,r.u);

	    r.u      = patch_u;
	    r.v      = patch_v;
#endif
            r.geomID = hit_patch->geom;
            r.primID = hit_patch->prim;
          }
        }
        
      };


      static __forceinline const Vec3<float4> getV012(const float *const grid,
						    const size_t offset0,
						    const size_t offset1)
      {
	const float4 row_a0 = loadu4f(grid + offset0); 
	const float4 row_b0 = loadu4f(grid + offset1);
	const float4 row_a1 = shuffle<1,2,3,3>(row_a0);
	const float4 row_b1 = shuffle<1,2,3,3>(row_b0);

	Vec3<float4> v;
	v[0] = unpacklo( row_a0 , row_b0 );
	v[1] = unpacklo( row_b0 , row_a1 );
	v[2] = unpacklo( row_a1 , row_b1 );
	return v;
      }

      static __forceinline Vec2<float4> decodeUV(const float4 &uv)
      {
	const int4 i_uv = cast(uv);
	const int4 i_u  = i_uv & 0xffff;
	const int4 i_v  = i_uv >> 16;
	const float4 u    = (float4)i_u * float4(2.0f/65535.0f);
	const float4 v    = (float4)i_v * float4(2.0f/65535.0f);
	return Vec2<float4>(u,v);
      }

      static __forceinline void intersect1_precise_2x3(Ray& ray,
						       const float *const grid_x,
						       const float *const grid_y,
						       const float *const grid_z,
						       const float *const grid_uv,
						       const size_t line_offset,
						       const void* geom,
						       Precalculations &pre)
      {
	const size_t offset0 = 0 * line_offset;
	const size_t offset1 = 1 * line_offset;

	const Vec3<float4> tri012_x = getV012(grid_x,offset0,offset1);
	const Vec3<float4> tri012_y = getV012(grid_y,offset0,offset1);
	const Vec3<float4> tri012_z = getV012(grid_z,offset0,offset1);

	const Vec3<float4> v0_org(tri012_x[0],tri012_y[0],tri012_z[0]);
	const Vec3<float4> v1_org(tri012_x[1],tri012_y[1],tri012_z[1]);
	const Vec3<float4> v2_org(tri012_x[2],tri012_y[2],tri012_z[2]);
        
	const Vec3<float4> O = ray.org;
	const Vec3<float4> D = ray.dir;
        
	const Vec3<float4> v0 = v0_org - O;
	const Vec3<float4> v1 = v1_org - O;
	const Vec3<float4> v2 = v2_org - O;
        
	const Vec3<float4> e0 = v2 - v0;
	const Vec3<float4> e1 = v0 - v1;	     
	const Vec3<float4> e2 = v1 - v2;	     
        
	/* calculate geometry normal and denominator */
	const Vec3<float4> Ng1 = cross(e1,e0);
	const Vec3<float4> Ng = Ng1+Ng1;
	const float4 den = dot(Ng,D);
	const float4 absDen = abs(den);
	const float4 sgnDen = signmsk(den);
        
	bool4 valid ( true );
	/* perform edge tests */
	const float4 U = dot(Vec3<float4>(cross(v2+v0,e0)),D) ^ sgnDen;
	valid &= U >= 0.0f;
	if (likely(none(valid))) return;
	const float4 V = dot(Vec3<float4>(cross(v0+v1,e1)),D) ^ sgnDen;
	valid &= V >= 0.0f;
	if (likely(none(valid))) return;
	const float4 W = dot(Vec3<float4>(cross(v1+v2,e2)),D) ^ sgnDen;

	valid &= W >= 0.0f;
	if (likely(none(valid))) return;
        
	/* perform depth test */
	const float4 _t = dot(v0,Ng) ^ sgnDen;
	valid &= (_t >= absDen*ray.tnear) & (absDen*ray.tfar >= _t);
	if (unlikely(none(valid))) return;
        
	/* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
	valid &= den > float4(zero);
	if (unlikely(none(valid))) return;
#else
	valid &= den != float4(zero);
	if (unlikely(none(valid))) return;
#endif
        
	/* calculate hit information */
	const float4 rcpAbsDen = rcp(absDen);
	const float4 u =  U*rcpAbsDen;
	const float4 v =  V*rcpAbsDen;
	const float4 t = _t*rcpAbsDen;
        
#if FORCE_TRIANGLE_UV == 0
	const Vec3<float4> tri012_uv = getV012(grid_uv,offset0,offset1);	
	const Vec2<float4> uv0 = decodeUV(tri012_uv[0]);
	const Vec2<float4> uv1 = decodeUV(tri012_uv[1]);
	const Vec2<float4> uv2 = decodeUV(tri012_uv[2]);        
	const Vec2<float4> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;        
	const float4 u_final = uv[1];
	const float4 v_final = uv[0];        
#else
	const float4 u_final = u;
	const float4 v_final = v;
#endif
	size_t i = select_min(valid,t);

	/* update hit information */
	pre.hit_patch = pre.current_patch;

	ray.u         = u_final[i];
	ray.v         = v_final[i];
	ray.tfar      = t[i];
	if (i % 2)
	  {
	    ray.Ng.x      = Ng.x[i];
	    ray.Ng.y      = Ng.y[i];
	    ray.Ng.z      = Ng.z[i];
	  }
	else
	  {
	    ray.Ng.x      = -Ng.x[i];
	    ray.Ng.y      = -Ng.y[i];
	    ray.Ng.z      = -Ng.z[i];	    
	  }
      };

      static __forceinline bool occluded1_precise_2x3(Ray& ray,
						      const float *const grid_x,
						      const float *const grid_y,
						      const float *const grid_z,
						      const float *const grid_uv,
						      const size_t line_offset,
						      const void* geom)
      {
	const size_t offset0 = 0 * line_offset;
	const size_t offset1 = 1 * line_offset;

	const Vec3<float4> tri012_x = getV012(grid_x,offset0,offset1);
	const Vec3<float4> tri012_y = getV012(grid_y,offset0,offset1);
	const Vec3<float4> tri012_z = getV012(grid_z,offset0,offset1);

	const Vec3<float4> v0_org(tri012_x[0],tri012_y[0],tri012_z[0]);
	const Vec3<float4> v1_org(tri012_x[1],tri012_y[1],tri012_z[1]);
	const Vec3<float4> v2_org(tri012_x[2],tri012_y[2],tri012_z[2]);
        
        const Vec3<float4> O = ray.org;
        const Vec3<float4> D = ray.dir;
        
        const Vec3<float4> v0 = v0_org - O;
        const Vec3<float4> v1 = v1_org - O;
        const Vec3<float4> v2 = v2_org - O;
        
        const Vec3<float4> e0 = v2 - v0;
        const Vec3<float4> e1 = v0 - v1;	     
        const Vec3<float4> e2 = v1 - v2;	     
        
        /* calculate geometry normal and denominator */
        const Vec3<float4> Ng1 = cross(e1,e0);
        const Vec3<float4> Ng = Ng1+Ng1;
        const float4 den = dot(Ng,D);
        const float4 absDen = abs(den);
        const float4 sgnDen = signmsk(den);
        
        bool4 valid ( true );
        /* perform edge tests */
        const float4 U = dot(Vec3<float4>(cross(v2+v0,e0)),D) ^ sgnDen;
        valid &= U >= 0.0f;
        if (likely(none(valid))) return false;
        const float4 V = dot(Vec3<float4>(cross(v0+v1,e1)),D) ^ sgnDen;
        valid &= V >= 0.0f;
        if (likely(none(valid))) return false;
        const float4 W = dot(Vec3<float4>(cross(v1+v2,e2)),D) ^ sgnDen;
        valid &= W >= 0.0f;
        if (likely(none(valid))) return false;
        
        /* perform depth test */
        const float4 _t = dot(v0,Ng) ^ sgnDen;
        valid &= (_t >= absDen*ray.tnear) & (absDen*ray.tfar >= _t);
        if (unlikely(none(valid))) return false;
        
        /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
        valid &= den > float4(zero);
        if (unlikely(none(valid))) return false;
#else
        valid &= den != float4(zero);
        if (unlikely(none(valid))) return false;
#endif
        return true;
      };




#if defined(__AVX__)

      static __forceinline const Vec3<float8> getV012(const float *const grid,
						    const size_t offset0,
						    const size_t offset1,
						    const size_t offset2)
      {
	const float4 row_a0 = loadu4f(grid + offset0 + 0); 
	const float4 row_b0 = loadu4f(grid + offset1 + 0);
	const float4 row_c0 = loadu4f(grid + offset2 + 0);
	const float8 row_ab = float8( row_a0, row_b0 );
	const float8 row_bc = float8( row_b0, row_c0 );

	const float8 row_ab_shuffle = shuffle<1,2,3,3>(row_ab);
	const float8 row_bc_shuffle = shuffle<1,2,3,3>(row_bc);

	Vec3<float8> v;
	v[0] = unpacklo(         row_ab , row_bc );
	v[1] = unpacklo(         row_bc , row_ab_shuffle );
	v[2] = unpacklo( row_ab_shuffle , row_bc_shuffle );
	return v;
      }

      static __forceinline Vec2<float8> decodeUV(const float8 &uv)
      {
	const int8 i_uv = cast(uv);
	const int8 i_u  = i_uv & 0xffff;
	const int8 i_v  = i_uv >> 16;
	const float8 u    = (float8)i_u * float8(2.0f/65535.0f);
	const float8 v    = (float8)i_v * float8(2.0f/65535.0f);
	return Vec2<float8>(u,v);
      }
     
 
      static __forceinline void intersect1_precise_3x3(Ray& ray,
						       const float *const grid_x,
						       const float *const grid_y,
						       const float *const grid_z,
						       const float *const grid_uv,
						       const size_t line_offset,
						       const void* geom,
						       Precalculations &pre)
      {
	const size_t offset0 = 0 * line_offset;
	const size_t offset1 = 1 * line_offset;
	const size_t offset2 = 2 * line_offset;

	const Vec3<float8> tri012_x = getV012(grid_x,offset0,offset1,offset2);
	const Vec3<float8> tri012_y = getV012(grid_y,offset0,offset1,offset2);
	const Vec3<float8> tri012_z = getV012(grid_z,offset0,offset1,offset2);

	const Vec3<float8> v0_org(tri012_x[0],tri012_y[0],tri012_z[0]);
	const Vec3<float8> v1_org(tri012_x[1],tri012_y[1],tri012_z[1]);
	const Vec3<float8> v2_org(tri012_x[2],tri012_y[2],tri012_z[2]);
        

#if ENABLE_NORMALIZED_INTERSECTION == 0
	const Vec3<float8> O = ray.org;
	const Vec3<float8> D = ray.dir;

	const Vec3<float8> v0 = v0_org - O;
	const Vec3<float8> v1 = v1_org - O;
	const Vec3<float8> v2 = v2_org - O;

	const Vec3<float8> e0 = v2 - v0;
	const Vec3<float8> e1 = v0 - v1;	     
	const Vec3<float8> e2 = v1 - v2;	     

	/* calculate geometry normal and denominator */
	const Vec3<float8> Ng1 = cross(e1,e0);
	const Vec3<float8> Ng = Ng1+Ng1;
	const float8 den = dot(Ng,D);
	const float8 absDen = abs(den);
	const float8 sgnDen = signmsk(den);
        
	bool8 valid ( true );
	/* perform edge tests */
	const float8 U = dot(Vec3<float8>(cross(v2+v0,e0)),D) ^ sgnDen;
	valid &= U >= 0.0f;
	if (likely(none(valid))) return;
	const float8 V = dot(Vec3<float8>(cross(v0+v1,e1)),D) ^ sgnDen;
	valid &= V >= 0.0f;
	if (likely(none(valid))) return;
	const float8 W = dot(Vec3<float8>(cross(v1+v2,e2)),D) ^ sgnDen;

#else
        const Vec3<float8> ray_rdir(pre.ray_rdir.x,pre.ray_rdir.y,pre.ray_rdir.z);
        const Vec3<float8> ray_org_rdir(pre.ray_org_rdir.x,pre.ray_org_rdir.y,pre.ray_org_rdir.z);

	const Vec3<float8> v0 = v0_org * ray_rdir - ray_org_rdir;
	const Vec3<float8> v1 = v1_org * ray_rdir - ray_org_rdir;
	const Vec3<float8> v2 = v2_org * ray_rdir - ray_org_rdir;

	const Vec3<float8> e0 = v2 - v0;
	const Vec3<float8> e1 = v0 - v1;	     
	const Vec3<float8> e2 = v1 - v2;	     

	/* calculate geometry normal and denominator */
	const Vec3<float8> Ng1 = cross(e1,e0);
	Vec3<float8> Ng = Ng1+Ng1;
	const float8 den = sum(Ng);
	const float8 absDen = abs(den);
	const float8 sgnDen = signmsk(den);
        
	bool8 valid ( true );
	/* perform edge tests */
	const float8 U = sum(Vec3<float8>(cross(v2+v0,e0))) ^ sgnDen;
	valid &= U >= 0.0f;
	if (likely(none(valid))) return;
	const float8 V = sum(Vec3<float8>(cross(v0+v1,e1))) ^ sgnDen;
	valid &= V >= 0.0f;
	if (likely(none(valid))) return;
	const float8 W = sum(Vec3<float8>(cross(v1+v2,e2))) ^ sgnDen;

#endif        


	valid &= W >= 0.0f;
	if (likely(none(valid))) return;
        
	/* perform depth test */
	const float8 _t = dot(v0,Ng) ^ sgnDen;
	valid &= (_t >= absDen*ray.tnear) & (absDen*ray.tfar >= _t);
	if (unlikely(none(valid))) return;
        
	/* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
	valid &= den > float8(zero);
	if (unlikely(none(valid))) return;
#else
	valid &= den != float8(zero);
	if (unlikely(none(valid))) return;
#endif
        
	/* calculate hit information */
	const float8 rcpAbsDen = rcp(absDen);
	const float8 u =  U*rcpAbsDen;
	const float8 v =  V*rcpAbsDen;
	const float8 t = _t*rcpAbsDen;
        
#if FORCE_TRIANGLE_UV == 0
	const Vec3<float8> tri012_uv = getV012(grid_uv,offset0,offset1,offset2);	
	const Vec2<float8> uv0 = decodeUV(tri012_uv[0]);
	const Vec2<float8> uv1 = decodeUV(tri012_uv[1]);
	const Vec2<float8> uv2 = decodeUV(tri012_uv[2]);        
	const Vec2<float8> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;
	const float8 u_final = uv[1];
	const float8 v_final = uv[0];
#else
	const float8 u_final = u;
	const float8 v_final = v;
#endif
        
	size_t i = select_min(valid,t);

	
	/* update hit information */
	pre.hit_patch = pre.current_patch;

	ray.u         = u_final[i];
	ray.v         = v_final[i];
	ray.tfar      = t[i];

#if ENABLE_NORMALIZED_INTERSECTION == 1
	Ng = Ng * Vec3<float8>(pre.ray_dir_scale.x,pre.ray_dir_scale.y,pre.ray_dir_scale.z);
#endif
	if (i % 2)
	  {
	    ray.Ng.x      = Ng.x[i];
	    ray.Ng.y      = Ng.y[i];
	    ray.Ng.z      = Ng.z[i];
	  }
	else
	  {
	    ray.Ng.x      = -Ng.x[i];
	    ray.Ng.y      = -Ng.y[i];
	    ray.Ng.z      = -Ng.z[i];	    
	  }
      };

        static __forceinline bool occluded1_precise_3x3(Ray& ray,
							const float *const grid_x,
							const float *const grid_y,
							const float *const grid_z,
							const float *const grid_uv,
							const size_t line_offset,
							const void* geom)
      {
	const size_t offset0 = 0 * line_offset;
	const size_t offset1 = 1 * line_offset;
	const size_t offset2 = 2 * line_offset;

	const Vec3<float8> tri012_x = getV012(grid_x,offset0,offset1,offset2);
	const Vec3<float8> tri012_y = getV012(grid_y,offset0,offset1,offset2);
	const Vec3<float8> tri012_z = getV012(grid_z,offset0,offset1,offset2);

	const Vec3<float8> v0_org(tri012_x[0],tri012_y[0],tri012_z[0]);
	const Vec3<float8> v1_org(tri012_x[1],tri012_y[1],tri012_z[1]);
	const Vec3<float8> v2_org(tri012_x[2],tri012_y[2],tri012_z[2]);
        
        const Vec3<float8> O = ray.org;
        const Vec3<float8> D = ray.dir;
        
        const Vec3<float8> v0 = v0_org - O;
        const Vec3<float8> v1 = v1_org - O;
        const Vec3<float8> v2 = v2_org - O;
        
        const Vec3<float8> e0 = v2 - v0;
        const Vec3<float8> e1 = v0 - v1;	     
        const Vec3<float8> e2 = v1 - v2;	     
        
        /* calculate geometry normal and denominator */
        const Vec3<float8> Ng1 = cross(e1,e0);
        const Vec3<float8> Ng = Ng1+Ng1;
        const float8 den = dot(Ng,D);
        const float8 absDen = abs(den);
        const float8 sgnDen = signmsk(den);
        
        bool8 valid ( true );
        /* perform edge tests */
        const float8 U = dot(Vec3<float8>(cross(v2+v0,e0)),D) ^ sgnDen;
        valid &= U >= 0.0f;
        if (likely(none(valid))) return false;
        const float8 V = dot(Vec3<float8>(cross(v0+v1,e1)),D) ^ sgnDen;
        valid &= V >= 0.0f;
        if (likely(none(valid))) return false;
        const float8 W = dot(Vec3<float8>(cross(v1+v2,e2)),D) ^ sgnDen;
        valid &= W >= 0.0f;
        if (likely(none(valid))) return false;
        
        /* perform depth test */
        const float8 _t = dot(v0,Ng) ^ sgnDen;
        valid &= (_t >= absDen*ray.tnear) & (absDen*ray.tfar >= _t);
        if (unlikely(none(valid))) return false;
        
        /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
        valid &= den > float8(zero);
        if (unlikely(none(valid))) return false;
#else
        valid &= den != float8(zero);
        if (unlikely(none(valid))) return false;
#endif
        return true;
      };

#endif      
      
      /* intersect ray with Quad2x2 structure => 1 ray vs. 8 triangles */
      template<class M, class T>
        static __forceinline void intersect1_precise(Ray& ray,
                                                     const Quad2x2 &qquad,
                                                     const void* geom,
                                                     Precalculations &pre,
                                                     const size_t delta = 0)
      {
        const Vec3<T> v0_org = qquad.getVtx( 0, delta);
        const Vec3<T> v1_org = qquad.getVtx( 1, delta);
        const Vec3<T> v2_org = qquad.getVtx( 2, delta);
        
        const Vec3<T> O = ray.org;
        const Vec3<T> D = ray.dir;
        
        const Vec3<T> v0 = v0_org - O;
        const Vec3<T> v1 = v1_org - O;
        const Vec3<T> v2 = v2_org - O;
        
        const Vec3<T> e0 = v2 - v0;
        const Vec3<T> e1 = v0 - v1;	     
        const Vec3<T> e2 = v1 - v2;	     
        
        /* calculate geometry normal and denominator */
        const Vec3<T> Ng1 = cross(e1,e0);
        const Vec3<T> Ng = Ng1+Ng1;
        const T den = dot(Ng,D);
        const T absDen = abs(den);
        const T sgnDen = signmsk(den);
        
        M valid ( true );
        /* perform edge tests */
        const T U = dot(Vec3<T>(cross(v2+v0,e0)),D) ^ sgnDen;
        valid &= U >= 0.0f;
        if (likely(none(valid))) return;
        const T V = dot(Vec3<T>(cross(v0+v1,e1)),D) ^ sgnDen;
        valid &= V >= 0.0f;
        if (likely(none(valid))) return;
        const T W = dot(Vec3<T>(cross(v1+v2,e2)),D) ^ sgnDen;

        valid &= W >= 0.0f;
        if (likely(none(valid))) return;
        
        /* perform depth test */
        const T _t = dot(v0,Ng) ^ sgnDen;
        valid &= (_t >= absDen*ray.tnear) & (absDen*ray.tfar >= _t);
        if (unlikely(none(valid))) return;
        
        /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
        valid &= den > T(zero);
        if (unlikely(none(valid))) return;
#else
        valid &= den != T(zero);
        if (unlikely(none(valid))) return;
#endif
        
        /* calculate hit information */
        const T rcpAbsDen = rcp(absDen);
        const T u =  U*rcpAbsDen;
        const T v =  V*rcpAbsDen;
        const T t = _t*rcpAbsDen;
        
#if FORCE_TRIANGLE_UV == 0
        const Vec2<T> uv0 = qquad.getUV( 0, delta );
        const Vec2<T> uv1 = qquad.getUV( 1, delta );
        const Vec2<T> uv2 = qquad.getUV( 2, delta );
        
        const Vec2<T> uv = u * uv1 + v * uv2 + (1.0f-u-v) * uv0;
        
        const T u_final = uv[0];
        const T v_final = uv[1];
        
#else
        const T u_final = u;
        const T v_final = v;
#endif
        
        size_t i = select_min(valid,t);

        
        /* update hit information */
        pre.hit_patch = pre.current_patch;

        ray.u         = u_final[i];
        ray.v         = v_final[i];
        ray.tfar      = t[i];
	if (i % 2)
	  {
	    ray.Ng.x      = Ng.x[i];
	    ray.Ng.y      = Ng.y[i];
	    ray.Ng.z      = Ng.z[i];
	  }
	else
	  {
	    ray.Ng.x      = -Ng.x[i];
	    ray.Ng.y      = -Ng.y[i];
	    ray.Ng.z      = -Ng.z[i];	    
	  }
      };
      
      
      /*! intersect ray with Quad2x2 structure => 1 ray vs. 8 triangles */
      template<class M, class T>
        static __forceinline bool occluded1_precise(Ray& ray,
                                                    const Quad2x2 &qquad,
                                                    const void* geom,
						    const size_t delta = 0)
      {
        const Vec3<T> v0_org = qquad.getVtx( 0, delta );
        const Vec3<T> v1_org = qquad.getVtx( 1, delta );
        const Vec3<T> v2_org = qquad.getVtx( 2, delta );
        
        const Vec3<T> O = ray.org;
        const Vec3<T> D = ray.dir;
        
        const Vec3<T> v0 = v0_org - O;
        const Vec3<T> v1 = v1_org - O;
        const Vec3<T> v2 = v2_org - O;
        
        const Vec3<T> e0 = v2 - v0;
        const Vec3<T> e1 = v0 - v1;	     
        const Vec3<T> e2 = v1 - v2;	     
        
        /* calculate geometry normal and denominator */
        const Vec3<T> Ng1 = cross(e1,e0);
        const Vec3<T> Ng = Ng1+Ng1;
        const T den = dot(Ng,D);
        const T absDen = abs(den);
        const T sgnDen = signmsk(den);
        
        M valid ( true );
        /* perform edge tests */
        const T U = dot(Vec3<T>(cross(v2+v0,e0)),D) ^ sgnDen;
        valid &= U >= 0.0f;
        if (likely(none(valid))) return false;
        const T V = dot(Vec3<T>(cross(v0+v1,e1)),D) ^ sgnDen;
        valid &= V >= 0.0f;
        if (likely(none(valid))) return false;
        const T W = dot(Vec3<T>(cross(v1+v2,e2)),D) ^ sgnDen;
        valid &= W >= 0.0f;
        if (likely(none(valid))) return false;
        
        /* perform depth test */
        const T _t = dot(v0,Ng) ^ sgnDen;
        valid &= (_t >= absDen*ray.tnear) & (absDen*ray.tfar >= _t);
        if (unlikely(none(valid))) return false;
        
        /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
        valid &= den > T(zero);
        if (unlikely(none(valid))) return false;
#else
        valid &= den != T(zero);
        if (unlikely(none(valid))) return false;
#endif
        return true;
      };

      static size_t lazyBuildPatch(Precalculations &pre, SubdivPatch1Cached* const subdiv_patch, const void* geom);                  
      
      /*! Evaluates grid over patch and builds BVH4 tree over the grid. */
      static BVH4::NodeRef buildSubdivPatchTree(const SubdivPatch1Cached &patch,
                                                void *const lazymem,
                                                const SubdivMesh* const geom);

      /*! Evaluates grid over patch and builds BVH4 tree over the grid. */
      static BVH4::NodeRef buildSubdivPatchTreeCompact(const SubdivPatch1Cached &patch,
						       void *const lazymem,
						       const SubdivMesh* const geom);
      
      /*! Create BVH4 tree over grid. */
      static BBox3fa createSubTree(BVH4::NodeRef &curNode,
                                   float *const lazymem,
                                   const SubdivPatch1Cached &patch,
                                   const float *const grid_x_array,
                                   const float *const grid_y_array,
                                   const float *const grid_z_array,
                                   const float *const grid_u_array,
                                   const float *const grid_v_array,
                                   const GridRange &range,
                                   unsigned int &localCounter,
                                   const SubdivMesh* const geom);

      /*! Create BVH4 tree over grid. */
      static BBox3fa createSubTreeCompact(BVH4::NodeRef &curNode,
					  float *const lazymem,
					  const SubdivPatch1Cached &patch,
					  const float *const grid_array,
					  const size_t grid_array_elements,
					  const GridRange &range,
					  unsigned int &localCounter);
      

        
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      
      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive* prim, size_t ty, const void* geom, size_t& lazy_node) 
      {
        STAT3(normal.trav_prims,1,1,1);
        
        if (likely(ty == 2))
        {

#if COMPACT == 1          
          const size_t dim_offset    = pre.current_patch->grid_size_simd_blocks * 8;
          const size_t line_offset   = pre.current_patch->grid_u_res;
          const size_t offset_bytes  = ((size_t)prim  - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr()) >> 2;   
          const float *const grid_x  = (float*)(offset_bytes + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr());
          const float *const grid_y  = grid_x + 1 * dim_offset;
          const float *const grid_z  = grid_x + 2 * dim_offset;
          const float *const grid_uv = grid_x + 3 * dim_offset;
#if defined(__AVX__)
	  intersect1_precise_3x3( ray, grid_x,grid_y,grid_z,grid_uv, line_offset, (SubdivMesh*)geom,pre);
#else
	  intersect1_precise_2x3( ray, grid_x            ,grid_y            ,grid_z            ,grid_uv            , line_offset, (SubdivMesh*)geom,pre);
	  intersect1_precise_2x3( ray, grid_x+line_offset,grid_y+line_offset,grid_z+line_offset,grid_uv+line_offset, line_offset, (SubdivMesh*)geom,pre);
#endif

#else

	  const Quad2x2 &q = *(Quad2x2*)prim;

#if defined(__AVX__)
          intersect1_precise<bool8,float8>( ray, q, (SubdivMesh*)geom,pre);
#else
          intersect1_precise<bool4,float4>( ray, q, (SubdivMesh*)geom,pre,0);
          intersect1_precise<bool4,float4>( ray, q, (SubdivMesh*)geom,pre,6);
#endif

#endif
        }
        else 
        {
	  lazy_node = lazyBuildPatch(pre,(SubdivPatch1Cached*)prim, geom);
	  assert(lazy_node);
          pre.current_patch = (SubdivPatch1Cached*)prim;
        }             
      }
      
      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive* prim, size_t ty, const void* geom, size_t& lazy_node) 
      {
        STAT3(shadow.trav_prims,1,1,1);
        
        if (likely(ty == 2))
        {

#if COMPACT == 1          
          const size_t dim_offset    = pre.current_patch->grid_size_simd_blocks * 8;
          const size_t line_offset   = pre.current_patch->grid_u_res;
          const size_t offset_bytes  = ((size_t)prim  - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr()) >> 2;   
          const float *const grid_x  = (float*)(offset_bytes + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr());
          const float *const grid_y  = grid_x + 1 * dim_offset;
          const float *const grid_z  = grid_x + 2 * dim_offset;
          const float *const grid_uv = grid_x + 3 * dim_offset;

#if defined(__AVX__)
	  return occluded1_precise_3x3( ray, grid_x,grid_y,grid_z,grid_uv, line_offset, (SubdivMesh*)geom);
#else
	  if (occluded1_precise_2x3( ray, grid_x            ,grid_y            ,grid_z            ,grid_uv            , line_offset, (SubdivMesh*)geom)) return true;
	  if (occluded1_precise_2x3( ray, grid_x+line_offset,grid_y+line_offset,grid_z+line_offset,grid_uv+line_offset, line_offset, (SubdivMesh*)geom)) return true;
#endif

          
#else

#if defined(__AVX__)
	  const Quad2x2 &q = *(Quad2x2*)prim;
	  return occluded1_precise<bool8,float8>( ray, q, (SubdivMesh*)geom);
#else
          const Quad2x2 &q = *(Quad2x2*)prim;
          if (occluded1_precise<bool4,float4>( ray, q, (SubdivMesh*)geom,0)) return true;
          if (occluded1_precise<bool4,float4>( ray, q, (SubdivMesh*)geom,6)) return true;
#endif

#endif
        }
        else 
        {
	  lazy_node = lazyBuildPatch(pre,(SubdivPatch1Cached*)prim, geom);
          pre.current_patch = (SubdivPatch1Cached*)prim;
        }             
        return false;
      }
      
      
    };
  }
}
