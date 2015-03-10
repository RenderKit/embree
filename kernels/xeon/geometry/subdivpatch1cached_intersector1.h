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

#include "common/ray.h"
#include "common/scene_subdiv_mesh.h"
#include "geometry/filter.h"
#include "bvh4/bvh4.h"
#include "common/subdiv/tessellation.h"
#include "common/subdiv/tessellation_cache.h"
#include "geometry/subdivpatch1cached.h"

/* returns u,v based on individual triangles instead relative to original patch */
#define FORCE_TRIANGLE_UV 1

#define TESSELLATION_REF_CACHE_ENTRIES  4

#define SHARED_LAZY_CACHE 1

#if defined(DEBUG)
#define CACHE_STATS(x) 
#else
#define CACHE_STATS(x) 
#endif

namespace embree
{
  typedef TessellationRefCacheT<TESSELLATION_REF_CACHE_ENTRIES> TessellationRefCache;

  namespace isa
  {

    class SubdivPatch1CachedIntersector1
    {
    public:
      typedef SubdivPatch1Cached Primitive;
      
      /*! Per thread tessellation ref cache */
      static __thread TessellationRefCache            * thread_cache;
      static __thread LocalTessellationCacheThreadInfo* localThreadInfo;


      /*! Creates per thread tessellation cache */
      static void createTessellationCache();
      static void createLocalThreadInfo();
      
      /*! Precalculations for subdiv patch intersection */
      class Precalculations {
      public:
        Vec3fa ray_rdir;
        Vec3fa ray_org_rdir;
        SubdivPatch1Cached   *current_patch;
        SubdivPatch1Cached   *hit_patch;
#if SHARED_LAZY_CACHE == 1
	unsigned int threadID;
#else
	TessellationCacheTag *local_tag;
	TessellationRefCache *local_cache;
#endif
        Ray &r;
        
        __forceinline Precalculations (Ray& ray, const void *ptr) : r(ray) 
        {
          ray_rdir      = rcp_safe(ray.dir);
          ray_org_rdir  = ray.org*ray_rdir;
          current_patch = NULL;
          hit_patch     = NULL;

#if SHARED_LAZY_CACHE == 1
	  if (unlikely(!localThreadInfo))
            createLocalThreadInfo();
	    //localThreadInfo = new LocalTessellationCacheThreadInfo( SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID() );	      
	  threadID = localThreadInfo->id;
#else
          /*! Initialize per thread tessellation cache */
	  local_tag     = NULL;
          if (unlikely(!thread_cache))
            createTessellationCache();
          local_cache = thread_cache;
          assert(local_cache != NULL);
#endif
	  
        }

          /*! Final per ray computations like smooth normal, patch u,v, etc. */        
        __forceinline ~Precalculations() 
        {
#if SHARED_LAZY_CACHE == 1
	  if (current_patch)
	    {
	      SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadID);	       	      
	    }
#else
	  if (local_tag)
	    {
	      local_tag->read_unlock();
	      local_tag = NULL;
	    }
#endif

          if (unlikely(hit_patch != NULL))
          {

#if defined(RTCORE_RETURN_SUBDIV_NORMAL)
	    if (likely(!hit_patch->hasDisplacement()))
	      {		 
		Vec3fa normal = hit_patch->normal(r.u,r.v);
		r.Ng = normal;
	      }
#endif
	    const Vec2f uv0 = hit_patch->getUV(0);
	    const Vec2f uv1 = hit_patch->getUV(1);
	    const Vec2f uv2 = hit_patch->getUV(2);
	    const Vec2f uv3 = hit_patch->getUV(3);
	    
	    const float patch_u = bilinear_interpolate(uv0.x,uv1.x,uv2.x,uv3.x,r.u,r.v);
	    const float patch_v = bilinear_interpolate(uv0.y,uv1.y,uv2.y,uv3.y,r.u,r.v);

	    r.u      = patch_u;
	    r.v      = patch_v;
            r.geomID = hit_patch->geom;
            r.primID = hit_patch->prim;
          }
        }
        
      };
      
      
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

#if SHARED_LAZY_CACHE == 1
      static size_t lazyBuildPatch(Precalculations &pre, SubdivPatch1Cached* const subdiv_patch, const void* geom);
#else      
      static size_t lazyBuildPatch(Precalculations &pre, SubdivPatch1Cached* const subdiv_patch, const void* geom,TessellationRefCache *ref_cache);
#endif
                  
      
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
					  unsigned int &localCounter,
					  const SubdivMesh* const geom);
      

        
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      
      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive* prim, size_t ty, const void* geom, size_t& lazy_node) 
      {
        STAT3(normal.trav_prims,1,1,1);
        
        if (likely(ty == 2))
        {
#if defined(__AVX__)
          intersect1_precise<avxb,avxf>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom,pre);
#else
          intersect1_precise<sseb,ssef>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom,pre,0);
          intersect1_precise<sseb,ssef>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom,pre,6);
#endif
        }
        else 
        {
#if SHARED_LAZY_CACHE == 1
	  lazy_node = lazyBuildPatch(pre,(SubdivPatch1Cached*)prim, geom);
#else
	  lazy_node = lazyBuildPatch(pre,(SubdivPatch1Cached*)prim, geom, pre.local_cache);
#endif
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
#if defined(__AVX__)
          return occluded1_precise<avxb,avxf>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom);
#else
          if (occluded1_precise<sseb,ssef>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom,0)) return true;
          if (occluded1_precise<sseb,ssef>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom,6)) return true;
#endif
        }
        else 
        {
#if SHARED_LAZY_CACHE == 1
	  lazy_node = lazyBuildPatch(pre,(SubdivPatch1Cached*)prim, geom);
#else
	  lazy_node = lazyBuildPatch(pre,(SubdivPatch1Cached*)prim, geom, pre.local_cache);
#endif
          pre.current_patch = (SubdivPatch1Cached*)prim;
        }             
        return false;
      }
      
      
    };
  }
}
