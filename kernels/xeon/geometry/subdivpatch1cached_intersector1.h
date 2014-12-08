// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "subdivpatch1cached.h"
#include "common/ray.h"
#include "geometry/filter.h"
#include "bvh4/bvh4.h"
#include "common/subdiv/tessellation.h"
#include "common/subdiv/tessellation_cache.h"


#define COMPUTE_SUBDIV_NORMALS_AFTER_PATCH_INTERSECTION 0
#define FORCE_TRIANGLE_UV 0
#define DISCRITIZED_UV 1

namespace embree
{

  /* 3x3 point grid => 2x2 quad grid */
  struct __aligned(64) Quad2x2
  {
   
    Quad2x2() 
      {
      }

    /*  v00 - v01 - v02 */
    /*  v10 - v11 - v12 */
    /*  v20 - v21 - v22 */

    /* v00 - v10 - v01 - v11 - v02 - v12 */
    /* v10 - v20 - v11 - v21 - v12 - v22 */

    float vtx_x[12];
    float vtx_y[12];
    float vtx_z[12];
#if DISCRITIZED_UV == 1
    unsigned short vtx_u[12];
    unsigned short vtx_v[12];
#else
    float vtx_u[12];
    float vtx_v[12];
#endif
    static __forceinline ssef u16_to_ssef(const unsigned short *const source)
    {
      const ssei t = _mm_cvtepu16_epi32(loadu4i(source));
      return ssef(t) * 1.0f/65535.0f;
    } 

    static __forceinline unsigned short float_to_u16(const float f)
    {
      return (unsigned short)(f*65535.0f);
    } 


    __forceinline void initFrom3x3Grid( const float *const source,
                                        float *const dest,
                                        const size_t offset_line0,
                                        const size_t offset_line1,
                                        const size_t offset_line2)
    {
      const float v00 = source[offset_line0 + 0];
      const float v01 = source[offset_line0 + 1];
      const float v02 = source[offset_line0 + 2];
      const float v10 = source[offset_line1 + 0];
      const float v11 = source[offset_line1 + 1];
      const float v12 = source[offset_line1 + 2];
      const float v20 = source[offset_line2 + 0];
      const float v21 = source[offset_line2 + 1];
      const float v22 = source[offset_line2 + 2];

      /* v00 - v10 - v01 - v11 - v02 - v12 */
      dest[ 0] = v00;
      dest[ 1] = v10;
      dest[ 2] = v01;
      dest[ 3] = v11;
      dest[ 4] = v02;
      dest[ 5] = v12;
      /* v10 - v20 - v11 - v21 - v12 - v22 */
      dest[ 6] = v10;
      dest[ 7] = v20;
      dest[ 8] = v11;
      dest[ 9] = v21;
      dest[10] = v12;
      dest[11] = v22;
    }

    __forceinline void initFrom3x3Grid_discritized( const float *const source,
                                                    unsigned short *const dest,
                                                    const size_t offset_line0,
                                                    const size_t offset_line1,
                                                    const size_t offset_line2)
    {
      const float v00 = source[offset_line0 + 0];
      const float v01 = source[offset_line0 + 1];
      const float v02 = source[offset_line0 + 2];
      const float v10 = source[offset_line1 + 0];
      const float v11 = source[offset_line1 + 1];
      const float v12 = source[offset_line1 + 2];
      const float v20 = source[offset_line2 + 0];
      const float v21 = source[offset_line2 + 1];
      const float v22 = source[offset_line2 + 2];

      /* v00 - v10 - v01 - v11 - v02 - v12 */
      dest[ 0] = float_to_u16(v00);
      dest[ 1] = float_to_u16(v10);
      dest[ 2] = float_to_u16(v01);
      dest[ 3] = float_to_u16(v11);
      dest[ 4] = float_to_u16(v02);
      dest[ 5] = float_to_u16(v12);
      /* v10 - v20 - v11 - v21 - v12 - v22 */
      dest[ 6] = float_to_u16(v10);
      dest[ 7] = float_to_u16(v20);
      dest[ 8] = float_to_u16(v11);
      dest[ 9] = float_to_u16(v21);
      dest[10] = float_to_u16(v12);
      dest[11] = float_to_u16(v22);
    }

    /* init from 3x3 point grid */
    void init( const float * const grid_x,
               const float * const grid_y,
               const float * const grid_z,
               const float * const grid_u,
               const float * const grid_v,
               const size_t offset_line0,
               const size_t offset_line1,
               const size_t offset_line2)
    {
      initFrom3x3Grid( grid_x, vtx_x, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_y, vtx_y, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_z, vtx_z, offset_line0, offset_line1, offset_line2 );
#if DISCRITIZED_UV == 1
      initFrom3x3Grid_discritized( grid_u, vtx_u, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid_discritized( grid_v, vtx_v, offset_line0, offset_line1, offset_line2 );
#else
      initFrom3x3Grid( grid_u, vtx_u, offset_line0, offset_line1, offset_line2 );
      initFrom3x3Grid( grid_v, vtx_v, offset_line0, offset_line1, offset_line2 );
#endif
    }

#if defined(__AVX__)

    __forceinline avxf combine( const float *const source, const size_t offset) const {
      return avxf( *(ssef*)&source[0+offset], *(ssef*)&source[6+offset] );            
    }

    __forceinline avxf combine_discritized( const unsigned short *const source, const size_t offset) const {
      return avxf( u16_to_ssef(&source[0+offset]), u16_to_ssef(&source[6+offset]) );            
    }

    __forceinline avx3f getVtx( const size_t offset, const size_t delta = 0 ) const {
      return avx3f(  combine(vtx_x,offset), combine(vtx_y,offset), combine(vtx_z,offset) );
    }

    __forceinline avx2f getUV( const size_t offset, const size_t delta = 0 ) const {
#if DISCRITIZED_UV == 1
      return avx2f(  combine_discritized(vtx_u,offset), combine_discritized(vtx_v,offset) );
#else
      return avx2f(  combine(vtx_u,offset), combine(vtx_v,offset) );
#endif
    }

#else

    __forceinline sse3f getVtx( const size_t offset, const size_t delta = 0 ) const {
      return sse3f( loadu4f(&vtx_x[offset+delta]), loadu4f(&vtx_y[offset+delta]), loadu4f(&vtx_z[offset+delta]) );
    }

    __forceinline sse2f getUV( const size_t offset, const size_t delta = 0 ) const {
#if DISCRITIZED_UV == 1
      return sse2f( u16_to_ssef(&vtx_u[offset+delta]), u16_to_ssef(&vtx_v[offset+delta]) );
#else
      return sse2f(  loadu4f(&vtx_u[offset+delta]), loadu4f(&vtx_v[offset+delta])  );
#endif
    }
    
#endif


      __forceinline BBox3fa bounds() const 
      {
      BBox3fa b( empty );
      for (size_t i=0;i<12;i++)
        b.extend( Vec3fa(vtx_x[i],vtx_y[i],vtx_z[i]) );
      return b;
    }
    
      __forceinline Vec3fa getVec3fa_xyz(const size_t i) const {
      return Vec3fa( vtx_x[i], vtx_y[i], vtx_z[i] );
    }

      __forceinline Vec2f getVec2f_uv(const size_t i) const {
      return Vec2f( vtx_u[i], vtx_v[i] );
    }

    };

      /*! Outputs ray to stream. */
      inline std::ostream& operator<<(std::ostream& cout, const Quad2x2& qquad) {
      for (size_t i=0;i<12;i++)
        cout << "i = " << i << " -> xyz = " << qquad.getVec3fa_xyz(i) << " uv = " << qquad.getVec2f_uv(i) << std::endl;
      return cout;
    }

      /* intersect ray with Quad2x2 structure => 1 ray vs. 8 triangles */
      template<class M, class T>
        static __forceinline void intersect1_precise(Ray& ray,
        const Quad2x2 &qquad,
        const void* geom,
        bool &hit,
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
      ray.u      = u_final[i];
      ray.v      = v_final[i];
      ray.tfar   = t[i];
      ray.Ng.x   = Ng.x[i];
      ray.Ng.y   = Ng.y[i];
      ray.Ng.z   = Ng.z[i];
      hit        = true;
    };


      /* intersect ray with Quad2x2 structure => 1 ray vs. 8 triangles */
      template<class M, class T>
        static __forceinline bool occluded1_precise(Ray& ray,
        const Quad2x2 &qquad,
        const void* geom)
      {
      const Vec3<T> v0_org = qquad.getVtx( 0 );
      const Vec3<T> v1_org = qquad.getVtx( 1 );
      const Vec3<T> v2_org = qquad.getVtx( 2 );

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

  
      __forceinline void evalGrid(const SubdivPatch1Cached &patch,
                                  float *__restrict__ const grid_x,
                                  float *__restrict__ const grid_y,
                                  float *__restrict__ const grid_z,
                                  float *__restrict__ const grid_u,
                                  float *__restrict__ const grid_v,
                                  const SubdivMesh* const geom)
      {
        gridUVTessellator(patch.level,
                          patch.grid_u_res,
                          patch.grid_v_res,
                          grid_u,
                          grid_v);

        if (unlikely(patch.needsStiching()))
          stichUVGrid(patch.level,patch.grid_u_res,patch.grid_v_res,grid_u,grid_v);

   
#if defined(__AVX__)
        for (size_t i=0;i<patch.grid_size_8wide_blocks;i++)
          {
            avxf uu = load8f(&grid_u[8*i]);
            avxf vv = load8f(&grid_v[8*i]);
            avx3f vtx = patch.eval8(uu,vv);

            if (unlikely(((SubdivMesh*)geom)->displFunc != NULL))
              {
                avx3f normal = patch.normal8(uu,vv);
                normal = normalize_safe(normal);
              
                ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                               patch.geom,
                                               patch.prim,
                                               (const float*)&uu,
                                               (const float*)&vv,
                                               (const float*)&normal.x,
                                               (const float*)&normal.y,
                                               (const float*)&normal.z,
                                               (float*)&vtx.x,
                                               (float*)&vtx.y,
                                               (float*)&vtx.z,
                                               8);
              }

            *(avxf*)&grid_x[8*i] = vtx.x;
            *(avxf*)&grid_y[8*i] = vtx.y;
            *(avxf*)&grid_z[8*i] = vtx.z;        
            *(avxf*)&grid_u[8*i] = uu;
            *(avxf*)&grid_v[8*i] = vv;
          }
#else
        for (size_t i=0;i<patch.grid_size_8wide_blocks*2;i++) // 4-wide blocks for SSE
          {
            ssef uu = load4f(&grid_u[4*i]);
            ssef vv = load4f(&grid_v[4*i]);
            sse3f vtx = patch.eval4(uu,vv);

            if (unlikely(((SubdivMesh*)geom)->displFunc != NULL))
              {
                sse3f normal = patch.normal4(uu,vv);
                normal = normalize_safe(normal);
              
                ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                               patch.geom,
                                               patch.prim,
                                               (const float*)&uu,
                                               (const float*)&vv,
                                               (const float*)&normal.x,
                                               (const float*)&normal.y,
                                               (const float*)&normal.z,
                                               (float*)&vtx.x,
                                               (float*)&vtx.y,
                                               (float*)&vtx.z,
                                               4);
              }

            *(ssef*)&grid_x[4*i] = vtx.x;
            *(ssef*)&grid_y[4*i] = vtx.y;
            *(ssef*)&grid_z[4*i] = vtx.z;        
            *(ssef*)&grid_u[4*i] = uu;
            *(ssef*)&grid_v[4*i] = vv;
          }

#endif
      }

#define TIMER(x) 


      struct SubdivPatch1CachedIntersector1
      {
        typedef SubdivPatch1Cached Primitive;

        /*! Precalculations for subdiv patch intersection */
        struct Precalculations {
          Vec3fa ray_rdir, ray_org_rdir;
          SubdivPatch1Cached *last_patch;

          __forceinline Precalculations (const Ray& ray) 
          {
            ray_rdir     = rcp_safe(ray.dir);
            ray_org_rdir = ray.org*ray_rdir;
            last_patch   = NULL;
          }
        };

        /*! Per thread tessellation cache */
        static __thread TessellationCache *thread_cache;

        /*! Creates per thread tessellation cache */
        static TessellationCache *createTessellationCache();

        /*! Returns BVH4 node reference for subtree over patch grid */
        static __forceinline size_t getSubtreeRootNode(const Primitive* const subdiv_patch, const void* geom)
        {
          const unsigned int commitCounter = ((Scene*)geom)->commitCounter;

          TessellationCache *local_cache = NULL;

          if (unlikely(!thread_cache))
            thread_cache = createTessellationCache();

          local_cache = thread_cache;

          SubdivPatch1Cached* tag = (SubdivPatch1Cached*)subdiv_patch;
    
          BVH4::NodeRef root = local_cache->lookup(tag,commitCounter);
          root.prefetch(0);

          if (unlikely(root == (size_t)-1))
            {
              subdiv_patch->prefetchData();
              const unsigned int blocks = subdiv_patch->grid_subtree_size_64b_blocks;

              TessellationCache::CacheTag &t = local_cache->request(tag,commitCounter,blocks);
              BVH4::Node* node = (BVH4::Node*)local_cache->getCacheMemoryPtr(t);
              prefetchL1(((float*)node + 0*16));
              prefetchL1(((float*)node + 1*16));
              prefetchL1(((float*)node + 2*16));

              size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));
              assert( new_root != BVH4::invalidNode);

              local_cache->updateRootRef(t,new_root);

              assert( (size_t)local_cache->getPtr() + (size_t)t.getRootRef() == new_root );

              return new_root;
            }


          return root;   
        }


        /*! Evaluates grid over patch and builds BVH4 tree over the grid. */
        static __forceinline BVH4::NodeRef buildSubdivPatchTree(const SubdivPatch1Cached &patch,
                                                                void *const lazymem,
                                                                const SubdivMesh* const geom)
        {
      
          TIMER(double msec = 0.0);
          TIMER(msec = getSeconds());
      
          assert( patch.grid_size_8wide_blocks >= 1 );
          __aligned(64) float grid_x[(patch.grid_size_8wide_blocks+1)*8]; 
          __aligned(64) float grid_y[(patch.grid_size_8wide_blocks+1)*8];
          __aligned(64) float grid_z[(patch.grid_size_8wide_blocks+1)*8]; 
      
          __aligned(64) float grid_u[(patch.grid_size_8wide_blocks+1)*8]; 
          __aligned(64) float grid_v[(patch.grid_size_8wide_blocks+1)*8];
      
          evalGrid(patch,grid_x,grid_y,grid_z,grid_u,grid_v,geom);
      
          BVH4::NodeRef subtree_root = BVH4::encodeNode( (BVH4::Node*)lazymem);
          unsigned int currentIndex = 0;
      
          BBox3fa bounds = createSubTree( subtree_root,
                                          (float*)lazymem,
                                          patch,
                                          grid_x,
                                          grid_y,
                                          grid_z,
                                          grid_u,
                                          grid_v,
                                          GridRange(0,patch.grid_u_res-1,0,patch.grid_v_res-1),
                                          currentIndex,
                                          geom);
      
          assert(currentIndex == patch.grid_subtree_size_64b_blocks);
          TIMER(msec = getSeconds()-msec);    

          //TessellationCache::printStats(); 

          return subtree_root;
        }

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


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        /*! Intersect a ray with the primitive. */
        static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive* prim, size_t ty, const void* geom, size_t& lazy_node) 
        {
          STAT3(normal.trav_prims,1,1,1);

          if (likely(ty == 2))
            {
              bool hit = false;
#if defined(__AVX__)
              intersect1_precise<avxb,avxf>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom,hit);
#else
              intersect1_precise<sseb,ssef>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom,hit,0);
              intersect1_precise<sseb,ssef>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom,hit,6);
#endif
              if (unlikely(hit == true))
                {
                  const SubdivPatch1Cached *const sptr = (SubdivPatch1Cached *)pre.last_patch;
#if COMPUTE_SUBDIV_NORMALS_AFTER_PATCH_INTERSECTION == 1
                  Vec3fa normal = sptr->normal(ray.u,ray.v);
                  ray.Ng = normal;
#endif
                  ray.geomID = sptr->geom;
                  ray.primID = sptr->prim;
                }
            }
          else 
            {
              lazy_node = getSubtreeRootNode(prim, geom);
              pre.last_patch = (SubdivPatch1Cached*)prim;
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
              if (occluded1_precise<sseb,ssef>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom),0) return true;
              if (occluded1_precise<sseb,ssef>( ray, *(Quad2x2*)prim, (SubdivMesh*)geom),6) return true;
#endif
            }
          else 
            {
              lazy_node = getSubtreeRootNode(prim, geom);        
              pre.last_patch = (SubdivPatch1Cached*)prim;
            }             
      

          return false;
        }


      };
  }
