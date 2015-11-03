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

#include "triangle.h"
#include "trianglev.h"
#include "trianglev_mb.h"
#include "intersector_epilog.h"

/*! Modified Pluecker ray/triangle intersector. The test first shifts
 *  the ray origin into the origin of the coordinate system and then
 *  uses Pluecker coordinates for the intersection. Due to the shift,
 *  the Pluecker coordinate calculation simplifies and the tests get
 *  numerically stable. The edge equations are watertight along the
 *  edge for neighboring triangles. */

namespace embree
{
  namespace isa
  {
    template<int M, typename UVMapper>
      struct PlueckerHitM
    {
      __forceinline PlueckerHitM(const vfloat<M>& U, const vfloat<M>& V, const vfloat<M>& T, const vfloat<M>& den, const Vec3<vfloat<M>>& Ng, const UVMapper& mapUV)
        : U(U), V(V), T(T), den(den), vNg(Ng), mapUV(mapUV) {}
      
      __forceinline void finalize() 
      {
        const vfloat<M> rcpDen = rcp(den);
        vt = T * rcpDen;
        vu = U * rcpDen;
        vv = V * rcpDen;
        mapUV(vu,vv);
      }
      
      __forceinline Vec2f uv (const size_t i) const { return Vec2f(vu[i],vv[i]); }
      __forceinline float t  (const size_t i) const { return vt[i]; }
      __forceinline Vec3fa Ng(const size_t i) const { return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]); }
      
    private:
      const vfloat<M> U;
      const vfloat<M> V;
      const vfloat<M> T;
      const vfloat<M> den;
      const UVMapper& mapUV;
      
    public:
      vfloat<M> vu;
      vfloat<M> vv;
      vfloat<M> vt;
      Vec3<vfloat<M>> vNg;
    };

    template<int M>
      struct PlueckerIntersector1
      {
        __forceinline PlueckerIntersector1(const Ray& ray, const void* ptr) {}
        
        template<typename UVMapper, typename Epilog>
          __forceinline bool intersect(Ray& ray, 
                                       const Vec3<vfloat<M>>& tri_v0, 
                                       const Vec3<vfloat<M>>& tri_v1, 
                                       const Vec3<vfloat<M>>& tri_v2,  
                                       const UVMapper& mapUV,
                                       const Epilog& epilog) const
        {
          /* calculate vertices relative to ray origin */
          typedef Vec3<vfloat<M>> Vec3vfM;
          const Vec3vfM O = Vec3vfM(ray.org);
          const Vec3vfM D = Vec3vfM(ray.dir);
          const Vec3vfM v0 = tri_v0-O;
          const Vec3vfM v1 = tri_v1-O;
          const Vec3vfM v2 = tri_v2-O;
          
          /* calculate triangle edges */
          const Vec3vfM e0 = v2-v0;
          const Vec3vfM e1 = v0-v1;
          const Vec3vfM e2 = v1-v2;
          
          /* perform edge tests */
          const vfloat<M> U = dot(cross(v2+v0,e0),D);
          const vfloat<M> V = dot(cross(v0+v1,e1),D);
          const vfloat<M> W = dot(cross(v1+v2,e2),D);
          const vfloat<M> minUVW = min(U,V,W);
          const vfloat<M> maxUVW = max(U,V,W);
          vbool<M> valid = minUVW >= 0.0f;
#if !defined(RTCORE_BACKFACE_CULLING)
          valid |= maxUVW <= 0.0f;
#endif
          if (unlikely(none(valid))) return false;
          
          /* calculate geometry normal and denominator */
          //const Vec3vfM Ng1 = cross(e1,e0);
          const Vec3vfM Ng1 = stable_triangle_normal(e2,e1,e0);
          const Vec3vfM Ng = Ng1+Ng1;
          const vfloat<M> den = dot(Ng,D);
          const vfloat<M> absDen = abs(den);
          const vfloat<M> sgnDen = signmsk(den);
          
          /* perform depth test */
          const vfloat<M> T = dot(v0,Ng);
          valid &= ((T^sgnDen) >= absDen*vfloat<M>(ray.tnear));
          valid &=(absDen*vfloat<M>(ray.tfar) >= (T^sgnDen));
          if (unlikely(none(valid))) return false;
          
          /* avoid division by 0 */
          valid &= den != vfloat<M>(zero);
          if (unlikely(none(valid))) return false;
          
          /* update hit information */
          PlueckerHitM<M,UVMapper> hit(U,V,T,den,Ng,mapUV);
          return epilog(valid,hit);
        }
      };

    template<int K, typename UVMapper>
      struct PlueckerHitK
    {
      __forceinline PlueckerHitK(const vfloat<K>& U, const vfloat<K>& V, const vfloat<K>& T, const vfloat<K>& den, const Vec3<vfloat<K>>& Ng, const UVMapper& mapUV)
        : U(U), V(V), T(T), den(den), Ng(Ng), mapUV(mapUV) {}
      
      __forceinline std::tuple<vfloat<K>,vfloat<K>,vfloat<K>,Vec3<vfloat<K>>> operator() () const
      {
        const vfloat<K> rcpDen = rcp(den);
        const vfloat<K> t = T * rcpDen;
        vfloat<K> u = U * rcpDen;
        vfloat<K> v = V * rcpDen;
        mapUV(u,v);
        return std::make_tuple(u,v,t,Ng);
      }
      
    private:
      const vfloat<K> U;
      const vfloat<K> V;
      const vfloat<K> T;
      const vfloat<K> den;
      const Vec3<vfloat<K>> Ng;
      const UVMapper& mapUV;
    };
    
    template<int M, int K>
      struct PlueckerIntersectorK
      {
        __forceinline PlueckerIntersectorK(const vbool<K>& valid, const RayK<K>& ray) {}
        
        /*! Intersects K rays with one of M triangles. */
        template<typename UVMapper, typename Epilog>
          __forceinline vbool<K> intersectK(const vbool<K>& valid0, 
                                            RayK<K>& ray, 
                                            const Vec3<vfloat<K>>& tri_v0, 
                                            const Vec3<vfloat<K>>& tri_v1, 
                                            const Vec3<vfloat<K>>& tri_v2, 
                                            const UVMapper& mapUV,
                                            const Epilog& epilog) const
        {
          /* calculate vertices relative to ray origin */
          typedef Vec3<vfloat<K>> Vec3vfK;
          vbool<K> valid = valid0;
          const Vec3vfK O = ray.org;
          const Vec3vfK D = ray.dir;
          const Vec3vfK v0 = tri_v0-O;
          const Vec3vfK v1 = tri_v1-O;
          const Vec3vfK v2 = tri_v2-O;
          
          /* calculate triangle edges */
          const Vec3vfK e0 = v2-v0;
          const Vec3vfK e1 = v0-v1;
          const Vec3vfK e2 = v1-v2;
           
          /* perform edge tests */
          const vfloat<K> U = dot(Vec3vfK(cross(v2+v0,e0)),D);
          const vfloat<K> V = dot(Vec3vfK(cross(v0+v1,e1)),D);
          const vfloat<K> W = dot(Vec3vfK(cross(v1+v2,e2)),D);
          const vfloat<K> minUVW = min(U,V,W);
          const vfloat<K> maxUVW = max(U,V,W);
#if defined(RTCORE_BACKFACE_CULLING)
          valid &= minUVW >= 0.0f;
#else
          valid &= (minUVW >= 0.0f) | (maxUVW <= 0.0f);
#endif
          if (unlikely(none(valid))) return false;
          
           /* calculate geometry normal and denominator */
          //const Vec3vfK Ng1 = cross(e1,e0);
          const Vec3vfK Ng1 = stable_triangle_normal(e2,e1,e0);
          const Vec3vfK Ng = Ng1+Ng1;
          const vfloat<K> den = dot(Vec3vfK(Ng),D);
          const vfloat<K> absDen = abs(den);
          const vfloat<K> sgnDen = signmsk(den);

          /* perform depth test */
          const vfloat<K> T = dot(v0,Vec3vfK(Ng));
          valid &= ((T^sgnDen) >= absDen*ray.tnear);
          valid &= (absDen*ray.tfar >= (T^sgnDen));
          if (unlikely(none(valid))) return false;
          
          /* avoid division by 0 */
          valid &= den != vfloat<K>(zero);
          if (unlikely(none(valid))) return false;
          
          /* calculate hit information */
          PlueckerHitK<K,UVMapper> hit(U,V,T,den,Ng,mapUV);
          return epilog(valid,hit);
        }
        
        /*! Intersect k'th ray from ray packet of size K with M triangles. */
        template<typename UVMapper, typename Epilog>
          __forceinline bool intersect(RayK<K>& ray, size_t k,
                                       const Vec3<vfloat<M>>& tri_v0, 
                                       const Vec3<vfloat<M>>& tri_v1, 
                                       const Vec3<vfloat<M>>& tri_v2, 
                                       const UVMapper& mapUV,
                                       const Epilog& epilog) const
        {
          /* calculate vertices relative to ray origin */
          typedef Vec3<vfloat<M>> Vec3vfM;
          const Vec3vfM O = broadcast<vfloat<M>>(ray.org,k);
          const Vec3vfM D = broadcast<vfloat<M>>(ray.dir,k);
          const Vec3vfM v0 = tri_v0-O;
          const Vec3vfM v1 = tri_v1-O;
          const Vec3vfM v2 = tri_v2-O;
          
          /* calculate triangle edges */
          const Vec3vfM e0 = v2-v0;
          const Vec3vfM e1 = v0-v1;
          const Vec3vfM e2 = v1-v2;
          
          /* perform edge tests */
          const vfloat<M> U = dot(cross(v2+v0,e0),D);
          const vfloat<M> V = dot(cross(v0+v1,e1),D);
          const vfloat<M> W = dot(cross(v1+v2,e2),D);
          const vfloat<M> minUVW = min(U,V,W);
          const vfloat<M> maxUVW = max(U,V,W);
#if defined(RTCORE_BACKFACE_CULLING)
          vbool<M> valid = minUVW >= 0.0f;
#else
          vbool<M> valid = (minUVW >= 0.0f) | (maxUVW <= 0.0f);
#endif
          if (unlikely(none(valid))) return false;
          
          /* calculate geometry normal and denominator */
          //const Vec3vfM Ng1 = cross(e1,e0);
          const Vec3vfM Ng1 = stable_triangle_normal(e2,e1,e0);
          const Vec3vfM Ng = Ng1+Ng1;
          const vfloat<M> den = dot(Ng,D);
          const vfloat<M> absDen = abs(den);
          const vfloat<M> sgnDen = signmsk(den);

          /* perform depth test */
          const vfloat<M> T = dot(v0,Ng);
          valid &= ((T^sgnDen) >= absDen*vfloat<M>(ray.tnear[k]));
          valid &= (absDen*vfloat<M>(ray.tfar[k]) >= (T^sgnDen));
          if (unlikely(none(valid))) return false;
          
          /* avoid division by 0 */
          valid &= den != vfloat<M>(zero);
          if (unlikely(none(valid))) return false;
          
          /* calculate hit information */
          PlueckerHitM<M,UVMapper> hit(U,V,T,den,Ng,mapUV);
          return epilog(valid,hit);
        }
      };
    
    /*! Intersects M triangles with 1 ray */
    template<int M, int Mx, bool filter>
      struct TriangleMvIntersector1Pluecker
      {
        typedef TriangleMv<M> Primitive;
        typedef PlueckerIntersector1<Mx> Precalculations;
        
        /*! Intersect a ray with M triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
          pre.intersect(ray,tri.v0,tri.v1,tri.v2,UVIdentity<M>(),Intersect1Epilog<M,Mx,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID)); 
        }
        
        /*! Test if the ray is occluded by one of the M triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return pre.intersect(ray,tri.v0,tri.v1,tri.v2,UVIdentity<M>(),Occluded1Epilog<M,Mx,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID)); 
        }
      };
    
    /*! Intersects M triangles with K rays */
    template<int M, int Mx, int K, bool filter>
      struct TriangleMvIntersectorKPluecker
      {
        typedef TriangleMv<M> Primitive;
        typedef PlueckerIntersectorK<Mx,K> Precalculations;
        
        /*! Intersects K rays with M triangles. */
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive& tri, Scene* scene)
        {
          for (size_t i=0; i<M; i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),K);
            const Vec3<vfloat<K>> v0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> v1 = broadcast<vfloat<K>>(tri.v1,i);
            const Vec3<vfloat<K>> v2 = broadcast<vfloat<K>>(tri.v2,i);
            pre.intersectK(valid_i,ray,v0,v1,v2,UVIdentity<K>(),IntersectKEpilog<M,K,filter>(ray,tri.geomIDs,tri.primIDs,i,scene));
          }
        }
        
        /*! Test for K rays if they are occluded by any of the M triangles. */
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive& tri, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          
          for (size_t i=0; i<M; i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid_i),K);
            const Vec3<vfloat<K>> v0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> v1 = broadcast<vfloat<K>>(tri.v1,i);
            const Vec3<vfloat<K>> v2 = broadcast<vfloat<K>>(tri.v2,i);
            pre.intersectK(valid0,ray,v0,v1,v2,UVIdentity<K>(),OccludedKEpilog<M,K,filter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with M triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          pre.intersect(ray,k,tri.v0,tri.v1,tri.v2,UVIdentity<Mx>(),Intersect1KEpilog<M,Mx,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene)); //FIXME: M,Mx
        }
        
        /*! Test if the ray is occluded by one of the M triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return pre.intersect(ray,k,tri.v0,tri.v1,tri.v2,UVIdentity<Mx>(),Occluded1KEpilog<M,Mx,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene)); //FIXME: M,Mx
        }
      };
  }
}
