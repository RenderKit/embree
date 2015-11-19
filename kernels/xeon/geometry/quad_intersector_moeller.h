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

#include "quadv.h"
#include "triangle_intersector_moeller.h"

/*! This intersector implements a modified version of the Moeller
 *  Trumbore intersector from the paper "Fast, Minimum Storage
 *  Ray-Triangle Intersection". In contrast to the paper we
 *  precalculate some factors and factor the calculations differently
 *  to allow precalculating the cross product e1 x e2. The resulting
 *  algorithm is similar to the fastest one of the paper "Optimizing
 *  Ray-Triangle Intersection via Automated Search". */

namespace embree
{
  namespace isa
  {
    template<int M>
      struct MoellerTrumboreQuadHitM
      {
        __forceinline MoellerTrumboreQuadHitM(const vfloat<M>& U, const vfloat<M>& V, const vfloat<M>& T, const vfloat<M>& absDen, const Vec3<vfloat<M>>& Ng, const vbool<M>& flags)
          : U(U), V(V), T(T), absDen(absDen), tri_Ng(Ng) {}
      
        __forceinline void finalize() 
        {
          const vfloat<M> rcpAbsDen = rcp(absDen);
          vt = T * rcpAbsDen;
          const vfloat<M> u = U * rcpAbsDen;
          const vfloat<M> v = V * rcpAbsDen;
          const vfloat<M> u1 = vfloat<M>(1.0f) - u;
          const vfloat<M> v1 = vfloat<M>(1.0f) - v;
          vu = select(flags,u1,u);
          vv = select(flags,v1,v);
#if defined(__AVX__)
          const vfloat<M> flip(vfloat<M/2>(-1.0f),vfloat<M/2>(1.0f));
#else
          const vfloat<M> flip(1.0f);
#endif
          vNg = Vec3<vfloat<M>>(tri_Ng.x*flip,tri_Ng.y*flip,tri_Ng.z*flip);
        }

        __forceinline Vec2f uv (const size_t i) 
        { 
          const float u = vu[i];
          const float v = vv[i];
          return Vec2f(u,v);
        }

        __forceinline float t  (const size_t i) { return vt[i]; }
        __forceinline Vec3fa Ng(const size_t i) { return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]); }
      
      private:
        const vfloat<M> U;
        const vfloat<M> V;
        const vfloat<M> T;
        const vfloat<M> absDen;
        const vbool<M> flags;
        const Vec3<vfloat<M>> tri_Ng;
      
      public:
        vfloat<M> vu;
        vfloat<M> vv;
        vfloat<M> vt;
        Vec3<vfloat<M>> vNg;
      };

    template<int M>
      struct MoellerTrumboreIntersectorQuad1
      {
        __forceinline MoellerTrumboreIntersectorQuad1(const Ray& ray, const void* ptr) {}

        template<typename Epilog>
        __forceinline bool intersect(Ray& ray, 
                                     const Vec3<vfloat<M>>& tri_v0, 
                                     const Vec3<vfloat<M>>& tri_e1, 
                                     const Vec3<vfloat<M>>& tri_e2, 
                                     const Vec3<vfloat<M>>& tri_Ng,
                                     const vbool<M>& flags,
                                     const Epilog& epilog) const
        {
          /* calculate denominator */
          typedef Vec3<vfloat<M>> Vec3vfM;
          const Vec3vfM O = Vec3vfM(ray.org);
          const Vec3vfM D = Vec3vfM(ray.dir);
          const Vec3vfM C = Vec3vfM(tri_v0) - O;
          const Vec3vfM R = cross(D,C);
          const vfloat<M> den = dot(Vec3vfM(tri_Ng),D);
          const vfloat<M> absDen = abs(den);
          const vfloat<M> sgnDen = signmsk(den);
        
          /* perform edge tests */
          const vfloat<M> U = dot(R,Vec3vfM(tri_e2)) ^ sgnDen;
          const vfloat<M> V = dot(R,Vec3vfM(tri_e1)) ^ sgnDen;

          /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
          vbool<M> valid = (den > vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
          vbool<M> valid = (den != vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
          if (likely(none(valid))) return false;
        
          /* perform depth test */
          const vfloat<M> T = dot(Vec3vfM(tri_Ng),C) ^ sgnDen;
          valid &= (T > absDen*vfloat<M>(ray.tnear)) & (T < absDen*vfloat<M>(ray.tfar));
          if (likely(none(valid))) return false;

          /* update hit information */
          MoellerTrumboreQuadHitM<M> hit(U,V,T,absDen,tri_Ng, flags);
          return epilog(valid,hit);
        }
      
        template<typename Epilog>
        __forceinline bool intersect(Ray& ray, 
                                     const Vec3<vfloat<M>>& v0, 
                                     const Vec3<vfloat<M>>& v1, 
                                     const Vec3<vfloat<M>>& v2, 
                                     const vbool<M>& flags,
                                     const Epilog& epilog) const
        {
          const Vec3<vfloat<M>> e1 = v0-v1;
          const Vec3<vfloat<M>> e2 = v2-v0;
          const Vec3<vfloat<M>> Ng = cross(e1,e2);
          return intersect(ray,v0,e1,e2,Ng,flags,epilog);
        }
      };

    template<int M, int Mx, bool filter>
      struct QuadMvIntersector1MoellerTrumbore;

    /*! Intersects 4 quads with 1 ray using SSE */
    template<bool filter>
      struct QuadMvIntersector1MoellerTrumbore<4,4,filter>
    {
      typedef QuadMv<4> Primitive;
      typedef MoellerTrumboreIntersectorQuad1<4> Precalculations;
        
      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        {
          Vec3vf4 vtx0(quad.v0.x,quad.v0.y,quad.v0.z);
          Vec3vf4 vtx1(quad.v1.x,quad.v1.y,quad.v1.z);
          Vec3vf4 vtx2(quad.v3.x,quad.v3.y,quad.v3.z);
          vint4   geomIDs(quad.geomIDs); 
          vint4   primIDs(quad.primIDs);
          const vbool4 flags( false );
          pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
        }
        {
          Vec3vf4 vtx0(quad.v2.x,quad.v2.y,quad.v2.z);
          Vec3vf4 vtx2(quad.v3.x,quad.v3.y,quad.v3.z);
          Vec3vf4 vtx1(quad.v1.x,quad.v1.y,quad.v1.z);
          vint4   geomIDs(quad.geomIDs); 
          vint4   primIDs(quad.primIDs);
          const vbool4 flags( true );
          pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
        }
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        {
          Vec3vf4 vtx0(quad.v0.x,quad.v0.y,quad.v0.z);
          Vec3vf4 vtx1(quad.v1.x,quad.v1.y,quad.v1.z);
          Vec3vf4 vtx2(quad.v3.x,quad.v3.y,quad.v3.z);
          vint4   geomIDs(quad.geomIDs); 
          vint4   primIDs(quad.primIDs);
          const vbool4 flags( false );
          if (pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)))
            return true;
        }
        {
          Vec3vf4 vtx0(quad.v2.x,quad.v2.y,quad.v2.z);
          Vec3vf4 vtx1(quad.v3.x,quad.v3.y,quad.v3.z);
          Vec3vf4 vtx2(quad.v1.x,quad.v1.y,quad.v1.z);
          vint4   geomIDs(quad.geomIDs); 
          vint4   primIDs(quad.primIDs);
          const vbool4 flags( true );
          if (pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)))
            return true;
        }          
        return false;
      }
    };

#if defined(__AVX__)

    /*! Intersects 4 quads with 1 ray using AVX */
    template<bool filter>
      struct QuadMvIntersector1MoellerTrumbore<4,8,filter>
    {
      typedef QuadMv<4> Primitive;
      typedef MoellerTrumboreIntersectorQuad1<8> Precalculations;
        
      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec3vf8 vtx0(vfloat8(quad.v0.x,quad.v2.x),vfloat8(quad.v0.y,quad.v2.y),vfloat8(quad.v0.z,quad.v2.z));
        Vec3vf8 vtx1(vfloat8(quad.v1.x),vfloat8(quad.v1.y),vfloat8(quad.v1.z));
        Vec3vf8 vtx2(vfloat8(quad.v3.x),vfloat8(quad.v3.y),vfloat8(quad.v3.z));

        vint8   geomIDs(quad.geomIDs); 
        vint8   primIDs(quad.primIDs);        
        const vbool8 flags(0,0,0,0,1,1,1,1);
        pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<8,8,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf8 vtx0(vfloat8(quad.v0.x,quad.v2.x),vfloat8(quad.v0.y,quad.v2.y),vfloat8(quad.v0.z,quad.v2.z));
        Vec3vf8 vtx1(vfloat8(quad.v1.x),vfloat8(quad.v1.y),vfloat8(quad.v1.z));
        Vec3vf8 vtx2(vfloat8(quad.v3.x),vfloat8(quad.v3.y),vfloat8(quad.v3.z));
        vint8   geomIDs(quad.geomIDs); 
        vint8   primIDs(quad.primIDs);
        const vbool8 flags(0,0,0,0,1,1,1,1);
        return pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<8,8,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
    };

#endif

#if defined(__AVX512F__)

    /*! Intersects 4 triangle pairs with 1 ray using AVX512KNL */
    template<bool filter>
      struct QuadMvIntersector1MoellerTrumbore<4,16,filter>
      {
        typedef QuadMv<4> Primitive;
        typedef MoellerTrumboreIntersectorQuad1<16> Precalculations;
        
        /*! Intersect a ray with the M triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
          Vec3vf16 vtx0(select(0x0f0f,vfloat16(quad.v0.x),vfloat16(quad.v2.x)),
                        select(0x0f0f,vfloat16(quad.v0.y),vfloat16(quad.v2.y)),
                        select(0x0f0f,vfloat16(quad.v0.z),vfloat16(quad.v2.z)));
          Vec3vf16 vtx1(quad.v1.x,quad.v1.y,quad.v1.z);
          Vec3vf16 vtx2(quad.v3.x,quad.v3.y,quad.v3.z);
          vint8   geomIDs(quad.geomIDs); 
          vint8   primIDs(quad.primIDs);        
          const vbool16 flags(0xf0f0);
          pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<8,16,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
        }
        
        /*! Test if the ray is occluded by one of M triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1); 
          Vec3vf16 vtx0(select(0x0f0f,vfloat16(quad.v0.x),vfloat16(quad.v2.x)),
                        select(0x0f0f,vfloat16(quad.v0.y),vfloat16(quad.v2.y)),
                        select(0x0f0f,vfloat16(quad.v0.z),vfloat16(quad.v2.z)));
          Vec3vf16 vtx1(quad.v1.x,quad.v1.y,quad.v1.z);
          Vec3vf16 vtx2(quad.v3.x,quad.v3.y,quad.v3.z);
          vint8   geomIDs(quad.geomIDs); 
          vint8   primIDs(quad.primIDs);        
          const vbool16 flags(0xf0f0);
          return pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<8,16,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
        }
      };
#endif


  }
}

