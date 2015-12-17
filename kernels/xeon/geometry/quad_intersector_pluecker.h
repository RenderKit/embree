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

#include "quad_intersector_moeller.h"

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
    struct PlueckerIntersectorTriangle1
    {
        template<int M, typename Epilog>
        static __forceinline bool intersect(Ray& ray, 
                                     const Vec3<vfloat<M>>& tri_v0, 
                                     const Vec3<vfloat<M>>& tri_v1, 
                                     const Vec3<vfloat<M>>& tri_v2, 
                                     const vbool<M>& flags,
                                     const Epilog& epilog)
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
          const Vec3vfM Ng1 = cross(e1,e0);
          //const Vec3vfM Ng1 = stable_triangle_normal(e2,e1,e0); // FIXME: enable
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
          QuadHitM<M> hit(valid,U,V,T,den,Ng,flags);
          return epilog(valid,hit);
        }  
      };

    /*! Intersects M quads with 1 ray */
    template<int M, bool filter>
      struct QuadMIntersector1Pluecker
    {
      __forceinline QuadMIntersector1Pluecker(const Ray& ray, const void* ptr) {}

      __forceinline void intersect(Ray& ray, const Vec3<vfloat<M>>& v0, const Vec3<vfloat<M>>& v1, const Vec3<vfloat<M>>& v2, const Vec3<vfloat<M>>& v3, 
                                   const vint<M>& geomID, const vint<M>& primID, Scene* scene, const unsigned* geomID_to_instID) const
      {
        Intersect1Epilog<M,M,filter> epilog(ray,geomID,primID,scene,geomID_to_instID);
        PlueckerIntersectorTriangle1::intersect(ray,v0,v1,v3,vbool<M>(false),epilog);
        PlueckerIntersectorTriangle1::intersect(ray,v2,v3,v1,vbool<M>(true ),epilog);
      }
      
      __forceinline bool occluded(Ray& ray, const Vec3<vfloat<M>>& v0, const Vec3<vfloat<M>>& v1, const Vec3<vfloat<M>>& v2, const Vec3<vfloat<M>>& v3, 
                                  const vint<M>& geomID, const vint<M>& primID, Scene* scene, const unsigned* geomID_to_instID) const
      {
        Occluded1Epilog<M,M,filter> epilog(ray,geomID,primID,scene,geomID_to_instID);
        if (PlueckerIntersectorTriangle1::intersect(ray,v0,v1,v3,vbool<M>(false),epilog)) return true;
        if (PlueckerIntersectorTriangle1::intersect(ray,v2,v3,v1,vbool<M>(true ),epilog)) return true;
        return false;
      }
    };

#if defined(__AVX512F__)

    /*! Intersects 4 quads with 1 ray using AVX512 */
    template<bool filter>
      struct QuadMIntersector1Pluecker<4,filter>
    {
      __forceinline QuadMIntersector1Pluecker(const Ray& ray, const void* ptr) {}

      template<typename Epilog>
        __forceinline bool intersect(Ray& ray, const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, const Epilog& epilog) const
      {
        const Vec3vf16 vtx0(select(0x0f0f,vfloat16(v0.x),vfloat16(v2.x)),
                            select(0x0f0f,vfloat16(v0.y),vfloat16(v2.y)),
                            select(0x0f0f,vfloat16(v0.z),vfloat16(v2.z)));
        const Vec3vf16 vtx1(vfloat16(v1.x),vfloat16(v1.y),vfloat16(v1.z));
        const Vec3vf16 vtx2(vfloat16(v3.x),vfloat16(v3.y),vfloat16(v3.z));
        const vbool16 flags(0xf0f0);
        return PlueckerIntersectorTriangle1::intersect(ray,vtx0,vtx1,vtx2,flags,epilog);
      }
      
      __forceinline bool intersect(Ray& ray, const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, 
                                   const vint4& geomID, const vint4& primID, Scene* scene, const unsigned* geomID_to_instID) const
      {
        return intersect(ray,v0,v1,v2,v3,Intersect1Epilog<8,16,filter>(ray,vint8(geomID),vint8(primID),scene,geomID_to_instID));
      }
      
      __forceinline bool occluded(Ray& ray, const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, 
                                  const vint4& geomID, const vint4& primID, Scene* scene, const unsigned* geomID_to_instID) const
      {
        return intersect(ray,v0,v1,v2,v3,Occluded1Epilog<8,16,filter>(ray,vint8(geomID),vint8(primID),scene,geomID_to_instID));
      }
    };

#elif defined (__AVX__)

    /*! Intersects 4 quads with 1 ray using AVX */
    template<bool filter>
      struct QuadMIntersector1Pluecker<4,filter>
    {
      __forceinline QuadMIntersector1Pluecker(const Ray& ray, const void* ptr) {}
      
      template<typename Epilog>
        __forceinline bool intersect(Ray& ray, const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, const Epilog& epilog) const
      {
        const Vec3vf8 vtx0(vfloat8(v0.x,v2.x),vfloat8(v0.y,v2.y),vfloat8(v0.z,v2.z));
        const Vec3vf8 vtx1(vfloat8(v1.x),vfloat8(v1.y),vfloat8(v1.z));
        const Vec3vf8 vtx2(vfloat8(v3.x),vfloat8(v3.y),vfloat8(v3.z));
        const vbool8 flags(0,0,0,0,1,1,1,1);
        return PlueckerIntersectorTriangle1::intersect(ray,vtx0,vtx1,vtx2,flags,epilog); 
      }
      
      __forceinline bool intersect(Ray& ray, const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, 
                                   const vint4& geomID, const vint4& primID, Scene* scene, const unsigned* geomID_to_instID) const
      {
        return intersect(ray,v0,v1,v2,v3,Intersect1Epilog<8,8,filter>(ray,vint8(geomID),vint8(primID),scene,geomID_to_instID));
      }
      
      __forceinline bool occluded(Ray& ray, const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, 
                                  const vint4& geomID, const vint4& primID, Scene* scene, const unsigned* geomID_to_instID) const
      {
        return intersect(ray,v0,v1,v2,v3,Occluded1Epilog<8,8,filter>(ray,vint8(geomID),vint8(primID),scene,geomID_to_instID));
      }
    };

#endif
  }
}

