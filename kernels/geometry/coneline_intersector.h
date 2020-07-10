// ======================================================================== //
// Copyright 2009-2020 Intel Corporation                                    //
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

#include "../common/ray.h"
#include "curve_intersector_precalculations.h"

namespace embree
{
  namespace isa
  {
    template<int M>
      struct ConeLineIntersectorHitM
      {
        __forceinline ConeLineIntersectorHitM() {}
        
        __forceinline ConeLineIntersectorHitM(const vfloat<M>& u, const vfloat<M>& v, const vfloat<M>& t, const Vec3vf<M>& Ng)
          : vu(u), vv(v), vt(t), vNg(Ng) {}
	
        __forceinline void finalize() {}
	
        __forceinline Vec2f uv (const size_t i) const { return Vec2f(vu[i],vv[i]); }
        __forceinline float t  (const size_t i) const { return vt[i]; }
        __forceinline Vec3fa Ng(const size_t i) const { return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]); }
	
      public:
        vfloat<M> vu;
        vfloat<M> vv;
        vfloat<M> vt;
        Vec3vf<M> vNg;
      };
    
    template<int M>
      struct ConeCurveIntersector1
      {
        typedef CurvePrecalculations1 Precalculations;
        
        struct ray_tfar {
          Ray& ray;
          __forceinline ray_tfar(Ray& ray) : ray(ray) {}
          __forceinline vfloat<M> operator() () const { return ray.tfar; };
        };

        template<typename Epilog>
        static __forceinline bool intersect(const vbool<M>& valid_i,
                                            Ray& ray,
                                            IntersectContext* context,
                                            const LineSegments* geom,
                                            const Precalculations& pre,
                                            const Vec4vf<M>& v0i, const Vec4vf<M>& v1i,
                                            const Vec4vf<M>& vLi, const Vec4vf<M>& vRi,
                                            const Epilog& epilog)
        {
          const Vec3vf<M> ray_org(ray.org.x, ray.org.y, ray.org.z);
          const Vec3vf<M> ray_dir(ray.dir.x, ray.dir.y, ray.dir.z);
          const vfloat<M> ray_tnear(ray.tnear());
          const Vec4vf<M> v0 = enlargeRadiusToMinWidth(context,geom,ray_org,v0i);
          const Vec4vf<M> v1 = enlargeRadiusToMinWidth(context,geom,ray_org,v1i);
          const Vec4vf<M> vL = enlargeRadiusToMinWidth(context,geom,ray_org,vLi);
          const Vec4vf<M> vR = enlargeRadiusToMinWidth(context,geom,ray_org,vRi);
          return false;
        //   return  __roundline_internal::intersectConeSphere(valid_i,ray_org,ray_dir,ray_tnear,ray_tfar(ray),v0,v1,vL,vR,epilog);
        }
      };
    
    template<int M, int K>
      struct ConeCurveIntersectorK
      {
        typedef CurvePrecalculationsK<K> Precalculations;
        
        struct ray_tfar {
          RayK<K>& ray;
          size_t k;
          __forceinline ray_tfar(RayK<K>& ray, size_t k) : ray(ray), k(k) {}
          __forceinline vfloat<M> operator() () const { return ray.tfar[k]; };
        };
        
        template<typename Epilog>
        static __forceinline bool intersect(const vbool<M>& valid_i,
                                            RayK<K>& ray, size_t k,
                                            IntersectContext* context,
                                            const LineSegments* geom,
                                            const Precalculations& pre,
                                            const Vec4vf<M>& v0i, const Vec4vf<M>& v1i,
                                            const Vec4vf<M>& vLi, const Vec4vf<M>& vRi,
                                            const Epilog& epilog)
        {
          const Vec3vf<M> ray_org(ray.org.x[k], ray.org.y[k], ray.org.z[k]);
          const Vec3vf<M> ray_dir(ray.dir.x[k], ray.dir.y[k], ray.dir.z[k]);
          const vfloat<M> ray_tnear = ray.tnear()[k];
          const Vec4vf<M> v0 = enlargeRadiusToMinWidth(context,geom,ray_org,v0i);
          const Vec4vf<M> v1 = enlargeRadiusToMinWidth(context,geom,ray_org,v1i);
          const Vec4vf<M> vL = enlargeRadiusToMinWidth(context,geom,ray_org,vLi);
          const Vec4vf<M> vR = enlargeRadiusToMinWidth(context,geom,ray_org,vRi);
          return false;
        //   return __roundline_internal::intersectConeSphere(valid_i,ray_org,ray_dir,ray_tnear,ray_tfar(ray,k),v0,v1,vL,vR,epilog);
        }
      };
  }
}
