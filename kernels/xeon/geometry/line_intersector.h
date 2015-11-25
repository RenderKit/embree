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
#include "../../common/globals.h"
#include "filter.h"

namespace embree
{
  namespace isa
  {
    template<int M>
    struct LineIntersector1
    {
      typedef Vec3<vfloat<M>> Vec3vfM;
      typedef Vec4<vfloat<M>> Vec4vfM;

      struct Precalculations
      {
        __forceinline Precalculations (const Ray& ray, const void *ptr)
          : depth_scale(rsqrt(dot(ray.dir,ray.dir))), ray_space(frame(depth_scale*ray.dir).transposed()) {}

        float depth_scale;
        LinearSpace3<Vec3vfM> ray_space;
      };

      static __forceinline void intersect(Ray& ray, const Precalculations& pre,
                                          const Vec4vfM& v0, const Vec4vfM& v1, const vint<M>& geomID, const vint<M>& primID,
                                          Scene* scene, const unsigned* geomID_to_instID)
      {
        /* transform control points into ray space */
        STAT3(normal.trav_prims,1,1,1);
        Vec3vfM w0 = xfmVector(pre.ray_space,Vec3vfM(v0.x,v0.y,v0.z)-Vec3vfM(ray.org));
        Vec3vfM w1 = xfmVector(pre.ray_space,Vec3vfM(v1.x,v1.y,v1.z)-Vec3vfM(ray.org));

        Vec4vfM p0(w0.x, w0.y, w0.z, v0.w);
        Vec4vfM p1(w1.x, w1.y, w1.z, v1.w);

        /* approximative intersection with cone */
        const Vec4vfM v = p1-p0;
        const Vec4vfM w = -p0;
        const vfloat<M> d0 = w.x*v.x + w.y*v.y;
        const vfloat<M> d1 = v.x*v.x + v.y*v.y;
        const vfloat<M> u = clamp(d0*rcp(d1),vfloat<M>(zero),vfloat<M>(one));
        const Vec4vfM p = p0 + u*v;
        const vfloat<M> t = p.z*pre.depth_scale;
        const vfloat<M> d2 = p.x*p.x + p.y*p.y;
        const vfloat<M> r = p.w;
        const vfloat<M> r2 = r*r;
        vbool<M> valid = d2 <= r2 & vfloat<M>(ray.tnear) < t & t < vfloat<M>(ray.tfar);

        //int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
      retry:
        if (unlikely(none(valid))) return;
        STAT3(normal.trav_prim_hits,1,1,1);
        size_t i = select_min(valid,t);

        /* ray masking test */
#if defined(RTCORE_RAY_MASK)
        BezierCurves* g = scene->getBezierCurves(geomID);
        if (unlikely(g->mask & ray.mask) == 0) return;
#endif

        /* intersection filter test */
        /*
#if defined(RTCORE_INTERSECTION_FILTER)
        Geometry* geometry = scene->get(geomID);
        if (!likely(geometry->hasIntersectionFilter1()))
        {
#endif
*/
          /* update hit information */
          Vec3fa T = Vec3fa(v1.x[i], v1.y[i], v1.z[i]) - Vec3fa(v0.x[i], v0.y[i], v0.z[i]);
          if (T == Vec3fa(zero)) { valid[i] = 0; goto retry; } // ignore denormalized curves
          ray.u = u[i];
          ray.v = 0.0f;
          ray.tfar = t[i];
          ray.Ng = T;
          ray.geomID = geomID[i];
          ray.primID = primID[i];
          /*
#if defined(RTCORE_INTERSECTION_FILTER)
          return;
        }

        while (true)
        {
          const float uu = (float(i)+u[i])*one_over_width;
          const BezierCurve3fa curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
          Vec3fa P,T; curve3D.eval(uu,P,T);
          if (T != Vec3fa(zero))
            if (runIntersectionFilter1(geometry,ray,uu,0.0f,t[i],T,instID,primID)) return;
          valid[i] = 0;
          if (none(valid)) return;
          i = select_min(valid,t);
          STAT3(normal.trav_prim_hits,1,1,1);
        }
#endif
        */
      }
    };
  }
}
