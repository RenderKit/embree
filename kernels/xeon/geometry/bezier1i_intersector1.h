// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#ifndef __EMBREE_ACCEL_BEZIER1I_INTERSECTOR1_MOELLER_H__
#define __EMBREE_ACCEL_BEZIER1I_INTERSECTOR1_MOELLER_H__

#include "bezier1i.h"
#include "common/ray.h"
#include "geometry/filter.h"

namespace embree
{
  struct BezierCurve3D
  {
    Vec3fa v0,v1,v2,v3;
    int depth;

    __forceinline BezierCurve3D() {}

    __forceinline BezierCurve3D(const Vec3fa& v0, 
                                const Vec3fa& v1, 
                                const Vec3fa& v2, 
                                const Vec3fa& v3,
                                int depth)
      : v0(v0), v1(v1), v2(v2), v3(v3), depth(depth) {}

    __forceinline const BBox3f bounds() const {
      return merge(BBox3f(v0),BBox3f(v1),BBox3f(v2),BBox3f(v3));
    }

    __forceinline void subdivide(BezierCurve3D& left, BezierCurve3D& right) const
    {
      const Vec3fa p00 = v0;
      const Vec3fa p01 = v1;
      const Vec3fa p02 = v2;
      const Vec3fa p03 = v3;

      const Vec3fa p10 = (p00 + p01) * 0.5f;
      const Vec3fa p11 = (p01 + p02) * 0.5f;
      const Vec3fa p12 = (p02 + p03) * 0.5f;
      const Vec3fa p20 = (p10 + p11) * 0.5f;
      const Vec3fa p21 = (p11 + p12) * 0.5f;
      const Vec3fa p30 = (p20 + p21) * 0.5f;

      left.v0 = p00;
      left.v1 = p10;
      left.v2 = p20;
      left.v3 = p30;
      left.depth = depth-1;
        
      right.v0 = p30;
      right.v1 = p21;
      right.v2 = p12;
      right.v3 = p03;
      right.depth = depth-1;
    }

    friend inline std::ostream& operator<<(std::ostream& cout, const BezierCurve3D& curve) {
      return cout << "{ v0 = " << curve.v0 << ", v1 = " << curve.v1 << ", v2 = " << curve.v2 << ", v3 = " << curve.v3 << ", depth = " << curve.depth << " }";
    }
  };

  __forceinline bool intersect_box(const BBox3f& box, const Ray& ray, float& dist)
  {
    const float clipNearX = (box.lower.x - ray.org.x) / ray.dir.x; // FIXME: use rdir
    const float clipNearY = (box.lower.y - ray.org.y) / ray.dir.y; // FIXME: use SSE for intersection
    const float clipNearZ = (box.lower.z - ray.org.z) / ray.dir.z;
    const float clipFarX = (box.upper.x - ray.org.x) / ray.dir.x;
    const float clipFarY = (box.upper.y - ray.org.y) / ray.dir.y;
    const float clipFarZ = (box.upper.z - ray.org.z) / ray.dir.z;

    const float near = max(max(min(clipNearX, clipFarX), min(clipNearY, clipFarY)), min(clipNearZ, clipFarZ));
    const float far   = min(min(max(clipNearX, clipFarX), max(clipNearY, clipFarY)), max(clipNearZ, clipFarZ));
    const bool hit    = max(near,ray.tnear) <= min(far,ray.tfar);
    dist = near;
    return hit;
  }

  /*! Intersector for a single ray with a bezier curve. */
  struct Bezier1iIntersector1
  {
    typedef Bezier1i Primitive;

    static __forceinline void intersect(Ray& ray, const Bezier1i& curve_in, const void* geom)
    {
      //PING;

      /* load bezier curve control points */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa v0 = curve_in.p[0];
      const Vec3fa v1 = curve_in.p[1];
      const Vec3fa v2 = curve_in.p[2];
      const Vec3fa v3 = curve_in.p[3];
      //PRINT(v0);
      //PRINT(v1);
      //PRINT(v2);
      //PRINT(v3);
      LinearSpace3f ray_space = rcp(frame(normalize(ray.dir)));
      Vec3fa w0 = xfmVector(ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(ray_space,v3-ray.org); w3.w = v3.w;
      
      //BezierCurve3D curve(w0,w1,w2,w3,0);
      BezierCurve3D curve(w0,w1,w2,w3,0);
      //float t = curve.intersect(ray);

      const Vec3fa v = curve.v3-curve.v0;
      const Vec3fa w = -curve.v0;
      const float d0 = w.x*v.x + w.y*v.y;
      const float d1 = v.x*v.x + v.y*v.y;
      const float b = clamp(d0/d1,0.0f,1.0f);
      const Vec3fa d = curve.v0 + b*v;
      const float dist = sqrt(d.x*d.x + d.y*d.y); 
      if (unlikely(dist > d.w)) return;
      const float t = d.z/length(ray.dir);
      if (unlikely(t < ray.tnear || t > ray.tfar)) return;
      ray.u = 0.0f;
      ray.v = 0.0f;
      ray.tfar = t;
      ray.Ng = Vec3fa(zero);
      ray.geomID = curve_in.geomID;
      ray.primID = curve_in.primID;
      
#if 0
      /* push first curve onto stack */
      BezierCurve3D stack[32];
      new (&stack[0]) BezierCurve3D(v0,v1,v2,v3,4);
      size_t sptr = 1;

      while (true) 
      {
      pop:
        if (sptr == 0) break;
        BezierCurve3D curve = stack[--sptr];
        //PRINT(sptr);
      
        while (curve.depth)
        {
          //PRINT(curve);
          BezierCurve3D curve0,curve1;
          curve.subdivide(curve0,curve1);

          BBox3f bounds0 = curve0.bounds();
          BBox3f bounds1 = curve1.bounds();
          float dist0; bool hit0 = intersect_box(bounds0,ray,dist0);
          float dist1; bool hit1 = intersect_box(bounds1,ray,dist1);
          
          if (!hit0 && !hit1)
            goto pop;
        
          if (likely(hit0 != hit1))
          {
            if (hit0) {
              curve = curve0;
              continue;
            } else {
              curve = curve1;
              continue;
            }
          }
          else 
          {
            if (dist0 < dist1) {
              curve = curve0;
              stack[sptr++] = curve1;
            } else {
              curve = curve1;
              stack[sptr++] = curve0;
            }
          }
        }

        /* intersect with line */
        //PRINT("leaf");
        float t = curve.intersect(ray);
        if (t < 0) continue;

        /* update hit information */
        ray.u = 0.0f;
        ray.v = 0.0f;
        ray.tfar = t;
        ray.Ng = Vec3fa(zero);
        ray.geomID = curve_in.geomID;
        ray.primID = curve_in.primID;
      }

      //PING;
#endif
    }

    static __forceinline void intersect(Ray& ray, const Bezier1i* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
        intersect(ray,curves[i],geom);
    }

    static __forceinline bool occluded(Ray& ray, const Bezier1i& curve_in, const void* geom) {
      return false;
    }

    static __forceinline bool occluded(Ray& ray, const Bezier1i* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (occluded(ray,curves[i],geom))
          return true;

      return false;
    }
  };
}

#endif


