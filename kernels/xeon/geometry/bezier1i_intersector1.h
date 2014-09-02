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

#include "bezier1i.h"
#include "bezier_intersector1.h"

namespace embree
{
  /*! Intersector for a single ray with a bezier curve. */
  struct Bezier1iIntersector1
  {
    typedef Bezier1i Primitive;
    typedef BezierIntersector1::Precalculations Precalculations;

    static __forceinline void intersect(Precalculations& pre, Ray& ray, const Bezier1i* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
        BezierIntersector1::intersect(ray,pre,curves[i].p[0],curves[i].p[1],curves[i].p[2],curves[i].p[3],curves[i].geomID,curves[i].primID,geom);
    }

    static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Bezier1i* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (BezierIntersector1::occluded(ray,pre,curves[i].p[0],curves[i].p[1],curves[i].p[2],curves[i].p[3],curves[i].geomID,curves[i].primID,geom))
          return true;

      return false;
    }
  };

  /*! Intersector for a single ray with a bezier curve. */
  struct Bezier1iIntersector1MB
  {
    typedef Bezier1iMB Primitive;
    typedef BezierIntersector1::Precalculations Precalculations;

    static __forceinline void intersect(Precalculations& pre, Ray& ray, const Bezier1iMB* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
      {
        const Vec3fa a0 = curves[i].p0[0], a1 = curves[i].p0[1], a2 = curves[i].p0[2], a3 = curves[i].p0[3];
        const Vec3fa b0 = curves[i].p1[0], b1 = curves[i].p1[1], b2 = curves[i].p1[2], b3 = curves[i].p1[3];
        const float t0 = 1.0f-ray.time, t1 = ray.time;
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        BezierIntersector1::intersect(ray,pre,p0,p1,p2,p3,curves[i].geomID,curves[i].primID,geom);
      }
    }

    static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Bezier1iMB* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
      {
        const Vec3fa a0 = curves[i].p0[0], a1 = curves[i].p0[1], a2 = curves[i].p0[2], a3 = curves[i].p0[3];
        const Vec3fa b0 = curves[i].p1[0], b1 = curves[i].p1[1], b2 = curves[i].p1[2], b3 = curves[i].p1[3];
        const float t0 = 1.0f-ray.time, t1 = ray.time;
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;

        if (BezierIntersector1::occluded(ray,pre,p0,p1,p2,p3,curves[i].geomID,curves[i].primID,geom))
          return true;
      }
      return false;
    }
  };
}
