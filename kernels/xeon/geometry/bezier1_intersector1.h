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

#include "bezier1.h"
#include "bezier_intersector1.h"

namespace embree
{
  /*! Intersector for a single ray with a bezier curve. */
  struct Bezier1Intersector1
  {
    typedef Bezier1 Primitive;
    typedef BezierIntersector1::Precalculations Precalculations;

    static __forceinline void intersect(Precalculations& pre, Ray& ray, const Bezier1* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
        BezierIntersector1::intersect(ray,pre,curves[i].p0,curves[i].p1,curves[i].p2,curves[i].p3,curves[i].geomID,curves[i].primID,geom);
    }

    static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Bezier1* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (BezierIntersector1::occluded(ray,pre,curves[i].p0,curves[i].p1,curves[i].p2,curves[i].p3,curves[i].geomID,curves[i].primID,geom))
          return true;

      return false;
    }
  };
}
