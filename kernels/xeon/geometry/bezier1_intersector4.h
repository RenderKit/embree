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
#include "bezier_intersector4.h"

namespace embree
{
  /*! Intersector for a single ray from a ray packet with a bezier curve. */
  struct Bezier1Intersector4
  {
    typedef Bezier1 Primitive;
    typedef BezierIntersector4::Precalculations Precalculations;

    static __forceinline void intersect(Precalculations& pre, Ray4& ray, const size_t k, const Bezier1* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) 
        BezierIntersector4::intersect(pre,ray,k,curves[i].p0,curves[i].p1,curves[i].p2,curves[i].p3,curves[i].geomID,curves[i].primID,geom);
    }

    static __forceinline void intersect(const sseb& valid_i, Precalculations& pre, Ray4& ray, const Bezier1* curves, size_t num, void* geom)
    {
      int mask = movemask(valid_i);
      while (mask) intersect(pre,ray,__bscf(mask),curves,num,geom);
    }

    static __forceinline bool occluded(Precalculations& pre, Ray4& ray, const size_t k, const Bezier1* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (BezierIntersector4::occluded(pre,ray,k,curves[i].p0,curves[i].p1,curves[i].p2,curves[i].p3,curves[i].geomID,curves[i].primID,geom))
          return true;

      return false;
    }

    static __forceinline sseb occluded(const sseb& valid_i, Precalculations& pre, Ray4& ray, const Bezier1* curves, size_t num, void* geom)
    {
      sseb valid_o = false;
      int mask = movemask(valid_i);
      while (mask) {
	size_t k = __bscf(mask);
	if (occluded(pre,ray,k,curves,num,geom))
	  valid_o[k] = -1;
      }
      return valid_o;
    }
  };
}
