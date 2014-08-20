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
#include "bezier_intersector8.h"

namespace embree
{
  /*! Intersector for a single ray from a ray packet with a bezier curve. */
  struct Bezier1iIntersector8
  {
    typedef Bezier1i Primitive;
    typedef BezierIntersector8::Precalculations Precalculations;

    static __forceinline void intersect(Precalculations& pre, Ray8& ray, const size_t k, const Bezier1i* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
        BezierIntersector8::intersect(pre,ray,k,curves[i].p[0],curves[i].p[1],curves[i].p[2],curves[i].p[3],curves[i].geomID,curves[i].primID,geom);
    }

    static __forceinline void intersect(const avxb& valid_i, Precalculations& pre, Ray8& ray, const Bezier1i* curves, size_t num, void* geom)
    {
      int mask = movemask(valid_i);
      while (mask) intersect(pre,ray,__bscf(mask),curves,num,geom);
    }

    static __forceinline bool occluded(Precalculations& pre, Ray8& ray, const size_t k, const Bezier1i* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (BezierIntersector8::occluded(pre,ray,k,curves[i].p[0],curves[i].p[1],curves[i].p[2],curves[i].p[3],curves[i].geomID,curves[i].primID,geom))
          return true;

      return false;
    }

    static __forceinline avxb occluded(const avxb& valid_i, Precalculations& pre, Ray8& ray, const Bezier1i* curves, size_t num, void* geom)
    {
      avxb valid_o = false;
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
