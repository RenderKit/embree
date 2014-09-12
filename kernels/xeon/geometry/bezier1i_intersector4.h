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
#include "bezier_intersector4.h"

namespace embree
{
  /*! Intersector for a single ray from a ray packet with a bezier curve. */
  struct Bezier1iIntersector4
  {
    typedef Bezier1i Primitive;
    typedef BezierIntersector4::Precalculations Precalculations;

    static __forceinline void intersect(Precalculations& pre, Ray4& ray, const size_t k, const Bezier1i* curve, size_t num, void* geom)
    {
      while (true) {
        BezierIntersector4::intersect(pre,ray,k,curve->p[0],curve->p[1],curve->p[2],curve->p[3],curve->geomID(),curve->primID(),geom);
	if (curve->last()) break;
	curve++;
      }
    }

    static __forceinline void intersect(const sseb& valid_i, Precalculations& pre, Ray4& ray, const Bezier1i* curves, size_t num, void* geom)
    {
      int mask = movemask(valid_i);
      while (mask) intersect(pre,ray,__bscf(mask),curves,num,geom);
    }

    static __forceinline bool occluded(Precalculations& pre, Ray4& ray, const size_t k, const Bezier1i* curve, size_t num, void* geom) 
    {
      while (true) {
        if (BezierIntersector4::occluded(pre,ray,k,curve->p[0],curve->p[1],curve->p[2],curve->p[3],curve->geomID(),curve->primID(),geom))
          return true;

	if (curve->last()) break;
	curve++;
      }
      return false;
    }

    static __forceinline sseb occluded(const sseb& valid_i, Precalculations& pre, Ray4& ray, const Bezier1i* curve, size_t num, void* geom)
    {
      sseb valid_o = false;
      int mask = movemask(valid_i);
      while (mask) {
	size_t k = __bscf(mask);
	if (occluded(pre,ray,k,curve,num,geom))
	  valid_o[k] = -1;
      }
      return valid_o;
    }
  };
}
