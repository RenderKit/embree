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

#ifndef __EMBREE_TRIANGLE4I_INTERSECTOR1_PLUECKER_H__
#define __EMBREE_TRIANGLE4I_INTERSECTOR1_PLUECKER_H__

#include "triangle4i.h"
#include "common/ray.h"

namespace embree
{
  /*! Intersector1 for triangle4i */
  struct Triangle4iIntersector1Pluecker
  {
    typedef Triangle4i Primitive;

    static __forceinline void intersect(Ray& ray, const Triangle4i& tri, const void* geom)
    {
      /* gather vertices */
      STAT3(normal.trav_prims,1,1,1);
      const ssef* base0 = (const ssef*) tri.v0[0];
      const ssef* base1 = (const ssef*) tri.v0[1];
      const ssef* base2 = (const ssef*) tri.v0[2];
      const ssef* base3 = (const ssef*) tri.v0[3];
      sse3f p0; transpose(base0[        0],base1[        0],base2[        0],base3[        0],p0.x,p0.y,p0.z);
      sse3f p1; transpose(base0[tri.v1[0]],base1[tri.v1[1]],base2[tri.v1[2]],base3[tri.v1[3]],p1.x,p1.y,p1.z);
      sse3f p2; transpose(base0[tri.v2[0]],base1[tri.v2[1]],base2[tri.v2[2]],base3[tri.v2[3]],p2.x,p2.y,p2.z);

      /* calculate vertices relative to ray origin */
      const sse3f O = sse3f(ray.org);
      const sse3f D = sse3f(ray.dir);
      const sse3f v0 = p0-O;
      const sse3f v1 = p1-O;
      const sse3f v2 = p2-O;

      /* calculate triangle edges */
      const sse3f e0 = v2-v0;
      const sse3f e1 = v0-v1;
      const sse3f e2 = v1-v2;

      /* calculate geometry normal and denominator */
      const sse3f Ng = cross(e1,e0);
      const sse3f Ng2 = Ng+Ng;
      const ssef den = dot(Ng2,D);
      const ssef absDen = abs(den);
      const ssef sgnDen = signmsk(den);

      /* perform edge tests */
      const ssef U = dot(cross(v2+v0,e0),D) ^ sgnDen;
      const ssef V = dot(cross(v0+v1,e1),D) ^ sgnDen;
      const ssef W = dot(cross(v1+v2,e2),D) ^ sgnDen;
      sseb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
      if (unlikely(none(valid))) return;

      /* perform depth test */
      const ssef T = dot(v0,Ng2) ^ sgnDen;
      valid &= (T >= absDen*ssef(ray.tnear)) & (absDen*ssef(ray.tfar) >= T);
      if (unlikely(none(valid))) return;

      /* perform backface culling */
#if defined(__BACKFACE_CULLING__)
      valid &= den > ssef(zero);
      if (unlikely(none(valid))) return;
#else
      valid &= den != ssef(zero);
      if (unlikely(none(valid))) return;
#endif

      // FIXME: add ray mask

      /* update hit information */
      const ssef u = U / absDen;
      const ssef v = V / absDen;
      const ssef t = T / absDen;
      const size_t i = select_min(valid,t);
      ray.tfar = t[i];
      ray.u = u[i];
      ray.v = v[i];
      ray.Ng.x = Ng2.x[i];
      ray.Ng.y = Ng2.y[i];
      ray.Ng.z = Ng2.z[i];
      ray.geomID = tri.geomID[i];
      ray.primID = tri.primID[i];
    }

    static __forceinline void intersect(Ray& ray, const Triangle4i* tri, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
        intersect(ray,tri[i],geom);
    }

    static __forceinline bool occluded(const Ray& ray, const Triangle4i& tri, const void* geom)
    {
      /* gather vertices */
      STAT3(shadow.trav_prims,1,1,1);
      const ssef* base0 = (const ssef*) tri.v0[0];
      const ssef* base1 = (const ssef*) tri.v0[1];
      const ssef* base2 = (const ssef*) tri.v0[2];
      const ssef* base3 = (const ssef*) tri.v0[3];
      sse3f p0; transpose(base0[        0],base1[        0],base2[        0],base3[        0],p0.x,p0.y,p0.z);
      sse3f p1; transpose(base0[tri.v1[0]],base1[tri.v1[1]],base2[tri.v1[2]],base3[tri.v1[3]],p1.x,p1.y,p1.z);
      sse3f p2; transpose(base0[tri.v2[0]],base1[tri.v2[1]],base2[tri.v2[2]],base3[tri.v2[3]],p2.x,p2.y,p2.z);
      
      /* calculate vertices relative to ray origin */
      const sse3f O = sse3f(ray.org);
      const sse3f D = sse3f(ray.dir);
      const sse3f v0 = p0-O;
      const sse3f v1 = p1-O;
      const sse3f v2 = p2-O;

      /* calculate triangle edges */
      const sse3f e0 = v2-v0;
      const sse3f e1 = v0-v1;
      const sse3f e2 = v1-v2;

      /* calculate geometry normal and denominator */
      const sse3f Ng = cross(e1,e0);
      const sse3f Ng2 = Ng+Ng;
      const ssef den = dot(Ng2,D);
      const ssef absDen = abs(den);
      const ssef sgnDen = signmsk(den);

      /* perform edge tests */
      const ssef U = dot(cross(v2+v0,e0),D) ^ sgnDen;
      const ssef V = dot(cross(v0+v1,e1),D) ^ sgnDen;
      const ssef W = dot(cross(v1+v2,e2),D) ^ sgnDen;
      sseb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
      if (unlikely(none(valid))) return false;
      
      /* perform depth test */
      const ssef T = dot(v0,Ng2) ^ sgnDen;
      valid &= (T >= absDen*ssef(ray.tnear)) & (absDen*ssef(ray.tfar) >= T);
      if (unlikely(none(valid))) return false;

      /* perform backface culling */
#if defined(__BACKFACE_CULLING__)
      valid &= den > ssef(zero);
      if (unlikely(none(valid))) return false;
#else
      valid &= den != ssef(zero);
      if (unlikely(none(valid))) return false;
#endif

      // FIMXE: add ray mask

      return true;
    }

    static __forceinline bool occluded(Ray& ray, const Triangle4i* tri, size_t num, const void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (occluded(ray,tri[i],geom))
          return true;

      return false;
    }
  };
}

#endif


