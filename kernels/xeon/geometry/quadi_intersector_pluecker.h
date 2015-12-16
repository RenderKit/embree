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

#include "quadi.h"
#include "quadi_mb.h"
#include "quad_intersector_pluecker.h"

namespace embree
{
  namespace isa
  {
    /*! Intersects M quads with 1 ray */
    template<int M, bool filter>
      struct QuadMiIntersector1Pluecker
    {
      typedef QuadMi<M> Primitive;
      typedef QuadMIntersector1Pluecker<M,filter> Precalculations;
        
      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec3<vfloat<M>> v0,v1,v2,v3; quad.gather(v0,v1,v2,v3,scene);
        pre.intersect(ray,v0,v1,v2,v3,quad.geomIDs,quad.primIDs,scene,geomID_to_instID); 
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3<vfloat<M>> v0,v1,v2,v3; quad.gather(v0,v1,v2,v3,scene);
        return pre.occluded(ray,v0,v1,v2,v3,quad.geomIDs,quad.primIDs,scene,geomID_to_instID);
      }
    };

    /*! Intersects 4 motion blur quads with 1 ray using SSE */
    template<int M, bool filter>
      struct QuadMiMBIntersector1Pluecker
    {
      typedef QuadMiMB<M> Primitive;
      typedef QuadMIntersector1Pluecker<M,filter> Precalculations;
        
      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec3vf4 v0,v1,v2,v3; quad.gather(v0,v1,v2,v3,scene,ray.time);
        pre.intersect(ray,v0,v1,v2,v3,quad.geomIDs,quad.primIDs,scene,geomID_to_instID); 
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf4 v0,v1,v2,v3; quad.gather(v0,v1,v2,v3,scene,ray.time);
        return pre.occluded(ray,v0,v1,v2,v3,quad.geomIDs,quad.primIDs,scene,geomID_to_instID); 
      }
    };
  }
}

