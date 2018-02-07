// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "subgrid.h"
#include "quad_intersector_moeller.h"
#include "quad_intersector_pluecker.h"

namespace embree
{
  namespace isa
  {
    /*! Intersects M quads with 1 ray */
    template<bool filter>
    struct SubGridIntersector1Moeller
    {
      typedef SubGrid Primitive;
      typedef QuadMIntersector1MoellerTrumbore<4,filter> Precalculations;

      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive& subgrid)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec3vf4 v0,v1,v2,v3; subgrid.gather(v0,v1,v2,v3,context->scene);
        pre.intersect(ray,context,v0,v1,v2,v3,subgrid.geomID(),subgrid.primID());
      }

      /*! Test if the ray is occluded by one of M subgrids. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& subgrid)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf4 v0,v1,v2,v3; subgrid.gather(v0,v1,v2,v3,context->scene);
        return pre.occluded(ray,context,v0,v1,v2,v3,subgrid.geomID(),subgrid.primID());
      }
    };

  }
}
