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

#include "subdivpatch1cached.h"
#include "common/ray.h"
#include "geometry/filter.h"

namespace embree
{

  struct SubdivPatch1CachedIntersector1
  {
    typedef SubdivPatch1Cached Primitive;

    struct Precalculations {
      Vec3fa ray_rdir, ray_org_rdir;

      __forceinline Precalculations (const Ray& ray) 
      {
	ray_rdir     = rcp_safe(ray.dir);
	ray_org_rdir = ray.org*ray_rdir;	
      }
    };

    static __forceinline bool intersectBounds(const Precalculations& pre,
					      const Ray& ray,
					      const BBox3fa &bounds)
    {
      Vec3fa b_lower = bounds.lower * pre.ray_rdir - pre.ray_org_rdir;
      Vec3fa b_upper = bounds.upper * pre.ray_rdir - pre.ray_org_rdir;
      Vec3fa b_min = min(b_lower,b_upper);
      Vec3fa b_max = max(b_lower,b_upper);
      const float tnear = max(b_min.x,b_min.y,b_min.z,ray.tnear);
      const float tfar = min(b_max.x,b_max.y,b_max.z,ray.tfar);
      return tnear <= tfar;
    }


    static void intersect_subdiv_patch(const Precalculations& pre,
                                       Ray& ray,
                                       const Primitive& subdiv_patch,
                                       const void* geom);

    static bool occluded_subdiv_patch(const Precalculations& pre,
                                      Ray& ray,
                                      const Primitive& subdiv_patch,
                                      const void* geom);


    /*! Intersect a ray with the primitive. */
    static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& subdiv_patch, const void* geom)
    {
      STAT3(normal.trav_prims,1,1,1);

      intersect_subdiv_patch(pre,ray,subdiv_patch,geom);

    }

    /*! Test if the ray is occluded by the primitive */
    static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& subdiv_patch, const void* geom)
    {
      STAT3(shadow.trav_prims,1,1,1);

      return occluded_subdiv_patch(pre,ray,subdiv_patch,geom);;
    }
  };
}
