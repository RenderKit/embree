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

#include "../../common/ray.h"
#include "filter.h"

namespace embree
{
  namespace isa
  {
    template<int M, bool enableIntersectionFilter>
      struct Intersect1Epilog
      {
        Ray& ray;
        const vint<M>& tri_geomIDs;
        const vint<M>& tri_primIDs;
        Scene* scene;
        const unsigned* geomID_to_instID;

        __forceinline Intersect1Epilog(Ray& ray,
                                          const vint<M>& tri_geomIDs, 
                                          const vint<M>& tri_primIDs, 
                                          Scene* scene,
                                          const unsigned* geomID_to_instID)
          : ray(ray), tri_geomIDs(tri_geomIDs), tri_primIDs(tri_primIDs), scene(scene), geomID_to_instID(geomID_to_instID) {}

        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          vfloat<M> u, v, t; 
          Vec3<vfloat<M>> tri_Ng;
          std::tie(u,v,t,tri_Ng) = hit();
          vbool<M> valid = valid_i;

          size_t i = select_min(valid,t);
          int geomID = tri_geomIDs[i];
          int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          goto entry;
          while (true) 
          {
            if (unlikely(none(valid))) return false;
            i = select_min(valid,t);
            geomID = tri_geomIDs[i];
            instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
          entry:
            Geometry* geometry = scene->get(geomID);
            
#if defined(RTCORE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
              valid[i] = 0;
              continue;
            }
#endif
            
#if defined(RTCORE_INTERSECTION_FILTER) 
            /* call intersection filter function */
            if (enableIntersectionFilter) {
              if (unlikely(geometry->hasIntersectionFilter1())) {
                Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
                if (runIntersectionFilter1(geometry,ray,u[i],v[i],t[i],Ng,instID,tri_primIDs[i])) return true;
                valid[i] = 0;
                continue;
              }
            }
#endif
            break;
          }
#endif
          
          /* update hit information */
          ray.u = u[i];
          ray.v = v[i];
          ray.tfar = t[i];
          ray.Ng.x = tri_Ng.x[i];
          ray.Ng.y = tri_Ng.y[i];
          ray.Ng.z = tri_Ng.z[i];
          ray.geomID = instID;
          ray.primID = tri_primIDs[i];
          return true;
        }
      };


    template<int M, bool enableIntersectionFilter>
      struct Occluded1Epilog
      {
        Ray& ray;
        const vint<M>& tri_geomIDs;
        const vint<M>& tri_primIDs;
        Scene* scene;
        const unsigned* geomID_to_instID;

        __forceinline Occluded1Epilog(Ray& ray,
                                         const vint<M>& tri_geomIDs, 
                                         const vint<M>& tri_primIDs, 
                                         Scene* scene,
                                         const unsigned* geomID_to_instID)
          : ray(ray), tri_geomIDs(tri_geomIDs), tri_primIDs(tri_primIDs), scene(scene), geomID_to_instID(geomID_to_instID) {}

        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid, const Hit& hit) const
        {
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          size_t m=movemask(valid);
          goto entry;
          while (true)
          {  
            if (unlikely(m == 0)) return false;
          entry:
            size_t i=__bsf(m);
            const int geomID = tri_geomIDs[i];
            const int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
            Geometry* geometry = scene->get(geomID);
            
#if defined(RTCORE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
              m=__btc(m,i);
              continue;
            }
#endif
            
#if defined(RTCORE_INTERSECTION_FILTER)
            /* if we have no filter then the test passed */
            if (enableIntersectionFilter) {
              if (unlikely(geometry->hasOcclusionFilter1())) 
              {
                Vec3<vfloat<M>> tri_Ng;
                vfloat<M> u,v,t; std::tie(u,v,t,tri_Ng) = hit();
                const Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
                if (runOcclusionFilter1(geometry,ray,u[i],v[i],t[i],Ng,instID,tri_primIDs[i])) return true;
                m=__btc(m,i);
                continue;
              }
            }
#endif
            break;
          }
#endif
          
          return true;
        }
      };
  }
}
