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
    template<int M>
      struct UVIdentity
      {
        __forceinline void operator() (vfloat<M>& u, vfloat<M>& v) const {
        }
      };

    template<int M, bool filter>
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
            if (filter) {
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


    template<int M, bool filter>
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
            if (filter) {
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

    template<int M, bool filter>
      struct Intersect1EpilogU
      {
        Ray& ray;
        const unsigned int geomID;
        const unsigned int primID;
        Scene* scene;
        const unsigned* geomID_to_instID;
        
        __forceinline Intersect1EpilogU(Ray& ray,
                                        const unsigned int geomID, 
                                        const unsigned int primID, 
                                        Scene* scene,
                                        const unsigned* geomID_to_instID)
          : ray(ray), geomID(geomID), primID(primID), scene(scene), geomID_to_instID(geomID_to_instID) {}
        
        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          /* ray mask test */
          Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
          if ((geometry->mask & ray.mask) == 0) return false;
#endif
          
          vfloat<M> u, v, t; 
          Vec3<vfloat<M>> tri_Ng;
          std::tie(u,v,t,tri_Ng) = hit();
          vbool<M> valid = valid_i;
          
          size_t i = select_min(valid,t);
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (unlikely(geometry->hasIntersectionFilter1())) 
          {
            while (true) 
            {
              /* call intersection filter function */
              Vec3fa Ng_i = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
              if (runIntersectionFilter1(geometry,ray,u[i],v[i],t[i],Ng_i,geomID,primID)) {
                return true;
              }
              valid[i] = 0;
              if (unlikely(none(valid))) return false;
              i = select_min(valid,t);
            }
            return false; // FIXME: not reachable?
          }
#endif
          
          /* update hit information */
          ray.u         = u[i];
          ray.v         = v[i];
          ray.tfar      = t[i];
          ray.geomID    = geomID;
          ray.primID    = primID;
          ray.Ng.x      = tri_Ng.x[i];
          ray.Ng.y      = tri_Ng.y[i];
          ray.Ng.z      = tri_Ng.z[i];
          return true;
        }
      };
        
    template<int M, bool filter>
      struct Occluded1EpilogU
      {
        Ray& ray;
        const unsigned int geomID;
        const unsigned int primID;
        Scene* scene;
        const unsigned* geomID_to_instID;

        __forceinline Occluded1EpilogU(Ray& ray,
                                       const unsigned int geomID, 
                                       const unsigned int primID, 
                                       Scene* scene,
                                       const unsigned* geomID_to_instID)
          : ray(ray), geomID(geomID), primID(primID), scene(scene), geomID_to_instID(geomID_to_instID) {}

        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid, const Hit& hit) const
        {
           /* ray mask test */
        Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
        if ((geometry->mask & ray.mask) == 0) return false;
#endif
        
        /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
        if (unlikely(geometry->hasOcclusionFilter1())) 
        {
          vfloat<M> u, v, t; 
          Vec3<vfloat<M>> tri_Ng;
          std::tie(u,v,t,tri_Ng) = hit();
          
          for (size_t m=movemask(valid), i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {  
            const Vec3fa Ng_i = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
            if (runOcclusionFilter1(geometry,ray,u[i],v[i],t[i],Ng_i,geomID,primID)) return true;
          }
          return false;
        }
#endif
        return true;
        }
      };
        
        
        
        




    template<int M, int K, bool filter>
      struct IntersectKEpilog
      {
        RayK<K>& ray;
        const vint<M>& tri_geomIDs;
        const vint<M>& tri_primIDs;
        const int i;
        Scene* const scene;

        __forceinline IntersectKEpilog(RayK<K>& ray,
                                       const vint<M>& tri_geomIDs, 
                                       const vint<M>& tri_primIDs, 
                                       int i,
                                       Scene* scene)
          : ray(ray), tri_geomIDs(tri_geomIDs), tri_primIDs(tri_primIDs), i(i), scene(scene) {}

        template<typename Hit>
        __forceinline vbool<K> operator() (const vbool<K>& valid_i, const Hit& hit) const
        {
          vfloat<K> u, v, t; 
          Vec3<vfloat<K>> tri_Ng;
          std::tie(u,v,t,tri_Ng) = hit();
          vbool<K> valid = valid_i;

          const int geomID = tri_geomIDs[i];
          const int primID = tri_primIDs[i];
          Geometry* geometry = scene->get(geomID);
          
          /* ray masking test */
#if defined(RTCORE_RAY_MASK)
          valid &= (geometry->mask & ray.mask) != 0;
          if (unlikely(none(valid))) return false;
#endif
          
          /* occlusion filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (filter) {
            if (unlikely(geometry->hasIntersectionFilter<vfloat<K>>())) {
              return runIntersectionFilter(valid,geometry,ray,u,v,t,tri_Ng,geomID,primID);
            }
          }
#endif
          
          /* update hit information */
          vfloat<K>::store(valid,&ray.u,u);
          vfloat<K>::store(valid,&ray.v,v);
          vfloat<K>::store(valid,&ray.tfar,t);
          vint<K>::store(valid,&ray.geomID,geomID);
          vint<K>::store(valid,&ray.primID,primID);
          vfloat<K>::store(valid,&ray.Ng.x,tri_Ng.x);
          vfloat<K>::store(valid,&ray.Ng.y,tri_Ng.y);
          vfloat<K>::store(valid,&ray.Ng.z,tri_Ng.z);
          return valid;
        }
      };

    template<int M, int K, bool filter>
      struct OccludedKEpilog
      {
        vbool<K>& valid0;
        RayK<K>& ray;
        const vint<M>& tri_geomIDs;
        const vint<M>& tri_primIDs;
        const int i;
        Scene* const scene;

        __forceinline OccludedKEpilog(vbool<K>& valid0,
                                       RayK<K>& ray,
                                       const vint<M>& tri_geomIDs, 
                                       const vint<M>& tri_primIDs, 
                                       int i,
                                       Scene* scene)
          : valid0(valid0), ray(ray), tri_geomIDs(tri_geomIDs), tri_primIDs(tri_primIDs), i(i), scene(scene) {}

        template<typename Hit>
        __forceinline vbool<K> operator() (const vbool<K>& valid_i, const Hit& hit) const
        {
          vbool<K> valid = valid_i;
      
          /* ray masking test */
          const int geomID = tri_geomIDs[i];
          Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
          valid &= (geometry->mask & ray.mask) != 0;
          if (unlikely(none(valid))) return;
#endif
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (filter) 
          {
            if (unlikely(geometry->hasOcclusionFilter<vfloat<K>>()))
            {
              vfloat<K> u, v, t; 
              Vec3<vfloat<K>> tri_Ng;
              std::tie(u,v,t,tri_Ng) = hit();
              const int primID = tri_primIDs[i];
              valid = runOcclusionFilter(valid,geometry,ray,u,v,t,tri_Ng,geomID,primID);
            }
          }
#endif
          
          /* update occlusion */
          valid0 &= !valid;
          return valid;
        }
      };

    template<int M, int K, bool filter>
      struct IntersectKEpilogU
      {
        RayK<K>& ray;
        const unsigned int geomID;
        const unsigned int primID;
        Scene* const scene;

        __forceinline IntersectKEpilogU(RayK<K>& ray,
                                        const unsigned int geomID, 
                                        const unsigned int primID, 
                                        Scene* scene)
          : ray(ray), geomID(geomID), primID(primID), scene(scene) {}

        template<typename Hit>
        __forceinline vbool<K> operator() (const vbool<K>& valid, const Hit& hit) const
        {
          vfloat<K> u, v, t; 
          Vec3<vfloat<K>> Ng;
          std::tie(u,v,t,Ng) = hit();
          
          Geometry* geometry = scene->get(geomID);
          
          /* ray masking test */
#if defined(RTCORE_RAY_MASK)
          valid &= (geometry->mask & ray.mask) != 0;
          if (unlikely(none(valid))) return false;
#endif
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (filter) {
            if (unlikely(geometry->hasIntersectionFilter<vfloat<K>>())) {
              return runIntersectionFilter(valid,geometry,ray,u,v,t,Ng,geomID,primID);
            }
          }
#endif
          
          /* update hit information */
          vfloat<K>::store(valid,&ray.u,u);
          vfloat<K>::store(valid,&ray.v,v);
          vfloat<K>::store(valid,&ray.tfar,t);
          vint<K>::store(valid,&ray.geomID,geomID);
          vint<K>::store(valid,&ray.primID,primID);
          vfloat<K>::store(valid,&ray.Ng.x,Ng.x);
          vfloat<K>::store(valid,&ray.Ng.y,Ng.y);
          vfloat<K>::store(valid,&ray.Ng.z,Ng.z);
          return valid;
        }
      };

     template<int M, int K, bool filter>
      struct OccludedKEpilogU
      {
        vbool<K>& valid0;
        RayK<K>& ray;
        const unsigned int geomID;
        const unsigned int primID;
        Scene* const scene;

        __forceinline OccludedKEpilogU(vbool<K>& valid0,
                                       RayK<K>& ray,
                                       const unsigned int geomID, 
                                       const unsigned int primID, 
                                       Scene* scene)
          : valid0(valid0), ray(ray), geomID(geomID), primID(primID), scene(scene) {}

        template<typename Hit>
        __forceinline vbool<K> operator() (const vbool<K>& valid_i, const Hit& hit) const
        {
          vbool<K> valid = valid_i;
          Geometry* geometry = scene->get(geomID);

#if defined(RTCORE_RAY_MASK)
          valid &= (geometry->mask & ray.mask) != 0;
          if (unlikely(none(valid))) return false;
#endif
          
          /* occlusion filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (filter) {
            if (unlikely(geometry->hasOcclusionFilter<vfloat<K>>()))
            {
              vfloat<K> u, v, t; 
              Vec3<vfloat<K>> Ng;
              std::tie(u,v,t,Ng) = hit();
              valid = runOcclusionFilter(valid,geometry,ray,u,v,t,Ng,geomID,primID);
            }
          }
#endif
          
          /* update occlusion */
          valid0 &= !valid;
          return valid;
        }
      };



     template<int M, int K, bool filter>
      struct Intersect1KEpilog
      {
        RayK<K>& ray;
        int k;
        const vint<M>& tri_geomIDs;
        const vint<M>& tri_primIDs;
        Scene* const scene;

        __forceinline Intersect1KEpilog(RayK<K>& ray, int k,
                                       const vint<M>& tri_geomIDs, 
                                       const vint<M>& tri_primIDs, 
                                       Scene* scene)
          : ray(ray), k(k), tri_geomIDs(tri_geomIDs), tri_primIDs(tri_primIDs), scene(scene) {}

        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          vfloat<M> u, v, t; 
          Vec3<vfloat<M>> tri_Ng;
          std::tie(u,v,t,tri_Ng) = hit();
          vbool<M> valid = valid_i;

          size_t i = select_min(valid,t);
          int geomID = tri_geomIDs[i];
      
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          goto entry;
          while (true) 
          {
            if (unlikely(none(valid))) return false;
            i = select_min(valid,t);
            geomID = tri_geomIDs[i];
          entry:
            Geometry* geometry = scene->get(geomID);
            
#if defined(RTCORE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask[k]) == 0) {
              valid[i] = 0;
              continue;
            }
#endif
            
#if defined(RTCORE_INTERSECTION_FILTER) 
            /* call intersection filter function */
            if (filter) {
              if (unlikely(geometry->hasIntersectionFilter<vfloat<K>>())) {
                Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
                if (runIntersectionFilter(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,tri_primIDs[i])) return true;
                valid[i] = 0;
                continue;
              }
            }
#endif
            break;
          }
#endif
          
          /* update hit information */
          ray.u[k] = u[i];
          ray.v[k] = v[i];
          ray.tfar[k] = t[i];
          ray.Ng.x[k] = tri_Ng.x[i];
          ray.Ng.y[k] = tri_Ng.y[i];
          ray.Ng.z[k] = tri_Ng.z[i];
          ray.geomID[k] = geomID;
          ray.primID[k] = tri_primIDs[i];
          return true;
        }
      };
      
      template<int M, int K, bool filter>
        struct Occluded1KEpilog
        {
          RayK<K>& ray;
          int k;
          const vint<M>& tri_geomIDs;
          const vint<M>& tri_primIDs;
          Scene* const scene;
          
          __forceinline Occluded1KEpilog(RayK<K>& ray, int k,
                                        const vint<M>& tri_geomIDs, 
                                        const vint<M>& tri_primIDs, 
                                        Scene* scene)
            : ray(ray), k(k), tri_geomIDs(tri_geomIDs), tri_primIDs(tri_primIDs), scene(scene) {}
          
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
              Geometry* geometry = scene->get(geomID);
              
#if defined(RTCORE_RAY_MASK)
              /* goto next hit if mask test fails */
              if ((geometry->mask & ray.mask[k]) == 0) {
                m=__btc(m,i);
                continue;
              }
#endif
              
#if defined(RTCORE_INTERSECTION_FILTER)
              /* execute occlusion filer */
              if (filter) {
                if (unlikely(geometry->hasOcclusionFilter<vfloat<K>>())) 
                {
                  Vec3<vfloat<M>> tri_Ng;
                  vfloat<M> u,v,t; std::tie(u,v,t,tri_Ng) = hit();
                  const Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
                  if (runOcclusionFilter(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,tri_primIDs[i])) return true;
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

      template<int M, int K, bool filter>
        struct Intersect1KEpilogU // FIXME: simplify
      {
        RayK<K>& ray;
        int k;
        const unsigned int geomID;
        const unsigned int primID;
        Scene* const scene;

        __forceinline Intersect1KEpilogU(RayK<K>& ray, int k,
                                         const unsigned int geomID, 
                                         const unsigned int primID, 
                                         Scene* scene)
          : ray(ray), k(k), geomID(geomID), primID(primID), scene(scene) {}

        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          vfloat<M> u, v, t; 
          Vec3<vfloat<M>> tri_Ng;
          std::tie(u,v,t,tri_Ng) = hit();
          vbool<M> valid = valid_i;
          
          size_t i = select_min(valid,t);
          //int geomID = tri_geomIDs[i];
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          goto entry;
          while (true) 
          {
            if (unlikely(none(valid))) return false;
            i = select_min(valid,t);
            //geomID = tri_geomIDs[i];
          entry:
            Geometry* geometry = scene->get(geomID);
            
#if defined(RTCORE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask[k]) == 0) {
              valid[i] = 0;
              continue;
            }
#endif
            
#if defined(RTCORE_INTERSECTION_FILTER) 
            /* call intersection filter function */
            if (filter) {
              if (unlikely(geometry->hasIntersectionFilter<vfloat<K>>())) {
                Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
                if (runIntersectionFilter(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,primID)) return true;
                valid[i] = 0;
                continue;
              }
            }
#endif
            break;
          }
#endif
          
          /* update hit information */
          ray.u[k] = u[i];
          ray.v[k] = v[i];
          ray.tfar[k] = t[i];
          ray.Ng.x[k] = tri_Ng.x[i];
          ray.Ng.y[k] = tri_Ng.y[i];
          ray.Ng.z[k] = tri_Ng.z[i];
          ray.geomID[k] = geomID;
          ray.primID[k] = primID;
          return true;
        }
      };

      template<int M, int K, bool filter>
        struct Occluded1KEpilogU // FIXME: simplify
        {
          RayK<K>& ray;
          int k;
          const unsigned int geomID;
          const unsigned int primID;
          Scene* const scene;
          
          __forceinline Occluded1KEpilogU(RayK<K>& ray, int k,
                                          const unsigned int geomID, 
                                          const unsigned int primID, 
                                          Scene* scene)
            : ray(ray), k(k), geomID(geomID), primID(primID), scene(scene) {}
          
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
              //const int geomID = tri_geomIDs[i];
              Geometry* geometry = scene->get(geomID);
              
#if defined(RTCORE_RAY_MASK)
              /* goto next hit if mask test fails */
              if ((geometry->mask & ray.mask[k]) == 0) {
                m=__btc(m,i);
                continue;
              }
#endif
              
#if defined(RTCORE_INTERSECTION_FILTER)
              /* execute occlusion filer */
              if (filter) {
                if (unlikely(geometry->hasOcclusionFilter<vfloat<K>>())) 
                {
                  vfloat<M> u, v, t; 
                  Vec3<vfloat<M>> tri_Ng;
                  std::tie(u,v,t,tri_Ng) = hit();
                  const Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
                  if (runOcclusionFilter(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,primID)) return true;
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
