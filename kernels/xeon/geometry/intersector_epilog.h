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
        const vint<M>& geomIDs;
        const vint<M>& primIDs;
        Scene* scene;
        const unsigned* geomID_to_instID;
        
        __forceinline Intersect1Epilog(Ray& ray,
                                       const vint<M>& geomIDs, 
                                       const vint<M>& primIDs, 
                                       Scene* scene,
                                       const unsigned* geomID_to_instID)
          : ray(ray), geomIDs(geomIDs), primIDs(primIDs), scene(scene), geomID_to_instID(geomID_to_instID) {}
        
        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          vfloat<M> u, v, t; 
          Vec3<vfloat<M>> Ng;
          vbool<M> valid = valid_i;
          size_t i;
          std::tie(u,v,t,Ng,i) = hit(valid);
          
          //size_t i = select_min(valid,t);
          int geomID = geomIDs[i];
          int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          goto entry;
          while (true) 
          {
            if (unlikely(none(valid))) return false;
            //i = select_min(valid,t);
            std::tie(u,v,t,Ng,i) = hit(valid);

            geomID = geomIDs[i];
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
                const Vec3fa Ngi = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
                if (runIntersectionFilter1(geometry,ray,u[i],v[i],t[i],Ngi,instID,primIDs[i])) return true;
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
          ray.Ng.x = Ng.x[i];
          ray.Ng.y = Ng.y[i];
          ray.Ng.z = Ng.z[i];
          ray.geomID = instID;
          ray.primID = primIDs[i];
          return true;
        }
      };
    
    
    template<int M, bool filter>
      struct Occluded1Epilog
      {
        Ray& ray;
        const vint<M>& geomIDs;
        const vint<M>& primIDs;
        Scene* scene;
        const unsigned* geomID_to_instID;
        
        __forceinline Occluded1Epilog(Ray& ray,
                                      const vint<M>& geomIDs, 
                                      const vint<M>& primIDs, 
                                      Scene* scene,
                                      const unsigned* geomID_to_instID)
          : ray(ray), geomIDs(geomIDs), primIDs(primIDs), scene(scene), geomID_to_instID(geomID_to_instID) {}
        
        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          vbool<M> valid = valid_i;
          goto entry;
          while (true)
          {  
            size_t m=movemask(valid);
            if (unlikely(m == 0)) return false;
          entry:
            //size_t i=__bsf(m);
            Vec3<vfloat<M>> Ng;
            vfloat<M> u,v,t; 
            size_t i;
            std::tie(u,v,t,Ng,i) = hit(valid);


            const int geomID = geomIDs[i];
            const int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
            Geometry* geometry = scene->get(geomID);
            
#if defined(RTCORE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
              //m=__btc(m,i);
              valid[i] = 0;
              continue;
            }
#endif
            
#if defined(RTCORE_INTERSECTION_FILTER)
            /* if we have no filter then the test passed */
            if (filter) {
              if (unlikely(geometry->hasOcclusionFilter1())) 
              {
                const Vec3fa Ngi = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
                if (runOcclusionFilter1(geometry,ray,u[i],v[i],t[i],Ngi,instID,primIDs[i])) return true;
                //m=__btc(m,i);
                valid[i] = 0;
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
          Vec3<vfloat<M>> Ng;
          vbool<M> valid = valid_i;
          size_t i;
          std::tie(u,v,t,Ng,i) = hit(valid);
          
          //size_t i = select_min(valid,t);
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (unlikely(geometry->hasIntersectionFilter1())) 
          {
            while (true) 
            {
              /* call intersection filter function */
              Vec3fa Ng_i = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
              if (runIntersectionFilter1(geometry,ray,u[i],v[i],t[i],Ng_i,geomID,primID)) {
                return true;
              }
              valid[i] = 0;
              if (unlikely(none(valid))) break;
              //i = select_min(valid,t);
              std::tie(u,v,t,Ng,i) = hit(valid);
            }
            return false;
          }
#endif
          
          /* update hit information */
          ray.u         = u[i];
          ray.v         = v[i];
          ray.tfar      = t[i];
          ray.geomID    = geomID;
          ray.primID    = primID;
          ray.Ng.x      = Ng.x[i];
          ray.Ng.y      = Ng.y[i];
          ray.Ng.z      = Ng.z[i];
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
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          /* ray mask test */
          Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
          if ((geometry->mask & ray.mask) == 0) return false;
#endif
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          vbool<M> valid = valid_i;
          if (unlikely(geometry->hasOcclusionFilter1())) 
          {
            vfloat<M> u, v, t; 
            Vec3<vfloat<M>> Ng;
            
            //for (size_t m=movemask(valid), i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) 
            while(any(valid))
            {  
              size_t i;
              std::tie(u,v,t,Ng,i) = hit(valid);
              const Vec3fa Ng_i = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
              if (runOcclusionFilter1(geometry,ray,u[i],v[i],t[i],Ng_i,geomID,primID)) return true;
              valid[i] = 0;
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
        const vint<M>& geomIDs;
        const vint<M>& primIDs;
        const int i;
        Scene* const scene;
        
        __forceinline IntersectKEpilog(RayK<K>& ray,
                                       const vint<M>& geomIDs, 
                                       const vint<M>& primIDs, 
                                       int i,
                                       Scene* scene)
          : ray(ray), geomIDs(geomIDs), primIDs(primIDs), i(i), scene(scene) {}
        
        template<typename Hit>
        __forceinline vbool<K> operator() (const vbool<K>& valid_i, const Hit& hit) const
        {
          vfloat<K> u, v, t; 
          Vec3<vfloat<K>> Ng;
          vbool<K> valid = valid_i;

          std::tie(u,v,t,Ng) = hit(valid);
          
          const int geomID = geomIDs[i];
          const int primID = primIDs[i];
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
      struct OccludedKEpilog
      {
        vbool<K>& valid0;
        RayK<K>& ray;
        const vint<M>& geomIDs;
        const vint<M>& primIDs;
        const int i;
        Scene* const scene;
        
        __forceinline OccludedKEpilog(vbool<K>& valid0,
                                      RayK<K>& ray,
                                      const vint<M>& geomIDs, 
                                      const vint<M>& primIDs, 
                                      int i,
                                      Scene* scene)
          : valid0(valid0), ray(ray), geomIDs(geomIDs), primIDs(primIDs), i(i), scene(scene) {}
        
        template<typename Hit>
        __forceinline vbool<K> operator() (const vbool<K>& valid_i, const Hit& hit) const
        {
          vbool<K> valid = valid_i;
          
          /* ray masking test */
          const int geomID = geomIDs[i];
          const int primID = primIDs[i];
          Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
          valid &= (geometry->mask & ray.mask) != 0;
          if (unlikely(none(valid))) return;
#endif
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (filter) {
            if (unlikely(geometry->hasOcclusionFilter<vfloat<K>>()))
            {
              vfloat<K> u, v, t; 
              Vec3<vfloat<K>> Ng;
              std::tie(u,v,t,Ng) = hit(valid);
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
          std::tie(u,v,t,Ng) = hit(valid);
          
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
              std::tie(u,v,t,Ng) = hit(valid);
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
        const vint<M>& geomIDs;
        const vint<M>& primIDs;
        Scene* const scene;
        
        __forceinline Intersect1KEpilog(RayK<K>& ray, int k,
                                        const vint<M>& geomIDs, 
                                        const vint<M>& primIDs, 
                                        Scene* scene)
          : ray(ray), k(k), geomIDs(geomIDs), primIDs(primIDs), scene(scene) {}
        
        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          vfloat<M> u, v, t; 
          Vec3<vfloat<M>> Ng;
          vbool<M> valid = valid_i;
          size_t i;
          std::tie(u,v,t,Ng,i) = hit(valid);
          
          //size_t i = select_min(valid,t);
          int geomID = geomIDs[i];
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          goto entry;
          while (true) 
          {
            if (unlikely(none(valid))) return false;
            //i = select_min(valid,t);
            std::tie(u,v,t,Ng,i) = hit(valid);

            geomID = geomIDs[i];
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
                const Vec3fa Ngi = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
                if (runIntersectionFilter(geometry,ray,k,u[i],v[i],t[i],Ngi,geomID,primIDs[i])) return true;
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
          ray.Ng.x[k] = Ng.x[i];
          ray.Ng.y[k] = Ng.y[i];
          ray.Ng.z[k] = Ng.z[i];
          ray.geomID[k] = geomID;
          ray.primID[k] = primIDs[i];
          return true;
        }
      };
    
    template<int M, int K, bool filter>
      struct Occluded1KEpilog
      {
        RayK<K>& ray;
        int k;
        const vint<M>& geomIDs;
        const vint<M>& primIDs;
        Scene* const scene;
        
        __forceinline Occluded1KEpilog(RayK<K>& ray, int k,
                                       const vint<M>& geomIDs, 
                                       const vint<M>& primIDs, 
                                       Scene* scene)
          : ray(ray), k(k), geomIDs(geomIDs), primIDs(primIDs), scene(scene) {}
        
        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          vbool<M> valid = valid_i;
          goto entry;
          while (true)
          {  
            size_t m=movemask(valid);
            if (unlikely(m == 0)) return false;
          entry:
            //size_t i=__bsf(m);
            Vec3<vfloat<M>> Ng;
            vfloat<M> u,v,t; 
            size_t i;
            std::tie(u,v,t,Ng,i) = hit(valid);

            const int geomID = geomIDs[i];
            Geometry* geometry = scene->get(geomID);
            
#if defined(RTCORE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask[k]) == 0) {
              //m=__btc(m,i);
              valid[i] = 0;
              continue;
            }
#endif
            
#if defined(RTCORE_INTERSECTION_FILTER)
            /* execute occlusion filer */
            if (filter) {
              if (unlikely(geometry->hasOcclusionFilter<vfloat<K>>())) 
              {
                const Vec3fa Ngi = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
                if (runOcclusionFilter(geometry,ray,k,u[i],v[i],t[i],Ngi,geomID,primIDs[i])) return true;
                //m=__btc(m,i);
                valid[i] = 0;
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
      struct Intersect1KEpilogU
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
          Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
          /* ray mask test */
          if ((geometry->mask & ray.mask[k]) == 0) 
            return false;
#endif

          /* finalize hit calculation */
          vfloat<M> u, v, t; 
          Vec3<vfloat<M>> Ng;
          vbool<M> valid = valid_i;
          size_t i;
          std::tie(u,v,t,Ng,i) = hit(valid);
          //size_t i = select_min(valid,t);
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (filter) {
            if (unlikely(geometry->hasIntersectionFilter<vfloat<K>>())) 
            {
              while (true) 
              {
                const Vec3fa Ngi = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
                if (runIntersectionFilter(geometry,ray,k,u[i],v[i],t[i],Ngi,geomID,primID)) return true;
                valid[i] = 0;
                if (unlikely(none(valid))) break;
                //i = select_min(valid,t);
                std::tie(u,v,t,Ng,i) = hit(valid);
              }
              return false;
            }
          }
#endif
          
          /* update hit information */
          ray.u[k] = u[i];
          ray.v[k] = v[i];
          ray.tfar[k] = t[i];
          ray.Ng.x[k] = Ng.x[i];
          ray.Ng.y[k] = Ng.y[i];
          ray.Ng.z[k] = Ng.z[i];
          ray.geomID[k] = geomID;
          ray.primID[k] = primID;
          return true;
        }
      };
    
    template<int M, int K, bool filter>
      struct Occluded1KEpilogU
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
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
          /* ray mask test */
          if ((geometry->mask & ray.mask[k]) == 0) 
            return false;
#endif

          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (filter) {
            if (unlikely(geometry->hasOcclusionFilter<vfloat<K>>())) 
            {
              vbool<M> valid = valid_i;
              //for (size_t m=movemask(valid), i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m))
              while(any(valid))
              {  
                vfloat<M> u, v, t; 
                Vec3<vfloat<M>> Ng;
                size_t i;
                std::tie(u,v,t,Ng,i) = hit(valid);
                const Vec3fa Ngi = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
                if (runOcclusionFilter(geometry,ray,k,u[i],v[i],t[i],Ngi,geomID,primID)) return true;
                valid[i] = 0;
              }
              return false;
            }
          }
#endif 
          return true;
        }
      };
  }
}
