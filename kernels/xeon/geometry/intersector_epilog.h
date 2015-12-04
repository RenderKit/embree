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
    
    template<int M, int Mx, bool filter>
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
        __forceinline bool operator() (const vbool<Mx>& valid_i, Hit& hit) const
        {
          vbool<Mx> valid = valid_i;
          if (Mx > M) valid &= (1<<M)-1;
          hit.finalize();          
          size_t i = select_min(valid,hit.vt);
          int geomID = geomIDs[i];
          int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
          //int instID = geomID_to_instID ? (int)(size_t)geomID_to_instID : geomID;
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          goto entry;
          while (true) 
          {
            if (unlikely(none(valid))) return false;
            i = select_min(valid,hit.vt);

            geomID = geomIDs[i];
            instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
          entry:
            Geometry* geometry = scene->get(geomID);
            
#if defined(RTCORE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask) == 0) {
              clear(valid,i);
              continue;
            }
#endif
            
#if defined(RTCORE_INTERSECTION_FILTER) 
            /* call intersection filter function */
            if (filter) {
              if (unlikely(geometry->hasIntersectionFilter1())) {
                const Vec2f uv = hit.uv(i);
                if (runIntersectionFilter1(geometry,ray,uv.x,uv.y,hit.t(i),hit.Ng(i),instID,primIDs[i])) return true;
                clear(valid,i);
                continue;
              }
            }
#endif
            break;
          }
#endif
          
          /* update hit information */
          const Vec2f uv = hit.uv(i);
          ray.u = uv.x;
          ray.v = uv.y;
          ray.tfar = hit.vt[i];
          ray.Ng.x = hit.vNg.x[i];
          ray.Ng.y = hit.vNg.y[i];
          ray.Ng.z = hit.vNg.z[i];
          ray.geomID = instID;
          ray.primID = primIDs[i];
          return true;
        }
      };
    
    
    template<int M, int Mx, bool filter>
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
        __forceinline bool operator() (const vbool<Mx>& valid_i, Hit& hit) const
        {
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          vbool<Mx> valid = valid_i;
          if (Mx > M) valid &= (1<<M)-1;
          size_t m=movemask(valid);
          goto entry;
          while (true)
          {  
            if (unlikely(m == 0)) return false;
          entry:
            size_t i=__bsf(m);
            hit.finalize(); // FIXME: executed too often

            const int geomID = geomIDs[i];
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
                //const Vec3fa Ngi = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
                const Vec2f uv = hit.uv(i);
                if (runOcclusionFilter1(geometry,ray,uv.x,uv.y,hit.t(i),hit.Ng(i),instID,primIDs[i])) return true;
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
        __forceinline bool operator() (const vbool<M>& valid_i, Hit& hit) const
        {
          /* ray mask test */
          Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
          if ((geometry->mask & ray.mask) == 0) return false;
#endif
          
          vbool<M> valid = valid_i;
          hit.finalize();
          
          size_t i = select_min(valid,hit.vt);
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (unlikely(geometry->hasIntersectionFilter1())) 
          {
            while (true) 
            {
              /* call intersection filter function */
              Vec2f uv = hit.uv(i);
              if (runIntersectionFilter1(geometry,ray,uv.x,uv.y,hit.t(i),hit.Ng(i),geomID,primID)) {
                return true;
              }
              clear(valid,i);
              if (unlikely(none(valid))) break;
              i = select_min(valid,hit.vt);
            }
            return false;
          }
#endif
          
          /* update hit information */
          const Vec2f uv = hit.uv(i);
          ray.u = uv.x;
          ray.v = uv.y;
          ray.tfar = hit.vt[i];
          ray.Ng.x = hit.vNg.x[i];
          ray.Ng.y = hit.vNg.y[i];
          ray.Ng.z = hit.vNg.z[i];
          ray.geomID = geomID;
          ray.primID = primID;
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
        __forceinline bool operator() (const vbool<M>& valid, Hit& hit) const
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
            hit.finalize();
            for (size_t m=movemask(valid), i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) 
            {  
              const Vec2f uv = hit.uv(i);
              if (runOcclusionFilter1(geometry,ray,uv.x,uv.y,hit.t(i),hit.Ng(i),geomID,primID)) return true;
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

          std::tie(u,v,t,Ng) = hit();
          
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
          if (unlikely(none(valid))) return valid;
#endif
          
          /* intersection filter test */
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
        __forceinline vbool<K> operator() (const vbool<K>& valid_org, const Hit& hit) const
        {
          vbool<K> valid = valid_org;
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
    
    
    
    template<int M, int Mx, int K, bool filter>
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
        __forceinline bool operator() (const vbool<Mx>& valid_i, Hit& hit) const
        {
          vbool<Mx> valid = valid_i;
          hit.finalize();
          if (Mx > M) valid &= (1<<M)-1;
          size_t i = select_min(valid,hit.vt);
          assert(i<M);
          int geomID = geomIDs[i];
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          goto entry;
          while (true) 
          {
            if (unlikely(none(valid))) return false;
            i = select_min(valid,hit.vt);
            assert(i<M);            
            geomID = geomIDs[i];
          entry:
            Geometry* geometry = scene->get(geomID);
            
#if defined(RTCORE_RAY_MASK)
            /* goto next hit if mask test fails */
            if ((geometry->mask & ray.mask[k]) == 0) {
              clear(valid,i);
              continue;
            }
#endif
            
#if defined(RTCORE_INTERSECTION_FILTER) 
            /* call intersection filter function */
            if (filter) {
              if (unlikely(geometry->hasIntersectionFilter<vfloat<K>>())) {
                assert(i<M);
                const Vec2f uv = hit.uv(i);
                if (runIntersectionFilter(geometry,ray,k,uv.x,uv.y,hit.t(i),hit.Ng(i),geomID,primIDs[i])) return true;
                clear(valid,i);
                continue;
              }
            }
#endif
            break;
          }
#endif
          assert(i<M);
          /* update hit information */
          const Vec2f uv = hit.uv(i);
          ray.u[k] = uv.x;
          ray.v[k] = uv.y;
          ray.tfar[k] = hit.vt[i];
          ray.Ng.x[k] = hit.vNg.x[i];
          ray.Ng.y[k] = hit.vNg.y[i];
          ray.Ng.z[k] = hit.vNg.z[i];
          ray.geomID[k] = geomID;
          ray.primID[k] = primIDs[i];
          return true;
        }
      };
    
    template<int M, int Mx, int K, bool filter>
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
        __forceinline bool operator() (const vbool<Mx>& valid_i, Hit& hit) const
        {

          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
          vbool<Mx> valid = valid_i;
          if (Mx > M) valid &= (1<<M)-1;
          size_t m=movemask(valid);
          goto entry;
          while (true)
          {  
            if (unlikely(m == 0)) return false;
          entry:
            size_t i=__bsf(m);
            hit.finalize(); // FIXME: executed too often

            const int geomID = geomIDs[i];
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
                const Vec2f uv = hit.uv(i);
                if (runOcclusionFilter(geometry,ray,k,uv.x,uv.y,hit.t(i),hit.Ng(i),geomID,primIDs[i])) return true;
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
        __forceinline bool operator() (const vbool<M>& valid_i, Hit& hit) const
        {
          Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
          /* ray mask test */
          if ((geometry->mask & ray.mask[k]) == 0) 
            return false;
#endif

          /* finalize hit calculation */
          vbool<M> valid = valid_i;
          hit.finalize();
          size_t i = select_min(valid,hit.vt);
          
          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (filter) {
            if (unlikely(geometry->hasIntersectionFilter<vfloat<K>>())) 
            {
              while (true) 
              {
                const Vec2f uv = hit.uv(i);
                if (runIntersectionFilter(geometry,ray,k,uv.x,uv.y,hit.t(i),hit.Ng(i),geomID,primID)) return true;
                clear(valid,i);

                if (unlikely(none(valid))) break;
                i = select_min(valid,hit.vt);
              }
              return false;
            }
          }
#endif
          
          /* update hit information */
          const Vec2f uv = hit.uv(i);
          ray.u[k] = uv.x;
          ray.v[k] = uv.y;
          ray.tfar[k] = hit.vt[i];
          ray.Ng.x[k] = hit.vNg.x[i];
          ray.Ng.y[k] = hit.vNg.y[i];
          ray.Ng.z[k] = hit.vNg.z[i];
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
        __forceinline bool operator() (const vbool<M>& valid_i, Hit& hit) const
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
              hit.finalize();
              for (size_t m=movemask(valid_i), i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m))
              {  
                const Vec2f uv = hit.uv(i);
                if (runOcclusionFilter(geometry,ray,k,uv.x,uv.y,hit.t(i),hit.Ng(i),geomID,primID)) return true;
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
