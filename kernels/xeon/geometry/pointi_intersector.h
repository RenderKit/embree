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

#include "pointi.h"
#include "point_intersector.h"
#include "intersector_epilog.h"

namespace embree
{
  namespace isa
  {
    template<int M, int Mx, bool filter>
    struct PointMiIntersector1
    {
      typedef PointMi<M> Primitive;
      typedef typename PointIntersector1<Mx>::Precalculations Precalculations;

      static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive& point, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec4<vfloat<M>> v0; point.gather(v0,scene);
        PointIntersector1<Mx>::intersect(ray,pre,point.valid(),v0,Intersect1Epilog<M,Mx,filter>(ray,point.geomIDs,point.primIDs,scene,geomID_to_instID));
      }

      static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive& point, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec4<vfloat<M>> v0; point.gather(v0,scene);
        return PointIntersector1<Mx>::intersect(ray,pre,point.valid(),v0,Occluded1Epilog<M,Mx,filter>(ray,point.geomIDs,point.primIDs,scene,geomID_to_instID));
      }
    };

    template<int M, int Mx, bool filter>
    struct PointMiMBIntersector1
    {
      typedef PointMi<M> Primitive;
      typedef typename PointIntersector1<Mx>::Precalculations Precalculations;

      static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive& point, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec4<vfloat<M>> v0; point.gather(v0,scene,ray.time);
        PointIntersector1<Mx>::intersect(ray,pre,point.valid(),v0,Intersect1Epilog<M,Mx,filter>(ray,point.geomIDs,point.primIDs,scene,geomID_to_instID));
      }

      static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive& point, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec4<vfloat<M>> v0; point.gather(v0,scene,ray.time);
        return PointIntersector1<Mx>::intersect(ray,pre,point.valid(),v0,Occluded1Epilog<M,Mx,filter>(ray,point.geomIDs,point.primIDs,scene,geomID_to_instID));
      }
    };

    template<int M, int Mx, int K, bool filter>
    struct PointMiIntersectorK
    {
      typedef PointMi<M> Primitive;
      typedef typename PointIntersectorK<Mx,K>::Precalculations Precalculations;

      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& point, Scene* scene)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec4<vfloat<M>> v0; point.gather(v0,scene);
        PointIntersectorK<Mx,K>::intersect(ray,k,pre,point.valid(),v0,Intersect1KEpilog<M,Mx,K,filter>(ray,k,point.geomIDs,point.primIDs,scene));
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& point, Scene* scene)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec4<vfloat<M>> v0; point.gather(v0,scene);
        return PointIntersectorK<Mx,K>::intersect(ray,k,pre,point.valid(),v0,Occluded1KEpilog<M,Mx,K,filter>(ray,k,point.geomIDs,point.primIDs,scene));
      }
    };

    template<int M, int Mx, int K, bool filter>
    struct PointMiMBIntersectorK
    {
      typedef PointMi<M> Primitive;
      typedef typename PointIntersectorK<Mx,K>::Precalculations Precalculations;

      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& point, Scene* scene)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec4<vfloat<M>> v0; point.gather(v0,scene,ray.time[k]);
        PointIntersectorK<Mx,K>::intersect(ray,k,pre,point.valid(),v0,Intersect1KEpilog<M,Mx,K,filter>(ray,k,point.geomIDs,point.primIDs,scene));
      }

      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& point, Scene* scene)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec4<vfloat<M>> v0; point.gather(v0,scene,ray.time[k]);
        return PointIntersectorK<Mx,K>::intersect(ray,k,pre,point.valid(),v0,Occluded1KEpilog<M,Mx,K,filter>(ray,k,point.geomIDs,point.primIDs,scene));
      }
    };
  }
}
