// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
#include "bezier_intersector.h"

namespace embree
{
  namespace isa
  {
    struct Bezier1iIntersector1
    {
      typedef Bezier1i Primitive;
      typedef Bezier1iIntersector1 Precalculations;

      __forceinline Bezier1iIntersector1() {}

      __forceinline Bezier1iIntersector1(const Ray& ray, const void* ptr)
        : intersectorHair(ray,ptr), intersectorCurve(ray,ptr) {}

      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const RTCIntersectContext* context, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        const BezierCurves* geom = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = geom->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = geom->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = geom->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = geom->vertex(prim.vertexID+3,0);
        if (likely(geom->subtype == BezierCurves::HAIR))
          pre.intersectorHair.intersect(ray,a0,a1,a2,a3,geom->tessellationRate,Intersect1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID(),scene,geomID_to_instID));
        else 
          pre.intersectorCurve.intersect(ray,a0,a1,a2,a3,Intersect1Epilog1<true>(ray,context,prim.geomID(),prim.primID(),scene,geomID_to_instID));
      }
      
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const RTCIntersectContext* context, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const BezierCurves* geom = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = geom->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = geom->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = geom->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = geom->vertex(prim.vertexID+3,0);
        if (likely(geom->subtype == BezierCurves::HAIR))
          return pre.intersectorHair.intersect(ray,a0,a1,a2,a3,geom->tessellationRate,Occluded1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID(),scene,geomID_to_instID));
        else
          return pre.intersectorCurve.intersect(ray,a0,a1,a2,a3,Occluded1Epilog1<true>(ray,context,prim.geomID(),prim.primID(),scene,geomID_to_instID));
      }

      /*! Intersect an array of rays with an array of M primitives. */
      static __forceinline size_t intersect(Precalculations* pre, size_t valid, Ray** rays, const RTCIntersectContext* context,  size_t ty, const Primitive* prim, size_t num, Scene* scene, const unsigned* geomID_to_instID)
      {
        size_t valid_isec = 0;
        do {
          const size_t i = __bscf(valid);
          const float old_far = rays[i]->tfar;
          for (size_t n=0; n<num; n++)
            intersect(pre[i],*rays[i],context,prim[n],scene,geomID_to_instID);
          valid_isec |= (rays[i]->tfar < old_far) ? ((size_t)1 << i) : 0;            
        } while(unlikely(valid));
        return valid_isec;
      }


    public:
      Bezier1Intersector1 intersectorHair;
      BezierGeometry1Intersector1 intersectorCurve;
    };

    template<int K>
      struct Bezier1iIntersectorK
    {
      typedef Bezier1i Primitive;
      typedef Bezier1iIntersectorK Precalculations;

      __forceinline Bezier1iIntersectorK(const vbool<K>& valid, const RayK<K>& ray) 
        : intersectorHair(valid,ray), intersectorCurve(valid,ray) {}

      __forceinline Bezier1iIntersectorK (const RayK<K>& ray, size_t k) 
        : intersectorHair(ray,k), intersectorCurve(ray,k) {}
      
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, const size_t k, const RTCIntersectContext* context, const Primitive& prim, Scene* scene)
      {
        STAT3(normal.trav_prims,1,1,1);
        const BezierCurves* geom = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = geom->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = geom->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = geom->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = geom->vertex(prim.vertexID+3,0);
        if (likely(geom->subtype == BezierCurves::HAIR))
          pre.intersectorHair.intersect(ray,k,a0,a1,a2,a3,geom->tessellationRate,Intersect1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID(),scene));
        else 
          pre.intersectorCurve.intersect(ray,k,a0,a1,a2,a3,Intersect1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID(),scene));
      }
      
      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const RTCIntersectContext* context, const Primitive& prim, Scene* scene)
      {
        int mask = movemask(valid_i);
        while (mask) intersect(pre,ray,context,__bscf(mask),prim,scene);
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, const size_t k, const RTCIntersectContext* context, const Primitive& prim, Scene* scene)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const BezierCurves* geom = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = geom->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = geom->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = geom->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = geom->vertex(prim.vertexID+3,0);
        if (likely(geom->subtype == BezierCurves::HAIR))
          return pre.intersectorHair.intersect(ray,k,a0,a1,a2,a3,geom->tessellationRate,Occluded1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID(),scene));
        else
          return pre.intersectorCurve.intersect(ray,k,a0,a1,a2,a3,Occluded1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID(),scene));
      }
      
      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const RTCIntersectContext* context, const Primitive& prim, Scene* scene)
      {
        vbool<K> valid_o = false;
        int mask = movemask(valid_i);
        while (mask) {
          size_t k = __bscf(mask);
          if (occluded(pre,ray,k,context,prim,scene))
            set(valid_o, k);
        }
        return valid_o;
      }

    public:
      Bezier1IntersectorK<K> intersectorHair;
      BezierGeometry1IntersectorK<K> intersectorCurve;
    };
    
    struct Bezier1iIntersector1MB
    {
      typedef Bezier1i Primitive;
      typedef Bezier1iIntersector1MB Precalculations;
      
      __forceinline Bezier1iIntersector1MB() {}

      __forceinline Bezier1iIntersector1MB(const Ray& ray, const void* ptr)
        : intersectorHair(ray,ptr), intersectorCurve(ray,ptr) {}
            
      static __forceinline void intersect(Precalculations& pre, Ray& ray, const RTCIntersectContext* context, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        const BezierCurves* geom = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = geom->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = geom->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = geom->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = geom->vertex(prim.vertexID+3,0);
        const Vec3fa b0 = geom->vertex(prim.vertexID+0,1);
        const Vec3fa b1 = geom->vertex(prim.vertexID+1,1);
        const Vec3fa b2 = geom->vertex(prim.vertexID+2,1);
        const Vec3fa b3 = geom->vertex(prim.vertexID+3,1);
        const float t0 = 1.0f-ray.time, t1 = ray.time;
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        if (likely(geom->subtype == BezierCurves::HAIR))
          pre.intersectorHair.intersect(ray,p0,p1,p2,p3,geom->tessellationRate,Intersect1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID(),scene,geomID_to_instID));
        else 
          pre.intersectorCurve.intersect(ray,p0,p1,p2,p3,Intersect1Epilog1<true>(ray,context,prim.geomID(),prim.primID(),scene,geomID_to_instID));
      }
      
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, const RTCIntersectContext* context, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID) 
      {
        STAT3(shadow.trav_prims,1,1,1);
        const BezierCurves* geom = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = geom->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = geom->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = geom->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = geom->vertex(prim.vertexID+3,0);
        const Vec3fa b0 = geom->vertex(prim.vertexID+0,1);
        const Vec3fa b1 = geom->vertex(prim.vertexID+1,1);
        const Vec3fa b2 = geom->vertex(prim.vertexID+2,1);
        const Vec3fa b3 = geom->vertex(prim.vertexID+3,1);
        const float t0 = 1.0f-ray.time, t1 = ray.time;
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        if (likely(geom->subtype == BezierCurves::HAIR))
          return pre.intersectorHair.intersect(ray,p0,p1,p2,p3,geom->tessellationRate,Occluded1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID(),scene,geomID_to_instID));
        else
          return pre.intersectorCurve.intersect(ray,p0,p1,p2,p3,Occluded1Epilog1<true>(ray,context,prim.geomID(),prim.primID(),scene,geomID_to_instID));
      }

    public:
      Bezier1Intersector1 intersectorHair;
      BezierGeometry1Intersector1 intersectorCurve;
    };

    template<int K>
    struct Bezier1iIntersectorKMB
    {
      typedef Bezier1i Primitive;
      typedef Bezier1iIntersectorKMB Precalculations;

      __forceinline Bezier1iIntersectorKMB (const vbool<K>& valid, const RayK<K>& ray) 
        : intersectorHair(valid,ray), intersectorCurve(valid,ray) {}

      __forceinline Bezier1iIntersectorKMB (const RayK<K>& ray, size_t k) 
        : intersectorHair(ray,k), intersectorCurve(ray,k) {}
      
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, const size_t k, const RTCIntersectContext* context, const Primitive& prim, Scene* scene)
      {
        STAT3(normal.trav_prims,1,1,1);
        const BezierCurves* geom = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = geom->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = geom->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = geom->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = geom->vertex(prim.vertexID+3,0);
        const Vec3fa b0 = geom->vertex(prim.vertexID+0,1);
        const Vec3fa b1 = geom->vertex(prim.vertexID+1,1);
        const Vec3fa b2 = geom->vertex(prim.vertexID+2,1);
        const Vec3fa b3 = geom->vertex(prim.vertexID+3,1);
        const float t0 = 1.0f-ray.time[k], t1 = ray.time[k];
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        if (likely(geom->subtype == BezierCurves::HAIR))
          pre.intersectorHair.intersect(ray,k,p0,p1,p2,p3,geom->tessellationRate,Intersect1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID(),scene));
        else 
          pre.intersectorCurve.intersect(ray,k,p0,p1,p2,p3,Intersect1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID(),scene));
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, const size_t k, const RTCIntersectContext* context, const Primitive& prim, Scene* scene)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const BezierCurves* geom = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = geom->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = geom->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = geom->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = geom->vertex(prim.vertexID+3,0);
        const Vec3fa b0 = geom->vertex(prim.vertexID+0,1);
        const Vec3fa b1 = geom->vertex(prim.vertexID+1,1);
        const Vec3fa b2 = geom->vertex(prim.vertexID+2,1);
        const Vec3fa b3 = geom->vertex(prim.vertexID+3,1);
        const float t0 = 1.0f-ray.time[k], t1 = ray.time[k];
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        if (likely(geom->subtype == BezierCurves::HAIR))
          return pre.intersectorHair.intersect(ray,k,p0,p1,p2,p3,geom->tessellationRate,Occluded1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID(),scene));
        else
          return pre.intersectorCurve.intersect(ray,k,a0,a1,a2,a3,Occluded1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID(),scene));
      }

    public:
      Bezier1IntersectorK<K> intersectorHair;
      BezierGeometry1IntersectorK<K> intersectorCurve;
    };
  }
}
