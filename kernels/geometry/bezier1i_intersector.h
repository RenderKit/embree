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

#include "bezier1i.h"
#include "bezier_hair_intersector.h"
#include "bezier_ribbon_intersector.h"
#include "bezier_curve_intersector.h"

namespace embree
{
  namespace isa
  {
    struct Bezier1iIntersector1
    {
      typedef Bezier1i Primitive;

      struct Precalculations
      {
        __forceinline Precalculations() {}

        __forceinline Precalculations(const Ray& ray, const void* ptr)
          : intersectorHair(ray,ptr), intersectorCurve(ray,ptr) {}

        Bezier1Intersector1<Curve3fa> intersectorHair;
        BezierCurve1Intersector1<Curve3fa> intersectorCurve;
      };

      static __forceinline void intersect(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& prim)
      {
        STAT3(normal.trav_prims,1,1,1);
        const NativeCurves* geom = (NativeCurves*) context->scene->get(prim.geomID());
        Vec3fa a0,a1,a2,a3; geom->gather(a0,a1,a2,a3,prim.vertexID);
        if (likely(geom->subtype == NativeCurves::HAIR))
          pre.intersectorHair.intersect(ray,a0,a1,a2,a3,geom->tessellationRate,Intersect1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID()));
        else 
          pre.intersectorCurve.intersect(ray,a0,a1,a2,a3,Intersect1Epilog1<true>(ray,context,prim.geomID(),prim.primID()));
      }
      
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& prim)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const NativeCurves* geom = (NativeCurves*) context->scene->get(prim.geomID());
        Vec3fa a0,a1,a2,a3; geom->gather(a0,a1,a2,a3,prim.vertexID);
        if (likely(geom->subtype == NativeCurves::HAIR))
          return pre.intersectorHair.intersect(ray,a0,a1,a2,a3,geom->tessellationRate,Occluded1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID()));
        else
          return pre.intersectorCurve.intersect(ray,a0,a1,a2,a3,Occluded1Epilog1<true>(ray,context,prim.geomID(),prim.primID()));
      }
    };

    template<int K>
      struct Bezier1iIntersectorK
    {
      typedef Bezier1i Primitive;

      struct Precalculations
      {
        __forceinline Precalculations() {}

        __forceinline Precalculations(const vbool<K>& valid, const RayK<K>& ray)
          : intersectorHair(valid,ray), intersectorCurve(valid,ray) {}

        __forceinline Precalculations(const RayK<K>& ray, size_t k)
          : intersectorHair(ray,k), intersectorCurve(ray,k) {}

        Bezier1IntersectorK<Curve3fa,K> intersectorHair;
        BezierCurve1IntersectorK<Curve3fa,K> intersectorCurve;
      };
      
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, const size_t k, IntersectContext* context, const Primitive& prim)
      {
        STAT3(normal.trav_prims,1,1,1);
        const NativeCurves* geom = (NativeCurves*) context->scene->get(prim.geomID());
        Vec3fa a0,a1,a2,a3; geom->gather(a0,a1,a2,a3,prim.vertexID);
        if (likely(geom->subtype == NativeCurves::HAIR))
          pre.intersectorHair.intersect(ray,k,a0,a1,a2,a3,geom->tessellationRate,Intersect1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID()));
        else 
          pre.intersectorCurve.intersect(ray,k,a0,a1,a2,a3,Intersect1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID()));
      }
      
      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive& prim)
      {
        size_t mask = movemask(valid_i);
        while (mask) intersect(pre,ray,__bscf(mask),context,prim);
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, const size_t k, IntersectContext* context, const Primitive& prim)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const NativeCurves* geom = (NativeCurves*) context->scene->get(prim.geomID());
        Vec3fa a0,a1,a2,a3; geom->gather(a0,a1,a2,a3,prim.vertexID);
        if (likely(geom->subtype == NativeCurves::HAIR))
          return pre.intersectorHair.intersect(ray,k,a0,a1,a2,a3,geom->tessellationRate,Occluded1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID()));
        else
          return pre.intersectorCurve.intersect(ray,k,a0,a1,a2,a3,Occluded1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID()));
      }
      
      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive& prim)
      {
        vbool<K> valid_o = false;
        size_t mask = movemask(valid_i);
        while (mask) {
          size_t k = __bscf(mask);
          if (occluded(pre,ray,k,context,prim))
            set(valid_o, k);
        }
        return valid_o;
      }
    };
    
    struct Bezier1iIntersector1MB
    {
      typedef Bezier1i Primitive;
      
      struct Precalculations
      {
        __forceinline Precalculations() {}

        __forceinline Precalculations(const Ray& ray, const void* ptr)
          : intersectorHair(ray,ptr), intersectorCurve(ray,ptr) {}

        Bezier1Intersector1<Curve3fa> intersectorHair;
        BezierCurve1Intersector1<Curve3fa> intersectorCurve;
      };
            
      static __forceinline void intersect(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& prim)
      {
        STAT3(normal.trav_prims,1,1,1);
        const NativeCurves* geom = (NativeCurves*) context->scene->get(prim.geomID());
        Vec3fa p0,p1,p2,p3; geom->gather(p0,p1,p2,p3,prim.vertexID,ray.time);
        if (likely(geom->subtype == NativeCurves::HAIR))
          pre.intersectorHair.intersect(ray,p0,p1,p2,p3,geom->tessellationRate,Intersect1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID()));
        else 
          pre.intersectorCurve.intersect(ray,p0,p1,p2,p3,Intersect1Epilog1<true>(ray,context,prim.geomID(),prim.primID()));
      }
      
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& prim) 
      {
        STAT3(shadow.trav_prims,1,1,1);
        const NativeCurves* geom = (NativeCurves*) context->scene->get(prim.geomID());
        Vec3fa p0,p1,p2,p3; geom->gather(p0,p1,p2,p3,prim.vertexID,ray.time);
        if (likely(geom->subtype == NativeCurves::HAIR))
          return pre.intersectorHair.intersect(ray,p0,p1,p2,p3,geom->tessellationRate,Occluded1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID()));
        else
          return pre.intersectorCurve.intersect(ray,p0,p1,p2,p3,Occluded1Epilog1<true>(ray,context,prim.geomID(),prim.primID()));
      }
    };

    template<int K>
    struct Bezier1iIntersectorKMB
    {
      typedef Bezier1i Primitive;

      struct Precalculations
      {
        __forceinline Precalculations() {}

        __forceinline Precalculations(const vbool<K>& valid, const RayK<K>& ray)
          : intersectorHair(valid,ray), intersectorCurve(valid,ray) {}

        __forceinline Precalculations(const RayK<K>& ray, size_t k)
          : intersectorHair(ray,k), intersectorCurve(ray,k) {}

        Bezier1IntersectorK<Curve3fa,K> intersectorHair;
        BezierCurve1IntersectorK<Curve3fa,K> intersectorCurve;
      };
      
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, const size_t k, IntersectContext* context, const Primitive& prim)
      {
        STAT3(normal.trav_prims,1,1,1);
        const NativeCurves* geom = (NativeCurves*) context->scene->get(prim.geomID());        
        Vec3fa p0,p1,p2,p3; geom->gather(p0,p1,p2,p3,prim.vertexID,ray.time[k]);
        if (likely(geom->subtype == NativeCurves::HAIR))
          pre.intersectorHair.intersect(ray,k,p0,p1,p2,p3,geom->tessellationRate,Intersect1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID()));
        else 
          pre.intersectorCurve.intersect(ray,k,p0,p1,p2,p3,Intersect1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID()));
      }

      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive& prim)
      {
        size_t mask = movemask(valid_i);
        while (mask) intersect(pre,ray,__bscf(mask),context,prim);
      }

      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, const size_t k, IntersectContext* context, const Primitive& prim)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const NativeCurves* geom = (NativeCurves*) context->scene->get(prim.geomID());
        Vec3fa p0,p1,p2,p3; geom->gather(p0,p1,p2,p3,prim.vertexID,ray.time[k]);
        if (likely(geom->subtype == NativeCurves::HAIR))
          return pre.intersectorHair.intersect(ray,k,p0,p1,p2,p3,geom->tessellationRate,Occluded1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID()));
        else
          return pre.intersectorCurve.intersect(ray,k,p0,p1,p2,p3,Occluded1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID()));
      }

      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive& prim)
      {
        vbool<K> valid_o = false;
        size_t mask = movemask(valid_i);
        while (mask) {
          size_t k = __bscf(mask);
          if (occluded(pre,ray,k,context,prim))
            set(valid_o, k);
        }
        return valid_o;
      }
    };
  }
}
