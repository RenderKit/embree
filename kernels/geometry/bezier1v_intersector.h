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

#include "bezier1v.h"
#include "bezier_hair_intersector.h"
#include "bezier_ribbon_intersector.h"
#include "bezier_curve_intersector.h"

namespace embree
{
  namespace isa
  {
    /*! Intersector for a single ray with a bezier curve. */
    struct Bezier1vIntersector1
    {
      typedef Bezier1v Primitive;
      typedef CurvePrecalculations1 Precalculations;
      
      static __forceinline void intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive& prim)
      {
        STAT3(normal.trav_prims,1,1,1);
        Bezier1Intersector1<Curve3fa> intersectorHair;
        BezierCurve1Intersector1<Curve3fa> intersectorCurve;
        const NativeCurves* geom = (NativeCurves*)context->scene->get(prim.geomID());
        if (likely(geom->subtype == FLAT_CURVE))
          intersectorHair.intersect(pre,ray,geom,prim.p0,prim.p1,prim.p2,prim.p3,Intersect1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID()));
        else 
          intersectorCurve.intersect(pre,ray,geom,prim.p0,prim.p1,prim.p2,prim.p3,Intersect1Epilog1<true>(ray,context,prim.geomID(),prim.primID()));
      }
      
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& prim)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Bezier1Intersector1<Curve3fa> intersectorHair;
        BezierCurve1Intersector1<Curve3fa> intersectorCurve;
        const NativeCurves* geom = (NativeCurves*)context->scene->get(prim.geomID());
        if (likely(geom->subtype == FLAT_CURVE))
          return intersectorHair.intersect(pre,ray,geom,prim.p0,prim.p1,prim.p2,prim.p3,Occluded1EpilogMU<VSIZEX,true>(ray,context,prim.geomID(),prim.primID()));
        else
          return intersectorCurve.intersect(pre,ray,geom,prim.p0,prim.p1,prim.p2,prim.p3,Occluded1Epilog1<true>(ray,context,prim.geomID(),prim.primID()));
      }
    };

    /*! Intersector for a single ray from a ray packet with a bezier curve. */
    template<int K>
      struct Bezier1vIntersectorK
    {
      typedef Bezier1v Primitive;
      typedef CurvePrecalculationsK<K> Precalculations;
      
      static __forceinline void intersect(Precalculations& pre, RayHitK<K>& ray, const size_t k, IntersectContext* context, const Primitive& prim) 
      {
        STAT3(normal.trav_prims,1,1,1);
        Bezier1IntersectorK<Curve3fa,K> intersectorHair;
        BezierCurve1IntersectorK<Curve3fa,K> intersectorCurve;
        const NativeCurves* geom = (NativeCurves*)context->scene->get(prim.geomID());
        if (likely(geom->subtype == FLAT_CURVE))
          intersectorHair.intersect(pre,ray,k,geom,prim.p0,prim.p1,prim.p2,prim.p3,Intersect1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID()));
        else
          intersectorCurve.intersect(pre,ray,k,geom,prim.p0,prim.p1,prim.p2,prim.p3,Intersect1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID()));
      }

      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const Primitive& prim)
      {
        size_t mask = movemask(valid_i);
        while (mask) intersect(pre,ray,__bscf(mask),context,prim);
      }
 
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, const size_t k, IntersectContext* context, const Primitive& prim)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Bezier1IntersectorK<Curve3fa,K> intersectorHair;
        BezierCurve1IntersectorK<Curve3fa,K> intersectorCurve;
        const NativeCurves* geom = (NativeCurves*)context->scene->get(prim.geomID());
         if (likely(geom->subtype == FLAT_CURVE))
           return intersectorHair.intersect(pre,ray,k,geom,prim.p0,prim.p1,prim.p2,prim.p3,Occluded1KEpilogMU<VSIZEX,K,true>(ray,k,context,prim.geomID(),prim.primID()));
         else
           return intersectorCurve.intersect(pre,ray,k,geom,prim.p0,prim.p1,prim.p2,prim.p3,Occluded1KEpilog1<K,true>(ray,k,context,prim.geomID(),prim.primID()));
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
