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

#include "bezierNv.h"
#include "bezierNi_intersector.h"

namespace embree
{
  namespace isa
  {
    template<int M>
      struct BezierNvIntersector1 : public BezierNiIntersector1<M>
    {
      typedef BezierNv<M> Primitive;
      using typename BezierNiIntersector1<M>::Precalculations;

      static __forceinline void intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive& prim)
      {
        vfloat<M> tNear;
        vbool<M> valid = BezierNiIntersector1<M>::intersect(ray,prim,tNear);

        const size_t N = prim.N;
        size_t mask = movemask(valid);
        while (mask)
        {
          const size_t i = __bscf(mask);
          STAT3(normal.trav_prims,1,1,1);
          const unsigned int geomID = prim.geomID(N);
          const unsigned int primID = prim.primID(N)[i];
          const NativeCurves* geom = (NativeCurves*) context->scene->get(geomID);
          const Vec3fa a0 = Vec3fa::loadu(&prim.vertices(i,N)[0]);
          const Vec3fa a1 = Vec3fa::loadu(&prim.vertices(i,N)[1]);
          const Vec3fa a2 = Vec3fa::loadu(&prim.vertices(i,N)[2]);
          const Vec3fa a3 = Vec3fa::loadu(&prim.vertices(i,N)[3]);

          size_t mask1 = mask;
          const size_t i1 = __bscf(mask1);
          if (mask) {
            prefetchL1(&prim.vertices(i1,N)[0]);
            prefetchL1(&prim.vertices(i1,N)[4]);
            if (mask1) {
              const size_t i2 = __bsf(mask1);
              prefetchL2(&prim.vertices(i2,N)[0]);
              prefetchL2(&prim.vertices(i2,N)[4]);
            }
          }
          
          if (likely(geom->subtype == FLAT_CURVE))
            pre.intersectorHair.intersect(ray,a0,a1,a2,a3,geom->tessellationRate,Intersect1EpilogMU<VSIZEX,true>(ray,context,geomID,primID));
          else 
            pre.intersectorCurve.intersect(ray,a0,a1,a2,a3,Intersect1Epilog1<true>(ray,context,geomID,primID));

          mask &= movemask(tNear <= vfloat<M>(ray.tfar));
        }
      }
      
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& prim)
      {
        vfloat<M> tNear;
        vbool<M> valid = BezierNiIntersector1<M>::intersect(ray,prim,tNear);

        const size_t N = prim.N;
        size_t mask = movemask(valid);
        while (mask)
        {
          const size_t i = __bscf(mask);
          STAT3(shadow.trav_prims,1,1,1);
          const unsigned int geomID = prim.geomID(N);
          const unsigned int primID = prim.primID(N)[i];
          const NativeCurves* geom = (NativeCurves*) context->scene->get(geomID);
          const Vec3fa a0 = Vec3fa::loadu(&prim.vertices(i,N)[0]);
          const Vec3fa a1 = Vec3fa::loadu(&prim.vertices(i,N)[1]);
          const Vec3fa a2 = Vec3fa::loadu(&prim.vertices(i,N)[2]);
          const Vec3fa a3 = Vec3fa::loadu(&prim.vertices(i,N)[3]);

          size_t mask1 = mask;
          const size_t i1 = __bscf(mask1);
          if (mask) {
            prefetchL1(&prim.vertices(i1,N)[0]);
            prefetchL1(&prim.vertices(i1,N)[4]);
            if (mask1) {
              const size_t i2 = __bsf(mask1);
              prefetchL2(&prim.vertices(i2,N)[0]);
              prefetchL2(&prim.vertices(i2,N)[4]);
            }
          }
          
          if (likely(geom->subtype == FLAT_CURVE)) {
            if (pre.intersectorHair.intersect(ray,a0,a1,a2,a3,geom->tessellationRate,Occluded1EpilogMU<VSIZEX,true>(ray,context,geomID,primID)))
              return true;
          } else {
            if (pre.intersectorCurve.intersect(ray,a0,a1,a2,a3,Occluded1Epilog1<true>(ray,context,geomID,primID)))
              return true;
          }

          mask &= movemask(tNear <= vfloat<M>(ray.tfar));
        }
        return false;
      }
    };

    template<int M, int K>
      struct BezierNvIntersectorK : public BezierNiIntersectorK<M,K>
    {
      typedef BezierNv<M> Primitive;

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

      static __forceinline void intersect(Precalculations& pre, RayHitK<K>& ray, const size_t k, IntersectContext* context, const Primitive& prim)
      {
        vfloat<M> tNear;
        vbool<M> valid = BezierNiIntersectorK<M,K>::intersect(ray,k,prim,tNear);

        const size_t N = prim.N;
        size_t mask = movemask(valid);
        while (mask)
        {
          const size_t i = __bscf(mask);
          STAT3(normal.trav_prims,1,1,1);
          const unsigned int geomID = prim.geomID(N);
          const unsigned int primID = prim.primID(N)[i];
          const NativeCurves* geom = (NativeCurves*) context->scene->get(geomID);
          const Vec3fa a0 = Vec3fa::loadu(&prim.vertices(i,N)[0]);
          const Vec3fa a1 = Vec3fa::loadu(&prim.vertices(i,N)[1]);
          const Vec3fa a2 = Vec3fa::loadu(&prim.vertices(i,N)[2]);
          const Vec3fa a3 = Vec3fa::loadu(&prim.vertices(i,N)[3]);

          size_t mask1 = mask;
          const size_t i1 = __bscf(mask1);
          if (mask) {
            prefetchL1(&prim.vertices(i1,N)[0]);
            prefetchL1(&prim.vertices(i1,N)[4]);
            if (mask1) {
              const size_t i2 = __bsf(mask1);
              prefetchL2(&prim.vertices(i2,N)[0]);
              prefetchL2(&prim.vertices(i2,N)[4]);
            }
          }
          
          if (likely(geom->subtype == FLAT_CURVE))
            pre.intersectorHair.intersect(ray,k,a0,a1,a2,a3,geom->tessellationRate,Intersect1KEpilogMU<VSIZEX,K,true>(ray,k,context,geomID,primID));
          else 
            pre.intersectorCurve.intersect(ray,k,a0,a1,a2,a3,Intersect1KEpilog1<K,true>(ray,k,context,geomID,primID));

          mask &= movemask(tNear <= vfloat<M>(ray.tfar[k]));
        }
      }
      
      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const Primitive& prim)
      {
        size_t mask = movemask(valid_i);
        while (mask) intersect(pre,ray,__bscf(mask),context,prim);
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, const size_t k, IntersectContext* context, const Primitive& prim)
      {
        vfloat<M> tNear;
        vbool<M> valid = BezierNiIntersectorK<M,K>::intersect(ray,k,prim,tNear);

        const size_t N = prim.N;
        size_t mask = movemask(valid);
        while (mask)
        {
          const size_t i = __bscf(mask);
          STAT3(shadow.trav_prims,1,1,1);
          const unsigned int geomID = prim.geomID(N);
          const unsigned int primID = prim.primID(N)[i];
          const NativeCurves* geom = (NativeCurves*) context->scene->get(geomID);
          const Vec3fa a0 = Vec3fa::loadu(&prim.vertices(i,N)[0]);
          const Vec3fa a1 = Vec3fa::loadu(&prim.vertices(i,N)[1]);
          const Vec3fa a2 = Vec3fa::loadu(&prim.vertices(i,N)[2]);
          const Vec3fa a3 = Vec3fa::loadu(&prim.vertices(i,N)[3]);

          size_t mask1 = mask;
          const size_t i1 = __bscf(mask1);
          if (mask) {
            prefetchL1(&prim.vertices(i1,N)[0]);
            prefetchL1(&prim.vertices(i1,N)[4]);
            if (mask1) {
              const size_t i2 = __bsf(mask1);
              prefetchL2(&prim.vertices(i2,N)[0]);
              prefetchL2(&prim.vertices(i2,N)[4]);
            }
          }
                     
          if (likely(geom->subtype == FLAT_CURVE)) {
            if (pre.intersectorHair.intersect(ray,k,a0,a1,a2,a3,geom->tessellationRate,Occluded1KEpilogMU<VSIZEX,K,true>(ray,k,context,geomID,primID)))
              return true;
          } else {
            if (pre.intersectorCurve.intersect(ray,k,a0,a1,a2,a3,Occluded1KEpilog1<K,true>(ray,k,context,geomID,primID)))
              return true;
          }

          mask &= movemask(tNear <= vfloat<M>(ray.tfar[k]));
        }
        return false;
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
