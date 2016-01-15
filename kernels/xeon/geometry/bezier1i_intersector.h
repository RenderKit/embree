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

#include "bezier1i.h"
#include "bezier_intersector.h"

namespace embree
{
  namespace isa
  {
    struct Bezier1iIntersector1
    {
      typedef Bezier1i Primitive;
      typedef Bezier1Intersector1 Precalculations;
      
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        const BezierCurves* in = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = in->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = in->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = in->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = in->vertex(prim.vertexID+3,0);
        pre.intersect(ray,a0,a1,a2,a3,Intersect1EpilogU<4,true>(ray,prim.geomID(),prim.primID(),scene,geomID_to_instID));
      }
      
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const BezierCurves* in = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = in->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = in->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = in->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = in->vertex(prim.vertexID+3,0);
        return pre.intersect(ray,a0,a1,a2,a3,Occluded1EpilogU<4,true>(ray,prim.geomID(),prim.primID(),scene,geomID_to_instID));
      }
    };

    template<int K>
      struct Bezier1iIntersectorK
    {
      typedef Bezier1i Primitive;
      typedef Bezier1IntersectorK<K> Precalculations;
      
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, const size_t k, const Primitive& curve, Scene* scene)
      {
        STAT3(normal.trav_prims,1,1,1);
        const BezierCurves* in = (BezierCurves*) scene->get(curve.geomID());
        const Vec3fa a0 = in->vertex(curve.vertexID+0,0);
        const Vec3fa a1 = in->vertex(curve.vertexID+1,0);
        const Vec3fa a2 = in->vertex(curve.vertexID+2,0);
        const Vec3fa a3 = in->vertex(curve.vertexID+3,0);
        pre.intersect(ray,k,a0,a1,a2,a3,Intersect1KEpilogU<4,K,true>(ray,k,curve.geomID(),curve.primID(),scene));
      }
      
      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive& curve, Scene* scene)
      {
        int mask = movemask(valid_i);
        while (mask) intersect(pre,ray,__bscf(mask),curve,scene);
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, const size_t k, const Primitive& curve, Scene* scene)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const BezierCurves* in = (BezierCurves*) scene->get(curve.geomID());
        const Vec3fa a0 = in->vertex(curve.vertexID+0,0);
        const Vec3fa a1 = in->vertex(curve.vertexID+1,0);
        const Vec3fa a2 = in->vertex(curve.vertexID+2,0);
        const Vec3fa a3 = in->vertex(curve.vertexID+3,0);
        return pre.intersect(ray,k,a0,a1,a2,a3,Occluded1KEpilogU<4,K,true>(ray,k,curve.geomID(),curve.primID(),scene));
      }
      
      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive& curve, Scene* scene)
      {
        vbool<K> valid_o = false;
        int mask = movemask(valid_i);
        while (mask) {
          size_t k = __bscf(mask);
          if (occluded(pre,ray,k,curve,scene))
            set(valid_o, k);
        }
        return valid_o;
      }
    };
    
    struct Bezier1iIntersector1MB
    {
      typedef Bezier1i Primitive;
      typedef Bezier1Intersector1 Precalculations;
      
      static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        const BezierCurves* in = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = in->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = in->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = in->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = in->vertex(prim.vertexID+3,0);
        const Vec3fa b0 = in->vertex(prim.vertexID+0,1);
        const Vec3fa b1 = in->vertex(prim.vertexID+1,1);
        const Vec3fa b2 = in->vertex(prim.vertexID+2,1);
        const Vec3fa b3 = in->vertex(prim.vertexID+3,1);
        const float t0 = 1.0f-ray.time, t1 = ray.time;
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        pre.intersect(ray,p0,p1,p2,p3,Intersect1EpilogU<4,true>(ray,prim.geomID(),prim.primID(),scene,geomID_to_instID));
      }
      
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive& prim, Scene* scene, const unsigned* geomID_to_instID) 
      {
        STAT3(shadow.trav_prims,1,1,1);
        const BezierCurves* in = (BezierCurves*) scene->get(prim.geomID());
        const Vec3fa a0 = in->vertex(prim.vertexID+0,0);
        const Vec3fa a1 = in->vertex(prim.vertexID+1,0);
        const Vec3fa a2 = in->vertex(prim.vertexID+2,0);
        const Vec3fa a3 = in->vertex(prim.vertexID+3,0);
        const Vec3fa b0 = in->vertex(prim.vertexID+0,1);
        const Vec3fa b1 = in->vertex(prim.vertexID+1,1);
        const Vec3fa b2 = in->vertex(prim.vertexID+2,1);
        const Vec3fa b3 = in->vertex(prim.vertexID+3,1);
        const float t0 = 1.0f-ray.time, t1 = ray.time;
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        return pre.intersect(ray,p0,p1,p2,p3,Occluded1EpilogU<4,true>(ray,prim.geomID(),prim.primID(),scene,geomID_to_instID));
      }
    };

    template<int K>
    struct Bezier1iIntersectorKMB
    {
      typedef Bezier1i Primitive;
      typedef Bezier1IntersectorK<K> Precalculations;
      
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, const size_t k, const Primitive& curve, Scene* scene)
      {
        STAT3(normal.trav_prims,1,1,1);
        const BezierCurves* in = (BezierCurves*) scene->get(curve.geomID());
        const Vec3fa a0 = in->vertex(curve.vertexID+0,0);
        const Vec3fa a1 = in->vertex(curve.vertexID+1,0);
        const Vec3fa a2 = in->vertex(curve.vertexID+2,0);
        const Vec3fa a3 = in->vertex(curve.vertexID+3,0);
        const Vec3fa b0 = in->vertex(curve.vertexID+0,1);
        const Vec3fa b1 = in->vertex(curve.vertexID+1,1);
        const Vec3fa b2 = in->vertex(curve.vertexID+2,1);
        const Vec3fa b3 = in->vertex(curve.vertexID+3,1);
        const float t0 = 1.0f-ray.time[k], t1 = ray.time[k];
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        pre.intersect(ray,k,p0,p1,p2,p3,Intersect1KEpilogU<4,K,true>(ray,k,curve.geomID(),curve.primID(),scene));
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, const size_t k, const Primitive& curve, Scene* scene)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const BezierCurves* in = (BezierCurves*) scene->get(curve.geomID());
        const Vec3fa a0 = in->vertex(curve.vertexID+0,0);
        const Vec3fa a1 = in->vertex(curve.vertexID+1,0);
        const Vec3fa a2 = in->vertex(curve.vertexID+2,0);
        const Vec3fa a3 = in->vertex(curve.vertexID+3,0);
        const Vec3fa b0 = in->vertex(curve.vertexID+0,1);
        const Vec3fa b1 = in->vertex(curve.vertexID+1,1);
        const Vec3fa b2 = in->vertex(curve.vertexID+2,1);
        const Vec3fa b3 = in->vertex(curve.vertexID+3,1);
        const float t0 = 1.0f-ray.time[k], t1 = ray.time[k];
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        return pre.intersect(ray,k,p0,p1,p2,p3,Occluded1KEpilogU<4,K,true>(ray,k,curve.geomID(),curve.primID(),scene));
      }
    };
  }
}
