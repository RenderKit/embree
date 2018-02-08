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
#include "bezier_hair_intersector.h"
#include "bezier_ribbon_intersector.h"
#include "bezier_curve_intersector.h"

namespace embree
{
  namespace isa
  {
    struct BezierNvIntersector1
    {
      typedef BezierNv Primitive;

      struct Precalculations
      {
        __forceinline Precalculations() {}

        __forceinline Precalculations(const Ray& ray, const void* ptr)
          : intersectorHair(ray,ptr), intersectorCurve(ray,ptr) {}

        Bezier1Intersector1<Curve3fa> intersectorHair;
        BezierCurve1Intersector1<Curve3fa> intersectorCurve;
      };

      static __forceinline vbool8 intersect(Ray& ray, const Primitive& prim, vfloat8& tNear_o)
      {
#if EMBREE_HAIR_LEAF_MODE == 0

        const size_t N = prim.N;
        STAT(if (N>1) STAT3(normal.trav_leaves,-1,-1,-1));
        STAT(if (N>1) STAT3(normal.trav_nodes,1,1,1));
        STAT(if (N<2) STAT3(normal.trav_prims,1,1,1));
        
        AffineSpace3vf<8> space(Vec3vf8(vfloat<8>::loadu(prim.vx_x(N)),vfloat<8>::loadu(prim.vx_y(N)),vfloat<8>::loadu(prim.vx_z(N))),
                                Vec3vf8(vfloat<8>::loadu(prim.vy_x(N)),vfloat<8>::loadu(prim.vy_y(N)),vfloat<8>::loadu(prim.vy_z(N))),
                                Vec3vf8(vfloat<8>::loadu(prim.vz_x(N)),vfloat<8>::loadu(prim.vz_y(N)),vfloat<8>::loadu(prim.vz_z(N))),
                                Vec3vf8(vfloat<8>::loadu(prim.p_x (N)),vfloat<8>::loadu(prim.p_y (N)),vfloat<8>::loadu(prim.p_z (N))));
        
        const Vec3vf8 dir1 = xfmVector(space,Vec3vf8(ray.dir));
        const Vec3vf8 org1 = xfmPoint (space,Vec3vf8(ray.org));
        const Vec3vf8 nrcp_dir1 = -rcp_safe(dir1);
        
        const vfloat8 t_lower_x = org1.x*nrcp_dir1.x;
        const vfloat8 t_lower_y = org1.y*nrcp_dir1.y;
        const vfloat8 t_lower_z = org1.z*nrcp_dir1.z;
        const vfloat8 t_upper_x = t_lower_x - nrcp_dir1.x;
        const vfloat8 t_upper_y = t_lower_y - nrcp_dir1.y;
        const vfloat8 t_upper_z = t_lower_z - nrcp_dir1.z;

        const vfloat8 tNear = max(mini(t_lower_x,t_upper_x),mini(t_lower_y,t_upper_y),mini(t_lower_z,t_upper_z),vfloat8(ray.tnear()));
        const vfloat8 tFar  = min(maxi(t_lower_x,t_upper_x),maxi(t_lower_y,t_upper_y),maxi(t_lower_z,t_upper_z),vfloat8(ray.tfar));
        tNear_o = tNear;
        return (vint8(step) < vint8(prim.N)) & (tNear <= tFar);

#endif

#if EMBREE_HAIR_LEAF_MODE == 1

        const size_t N = prim.N;
        STAT(if (N>1) STAT3(normal.trav_leaves,-1,-1,-1));
        STAT(if (N>1) STAT3(normal.trav_nodes,1,1,1));
        STAT(if (N<2) STAT3(normal.trav_prims,1,1,1));
        
        const AffineSpace3fa space(Vec3fa::loadu(&prim.space.l.vx),
                                   Vec3fa::loadu(&prim.space.l.vy),
                                   Vec3fa::loadu(&prim.space.l.vz),
                                   Vec3fa::loadu(&prim.space.p));
        
        const Vec3fa org1 = xfmPoint (space,ray.org);
        const Vec3fa dir1 = xfmVector(space,ray.dir);
        const Vec3fa rcp_dir1 = rcp_safe(dir1);
                
        const vfloat8 t_lower_x = (vfloat8::load(prim.lower_x(N))-vfloat8(org1.x))*vfloat8(rcp_dir1.x);
        const vfloat8 t_upper_x = (vfloat8::load(prim.upper_x(N))-vfloat8(org1.x))*vfloat8(rcp_dir1.x);
        const vfloat8 t_lower_y = (vfloat8::load(prim.lower_y(N))-vfloat8(org1.y))*vfloat8(rcp_dir1.y);
        const vfloat8 t_upper_y = (vfloat8::load(prim.upper_y(N))-vfloat8(org1.y))*vfloat8(rcp_dir1.y);
        const vfloat8 t_lower_z = (vfloat8::load(prim.lower_z(N))-vfloat8(org1.z))*vfloat8(rcp_dir1.z);
        const vfloat8 t_upper_z = (vfloat8::load(prim.upper_z(N))-vfloat8(org1.z))*vfloat8(rcp_dir1.z);

        const vfloat8 tNear = max(mini(t_lower_x,t_upper_x),mini(t_lower_y,t_upper_y),mini(t_lower_z,t_upper_z),vfloat8(ray.tnear()));
        const vfloat8 tFar  = min(maxi(t_lower_x,t_upper_x),maxi(t_lower_y,t_upper_y),maxi(t_lower_z,t_upper_z),vfloat8(ray.tfar));
        tNear_o = tNear;
        return (vint8(step) < vint8(prim.N)) & (tNear <= tFar);
#endif

#if EMBREE_HAIR_LEAF_MODE == 2

        const size_t N = prim.N;
        STAT(if (N>1) STAT3(normal.trav_leaves,-1,-1,-1));
        STAT(if (N>1) STAT3(normal.trav_nodes,1,1,1));
        STAT(if (N<2) STAT3(normal.trav_prims,1,1,1));

        const vfloat4 offset_scale = vfloat4::loadu(prim.offset(N));
        const Vec3fa offset = Vec3fa(offset_scale);
        const Vec3fa scale = Vec3fa(shuffle<3,3,3,3>(offset_scale));
        const Vec3fa org1 = (ray.org-offset)*scale;
        const Vec3fa dir1 = ray.dir*scale;
        
        const LinearSpace3vf8 space(vfloat8::load(prim.bounds_vx_x(N)), vfloat8::load(prim.bounds_vx_y(N)), vfloat8::load(prim.bounds_vx_z(N)),
                                    vfloat8::load(prim.bounds_vy_x(N)), vfloat8::load(prim.bounds_vy_y(N)), vfloat8::load(prim.bounds_vy_z(N)),
                                    vfloat8::load(prim.bounds_vz_x(N)), vfloat8::load(prim.bounds_vz_y(N)), vfloat8::load(prim.bounds_vz_z(N)));

        const Vec3vf8 dir2 = xfmVector(space,Vec3vf8(dir1));
        const Vec3vf8 org2 = xfmPoint (space,Vec3vf8(org1));
        const Vec3vf8 rcp_dir2 = rcp_safe(dir2);
       
        const vfloat8 t_lower_x = (vfloat8::load(prim.bounds_vx_lower(N))-vfloat8(org2.x))*vfloat8(rcp_dir2.x);
        const vfloat8 t_upper_x = (vfloat8::load(prim.bounds_vx_upper(N))-vfloat8(org2.x))*vfloat8(rcp_dir2.x);
        const vfloat8 t_lower_y = (vfloat8::load(prim.bounds_vy_lower(N))-vfloat8(org2.y))*vfloat8(rcp_dir2.y);
        const vfloat8 t_upper_y = (vfloat8::load(prim.bounds_vy_upper(N))-vfloat8(org2.y))*vfloat8(rcp_dir2.y);
        const vfloat8 t_lower_z = (vfloat8::load(prim.bounds_vz_lower(N))-vfloat8(org2.z))*vfloat8(rcp_dir2.z);
        const vfloat8 t_upper_z = (vfloat8::load(prim.bounds_vz_upper(N))-vfloat8(org2.z))*vfloat8(rcp_dir2.z);

        const vfloat8 tNear = max(mini(t_lower_x,t_upper_x),mini(t_lower_y,t_upper_y),mini(t_lower_z,t_upper_z),vfloat8(ray.tnear()));
        const vfloat8 tFar  = min(maxi(t_lower_x,t_upper_x),maxi(t_lower_y,t_upper_y),maxi(t_lower_z,t_upper_z),vfloat8(ray.tfar));
        tNear_o = tNear;
        return (vint8(step) < vint8(prim.N)) & (tNear <= tFar);
#endif
      }

      static __forceinline void intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive& prim)
      {
        vfloat8 tNear;
        vbool8 valid = intersect(ray,prim,tNear);

        const size_t N = prim.N;
        size_t mask = movemask(valid);
        while (mask)
        {
          const size_t i = __bscf(mask);
          STAT(if (N>1) STAT3(normal.trav_leaves,1,1,1));
          STAT(if (N>1) STAT3(normal.trav_prims,1,1,1))
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

          mask &= movemask(tNear <= vfloat8(ray.tfar));
        }
      }
      
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive& prim)
      {
        vfloat8 tNear;
        vbool8 valid = intersect(ray,prim,tNear);

        const size_t N = prim.N;
        size_t mask = movemask(valid);
        while (mask)
        {
          const size_t i = __bscf(mask);
          STAT(if (N>1) STAT3(shadow.trav_leaves,1,1,1));
          STAT(if (N>1) STAT3(shadow.trav_prims,1,1,1))
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

          mask &= movemask(tNear <= vfloat8(ray.tfar));
        }
        return false;
      }
    };
  }
}
