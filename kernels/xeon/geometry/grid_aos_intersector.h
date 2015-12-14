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

#include "grid_aos.h"
#include "../../common/ray.h"
#include "../geometry/filter.h"

namespace embree
{
  namespace isa
  {
    template<int K>
    struct GridAOSIntersectorK
    {
      static const int N = (VSIZEX == 4) ? 4 : 8;
      typedef Vec3<vfloat<N>> Vec3vfN;

      typedef GridAOS::EagerLeaf Primitive;

      struct Precalculations {
        __forceinline Precalculations (const vbool<K>& valid, const RayK<K>& ray) {}
      };

      struct Precalculations1
      {
        __forceinline Precalculations1 (const RayK<K>& ray, size_t k)
        {
          /*! load the ray into SIMD registers */
          const Vec3fa ray_org = Vec3fa(ray.org.x[k],ray.org.y[k],ray.org.z[k]);
          const Vec3fa ray_dir = Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
          const Vec3fa ray_rdir = rcp_safe(ray_dir);
          const Vec3fa ray_org_rdir = ray_org*ray_rdir;
          org_xyz = ray_org;
          dir_xyz = ray_dir;
          org = Vec3vfN(ray.org.x[k],ray.org.y[k],ray.org.z[k]);
          rdir = Vec3vfN(ray_rdir.x,ray_rdir.y,ray_rdir.z);
          org_rdir = Vec3vfN(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
          ray_tnear = ray.tnear[k];

          /*! offsets to select the side that becomes the lower or upper bound */
          nearX = ray_rdir.x >= 0.0f ? 0*sizeof(vfloat4) : 1*sizeof(vfloat4);
          nearY = ray_rdir.y >= 0.0f ? 2*sizeof(vfloat4) : 3*sizeof(vfloat4);
          nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(vfloat4) : 5*sizeof(vfloat4);
        }

        Vec3fa org_xyz, dir_xyz;
        Vec3vfN org, rdir, org_rdir;
        vfloat<N> ray_tnear;
        size_t nearX, nearY, nearZ;
      };

      static __forceinline void intersectFinish (const Precalculations1& pre, RayK<K>& ray, size_t k, const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const vfloat4& uvw, const Primitive& prim, Scene* scene)
      {
        const Vec3fa Ng0 = cross(p1-p0,p2-p0);
        const Vec3fa Ng = Ng0+Ng0;
        const float det = dot(pre.dir_xyz,Ng);
        const float rcpDet = rcp(det);
        const float T   = dot(p0,Ng);
        const float t = T*rcpDet;
        if (unlikely(ray.tnear[k] <= t && t <= ray.tfar[k]))
        {
          const float rcp0xFFFF = 1.0f/0xFFFF;
          const Vec3fa uv0 = Vec3fa(p2.u & 0xFFFF, p2.u >> 16, 0.0f)*rcp0xFFFF;
          const Vec3fa uv1 = Vec3fa(p0.u & 0xFFFF, p0.u >> 16, 0.0f)*rcp0xFFFF;
          const Vec3fa uv2 = Vec3fa(p1.u & 0xFFFF, p1.u >> 16, 0.0f)*rcp0xFFFF;
          const Vec3fa uv = uvw[0]*uv0+uvw[1]*uv1+uvw[2]*uv2;
          const float u = uv.x * rcpDet;
          const float v = uv.y * rcpDet;

          /* ray masking test */
          Geometry* geometry = scene->get(prim.grid.geomID);
#if defined(RTCORE_RAY_MASK)
          if ((geometry->mask & ray.mask[k]) == 0) return;
#endif

          /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
          if (unlikely(geometry->hasIntersectionFilter<vfloat<K>>())) {
            runIntersectionFilter(geometry,ray,k,u,v,t,Ng,prim.grid.geomID,prim.grid.primID);
            return;
          }
#endif
          ray.u[k]      = u;
          ray.v[k]      = v;
          //ray.u[k]    = uvw[0] * rcpDet;
          //ray.v[k]    = uvw[1] * rcpDet;
          ray.tfar[k]   = t;
          ray.Ng.x[k]   = Ng.x;
          ray.Ng.y[k]   = Ng.y;
          ray.Ng.z[k]   = Ng.z;
          ray.geomID[k] = prim.grid.geomID;
          ray.primID[k] = prim.grid.primID;
        }
      }

      __forceinline static void intersectQuad(const Precalculations1& pre, RayK<K>& ray, size_t k, const Vec3fa& O, const Vec3fa& D,
                                              const Vec3fa& q00, const Vec3fa& q01,
                                              const Vec3fa& q10, const Vec3fa& q11,
                                              const Primitive& prim, Scene* scene)
      {
        const Vec3vf4 DDDD(D.x,D.y,D.z);
        Vec3vf4 p00; transpose((vfloat4)q00,(vfloat4)q01,(vfloat4)q11,(vfloat4)q10,p00.x,p00.y,p00.z);

        const Vec3vf4 t000_start = shuffle<0,1,3,0>(p00), t000_end = shuffle<1,3,0,0>(p00);
        const Vec3vf4 e000 = t000_end - t000_start;
        const Vec3vf4 s000 = t000_end + t000_start;
        const vfloat4  u000 = dot(cross(s000,e000),DDDD);
        if (all(ge_mask(Vec3fa(u000),Vec3fa(0.0f))) || all(le_mask(Vec3fa(u000),Vec3fa(0.0f))))
          intersectFinish(pre,ray,k,q00,q01,q10,u000,prim,scene);

        const Vec3vf4 t001_start = shuffle<2,3,1,0>(p00), t001_end = shuffle<3,1,2,0>(p00);
        const Vec3vf4 e001 = t001_end - t001_start;
        const Vec3vf4 s001 = t001_end + t001_start;
        const vfloat4  u001 = dot(cross(s001,e001),DDDD);
        if (all(ge_mask(Vec3fa(u001),Vec3fa(0.0f))) || all(le_mask(Vec3fa(u001),Vec3fa(0.0f))))
          intersectFinish(pre,ray,k,q11,q10,q01,u001,prim,scene);
      }

      __forceinline static void intersectDualQuad(const Precalculations1& pre, RayK<K>& ray, size_t k, const Vec3fa& O, const Vec3fa& D,
                                                  const Vec3fa& q00, const Vec3fa& q01,
                                                  const Vec3fa& q10, const Vec3fa& q11,
                                                  const Vec3fa& q20, const Vec3fa& q21,
                                                  const Primitive& prim, Scene* scene)
#if defined(__AVX__)
      {
        const Vec3vf8 D8(D.x,D.y,D.z);

        const vfloat8 q00_q10((vfloat4)q00,(vfloat4)q10);
        const vfloat8 q01_q11((vfloat4)q01,(vfloat4)q11);
        const vfloat8 q11_q21((vfloat4)q11,(vfloat4)q21);
        const vfloat8 q10_q20((vfloat4)q10,(vfloat4)q20);
        Vec3vf8 p00_p10; transpose(q00_q10,q01_q11,q11_q21,q10_q20,p00_p10.x,p00_p10.y,p00_p10.z);

        const Vec3vf8 t000_t100_start = shuffle<0,1,3,0>(p00_p10), t000_t100_end = shuffle<1,3,0,0>(p00_p10);
        const Vec3vf8 e000_e100 = t000_t100_end - t000_t100_start;
        const Vec3vf8 s000_s100 = t000_t100_end + t000_t100_start;
        const vfloat8  u000_u100 = dot(cross(s000_s100,e000_e100),D8);
        if (all(ge_mask(Vec3fa(extract4<0>(u000_u100)),Vec3fa(0.0f))) || all(le_mask(Vec3fa(extract4<0>(u000_u100)),Vec3fa(0.0f))))
          intersectFinish(pre,ray,k,q00,q01,q10,extract4<0>(u000_u100),prim,scene);
        if (all(ge_mask(Vec3fa(extract4<1>(u000_u100)),Vec3fa(0.0f))) || all(le_mask(Vec3fa(extract4<1>(u000_u100)),Vec3fa(0.0f))))
          intersectFinish(pre,ray,k,q10,q11,q20,extract4<1>(u000_u100),prim,scene);

        const Vec3vf8 t001_t101_start = shuffle<2,3,1,0>(p00_p10), t001_t101_end = shuffle<3,1,2,0>(p00_p10);
        const Vec3vf8 e001_e101 = t001_t101_end - t001_t101_start;
        const Vec3vf8 s001_s101 = t001_t101_end + t001_t101_start;
        const vfloat8  u001_u101 = dot(cross(s001_s101,e001_e101),D8);
        if (all(ge_mask(Vec3fa(extract4<0>(u001_u101)),Vec3fa(0.0f))) || all(le_mask(Vec3fa(extract4<0>(u001_u101)),Vec3fa(0.0f))))
          intersectFinish(pre,ray,k,q11,q10,q01,extract4<0>(u001_u101),prim,scene);
        if (all(ge_mask(Vec3fa(extract4<1>(u001_u101)),Vec3fa(0.0f))) || all(le_mask(Vec3fa(extract4<1>(u001_u101)),Vec3fa(0.0f))))
          intersectFinish(pre,ray,k,q21,q20,q11,extract4<1>(u001_u101),prim,scene);
      }
#else
      {
        intersectQuad(pre,ray,k,O,D, q00,q01,q10,q11, prim, scene);
        intersectQuad(pre,ray,k,O,D, q10,q11,q20,q21, prim, scene);
      }
#endif

      static __forceinline void intersectQuad (const Precalculations1& pre, RayK<K>& ray, size_t k,
                                               const Vec3fa& v00, const Vec3fa& v10,
                                               const Vec3fa& v01, const Vec3fa& v11,
                                               const Primitive& prim, Scene* scene)
      {
        const Vec3fa O = pre.org_xyz;
        const Vec3fa D = pre.dir_xyz;
        const Vec3fa q00 = copy_a(v00-O,v00), q10 = copy_a(v10-O,v10);
        const Vec3fa q01 = copy_a(v01-O,v01), q11 = copy_a(v11-O,v11);
        intersectQuad(pre,ray,k,O,D, q00,q01,q10,q11, prim, scene);
      }

      static __forceinline void intersectQuads (const Precalculations1& pre, RayK<K>& ray, size_t k,
                                                const Vec3fa& v00, const Vec3fa& v10, const Vec3fa& v20,
                                                const Vec3fa& v01, const Vec3fa& v11, const Vec3fa& v21,
                                                const Primitive& prim, Scene* scene)
      {
        const Vec3fa O = pre.org_xyz;
        const Vec3fa D = pre.dir_xyz;
        const Vec3fa q00 = copy_a(v00-O,v00), q10 = copy_a(v10-O,v10), q20 = copy_a(v20-O,v20);
        const Vec3fa q01 = copy_a(v01-O,v01), q11 = copy_a(v11-O,v11), q21 = copy_a(v21-O,v21);
        intersectDualQuad(pre,ray,k,O,D,q00,q01,q10,q11,q20,q21,prim,scene);
      }

      static __forceinline void intersectQuads (const Precalculations1& pre, RayK<K>& ray, size_t k,
                                                const Vec3fa& v00, const Vec3fa& v10, const Vec3fa& v20,
                                                const Vec3fa& v01, const Vec3fa& v11, const Vec3fa& v21,
                                                const Vec3fa& v02, const Vec3fa& v12, const Vec3fa& v22,
                                                const Primitive& prim, Scene* scene)
      {
        const Vec3fa O = pre.org_xyz;
        const Vec3fa D = pre.dir_xyz;
        const Vec3fa q00 = copy_a(v00-O,v00), q10 = copy_a(v10-O,v10), q20 = copy_a(v20-O,v20);
        const Vec3fa q01 = copy_a(v01-O,v01), q11 = copy_a(v11-O,v11), q21 = copy_a(v21-O,v21);
        const Vec3fa q02 = copy_a(v02-O,v02), q12 = copy_a(v12-O,v12), q22 = copy_a(v22-O,v22);
        intersectDualQuad(pre,ray,k,O,D,q00,q01,q10,q11,q20,q21,prim,scene);
        intersectDualQuad(pre,ray,k,O,D,q01,q02,q11,q12,q21,q22,prim,scene);
      }

      /*! Intersect a ray with the triangle and updates the hit. */
      static __forceinline void intersect(const Precalculations1& pre, RayK<K>& ray, size_t k, const Primitive& prim, Scene* scene)
      {
        STAT3(normal.trav_prims,1,1,1);

        /* perform box tests */
        const vfloat<N> ray_tfar(ray.tfar[k]);
        size_t mask = prim.bounds.intersect<false>(pre.nearX, pre.nearY, pre.nearZ, pre.org, pre.rdir, pre.org_rdir, pre.ray_tnear, ray_tfar);

        /* intersect quad-quads */
        while (mask)
        {
          const size_t i = __bscf(mask);
          const size_t ofs = prim.quads[i].ofs;
          switch (prim.quads[i].type) {
          case GridAOS::EagerLeaf::Quads::QUAD1X1: {
            const Vec3fa& v00 = prim.grid.point(ofs,0), v10 = (&v00)[1];
            const Vec3fa& v01 = prim.grid.point(ofs,1), v11 = (&v01)[1];
            intersectQuad(pre,ray,k, v00,v10,v01,v11, prim, scene);
            break;
          }
          case GridAOS::EagerLeaf::Quads::QUAD1X2: {
            const Vec3fa& v00 = prim.grid.point(ofs,0), v10 = (&v00)[1];
            const Vec3fa& v01 = prim.grid.point(ofs,1), v11 = (&v01)[1];
            const Vec3fa& v02 = prim.grid.point(ofs,2), v12 = (&v02)[1];
            intersectQuads(pre,ray,k, v10,v11,v12, v00,v01,v02, prim, scene);
            break;
          }
          case GridAOS::EagerLeaf::Quads::QUAD2X1: {
            const Vec3fa& v00 = prim.grid.point(ofs,0), v10 = (&v00)[1], v20 = (&v00)[2];
            const Vec3fa& v01 = prim.grid.point(ofs,1), v11 = (&v01)[1], v21 = (&v01)[2];
            intersectQuads(pre,ray,k, v00,v10,v20,v01,v11,v21, prim, scene);
            break;
          }
          case GridAOS::EagerLeaf::Quads::QUAD2X2: {
            const Vec3fa& v00 = prim.grid.point(ofs,0), v10 = (&v00)[1], v20 = (&v00)[2];
            const Vec3fa& v01 = prim.grid.point(ofs,1), v11 = (&v01)[1], v21 = (&v01)[2];
            const Vec3fa& v02 = prim.grid.point(ofs,2), v12 = (&v02)[1], v22 = (&v02)[2];
            intersectQuads(pre,ray,k, v00,v10,v20,v01,v11,v21,v02,v12,v22, prim, scene);
            break;
          }
          default: assert(false);
          }
        }
      }

      /*! Intersect a ray with the triangle and updates the hit. */
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
      {
        Precalculations1 pre1(ray, k);
        intersect(pre1, ray, k, prim[0], scene);
      }

      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
      {
        int mask = movemask(valid_i);
        while (mask) intersect(pre, ray, __bscf(mask), prim, num, scene, lazy_node);
      }

      static __forceinline bool occludedFinish (const Precalculations1& pre, RayK<K>& ray, size_t k, const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const vfloat4& uvw, const Primitive& prim, Scene* scene)
      {
        const Vec3fa Ng0 = cross(p2-p0,p1-p0);
        const Vec3fa Ng = Ng0+Ng0;
        const float det = dot(pre.dir_xyz,Ng);
        const float rcpDet = rcp(det);
        const float T   = dot(p0,Ng);
        const float t = T*rcpDet;
        if (ray.tnear[k] > t || t > ray.tfar[k]) return false;

        /* ray masking test */
        Geometry* geometry = scene->get(prim.grid.geomID);
#if defined(RTCORE_RAY_MASK)
        if ((geometry->mask & ray.mask[k]) == 0) return false;
#endif

        /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
        if (unlikely(geometry->hasOcclusionFilter<vfloat<K>>()))
        {
          // calculate hit information
          const float rcp0xFFFF = 1.0f/0xFFFF;
          const Vec3fa uv0 = Vec3fa(p2.u & 0xFFFF, p2.u >> 16, 0.0f)*rcp0xFFFF;
          const Vec3fa uv1 = Vec3fa(p0.u & 0xFFFF, p0.u >> 16, 0.0f)*rcp0xFFFF;
          const Vec3fa uv2 = Vec3fa(p1.u & 0xFFFF, p1.u >> 16, 0.0f)*rcp0xFFFF;
          const Vec3fa uv = uvw[0]*uv0+uvw[1]*uv1+uvw[2]*uv2;
          const float u = uv.x * rcpDet;
          const float v = uv.y * rcpDet;
          return runOcclusionFilter(geometry,ray,k,u,v,t,Ng,prim.grid.geomID,prim.grid.primID);
        }
#endif
        return true;
      }

      __forceinline static bool occludedQuad(const Precalculations1& pre, RayK<K>& ray, size_t k, const Vec3fa& O, const Vec3fa& D,
                                             const Vec3fa& q00, const Vec3fa& q01,
                                             const Vec3fa& q10, const Vec3fa& q11,
                                             const Primitive& prim, Scene* scene)
      {
        const Vec3vf4 DDDD(D.x,D.y,D.z);
        Vec3vf4 p00; transpose((vfloat4)q00,(vfloat4)q01,(vfloat4)q11,(vfloat4)q10,p00.x,p00.y,p00.z);

        const Vec3vf4 t000_start = shuffle<0,1,3,0>(p00), t000_end = shuffle<1,3,0,0>(p00);
        const Vec3vf4 e000 = t000_end - t000_start;
        const Vec3vf4 s000 = t000_end + t000_start;
        const vfloat4  u000 = dot(cross(s000,e000),DDDD);
        if (all(ge_mask(Vec3fa(u000),Vec3fa(0.0f))) || all(le_mask(Vec3fa(u000),Vec3fa(0.0f))))
          if (occludedFinish(pre,ray,k,q00,q01,q10,u000,prim,scene)) return true;

        const Vec3vf4 t001_start = shuffle<2,3,1,0>(p00), t001_end = shuffle<3,1,2,0>(p00);
        const Vec3vf4 e001 = t001_end - t001_start;
        const Vec3vf4 s001 = t001_end + t001_start;
        const vfloat4  u001 = dot(cross(s001,e001),DDDD);
        if (all(ge_mask(Vec3fa(u001),Vec3fa(0.0f))) || all(le_mask(Vec3fa(u001),Vec3fa(0.0f))))
          if (occludedFinish(pre,ray,k,q11,q10,q01,u001,prim,scene)) return true;

        return false;
      }

      __forceinline static bool occludedDualQuad(const Precalculations1& pre, RayK<K>& ray, size_t k, const Vec3fa& O, const Vec3fa& D,
                                                 const Vec3fa& q00, const Vec3fa& q01,
                                                 const Vec3fa& q10, const Vec3fa& q11,
                                                 const Vec3fa& q20, const Vec3fa& q21,
                                                 const Primitive& prim, Scene* scene)
#if defined(__AVX__)
      {
        const Vec3vf8 D8(D.x,D.y,D.z);

        const vfloat8 q00_q10((vfloat4)q00,(vfloat4)q10);
        const vfloat8 q01_q11((vfloat4)q01,(vfloat4)q11);
        const vfloat8 q11_q21((vfloat4)q11,(vfloat4)q21);
        const vfloat8 q10_q20((vfloat4)q10,(vfloat4)q20);
        Vec3vf8 p00_p10; transpose(q00_q10,q01_q11,q11_q21,q10_q20,p00_p10.x,p00_p10.y,p00_p10.z);

        const Vec3vf8 t000_t100_start = shuffle<0,1,3,0>(p00_p10), t000_t100_end = shuffle<1,3,0,0>(p00_p10);
        const Vec3vf8 e000_e100 = t000_t100_end - t000_t100_start;
        const Vec3vf8 s000_s100 = t000_t100_end + t000_t100_start;
        const vfloat8  u000_u100 = dot(cross(s000_s100,e000_e100),D8);
        if (all(ge_mask(Vec3fa(extract4<0>(u000_u100)),Vec3fa(0.0f))) || all(le_mask(Vec3fa(extract4<0>(u000_u100)),Vec3fa(0.0f))))
          if (occludedFinish(pre,ray,k,q00,q01,q10,extract4<0>(u000_u100),prim,scene)) return true;
        if (all(ge_mask(Vec3fa(extract4<1>(u000_u100)),Vec3fa(0.0f))) || all(le_mask(Vec3fa(extract4<1>(u000_u100)),Vec3fa(0.0f))))
          if (occludedFinish(pre,ray,k,q10,q11,q20,extract4<1>(u000_u100),prim,scene)) return true;

        const Vec3vf8 t001_t101_start = shuffle<2,3,1,0>(p00_p10), t001_t101_end = shuffle<3,1,2,0>(p00_p10);
        const Vec3vf8 e001_e101 = t001_t101_end - t001_t101_start;
        const Vec3vf8 s001_s101 = t001_t101_end + t001_t101_start;
        const vfloat8  u001_u101 = dot(cross(s001_s101,e001_e101),D8);
        if (all(ge_mask(Vec3fa(extract4<0>(u001_u101)),Vec3fa(0.0f))) || all(le_mask(Vec3fa(extract4<0>(u001_u101)),Vec3fa(0.0f))))
          if (occludedFinish(pre,ray,k,q11,q10,q01,extract4<0>(u001_u101),prim,scene)) return true;
        if (all(ge_mask(Vec3fa(extract4<1>(u001_u101)),Vec3fa(0.0f))) || all(le_mask(Vec3fa(extract4<1>(u001_u101)),Vec3fa(0.0f))))
          if (occludedFinish(pre,ray,k,q21,q20,q11,extract4<1>(u001_u101),prim,scene)) return true;

        return false;
      }
#else
      {
        if (occludedQuad(pre,ray,k,O,D, q00,q01,q10,q11, prim, scene)) return true;
        if (occludedQuad(pre,ray,k,O,D, q10,q11,q20,q21, prim, scene)) return true;
        return false;
      }
#endif

      static __forceinline bool occludedQuad (const Precalculations1& pre, RayK<K>& ray, size_t k,
                                              const Vec3fa& v00, const Vec3fa& v10,
                                              const Vec3fa& v01, const Vec3fa& v11,
                                              const Primitive& prim, Scene* scene)
      {
        const Vec3fa O = pre.org_xyz;
        const Vec3fa D = pre.dir_xyz;
        const Vec3fa q00 = copy_a(v00-O,v00), q10 = copy_a(v10-O,v10);
        const Vec3fa q01 = copy_a(v01-O,v01), q11 = copy_a(v11-O,v11);
        return occludedQuad(pre,ray,k,O,D, q00,q01,q10,q11, prim, scene);
      }

      static __forceinline bool occludedQuads (const Precalculations1& pre, RayK<K>& ray, size_t k,
                                               const Vec3fa& v00, const Vec3fa& v10, const Vec3fa& v20,
                                               const Vec3fa& v01, const Vec3fa& v11, const Vec3fa& v21,
                                               const Primitive& prim, Scene* scene)
      {
        const Vec3fa O = pre.org_xyz;
        const Vec3fa D = pre.dir_xyz;
        const Vec3fa q00 = copy_a(v00-O,v00), q10 = copy_a(v10-O,v10), q20 = copy_a(v20-O,v20);
        const Vec3fa q01 = copy_a(v01-O,v01), q11 = copy_a(v11-O,v11), q21 = copy_a(v21-O,v21);
        return occludedDualQuad(pre,ray,k,O,D,q00,q01,q10,q11,q20,q21,prim,scene);
      }

      static __forceinline bool occludedQuads (const Precalculations1& pre, RayK<K>& ray, size_t k,
                                               const Vec3fa& v00, const Vec3fa& v10, const Vec3fa& v20,
                                               const Vec3fa& v01, const Vec3fa& v11, const Vec3fa& v21,
                                               const Vec3fa& v02, const Vec3fa& v12, const Vec3fa& v22,
                                               const Primitive& prim, Scene* scene)
      {
        const Vec3fa O = pre.org_xyz;
        const Vec3fa D = pre.dir_xyz;
        const Vec3fa q00 = copy_a(v00-O,v00), q10 = copy_a(v10-O,v10), q20 = copy_a(v20-O,v20);
        const Vec3fa q01 = copy_a(v01-O,v01), q11 = copy_a(v11-O,v11), q21 = copy_a(v21-O,v21);
        const Vec3fa q02 = copy_a(v02-O,v02), q12 = copy_a(v12-O,v12), q22 = copy_a(v22-O,v22);
        if (occludedDualQuad(pre,ray,k,O,D,q00,q01,q10,q11,q20,q21,prim,scene)) return true;
        if (occludedDualQuad(pre,ray,k,O,D,q01,q02,q11,q12,q21,q22,prim,scene)) return true;
        return false;
      }

      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(const Precalculations1& pre, RayK<K>& ray, size_t k, const Primitive& prim, Scene* scene)
      {
        STAT3(shadow.trav_prims,1,1,1);

        /* perform box tests */
        const vfloat<N> ray_tfar(ray.tfar[k]);
        size_t mask = prim.bounds.intersect<false>(pre.nearX, pre.nearY, pre.nearZ, pre.org, pre.rdir, pre.org_rdir, pre.ray_tnear, ray_tfar);

        /* intersect quad-quads */
        while (mask)
        {
          const size_t i = __bscf(mask);
          const size_t ofs = prim.quads[i].ofs;
          switch (prim.quads[i].type) {
          case GridAOS::EagerLeaf::Quads::QUAD1X1: {
            const Vec3fa& v00 = prim.grid.point(ofs,0), v10 = (&v00)[1];
            const Vec3fa& v01 = prim.grid.point(ofs,1), v11 = (&v01)[1];
            if (occludedQuad(pre,ray,k, v00,v10,v01,v11, prim, scene)) return true;
            break;
          }
          case GridAOS::EagerLeaf::Quads::QUAD1X2: {
            const Vec3fa& v00 = prim.grid.point(ofs,0), v10 = (&v00)[1];
            const Vec3fa& v01 = prim.grid.point(ofs,1), v11 = (&v01)[1];
            const Vec3fa& v02 = prim.grid.point(ofs,2), v12 = (&v02)[1];
            if (occludedQuads(pre,ray,k, v10,v11,v12,v00,v01,v02,  prim,scene)) return true;
            break;
          }
          case GridAOS::EagerLeaf::Quads::QUAD2X1: {
            const Vec3fa& v00 = prim.grid.point(ofs,0), v10 = (&v00)[1], v20 = (&v00)[2];
            const Vec3fa& v01 = prim.grid.point(ofs,1), v11 = (&v01)[1], v21 = (&v01)[2];
            if (occludedQuads(pre,ray,k, v00,v10,v20,v01,v11,v21, prim,scene)) return true;
            break;
          }
          case GridAOS::EagerLeaf::Quads::QUAD2X2: {
            const Vec3fa& v00 = prim.grid.point(ofs,0), v10 = (&v00)[1], v20 = (&v00)[2];
            const Vec3fa& v01 = prim.grid.point(ofs,1), v11 = (&v01)[1], v21 = (&v01)[2];
            const Vec3fa& v02 = prim.grid.point(ofs,2), v12 = (&v02)[1], v22 = (&v02)[2];
            if (occludedQuads(pre,ray,k, v00,v10,v20,v01,v11,v21,v02,v12,v22, prim,scene)) return true;
            break;
          }
          default: assert(false);
          }
        }

        return false;
      }

      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
      {
        Precalculations1 pre1(ray, k);
        return occluded(pre1, ray, k, prim[0], scene);
      }

      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
      {
        vbool<K> valid_o = false;
        int mask = movemask(valid_i);
        while (mask) {
          size_t k = __bscf(mask);
          if (occluded(pre, ray, k, prim, num, scene, lazy_node))
            set(valid_o, k);
        }
        return valid_o;
      }
    };
  }
}
