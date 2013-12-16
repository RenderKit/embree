// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#ifndef __EMBREE_VIRTUAL_ACCEL_INTERSECTOR4_H__
#define __EMBREE_VIRTUAL_ACCEL_INTERSECTOR4_H__

#include "common/accel.h"
#include "common/ray4.h"

namespace embree
{
  struct VirtualAccelIntersector4
  {
    typedef AccelSet* Primitive;

    static __forceinline void intersect(const sseb& valid_i, Ray4& ray, const Primitive& mesh, const void* geom) 
    {
      AVX_ZERO_UPPER();
      mesh->intersect4(&valid_i,(RTCRay4&)ray);
    }

    static __forceinline void intersect(const sseb& valid, Ray4& ray, const Primitive* tri, size_t num, const void* geom)
    {
      for (size_t i=0; i<num; i++)
        intersect(valid,ray,tri[i],geom);
    }

    static __forceinline sseb occluded(const sseb& valid_i, const Ray4& ray, const Primitive& mesh, const void* geom) 
    {
      AVX_ZERO_UPPER();
      mesh->occluded4(&valid_i,(RTCRay4&)ray);
      return ray.geomID == 0;
    }

    static __forceinline sseb occluded(const sseb& valid, const Ray4& ray, const Primitive* tri, size_t num, void* geom)
    {
      sseb terminated = !valid;
      for (size_t i=0; i<num; i++) {
        terminated |= occluded(!terminated,ray,tri[i],geom);
        if (all(terminated)) return terminated;
      }
      return terminated;
    }
  };

  struct VirtualAccelIntersector4To8
  {
    typedef Accel* Primitive;

    static __forceinline void intersect(const sseb& valid_i, Ray4& ray, const Primitive& mesh, const void* geom) 
    {
      AVX_ZERO_UPPER();
      __align(32) Ray4x<2> ray8;
      ray8.set(0,ray);
      __align(32) sseb valid_i8[2] = { valid_i, false };
      mesh->intersect8(valid_i8,(RTCRay8&)ray8);
      ray8.get(0,ray);
    }

    static __forceinline void intersect(const sseb& valid, Ray4& ray, const Primitive* tri, size_t num, const void* geom)
    {
      for (size_t i=0; i<num; i++)
        intersect(valid,ray,tri[i],geom);
    }

    static __forceinline sseb occluded(const sseb& valid_i, Ray4& ray, const Primitive& mesh, const void* geom) 
    {
      AVX_ZERO_UPPER();
      __align(32) Ray4x<2> ray8;
      ray8.set(0,ray);
      __align(32) sseb valid_i8[2] = { valid_i, false };
      mesh->occluded8(valid_i8,(RTCRay8&)ray8);
      ray8.get(0,ray);
      return ray.geomID == 0;
    }

    static __forceinline sseb occluded(const sseb& valid, Ray4& ray, const Primitive* tri, size_t num, void* geom)
    {
      sseb terminated = !valid;
      for (size_t i=0; i<num; i++) {
        terminated |= occluded(!terminated,ray,tri[i],geom);
        if (all(terminated)) return terminated;
      }
      return terminated;
    }
  };

  struct VirtualAccelIntersector4To16
  {
    typedef Accel* Primitive;

    static __forceinline void intersect(const sseb& valid_i, Ray4& ray, const Primitive& mesh, const void* geom) 
    {
      AVX_ZERO_UPPER();
      __align(64) Ray4x<4> ray16;
      ray16.set(0,ray);
      __align(64) sseb valid_i16[4] = { valid_i, false, false, false };
      mesh->intersect16(valid_i16,(RTCRay16&)ray16);
      ray16.get(0,ray);
    }

    static __forceinline void intersect(const sseb& valid, Ray4& ray, const Primitive* tri, size_t num, const void* geom)
    {
      for (size_t i=0; i<num; i++)
        intersect(valid,ray,tri[i],geom);
    }

    static __forceinline sseb occluded(const sseb& valid_i, Ray4& ray, const Primitive& mesh, const void* geom) 
    {
      AVX_ZERO_UPPER();
      __align(64) Ray4x<4> ray16;
      ray16.set(0,ray);
      __align(64) sseb valid_i16[4] = { valid_i, false, false, false };
      mesh->occluded16(valid_i16,(RTCRay16&)ray16);
      ray16.get(0,ray);
      return ray.geomID == 0;
    }

    static __forceinline sseb occluded(const sseb& valid, Ray4& ray, const Primitive* tri, size_t num, void* geom)
    {
      sseb terminated = !valid;
      for (size_t i=0; i<num; i++) {
        terminated |= occluded(!terminated,ray,tri[i],geom);
        if (all(terminated)) return terminated;
      }
      return terminated;
    }
  };
}

#endif


