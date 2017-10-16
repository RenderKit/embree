// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "../common/geometry.h"
#include "../common/ray.h"
#include "../common/hit.h"
#include "../common/context.h"

namespace embree
{
  namespace isa
  {
    __forceinline bool runIntersectionFilter1(const Geometry* const geometry, Ray& ray, IntersectContext* context,
                                              const float& u, const float& v, const float& t, const Vec3fa& Ng, const int geomID, const int primID)
    {
      assert(geometry->intersectionFilterN);
      int mask = -1; Hit hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->intersectionFilterN(&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,1);
      return mask != 0;
    }
    
    __forceinline bool runOcclusionFilter1(const Geometry* const geometry, Ray& ray, IntersectContext* context,
                                           const float& u, const float& v, const float& t, const Vec3fa& Ng, const int geomID, const int primID)
    {
      assert(geometry->occlusionFilterN);
      int mask = -1; Hit hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->occlusionFilterN(&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,1);
      return ray.geomID == 0;
    }

    __forceinline vbool4 runIntersectionFilter(const vbool4& valid, const Geometry* const geometry, Ray4& ray, IntersectContext* context,
                                               const vfloat4& u, const vfloat4& v, const vfloat4& t, const Vec3vf4& Ng, const int geomID, const int primID)
    {
      assert(geometry->intersectionFilterN);
      vint4 mask = valid.mask32(); Hit4 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->intersectionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,4);
      return mask != vint4(0);
    }
    
    __forceinline vbool4 runOcclusionFilter(const vbool4& valid, const Geometry* const geometry, Ray4& ray, IntersectContext* context,
                                            const vfloat4& u, const vfloat4& v, const vfloat4& t, const Vec3vf4& Ng, const int geomID, const int primID)
    {
      assert(geometry->occlusionFilterN);
      vint4 mask = valid.mask32(); Hit4 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,4);
      return (mask != vint4(zero)) & (ray.geomID == 0);
    }
    
    __forceinline bool runIntersectionFilter(const Geometry* const geometry, Ray4& ray, const size_t k, IntersectContext* context,
                                             const float& u, const float& v, const float& t, const Vec3fa& Ng, const int geomID, const int primID)
    {
      const vbool4 valid(1 << k);
      assert(geometry->intersectionFilterN);
      vint4 mask = valid.mask32(); Hit4 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->intersectionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,4);
      return mask[k] != 0;
    }
    
    __forceinline bool runOcclusionFilter(const Geometry* const geometry, Ray4& ray, const size_t k, IntersectContext* context,
                                          const float& u, const float& v, const float& t, const Vec3fa& Ng, const int geomID, const int primID)
    {
      const vbool4 valid(1 << k);
      assert(geometry->occlusionFilterN);
      vint4 mask = valid.mask32(); Hit4 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,4);
      return mask[k] != 0 && ray.geomID[k] == 0;
    }
    
#if defined(__AVX__)
    __forceinline vbool8 runIntersectionFilter(const vbool8& valid, const Geometry* const geometry, Ray8& ray, IntersectContext* context,
                                               const vfloat8& u, const vfloat8& v, const vfloat8& t, const Vec3vf8& Ng, const int geomID, const int primID)
    {
      assert(geometry->intersectionFilterN);
      vint8 mask = valid.mask32(); Hit8 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->intersectionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,8);
      return mask != vint8(0);
    }
    
    __forceinline vbool8 runOcclusionFilter(const vbool8& valid, const Geometry* const geometry, Ray8& ray, IntersectContext* context,
                                            const vfloat8& u, const vfloat8& v, const vfloat8& t, const Vec3vf8& Ng, const int geomID, const int primID)
    {
      assert(geometry->occlusionFilterN);
      vint8 mask = valid.mask32(); Hit8 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,8);
      return (mask != vint8(zero)) & (ray.geomID == 0);
    }
    
    __forceinline bool runIntersectionFilter(const Geometry* const geometry, Ray8& ray, const size_t k, IntersectContext* context,
                                             const float& u, const float& v, const float& t, const Vec3fa& Ng, const int geomID, const int primID)
    {
      const vbool8 valid(1 << k);
      assert(geometry->intersectionFilterN);
      vint8 mask = valid.mask32(); Hit8 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->intersectionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,8);
      return mask[k] != 0;
    }
    
    __forceinline bool runOcclusionFilter(const Geometry* const geometry, Ray8& ray, const size_t k, IntersectContext* context,
                                          const float& u, const float& v, const float& t, const Vec3fa& Ng, const int geomID, const int primID)
    {
      const vbool8 valid(1 << k);
      assert(geometry->occlusionFilterN);
      vint8 mask = valid.mask32(); Hit8 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,8);
      return mask[k] != 0 && ray.geomID[k] == 0;
    }
    
#endif


#if defined(__AVX512F__)
    __forceinline vbool16 runIntersectionFilter(const vbool16& valid, const Geometry* const geometry, Ray16& ray, IntersectContext* context,
                                                const vfloat16& u, const vfloat16& v, const vfloat16& t, const Vec3vf16& Ng, const int geomID, const int primID)
    {
      assert(geometry->intersectionFilterN);
      vint16 mask = valid.mask32(); Hit16 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->intersectionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,16);
      return mask != vint16(0);
    }
    
    __forceinline vbool16 runOcclusionFilter(const vbool16& valid, const Geometry* const geometry, Ray16& ray, IntersectContext* context,
                                             const vfloat16& u, const vfloat16& v, const vfloat16& t, const Vec3vf16& Ng, const int geomID, const int primID)
    {
      assert(geometry->occlusionFilterN);
      vint16 mask = valid.mask32(); Hit16 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,16);
      return (mask != vint16(zero)) & (ray.geomID == 0);
    }
      
    __forceinline bool runIntersectionFilter(const Geometry* const geometry, Ray16& ray, const size_t k, IntersectContext* context,
                                             const float& u, const float& v, const float& t, const Vec3fa& Ng, const int geomID, const int primID)
    {
      const vbool16 valid(1 << k);
      assert(geometry->intersectionFilterN);
      vint16 mask = valid.mask32(); Hit16 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->intersectionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,16);
      return mask[k] != 0;
    }
    
    __forceinline bool runOcclusionFilter(const Geometry* const geometry, Ray16& ray, const size_t k, IntersectContext* context,
                                          const float& u, const float& v, const float& t, const Vec3fa& Ng, const int geomID, const int primID)
    {
      const vbool16 valid(1 << k);
      assert(geometry->occlusionFilterN);
      vint16 mask = valid.mask32(); Hit16 hit(ray.instID,geomID,primID,u,v,t,Ng);
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,16);
      return mask[k] != 0 && ray.geomID[k] == 0;
    }    
#endif

  }
}
