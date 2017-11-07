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
    __forceinline bool runIntersectionFilter1(const Geometry* const geometry, Ray& ray, IntersectContext* context, Hit& hit)
    {
      assert(geometry->intersectionFilterN);
      int mask = -1; int accept = 0;
      geometry->intersectionFilterN(&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,1,&accept);
      if (accept != 0) copyHitToRay(ray,hit);
      return accept != 0;
    }
    
    __forceinline bool runOcclusionFilter1(const Geometry* const geometry, Ray& ray, IntersectContext* context, Hit& hit)
    {
      assert(geometry->occlusionFilterN);
      int mask = -1; int accept = 0;
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,1, (int*)&accept);
      return accept != 0;
    }

    template<int K>
    __forceinline vbool<K> runIntersectionFilter(const vbool<K>& valid, const Geometry* const geometry, RayK<K>& ray, IntersectContext* context, HitK<K>& hit)
    {
      assert(geometry->intersectionFilterN);
      vint<K> mask = valid.mask32(); vint<K> accept(zero);
      geometry->intersectionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,K,(int*)&accept);
      accept &= mask;
      const vbool<K> final = accept != vint<K>(zero);
      if (any(final)) copyHitToRay(final,ray,hit);
      return final;
    }

    template<int K>
      __forceinline vbool<K> runOcclusionFilter(const vbool<K>& valid, const Geometry* const geometry, RayK<K>& ray, IntersectContext* context, HitK<K>& hit)
    {
      assert(geometry->occlusionFilterN);
      vint<K> mask = valid.mask32(); vint<K> accept(zero);
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,K,(int*)&accept);
      accept &= mask;
      const vbool<K> final = accept != vint<K>(zero);
      ray.geomID = select(final, vint<K>(zero), ray.geomID);
      return final;
    }

    template<int K>
      __forceinline bool runIntersectionFilter(const Geometry* const geometry, RayK<K>& ray, const size_t k, IntersectContext* context, HitK<K>& hit)
    {
      const vbool<K> valid(1 << k);
      assert(geometry->intersectionFilterN);
      vint<K> mask = valid.mask32(); vint<K> accept(zero);
      geometry->intersectionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,K,(int*)&accept);
      accept &= mask;
      const vbool<K> final = accept != vint<K>(zero);
      if (any(final)) { copyHitToRay(final,ray,hit); return true; }
      return false;
    }

    template<int K>
      __forceinline bool runOcclusionFilter(const Geometry* const geometry, RayK<K>& ray, const size_t k, IntersectContext* context, HitK<K>& hit)
    {
      const vbool<K> valid(1 << k);
      assert(geometry->occlusionFilterN);
      vint<K> mask = valid.mask32(); vint<K> accept(zero);
      geometry->occlusionFilterN((int*)&mask,geometry->userPtr,context->user,(RTCRayN*)&ray,(RTCHitN*)&hit,K,(int*)&accept);
      accept &= mask;
      const vbool<K> final = accept != vint<K>(zero);
      ray.geomID = select(final, vint<K>(zero), ray.geomID);
      return any(final);
    }
  }
}
