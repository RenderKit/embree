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

#define EMBREE_INTERSERTION_FILTER_CONTEXT 0

namespace embree
{
  namespace isa
  {
    __forceinline bool runIntersectionFilter1(const Geometry* const geometry, Ray& ray, IntersectContext* context, Hit& hit)
    {
      int mask = -1; int accept = 0;
      RTCFilterFunctionNArguments args;
      args.valid = &mask;
      args.geomUserPtr = geometry->userPtr;
      args.context = context->user;
      args.ray = (RTCRayN*)&ray;
      args.potentialHit = (RTCHitN*)&hit;
      args.N = 1;
      args.acceptHit = &accept;
#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (geometry->intersectionFilterN)
#endif
        geometry->intersectionFilterN(&args);
      if (accept == 0)
        return false;
#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (context->context->filter)
        context->context->filter(&args);
      if (accept == 0)
        return false;
#endif
      copyHitToRay(ray,hit);
      return true;
    }
    
    __forceinline bool runOcclusionFilter1(const Geometry* const geometry, Ray& ray, IntersectContext* context, Hit& hit)
    {
      int mask = -1; int accept = 0;
      RTCFilterFunctionNArguments args;
      args.valid = &mask;
      args.geomUserPtr = geometry->userPtr;
      args.context = context->user;
      args.ray = (RTCRayN*)&ray;
      args.potentialHit = (RTCHitN*)&hit;
      args.N = 1;
      args.acceptHit = &accept;
#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (geometry->occlusionFilterN)
#endif
        geometry->occlusionFilterN(&args);
#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (accept == 0)
        return false;
      if (context->context->filter)
        context->context->filter(&args);
#endif
      return accept != 0;
    }

    template<int K>
    __forceinline vbool<K> runIntersectionFilter(const vbool<K>& valid, const Geometry* const geometry, RayK<K>& ray, IntersectContext* context, HitK<K>& hit)
    {
      vint<K> mask = valid.mask32(); vint<K> accept(zero);
      
      RTCFilterFunctionNArguments args;
      args.valid = (int*)&mask;
      args.geomUserPtr = geometry->userPtr;
      args.context = context->user;
      args.ray = (RTCRayN*)&ray;
      args.potentialHit = (RTCHitN*)&hit;
      args.N = K;
      args.acceptHit = (int*)&accept;

#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (geometry->intersectionFilterN)
#endif
      {
        geometry->intersectionFilterN(&args);
        mask &= accept;
      }
      vbool<K> valid_o = mask != vint<K>(zero);
      if (none(valid_o)) return valid_o;

#if EMBREE_INTERSERTION_FILTER_CONTEXT      
      if (context->context->filter) {
        context->context->filter(&args);
        mask &= accept;
      }
      valid_o = mask != vint<K>(zero);
      if (none(valid_o)) return valid_o;
#endif
      
      copyHitToRay(valid_o,ray,hit);
      return valid_o;
    }

    template<int K>
      __forceinline vbool<K> runOcclusionFilter(const vbool<K>& valid, const Geometry* const geometry, RayK<K>& ray, IntersectContext* context, HitK<K>& hit)
    {
      assert(geometry->occlusionFilterN);
      vint<K> mask = valid.mask32(); vint<K> accept(zero);
      
      RTCFilterFunctionNArguments args;
      args.valid = (int*)&mask;
      args.geomUserPtr = geometry->userPtr;
      args.context = context->user;
      args.ray = (RTCRayN*)&ray;
      args.potentialHit = (RTCHitN*)&hit;
      args.N = K;
      args.acceptHit = (int*)&accept;

#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (geometry->occlusionFilterN)
#endif
      {
        geometry->occlusionFilterN(&args);
        mask &= accept;
      }
      vbool<K> valid_o = mask != vint<K>(zero);
      
#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (none(valid_o)) return valid_o;

      if (context->context->filter) {
        context->context->filter(&args);
        mask &= accept;
      }

      valid_o = mask != vint<K>(zero);
#endif
      
      ray.geomID = select(valid_o, vint<K>(zero), ray.geomID);
      return valid_o;
    }
  }
}
