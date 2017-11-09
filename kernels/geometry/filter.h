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
      RTCFilterFunctionNArguments args;
      int mask = -1;
      args.valid = &mask;
      args.geomUserPtr = geometry->userPtr;
      args.context = context->user;
      args.ray = (RTCRayN*)&ray;
      args.potentialHit = (RTCHitN*)&hit;
      args.N = 1;

#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (geometry->intersectionFilterN)
#endif
        geometry->intersectionFilterN(&args);
      
      if (mask == 0)
        return false;
#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (context->context->filter)
        context->context->filter(&args);

      if (mask == 0)
        return false;
#endif
      copyHitToRay(ray,hit);
      return true;
    }
    
    __forceinline bool runOcclusionFilter1(const Geometry* const geometry, Ray& ray, IntersectContext* context, Hit& hit)
    {
      RTCFilterFunctionNArguments args;
      int mask = -1;
      args.valid = &mask;
      args.geomUserPtr = geometry->userPtr;
      args.context = context->user;
      args.ray = (RTCRayN*)&ray;
      args.potentialHit = (RTCHitN*)&hit;
      args.N = 1;

#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (geometry->occlusionFilterN)
#endif
        geometry->occlusionFilterN(&args);
#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (mask == 0)
        return false;
      if (context->context->filter)
        context->context->filter(&args);
#endif
      return mask != 0;
    }

    template<int K>
    __forceinline vbool<K> runIntersectionFilter(const vbool<K>& valid, const Geometry* const geometry, RayK<K>& ray, IntersectContext* context, HitK<K>& hit)
    {
      RTCFilterFunctionNArguments args;
      vint<K> mask = valid.mask32();
      args.valid = (int*)&mask;
      args.geomUserPtr = geometry->userPtr;
      args.context = context->user;
      args.ray = (RTCRayN*)&ray;
      args.potentialHit = (RTCHitN*)&hit;
      args.N = K;

#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (geometry->intersectionFilterN)
#endif
        geometry->intersectionFilterN(&args);

      vbool<K> valid_o = mask != vint<K>(zero);
      assert(none(!valid & valid_o));
      if (none(valid_o)) return valid_o;

#if EMBREE_INTERSERTION_FILTER_CONTEXT      
      if (context->context->filter)
        context->context->filter(&args);

      valid_o = mask != vint<K>(zero);
      assert(none(!valid & valid_o));
      if (none(valid_o)) return valid_o;
#endif
      
      copyHitToRay(valid_o,ray,hit);
      return valid_o;
    }

    template<int K>
      __forceinline vbool<K> runOcclusionFilter(const vbool<K>& valid, const Geometry* const geometry, RayK<K>& ray, IntersectContext* context, HitK<K>& hit)
    {
      RTCFilterFunctionNArguments args;
      vint<K> mask = valid.mask32();
      args.valid = (int*)&mask;
      args.geomUserPtr = geometry->userPtr;
      args.context = context->user;
      args.ray = (RTCRayN*)&ray;
      args.potentialHit = (RTCHitN*)&hit;
      args.N = K;

#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (geometry->occlusionFilterN)
#endif
        geometry->occlusionFilterN(&args);

      vbool<K> valid_o = mask != vint<K>(zero);
      assert(none(!valid & valid_o));
      
#if EMBREE_INTERSERTION_FILTER_CONTEXT
      if (none(valid_o)) return valid_o;

      if (context->context->filter)
        context->context->filter(&args);

      valid_o = mask != vint<K>(zero);
      assert(none(!valid & valid_o));
#endif
      
      ray.geomID = select(valid_o, vint<K>(zero), ray.geomID);
      return valid_o;
    }
  }
}
