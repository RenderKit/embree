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
    __forceinline bool runIntersectionFilter1Helper(RTCFilterFunctionNArguments* args, const Geometry* const geometry, IntersectContext* context)
    {
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      if (geometry->intersectionFilterN)
#endif
      {
        assert(context->scene->hasGeometryFilterFunction());
        geometry->intersectionFilterN(args);
      }
      
      if (args->valid[0] == 0)
        return false;
      
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      if (context->user->filter) {
        assert(context->scene->hasContextFilterFunction());
        context->user->filter(args);
      }

      if (args->valid[0] == 0)
        return false;
#endif
      copyHitToRay(*(Ray*)args->ray,*(Hit*)args->potentialHit);
      return true;
    }
    
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
      return runIntersectionFilter1Helper(&args,geometry,context);
    }

    __forceinline void reportIntersection1(IntersectFunctionNArguments* args, const RTCFilterFunctionNArguments* filter_args)
    {
#if defined(EMBREE_FILTER_FUNCTION)
      IntersectContext* context = args->internal_context;
      const Geometry* const geometry = args->geometry;
      if (geometry->intersectionFilterN) {
        assert(context->scene->hasGeometryFilterFunction());
        geometry->intersectionFilterN(filter_args);
      }
      
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      //if (args->valid[0] == 0)
      //  return;

      if (context->user->filter) {
        assert(context->scene->hasContextFilterFunction());
        context->user->filter(filter_args);
      }
#endif
#endif
    }
    
    __forceinline bool runOcclusionFilter1Helper(RTCFilterFunctionNArguments* args, const Geometry* const geometry, IntersectContext* context)
    {
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      if (geometry->occlusionFilterN)
#endif
      {
        assert(context->scene->hasGeometryFilterFunction());
        geometry->occlusionFilterN(args);
      }
      
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      if (args->valid[0] == 0)
        return false;
      
      if (context->user->filter) {
        assert(context->scene->hasContextFilterFunction());
        context->user->filter(args);
      }
#endif
      return args->valid[0] != 0;
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
      return runOcclusionFilter1Helper(&args,geometry,context);
    }

    __forceinline void reportOcclusion1(OccludedFunctionNArguments* args, const RTCFilterFunctionNArguments* filter_args)
    {
#if defined(EMBREE_FILTER_FUNCTION)
      IntersectContext* context = args->internal_context;
      const Geometry* const geometry = args->geometry;
      if (geometry->occlusionFilterN) {
        assert(context->scene->hasGeometryFilterFunction());
        geometry->occlusionFilterN(filter_args);
      }
      
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      //if (args->valid[0] == 0)
      //  return false;
      
      if (context->user->filter) {
        assert(context->scene->hasContextFilterFunction());
        context->user->filter(filter_args);
      }
#endif
#endif
    }

    template<int K>
      __forceinline vbool<K> runIntersectionFilterHelper(RTCFilterFunctionNArguments* args, const Geometry* const geometry, IntersectContext* context)
    {
      vint<K>* mask = (vint<K>*) args->valid;
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      if (geometry->intersectionFilterN)
#endif
      {
        assert(context->scene->hasGeometryFilterFunction());
        geometry->intersectionFilterN(args);
      }

      vbool<K> valid_o = *mask != vint<K>(zero);
      if (none(valid_o)) return valid_o;

#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)      
      if (context->user->filter) {
        assert(context->scene->hasContextFilterFunction());
        context->user->filter(args);
      }

      valid_o = *mask != vint<K>(zero);
      if (none(valid_o)) return valid_o;
#endif
      
      copyHitToRay(valid_o,*(RayK<K>*)args->ray,*(HitK<K>*)args->potentialHit);
      return valid_o;
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
      return runIntersectionFilterHelper<K>(&args,geometry,context);
    }

    template<int K>
      __forceinline vbool<K> runOcclusionFilterHelper(RTCFilterFunctionNArguments* args, const Geometry* const geometry, IntersectContext* context)
    {
      vint<K>* mask = (vint<K>*) args->valid;
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      if (geometry->occlusionFilterN)
#endif
      {
        assert(context->scene->hasGeometryFilterFunction());
        geometry->occlusionFilterN(args);
      }

      vbool<K> valid_o = *mask != vint<K>(zero);
      
#if defined(EMBREE_FILTER_FUNCTION_CONTEXT)
      if (none(valid_o)) return valid_o;

      if (context->user->filter) {
        assert(context->scene->hasContextFilterFunction());
        context->user->filter(args);
      }

      valid_o = *mask != vint<K>(zero);
#endif

      RayK<K>* ray = (RayK<K>*) args->ray;
      ray->geomID = select(valid_o, vint<K>(zero), ray->geomID);
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
      return runOcclusionFilterHelper<K>(&args,geometry,context);
    }
  }
}
