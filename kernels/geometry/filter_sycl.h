// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../common/geometry.h"
#include "../common/ray.h"
#include "../common/hit.h"
#include "../common/context.h"

namespace embree
{
  __forceinline bool runIntersectionFilter1Helper(RTCFilterFunctionNArguments* args, int& mask, const Geometry* const geometry, IntersectContext* context)
  {
    typedef void (*RTCFilterFunctionSYCL)(const void* args);
    const RTCFeatureFlags feature_mask MAYBE_UNUSED = context->args->feature_mask;
    
#if EMBREE_FILTER_FUNCTION_IN_GEOMETRY
    if (feature_mask & RTC_FEATURE_FLAGS_FILTER_FUNCTION_IN_GEOMETRY)
    {
      RTCFilterFunctionSYCL gfilter = (RTCFilterFunctionSYCL) geometry->intersectionFilterN;
      if (gfilter)
      {
        assert(context->scene->hasGeometryFilterFunction());
        gfilter(args);
        
        if (mask == 0)
          return false;
      }
    }
#endif

#if EMBREE_FILTER_FUNCTION_IN_CONTEXT
    if (feature_mask & RTC_FEATURE_FLAGS_FILTER_FUNCTION_IN_CONTEXT)
    {
      RTCFilterFunctionSYCL cfilter = (RTCFilterFunctionSYCL) context->args->filter;
      if (cfilter)
      {
        assert(context->scene->hasContextFilterFunction());
        cfilter(args);
        
        if (mask == 0)
          return false;
      }
    }
#endif
    
    return true;
  }

  __forceinline bool runOcclusionFilter1Helper(RTCFilterFunctionNArguments* args, int& mask, const Geometry* const geometry, IntersectContext* context)
  {
    typedef void (*RTCFilterFunctionSYCL)(const void* args);
    const RTCFeatureFlags feature_mask MAYBE_UNUSED = context->args->feature_mask;
    
#if EMBREE_FILTER_FUNCTION_IN_GEOMETRY
    if (feature_mask & RTC_FEATURE_FLAGS_FILTER_FUNCTION_IN_GEOMETRY)
    {
      RTCFilterFunctionSYCL gfilter = (RTCFilterFunctionSYCL) geometry->occlusionFilterN;
      if (gfilter)
      {
        assert(context->scene->hasGeometryFilterFunction());
        gfilter(args);
        
        if (mask == 0)
          return false;
      }
    }
#endif

#if EMBREE_FILTER_FUNCTION_IN_CONTEXT
    if (feature_mask & RTC_FEATURE_FLAGS_FILTER_FUNCTION_IN_CONTEXT)
    {
      RTCFilterFunctionSYCL cfilter = (RTCFilterFunctionSYCL) context->args->filter;
      if (cfilter)
      {
        assert(context->scene->hasContextFilterFunction());
        cfilter(args);
        
        if (mask == 0)
          return false;
      }
    }
#endif
    return true;
  }
  
  __forceinline bool runIntersectionFilter1SYCL(Geometry* geometry, RayHit& ray, sycl::private_ptr<IntersectContext> context, Hit& hit)
  {
    RTCFilterFunctionNArguments args;
    int mask = -1;
    args.valid = &mask;
    args.geometryUserPtr = geometry->userPtr;
    args.context = context->user;
    args.ray = (RTCRayN*) &ray;
    args.hit = (RTCHitN*) &hit;
    args.N = 1;
    return runIntersectionFilter1Helper(&args,mask,geometry,context);
  }


  __forceinline bool runIntersectionFilter1SYCL(Geometry* geometry, Ray& ray, sycl::private_ptr<IntersectContext> context, Hit& hit)
  {
    RTCFilterFunctionNArguments args;
    int mask = -1;
    args.valid = &mask;
    args.geometryUserPtr = geometry->userPtr;
    args.context = context->user;
    args.ray = (RTCRayN*) &ray;
    args.hit = (RTCHitN*) &hit;
    args.N = 1;
    return runOcclusionFilter1Helper(&args,mask,geometry,context);
  }
}
