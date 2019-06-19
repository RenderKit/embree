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

#include "default.h"
#include "rtcore.h"
#include "point_query.h"

namespace embree
{
  class Scene;

  struct IntersectContext
  {
  public:
    __forceinline IntersectContext(Scene* scene, RTCIntersectContext* user_context)
      : scene(scene), user(user_context) {}

    __forceinline bool hasContextFilter() const {
      return user->filter != nullptr;
    }

    __forceinline bool isCoherent() const {
      return embree::isCoherent(user->flags);
    }

    __forceinline bool isIncoherent() const {
      return embree::isIncoherent(user->flags);
    }
    
  public:
    Scene* scene;
    RTCIntersectContext* user;
  };

  struct __aligned(16) PointQueryInstanceStack
  {
    ALIGNED_STRUCT_(16);

    __forceinline PointQueryInstanceStack()
      : size(0u)
    { }

    AffineSpace3fa instW2I[RTC_MAX_INSTANCE_LEVEL_COUNT]; // the current stack of accumulated transformations from world to instance space
    AffineSpace3fa instI2W[RTC_MAX_INSTANCE_LEVEL_COUNT]; // the current stack of accumulated transformations fom instance to world space
    unsigned int   instID [RTC_MAX_INSTANCE_LEVEL_COUNT]; // the current stack of instance ids.
    unsigned int   size;                                  // number of instances currently on the stack.
  };
  
  enum PointQueryType
  {
    POINT_QUERY_TYPE_UNDEFINED = 0,
    POINT_QUERY_TYPE_SPHERE = 1,
    POINT_QUERY_TYPE_AABB = 2,
  };

  typedef bool (*PointQueryFunction)(struct RTCPointQueryFunctionArguments* args);
  
  struct PointQueryContext
  {
  public:
    __forceinline PointQueryContext(Scene* scene, 
                                    PointQuery* query_ws, 
                                    PointQueryType query_type,
                                    PointQueryFunction func, 
                                    RTCPointQueryInstanceStack* instStack,
                                    float similarityScale,
                                    void* userPtr)
      : scene(scene)
      , query_ws(query_ws)
      , query_type(query_type)
      , func(func)
      , instStack((PointQueryInstanceStack*)instStack)
      , similarityScale(similarityScale)
      , userPtr(userPtr) 
      , primID(RTC_INVALID_GEOMETRY_ID)
      , geomID(RTC_INVALID_GEOMETRY_ID)
      , query_radius(query_ws->radius)
    { 
      if (query_type == POINT_QUERY_TYPE_AABB) {
        assert(similarityScale == 0.f);
        updateAABB();
      }
      if (instStack->size == 0) {
        assert(similarityScale == 1.f);
      }
    }

  public:
    __forceinline void updateAABB() 
    {
      if (likely(query_ws->radius == (float)inf || instStack->size == 0)) {
        query_radius = Vec3fa(query_ws->radius);
        return;
      }

      AffineSpace3fa const& m = instStack->instW2I[instStack->size-1];
      Vec3fa bbmin(inf), bbmax(neg_inf);

      // iterate over 8 AABB corners
      for (int i = 0; i < 8; ++i) 
      {
        // compute i-th AABB corner c
        const Vec3fa c((i/1)%2 == 0 ? -1.f : 1.f, (i/2)%2 == 0 ? -1.f : 1.f, (i/4)%2 == 0 ? -1.f : 1.f);
        const Vec3fa v = xfmPoint(m, query_ws->p + query_ws->radius * c);
        bbmin = min(bbmin, v);
        bbmax = max(bbmax, v);
      }
      query_radius = Vec3fa(0.5f * (bbmax.x - bbmin.x));
    }

public:
    Scene* scene;

    PointQuery* query_ws; // the original world space point query 
    PointQueryType query_type;
    PointQueryFunction func;
    PointQueryInstanceStack* instStack;
    const float similarityScale;

    void* userPtr;

    unsigned int primID;
    unsigned int geomID;

    Vec3fa query_radius;  // used if the query is converted to an AABB internally
  };
}

