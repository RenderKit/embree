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

#include "object.h"
#include "../../common/ray.h"

namespace embree
{
  namespace isa
  {
    template<int K>
    struct ObjectIntersectorK
    {
      typedef Object Primitive;
      
      struct Precalculations {
        __forceinline Precalculations (const vbool<K>& valid, const RayK<K>& ray) {}
      };
      
      static __forceinline void intersect(const vbool<K>& valid_i, const Precalculations& pre, RayK<K>& ray, const Primitive& prim, Scene* scene);
      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, const Precalculations& pre, const RayK<K>& ray, const Primitive& prim, Scene* scene);
    };

    typedef ObjectIntersectorK<4>  ObjectIntersector4;
    typedef ObjectIntersectorK<8>  ObjectIntersector8;
    typedef ObjectIntersectorK<16> ObjectIntersector16;

    template<>
    __forceinline void ObjectIntersector4::intersect(const vbool4& valid_i, const Precalculations& pre, Ray4& ray, const Primitive& prim, Scene* scene)
    {
      AVX_ZERO_UPPER();
      vbool4 valid = valid_i;
      AccelSet* accel = (AccelSet*) scene->get(prim.geomID);

      /* perform ray mask test */
#if defined(RTCORE_RAY_MASK)
      valid &= (ray.mask & accel->mask) != 0;
      if (none(valid)) return;
#endif
      accel->intersect4(valid,(RTCRay4&)ray,prim.primID);
    }

    template<>
    __forceinline vbool4 ObjectIntersector4::occluded(const vbool4& valid_i, const Precalculations& pre, const Ray4& ray, const Primitive& prim, Scene* scene)
    {
      AVX_ZERO_UPPER();
      vbool4 valid = valid_i;
      AccelSet* accel = (AccelSet*) scene->get(prim.geomID);

      /* perform ray mask test */
#if defined(RTCORE_RAY_MASK)
      valid &= (ray.mask & accel->mask) != 0;
      if (none(valid)) return false;
#endif
      accel->occluded4(valid,(RTCRay4&)ray,prim.primID);
      return ray.geomID == 0;
    }

#if defined(__AVX__)
    template<>
    __forceinline void ObjectIntersector8::intersect(const vbool8& valid_i, const Precalculations& pre, Ray8& ray, const Primitive& prim, Scene* scene)
    {
      vbool8 valid = valid_i;
      AccelSet* accel = (AccelSet*) scene->get(prim.geomID);

      /* perform ray mask test */
#if defined(RTCORE_RAY_MASK)
      valid &= (ray.mask & accel->mask) != 0;
      if (none(valid)) return;
#endif
      accel->intersect8(valid,(RTCRay8&)ray,prim.primID);
    }

    template<>
    __forceinline vbool8 ObjectIntersector8::occluded(const vbool8& valid_i, const Precalculations& pre, const Ray8& ray, const Primitive& prim, Scene* scene)
    {
      vbool8 valid = valid_i;
      AccelSet* accel = (AccelSet*) scene->get(prim.geomID);

      /* perform ray mask test */
#if defined(RTCORE_RAY_MASK)
      valid &= (ray.mask & accel->mask) != 0;
      if (none(valid)) return false;
#endif
      accel->occluded8(valid,(RTCRay8&)ray,prim.primID);
      return ray.geomID == 0;
    }
#endif

#if defined(__AVX512F__)
    template<>
    __forceinline void ObjectIntersector16::intersect(const vbool16& valid_i, const Precalculations& pre, Ray16& ray, const Primitive& prim, Scene* scene)
    {
      vbool16 valid = valid_i;
      AccelSet* accel = (AccelSet*) scene->get(prim.geomID);

      /* perform ray mask test */
#if defined(RTCORE_RAY_MASK)
      valid &= (ray.mask & accel->mask) != 0;
      if (none(valid)) return;
#endif
      accel->intersect16(valid,(RTCRay16&)ray,prim.primID);
    }

    template<>
    __forceinline vbool16 ObjectIntersector16::occluded(const vbool16& valid_i, const Precalculations& pre, const Ray16& ray, const Primitive& prim, Scene* scene)
    {
      vbool16 valid = valid_i;
      AccelSet* accel = (AccelSet*) scene->get(prim.geomID);

      /* perform ray mask test */
#if defined(RTCORE_RAY_MASK)
      valid &= (ray.mask & accel->mask) != 0;
      if (none(valid)) return false;
#endif
      accel->occluded16(valid,(RTCRay16&)ray,prim.primID);
      return ray.geomID == 0;
    }
#endif
  }
}
