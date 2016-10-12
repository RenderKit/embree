// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
#include "../common/ray.h"

namespace embree
{
  namespace isa
  {
    template<int K, bool mblur>
      struct ObjectIntersectorK
    {
      typedef Object Primitive;
      
      struct PrecalculationsBase {
        __forceinline PrecalculationsBase (const vbool<K>& valid, const RayK<K>& ray) {}
      };

      typedef typename std::conditional<mblur, 
        IntersectorKPrecalculationsMB<K,PrecalculationsBase>,
        IntersectorKPrecalculations<K,PrecalculationsBase>>::type Precalculations;
      
      static __forceinline void intersect(const vbool<K>& valid_i, const Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive& prim, Scene* scene)
      {
        AVX_ZERO_UPPER();
        vbool<K> valid = valid_i;
        AccelSet* accel = (AccelSet*) scene->get(prim.geomID);
        
        /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
        valid &= (ray.mask & accel->mask) != 0;
        if (none(valid)) return;
#endif
        accel->intersect(valid,ray,prim.primID,context);
      }

      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, const Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive& prim, Scene* scene)
      {
        vbool<K> valid = valid_i;
        AccelSet* accel = (AccelSet*) scene->get(prim.geomID);
        
        /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
        valid &= (ray.mask & accel->mask) != 0;
        if (none(valid)) return false;
#endif
        accel->occluded(valid,ray,prim.primID,context);
        return ray.geomID == 0;
      }

      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive& prim, Scene* scene) {
        intersect(vbool<K>(1<<int(k)),pre,ray,context,prim,scene);
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive& prim, Scene* scene) {
        occluded(vbool<K>(1<<int(k)),pre,ray,context,prim,scene);
        return ray.geomID[k] == 0; 
      }
    };

    typedef ObjectIntersectorK<4,false>  ObjectIntersector4;
    typedef ObjectIntersectorK<8,false>  ObjectIntersector8;
    typedef ObjectIntersectorK<16,false> ObjectIntersector16;

    typedef ObjectIntersectorK<4,true>  ObjectIntersector4MB;
    typedef ObjectIntersectorK<8,true>  ObjectIntersector8MB;
    typedef ObjectIntersectorK<16,true> ObjectIntersector16MB;
  }
}
