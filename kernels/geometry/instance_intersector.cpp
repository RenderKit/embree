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

#include "instance_intersector.h"
#include "../common/scene.h"

namespace embree
{
  namespace isa
  {
    /* Push an instance to the stack. */
    RTC_FORCEINLINE bool pushInstance(RTCIntersectContext* context, unsigned int instanceId)
    {
      const bool spaceAvailable = context && context->instStackSize < RTC_MAX_INSTANCE_LEVEL_COUNT;
      /* We assert here because instances are silently dropped when the stack is full. 
         This might be quite hard to find in production. */
      assert(spaceAvailable); 
      if (likely(spaceAvailable))
        context->instID[context->instStackSize++] = instanceId;
      return spaceAvailable;
    }

    /* Pop the last instance pushed to the stack. Do not call on an empty stack. */
    RTC_FORCEINLINE void popInstance(RTCIntersectContext* context)
    {
      assert(context && context->instStackSize > 0);
      context->instID[--context->instStackSize] = RTC_INVALID_GEOMETRY_ID;
    }

    void InstanceIntersector1::intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      const Instance* instance = prim.instance;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      if ((ray.mask & instance->mask) == 0) 
        return;
#endif

      RTCIntersectContext* user_context = context->user;
      if (likely(pushInstance(user_context, instance->geomID)))
      {
        const AffineSpace3fa world2local = instance->getWorld2Local();
        const Vec3fa ray_org = ray.org;
        const Vec3fa ray_dir = ray.dir;
        ray.org = Vec3fa(xfmPoint(world2local, ray_org), ray.tnear());
        ray.dir = Vec3fa(xfmVector(world2local, ray_dir), ray.time());
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.intersect((RTCRayHit&)ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        popInstance(user_context);
      }
    }
    
    bool InstanceIntersector1::occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      const Instance* instance = prim.instance;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      if ((ray.mask & instance->mask) == 0) 
        return false;
#endif
      
      RTCIntersectContext* user_context = context->user;
      bool occluded = false;
      if (likely(pushInstance(user_context, instance->geomID)))
      {
        const AffineSpace3fa world2local = instance->getWorld2Local();
        const Vec3fa ray_org = ray.org;
        const Vec3fa ray_dir = ray.dir;
        ray.org = Vec3fa(xfmPoint(world2local, ray_org), ray.tnear());
        ray.dir = Vec3fa(xfmVector(world2local, ray_dir), ray.time());
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.occluded((RTCRay&)ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        occluded = ray.tfar < 0.0f;
        popInstance(user_context);
      }
      return occluded;
    }

    void InstanceIntersector1MB::intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      const Instance* instance = prim.instance;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      if ((ray.mask & instance->mask) == 0) 
        return;
#endif
      
      RTCIntersectContext* user_context = context->user;
      if (likely(pushInstance(user_context, instance->geomID)))
      {
        const AffineSpace3fa world2local = instance->getWorld2Local(ray.time());
        const Vec3fa ray_org = ray.org;
        const Vec3fa ray_dir = ray.dir;
        ray.org = Vec3fa(xfmPoint(world2local, ray_org), ray.tnear());
        ray.dir = Vec3fa(xfmVector(world2local, ray_dir), ray.time());
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.intersect((RTCRayHit&)ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        popInstance(user_context);
      }
    }
    
    bool InstanceIntersector1MB::occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      const Instance* instance = prim.instance;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      if ((ray.mask & instance->mask) == 0) 
        return false;
#endif
      
      RTCIntersectContext* user_context = context->user;
      bool occluded = false;
      if (likely(pushInstance(user_context, instance->geomID)))
      {
        const AffineSpace3fa world2local = instance->getWorld2Local(ray.time());
        const Vec3fa ray_org = ray.org;
        const Vec3fa ray_dir = ray.dir;
        ray.org = Vec3fa(xfmPoint(world2local, ray_org), ray.tnear());
        ray.dir = Vec3fa(xfmVector(world2local, ray_dir), ray.time());
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.occluded((RTCRay&)ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        occluded = ray.tfar < 0.0f;
        popInstance(user_context);      
      }
      return occluded;
    }
    
    template<int K>
    void InstanceIntersectorK<K>::intersect(const vbool<K>& valid_i, const Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      vbool<K> valid = valid_i;
      const Instance* instance = prim.instance;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      valid &= (ray.mask & instance->mask) != 0;
      if (none(valid)) return;
#endif
        
      RTCIntersectContext* user_context = context->user;
      if (likely(pushInstance(user_context, instance->geomID)))
      {
        AffineSpace3vf<K> world2local = instance->getWorld2Local();
        const Vec3vf<K> ray_org = ray.org;
        const Vec3vf<K> ray_dir = ray.dir;
        ray.org = xfmPoint(world2local, ray_org);
        ray.dir = xfmVector(world2local, ray_dir);
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.intersect(valid, ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        popInstance(user_context);
      }
    }

    template<int K>
    vbool<K> InstanceIntersectorK<K>::occluded(const vbool<K>& valid_i, const Precalculations& pre, RayK<K>& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      vbool<K> valid = valid_i;
      const Instance* instance = prim.instance;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      valid &= (ray.mask & instance->mask) != 0;
      if (none(valid)) return false;
#endif
        
      RTCIntersectContext* user_context = context->user;
      vbool<K> occluded = false;
      if (likely(pushInstance(user_context, instance->geomID)))
      {
        AffineSpace3vf<K> world2local = instance->getWorld2Local();
        const Vec3vf<K> ray_org = ray.org;
        const Vec3vf<K> ray_dir = ray.dir;
        ray.org = xfmPoint(world2local, ray_org);
        ray.dir = xfmVector(world2local, ray_dir);
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.occluded(valid, ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        occluded = ray.tfar < 0.0f;
        popInstance(user_context);
      }
      return occluded;    
    }

    template<int K>
    void InstanceIntersectorKMB<K>::intersect(const vbool<K>& valid_i, const Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      vbool<K> valid = valid_i;
      const Instance* instance = prim.instance;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      valid &= (ray.mask & instance->mask) != 0;
      if (none(valid)) return;
#endif
        
      RTCIntersectContext* user_context = context->user;
      if (likely(pushInstance(user_context, instance->geomID)))
      {
        AffineSpace3vf<K> world2local = instance->getWorld2Local<K>(valid, ray.time());
        const Vec3vf<K> ray_org = ray.org;
        const Vec3vf<K> ray_dir = ray.dir;
        ray.org = xfmPoint(world2local, ray_org);
        ray.dir = xfmVector(world2local, ray_dir);
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.intersect(valid, ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        popInstance(user_context);
      }
    }

    template<int K>
    vbool<K> InstanceIntersectorKMB<K>::occluded(const vbool<K>& valid_i, const Precalculations& pre, RayK<K>& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      vbool<K> valid = valid_i;
      const Instance* instance = prim.instance;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      valid &= (ray.mask & instance->mask) != 0;
      if (none(valid)) return false;
#endif
        
      RTCIntersectContext* user_context = context->user;
      vbool<K> occluded = false;
      if (likely(pushInstance(user_context, instance->geomID)))
      {
        AffineSpace3vf<K> world2local = instance->getWorld2Local<K>(valid, ray.time());
        const Vec3vf<K> ray_org = ray.org;
        const Vec3vf<K> ray_dir = ray.dir;
        ray.org = xfmPoint(world2local, ray_org);
        ray.dir = xfmVector(world2local, ray_dir);
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.occluded(valid, ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        occluded = ray.tfar < 0.0f;
        popInstance(user_context);
      }
      return occluded;
    }

#if defined(__SSE__)
    template struct InstanceIntersectorK<4>;
    template struct InstanceIntersectorKMB<4>;
#endif
    
#if defined(__AVX__)
    template struct InstanceIntersectorK<8>;
    template struct InstanceIntersectorKMB<8>;
#endif

#if defined(__AVX512F__)
    template struct InstanceIntersectorK<16>;
    template struct InstanceIntersectorKMB<16>;
#endif
  }
}
