// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "instance_intersector.h"
#include "../common/scene.h"
#include "../common/instance_stack.h"

namespace embree
{
  namespace isa
  {

    /* Push an instance to the stack. */
    RTC_FORCEINLINE bool pushInstance(RTCPointQueryContext* context,
                      unsigned int instanceId,
                      AffineSpace3fa const& w2i,
                      AffineSpace3fa const& i2w)
    {
      assert(context);
      const size_t stackSize = context->instStackSize;
      assert(stackSize < RTC_MAX_INSTANCE_LEVEL_COUNT); 
      context->instID[stackSize] = instanceId;

      AffineSpace3fa_store_unaligned(w2i,(AffineSpace3fa*)context->world2inst[stackSize]);
      AffineSpace3fa_store_unaligned(i2w,(AffineSpace3fa*)context->inst2world[stackSize]);

      if (unlikely(stackSize > 0))
      {
        const AffineSpace3fa world2inst = AffineSpace3fa_load_unaligned((AffineSpace3fa*)context->world2inst[stackSize  ]) 
                                        * AffineSpace3fa_load_unaligned((AffineSpace3fa*)context->world2inst[stackSize-1]);
        const AffineSpace3fa inst2world = AffineSpace3fa_load_unaligned((AffineSpace3fa*)context->inst2world[stackSize-1]) 
                                        * AffineSpace3fa_load_unaligned((AffineSpace3fa*)context->inst2world[stackSize  ]);
        AffineSpace3fa_store_unaligned(world2inst,(AffineSpace3fa*)context->world2inst[stackSize]);
        AffineSpace3fa_store_unaligned(inst2world,(AffineSpace3fa*)context->inst2world[stackSize]);
      }
      context->instStackSize++;
      return true;
    }

    /* Pop the last instance pushed to the stack. Do not call on an empty stack. */
    RTC_FORCEINLINE void popInstance(RTCPointQueryContext* context)
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
      if (likely(instance_id_stack::push(user_context, prim.instID_)))
      {
        const AffineSpace3fa world2local = instance->getWorld2Local();
        const Vec3ff ray_org = ray.org;
        const Vec3ff ray_dir = ray.dir;
        ray.org = Vec3ff(xfmPoint(world2local, ray_org), ray.tnear());
        ray.dir = Vec3ff(xfmVector(world2local, ray_dir), ray.time());
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.intersect((RTCRayHit&)ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        instance_id_stack::pop(user_context);
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
      if (likely(instance_id_stack::push(user_context, prim.instID_)))
      {
        const AffineSpace3fa world2local = instance->getWorld2Local();
        const Vec3ff ray_org = ray.org;
        const Vec3ff ray_dir = ray.dir;
        ray.org = Vec3ff(xfmPoint(world2local, ray_org), ray.tnear());
        ray.dir = Vec3ff(xfmVector(world2local, ray_dir), ray.time());
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.occluded((RTCRay&)ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        occluded = ray.tfar < 0.0f;
        instance_id_stack::pop(user_context);
      }
      return occluded;
    }
    
    bool InstanceIntersector1::pointQuery(PointQuery* query, PointQueryContext* context, const InstancePrimitive& prim)
    {
      const Instance* instance = prim.instance;

      const AffineSpace3fa local2world = instance->getLocal2World();
      const AffineSpace3fa world2local = instance->getWorld2Local();
      float similarityScale = 0.f;
      const bool similtude = context->query_type == POINT_QUERY_TYPE_SPHERE
                           && similarityTransform(world2local, &similarityScale);
      assert((similtude && similarityScale > 0) || !similtude);

      if (likely(pushInstance(context->userContext, prim.instID_, world2local, local2world)))
      {
        PointQuery query_inst;
        query_inst.time = query->time;
        query_inst.p = xfmPoint(world2local, query->p); 
        query_inst.radius = query->radius * similarityScale;
        
        PointQueryContext context_inst(
          (Scene*)instance->object, 
          context->query_ws, 
          similtude ? POINT_QUERY_TYPE_SPHERE : POINT_QUERY_TYPE_AABB,
          context->func, 
          context->userContext,
          similarityScale,
          context->userPtr); 

        bool changed = instance->object->intersectors.pointQuery(&query_inst, &context_inst);
        popInstance(context->userContext);
        return changed;
      }
      return false;
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
      if (likely(instance_id_stack::push(user_context, prim.instID_)))
      {
        const AffineSpace3fa world2local = instance->getWorld2Local(ray.time());
        const Vec3ff ray_org = ray.org;
        const Vec3ff ray_dir = ray.dir;
        ray.org = Vec3ff(xfmPoint(world2local, ray_org), ray.tnear());
        ray.dir = Vec3ff(xfmVector(world2local, ray_dir), ray.time());
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.intersect((RTCRayHit&)ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        instance_id_stack::pop(user_context);
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
      if (likely(instance_id_stack::push(user_context, prim.instID_)))
      {
        const AffineSpace3fa world2local = instance->getWorld2Local(ray.time());
        const Vec3ff ray_org = ray.org;
        const Vec3ff ray_dir = ray.dir;
        ray.org = Vec3ff(xfmPoint(world2local, ray_org), ray.tnear());
        ray.dir = Vec3ff(xfmVector(world2local, ray_dir), ray.time());
        IntersectContext newcontext((Scene*)instance->object, user_context);
        instance->object->intersectors.occluded((RTCRay&)ray, &newcontext);
        ray.org = ray_org;
        ray.dir = ray_dir;
        occluded = ray.tfar < 0.0f;
        instance_id_stack::pop(user_context);      
      }
      return occluded;
    }
    
    bool InstanceIntersector1MB::pointQuery(PointQuery* query, PointQueryContext* context, const InstancePrimitive& prim)
    {
      const Instance* instance = prim.instance;

      const AffineSpace3fa local2world = instance->getLocal2World(query->time);
      const AffineSpace3fa world2local = instance->getWorld2Local(query->time);
      float similarityScale = 0.f;
      const bool similtude = context->query_type == POINT_QUERY_TYPE_SPHERE
                           && similarityTransform(world2local, &similarityScale);

      if (likely(pushInstance(context->userContext, prim.instID_, world2local, local2world)))
      {
        PointQuery query_inst;
        query_inst.time = query->time;
        query_inst.p = xfmPoint(world2local, query->p); 
        query_inst.radius = query->radius * similarityScale;
        
        PointQueryContext context_inst(
          (Scene*)instance->object, 
          context->query_ws, 
          similtude ? POINT_QUERY_TYPE_SPHERE : POINT_QUERY_TYPE_AABB,
          context->func, 
          context->userContext,
          similarityScale,
          context->userPtr); 

        bool changed = instance->object->intersectors.pointQuery(&query_inst, &context_inst);
        popInstance(context->userContext);
        return changed;
      }
      return false;
    }
    
    template<int K>
    void InstanceIntersectorK<K>::intersect(const vbool<K>& valid_i, const Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const InstancePrimitive& prim)
    {
      vbool<K> valid = valid_i;
      const Instance* instance = prim.instance;
      //ray.geomID = 10;
      
      /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
      valid &= (ray.mask & instance->mask) != 0;
      if (none(valid)) return;
#endif
        
      RTCIntersectContext* user_context = context->user;
      if (likely(instance_id_stack::push(user_context, prim.instID_)))
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
        instance_id_stack::pop(user_context);
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
      if (likely(instance_id_stack::push(user_context, prim.instID_)))
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
        instance_id_stack::pop(user_context);
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
      if (likely(instance_id_stack::push(user_context, prim.instID_)))
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
        instance_id_stack::pop(user_context);
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
      if (likely(instance_id_stack::push(user_context, prim.instID_)))
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
        instance_id_stack::pop(user_context);
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
