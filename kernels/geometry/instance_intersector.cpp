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

#include "instance_intersector.h"
#include "../common/scene.h"

namespace embree
{
  namespace isa
  {
    void InstanceBoundsFunction(const struct RTCBoundsFunctionArguments* const args)
    {
      const Instance* instance = (const Instance*) args->geomUserPtr;
      unsigned int itime = args->timeStep;

      assert(itime < instance->numTimeSteps);
      unsigned num_time_segments = instance->numTimeSegments();
      if (num_time_segments == 0) {
        *((BBox3fa*)args->bounds_o) = xfmBounds(instance->local2world[itime],instance->object->bounds.bounds());
      }
      else {
        const float ftime = float(itime) / float(num_time_segments);
        const BBox3fa obounds = instance->object->bounds.interpolate(ftime);
        *((BBox3fa*)args->bounds_o) = xfmBounds(instance->local2world[itime],obounds);
      }
    }

    RTCBoundsFunction InstanceBoundsFunc() {
      return InstanceBoundsFunction;
    }

    __forceinline void FastInstanceIntersectorN::intersect1(const struct RTCIntersectFunctionNArguments* const args)
    {
      const Instance* instance = (const Instance*) args->geomUserPtr;
      RTCIntersectContext* user_context = args->context;
      Ray& ray = *(Ray*)args->ray;
      
      const AffineSpace3fa world2local = 
        likely(instance->numTimeSteps == 1) ? instance->getWorld2Local() : instance->getWorld2Local(ray.time);
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      ray.org = Vec3fa(xfmPoint (world2local,ray_org),ray.tnear());
      ray.dir = Vec3fa(xfmVector(world2local,ray_dir),ray.tfar());      
      user_context->instID = instance->geomID;
      IntersectContext context(instance->object,user_context);
      instance->object->intersectors.intersect((RTCRayHit&)ray,&context);
      user_context->instID = -1;
      ray.org = ray_org;
      ray.dir = Vec3fa(ray_dir,ray.tfar());
    }
    
    __forceinline void FastInstanceIntersectorN::occluded1(const struct RTCOccludedFunctionNArguments* const args)
    {
      const Instance* instance = (const Instance*) args->geomUserPtr;
      RTCIntersectContext* user_context = args->context;
      Ray& ray = *(Ray*)args->ray;
      
      const AffineSpace3fa world2local = 
        likely(instance->numTimeSteps == 1) ? instance->getWorld2Local() : instance->getWorld2Local(ray.time);
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      ray.org = Vec3fa(xfmPoint (world2local,ray_org),ray.tnear());
      ray.dir = Vec3fa(xfmVector(world2local,ray_dir),ray.tfar());
      user_context->instID = instance->geomID;
      IntersectContext context(instance->object,user_context);
      instance->object->intersectors.occluded((RTCRayHit&)ray,&context);
      user_context->instID = -1;
      ray.org = ray_org;
      ray.dir = Vec3fa(ray_dir,ray.tfar());
    }

    template<int K>
    __forceinline void intersectObject(vint<K>* valid, Scene* object, IntersectContext* context, RayK<K>& ray);
    template<int K>
    __forceinline void occludedObject(vint<K>* valid, Scene* object, IntersectContext* context, RayK<K>& ray);
        
#if defined (__SSE__)
    template<> __forceinline void intersectObject<4>(vint4* valid, Scene* object, IntersectContext* context, Ray4& ray) { object->intersectors.intersect4(valid,(RTCRayHit4&)ray,context); }
    template<> __forceinline void occludedObject <4>(vint4* valid, Scene* object, IntersectContext* context, Ray4& ray) { object->intersectors.occluded4 (valid,(RTCRayHit4&)ray,context); }
#endif
#if defined (__AVX__)
    template<> __forceinline void intersectObject<8>(vint8* valid, Scene* object, IntersectContext* context, Ray8& ray) { object->intersectors.intersect8(valid,(RTCRayHit8&)ray,context); }
    template<> __forceinline void occludedObject <8>(vint8* valid, Scene* object, IntersectContext* context, Ray8& ray) { object->intersectors.occluded8 (valid,(RTCRayHit8&)ray,context); }
#endif
#if defined (__AVX512F__)
    template<> __forceinline void intersectObject<16>(vint16* valid, Scene* object, IntersectContext* context, Ray16& ray) { object->intersectors.intersect16(valid,(RTCRayHit16&)ray,context); }
    template<> __forceinline void occludedObject <16>(vint16* valid, Scene* object, IntersectContext* context, Ray16& ray) { object->intersectors.occluded16 (valid,(RTCRayHit16&)ray,context); }
#endif

    template<int N>
    __noinline void FastInstanceIntersectorN::intersectN(const struct RTCIntersectFunctionNArguments* const args)
    {
      const vint<N>* validi = (const vint<N>*) args->valid;
      const Instance* instance = (const Instance*) args->geomUserPtr;
      RTCIntersectContext* user_context = args->context;
      RayK<N>& ray = *(RayK<N>*)args->ray;
      
      AffineSpace3vf<N> world2local;
      const vbool<N> valid = *validi == vint<N>(-1);
      if (likely(instance->numTimeSteps == 1)) world2local = instance->getWorld2Local();
      else                                     world2local = instance->getWorld2Local<N>(valid,ray.time);

      const Vec3vf<N> ray_org = ray.org;
      const Vec3vf<N> ray_dir = ray.dir;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      user_context->instID = instance->geomID;
      IntersectContext context(instance->object,user_context); 
      intersectObject((vint<N>*)validi,instance->object,&context,ray);
      user_context->instID = -1;
      ray.org = ray_org;
      ray.dir = ray_dir;
    }

    template<int N>
    __noinline void FastInstanceIntersectorN::occludedN(const struct RTCOccludedFunctionNArguments* const args)
    {
      const vint<N>* validi = (const vint<N>*) args->valid;
      const Instance* instance = (const Instance*) args->geomUserPtr;
      RTCIntersectContext* user_context = args->context;
      RayK<N>& ray = *(RayK<N>*)args->ray;
      
      AffineSpace3vf<N> world2local;
      const vbool<N> valid = *validi == vint<N>(-1);
      if (likely(instance->numTimeSteps == 1)) world2local = instance->getWorld2Local();
      else                                     world2local = instance->getWorld2Local<N>(valid,ray.time);

      const Vec3vf<N> ray_org = ray.org;
      const Vec3vf<N> ray_dir = ray.dir;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      user_context->instID = instance->geomID;
      IntersectContext context(instance->object,user_context);
      occludedObject((vint<N>*)validi,instance->object,&context,ray);
      user_context->instID = -1;
      ray.org = ray_org;
      ray.dir = ray_dir;
    }

    void FastInstanceIntersectorN::intersect(const struct RTCIntersectFunctionNArguments* const args)
    {
      const unsigned int N = args->N;
      if      (likely(N ==  1)) return intersect1(args);
      else if (likely(N ==  4)) return intersectN<4>(args);
#if defined(__AVX__)
      else if (likely(N ==  8)) return intersectN<8>(args);
#endif
#if defined(__AVX512F__)
      else if (likely(N == 16)) return intersectN<16>(args);
#endif
      assert(false);
    }
    
    void FastInstanceIntersectorN::occluded(const struct RTCOccludedFunctionNArguments* const args)
    {
      const unsigned int N = args->N;
      if      (likely(N ==  1)) return occluded1(args);
      else if (likely(N ==  4)) return occludedN<4>(args);    
#if defined(__AVX__)
      else if (likely(N ==  8)) return occludedN<8>(args);
#endif
#if defined(__AVX512F__)
      else if (likely(N == 16)) return occludedN<16>(args);
#endif
      assert(false);
    }
    
    DEFINE_SET_INTERSECTORN(InstanceIntersectorN,FastInstanceIntersectorN);
  }
}
