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
    void InstanceBoundsFunction(void* userPtr, const Instance* instance, size_t item, size_t itime, BBox3fa& bounds_o)
    {
      assert(itime < instance->numTimeSteps);
      unsigned num_time_segments = instance->numTimeSegments();
      if (num_time_segments == 0) {
        bounds_o = xfmBounds(instance->local2world[itime],instance->object->bounds.bounds());
      }
      else {
        const float ftime = float(itime) / float(num_time_segments);
        const BBox3fa obounds = instance->object->bounds.interpolate(ftime);
        bounds_o = xfmBounds(instance->local2world[itime],obounds);
      }
    }

    RTCBoundsFunc3 InstanceBoundsFunc() {
      return (RTCBoundsFunc3) InstanceBoundsFunction;
    }

    __forceinline void FastInstanceIntersectorN::intersect1(const Instance* instance, const RTCIntersectContext* user_context, Ray& ray, size_t item)
    {
      const AffineSpace3fa world2local = 
        likely(instance->numTimeSteps == 1) ? instance->getWorld2Local() : instance->getWorld2Local(ray.time);
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      const int ray_geomID = ray.geomID;
      const int ray_instID = ray.instID;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      ray.instID = instance->geomID;
      IntersectContext context(instance->object,user_context);
      instance->object->intersectors.intersect((RTCRay&)ray,&context);
      ray.org = ray_org;
      ray.dir = ray_dir;
      if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
        ray.geomID = ray_geomID;
        ray.instID = ray_instID;
      }
    }
    
    __forceinline void FastInstanceIntersectorN::occluded1(const Instance* instance, const RTCIntersectContext* user_context, Ray& ray, size_t item)
    {
      const AffineSpace3fa world2local = 
        likely(instance->numTimeSteps == 1) ? instance->getWorld2Local() : instance->getWorld2Local(ray.time);
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.instID = instance->geomID;
      IntersectContext context(instance->object,user_context);
      instance->object->intersectors.occluded((RTCRay&)ray,&context);
      ray.org = ray_org;
      ray.dir = ray_dir;
    }

    template<int K>
    __forceinline void intersectObject(vint<K>* valid, Scene* object, IntersectContext* context, RayK<K>& ray);
    template<int K>
    __forceinline void occludedObject(vint<K>* valid, Scene* object, IntersectContext* context, RayK<K>& ray);
        
#if defined (__SSE__)
    template<> __forceinline void intersectObject<4>(vint4* valid, Scene* object, IntersectContext* context, Ray4& ray) { object->intersectors.intersect4(valid,(RTCRay4&)ray,context); }
    template<> __forceinline void occludedObject <4>(vint4* valid, Scene* object, IntersectContext* context, Ray4& ray) { object->intersectors.occluded4 (valid,(RTCRay4&)ray,context); }
#endif
#if defined (__AVX__)
    template<> __forceinline void intersectObject<8>(vint8* valid, Scene* object, IntersectContext* context, Ray8& ray) { object->intersectors.intersect8(valid,(RTCRay8&)ray,context); }
    template<> __forceinline void occludedObject <8>(vint8* valid, Scene* object, IntersectContext* context, Ray8& ray) { object->intersectors.occluded8 (valid,(RTCRay8&)ray,context); }
#endif
#if defined (__AVX512F__)
    template<> __forceinline void intersectObject<16>(vint16* valid, Scene* object, IntersectContext* context, Ray16& ray) { object->intersectors.intersect16(valid,(RTCRay16&)ray,context); }
    template<> __forceinline void occludedObject <16>(vint16* valid, Scene* object, IntersectContext* context, Ray16& ray) { object->intersectors.occluded16 (valid,(RTCRay16&)ray,context); }
#endif

    template<int N>
    __noinline void FastInstanceIntersectorN::intersectN(vint<N>* validi, const Instance* instance, const RTCIntersectContext* user_context, RayK<N>& ray, size_t item)
    {
      AffineSpace3vf<N> world2local;
      const vbool<N> valid = *validi == vint<N>(-1);
      if (likely(instance->numTimeSteps == 1)) world2local = instance->getWorld2Local();
      else                                     world2local = instance->getWorld2Local<N>(valid,ray.time);

      const Vec3vf<N> ray_org = ray.org;
      const Vec3vf<N> ray_dir = ray.dir;
      const vint<N> ray_geomID = ray.geomID;
      const vint<N> ray_instID = ray.instID;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      ray.instID = instance->geomID;
      IntersectContext context(instance->object,user_context); 
      intersectObject((vint<N>*)validi,instance->object,&context,ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
      vbool<N> nohit = ray.geomID == vint<N>(RTC_INVALID_GEOMETRY_ID);
      ray.geomID = select(nohit,ray_geomID,ray.geomID);
      ray.instID = select(nohit,ray_instID,ray.instID);
    }

    template<int N>
    __noinline void FastInstanceIntersectorN::occludedN(vint<N>* validi, const Instance* instance, const RTCIntersectContext* user_context, RayK<N>& ray, size_t item)
    {
      AffineSpace3vf<N> world2local;
      const vbool<N> valid = *validi == vint<N>(-1);
      if (likely(instance->numTimeSteps == 1)) world2local = instance->getWorld2Local();
      else                                     world2local = instance->getWorld2Local<N>(valid,ray.time);

      const Vec3vf<N> ray_org = ray.org;
      const Vec3vf<N> ray_dir = ray.dir;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.instID = instance->geomID;
      IntersectContext context(instance->object,user_context);
      occludedObject((vint<N>*)validi,instance->object,&context,ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
    }

    void FastInstanceIntersectorN::intersect(int* validi, void* ptr, const RTCIntersectContext* user_context, RTCRayN* rays, size_t N, size_t item)
    { 
      if (likely(N == 1)) {
        assert(*validi == -1);
        return intersect1((const Instance*)ptr,user_context,*(Ray*)rays,item);
      }
      else if (likely(N == 4)) {
        return intersectN((vint4*)validi,(const Instance*)ptr,user_context,*(Ray4*)rays,item);
      }
#if defined(__AVX__)
      else if (likely(N == 8)) {
        return intersectN((vint8*)validi,(const Instance*)ptr,user_context,*(Ray8*)rays,item);
      }
#endif
#if defined(__AVX512F__)
      else if (likely(N == 16)) {
        return intersectN((vint16*)validi,(const Instance*)ptr,user_context,*(Ray16*)rays,item);
      }
#endif
      assert(false);
    }
    
    void FastInstanceIntersectorN::occluded(int* validi, void* ptr, const RTCIntersectContext* user_context, RTCRayN* rays, size_t N, size_t item)
    {
      if (likely(N == 1)) {
        assert(*validi == -1);
        return occluded1((const Instance*)ptr,user_context,*(Ray*)rays,item);
      }
      else if (likely(N == 4)) {
        return occludedN((vint4*)validi,(const Instance*)ptr,user_context,*(Ray4*)rays,item);
      }
#if defined(__AVX__)
      else if (likely(N == 8)) {
        return occludedN((vint8*)validi,(const Instance*)ptr,user_context,*(Ray8*)rays,item);
      }
#endif
#if defined(__AVX512F__)
      else if (likely(N == 16)) {
        return occludedN((vint16*)validi,(const Instance*)ptr,user_context,*(Ray16*)rays,item);
      }
#endif
      assert(false);
    }
    
    DEFINE_SET_INTERSECTORN(InstanceIntersectorN,FastInstanceIntersectorN);
  }
}
