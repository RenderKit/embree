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

#include "instance_intersector.h"
#include "../common/scene.h"

namespace embree
{
  namespace isa
  {
    template<int K>
    __forceinline void intersectObject(vint<K>* valid, Scene* object, RayK<K>& ray);
    template<int K>
    __forceinline void occludedObject(vint<K>* valid, Scene* object, RayK<K>& ray);
        
#if defined (__SSE__)
    template<> __forceinline void intersectObject<4>(vint4* valid, Scene* object, Ray4& ray) { object->intersect4(valid,(RTCRay4&)ray,nullptr); }
    template<> __forceinline void occludedObject <4>(vint4* valid, Scene* object, Ray4& ray) { object->occluded4 (valid,(RTCRay4&)ray,nullptr); }
#endif
#if defined (__AVX__)
    template<> __forceinline void intersectObject<8>(vint8* valid, Scene* object, Ray8& ray) { object->intersect8(valid,(RTCRay8&)ray,nullptr); }
    template<> __forceinline void occludedObject <8>(vint8* valid, Scene* object, Ray8& ray) { object->occluded8 (valid,(RTCRay8&)ray,nullptr); }
#endif
#if defined (__AVX512F__)
    template<> __forceinline void intersectObject<16>(vint16* valid, Scene* object, Ray16& ray) { object->intersect16(valid,(RTCRay16&)ray,nullptr); }
    template<> __forceinline void occludedObject <16>(vint16* valid, Scene* object, Ray16& ray) { object->occluded16 (valid,(RTCRay16&)ray,nullptr); }
#endif

    template<int K>
    void FastInstanceIntersectorK<K>::intersect(vint<K>* valid, const Instance* instance, RayK<K>& ray, size_t item)
    {
      typedef Vec3<vfloat<K>> Vec3vfK;
      typedef AffineSpaceT<LinearSpace3<Vec3vfK>> AffineSpace3vfK;
      
      AffineSpace3vfK world2local;
      if (likely(instance->numTimeSteps == 1)) {
        world2local = instance->world2local[0];
      } else {
        vfloat<K> t1 = ray.time, t0 = vfloat<K>(1.0f)-t1;
        world2local = rcp(t0*AffineSpace3vfK(instance->local2world[0]) + t1*AffineSpace3vfK(instance->local2world[1]));
      }
      
      const Vec3vfK ray_org = ray.org;
      const Vec3vfK ray_dir = ray.dir;
      const vint<K> ray_geomID = ray.geomID;
      const vint<K> ray_instID = ray.instID;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      ray.instID = instance->id;
      intersectObject(valid,instance->object,ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
      vbool<K> nohit = ray.geomID == vint<K>(RTC_INVALID_GEOMETRY_ID);
      ray.geomID = select(nohit,ray_geomID,ray.geomID);
      ray.instID = select(nohit,ray_instID,ray.instID);
    }
    
    template<int K>
    void FastInstanceIntersectorK<K>::occluded(vint<K>* valid, const Instance* instance, RayK<K>& ray, size_t item)
    {
      typedef Vec3<vfloat<K>> Vec3vfK;
      typedef AffineSpaceT<LinearSpace3<Vec3vfK>> AffineSpace3vfK;

      AffineSpace3vfK world2local;
      if (likely(instance->numTimeSteps == 1)) {
        world2local = instance->world2local[0];
      } else {
        vfloat<K> t1 = ray.time, t0 = vfloat<K>(1.0f)-t1;
        world2local = rcp(t0*AffineSpace3vfK(instance->local2world[0]) + t1*AffineSpace3vfK(instance->local2world[1]));
      }

      const Vec3vfK ray_org = ray.org;
      const Vec3vfK ray_dir = ray.dir;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.instID = instance->id;
      occludedObject(valid,instance->object,ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
    }

    DEFINE_SET_INTERSECTOR4(InstanceIntersector4,FastInstanceIntersector4);

#if defined(__AVX__)
    DEFINE_SET_INTERSECTOR8(InstanceIntersector8,FastInstanceIntersector8);
#endif

#if defined(__AVX512F__)
    DEFINE_SET_INTERSECTOR16(InstanceIntersector16,FastInstanceIntersector16);
#endif
  }
}
