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

#include "instance_intersector1.h"

namespace embree
{
  namespace isa
  {
    void InstanceBoundsFunction(void* userPtr, const Instance* instance, size_t item, BBox3fa* bounds_o) 
    {
      if (instance->numTimeSteps == 1) {
        bounds_o[0] = xfmBounds(instance->local2world[0],instance->object->bounds);
      } else {
        bounds_o[0] = xfmBounds(instance->local2world[0],instance->object->bounds);
        bounds_o[1] = xfmBounds(instance->local2world[1],instance->object->bounds);
      }
    }

    RTCBoundsFunc2 InstanceBoundsFunc = (RTCBoundsFunc2) InstanceBoundsFunction;

    void FastInstanceIntersector1::intersect(const Instance* instance, Ray& ray, size_t item)
    {
      const AffineSpace3fa world2local = instance->getWorld2LocalSpecial(ray.time);
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      const int ray_geomID = ray.geomID;
      const int ray_instID = ray.instID;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.geomID = -1;
      ray.instID = instance->id;
      instance->object->intersect((RTCRay&)ray,nullptr);
      ray.org = ray_org;
      ray.dir = ray_dir;
      if (ray.geomID == -1) {
        ray.geomID = ray_geomID;
        ray.instID = ray_instID;
      }
    }
    
    void FastInstanceIntersector1::occluded (const Instance* instance, Ray& ray, size_t item)
    {
      const AffineSpace3fa world2local = instance->getWorld2LocalSpecial(ray.time);
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.instID = instance->id;
      instance->object->occluded((RTCRay&)ray,nullptr);
      ray.org = ray_org;
      ray.dir = ray_dir;
    }
    
    DEFINE_SET_INTERSECTOR1(InstanceIntersector1,FastInstanceIntersector1);

    void FastInstanceIntersector1M::intersect(const Instance* instance, Ray** rays, size_t M, size_t item, const RTCIntersectionContext* context)
    {
      assert(M<MAX_INTERNAL_STREAM_SIZE);
      Vec3fa ray_org[MAX_INTERNAL_STREAM_SIZE];
      Vec3fa ray_dir[MAX_INTERNAL_STREAM_SIZE];
      int ray_geomID[MAX_INTERNAL_STREAM_SIZE];
      int ray_instID[MAX_INTERNAL_STREAM_SIZE];
      AffineSpace3fa world2local = instance->getWorld2Local();

      for (size_t i=0; i<M; i++)
      {
        if (unlikely(instance->numTimeSteps != 1)) 
          world2local = instance->getWorld2Local(rays[i]->time);

        const Vec3fa org = rays[i]->org;
        const Vec3fa dir = rays[i]->dir;
        ray_org[i] = org;
        ray_dir[i] = dir;
        ray_geomID[i] = rays[i]->geomID;
        ray_instID[i] = rays[i]->instID;
        rays[i]->org = xfmPoint (world2local,org);
        rays[i]->dir = xfmVector(world2local,dir);
        rays[i]->geomID = -1;
        rays[i]->instID = instance->id;
      }

      instance->object->intersectN((RTCRay**)rays,M,context);
        
      for (size_t i=0; i<M; i++)
      {
        rays[i]->org = ray_org[i];
        rays[i]->dir = ray_dir[i];
        if (rays[i]->geomID == -1) {
          rays[i]->geomID = ray_geomID[i];
          rays[i]->instID = ray_instID[i];
        }
      }
    }
    
    void FastInstanceIntersector1M::occluded (const Instance* instance, Ray** rays, size_t M, size_t item, const RTCIntersectionContext* context)
    {
      assert(M<MAX_INTERNAL_STREAM_SIZE);
      Vec3fa ray_org[MAX_INTERNAL_STREAM_SIZE];
      Vec3fa ray_dir[MAX_INTERNAL_STREAM_SIZE];
      AffineSpace3fa world2local = instance->getWorld2Local();
      
      for (size_t i=0; i<M; i++)
      {
        if (unlikely(instance->numTimeSteps != 1)) 
          world2local = instance->getWorld2Local(rays[i]->time);

        const Vec3fa org = rays[i]->org;
        const Vec3fa dir = rays[i]->dir;
        ray_org[i] = org;
        ray_dir[i] = dir;
        rays[i]->org = xfmPoint (world2local,org);
        rays[i]->dir = xfmVector(world2local,dir);
        rays[i]->instID = instance->id;
      }

      instance->object->occludedN((RTCRay**)rays,M,context);
        
      for (size_t i=0; i<M; i++)
      {
        rays[i]->org = ray_org[i];
        rays[i]->dir = ray_dir[i];
      }
    }

    DEFINE_SET_INTERSECTOR1M(InstanceIntersector1M,FastInstanceIntersector1M);
  }
}
