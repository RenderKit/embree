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

#include "instance_intersector1.h"

namespace embree
{
  namespace isa
  {
    void InstanceBoundsFunction(void* userPtr, const Instance* instance, size_t item, BBox3fa* bounds_o) 
    {
      *bounds_o = xfmBounds(instance->local2world[0],instance->object->bounds);
    }

    RTCBoundsFunc2 InstanceBoundsFunc = (RTCBoundsFunc2) InstanceBoundsFunction;

    void FastInstanceIntersector1::intersect(const Instance* instance, Ray& ray, size_t item)
    {
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      const int ray_geomID = ray.geomID;
      const int ray_instID = ray.instID;
      ray.org = xfmPoint (instance->world2local[0],ray_org);
      ray.dir = xfmVector(instance->world2local[0],ray_dir);
      ray.geomID = -1;
      ray.instID = instance->id;
      instance->object->intersect((RTCRay&)ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
      if (ray.geomID == -1) {
        ray.geomID = ray_geomID;
        ray.instID = ray_instID;
      }
    }
    
    void FastInstanceIntersector1::occluded (const Instance* instance, Ray& ray, size_t item)
    {
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      ray.org = xfmPoint (instance->world2local[0],ray_org);
      ray.dir = xfmVector(instance->world2local[0],ray_dir);
      ray.instID = instance->id;
      instance->object->occluded((RTCRay&)ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
    }
    
    DEFINE_SET_INTERSECTOR1(InstanceIntersector1,FastInstanceIntersector1);
  }
}
