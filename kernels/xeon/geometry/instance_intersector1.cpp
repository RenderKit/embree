// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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
    void FastInstanceIntersector1::intersect(const UserGeometryScene::Instance* instance, Ray& ray)
    {
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      const int ray_geomID = ray.geomID;
      ray.org = xfmPoint (instance->world2local,ray_org);
      ray.dir = xfmVector(instance->world2local,ray_dir);
      ray.geomID = -1;
      instance->object->intersect((RTCRay&)ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
      if (ray.geomID == -1) ray.geomID = ray_geomID;
      else ray.instID = instance->id;
    }
    
    void FastInstanceIntersector1::occluded (const UserGeometryScene::Instance* instance, Ray& ray)
    {
      const Vec3fa ray_org = ray.org;
      const Vec3fa ray_dir = ray.dir;
      ray.org = xfmPoint (instance->world2local,ray_org);
      ray.dir = xfmVector(instance->world2local,ray_dir);
      instance->object->occluded((RTCRay&)ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
    }
    
    DEFINE_INTERSECTOR1(InstanceIntersector1,FastInstanceIntersector1);
  }
}
