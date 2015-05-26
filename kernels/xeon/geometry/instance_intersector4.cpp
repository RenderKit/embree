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

#include "instance_intersector4.h"

namespace embree
{
  namespace isa
  {
    typedef AffineSpaceT<LinearSpace3<Vec3f4> > AffineSpace3faSSE;
    
    void FastInstanceIntersector4::intersect(bool4* valid, const Instance* instance, Ray4& ray, size_t item)
    {
      const Vec3f4 ray_org = ray.org;
      const Vec3f4 ray_dir = ray.dir;
      const int4 ray_geomID = ray.geomID;
      const int4 ray_instID = ray.instID;
      const AffineSpace3faSSE world2local(instance->world2local);
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.geomID = -1;
      ray.instID = instance->id;
      instance->object->intersect4(valid,(RTCRay4&)ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
      bool4 nohit = ray.geomID == int4(-1);
      ray.geomID = select(nohit,ray_geomID,ray.geomID);
      ray.instID = select(nohit,ray_instID,ray.instID);
    }
    
    void FastInstanceIntersector4::occluded (bool4* valid, const Instance* instance, Ray4& ray, size_t item)
    {
      const Vec3f4 ray_org = ray.org;
      const Vec3f4 ray_dir = ray.dir;
      const int4 ray_geomID = ray.geomID;
      const AffineSpace3faSSE world2local(instance->world2local);
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.instID = instance->id;
      instance->object->occluded4(valid,(RTCRay4&)ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
    }

    DEFINE_SET_INTERSECTOR4(InstanceIntersector4,FastInstanceIntersector4);
  }
}
