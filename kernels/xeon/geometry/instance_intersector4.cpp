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

#include "instance_intersector4.h"

namespace embree
{
  namespace isa
  {
    typedef AffineSpaceT<LinearSpace3<sse3f> > AffineSpace3fSSE;
    
    void FastInstanceIntersector4::intersect(sseb* valid, const UserGeometryScene::Instance* instance, Ray4& ray)
    {
      const sse3f ray_org = ray.org;
      const sse3f ray_dir = ray.dir;
      const ssei ray_geomID = ray.geomID;
      const AffineSpace3fSSE world2local(instance->world2local);
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      ray.geomID = -1;
      instance->object->intersect4(valid,(RTCRay4&)ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
      sseb nohit = ray.geomID == ssei(-1);
      ray.geomID = select(nohit,ray_geomID,ray.geomID);
      ray.instID = select(nohit,ray.instID,ssei(instance->id));
    }
    
    void FastInstanceIntersector4::occluded (sseb* valid, const UserGeometryScene::Instance* instance, Ray4& ray)
    {
      const sse3f ray_org = ray.org;
      const sse3f ray_dir = ray.dir;
      const ssei ray_geomID = ray.geomID;
      const AffineSpace3fSSE world2local(instance->world2local);
      ray.org = xfmPoint (world2local,ray_org);
      ray.dir = xfmVector(world2local,ray_dir);
      instance->object->occluded4(valid,(RTCRay4&)ray);
      ray.org = ray_org;
      ray.dir = ray_dir;
    }

    DEFINE_INTERSECTOR4    (InstanceIntersector4,FastInstanceIntersector4);
#if !defined(__AVX__)
    DEFINE_INTERSECTOR4TO8 (InstanceIntersector8,FastInstanceIntersector4,UserGeometryScene::Instance);
    DEFINE_INTERSECTOR4TO16(InstanceIntersector16,FastInstanceIntersector4,UserGeometryScene::Instance);
#endif
  }
}
