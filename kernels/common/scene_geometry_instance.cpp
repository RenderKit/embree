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

#include "scene_geometry_instance.h"
#include "scene.h"

namespace embree
{
  GeometryInstance::GeometryInstance (Scene* parent, Geometry* geom) 
    : Geometry(parent,Type(geom->type | INSTANCE), 1, geom->numTimeSteps, geom->flags), local2world(one), world2local(one), geom(geom) 
  {
    enabling();
  }

  void GeometryInstance::enabling () {
    atomic_add(&geom->used,+1);
    atomic_add(&parent->numInstancedTriangles,+ssize_t(geom->size())); // FIXME: currently only triangle meshes are supported
  }

  void GeometryInstance::disabling() {
     atomic_add(&geom->used,-1);
     atomic_add(&parent->numInstancedTriangles,-ssize_t(geom->size())); // FIXME: currently only triangle meshes are supported
  }
  
  void GeometryInstance::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
    Geometry::update();
  } 

  void GeometryInstance::setTransform(const AffineSpace3fa& xfm)
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    local2world = xfm;
    world2local = rcp(xfm);
  }
}
