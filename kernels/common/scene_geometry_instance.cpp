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

  void GeometryInstance::count(ssize_t f)
  {
    if (geom->numTimeSteps == 1)
    {
      switch (geom->type) {
      case TRIANGLE_MESH: atomic_add(&parent->instanced1.numTriangles     ,f*ssize_t(geom->size())); break;
      case USER_GEOMETRY: atomic_add(&parent->instanced1.numUserGeometries,f*ssize_t(geom->size())); break;
      case BEZIER_CURVES: atomic_add(&parent->instanced1.numBezierCurves  ,f*ssize_t(geom->size())); break;
      case SUBDIV_MESH  : atomic_add(&parent->instanced1.numSubdivPatches ,f*ssize_t(geom->size())); break;
      default           : throw_RTCError(RTC_INVALID_OPERATION,"cannot instantiate this geometry ");
      };
    }
    else
    {
      switch (geom->type) {
      case TRIANGLE_MESH: atomic_add(&parent->instanced2.numTriangles     ,f*ssize_t(geom->size())); break;
      case USER_GEOMETRY: atomic_add(&parent->instanced2.numUserGeometries,f*ssize_t(geom->size())); break;
      case BEZIER_CURVES: atomic_add(&parent->instanced2.numBezierCurves  ,f*ssize_t(geom->size())); break;
      case SUBDIV_MESH  : atomic_add(&parent->instanced2.numSubdivPatches ,f*ssize_t(geom->size())); break;
      default           : throw_RTCError(RTC_INVALID_OPERATION,"cannot instantiate this geometry");
      };
    }
  }

  void GeometryInstance::enabling () 
  {
    atomic_add(&geom->used,+1);
    count(+1);
  }

  void GeometryInstance::disabling() 
  {
    atomic_add(&geom->used,-1);
    count(-1);
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
