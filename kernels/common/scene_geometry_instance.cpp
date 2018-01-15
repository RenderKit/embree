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

#include "scene_geometry_instance.h"
#include "scene.h"

namespace embree
{
  GeometryInstance::GeometryInstance (Scene* scene, Geometry* geom) 
    : Geometry(scene,Type(geom->type | INSTANCE), 1, geom->numTimeSteps, geom->flags), local2world(one), world2local(one), geom(geom) 
  {
    enabling();
  }

  void GeometryInstance::count(Geometry* geom, ssize_t f)
  {
    if (geom->numTimeSteps == 1)
    {
      switch (geom->type) {
      case TRIANGLE_MESH: scene->instanced.numTriangles      += f*ssize_t(geom->size()); break;
      case USER_GEOMETRY: scene->instanced.numUserGeometries += f*ssize_t(geom->size()); break;
      case BEZIER_CURVES: scene->instanced.numBezierCurves   += f*ssize_t(geom->size()); break;
      case SUBDIV_MESH  : scene->instanced.numSubdivPatches  += f*ssize_t(geom->size()); break;
      default           : throw_RTCError(RTC_INVALID_OPERATION,"cannot instantiate this geometry ");
      };
    }
    else
    {
      switch (geom->type) {
      case TRIANGLE_MESH: scene->instancedMB.numTriangles      += f*ssize_t(geom->size()); break;
      case USER_GEOMETRY: scene->instancedMB.numUserGeometries += f*ssize_t(geom->size()); break;
      case BEZIER_CURVES: scene->instancedMB.numBezierCurves   += f*ssize_t(geom->size()); break;
      case SUBDIV_MESH  : scene->instancedMB.numSubdivPatches  += f*ssize_t(geom->size()); break;
      default           : throw_RTCError(RTC_INVALID_OPERATION,"cannot instantiate this geometry");
      };
    }
  }

  void GeometryInstance::enabling () 
  {
    geom->used++;
    if (geom->getType() == Geometry::GROUP) {
      GeometryGroup* group = (GeometryGroup*) geom;
      for (size_t i=0; i<group->size(); i++)
        count((*group)[i],+1);
    }
    else {
      count(geom,+1);
    }
  }

  void GeometryInstance::disabling() 
  {
    geom->used--;
     geom->used++;
    if (geom->getType() == Geometry::GROUP) {
      GeometryGroup* group = (GeometryGroup*) geom;
      for (size_t i=0; i<group->size(); i++)
        count((*group)[i],-1);
    }
    else {
      count(geom,-1);
    }
  }
  
  void GeometryInstance::setMask (unsigned mask) 
  {
    if (scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
    Geometry::update();
  } 

  void GeometryInstance::setTransform(const AffineSpace3fa& xfm, size_t timeStep)
  {
    if (scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (timeStep != 0)
      throw_RTCError(RTC_INVALID_OPERATION,"geometry instances only support a single timestep");

    local2world = xfm;
    world2local = rcp(xfm);
  }

  GeometryGroup::GeometryGroup (Scene* scene, RTCGeometryFlags gflags, const std::vector<Geometry*>& geometries) 
    : Geometry(scene,GROUP, geometries.size(), 1, gflags), geometries(geometries)
  {
    enabling();
  }

  void GeometryGroup::enabling () 
  {
  }

  void GeometryGroup::disabling() 
  {
  }
  
  void GeometryGroup::setMask (unsigned mask) 
  {
    if (scene->isStatic() && scene->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
    Geometry::update();
  } 
}
