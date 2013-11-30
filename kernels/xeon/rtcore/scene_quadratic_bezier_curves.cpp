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

#include "scene_quadratic_bezier_curves.h"
#include "scene.h"

namespace embree
{
  QuadraticBezierCurvesScene::QuadraticBezierCurves::QuadraticBezierCurves (Scene* parent, RTCFlags flags, size_t numCurves, size_t numVertices) 
    : Geometry(parent,QUADRATIC_BEZIER_CURVES,numCurves,flags), mask(-1), built(false),
      curves(NULL), numCurves(numCurves), mappedCurves(false), 
      vertices(NULL), numVertices(numVertices), mappedVertices(false)
  {
    curves   = (Curve* ) alignedMalloc(numCurves*sizeof(Curve));
    vertices = (Vertex*) alignedMalloc(numVertices*sizeof(Vertex));
  }
  
  QuadraticBezierCurvesScene::QuadraticBezierCurves::~QuadraticBezierCurves () 
  {
    alignedFree(curves);
    alignedFree(vertices);
  }
  
  void QuadraticBezierCurvesScene::QuadraticBezierCurves::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    this->mask = mask; 
  }

  void QuadraticBezierCurvesScene::QuadraticBezierCurves::enable () 
  {
    if (parent->isStatic() || anyMappedBuffers()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    Geometry::enable();
  }

  void QuadraticBezierCurvesScene::QuadraticBezierCurves::update () 
  {
    if (parent->isStatic() || anyMappedBuffers()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    Geometry::update();
  }

  void QuadraticBezierCurvesScene::QuadraticBezierCurves::disable () 
  {
    if (parent->isStatic() || anyMappedBuffers()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    Geometry::disable();
  }

  void QuadraticBezierCurvesScene::QuadraticBezierCurves::erase () 
  {
    if (parent->isStatic() || anyMappedBuffers()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    Geometry::erase();
  }

  void* QuadraticBezierCurvesScene::QuadraticBezierCurves::map(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      recordError(RTC_INVALID_OPERATION);
      return NULL;
    }

    switch (type) {
    case RTC_INDEX_BUFFER : 
    {
      if (mappedCurves || (built && isDeformable())) {
        recordError(RTC_INVALID_OPERATION);
        return NULL;
      }
      mappedCurves = true; 
      atomic_add(&parent->numMappedBuffers,1); 
      return curves;
    }
    case RTC_VERTEX_BUFFER: 
    {
      if (mappedVertices) {
        recordError(RTC_INVALID_OPERATION);
        return NULL;
      }
      mappedVertices = true; 
      atomic_add(&parent->numMappedBuffers,1); 
      return vertices;
    }
    default: 
      recordError(RTC_INVALID_ARGUMENT); 
      return NULL;
    }
  }

  void QuadraticBezierCurvesScene::QuadraticBezierCurves::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }

    switch (type) {
    case RTC_INDEX_BUFFER : 
    {
      if (mappedCurves) {
        mappedCurves = false; 
        atomic_add(&parent->numMappedBuffers,-1); 
      } else {
        recordError(RTC_INVALID_OPERATION);
      }
      break;
    }
    case RTC_VERTEX_BUFFER: 
    {
      if (mappedVertices) {
        mappedVertices = false; 
        atomic_add(&parent->numMappedBuffers,-1); 
      } else {
        recordError(RTC_INVALID_OPERATION);
      }
      break;
    }
    default: 
      recordError(RTC_INVALID_ARGUMENT); 
    }
  }

  void QuadraticBezierCurvesScene::QuadraticBezierCurves::postBuild (bool needVertices) 
  {
    built = true;
    if (parent->isStatic()) {
      alignedFree(curves); curves = NULL;
      if (!needVertices) {
        alignedFree(vertices); vertices = NULL;
      }
    }
  }
}
