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

#include "geometry.h"
#include "scene.h"

namespace embree
{
  Geometry::Geometry (Scene* parent, GeometryTy type, size_t numPrimitives, RTCGeometryFlags flags) 
    : parent(parent), type(type), numPrimitives(numPrimitives), id(0), flags(flags), state(ENABLING),
      filter1(NULL), filter4(NULL), filter8(NULL), filter16(NULL),
      ispcFilter1(NULL), ispcFilter4(NULL), ispcFilter8(NULL), ispcFilter16(NULL)
  {
    id = parent->add(this);
  }

  void Geometry::enable () 
  {
    switch (state) {
    case ENABLING:
      break;
    case ENABLED:
      break;
    case MODIFIED:
      break;
    case DISABLING: 
      state = MODIFIED;
      enabling();
      break;
    case DISABLED: 
      state = ENABLING;
      enabling();
      break;
    case ERASING:
      break;
    }
  }

  void Geometry::update() 
  {
    switch (state) {
    case ENABLING:
      break;
    case ENABLED:
      state = MODIFIED;
      break;
    case MODIFIED:
      break;
    case DISABLING: 
      break;
    case DISABLED: 
      break;
    case ERASING:
      break;
    }
  }

  void Geometry::disable () 
  {
    switch (state) {
    case ENABLING:
      state = DISABLED;
      disabling();
      break;
    case ENABLED:
      state = DISABLING;
      disabling();
      break;
    case MODIFIED:
      state = DISABLING;
      disabling();
      break;
    case DISABLING: 
      break;
    case DISABLED: 
      break;
    case ERASING:
      break;
    }
  }

  void Geometry::erase () 
  {
    switch (state) {
    case ENABLING:
      state = ERASING;
      disabling();
      break;
    case ENABLED:
      state = ERASING;
      disabling();
      break;
    case MODIFIED:
      state = ERASING;
      disabling();
      break;
    case DISABLING: 
      state = ERASING;
      break;
    case DISABLED: 
      state = ERASING;
      break;
    case ERASING:
      break;
    }
  }

  void Geometry::setFilterFunction (RTCFilterFunc filter, bool ispc) 
  {
    if (type != TRIANGLE_MESH) {
      recordError(RTC_INVALID_OPERATION); 
      return;
    }
    filter1 = filter;
  }
    
  void Geometry::setFilterFunction4 (RTCFilterFunc4 filter4, bool ispc) 
  { 
    if (type != TRIANGLE_MESH) {
      recordError(RTC_INVALID_OPERATION); 
      return;
    }
    if (ispc) {
      ispcFilter4 = (void*) filter4;
    } else {
      filter4 = filter4;
    }
  }
    
  void Geometry::setFilterFunction8 (RTCFilterFunc8 filter8, bool ispc) 
  { 
    if (type != TRIANGLE_MESH) {
      recordError(RTC_INVALID_OPERATION); 
      return;
    }
    if (ispc) {
      ispcFilter8 = (void*) filter8;
    } else {
      filter8 = filter8;
    }
  }
  
  void Geometry::setFilterFunction16 (RTCFilterFunc16 filter16, bool ispc) 
  { 
    if (type != TRIANGLE_MESH) {
      recordError(RTC_INVALID_OPERATION); 
      return;
    }
    if (ispc) {
      ispcFilter16 = (void*) filter16;
    } else {
      filter16 = filter16;
    }
  }
}
