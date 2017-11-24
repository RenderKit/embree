// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
  Geometry::Geometry (Device* device, Type type, unsigned int numPrimitives, unsigned int numTimeSteps) 
    : device(device), scene(nullptr), geomID(0), type(type), 
      numPrimitives(numPrimitives), numPrimitivesChanged(false),
      numTimeSteps(unsigned(numTimeSteps)), fnumTimeSegments(float(numTimeSteps-1)), quality(RTC_BUILD_QUALITY_MEDIUM),
      enabled(true), state(MODIFIED), userPtr(nullptr), mask(-1), used(1),
      intersectionFilterN(nullptr), occlusionFilterN(nullptr)
  {
    device->refInc();
  }

  Geometry::~Geometry()
  {
    device->refDec();
  }

  void Geometry::update() 
  {
    if (scene)
      scene->setModified();

    state = MODIFIED;
  }
  
  void Geometry::commit() 
  {
    if (scene)
      scene->setModified();

    state = COMMITTED;
  }

  void Geometry::preCommit()
  {
    if (state == MODIFIED)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"geometry got not committed");
  }

  void Geometry::postCommit()
  {
    /* set state to build */
    if (isEnabled())
      state = BUILD;
  }

  void Geometry::updateIntersectionFilters(bool enable)
  {
    const size_t numN  = (intersectionFilterN  != nullptr) + (occlusionFilterN  != nullptr);

    if (enable) {
      scene->numIntersectionFiltersN += numN;
    } else {
      scene->numIntersectionFiltersN -= numN;
    }
  }

  Geometry* Geometry::attach(Scene* scene, unsigned int geomID)
  {
    assert(scene);
    this->scene = scene;
    this->geomID = geomID;
    if (isEnabled()) {
      scene->setModified();
      updateIntersectionFilters(true);
      enabling();
    }
    return this;
  }

  void Geometry::detach()
  {
    if (isEnabled()) {
      scene->setModified();
      updateIntersectionFilters(false);
      disabling();
    }
    this->scene = nullptr;
    this->geomID = -1;
  }
  
  void Geometry::enable () 
  {
    if (isEnabled()) 
      return;

    if (scene) {
      updateIntersectionFilters(true);
      scene->setModified();
      enabling();
    }

    used++;
    enabled = true;
  }

  void Geometry::disable () 
  {
    if (isDisabled()) 
      return;

    if (scene) {
      updateIntersectionFilters(false);
      scene->setModified();
      disabling();
    }
    
    used--;
    enabled = false;
  }

  void Geometry::setUserData (void* ptr)
  {
    userPtr = ptr;
  }
  
  void Geometry::setIntersectionFilterFunctionN (RTCFilterFunctionN filter) 
  { 
    if (type != TRIANGLE_MESH && type != QUAD_MESH && type != LINE_SEGMENTS && type != BEZIER_CURVES && type != SUBDIV_MESH && type != USER_GEOMETRY)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    if (scene && isEnabled()) {
      scene->numIntersectionFiltersN -= intersectionFilterN != nullptr;
      scene->numIntersectionFiltersN += filter != nullptr;
    }
    intersectionFilterN = filter;
  }

  void Geometry::setOcclusionFilterFunctionN (RTCFilterFunctionN filter) 
  { 
    if (type != TRIANGLE_MESH && type != QUAD_MESH && type != LINE_SEGMENTS && type != BEZIER_CURVES && type != SUBDIV_MESH && type != USER_GEOMETRY) 
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    if (scene && isEnabled()) {
      scene->numIntersectionFiltersN -= occlusionFilterN != nullptr;
      scene->numIntersectionFiltersN += filter != nullptr;
    }
    occlusionFilterN = filter;
  }

  void Geometry::interpolateN(const void* valid_i, const unsigned* primIDs, const float* u, const float* v, unsigned int numUVs, 
                              RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, unsigned int numFloats)
  {
    if (numFloats > 256) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"maximally 256 floating point values can be interpolated per vertex");
    const int* valid = (const int*) valid_i;

    __aligned(64) float P_tmp[256];
    __aligned(64) float dPdu_tmp[256];
    __aligned(64) float dPdv_tmp[256];
    __aligned(64) float ddPdudu_tmp[256];
    __aligned(64) float ddPdvdv_tmp[256];
    __aligned(64) float ddPdudv_tmp[256];

    float* Pt = P ? P_tmp : nullptr;
    float* dPdut = nullptr, *dPdvt = nullptr;
    if (dPdu) { dPdut = dPdu_tmp; dPdvt = dPdv_tmp; }
    float* ddPdudut = nullptr, *ddPdvdvt = nullptr, *ddPdudvt = nullptr;
    if (ddPdudu) { ddPdudut = ddPdudu_tmp; ddPdvdvt = ddPdvdv_tmp; ddPdudvt = ddPdudv_tmp; }
    
    for (unsigned int i=0; i<numUVs; i++)
    {
      if (valid && !valid[i]) continue;
      interpolate(primIDs[i],u[i],v[i],buffer,Pt,dPdut,dPdvt,ddPdudut,ddPdvdvt,ddPdudvt,numFloats);
      
      if (likely(P)) {
        for (unsigned int j=0; j<numFloats; j++) 
          P[j*numUVs+i] = Pt[j];
      }
      if (likely(dPdu)) 
      {
        for (unsigned int j=0; j<numFloats; j++) {
          dPdu[j*numUVs+i] = dPdut[j];
          dPdv[j*numUVs+i] = dPdvt[j];
        }
      }
      if (likely(ddPdudu)) 
      {
        for (unsigned int j=0; j<numFloats; j++) {
          ddPdudu[j*numUVs+i] = ddPdudut[j];
          ddPdvdv[j*numUVs+i] = ddPdvdvt[j];
          ddPdudv[j*numUVs+i] = ddPdudvt[j];
        }
      }
    }
  }
}
