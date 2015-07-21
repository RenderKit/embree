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

#include "geometry.h"
#include "scene.h"

namespace embree
{
  Geometry::Geometry (Scene* parent, Type type, size_t numPrimitives, size_t numTimeSteps, RTCGeometryFlags flags) 
    : parent(parent), type(type), numPrimitives(numPrimitives), numTimeSteps(numTimeSteps), id(0), flags(flags),
      enabled(true), modified(true), erasing(false), mask(-1),
      intersectionFilter1(nullptr), occlusionFilter1(nullptr),
      intersectionFilter4(nullptr), occlusionFilter4(nullptr), ispcIntersectionFilter4(false), ispcOcclusionFilter4(false), 
      intersectionFilter8(nullptr), occlusionFilter8(nullptr), ispcIntersectionFilter8(false), ispcOcclusionFilter8(false), 
      intersectionFilter16(nullptr), occlusionFilter16(nullptr), ispcIntersectionFilter16(false), ispcOcclusionFilter16(false), 
      userPtr(nullptr)
  {
    id = parent->add(this);
    parent->setModified();
  }

  Geometry::~Geometry() {
  }

  void Geometry::write(std::ofstream& file) {
    int type = -1; file.write((char*)&type,sizeof(type));
  }

  void Geometry::updateIntersectionFilters(bool enable)
  {
    if (enable) {
      atomic_add(&parent->numIntersectionFilters4,(intersectionFilter4 != nullptr) + (occlusionFilter4 != nullptr));
      atomic_add(&parent->numIntersectionFilters8,(intersectionFilter8 != nullptr) + (occlusionFilter8 != nullptr));
      atomic_add(&parent->numIntersectionFilters16,(intersectionFilter16 != nullptr) + (occlusionFilter16 != nullptr));
    } else {
      atomic_sub(&parent->numIntersectionFilters4,(intersectionFilter4 != nullptr) + (occlusionFilter4 != nullptr));
      atomic_sub(&parent->numIntersectionFilters8,(intersectionFilter8 != nullptr) + (occlusionFilter8 != nullptr));
      atomic_sub(&parent->numIntersectionFilters16,(intersectionFilter16 != nullptr) + (occlusionFilter16 != nullptr));
    }
  }

  void Geometry::enable () 
  {
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (isEnabled() || isErasing()) 
      return;

    updateIntersectionFilters(true);
    parent->setModified();
    enabled = true;
    enabling();
  }

  void Geometry::update() 
  {
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    parent->setModified();
    modified = true;
  }

  void Geometry::disable () 
  {
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (isDisabled() || isErasing()) 
      return;

    updateIntersectionFilters(false);
    parent->setModified();
    enabled = false;
    disabling();
  }

  void Geometry::erase () 
  {
    if (parent->isStatic())
      throw_RTCError(RTC_INVALID_OPERATION,"rtcDeleteGeometry cannot get called in static scenes");

    if (isErasing())
      return;

    parent->setModified();
    erasing = true;

    if (isDisabled())
      return;
    
    updateIntersectionFilters(false);
    enabled = false;
    disabling();
  }
  
  void Geometry::setUserData (void* ptr)
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    userPtr = ptr;
  }
  
  void Geometry::setIntersectionFilterFunction (RTCFilterFunc filter, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (type != TRIANGLE_MESH && type != BEZIER_CURVES && type != SUBDIV_MESH)
      throw_RTCError(RTC_INVALID_OPERATION,"filter functions not supported for this geometry"); 
    
    intersectionFilter1 = filter;
  }
    
  void Geometry::setIntersectionFilterFunction4 (RTCFilterFunc4 filter, bool ispc) 
  { 
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (type != TRIANGLE_MESH && type != BEZIER_CURVES && type != SUBDIV_MESH)
      throw_RTCError(RTC_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    atomic_sub(&parent->numIntersectionFilters4,intersectionFilter4 != nullptr);
    atomic_add(&parent->numIntersectionFilters4,filter != nullptr);
    intersectionFilter4 = filter;
    ispcIntersectionFilter4 = ispc;
  }
    
  void Geometry::setIntersectionFilterFunction8 (RTCFilterFunc8 filter, bool ispc) 
  { 
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");
    
    if (type != TRIANGLE_MESH && type != BEZIER_CURVES && type != SUBDIV_MESH)
      throw_RTCError(RTC_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    atomic_sub(&parent->numIntersectionFilters8,intersectionFilter8 != nullptr);
    atomic_add(&parent->numIntersectionFilters8,filter != nullptr);
    intersectionFilter8 = filter;
    ispcIntersectionFilter8 = ispc;
  }
  
  void Geometry::setIntersectionFilterFunction16 (RTCFilterFunc16 filter, bool ispc) 
  { 
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (type != TRIANGLE_MESH && type != BEZIER_CURVES && type != SUBDIV_MESH)
      throw_RTCError(RTC_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    atomic_sub(&parent->numIntersectionFilters16,intersectionFilter16 != nullptr);
    atomic_add(&parent->numIntersectionFilters16,filter != nullptr);
    intersectionFilter16 = filter;
    ispcIntersectionFilter16 = ispc;
  }

  void Geometry::setOcclusionFilterFunction (RTCFilterFunc filter, bool ispc) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (type != TRIANGLE_MESH && type != BEZIER_CURVES && type != SUBDIV_MESH)
      throw_RTCError(RTC_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    occlusionFilter1 = filter;
  }
    
  void Geometry::setOcclusionFilterFunction4 (RTCFilterFunc4 filter, bool ispc) 
  { 
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (type != TRIANGLE_MESH && type != BEZIER_CURVES && type != SUBDIV_MESH)
      throw_RTCError(RTC_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    atomic_sub(&parent->numIntersectionFilters4,occlusionFilter4 != nullptr);
    atomic_add(&parent->numIntersectionFilters4,filter != nullptr);
    occlusionFilter4 = filter;
    ispcOcclusionFilter4 = ispc;
  }
    
  void Geometry::setOcclusionFilterFunction8 (RTCFilterFunc8 filter, bool ispc) 
  { 
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (type != TRIANGLE_MESH && type != BEZIER_CURVES && type != SUBDIV_MESH)
      throw_RTCError(RTC_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    atomic_sub(&parent->numIntersectionFilters8,occlusionFilter8 != nullptr);
    atomic_add(&parent->numIntersectionFilters8,filter != nullptr);
    occlusionFilter8 = filter;
    ispcOcclusionFilter8 = ispc;
  }
  
  void Geometry::setOcclusionFilterFunction16 (RTCFilterFunc16 filter, bool ispc) 
  { 
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    if (type != TRIANGLE_MESH && type != BEZIER_CURVES && type != SUBDIV_MESH) 
      throw_RTCError(RTC_INVALID_OPERATION,"filter functions not supported for this geometry"); 

    atomic_sub(&parent->numIntersectionFilters16,occlusionFilter16 != nullptr);
    atomic_add(&parent->numIntersectionFilters16,filter != nullptr);
    occlusionFilter16 = filter;
    ispcOcclusionFilter16 = ispc;
  }

  void Geometry::interpolateN(const void* valid_i, const unsigned* primIDs, const float* u, const float* v, size_t numUVs, 
                              RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats)
  {
    if (numFloats > 256) throw_RTCError(RTC_INVALID_OPERATION,"maximally 256 floating point values can be interpolated per vertex");
    const int* valid = (const int*) valid_i;

    __aligned(64) float P_tmp[256];
    __aligned(64) float dPdu_tmp[256];
    __aligned(64) float dPdv_tmp[256];
    float* Pt = P ? P_tmp : nullptr;
    float* dPdut = dPdu ? dPdu_tmp : nullptr;
    float* dPdvt = dPdv ? dPdv_tmp : nullptr;

    for (size_t i=0; i<numUVs; i++)
    {
      if (valid && !valid[i]) continue;
      interpolate(primIDs[i],u[i],v[i],buffer,Pt,dPdut,dPdvt,numFloats);
      if (P   ) for (size_t j=0; j<numFloats; j++) P[j*numUVs+i] = Pt[j];
      if (dPdu) for (size_t j=0; j<numFloats; j++) dPdu[j*numUVs+i] = dPdut[j];
      if (dPdv) for (size_t j=0; j<numFloats; j++) dPdv[j*numUVs+i] = dPdvt[j];
    }
  }
}
