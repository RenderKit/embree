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

#include "default.h"
#include "scene.h"

DISABLE_DEPRECATED_WARNING;

namespace embree
{
  extern "C" void ispcIntersect1 (RTCScene scene, const RTCIntersectContext* context, RTCRay& ray) {
    rtcIntersect1(scene,context,ray);
  }
  
  extern "C" void ispcIntersect4 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay4& ray) {
    rtcIntersect4(valid,scene,context,ray);
  }
  
  extern "C" void ispcIntersect8 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay8& ray) {
    rtcIntersect8(valid,scene,context,ray);
  }
  
  extern "C" void ispcIntersect16 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay16& ray) {
    rtcIntersect16(valid,scene,context,ray);
  }

  extern "C" void ispcOccluded1 (RTCScene scene, const RTCIntersectContext* context, RTCRay& ray) {
    rtcOccluded1(scene,context,ray);
  }
  
  extern "C" void ispcOccluded4 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay4& ray) {
    rtcOccluded4(valid,scene,context,ray);
  }
  
  extern "C" void ispcOccluded8 (const void* valid, RTCScene scene, const RTCIntersectContext* context,  RTCRay8& ray) {
    rtcOccluded8(valid,scene,context,ray);
  }
  
  extern "C" void ispcOccluded16 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay16& ray) {
    rtcOccluded16(valid,scene,context,ray);
  }

  extern "C" void ispcInterpolateN(RTCScene scene, unsigned int geomID, 
                                   const void* valid, const unsigned int* primIDs, const float* u, const float* v, unsigned int numUVs, 
                                   RTCBufferType buffer, 
                                   float* P, float* dPdu, float* dPdv, unsigned int numFloats)
  {
    rtcInterpolateN(scene,geomID,valid,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,numFloats);
  }

  extern "C" void ispcInterpolateN2(RTCScene scene, unsigned int geomID, 
                                    const void* valid, const unsigned int* primIDs, const float* u, const float* v, unsigned int numUVs, 
                                    RTCBufferType buffer, 
                                    float* P, float* dPdu, float* dPdv,
                                    float* ddPdudu, float* ddPdvdv, float* ddPdudv,
                                    unsigned int numFloats)
  {
    rtcInterpolateN2(scene,geomID,valid,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats);
  }
}
