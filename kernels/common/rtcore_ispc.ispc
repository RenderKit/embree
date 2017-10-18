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

#include "../../include/embree2/rtcore.isph" 
#include "../../include/embree2/rtcore_ray.isph" 

extern "C" void ispcIntersect1 (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1& ray);
extern "C" void ispcIntersect4 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcIntersect8 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcIntersect16 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);

extern "C" void ispcOccluded1 (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1& ray);
extern "C" void ispcOccluded4 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcOccluded8 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcOccluded16 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);

extern "C" void ispcInterpolateN(RTCScene scene, uniform unsigned int geomID, 
                                 const void* uniform valid, const uniform unsigned int* uniform primIDs, const uniform float* uniform u, const uniform float* uniform v, uniform unsigned int numUVs, 
                                 uniform RTCBufferType buffer, 
                                 uniform float* uniform P, uniform float* uniform dPdu, uniform float* uniform dPdv, uniform unsigned int numFloats);

extern "C" void ispcInterpolateN2(RTCScene scene, uniform unsigned int geomID, 
                                 const void* uniform valid, const uniform unsigned int* uniform primIDs, const uniform float* uniform u, const uniform float* uniform v, uniform unsigned int numUVs, 
                                 uniform RTCBufferType buffer, 
                                 uniform float* uniform P, uniform float* uniform dPdu, uniform float* uniform dPdv, 
                                 uniform float* uniform ddPdudu, uniform float* uniform ddPdvdv, uniform float* uniform ddPdudv, 
                                 uniform unsigned int numFloats);


void rtcIntersect1 (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1& ray) {
  ispcIntersect1(scene,context,ray);
}

void rtcIntersect (RTCScene scene, const uniform RTCIntersectContext* uniform context, varying RTCRay& ray) 
{
  varying bool mask = __mask & (ray.tnear <= ray.tfar);
  unmasked {
    varying int imask = mask ? -1 : 0;
  }

  if (sizeof(varying float) == 16)
    ispcIntersect4(&imask,scene,context,&ray);
  else if (sizeof(varying float) == 32)
    ispcIntersect8(&imask,scene,context,&ray);
  else if (sizeof(varying float) == 64)
    ispcIntersect16(&imask,scene,context,&ray);
}

void rtcIntersectVM (RTCScene scene, const uniform RTCIntersectContext* uniform context, varying RTCRay* uniform rays, const uniform unsigned int M, const uniform size_t stride) {
  rtcIntersectNM(scene,context,(struct RTCRayN*)rays,sizeof(varying float)/4,M,stride);
}

void rtcOccluded1 (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1& ray) {
  ispcOccluded1(scene,context,ray);
}

void rtcOccluded (RTCScene scene, const uniform RTCIntersectContext* uniform context, varying RTCRay& ray) 
{
  varying bool mask = __mask & (ray.tnear <= ray.tfar);
  unmasked {
    varying int imask = mask ? -1 : 0;
  }

  if (sizeof(varying float) == 16)
    ispcOccluded4(&imask,scene,context,&ray);
  else if (sizeof(varying float) == 32)
    ispcOccluded8(&imask,scene,context,&ray);
  else if (sizeof(varying float) == 64)
    ispcOccluded16(&imask,scene,context,&ray);
}

void rtcOccludedVM (RTCScene scene, const uniform RTCIntersectContext* uniform context, varying RTCRay* uniform rays, const uniform unsigned int M, const uniform size_t stride) {
  rtcOccludedNM(scene,context,(struct RTCRayN*)rays,sizeof(varying float)/4,M,stride);
}

void rtcInterpolate(RTCScene scene, uniform unsigned int geomID, varying unsigned int primID, varying float u, varying float v, 
                    uniform RTCBufferType buffer,
                    varying float* uniform P, varying float* uniform dPdu, varying float* uniform dPdv, uniform unsigned int numFloats)
{
  varying bool mask = __mask;
  unmasked {
    varying int imask = mask ? -1 : 0;
  }
  ispcInterpolateN(scene,geomID,(const void* uniform)&imask,(const uniform unsigned int* uniform)&primID,
                   (const uniform float* uniform)&u,(const uniform float* uniform)&v,sizeof(varying float)/4,buffer,
                   (uniform float* uniform)P,(uniform float* uniform)dPdu,(uniform float* uniform)dPdv,numFloats);
}

void rtcInterpolate2(RTCScene scene, uniform unsigned int geomID, varying unsigned int primID, varying float u, varying float v, 
                    uniform RTCBufferType buffer,
                    varying float* uniform P, varying float* uniform dPdu, varying float* uniform dPdv, 
                    varying float* uniform ddPdudu, varying float* uniform ddPdvdv, varying float* uniform ddPdudv,
                    uniform unsigned int numFloats)
{
  varying bool mask = __mask;
  unmasked {
    varying int imask = mask ? -1 : 0;
  }
  ispcInterpolateN2(scene,geomID,(const void* uniform)&imask,(const uniform unsigned int* uniform)&primID,
                    (const uniform float* uniform)&u,(const uniform float* uniform)&v,sizeof(varying float)/4,buffer,
                    (uniform float* uniform)P,(uniform float* uniform)dPdu,(uniform float* uniform)dPdv,
                    (uniform float* uniform)ddPdudu,(uniform float* uniform)ddPdvdv,(uniform float* uniform)ddPdudv,
                    numFloats);
}

export void dummy_rtcore_ispc() {}  // just to avoid compile warning "libembree.a(rtcore_ispc.dev.o) has no symbols" under MacOSX
