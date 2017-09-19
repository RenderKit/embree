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

extern "C" RTCScene ispcNewScene2 (RTCDevice device, uniform RTCSceneFlags flags, uniform RTCAlgorithmFlags aflags);

extern "C" void ispcSetIntersectFunction1 (RTCScene scene, uniform unsigned int geomID, void* uniform intersect);
extern "C" void ispcSetIntersectFunction4 (RTCScene scene, uniform unsigned int geomID, void* uniform intersect); 
extern "C" void ispcSetIntersectFunction8 (RTCScene scene, uniform unsigned int geomID, void* uniform intersect);
extern "C" void ispcSetIntersectFunction16 (RTCScene scene, uniform unsigned int geomID, void* uniform intersect); 
extern "C" void ispcSetIntersectFunction1Mp (RTCScene scene, uniform unsigned int geomID, void* uniform intersect); 
extern "C" void ispcSetIntersectFunctionN (RTCScene scene, uniform unsigned int geomID, void* uniform intersect); 

extern "C" void ispcSetOccludedFunction1 (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);
extern "C" void ispcSetOccludedFunction4 (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);
extern "C" void ispcSetOccludedFunction8 (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);
extern "C" void ispcSetOccludedFunction16 (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);
extern "C" void ispcSetOccludedFunction1Mp (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);
extern "C" void ispcSetOccludedFunctionN (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);

extern "C" void ispcSetIntersectionFilterFunction1 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetIntersectionFilterFunction4 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetIntersectionFilterFunction8 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetIntersectionFilterFunction16 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetIntersectionFilterFunctionN (RTCScene scene, uniform unsigned int geomID, void* uniform filter);

extern "C" void ispcSetOcclusionFilterFunction1 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetOcclusionFilterFunction4 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetOcclusionFilterFunction8 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetOcclusionFilterFunction16 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetOcclusionFilterFunctionN (RTCScene scene, uniform unsigned int geomID, void* uniform filter);

extern "C" void ispcIntersect1 (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1& ray);
extern "C" void ispcIntersect4 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcIntersect8 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcIntersect16 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcIntersect1M  (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1* uniform rays, const uniform size_t M, const uniform size_t stride);
extern "C" void ispcIntersect1Mp (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1** uniform rays, const uniform size_t M);
extern "C" void ispcIntersectNM  (RTCScene scene, const uniform RTCIntersectContext* uniform context, struct RTCRayN* uniform rays, const uniform size_t M, const uniform size_t N, const uniform size_t stride);
extern "C" void ispcIntersectNp  (RTCScene scene, const uniform RTCIntersectContext* uniform context, const uniform RTCRayNp& rays, const uniform size_t N);

extern "C" void ispcOccluded1 (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1& ray);
extern "C" void ispcOccluded4 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcOccluded8 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcOccluded16 (void* uniform valid, RTCScene scene, const uniform RTCIntersectContext* uniform context, void* uniform ray);
extern "C" void ispcOccluded1M  (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1* uniform rays, const uniform size_t M, const uniform size_t stride);
extern "C" void ispcOccluded1Mp (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1** uniform rays, const uniform size_t M);
extern "C" void ispcOccludedNM (RTCScene scene, const uniform RTCIntersectContext* uniform context, struct RTCRayN* uniform rays, const uniform size_t M, const uniform size_t N, const uniform size_t stride);
extern "C" void ispcOccludedNp (RTCScene scene, const uniform RTCIntersectContext* uniform context, const uniform RTCRayNp& rays, const uniform size_t N);

extern "C" void ispcInterpolateN(RTCScene scene, uniform unsigned int geomID, 
                                 const void* uniform valid, const uniform unsigned int* uniform primIDs, const uniform float* uniform u, const uniform float* uniform v, uniform size_t numUVs, 
                                 uniform RTCBufferType buffer, 
                                 uniform float* uniform P, uniform float* uniform dPdu, uniform float* uniform dPdv, uniform size_t numFloats);

extern "C" void ispcInterpolateN2(RTCScene scene, uniform unsigned int geomID, 
                                 const void* uniform valid, const uniform unsigned int* uniform primIDs, const uniform float* uniform u, const uniform float* uniform v, uniform size_t numUVs, 
                                 uniform RTCBufferType buffer, 
                                 uniform float* uniform P, uniform float* uniform dPdu, uniform float* uniform dPdv, 
                                 uniform float* uniform ddPdudu, uniform float* uniform ddPdvdv, uniform float* uniform ddPdudv, 
                                 uniform size_t numFloats);


RTCScene rtcDeviceNewScene (RTCDevice device, uniform RTCSceneFlags flags, uniform RTCAlgorithmFlags aflags) 
{  
  if (aflags & RTC_INTERSECT_VARYING) {
    if (sizeof(varying float) == 16)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 1));
    else if (sizeof(varying float) == 32)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 2));
    else if (sizeof(varying float) == 64)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 3));
  }
  return ispcNewScene2(device,flags,aflags);
}

void rtcIntersect1 (RTCScene scene, uniform RTCRay1& ray) {
  ispcIntersect1(scene,NULL,ray);
}

void rtcIntersect1Ex (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1& ray) {
  ispcIntersect1(scene,context,ray);
}

void rtcIntersect (RTCScene scene, varying RTCRay& ray) 
{
  varying bool mask = __mask;
  unmasked {
    varying int imask = mask ? -1 : 0;
  }

  if (sizeof(varying float) == 16)
    ispcIntersect4(&imask,scene,NULL,&ray);
  else if (sizeof(varying float) == 32)
    ispcIntersect8(&imask,scene,NULL,&ray);
  else if (sizeof(varying float) == 64)
    ispcIntersect16(&imask,scene,NULL,&ray);
}

void rtcIntersectEx (RTCScene scene, const uniform RTCIntersectContext* uniform context, varying RTCRay& ray) 
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

void rtcIntersect1M (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1* uniform rays, const uniform size_t M, const uniform size_t stride) {
  ispcIntersect1M(scene,context,rays,M,stride);
}

void rtcIntersect1M (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1** uniform rays, const uniform size_t M, const uniform size_t stride) {
  ispcIntersect1Mp(scene,context,rays,M);
}

void rtcIntersectVM (RTCScene scene, const uniform RTCIntersectContext* uniform context, varying RTCRay* uniform rays, const uniform size_t M, const uniform size_t stride) {
  ispcIntersectNM(scene,context,(struct RTCRayN*)rays,sizeof(varying float)/4,M,stride);
}

void rtcIntersectNM (RTCScene scene, const uniform RTCIntersectContext* uniform context, struct RTCRayN* uniform rays, const uniform size_t N, const uniform size_t M, const uniform size_t stride) {
  ispcIntersectNM(scene,context,(struct RTCRayN*)rays,N,M,stride);
}

void rtcIntersectNp (RTCScene scene, const uniform RTCIntersectContext* uniform context, const uniform RTCRayNp& rays, const uniform size_t N) {
  ispcIntersectNp(scene,context,rays,N);
}

void rtcOccluded1 (RTCScene scene, uniform RTCRay1& ray) {
  ispcOccluded1(scene,NULL,ray);
}

void rtcOccluded1Ex (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1& ray) {
  ispcOccluded1(scene,context,ray);
}

void rtcOccluded (RTCScene scene, varying RTCRay& ray) 
{
  varying bool mask = __mask;
  unmasked {
    varying int imask = mask ? -1 : 0;
  }

  if (sizeof(varying float) == 16)
    ispcOccluded4(&imask,scene,NULL,&ray);
  else if (sizeof(varying float) == 32)
    ispcOccluded8(&imask,scene,NULL,&ray);
  else if (sizeof(varying float) == 64)
    ispcOccluded16(&imask,scene,NULL,&ray);
}

void rtcOccludedEx (RTCScene scene, const uniform RTCIntersectContext* uniform context, varying RTCRay& ray) 
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

void rtcOccluded1M (RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1* uniform rays, const uniform size_t M, const uniform size_t stride) {
  ispcOccluded1M(scene,context,rays,M,stride);
}

void rtcOccluded1Mp(RTCScene scene, const uniform RTCIntersectContext* uniform context, uniform RTCRay1** uniform rays, const uniform size_t M, const uniform size_t stride) {
  ispcOccluded1Mp(scene,context,rays,M);
}

void rtcOccludedVM (RTCScene scene, const uniform RTCIntersectContext* uniform context, varying RTCRay* uniform rays, const uniform size_t M, const uniform size_t stride) {
  ispcOccludedNM(scene,context,(struct RTCRayN*)rays,sizeof(varying float)/4,M,stride);
}

void rtcOccludedNM (RTCScene scene, const uniform RTCIntersectContext* uniform context, struct RTCRayN* uniform rays, const uniform size_t N, const uniform size_t M, const uniform size_t stride) {
  ispcOccludedNM(scene,context,(struct RTCRayN*)rays,N,M,stride);
}

void rtcOccludedNp (RTCScene scene, const uniform RTCIntersectContext* uniform context, const uniform RTCRayNp& rays, const uniform size_t N) {
  ispcOccludedNp(scene,context,rays,N);
}

void rtcSetIntersectFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCIntersectFuncVarying intersect)
{
  if (sizeof(varying float) == 16) 
    ispcSetIntersectFunction4(scene,geomID,intersect);
  else if (sizeof(varying float) == 32) 
    ispcSetIntersectFunction8(scene,geomID,intersect);
  else if (sizeof(varying float) == 64) 
    ispcSetIntersectFunction16(scene,geomID,intersect);
}

void rtcSetOccludedFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCOccludedFuncVarying occluded)
{
  if (sizeof(varying float) == 16)
    ispcSetOccludedFunction4(scene,geomID,occluded);
  else if (sizeof(varying float) == 32)
    ispcSetOccludedFunction8(scene,geomID,occluded);
  else if (sizeof(varying float) == 64)
    ispcSetOccludedFunction16(scene,geomID,occluded);
}

void rtcSetIntersectionFilterFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCFilterFuncVarying filter)
{
  if (sizeof(varying float) == 16)
    ispcSetIntersectionFilterFunction4(scene,geomID,filter);
  else if (sizeof(varying float) == 32)
    ispcSetIntersectionFilterFunction8(scene,geomID,filter);
  else if (sizeof(varying float) == 64)
    ispcSetIntersectionFilterFunction16(scene,geomID,filter);
}

void rtcSetOcclusionFilterFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCFilterFuncVarying filter)
{
  if (sizeof(varying float) == 16)
    ispcSetOcclusionFilterFunction4(scene,geomID,filter);
  else if (sizeof(varying float) == 32)
    ispcSetOcclusionFilterFunction8(scene,geomID,filter);
  else if (sizeof(varying float) == 64)
    ispcSetOcclusionFilterFunction16(scene,geomID,filter);
}

void rtcInterpolate(RTCScene scene, uniform unsigned int geomID, varying unsigned int primID, varying float u, varying float v, 
                    uniform RTCBufferType buffer,
                    varying float* uniform P, varying float* uniform dPdu, varying float* uniform dPdv, uniform size_t numFloats)
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
                    uniform size_t numFloats)
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
