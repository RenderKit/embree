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

#pragma once

#include "rtcore_device.h"

#if defined(__cplusplus)
extern "C" {
#endif
  
/* Forward declarations for ray structures */
struct RTCRayHit;
struct RTCRayHit4;
struct RTCRayHit8;
struct RTCRayHit16;
struct RTCRayHitNp;

/* Scene flags */
enum RTCSceneFlags
{
  RTC_SCENE_FLAG_NONE                    = 0,
  RTC_SCENE_FLAG_DYNAMIC                 = (1 << 0),
  RTC_SCENE_FLAG_COMPACT                 = (1 << 1),
  RTC_SCENE_FLAG_ROBUST                  = (1 << 2),
  RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION = (1 << 3)
};

/* Creates a new scene. */
RTC_API RTCScene rtcNewScene(RTCDevice device);

/* Retains the scene (increments reference count). */
RTC_API void rtcRetainScene(RTCScene scene);

/* Releases the scene (decrements reference count). */
RTC_API void rtcReleaseScene(RTCScene scene);

  
/* Attaches the geometry to some scene. */
RTC_API unsigned int rtcAttachGeometry(RTCScene scene, RTCGeometry geometry);

/* Attaches the geometry to some scene using the specified geometry ID. */
RTC_API void rtcAttachGeometryByID(RTCScene scene, RTCGeometry geometry, unsigned int geomID);

/* Detaches the geometry from the scene. */
RTC_API void rtcDetachGeometry(RTCScene scene, unsigned int geomID);

/* Gets geometry handle from scene. */
RTC_API RTCGeometry rtcGetGeometry(RTCScene scene, unsigned int geomID);


/* Commits a scene. */
RTC_API void rtcCommitScene(RTCScene scene);

/* Commits a scene from multiple threads */
RTC_API void rtcJoinCommitScene(RTCScene scene);

  
/* Type of progress monitor callback function. */
typedef bool (*RTCProgressMonitorFunction)(void* ptr, const double n);

/* Sets the progress monitor callback function. */
RTC_API void rtcSetSceneProgressMonitorFunction(RTCScene scene, RTCProgressMonitorFunction progress, void* ptr);

/* Sets the build quality of a scene. */
RTC_API void rtcSetSceneBuildQuality(RTCScene scene, enum RTCBuildQuality quality);

/* Sets the scene flags. */
RTC_API void rtcSetSceneFlags(RTCScene scene, enum RTCSceneFlags flags);

/* Returns the scene flags. */
RTC_API enum RTCSceneFlags rtcGetSceneFlags(RTCScene scene);
  
/* Returns AABB of the scene. */
RTC_API void rtcGetSceneBounds(RTCScene scene, struct RTCBounds* bounds_o);

/* Returns linear AABBs of the scene. */
RTC_API void rtcGetSceneLinearBounds(RTCScene scene, struct RTCLinearBounds* bounds_o);

/* Intersects a single ray with the scene. */
RTC_API void rtcIntersect1(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit* rayhit);

/* Intersects a packet of 4 rays with the scene. */
RTC_API void rtcIntersect4(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit4* rayhit);

/* Intersects a packet of 8 rays with the scene. */
RTC_API void rtcIntersect8(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit8* rayhit);

/* Intersects a packet of 16 rays with the scene. */
RTC_API void rtcIntersect16(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit16* rayhit);

/* Intersects a stream of M rays with the scene. */
RTC_API void rtcIntersect1M(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit* rayhit, const unsigned int M, const size_t byteStride);

/* Intersects a stream of pointers to M rays with the scene. */
RTC_API void rtcIntersect1Mp(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit** rayhit, const unsigned int M);

/* Intersects a stream of M ray packets of size N in SOA format with the scene. */
RTC_API void rtcIntersectNM(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHitN* rayhit, const unsigned int N, const unsigned int M, const size_t byteStride);

/* Intersects a stream of M ray packets of size N in SOA format with the scene. */
RTC_API void rtcIntersectNp(RTCScene scene, struct RTCIntersectContext* context, const struct RTCRayHitNp* rayhit, const unsigned int N);

/* Tests a single ray for occlusion with the scene. */
RTC_API void rtcOccluded1(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit* ray);

/* Tests a packet of 4 rays for occlusion occluded with the scene. */
RTC_API void rtcOccluded4(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit4* ray);

/* Tests a packet of 8 rays for occlusion with the scene. */
RTC_API void rtcOccluded8(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit8* ray);

/* Tests a packet of 16 rays for occlusion with the scene. */
RTC_API void rtcOccluded16(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit16* ray);

/* Tests a stream of M rays for occlusion with the scene. */
RTC_API void rtcOccluded1M(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit* ray, const unsigned int M, const size_t byteStride);

/* Tests a stream of pointers to M rays for occlusion with the scene. */
RTC_API void rtcOccluded1Mp(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHit** ray, const unsigned int M);

/* Tests a stream of M ray packets of size N in SOA format for occlusion with the scene. */
RTC_API void rtcOccludedNM(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayHitN* ray, const unsigned int N, const unsigned int M, const size_t byteStride);

/* Tests a stream of M ray packets of size N in SOA format for occlusion with the scene. */
RTC_API void rtcOccludedNp(RTCScene scene, struct RTCIntersectContext* context, const struct RTCRayHitNp* ray, const unsigned int N);

#if defined(__cplusplus)

/* Helper to easily combing scene flags */
inline RTCSceneFlags operator|(const RTCSceneFlags a, const RTCSceneFlags b) {
  return (RTCSceneFlags)((size_t)a | (size_t)b);
}
  
}
#endif
