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
struct RTCRay;
struct RTCRay4;
struct RTCRay8;
struct RTCRay16;
struct RTCRayNp;

/* Build quality levels */
enum RTCBuildQuality
{
  RTC_BUILD_QUALITY_LOW    = 0, // create lower quality data structures (for dynamic scenes)
  RTC_BUILD_QUALITY_MEDIUM = 1, // default build quality for most usages
  RTC_BUILD_QUALITY_HIGH   = 2, // create higher quality data structures (longer build times)
  RTC_BUILD_QUALITY_REFIT  = 3, // refits the BVH
};

/* Scene flags */
enum RTCSceneFlags
{
  RTC_SCENE_FLAG_NONE                    = 0,
  RTC_SCENE_FLAG_DYNAMIC                 = (1 << 0), // provides better build performance for dynamic scenes
  RTC_SCENE_FLAG_COMPACT                 = (1 << 1),   // use memory conservative acceleration structure
  RTC_SCENE_FLAG_ROBUST                  = (1 << 2),   // use acceleration structure that allows robust traversal
  RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION = (1 << 3)  // enables support for intersection filter function inside context
};

/* Intersection context flags */
enum RTCIntersectContextFlags
{
  RTC_INTERSECT_CONTEXT_FLAG_NONE       = 0,
  RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT = (0 << 0),  // optimize for incoherent rays
  RTC_INTERSECT_CONTEXT_FLAG_COHERENT   = (1 << 0)   // optimize for coherent rays
};

/* Arguments for RTCFilterFunctionN callback */
struct RTCFilterFunctionNArguments
{
  int* valid;                                // pointer to valid mask
  void* geomUserPtr;                         // pointer to geometry user data
  const struct RTCIntersectContext* context; // intersection context as passed to rtcIntersect/rtcOccluded
  struct RTCRayN* ray;                       // ray and previous hit
  struct RTCHitN* potentialHit;              // potential new hit
  unsigned int N;                            // size of ray packet
};
  
/* Intersection filter function for ray packets of size N */
typedef void (*RTCFilterFunctionN)(const struct RTCFilterFunctionNArguments* const args);
  
/* Intersection context passed to intersect/occluded calls */
struct RTCIntersectContext
{
  enum RTCIntersectContextFlags flags; // intersection flags
  RTCFilterFunctionN filter;           // filter function to execute
  unsigned int instID;                 // will be set to geomID of instance when instance is entered
};

RTCORE_FORCEINLINE void rtcInitIntersectContext(struct RTCIntersectContext* context)
{
  context->flags = RTC_INTERSECT_CONTEXT_FLAG_INCOHERENT;
  context->filter = NULL;
  context->instID = -1;
}

/* Defines an opaque scene type. */
typedef struct __RTCScene* RTCScene;

/* Creates a new scene. */
RTCORE_API RTCScene rtcNewScene(RTCDevice device);

/* Type of progress callback function. */
typedef bool (*RTCProgressMonitorFunction)(void* ptr, const double n);

/* Sets the progress callback function which is called during hierarchy build of this scene. */
RTCORE_API void rtcSetSceneProgressMonitorFunction(RTCScene scene, RTCProgressMonitorFunction func, void* ptr);

/* Sets the build quality of a scene. */
RTCORE_API void rtcSetSceneBuildQuality(RTCScene scene, enum RTCBuildQuality quality);

/* Sets the scene flags. */
RTCORE_API void rtcSetSceneFlags(RTCScene scene, enum RTCSceneFlags sflags);

/* Returns the scene flags. */
RTCORE_API enum RTCSceneFlags rtcGetSceneFlags(RTCScene scene);
  
/* Commits the geometry of the scene. After initializing or modifying
 *  geometries, commit has to get called before tracing
 *  rays. */
RTCORE_API void rtcCommitScene(RTCScene scene);

/* Commits the geometry of the scene in join mode. When Embree is
 *  using TBB (default), threads that call `rtcJoinCommitScene` will
 *  participate in the hierarchy build procedure. When Embree is using
 *  the internal tasking system, exclusively threads that call
 *  `rtcJoinCommitScene` will execute the build procedure. Do not
 *  mix `rtcJoinCommitScene` with other commit calls. */
RTCORE_API void rtcJoinCommitScene(RTCScene scene);

/* Returns AABB of the scene. rtcCommitScene has to get called
 *  previously to this function. */
RTCORE_API void rtcGetSceneBounds(RTCScene scene, struct RTCBounds* bounds_o);

/* Returns linear AABBs of the scene. The result bounds_o gets filled
 *  with AABBs for time 0 and time 1. rtcCommitScene has to get called
 *  previously to this function. */
RTCORE_API void rtcGetSceneLinearBounds(RTCScene scene, struct RTCLinearBounds* bounds_o);

/* Intersects a single ray with the scene. The ray has to be aligned
 *  to 16 bytes. This function can only be called for scenes with the
 *  RTC_INTERSECT1 flag set. */
RTCORE_API void rtcIntersect1(RTCScene scene, struct RTCIntersectContext* context, struct RTCRay* ray);

/* Intersects a packet of 4 rays with the scene. The valid mask and
 *  ray have both to be aligned to 16 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT4 flag set. */
RTCORE_API void rtcIntersect4(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRay4* ray);

/* Intersects a packet of 8 rays with the scene. The valid mask and
 *  ray have both to be aligned to 32 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT8 flag set. For performance
 *  reasons, the rtcIntersect8 function should only get called if the
 *  CPU supports AVX. */
RTCORE_API void rtcIntersect8(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRay8* ray);

/* Intersects a packet of 16 rays with the scene. The valid mask and
 *  ray have both to be aligned to 64 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT16 flag set. For
 *  performance reasons, the rtcIntersect16 function should only get
 *  called if the CPU supports the 16-wide SIMD instructions. */
RTCORE_API void rtcIntersect16(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRay16* ray);

/* Intersects a stream of M rays with the scene. This function can
 *  only be called for scenes with the RTC_INTERSECT_STREAM flag set. The
 *  stride specifies the offset between rays in bytes. */
RTCORE_API void rtcIntersect1M(RTCScene scene, struct RTCIntersectContext* context, struct RTCRay* rays, const unsigned int M, const size_t stride);

/* Intersects a stream of pointers to M rays with the scene. This function can
 *  only be called for scenes with the RTC_INTERSECT_STREAM flag set. */
RTCORE_API void rtcIntersect1Mp(RTCScene scene, struct RTCIntersectContext* context, struct RTCRay** rays, const unsigned int M);

/* Intersects a stream of M ray packets of size N in SOA format with the
 *  scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  ray packets in bytes. */
RTCORE_API void rtcIntersectNM(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayN* rays, const unsigned int N, const unsigned int M, const size_t stride);

/* Intersects a stream of M ray packets of size N in SOA format with
 *  the scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  ray packets in bytes. In contrast to the rtcIntersectNM function
 *  this function accepts a separate data pointer for each component
 *  of the ray packet. */
RTCORE_API void rtcIntersectNp(RTCScene scene, struct RTCIntersectContext* context, const struct RTCRayNp* rays, const unsigned int N);

/* Tests if a single ray is occluded by the scene. The ray has to be
 *  aligned to 16 bytes. This function can only be called for scenes
 *  with the RTC_INTERSECT1 flag set. */
RTCORE_API void rtcOccluded1(RTCScene scene, struct RTCIntersectContext* context, struct RTCRay* ray);

/* Tests if a packet of 4 rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT4
 *  flag set. The valid mask and ray have both to be aligned to 16
 *  bytes. */
RTCORE_API void rtcOccluded4(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRay4* ray);

/* Tests if a packet of 8 rays is occluded by the scene. The valid
 *  mask and ray have both to be aligned to 32 bytes. This function
 *  can only be called for scenes with the RTC_INTERSECT8 flag
 *  set. For performance reasons, the rtcOccluded8 function should
 *  only get called if the CPU supports AVX. */
RTCORE_API void rtcOccluded8(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRay8* ray);

/* Tests if a packet of 16 rays is occluded by the scene. The valid
 *  mask and ray have both to be aligned to 64 bytes. This function
 *  can only be called for scenes with the RTC_INTERSECT16 flag
 *  set. For performance reasons, the rtcOccluded16 function should
 *  only get called if the CPU supports the 16-wide SIMD
 *  instructions. */
RTCORE_API void rtcOccluded16(const int* valid, RTCScene scene, struct RTCIntersectContext* context, struct RTCRay16* ray);

/* Tests if a stream of M rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT_STREAM
 *  flag set. The stride specifies the offset between rays in bytes.*/
RTCORE_API void rtcOccluded1M(RTCScene scene, struct RTCIntersectContext* context, struct RTCRay* rays, const unsigned int M, const size_t stride);

/* Tests if a stream of pointers to M rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT_STREAM
 *  flag set. */
RTCORE_API void rtcOccluded1Mp(RTCScene scene, struct RTCIntersectContext* context, struct RTCRay** rays, const unsigned int M);

/* Tests if a stream of M ray packets of size N in SOA format is occluded by
 *  the scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  rays in bytes.*/
RTCORE_API void rtcOccludedNM(RTCScene scene, struct RTCIntersectContext* context, struct RTCRayN* rays, const unsigned int N, const unsigned int M, const size_t stride);

/* Tests if a stream of M ray packets of size N in SOA format is
 *  occluded by the scene. This function can only be called for scenes
 *  with the RTC_INTERSECT_STREAM flag set. The stride specifies the offset
 *  between rays in bytes. In contrast to the rtcOccludedNM function
 *  this function accepts a separate data pointer for each component
 *  of the ray packet. */
RTCORE_API void rtcOccludedNp(RTCScene scene, struct RTCIntersectContext* context, const struct RTCRayNp* rays, const unsigned int N);

/* Retains the scene (increments reference count). */
RTCORE_API void rtcRetainScene(RTCScene scene);

/* Releases the scene (decrements reference count). */
RTCORE_API void rtcReleaseScene(RTCScene scene);

#if defined(__cplusplus)

/* Helper to easily combing scene flags */
inline RTCSceneFlags operator|(const RTCSceneFlags a, const RTCSceneFlags b) {
  return (RTCSceneFlags)((size_t)a | (size_t)b);
}
  
}
#endif
