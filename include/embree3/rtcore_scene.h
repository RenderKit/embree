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

/*! \ingroup embree_kernel_api */
/*! \{ */

#if defined(__cplusplus)
extern "C" {
#endif
  
/*! forward declarations for ray structures */
struct RTCRay;
struct RTCRay4;
struct RTCRay8;
struct RTCRay16;
struct RTCRayNp;

/*! acceleration structure flags */
enum RTCAccelFlags
{
  RTC_ACCEL_FAST      = 0,          //!< default mode
  RTC_ACCEL_COMPACT      = (1 << 1),   //!< use memory conservative acceleration structure
  RTC_ACCEL_ROBUST       = (1 << 2),   //!< use acceleration structure that allows robust traversal
  RTC_ACCEL_ROBUST_COMPACT = (1 << 1) | (1 << 2)
};

/*! scene commit flags */
enum RTCBuildQuality
{
  RTC_BUILD_QUALITY_LOW = 0,     //!< create lower quality data structures (for dynamic scenes)
  RTC_BUILD_QUALITY_MEDIUM = 1,  //!< default build quality for most usages
  RTC_BUILD_QUALITY_HIGH = 2     //!< create higher quality data structures (longer build times)
};

/*! some additional flags to control the build */
enum RTCBuildHints
{
  RTC_BUILD_HINT_NONE = 0,
  RTC_BUILD_HINT_DYNAMIC = (1 << 0)
};

/*! intersection flags */
enum RTCIntersectFlags
{
  RTC_INTERSECT_COHERENT                 = 0,  //!< optimize for coherent rays
  RTC_INTERSECT_INCOHERENT               = 1   //!< optimize for incoherent rays
};

/*! Arguments for RTCFilterFunctionN callback */
struct RTCFilterFunctionNArguments
{
  int* valid;                                /*!< pointer to valid mask */
  void* geomUserPtr;                         /*!< pointer to geometry user data */
  const struct RTCIntersectContext* context; /*!< intersection context as passed to rtcIntersect/rtcOccluded */
  struct RTCRayN* ray;                       /*!< ray and previous hit */
  struct RTCHitN* potentialHit;              /*!< potential new hit */
  unsigned int N;                            /*!< size of ray packet */
};
  
/*! Intersection filter function for ray packets of size N. */
typedef void (*RTCFilterFunctionN)(const struct RTCFilterFunctionNArguments* const args);
  
/*! intersection context passed to intersect/occluded calls */
struct RTCIntersectContext
{
  enum RTCIntersectFlags flags;   //!< intersection flags
  //RTCFilterFunctionN filter;      //!< filter function to execute
  void* userRayExt;               //!< can be used to pass extended ray data to callbacks
};

RTCORE_FORCEINLINE void rtcInitIntersectionContext(struct RTCIntersectContext* context)
{
  context->flags = RTC_INTERSECT_INCOHERENT;
  //context->filter = NULL;
  context->userRayExt = NULL;
}

/*! \brief Defines an opaque scene type */
typedef struct __RTCScene* RTCScene;

/*! Creates a new scene. */
RTCORE_API RTCScene rtcDeviceNewScene (RTCDevice device);

/*! \brief Type of progress callback function. */
typedef bool (*RTCProgressMonitorFunction)(void* ptr, const double n);

/*! \brief Sets the progress callback function which is called during hierarchy build of this scene. */
RTCORE_API void rtcSetProgressMonitorFunction(RTCScene scene, RTCProgressMonitorFunction func, void* ptr);

/*! sets the acceleration structure for a scene */
RTCORE_API void rtcSetAccelFlags(RTCScene scene, enum RTCAccelFlags accel);

/*! sets the build quality of a scene */
RTCORE_API void rtcSetBuildQuality(RTCScene scene, enum RTCBuildQuality quality);

/*! sets the build hints of a scene */
RTCORE_API void rtcSetBuildHints(RTCScene scene, enum RTCBuildHints hints);

/*! Commits the geometry of the scene. After initializing or modifying
 *  geometries, commit has to get called before tracing
 *  rays. */
RTCORE_API void rtcCommit (RTCScene scene);

/*! Commits the geometry of the scene in join mode. When Embree is
 *  using TBB (default), threads that call `rtcCommitJoin` will
 *  participate in the hierarchy build procedure. When Embree is using
 *  the internal tasking system, exclusively threads that call
 *  `rtcCommitJoin` will execute the build procedure. Do not
 *  mix `rtcCommitJoin` with other commit calls. */
RTCORE_API void rtcCommitJoin (RTCScene scene);

/*! Returns AABB of the scene. rtcCommit has to get called
 *  previously to this function. */
RTCORE_API void rtcGetBounds(RTCScene scene, struct RTCBounds* bounds_o);

/*! Returns linear AABBs of the scene. The result bounds_o gets filled
 *  with AABBs for time 0 and time 1. rtcCommit has to get called
 *  previously to this function. */
RTCORE_API void rtcGetLinearBounds(RTCScene scene, struct RTCBounds* bounds_o);

/*! Intersects a single ray with the scene. The ray has to be aligned
 *  to 16 bytes. This function can only be called for scenes with the
 *  RTC_INTERSECT1 flag set. */
RTCORE_API void rtcIntersect1 (RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay* ray);

/*! Intersects a packet of 4 rays with the scene. The valid mask and
 *  ray have both to be aligned to 16 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT4 flag set. */
RTCORE_API void rtcIntersect4 (const int* valid, RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay4* ray);

/*! Intersects a packet of 8 rays with the scene. The valid mask and
 *  ray have both to be aligned to 32 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT8 flag set. For performance
 *  reasons, the rtcIntersect8 function should only get called if the
 *  CPU supports AVX. */
RTCORE_API void rtcIntersect8 (const int* valid, RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay8* ray);

/*! Intersects a packet of 16 rays with the scene. The valid mask and
 *  ray have both to be aligned to 64 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT16 flag set. For
 *  performance reasons, the rtcIntersect16 function should only get
 *  called if the CPU supports the 16-wide SIMD instructions. */
RTCORE_API void rtcIntersect16 (const int* valid, RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay16* ray);

/*! Intersects a stream of M rays with the scene. This function can
 *  only be called for scenes with the RTC_INTERSECT_STREAM flag set. The
 *  stride specifies the offset between rays in bytes. */
RTCORE_API void rtcIntersect1M (RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay* rays, const unsigned int M, const size_t stride);

/*! Intersects a stream of pointers to M rays with the scene. This function can
 *  only be called for scenes with the RTC_INTERSECT_STREAM flag set. */
RTCORE_API void rtcIntersect1Mp (RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay** rays, const unsigned int M);

/*! Intersects a stream of M ray packets of size N in SOA format with the
 *  scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  ray packets in bytes. */
RTCORE_API void rtcIntersectNM (RTCScene scene, const struct RTCIntersectContext* context, struct RTCRayN* rays, const unsigned int N, const unsigned int M, const size_t stride);

/*! Intersects a stream of M ray packets of size N in SOA format with
 *  the scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  ray packets in bytes. In contrast to the rtcIntersectNM function
 *  this function accepts a separate data pointer for each component
 *  of the ray packet. */
RTCORE_API void rtcIntersectNp (RTCScene scene, const struct RTCIntersectContext* context, const struct RTCRayNp* rays, const unsigned int N);

/*! Tests if a single ray is occluded by the scene. The ray has to be
 *  aligned to 16 bytes. This function can only be called for scenes
 *  with the RTC_INTERSECT1 flag set. */
RTCORE_API void rtcOccluded1 (RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay* ray);

/*! Tests if a packet of 4 rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT4
 *  flag set. The valid mask and ray have both to be aligned to 16
 *  bytes. */
RTCORE_API void rtcOccluded4 (const int* valid, RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay4* ray);

/*! Tests if a packet of 8 rays is occluded by the scene. The valid
 *  mask and ray have both to be aligned to 32 bytes. This function
 *  can only be called for scenes with the RTC_INTERSECT8 flag
 *  set. For performance reasons, the rtcOccluded8 function should
 *  only get called if the CPU supports AVX. */
RTCORE_API void rtcOccluded8 (const int* valid, RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay8* ray);

/*! Tests if a packet of 16 rays is occluded by the scene. The valid
 *  mask and ray have both to be aligned to 64 bytes. This function
 *  can only be called for scenes with the RTC_INTERSECT16 flag
 *  set. For performance reasons, the rtcOccluded16 function should
 *  only get called if the CPU supports the 16-wide SIMD
 *  instructions. */
RTCORE_API void rtcOccluded16 (const int* valid, RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay16* ray);

/*! Tests if a stream of M rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT_STREAM
 *  flag set. The stride specifies the offset between rays in bytes.*/
RTCORE_API void rtcOccluded1M (RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay* rays, const unsigned int M, const size_t stride);

/*! Tests if a stream of pointers to M rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT_STREAM
 *  flag set. */
RTCORE_API void rtcOccluded1Mp (RTCScene scene, const struct RTCIntersectContext* context, struct RTCRay** rays, const unsigned int M);

/*! Tests if a stream of M ray packets of size N in SOA format is occluded by
 *  the scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  rays in bytes.*/
RTCORE_API void rtcOccludedNM (RTCScene scene, const struct RTCIntersectContext* context, struct RTCRayN* rays, const unsigned int N, const unsigned int M, const size_t stride);

/*! Tests if a stream of M ray packets of size N in SOA format is
 *  occluded by the scene. This function can only be called for scenes
 *  with the RTC_INTERSECT_STREAM flag set. The stride specifies the offset
 *  between rays in bytes. In contrast to the rtcOccludedNM function
 *  this function accepts a separate data pointer for each component
 *  of the ray packet. */
RTCORE_API void rtcOccludedNp (RTCScene scene, const struct RTCIntersectContext* context, const struct RTCRayNp* rays, const unsigned int N);

/*! Releases the scene. */
RTCORE_API void rtcReleaseScene (RTCScene scene);

#if defined(__cplusplus)
}
#endif
  
/*! @} */
