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

/*! forward declarations for ray structures */
struct RTCRay;
struct RTCRay4;
struct RTCRay8;
struct RTCRay16;
struct RTCRayNp;

/*! acceleration structure flags */
enum RTCAccelFlags
{
  RTC_ACCEL_DEFAULT      = 0,          //!< default mode
  RTC_ACCEL_COMPACT      = (1 << 1),   //!< use memory conservative acceleration structure
  RTC_ACCEL_ROBUST       = (1 << 2),   //!< use acceleration structure that allows robust traversal
  RTC_ACCEL_ROBUST_COMPACT = (1 << 1) | (1 << 2)
};

/*! scene commit flags */
enum RTCBuildQuality
{
  RTC_BUILD_QUALITY_LOW = 0,     //!< create lower quality data structures (for dynamic scenes)
  RTC_BUILD_QUALITY_NORMAL = 1,  //!< default build quality for most usages
  RTC_BUILD_QUALITY_HIGH = 2     //!< create higher quality data structures (longer build times)
};

/*! some additional flags to control the build */
enum RTCBuildHint
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

/*! intersection context passed to intersect/occluded calls */
struct RTCIntersectContext
{
  RTCIntersectFlags flags;   //!< intersection flags
  void* userRayExt;          //!< can be used to pass extended ray data to callbacks
};

RTCORE_FORCEINLINE void rtcInitIntersectionContext(RTCIntersectContext* context)
{
  context->flags = RTC_INTERSECT_INCOHERENT;
  context->userRayExt = NULL;
}

/*! \brief Defines an opaque scene type */
typedef struct __RTCScene {}* RTCScene;

/*! Creates a new scene. */
RTCORE_API RTCScene rtcDeviceNewScene (RTCDevice device);

/*! \brief Type of progress callback function. */
typedef bool (*RTCProgressMonitorFunc)(void* ptr, const double n);

/*! \brief Sets the progress callback function which is called during hierarchy build of this scene. */
RTCORE_API void rtcSetProgressMonitorFunction(RTCScene scene, RTCProgressMonitorFunc func, void* ptr);

/*! sets the build mode for a scene */
RTCORE_API void rtcSetBuildMode(RTCScene scene, RTCAccelFlags accel, RTCBuildQuality quality, RTCBuildHint hint);

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

/*! Commits the geometry of the scene. The calling threads will be
 *  used internally as a worker threads on some implementations. The
 *  function will wait until 'numThreads' threads have called this
 *  function and all threads return from the function after the scene
 *  commit is finished. The application threads will not be used as
 *  worker threads when the TBB tasking system is enabled (which is
 *  the default). On CPUs, we recommend also using TBB inside your
 *  application to share threads. We recommend using the
 *  rtcCommitThread feature to share threads on the Xeon Phi
 *  coprocessor. */
RTCORE_API void rtcCommitThread(RTCScene scene, unsigned int threadID, unsigned int numThreads);

/*! Returns AABB of the scene. rtcCommit has to get called
 *  previously to this function. */
RTCORE_API void rtcGetBounds(RTCScene scene, RTCBounds& bounds_o);

/*! Returns linear AABBs of the scene. The result bounds_o gets filled
 *  with AABBs for time 0 and time 1. rtcCommit has to get called
 *  previously to this function. */
RTCORE_API void rtcGetLinearBounds(RTCScene scene, RTCBounds* bounds_o);

/*! Intersects a single ray with the scene. The ray has to be aligned
 *  to 16 bytes. This function can only be called for scenes with the
 *  RTC_INTERSECT1 flag set. */
RTCORE_API void rtcIntersect1 (RTCScene scene, const RTCIntersectContext* context, RTCRay& ray);

/*! Intersects a packet of 4 rays with the scene. The valid mask and
 *  ray have both to be aligned to 16 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT4 flag set. */
RTCORE_API void rtcIntersect4 (const int* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay4& ray);

/*! Intersects a packet of 8 rays with the scene. The valid mask and
 *  ray have both to be aligned to 32 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT8 flag set. For performance
 *  reasons, the rtcIntersect8 function should only get called if the
 *  CPU supports AVX. */
RTCORE_API void rtcIntersect8 (const int* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay8& ray);

/*! Intersects a packet of 16 rays with the scene. The valid mask and
 *  ray have both to be aligned to 64 bytes. This function can only be
 *  called for scenes with the RTC_INTERSECT16 flag set. For
 *  performance reasons, the rtcIntersect16 function should only get
 *  called if the CPU supports the 16-wide SIMD instructions. */
RTCORE_API void rtcIntersect16 (const int* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay16& ray);

/*! Intersects a stream of M rays with the scene. This function can
 *  only be called for scenes with the RTC_INTERSECT_STREAM flag set. The
 *  stride specifies the offset between rays in bytes. */
RTCORE_API void rtcIntersect1M (RTCScene scene, const RTCIntersectContext* context, RTCRay* rays, const unsigned int M, const size_t stride);

/*! Intersects a stream of pointers to M rays with the scene. This function can
 *  only be called for scenes with the RTC_INTERSECT_STREAM flag set. */
RTCORE_API void rtcIntersect1Mp (RTCScene scene, const RTCIntersectContext* context, RTCRay** rays, const unsigned int M);

/*! Intersects a stream of M ray packets of size N in SOA format with the
 *  scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  ray packets in bytes. */
RTCORE_API void rtcIntersectNM (RTCScene scene, const RTCIntersectContext* context, struct RTCRayN* rays, const unsigned int N, const unsigned int M, const size_t stride);

/*! Intersects a stream of M ray packets of size N in SOA format with
 *  the scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  ray packets in bytes. In contrast to the rtcIntersectNM function
 *  this function accepts a separate data pointer for each component
 *  of the ray packet. */
RTCORE_API void rtcIntersectNp (RTCScene scene, const RTCIntersectContext* context, const RTCRayNp& rays, const unsigned int N);

/*! Tests if a single ray is occluded by the scene. The ray has to be
 *  aligned to 16 bytes. This function can only be called for scenes
 *  with the RTC_INTERSECT1 flag set. */
RTCORE_API void rtcOccluded1 (RTCScene scene, const RTCIntersectContext* context, RTCRay& ray);

/*! Tests if a packet of 4 rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT4
 *  flag set. The valid mask and ray have both to be aligned to 16
 *  bytes. */
RTCORE_API void rtcOccluded4 (const int* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay4& ray);

/*! Tests if a packet of 8 rays is occluded by the scene. The valid
 *  mask and ray have both to be aligned to 32 bytes. This function
 *  can only be called for scenes with the RTC_INTERSECT8 flag
 *  set. For performance reasons, the rtcOccluded8 function should
 *  only get called if the CPU supports AVX. */
RTCORE_API void rtcOccluded8 (const int* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay8& ray);

/*! Tests if a packet of 16 rays is occluded by the scene. The valid
 *  mask and ray have both to be aligned to 64 bytes. This function
 *  can only be called for scenes with the RTC_INTERSECT16 flag
 *  set. For performance reasons, the rtcOccluded16 function should
 *  only get called if the CPU supports the 16-wide SIMD
 *  instructions. */
RTCORE_API void rtcOccluded16 (const int* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay16& ray);

/*! Tests if a stream of M rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT_STREAM
 *  flag set. The stride specifies the offset between rays in bytes.*/
RTCORE_API void rtcOccluded1M (RTCScene scene, const RTCIntersectContext* context, RTCRay* rays, const unsigned int M, const size_t stride);

/*! Tests if a stream of pointers to M rays is occluded by the scene. This
 *  function can only be called for scenes with the RTC_INTERSECT_STREAM
 *  flag set. */
RTCORE_API void rtcOccluded1Mp (RTCScene scene, const RTCIntersectContext* context, RTCRay** rays, const unsigned int M);

/*! Tests if a stream of M ray packets of size N in SOA format is occluded by
 *  the scene. This function can only be called for scenes with the
 *  RTC_INTERSECT_STREAM flag set. The stride specifies the offset between
 *  rays in bytes.*/
RTCORE_API void rtcOccludedNM (RTCScene scene, const RTCIntersectContext* context, struct RTCRayN* rays, const unsigned int N, const unsigned int M, const size_t stride);

/*! Tests if a stream of M ray packets of size N in SOA format is
 *  occluded by the scene. This function can only be called for scenes
 *  with the RTC_INTERSECT_STREAM flag set. The stride specifies the offset
 *  between rays in bytes. In contrast to the rtcOccludedNM function
 *  this function accepts a separate data pointer for each component
 *  of the ray packet. */
RTCORE_API void rtcOccludedNp (RTCScene scene, const RTCIntersectContext* context, const RTCRayNp& rays, const unsigned int N);

/*! Deletes the scene. All contained geometry get also destroyed. */
RTCORE_API void rtcDeleteScene (RTCScene scene);

/*! @} */
