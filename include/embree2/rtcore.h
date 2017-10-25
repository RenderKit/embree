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

#include <stddef.h>
#include <sys/types.h>
#include <stdbool.h>

#include "rtcore_version.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(_WIN32)
#if defined(_M_X64)
typedef long long ssize_t;
#else
typedef int ssize_t;
#endif
#endif

#ifndef RTCORE_API
#if defined(_WIN32) && !defined(EMBREE_STATIC_LIB)
#  define RTCORE_API __declspec(dllimport) 
#else
#  define RTCORE_API 
#endif
#endif

#ifdef _WIN32
#  define RTCORE_ALIGN(...) __declspec(align(__VA_ARGS__))
#else
#  define RTCORE_ALIGN(...) __attribute__((aligned(__VA_ARGS__)))
#endif

#if !defined (RTCORE_DEPRECATED)
#ifdef __GNUC__
  #define RTCORE_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
  #define RTCORE_DEPRECATED __declspec(deprecated)
#else
  #define RTCORE_DEPRECATED
#endif
#endif

#if defined(_WIN32) 
#  define RTCORE_FORCEINLINE __forceinline
#else
#  define RTCORE_FORCEINLINE inline __attribute__((always_inline))
#endif

/*! \file rtcore.h Defines the Embree Ray Tracing Kernel API for C and C++ 

   This file defines the Embree ray tracing kernel API for C and
   C++. The user is supposed to include this file, and alternatively
   the rtcore_ray.h file, but none of the other .h files in this
   folder. */

/*! \{ */

/*! Axis aligned bounding box representation */
struct RTCORE_ALIGN(16) RTCBounds
{
  float lower_x, lower_y, lower_z, align0;
  float upper_x, upper_y, upper_z, align1;
};

/*! \brief Defines an opaque device type */
typedef struct __RTCDevice* RTCDevice;

/*! \brief Creates a new Embree device.

  Creates a new Embree device to be used by the application. An
  application typically creates only a single Embree device, but it is
  valid to use multiple devices inside an application. A configuration
  string can be passed at construction time, that allows to configure
  implementation specific parameters. If this string is NULL, a
  default configuration is used. The following configuration flags are
  supported by the Embree implementation of the API:
  
  verbose = num,       // sets verbosity level (default is 0)

  If Embree is started on an unsupported CPU, rtcNewDevice will fail and
  set the RTC_UNSUPPORTED_CPU error code.
  
*/
RTCORE_API RTCDevice rtcNewDevice(const char* cfg);

/*! \brief Deletes an Embree device.

  Deletes the Embree device again. After deletion, all scene handles
  are invalid. */
RTCORE_API void rtcDeleteDevice(RTCDevice device);

/*! \brief Parameters that can get configured using the rtcSetParameter functions. */
enum RTCParameter {
  RTC_SOFTWARE_CACHE_SIZE = 0,                /*! Configures the software cache size (used
                                                to cache subdivision surfaces for
                                                instance). The size is specified as an
                                                integer number of bytes. The software
                                                cache cannot be configured during
                                                rendering. (write only) */

  RTC_CONFIG_INTERSECT1 = 1,                  //!< checks if rtcIntersect1 is supported (read only)
  RTC_CONFIG_INTERSECT4 = 2,                  //!< checks if rtcIntersect4 is supported (read only)
  RTC_CONFIG_INTERSECT8 = 3,                  //!< checks if rtcIntersect8 is supported (read only)
  RTC_CONFIG_INTERSECT16 = 4,                 //!< checks if rtcIntersect16 is supported (read only)
  RTC_CONFIG_INTERSECT_STREAM = 5,            //!< checks if rtcIntersect1M, rtcIntersectNM and rtcIntersectNp are supported (read only)

  RTC_CONFIG_RAY_MASK = 6,                    //!< checks if ray masks are supported (read only)
  RTC_CONFIG_BACKFACE_CULLING = 7,            //!< checks if backface culling is supported (read only)
  RTC_CONFIG_INTERSECTION_FILTER = 8,         //!< checks if intersection filters are enabled (read only)
  RTC_CONFIG_INTERSECTION_FILTER_RESTORE = 9, //!< checks if intersection filters restores previous hit (read only)
  RTC_CONFIG_IGNORE_INVALID_RAYS = 11,        //!< checks if invalid rays are ignored (read only)
  RTC_CONFIG_TASKING_SYSTEM = 12,             //!< return used tasking system (0 = INTERNAL, 1 = TBB, 2 = PPL) (read only)

  RTC_CONFIG_VERSION_MAJOR = 13,             //!< returns Embree major version (read only)
  RTC_CONFIG_VERSION_MINOR = 14,             //!< returns Embree minor version (read only)
  RTC_CONFIG_VERSION_PATCH = 15,             //!< returns Embree patch version (read only)
  RTC_CONFIG_VERSION = 16,                   //!< returns Embree version as integer (e.g. Embree v2.8.2 -> 20802) (read only)

  RTC_CONFIG_TRIANGLE_GEOMETRY = 17,         //!< checks if triangle geometries are supported
  RTC_CONFIG_QUAD_GEOMETRY = 18,             //!< checks if quad geometries are supported
  RTC_CONFIG_LINE_GEOMETRY = 19,             //!< checks if line geometries are supported
  RTC_CONFIG_HAIR_GEOMETRY = 20,              //!< checks if hair geometries are supported
  RTC_CONFIG_SUBDIV_GEOMETRY = 21,           //!< checks if subdiv geometries are supported
  RTC_CONFIG_USER_GEOMETRY = 22,             //!< checks if user geometries are supported

  RTC_CONFIG_COMMIT_JOIN = 23,               //!< checks if rtcCommitJoin can be used to join build operation (not supported when compiled with some older TBB versions)
  RTC_CONFIG_COMMIT_THREAD = 24,             //!< checks if rtcCommitThread is available (not supported when compiled with some older TBB versions)
};

/*! \brief Configures some device parameters. */
RTCORE_API void rtcDeviceSetParameter1i(RTCDevice device, const enum RTCParameter parm, ssize_t val);

/*! \brief Reads some device parameter. */
RTCORE_API ssize_t rtcDeviceGetParameter1i(RTCDevice device, const enum RTCParameter parm);

/*! \brief Error codes returned by the rtcGetError function. */
enum RTCError {
  RTC_NO_ERROR = 0,          //!< No error has been recorded.
  RTC_UNKNOWN_ERROR = 1,     //!< An unknown error has occured.
  RTC_INVALID_ARGUMENT = 2,  //!< An invalid argument is specified
  RTC_INVALID_OPERATION = 3, //!< The operation is not allowed for the specified object.
  RTC_OUT_OF_MEMORY = 4,     //!< There is not enough memory left to execute the command.
  RTC_UNSUPPORTED_CPU = 5,   //!< The CPU is not supported as it does not support SSE2.
  RTC_CANCELLED = 6,         //!< The user has cancelled the operation through the RTC_PROGRESS_MONITOR_FUNCTION callback
};

/*! \brief Returns the value of the per-thread error flag. 

  If an error occurs this flag is set to an error code if it stores no
  previous error. The rtcGetError function reads and returns the
  currently stored error and clears the error flag again. */
RTCORE_API enum RTCError rtcDeviceGetError(RTCDevice device);

/*! \brief Type of error callback function. */
typedef void (*RTCErrorFunc)(void* userPtr, const enum RTCError code, const char* str);

/*! \brief Sets a callback function that is called whenever an error occurs. */
RTCORE_API void rtcDeviceSetErrorFunction(RTCDevice device, RTCErrorFunc func, void* userPtr);

/*! \brief Type of memory consumption callback function. */
typedef bool (*RTCMemoryMonitorFunc)(void* ptr, const ssize_t bytes, const bool post);

/*! \brief Sets the memory consumption callback function which is
 *  called before or after the library allocates or frees memory. The
 *  userPtr pointer is passed to each invokation of the callback
 *  function. */
RTCORE_API void rtcDeviceSetMemoryMonitorFunction(RTCDevice device, RTCMemoryMonitorFunc func, void* userPtr);

#if defined(__cplusplus)
}
#endif

#include "rtcore_ray.h"
#include "rtcore_scene.h"
#include "rtcore_geometry.h"
#include "rtcore_geometry_user.h"
#include "rtcore_builder.h"

#if defined(__cplusplus)

/*! \brief Helper to easily combing geometry flags */
inline RTCGeometryFlags operator|(const RTCGeometryFlags a, const RTCGeometryFlags b) {
  return (RTCGeometryFlags)((size_t)a | (size_t)b);
}

#endif

/*! \} */
