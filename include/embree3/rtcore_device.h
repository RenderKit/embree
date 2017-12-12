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

#include "rtcore_common.h"

#if defined(__cplusplus)
extern "C" {
#endif

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
  set the RTC_ERROR_UNSUPPORTED_CPU error code.
  
*/
RTCORE_API RTCDevice rtcNewDevice(const char* cfg);

/*! \brief Deletes an Embree device.

  Deletes the Embree device again. After deletion, all scene handles
  are invalid. */
RTCORE_API void rtcReleaseDevice(RTCDevice device);

/*! \brief Device properties that can be queried using the rtcGetDeviceParameter function. */
enum RTCDeviceProperty {
  RTC_DEVICE_PROPERTY_SOFTWARE_CACHE_SIZE = 0,             /*! Configures the software cache size (used
                                                               to cache subdivision surfaces for
                                                               instance). The size is specified as an
                                                               integer number of bytes. The software
                                                               cache cannot be configured during
                                                               rendering. (write only) */

  RTC_DEVICE_PROPERTY_INTERSECT1_SUPPORTED = 1,            //!< flag whether rtcIntersect1 is supported (read only)
  RTC_DEVICE_PROPERTY_INTERSECT4_SUPPORTED = 2,            //!< flag whether rtcIntersect4 is supported (read only)
  RTC_DEVICE_PROPERTY_INTERSECT8_SUPPORTED = 3,            //!< flag whether rtcIntersect8 is supported (read only)
  RTC_DEVICE_PROPERTY_INTERSECT16_SUPPORTED = 4,           //!< flag whether rtcIntersect16 is supported (read only)
  RTC_DEVICE_PROPERTY_INTERSECT_STREAM_SUPPORTED = 5,      //!< flag whether rtcIntersect1M, rtcIntersectNM and rtcIntersectNp are supported (read only)

  RTC_DEVICE_PROPERTY_RAY_MASK_SUPPORTED = 6,              //!< flag whether ray masks are supported (read only)
  RTC_DEVICE_PROPERTY_BACKFACE_CULLING_ENABLED = 7,        //!< flag whether backface culling is enabled (read only)
  RTC_DEVICE_PROPERTY_FILTER_FUNCTION_SUPPORTED = 8,       //!< flag whether filter functions are supported (read only)
  RTC_DEVICE_PROPERTY_IGNORE_INVALID_RAYS_ENABLED = 11,    //!< flag whether invalid rays are ignored (read only)
  RTC_DEVICE_PROPERTY_TASKING_SYSTEM = 12,                 //!< tasking system used (0 = INTERNAL, 1 = TBB, 2 = PPL) (read only)

  RTC_DEVICE_PROPERTY_VERSION_MAJOR = 13,                  //!< Embree major version (read only)
  RTC_DEVICE_PROPERTY_VERSION_MINOR = 14,                  //!< Embree minor version (read only)
  RTC_DEVICE_PROPERTY_VERSION_PATCH = 15,                  //!< Embree patch version (read only)
  RTC_DEVICE_PROPERTY_VERSION = 16,                        //!< Embree version as integer (e.g. Embree v2.8.2 -> 20802) (read only)

  RTC_DEVICE_PROPERTY_TRIANGLE_GEOMETRY_SUPPORTED = 17,    //!< flag whether triangle geometries are supported (read only)
  RTC_DEVICE_PROPERTY_QUAD_GEOMETRY_SUPPORTED = 18,        //!< flag whether quad geometries are supported (read only)
  RTC_DEVICE_PROPERTY_CURVE_GEOMETRY_SUPPORTED = 19,        //!< flag whether curve geometries are supported (read only)
  RTC_DEVICE_PROPERTY_SUBDIVISION_GEOMETRY_SUPPORTED = 20, //!< flag whether subdivision geometries are supported (read only)
  RTC_DEVICE_PROPERTY_USER_GEOMETRY_SUPPORTED = 21,        //!< flag whether user geometries are supported (read only)

  RTC_DEVICE_PROPERTY_COMMIT_JOIN_SUPPORTED = 30,          //!< flag whether rtcCommitJoinScene can be used to join build operation (not supported when compiled with some older TBB versions) (read only)
};

/*! \brief Sets a device property value. */
RTCORE_API void rtcSetDeviceProperty(RTCDevice device, const enum RTCDeviceProperty prop, ssize_t val);

/*! \brief Gets a device property value. */
RTCORE_API ssize_t rtcGetDeviceProperty(RTCDevice device, const enum RTCDeviceProperty prop);

/*! \brief Error codes returned by the rtcGetDeviceError function. */
enum RTCError {
  RTC_ERROR_NONE = 0,              //!< no error has been recorded
  RTC_ERROR_UNKNOWN = 1,           //!< an unknown error has occured
  RTC_ERROR_INVALID_ARGUMENT = 2,  //!< an invalid argument is specified
  RTC_ERROR_INVALID_OPERATION = 3, //!< the operation is not allowed for the specified object
  RTC_ERROR_OUT_OF_MEMORY = 4,     //!< there is not enough memory left to execute the command
  RTC_ERROR_UNSUPPORTED_CPU = 5,   //!< the CPU is not supported as it does not support SSE2
  RTC_ERROR_CANCELLED = 6,         //!< the user has cancelled the operation through the RTC_PROGRESS_MONITOR_FUNCTION callback
};

/*! \brief Returns the value of the per-thread error flag. 

  If an error occurs this flag is set to an error code if it stores no
  previous error. The rtcGetError function reads and returns the
  currently stored error and clears the error flag again. */
RTCORE_API enum RTCError rtcGetDeviceError(RTCDevice device);

/*! \brief Type of error callback function. */
typedef void (*RTCErrorFunction)(void* userPtr, const enum RTCError code, const char* str);

/*! \brief Sets a callback function that is called whenever an error occurs. */
RTCORE_API void rtcSetDeviceErrorFunction(RTCDevice device, RTCErrorFunction func, void* userPtr);

/*! \brief Type of memory consumption callback function. */
typedef bool (*RTCMemoryMonitorFunction)(void* ptr, const ssize_t bytes, const bool post);

/*! \brief Sets the memory consumption callback function which is
 *  called before or after the library allocates or frees memory. The
 *  userPtr pointer is passed to each invokation of the callback
 *  function. */
RTCORE_API void rtcSetDeviceMemoryMonitorFunction(RTCDevice device, RTCMemoryMonitorFunction func, void* userPtr);

#if defined(__cplusplus)
}
#endif
