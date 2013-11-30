// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#ifndef __RTCORE_H__
#define __RTCORE_H__

#include <stddef.h>

#ifndef RTCORE_API
#ifdef _WIN32
#  define RTCORE_API extern "C" //__declspec(dllimport) // FIXME
#else
#  define RTCORE_API extern "C"
#endif
#endif

#include "rtcore_scene.h"
#include "rtcore_geometry.h"
#include "rtcore_geometry_user.h"

// FIXME: describe alignment requirements for rays and valid masks
// FIXME: describe that tnear value has to be > 0
// FIMXE: intersect8 should only get called if CPU supports AVX, etc.
// FIXME: user geometry, intersect8 will call intersect8ptr, etc.

/* 

   This file defines the Embree ray tracing kernel API for C and
   C++. It is a low level API that supports defining and committing of
   geometry and tracing of rays. Static and dynamic geometry are both
   supported, as well as finding the closest intersection of a ray,
   and testing a ray segment for any intersection with the
   scene. Single rays, as well as packets of rays in struct of array
   layout are supported for packet sizes of 1, 4, 8, and 16. Linear
   motion blur is also supported.

   Before invoking any API call, the Embree ray tracing core has to
   get initialized through the rtcInit call. Before the application exits
   it should call rtcExit. Initializing Embree again after an rtcExit
   is safe.

   A scene, represented by the RTCScene type, is a container for
   different geometries. The current version of the API supports
   triangle meshes, single level instances of other scenes, and user
   defined geometries. The API is designed in a way that easily allows
   adding new geometry types in later releases.

   Geometries are always contained in the scene they are created
   in. Each geometry is assigned an integer ID at creating time, which
   is unique for that scene. Two types of scene are supported, dynamic
   and static scenes.

   A dynamic scene is created by invoking the rtcNewDynamicScene
   call. Different geometries can now be created inside that scene
   through rtcNewTriangleMesh, rtcNewInstance, and rtcNewUserGeometry
   calls. Geometries are enabled by default and data for these
   geometries can be set by other API calls as described below for
   each geometry type. Once the scene geometry is specified, an
   rtcCommit calls will finish the scene description and trigger
   building of internal data structures. After the rtcCommit call it
   is safe to trace rays. Geometries can be disabled (rtcDisable call)
   and enabled again (rtcEnable call). Geometries can also get
   modified again, including vertex and index arrays. After the
   modification to some geometry is finished, rtcModified call has to
   get invoked on that geometry. If geometries got enabled, disabled,
   or modified an rtcCommit call has to get invoked before tracing
   rays. For dynamic scenes, assigned geometry IDs fulfill the
   following properties. As long as no geometry got deleted, all IDs
   are assigned sequentially, starting from 0. If geometries got
   deleted, the implementation will reuse IDs later on in an
   implementation dependent way. Consequently sequential assignment is
   no longer guaranteed, but a compact range of IDs. These rules allow
   the application to manage a dynamic array to efficiently map from
   geometry IDs to its own geometry representation.

   A static scene is created by the rtcNewStaticScene call. Geometries
   can only be created and modified until the first rtcCommit
   call. After that call, each access to any geometry of that static
   scene is invalid, including the deletion of geometries. For static
   scenes, geometry IDs are assigned sequentially starting at 0. This
   allows the application to use a fixed size array to map from
   geometry IDs to its own geometry representation.
   
   A 32 bit geometry mask can be assigned to triangle mesh
   geometries. Only if the bitwise and operation of this mask with the
   mask stored inside the ray is not 0, triangles of this mesh are hit
   by a ray. This feature can be used to disable selected triangle
   meshes for specifically tagged rays, e.g. to disable shadow casting
   for some geometry. This API feature is optional and disabled in
   Embree by default at compile time.

   API calls that access geometries are only thread safe as long as
   different geometries are accessed. Accesses to one geometry have to
   get sequentialized by the application. All other API calls are
   thread safe. The rtcIntersect and rtcOccluded calls are re-entrant,
   but only for other rtcIntersect and rtcOccluded calls. It is thus
   safe to trace new rays when intersecting a user defined object, but
   not supported to create new geometry inside the intersect function
   of that user defined object.

   The API supports finding the closest hit of a ray segment with the
   scene, and determining if any hit between a ray segment and the
   scene exists. In the ray packet mode (with packet size of N), the
   user has to provide a pointer to N 32 bit integers that act as a
   ray activity mask. If one of these integers is set to 0x00000000
   the corresponding ray is considered inactive and if the integer is
   set to 0xFFFFFFFF, the ray is considered active. Rays that are
   inactive will not update any hit information.
   
   Finding the closest hit distance if done through the rtcIntersect
   functions. These get the activity mask, the scene, and a ray as
   input. The user has to initialize the ray origin (org), ray
   direction (dir), and ray segment (tnear, tfar). The geometry ID
   (geomID member) has to get initialized to INVALID_GEOMETRY_ID
   (-1). If the scene contains linear motion blur, also the ray time
   (time) has to get initialized. If the scene contains instances,
   also the instance ID (instID) has to get initialized. If ray masks
   are enabled at compile time, also the ray mask (mask) have to get
   initialized. After tracing the ray, the hit distance (tfar),
   geometry normal (Ng), local hit coordinates (u,v), geometry ID
   (geomID), and primitive ID (primID) are set. The geometry ID
   corresponds to the ID returned at creation time of the hit
   geometry, and the primitive ID corresponds to the nth primitive of
   that geometry, e.g. nth triangle.

   Testing if any geometry intersects with the ray segment is done
   through the rtcOccluded functions. Initialization has to be done as
   for rtcIntersect. If some geometry got found along the ray segment,
   the geometry ID (geomID) will get set to 0. No other member of the
   ray will get updated.
  
*/

/*! Initializes the ray tracing core and passed some configuration
  string. The configuration string allows to configure implementation
  specific parameters. If this string is NULL, a default configuration
  is used. The following configuration flags are supported by the
  Embree implementation of the API:
  
  threads = num,       // sets the number of threads to use (default is to use all threads)
  verbose = num,       // sets verbosity level (default is 0)
  
*/
RTCORE_API void rtcInit(const char* cfg = NULL);

/*! Shuts down the ray tracing core. After shutdown, all scene handles
 *  are invalid, and invoking any API call except rtcInit is not
 *  allowed. The application should invoke this call before
 *  terminating. It is safe to call rtcInit again after an rtcExit
 *  call. */
RTCORE_API void rtcExit();

/*! Error codes returned by the rtcGetError function. */
enum RTCError {
  RTC_NO_ERROR = 0,          //!< No error has been recorded.
  RTC_UNKNOWN_ERROR = 1,     //!< An unknown error has occured.
  RTC_INVALID_ARGUMENT = 2,  //!< An invalid argument is specified
  RTC_INVALID_OPERATION = 3, //!< The operation is not allowed for the specified object.
  RTC_OUT_OF_MEMORY = 4      //!< There is not enough memory left to execute the command.
};

/*! Returns the value of the per-thread error flag. If an error occurs
 *  this flag is set to an error code if it stores no previous
 *  error. The rtcGetError function reads and returns the currently
 *  stored error and clears the error flag again. */
RTCORE_API RTCError rtcGetError();

/*! This function is implementation specific and only for
 *  debugging. */
RTCORE_API void rtcDebug(); // FIXME: remove

#endif
