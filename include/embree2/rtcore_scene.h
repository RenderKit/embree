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

#ifndef __RTCORE_SCENE_H__
#define __RTCORE_SCENE_H__

/*! forward declarations for ray structures */
struct RTCRay;
struct RTCRay4;
struct RTCRay8;
struct RTCRay16;

/*! geometry flags */
enum RTCFlags 
{
  /* dynamic type flags */
  RTC_STATIC     = (0 << 0),    //!< specifies static geometry that will not change
  RTC_DEFORMABLE = (1 << 0),    //!< specifies dynamic geometry with deformable motion (BVH refit possible)
  RTC_DYNAMIC    = (2 << 0),    //!< specifies dynamic geometry with arbitrary motion (BVH refit not possible)

  /* acceleration structure flags */
  RTC_COMPACT    = (1 << 8),    //!< use memory conservative data structures
  RTC_COHERENT   = (1 << 9),    //!< optimize data structures for coherent rays (disabled by default)
  RTC_INCOHERENT = (1 << 10),    //!< optimize data structures for in-coherent rays (disabled by default)
  RTC_HIGH_QUALITY = (1 << 11),  //!< create higher quality data structures (disabled by default)

  /* traversal algorithm flags */
  RTC_ROBUST     = (1 << 16)     //!< use more robust traversal algorithms
};

/*! enabled algorithm flags */
enum RTCAlgorithmFlags 
{
  RTC_INTERSECT1 = (1 << 0),    //!< enables the rtcIntersect1 and rtcOccluded1 functions for this scene
  RTC_INTERSECT4 = (1 << 1),    //!< enables the rtcIntersect4 and rtcOccluded4 functions for this scene
  RTC_INTERSECT8 = (1 << 2),    //!< enables the rtcIntersect8 and rtcOccluded8 functions for this scene
  RTC_INTERSECT16 = (1 << 3),   //!< enables the rtcIntersect16 and rtcOccluded16 functions for this scene
};

/*! Scene type */
typedef struct __RTCScene {}* RTCScene;

/*! Creates a new scene. */
RTCORE_API RTCScene rtcNewScene (RTCFlags flags, RTCAlgorithmFlags aflags); // FIXME: rtcNewStaticScene and rtcNewDynamicScene

/*! Commits the geometry of the scene. After initializing or modifying
 *  geometries, this function has to get called before tracing
 *  rays. */
RTCORE_API void rtcCommit (RTCScene scene);

/*! Intersects a single ray with the scene. */
RTCORE_API void rtcIntersect (RTCScene scene, RTCRay& ray);

/*! Intersects a packet of 4 rays with the scene. */
RTCORE_API void rtcIntersect4 (const void* valid, RTCScene scene, RTCRay4& ray);

/*! Intersects a packet of 8 rays with the scene. */
RTCORE_API void rtcIntersect8 (const void* valid, RTCScene scene, RTCRay8& ray);

/*! Intersects a packet of 16 rays with the scene. */
RTCORE_API void rtcIntersect16 (const void* valid, RTCScene scene, RTCRay16& ray);

/*! Tests if a single ray is occluded by the scene. */
RTCORE_API void rtcOccluded (RTCScene scene, RTCRay& ray);

/*! Tests if a packet of 4 rays is occluded by the scene. */
RTCORE_API void rtcOccluded4 (const void* valid, RTCScene scene, RTCRay4& ray);

/*! Tests if a packet of 8 rays is occluded by the scene. */
RTCORE_API void rtcOccluded8 (const void* valid, RTCScene scene, RTCRay8& ray);

/*! Tests if a packet of 16 rays is occluded by the scene. */
RTCORE_API void rtcOccluded16 (const void* valid, RTCScene scene, RTCRay16& ray);

/*! Deletes the scene. All contained geometry get also destroyed. */
RTCORE_API void rtcDeleteScene (RTCScene scene);

#endif
