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

#if defined(__cplusplus)
extern "C" {
#endif
  
/*! \ingroup embree_kernel_api */
/*! \{ */

/*! Type of bounding function. */
typedef void (*RTCBoundsFunc)(void* userPtr,         /*!< pointer to user data */
                               void* geomUserPtr,     /*!< pointer to geometry user data */
                               unsigned int item,           /*!< item to calculate bounds for */
                               unsigned int time,           /*!< time to calculate bounds for */
                               struct RTCBounds* bounds_o    /*!< returns calculated bounds */);

/*! Type of intersect function pointer for ray packets of size N. */
typedef void (*RTCIntersectFuncN)(const int* valid,                        /*!< pointer to valid mask */
                                  void* ptr,                               /*!< pointer to geometry user data */
                                  const struct RTCIntersectContext* context,   /*!< intersection context as passed to rtcIntersect/rtcOccluded */
                                  struct RTCRayN* rays,                           /*!< ray packet to intersect */
                                  unsigned int N,                                /*!< number of rays in packet */
                                  unsigned int item                              /*!< item to intersect */);

/*! Type of occlusion function pointer for ray packets of size N. */
typedef void (*RTCOccludedFuncN) (const int* valid,                      /*! pointer to valid mask */
                                  void* ptr,                             /*!< pointer to user data */
                                  const struct RTCIntersectContext* context, /*!< intersection context as passed to rtcIntersect/rtcOccluded */
                                  struct RTCRayN* rays,                            /*!< Ray packet to test occlusion for. */
                                  unsigned int N,                              /*!< number of rays in packet */
                                  unsigned int item                            /*!< item to test for occlusion */);

/*! Creates a new user geometry object. This feature makes it possible
 *  to add arbitrary types of geometry to the scene by providing
 *  appropiate bounding, intersect and occluded functions. A user
 *  geometry object is a set of user geometries. As the rtcIntersect
 *  and rtcOccluded functions support different ray packet sizes, the
 *  user also has to provide different versions of intersect and
 *  occluded function pointers for these packet sizes. However, the
 *  ray packet size of the called function pointer always matches the
 *  packet size of the originally invoked rtcIntersect and rtcOccluded
 *  functions. A user data pointer, that points to a user specified
 *  representation of the geometry, is passed to each intersect and
 *  occluded function invokation, as well as the index of the geometry
 *  of the set to intersect. */
RTCORE_API RTCGeometry rtcNewUserGeometry (RTCDevice device,
                                           enum RTCGeometryFlags gflags, //!< geometry flags
                                           unsigned int numGeometries,    /*!< the number of geometries contained in the set */
                                           unsigned int numTimeSteps  /*!< number of motion blur time steps */
  );

/*! Sets the bounding function to calculate bounding boxes of the user
 *  geometry items when building spatial index structures. The
 *  calculated bounding box have to be conservative and should be
 *  tight. */
RTCORE_API void rtcSetBoundsFunction (RTCGeometry hgeometry, RTCBoundsFunc bounds, void* userPtr);

/*! Set intersect function for ray packets of size N. The rtcIntersectN function
 *  will call the passed function for intersecting the user
 *  geometry. */
RTCORE_API void rtcSetIntersectFunction(RTCGeometry hgeometry, RTCIntersectFuncN intersect);

/*! Set occlusion function for ray packets of size N. The rtcOccludedN function
 *  will call the passed function for intersecting the user
 *  geometry. */
RTCORE_API void rtcSetOccludedFunction(RTCGeometry hgeometry, RTCOccludedFuncN occluded);

#if defined(__cplusplus)
}
#endif
  
/*! @} */
