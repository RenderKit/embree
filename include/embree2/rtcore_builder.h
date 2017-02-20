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

#ifndef __RTCORE_BUILDER_H__
#define __RTCORE_BUILDER_H__

#include "rtcore.h"

/*! \brief Defines an opaque BVH type */
typedef struct __RTCBvh {}* RTCBVH;

/*! \brief Defines an opaque thread local allocator type */
typedef struct __RTCThreadLocalAllocator {}* RTCThreadLocalAllocator;

/*! Creates a new BVH. */
RTCORE_API RTCBVH rtcNewBVH(RTCDevice device);

/*! Allocates memory using the thread local allocator. */
RTCORE_API void* rtcThreadLocalAlloc(RTCThreadLocalAllocator allocator, size_t bytes, size_t align);

/*! Makes the BVH static. No further rtcBVHBuild can be called anymore on the BVH. */
RTCORE_API void rtcMakeStaticBVH(RTCBVH bvh);

/*! Deletes the BVH. */
RTCORE_API void rtcDeleteBVH(RTCBVH bvh);

/*! Settings for builders */
struct RTCBuildSettings
{
  unsigned size;             //!< Size of this structure in bytes. Makes future extension easier.
  unsigned branchingFactor;  //!< branching factor of BVH to build
  unsigned maxDepth;         //!< maximal depth of BVH to build
  unsigned blockSize;        //!< blocksize for SAH heuristic
  unsigned minLeafSize;      //!< minimal size of a leaf
  unsigned maxLeafSize;      //!< maximal size of a leaf
  float travCost;            //!< estimated cost of one traversal step
  float intCost;             //!< estimated cost of one primitive intersection
};

/*! Creates default build settings.  */
inline RTCBuildSettings rtcDefaultBuildSettings()
{
  RTCBuildSettings settings;
  settings.size = sizeof(settings);
  settings.branchingFactor = 2;
  settings.maxDepth = 32;
  settings.blockSize = 1;
  settings.minLeafSize = 1;
  settings.maxLeafSize = 32;
  settings.travCost = 1.0f;
  settings.intCost = 1.0f;
  return settings;
}

/*! Input primitives for the builder. Stores primitive bounds and
 *  ID. */
struct RTCORE_ALIGN(32) RTCBuildPrimitive
{
  float lower_x, lower_y, lower_z;  //!< lower bounds in x/y/z
  int geomID;                       //!< first ID
  float upper_x, upper_y, upper_z;  //!< upper bounds in x/y/z
  int primID;                       //!< second ID
};

/*! Callback to create a node. */
typedef void* (*RTCCreateNodeFunc) (RTCThreadLocalAllocator allocator, size_t numChildren, void* userPtr);

/*! Callback to set the pointer to the i'th child. */
typedef void  (*RTCSetNodeChildFunc) (void* nodePtr, size_t i, void* childPtr, void* userPtr);

/*! Callback to set the bounds of the i'th child. */
typedef void  (*RTCSetNodeBoundsFunc) (void* nodePtr, size_t i, RTCBounds& bounds, void* userPtr);

/*! Callback to create a leaf node. */
typedef void* (*RTCCreateLeafFunc) (RTCThreadLocalAllocator allocator, const RTCBuildPrimitive* prims, size_t numPrims, void* userPtr);

/*! Callback to provide build progress. */
typedef void (*RTCBuildProgressFunc) (size_t dn, void* userPtr);

/*! Standard BVH builder (internally using binning and SAH heuristic). */
RTCORE_API void* rtcBuildBVH(RTCBVH bvh,                                     //!< BVH to build
                             const RTCBuildSettings& settings,               //!< settings for BVH builder
                             RTCBuildPrimitive* prims,                       //!< list of input primitives
                             size_t numPrims,                                //!< number of input primitives
                             void* userPtr,                                  //!< user pointer passed to callback functions
                             RTCCreateNodeFunc createNode,                   //!< creates a node
                             RTCSetNodeChildFunc setNodeChild,               //!< sets pointer to a child
                             RTCSetNodeBoundsFunc setNodeBounds,             //!< sets bound of a child
                             RTCCreateLeafFunc createLeaf,                   //!< creates a leaf
                             RTCBuildProgressFunc buildProgress              //!< used to report build progress
  ); 

/*! Faster builder producing lower quality trees (internally operating with morton codes). */
RTCORE_API void* rtcBuildBVHFast(RTCBVH bvh,                                     //!< BVH to build
                                 const RTCBuildSettings& settings,               //!< settings for BVH builder
                                 RTCBuildPrimitive* prims,                       //!< list of input primitives
                                 size_t numPrims,                                //!< number of input primitives
                                 void* userPtr,                                  //!< user pointer passed to createThreadLocal callback
                                 RTCCreateNodeFunc createNode,                   //!< creates a node
                                 RTCSetNodeChildFunc setNodeChild,               //!< sets pointer to a child
                                 RTCSetNodeBoundsFunc setNodeBounds,             //!< sets bound of a child
                                 RTCCreateLeafFunc createLeaf,                   //!< creates a leaf
                                 RTCBuildProgressFunc buildProgress              //!< used to report build progress
  );

#endif
