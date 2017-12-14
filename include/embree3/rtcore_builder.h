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

#include "rtcore_scene.h"

#if defined(__cplusplus)
extern "C" {
#endif
  
/* Defines an opaque BVH type */
typedef struct RTCBVHTy* RTCBVH;

/* Input build primitives for the builder. */
struct RTCORE_ALIGN(32) RTCBuildPrimitive
{
  float lower_x, lower_y, lower_z; 
  int geomID;
  float upper_x, upper_y, upper_z;
  int primID;
};

/* Defines an opaque thread local allocator type */
typedef struct RTCThreadLocalAllocatorTy* RTCThreadLocalAllocator;

/* Callback to create a node. */
typedef void* (*RTCCreateNodeFunction) (RTCThreadLocalAllocator allocator, unsigned int numChildren, void* userPtr);

/* Callback to set the pointer to all children. */
typedef void  (*RTCSetNodeChildrenFunction) (void* nodePtr, void** children, unsigned int numChildren, void* userPtr);

/* Callback to set the bounds of all children. */
typedef void  (*RTCSetNodeBoundsFunction) (void* nodePtr, const struct RTCBounds** bounds, unsigned int numChildren, void* userPtr);

/* Callback to create a leaf node. */
typedef void* (*RTCCreateLeafFunction) (RTCThreadLocalAllocator allocator, const struct RTCBuildPrimitive* prims, size_t numPrims, void* userPtr);

/* Callback to split a build primitive. */
typedef void  (*RTCSplitPrimitiveFunction) (const struct RTCBuildPrimitive* prim, unsigned int dim, float pos, struct RTCBounds* lbounds, struct RTCBounds* rbounds, void* userPtr);

/* Input for builders */
struct RTCBuildSettings
{
  unsigned int size;
  enum RTCBuildQuality quality;
  unsigned int maxBranchingFactor;
  unsigned int maxDepth;
  unsigned int sahBlockSize;
  unsigned int minLeafSize;
  unsigned int maxLeafSize;
  float travCost;
  float intCost;
  unsigned int extraSpace;

  RTCBVH bvh;
  struct RTCBuildPrimitive* primitives;
  size_t numPrimitives;
  RTCCreateNodeFunction createNode;
  RTCSetNodeChildrenFunction setNodeChildren;
  RTCSetNodeBoundsFunction setNodeBounds;
  RTCCreateLeafFunction createLeaf;
  RTCSplitPrimitiveFunction splitPrimitive;
  RTCProgressMonitorFunction buildProgress;
  void* userPtr;
};

/* Creates default build settings.  */
RTCORE_FORCEINLINE struct RTCBuildSettings rtcDefaultBuildSettings()
{
  struct RTCBuildSettings settings;
  settings.size = sizeof(settings);
  settings.quality = RTC_BUILD_QUALITY_MEDIUM;
  settings.maxBranchingFactor = 2;
  settings.maxDepth = 32;
  settings.sahBlockSize = 1;
  settings.minLeafSize = 1;
  settings.maxLeafSize = 32;
  settings.travCost = 1.0f;
  settings.intCost = 1.0f;
  settings.extraSpace = 0;
  settings.bvh = NULL;
  settings.primitives = NULL;
  settings.numPrimitives = 0;
  settings.createNode = NULL;
  settings.setNodeChildren = NULL;
  settings.setNodeBounds = NULL;
  settings.createLeaf = NULL;
  settings.splitPrimitive = NULL;
  settings.buildProgress = NULL;
  settings.userPtr = NULL;
  return settings;
}

/* Creates a new BVH. */
RTCORE_API RTCBVH rtcNewBVH(RTCDevice device);

/* Builds a BVH. */
RTCORE_API void* rtcBuildBVH(const struct RTCBuildSettings* settings);

/* Allocates memory using the thread local allocator. */
RTCORE_API void* rtcThreadLocalAlloc(RTCThreadLocalAllocator allocator, size_t bytes, size_t align);

/* Frees internal builder state. */
RTCORE_API void rtcMakeStaticBVH(RTCBVH bvh);

/* Retains the BVH (increments reference count). */
RTCORE_API void rtcRetainBVH(RTCBVH bvh);

/* Releases the BVH (decrements reference count). */
RTCORE_API void rtcReleaseBVH(RTCBVH bvh);

#if defined(__cplusplus)
}
#endif
