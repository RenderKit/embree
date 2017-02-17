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

/*! \brief Defines an opaque allocator type */
typedef struct __RTCAllocator {}* RTCAllocator;

/*! \brief Defines an opaque thread local allocator type */
typedef struct __RTCThreadLocalAllocator {}* RTCThreadLocalAllocator;

/*! creates a new allocator */
RTCORE_API RTCAllocator rtcNewAllocator(RTCDevice device, size_t estimatedBytes);

/*! get thread local allocator */
RTCORE_API RTCThreadLocalAllocator rtcGetThreadLocalAllocator(RTCAllocator allocator);

/*! allocates memory */
RTCORE_API void* rtcMalloc(RTCThreadLocalAllocator allocator, size_t bytes, size_t align);

/*! Deletes all thread local allocators again. This function should be
 *  called after the build finished. */
RTCORE_API void rtcDeleteThreadLocalAllocators(RTCAllocator allocator);

/*! frees all data stored inside the allocator */
RTCORE_API void rtcClearAllocator(RTCAllocator allocator);

/*! Resets the allocator. No data is freed, but is re-used in new allocations. */
RTCORE_API void rtcResetAllocator(RTCAllocator allocator);

/*! deletes the allocator again */
RTCORE_API void rtcDeleteAllocator(RTCAllocator allocator);

/*! Settings for builders */
struct RTCBuildSettings
{
  unsigned size;             //!< size of this structure in bytes  
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
struct RTCORE_ALIGN(32) RTCPrimRef
{
  float lower_x, lower_y, lower_z;  //!< lower bounds in x/y/z
  int geomID;                       //!< first ID
  float upper_x, upper_y, upper_z;  //!< upper bounds in x/y/z
  int primID;                       //!< second ID
};

/*! Function that should return a pointer to thread local data. When a
  node or leaf creation callback is invoked, the appropiate thread
  local pointer returned by invoking this callback is passed as
  agument. This is only an optimization as the thread local pointer
  could also be obtained inside the create node and leaf callbacks
  directly. */
typedef void* (*RTCCreateThreadLocalFunc) (void* userPtr);

/*! Callback that should create  a node for the number of children. */
typedef void* (*RTCCreateNodeFunc) (void* threadLocalPtr, size_t numChildren);

/*! Callback that should set the pointer to the i'th child. */
typedef void  (*RTCSetNodeChildFunc) (void* nodePtr, size_t i, void* childPtr);

/*! Callback that should set the bounds of the i'th child. */
typedef void  (*RTCSetNodeBoundsFunc) (void* nodePtr, size_t i, RTCBounds& bounds);

/*! Callback that should create a leaf. */
typedef void* (*RTCCreateLeafFunc) (void* threadLocalPtr, const RTCPrimRef* prims, size_t numPrims);

/*! Callback that provides build progress */
typedef void (*RTCBuildProgressFunc) (void* userPtr, size_t dn);

/*! SAH based BVH builder. */
RTCORE_API void* rtcBVHBuildSAH(RTCDevice device,                               //!< embree device
                                const RTCBuildSettings& settings,               //!< settings for BVH builder
                                RTCPrimRef* prims,                              //!< list of input primitives
                                size_t numPrims,                                //!< number of input primitives
                                void* userPtr,                                  //!< user pointer passed to createThreadLocal callback
                                RTCCreateThreadLocalFunc createThreadLocal,     //!< returns thread local data pointer passed to create functions
                                RTCCreateNodeFunc createNode,                   //!< creates a node
                                RTCSetNodeChildFunc setNodeChild,               //!< sets pointer to a child
                                RTCSetNodeBoundsFunc setNodeBounds,             //!< sets bound of a child
                                RTCCreateLeafFunc createLeaf,                   //!< creates a leaf
                                RTCBuildProgressFunc buildProgress              //!< used to report build progress
  ); 

/*! Faster builder working with Morton Codes.. */
RTCORE_API void* rtcBVHBuildMorton(RTCDevice device,                               //!< embree device
                                   const RTCBuildSettings& settings,               //!< settings for BVH builder
                                   RTCPrimRef* prims,                              //!< list of input primitives
                                   size_t numPrims,                                //!< number of input primitives
                                   void* userPtr,                                  //!< user pointer passed to createThreadLocal callback
                                   RTCCreateThreadLocalFunc createThreadLocal,     //!< returns thread local data pointer passed to create functions
                                   RTCCreateNodeFunc createNode,                   //!< creates a node
                                   RTCSetNodeChildFunc setNodeChild,               //!< sets pointer to a child
                                   RTCSetNodeBoundsFunc setNodeBounds,             //!< sets bound of a child
                                   RTCCreateLeafFunc createLeaf,                   //!< creates a leaf
                                   RTCBuildProgressFunc buildProgress              //!< used to report build progress
  );

#endif
