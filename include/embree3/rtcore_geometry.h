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

#include "rtcore_buffer.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Geometry type */
enum RTCGeometryType
{
  RTC_GEOMETRY_TYPE_TRIANGLE,
  RTC_GEOMETRY_TYPE_QUAD,
  RTC_GEOMETRY_TYPE_SUBDIVISION,
  RTC_GEOMETRY_TYPE_LINEAR_CURVE,
  RTC_GEOMETRY_TYPE_BEZIER_CURVE,
  RTC_GEOMETRY_TYPE_BSPLINE_CURVE,
  RTC_GEOMETRY_TYPE_USER,
  RTC_GEOMETRY_TYPE_INSTANCE
};

/* Geometry subtype */
enum RTCGeometrySubtype
{
  RTC_GEOMETRY_SUBTYPE_DEFAULT = 0,
  RTC_GEOMETRY_SUBTYPE_ROUND = 1,
  RTC_GEOMETRY_SUBTYPE_FLAT = 2,
};

/* Interpolation mode for subdivision surfaces. */
enum RTCSubdivisionMode
{
  RTC_SUBDIVISION_MODE_NO_BOUNDARY     = 0,
  RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY = 1,
  RTC_SUBDIVISION_MODE_PIN_CORNERS     = 2,
  RTC_SUBDIVISION_MODE_PIN_BOUNDARY    = 3,
  RTC_SUBDIVISION_MODE_PIN_ALL         = 4,
};

/* Curve segment flags. */
enum RTCCurveFlags
{
  RTC_CURVE_FLAG_NEIGHBOR_LEFT  = (1 << 0), 
  RTC_CURVE_FLAG_NEIGHBOR_RIGHT = (1 << 1) 
};

/* Arguments for RTCBoundsFunction */
struct RTCBoundsFunctionArguments
{
  void* geomUserPtr;
  unsigned int primID;
  unsigned int timeStep;
  struct RTCBounds* bounds_o;
};
  
/* Type of bounding function */
typedef void (*RTCBoundsFunction)(const struct RTCBoundsFunctionArguments* const args);

/* Arguments for RTCIntersectFunctionN */
struct RTCIntersectFunctionNArguments
{
  int* valid;
  void* geomUserPtr;
  unsigned int primID;
  struct RTCIntersectContext* context;
  struct RTCRayN* ray;
  unsigned int N;
};

/* Type of intersect callback function */
typedef void (*RTCIntersectFunctionN)(const struct RTCIntersectFunctionNArguments* const args);

/* Arguments for RTCOccludedFunctionN */
struct RTCOccludedFunctionNArguments
{
  int* valid;
  void* geomUserPtr;
  unsigned int primID;
  struct RTCIntersectContext* context;
  struct RTCRayN* ray;
  unsigned int N;
};
  
/* Type of occlusion callback function pointer. */
typedef void (*RTCOccludedFunctionN)(const struct RTCOccludedFunctionNArguments* const args);

/* Reports intersection from intersect callback function. */
RTCORE_API void rtcFilterIntersection(const struct RTCIntersectFunctionNArguments* const args, const struct RTCFilterFunctionNArguments* filterArgs);

/* Reports intersection from occluded callback function. */
RTCORE_API void rtcFilterOcclusion(const struct RTCOccludedFunctionNArguments* const args, const struct RTCFilterFunctionNArguments* filterArgs);

/* Defines an opaque geometry type */
typedef struct RTCGeometryTy* RTCGeometry;

/* Arguments for RTCDisplacementFunction callback */
struct RTCDisplacementFunctionNArguments
{
  void* geomUserPtr;
  RTCGeometry geometry;
  unsigned int primID;
  unsigned int timeStep;
  const float* u;
  const float* v;
  const float* Ng_x;
  const float* Ng_y;
  const float* Ng_z;
  float* P_x;
  float* P_y;
  float* P_z;
  unsigned int N;
};
 
/* Type of displacement mapping callback function. */
typedef void (*RTCDisplacementFunction)(const struct RTCDisplacementFunctionNArguments* const args);

/* Creates a new geometry of specified type and subtype. */
RTCORE_API RTCGeometry rtcNewGeometry(RTCDevice device, enum RTCGeometryType type, enum RTCGeometrySubtype subtype);

/* Sets the bounding callback function to calculate bounding boxes for user primitives. */
RTCORE_API void rtcSetGeometryBoundsFunction(RTCGeometry geometry, RTCBoundsFunction bounds, void* userPtr);

/* Set intersect callback function for user geometries. */
RTCORE_API void rtcSetGeometryIntersectFunction(RTCGeometry geometry, RTCIntersectFunctionN intersect);

/* Set occlusion callback function for user geometries. */
RTCORE_API void rtcSetGeometryOccludedFunction(RTCGeometry geometry, RTCOccludedFunctionN occluded);

/* Sets instanced scene of instance geometry. */
RTCORE_API void rtcSetGeometryInstancedScene(RTCGeometry geometry, RTCScene scene);

/* Sets transformation of the instance for specified timestep */
RTCORE_API void rtcSetGeometryTransform(RTCGeometry geometry, enum RTCFormat format, const void* xfm, unsigned int timeStep);

/* Returns transformation of the instance for specified time. */
RTCORE_API void rtcGetGeometryTransform(RTCGeometry geometry, float time, enum RTCFormat format, void* xfm);

/* Sets the number of primitives for user geometry. */
RTCORE_API void rtcSetGeometryUserPrimitiveCount(RTCGeometry geometry, unsigned int userPrimCount);

/* Sets the number of time steps. */
RTCORE_API void rtcSetGeometryTimeStepCount(RTCGeometry geometry, unsigned int timeStepCount);

/* Sets the number of vertex attributes. */
RTCORE_API void rtcSetGeometryVertexAttributeCount(RTCGeometry geometry, unsigned int vertexAttributeCount);

/* Sets the number of topologies for subdivision surfaces. */
RTCORE_API void rtcSetGeometryTopologyCount(RTCGeometry geometry, unsigned int topologyCount);
 
/* Sets a uniform tessellation rate */
RTCORE_API void rtcSetGeometryTessellationRate(RTCGeometry geometry, float tessellationRate);

/* Sets the build quality of the geometry. */
RTCORE_API void rtcSetGeometryBuildQuality(RTCGeometry geometry, enum RTCBuildQuality quality);

/* Sets the ray mask of the geometry. */
RTCORE_API void rtcSetGeometryMask(RTCGeometry geometry, unsigned int mask);

/* Sets subdivision interpolation mode. */
RTCORE_API void rtcSetGeometrySubdivisionMode(RTCGeometry geometry, unsigned int topologyID, enum RTCSubdivisionMode mode);

/* Binds a vertex attribute to some topology. */
RTCORE_API void rtcSetGeometryVertexAttributeTopology(RTCGeometry geometry, unsigned int vertexAttributeID, unsigned int topologyID);

/* Sets geometry buffer. */
RTCORE_API void rtcSetGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format,
                                     RTCBuffer buffer, size_t byteOffset, size_t byteStride, unsigned int itemCount);

/* Sets shared geometry buffer. */
RTCORE_API void rtcSetSharedGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format,
                                           const void* ptr, size_t byteOffset, size_t byteStride, unsigned int itemCount);

/* Sets new geometry buffer. */
RTCORE_API void* rtcSetNewGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format,
                                         size_t byteStride, unsigned int itemCount);

/* Returns a pointer to the buffer data. */
RTCORE_API void* rtcGetGeometryBufferData(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot);

/* Enable geometry. */
RTCORE_API void rtcEnableGeometry(RTCGeometry geometry);

/* Update geometry buffer. */
RTCORE_API void rtcUpdateGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot);

/* Disable geometry. */
RTCORE_API void rtcDisableGeometry(RTCGeometry geometry);

/* Sets the displacement callback function. */
RTCORE_API void rtcSetGeometryDisplacementFunction(RTCGeometry geometry, RTCDisplacementFunction func);

/* Sets the intersection filter callback function. */
RTCORE_API void rtcSetGeometryIntersectFilterFunction(RTCGeometry geometry, RTCFilterFunctionN func);

/* Sets the occlusion filter callback function. */
RTCORE_API void rtcSetGeometryOccludedFilterFunction(RTCGeometry geometry, RTCFilterFunctionN func);

/* Sets user defined data pointer of geometry. */
RTCORE_API void rtcSetGeometryUserData(RTCGeometry geometry, void* ptr);

/* Gets user defined data pointer of geometry. */
RTCORE_API void* rtcGetGeometryUserData(RTCGeometry geometry);

struct RTCInterpolateArguments
{
  RTCGeometry geometry;
  unsigned int primID;
  float u;
  float v;
  enum RTCBufferType bufferType;
  unsigned int bufferSlot;
  float* P;
  float* dPdu;
  float* dPdv;
  float* ddPdudu;
  float* ddPdvdv;
  float* ddPdudv;
  unsigned int valueCount;
};

/* Interpolates vertex data to some u/v location. */
RTCORE_API void rtcInterpolate(const struct RTCInterpolateArguments* const args);

RTCORE_FORCEINLINE void rtcInterpolate0(RTCGeometry geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot, float* P, unsigned int valueCount)
{
  struct RTCInterpolateArguments args;
  args.geometry = geometry;
  args.primID = primID;
  args.u = u;
  args.v = v;
  args.bufferType = bufferType;
  args.bufferSlot = bufferSlot;
  args.P = P;
  args.dPdu = NULL;
  args.dPdv = NULL;
  args.ddPdudu = NULL;
  args.ddPdvdv = NULL;
  args.ddPdudv = NULL;
  args.valueCount = valueCount;
  rtcInterpolate(&args);
}

RTCORE_FORCEINLINE void rtcInterpolate1(RTCGeometry geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot,
                                         float* P, float* dPdu, float* dPdv, unsigned int valueCount)
{
  struct RTCInterpolateArguments args;
  args.geometry = geometry;
  args.primID = primID;
  args.u = u;
  args.v = v;
  args.bufferType = bufferType;
  args.bufferSlot = bufferSlot;
  args.P = P;
  args.dPdu = dPdu;
  args.dPdv = dPdv;
  args.ddPdudu = NULL;
  args.ddPdvdv = NULL;
  args.ddPdudv = NULL;
  args.valueCount = valueCount;
  rtcInterpolate(&args);
}

RTCORE_FORCEINLINE void rtcInterpolate2(RTCGeometry geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot,
                                         float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, unsigned int valueCount)
{
  struct RTCInterpolateArguments args;
  args.geometry = geometry;
  args.primID = primID;
  args.u = u;
  args.v = v;
  args.bufferType = bufferType;
  args.bufferSlot = bufferSlot;
  args.P = P;
  args.dPdu = dPdu;
  args.dPdv = dPdv;
  args.ddPdudu = ddPdudu;
  args.ddPdvdv = ddPdvdv;
  args.ddPdudv = ddPdudv;
  args.valueCount = valueCount;
  rtcInterpolate(&args);
}

struct RTCInterpolateNArguments
{
  RTCGeometry geometry;
  const void* valid;
  const unsigned int* primIDs;
  const float* u;
  const float* v;
  unsigned int N;
  enum RTCBufferType bufferType;
  unsigned int bufferSlot;
  float* P;
  float* dPdu;
  float* dPdv;
  float* ddPdudu;
  float* ddPdvdv;
  float* ddPdudv;
  unsigned int valueCount;
};

/* Interpolates vertex data to an array of u/v locations. */
RTCORE_API void rtcInterpolateN(const struct RTCInterpolateNArguments* const args);

/* Commits the geometry. */
RTCORE_API void rtcCommitGeometry(RTCGeometry geometry);

/* Attaches the geometry to some scene. */
RTCORE_API unsigned int rtcAttachGeometry(RTCScene scene, RTCGeometry geometry);

/* Attaches the geometry to some scene using the specified geometry ID. */
RTCORE_API void rtcAttachGeometryByID(RTCScene scene, RTCGeometry geometry, unsigned int geomID);

/* Detaches the geometry from the scene. */
RTCORE_API void rtcDetachGeometry(RTCScene scene, unsigned int geomID);

/* Retains the geometry (increments reference count). */
RTCORE_API void rtcRetainGeometry(RTCGeometry geometry);

/* Releases the geometry (decrements reference count) */
RTCORE_API void rtcReleaseGeometry(RTCGeometry geometry);

/* Gets geometry handle from scene. */
RTCORE_API RTCGeometry rtcGetGeometry(RTCScene scene, unsigned int geomID);

#if defined(__cplusplus)
}
#endif

