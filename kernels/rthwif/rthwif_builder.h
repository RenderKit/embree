// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <sycl/sycl.hpp>

#if defined(__cplusplus)
#  define RTHWIF_API_EXTERN_C extern "C"
#else
#  define RTHWIF_API_EXTERN_C
#endif

#if defined(_WIN32)
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C __declspec(dllimport)
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C __declspec(dllexport)
#else
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C __attribute__ ((visibility ("default")))
#endif

#if defined(RTHWIF_EXPORT_API)
#  define RTHWIF_API RTHWIF_API_EXPORT
#else
#  define RTHWIF_API RTHWIF_API_IMPORT
#endif
  
typedef enum RTHWIF_GEOMETRY_FLAGS : uint8_t
{
  RTHWIF_GEOMETRY_FLAG_NONE = 0,
  RTHWIF_GEOMETRY_FLAG_OPAQUE = 0x1,
  RTHWIF_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION = 0x2
} RTHWIF_GEOMETRY_FLAGS;

typedef struct RTHWIF_FLOAT3 {
  float x, y, z;
} RTHWIF_FLOAT3;

typedef struct RTHWIF_FLOAT4 {
  float x, y, z, w;
} RTHWIF_FLOAT4;

typedef struct RTHWIF_TRANSFORM4X4 {
  RTHWIF_FLOAT4 vx, vy, vz, p;
} RTHWIF_TRANSFORM4X4;

typedef struct RTHWIF_AABB
{
  RTHWIF_FLOAT3 lower;
  RTHWIF_FLOAT3 upper;
} RTHWIF_AABB;

typedef struct RTHWIF_UINT3 {
  uint32_t v0, v1, v2;
} RTHWIF_UINT3;

typedef struct RTHWIF_UINT4 { 
  uint32_t v0, v1, v2, v3;
} RTHWIF_UINT4;

typedef enum RTHWIF_GEOMETRY_TYPE : uint8_t
{
  RTHWIF_GEOMETRY_TYPE_TRIANGLES = 0,
  RTHWIF_GEOMETRY_TYPE_QUADS = 1,
  RTHWIF_GEOMETRY_TYPE_PROCEDURALS = 2,
  RTHWIF_GEOMETRY_TYPE_INSTANCES = 3,
  RTHWIF_GEOMETRY_TYPE_INSTANCEREF = 4,
} RTHWIF_GEOMETRY_TYPE;

typedef enum RTHWIF_INSTANCE_FLAGS : uint8_t
{
  RTHWIF_INSTANCE_FLAG_NONE = 0,
  RTHWIF_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE = 0x1,
  RTHWIF_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE = 0x2,
  RTHWIF_INSTANCE_FLAG_FORCE_OPAQUE = 0x4,
  RTHWIF_INSTANCE_FLAG_FORCE_NON_OPAQUE = 0x8
} RTHWIF_INSTANCE_FLAGS;

typedef struct RTHWIF_GEOMETRY_TRIANGLES_DESC // 32 bytes
{
  RTHWIF_GEOMETRY_TYPE GeometryType;
  RTHWIF_GEOMETRY_FLAGS GeometryFlags;
  uint8_t GeometryMask;
  uint8_t reserved;
  unsigned int TriangleCount;
  unsigned int TriangleStride;
  unsigned int VertexCount;
  unsigned int VertexStride;
  RTHWIF_UINT3* IndexBuffer;
  RTHWIF_FLOAT3* VertexBuffer;
  
} RTHWIF_RAYTRACING_GEOMETRY_TRIANGLES_DESC;

typedef struct RTHWIF_GEOMETRY_QUADS_DESC // 32 bytes
{
  RTHWIF_GEOMETRY_TYPE GeometryType;
  RTHWIF_GEOMETRY_FLAGS GeometryFlags;
  uint8_t GeometryMask;
  uint8_t reserved;
  unsigned int QuadCount;
  unsigned int QuadStride;
  unsigned int VertexCount;
  unsigned int VertexStride;
  RTHWIF_UINT4* IndexBuffer;
  RTHWIF_FLOAT3* VertexBuffer;
  
} RTHWIF_RAYTRACING_GEOMETRY_QUADS_DESC;

typedef RTHWIF_AABB (*RTHWIF_GET_BOUNDS_FUNC)(const uint32_t primID, void* geomUserPtr, void* userPtr);

typedef struct RTHWIF_GEOMETRY_AABBS_DESC // 16 bytes
{
  RTHWIF_GEOMETRY_TYPE GeometryType;
  RTHWIF_GEOMETRY_FLAGS GeometryFlags;
  uint8_t GeometryMask;
  uint8_t reserved;
  unsigned int AABBCount;
  //RTHWIF_AABB* AABBs;
  RTHWIF_GET_BOUNDS_FUNC AABBs;
  void* userPtr;
} RTHWIF_GEOMETRY_AABBS_DESC;

typedef struct RTHWIF_GEOMETRY_INSTANCE_DESC // 80 bytes
{
  RTHWIF_GEOMETRY_TYPE GeometryType;
  RTHWIF_INSTANCE_FLAGS InstanceFlags;
  uint8_t GeometryMask;
  uint8_t reserved0;
  unsigned int InstanceID;
  RTHWIF_TRANSFORM4X4 Transform;
  void* Accel;
  // FIXME: add bounds
  
} RTHWIF_GEOMETRY_INSTANCE_DESC;

typedef struct RTHWIF_GEOMETRY_INSTANCEREF_DESC // 24 bytes
{
  RTHWIF_GEOMETRY_TYPE GeometryType;
  RTHWIF_INSTANCE_FLAGS InstanceFlags;
  uint8_t GeometryMask;
  uint8_t reserved0;
  unsigned int InstanceID;
  RTHWIF_TRANSFORM4X4* Transform;
  void* Accel;
  // FIXME: add bounds?
  
} RTHWIF_GEOMETRY_INSTANCEREF_DESC;

typedef union RTHWIF_GEOMETRY_DESC
{
  RTHWIF_GEOMETRY_TYPE GeometryType;
  RTHWIF_GEOMETRY_TRIANGLES_DESC Triangles;
  RTHWIF_GEOMETRY_QUADS_DESC Quads;
  RTHWIF_GEOMETRY_AABBS_DESC AABBs;
  RTHWIF_GEOMETRY_INSTANCE_DESC Instances;
  RTHWIF_GEOMETRY_INSTANCEREF_DESC InstanceRef;
  
} RTHWIF_GEOMETRY_DESC;


typedef enum RTHWIF_FEATURES {
  RTHWIF_FEATURES_NONE = 0,
  RTHWIF_FEATURES_TRIANGLES = 1 << 0,
  RTHWIF_GEOMETRY_QUADS = 1 << 1,
  RTHWIF_GEOMETRY_PROCEDURALS = 1 << 2,
  RTHWIF_GEOMETRY_INSTANCE = 1 << 3,
} RTHWIF_FEATURES;

RTHWIF_API void rthwifInit();
RTHWIF_API void rthwifExit();

RTHWIF_API RTHWIF_FEATURES rthwifGetSupportedFeatures(sycl::device device);

typedef enum RTHWIF_ERROR
{
  RTHWIF_ERROR_NONE = 0,
  RTHWIF_ERROR_OUT_OF_MEMORY = 0x1,  // build ran out of memory, app can re-try with more memory
  RTHWIF_ERROR_OTHER = 0x2
} RTHWIF_ERROR;

typedef enum RTHWIF_BUILD_QUALITY
{
  RTHWIF_BUILD_QUALITY_LOW    = 0,
  RTHWIF_BUILD_QUALITY_MEDIUM = 1,
  RTHWIF_BUILD_QUALITY_HIGH   = 2,
  RTHWIF_BUILD_QUALITY_REFIT  = 3,
} RTHWIF_BUILD_QUALITY;

typedef enum RTHWIF_BUILD_FLAGS : uint64_t
{
  RTHWIF_BUILD_FLAG_NONE                    = 0,
  RTHWIF_BUILD_FLAG_DYNAMIC                 = (1 << 0),
  RTHWIF_BUILD_FLAG_COMPACT                 = (1 << 1),
  RTHWIF_BUILD_FLAG_ROBUST                  = (1 << 2),
} RTHWIF_BUILD_FLAGS;

typedef struct RTHWIF_ACCEL_REF
{
  void* Accel;
  RTHWIF_AABB bounds;
} RTHWIF_ACCEL_REF;

#define RTHWIF_BVH_ALIGNMENT 128

typedef struct RTHWIF_BUILD_ACCEL_ARGS
{
  size_t bytes;
  sycl::device* device;
  void* embree_device; // FIXME: remove
  const RTHWIF_GEOMETRY_DESC** geometries;
  size_t numGeometries;
  void* accel;  // has to be 128 bytes aligned
  size_t numBytes;
  // FIXME: add scratch space buffer
  RTHWIF_BUILD_QUALITY quality;
  RTHWIF_BUILD_FLAGS flags;
  RTHWIF_AABB* bounds;
  void* userPtr;
  RTHWIF_ACCEL_REF AddAccel;   // add link to that acceleration structure
  // FIXME: return used bytes
  
} RTHWIF_BUILD_ACCEL_ARGS;

typedef struct RTHWIF_ACCEL_SIZE
{
  size_t bytes;
  size_t expectedBytes;
  size_t worstCaseBytes;
} RTHWIF_ACCEL_SIZE;


RTHWIF_API RTHWIF_ERROR rthwifGetAccelSize(const RTHWIF_BUILD_ACCEL_ARGS& args, RTHWIF_ACCEL_SIZE& size_o);

RTHWIF_API RTHWIF_ERROR rthwifBuildAccel(const RTHWIF_BUILD_ACCEL_ARGS& args);
