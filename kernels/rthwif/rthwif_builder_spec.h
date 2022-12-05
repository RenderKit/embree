// Copyright 2009-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <level_zero/ze_api.h>

/* Required alignment of acceleration structure buffers. */
#define ZE_RAYTRACING_ACCELERATION_STRUCTURE_ALIGNMENT 128

/* Geometry flags supported (identical to DXR spec). */
typedef enum ze_raytracing_geometry_flags_t : uint8_t
{
  ZE_RAYTRACING_GEOMETRY_FLAG_NONE = 0,
  ZE_RAYTRACING_GEOMETRY_FLAG_OPAQUE = 0x1,                           // Opaque geometries do not invoke an anyhit shader
  ZE_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION = 0x2    // Guarantees single anyhit shader invokation for primitives.
} ze_raytracing_geometry_flags_t;


/* A 3-component short vector type. */
typedef struct ze_raytracing_float3_t {
  float x, y, z;
} ze_raytracing_float3_t;


/* A 3x4 affine transformation with column vectors vx, vy, vz, and p
 * transforming a point (x,y,z) to x*vx + y*vy + z*vz + p. */
typedef struct ze_raytracing_transform_float3x4_column_major_t {
  float vx_x, vx_y, vx_z; // column 0
  float vy_x, vy_y, vy_z; // column 1
  float vz_x, vz_y, vz_z; // column 2
  float  p_x,  p_y,  p_z; // column 3
} ze_raytracing_transform_float3x4_column_major_t;


/* Same as ze_raytracing_transform_float3x4_column_major_t, with ignored 4th
 * component of column vectors. */
typedef struct ze_raytracing_transform_float4x4_column_major_t {
  float vx_x, vx_y, vx_z, pad0; // column 0
  float vy_x, vy_y, vy_z, pad1; // column 1
  float vz_x, vz_y, vz_z, pad2; // column 2
  float  p_x,  p_y,  p_z, pad3; // column 3
} ze_raytracing_transform_float4x4_column_major_t;


/* Same as ze_raytracing_transform_float3x4_column_major_t, but using row major
 * layout. */
typedef struct ze_raytracing_transform_float3x4_row_major_t {
  float vx_x, vy_x, vz_x, p_x; // row 0
  float vx_y, vy_y, vz_y, p_y; // row 1
  float vx_z, vy_z, vz_z, p_z; // row 2
} ze_raytracing_transform_float3x4_row_major_t;


/* An axis aligned bounding box with lower and upper bounds in each
 * dimension. */
typedef struct ze_raytracing_aabb_t
{
  ze_raytracing_float3_t lower; // lower bounds
  ze_raytracing_float3_t upper; // upper bounds
} ze_raytracing_aabb_t;


/* A triangle represented using 3 vertex indices. */
typedef struct ze_raytracing_triangle_indices_t {
  uint32_t v0, v1, v2;      // 3 triangle indices pointing into vertex array
} ze_raytracing_triangle_indices_t;


/* A quad (triangle pair) represented using 4 vertex indices. */
typedef struct ze_raytracing_quad_indices_t { 
  uint32_t v0, v1, v2, v3; // 4 quad indices pointing into vertex array
} ze_raytracing_quad_indices_t;


/* The types of geometries supported. */
typedef enum ze_raytracing_geometry_type_t : uint8_t
{
  ZE_RAYTRACING_GEOMETRY_TYPE_TRIANGLES = 0,   // triangle mesh geometry type
  ZE_RAYTRACING_GEOMETRY_TYPE_QUADS = 1,       // quad mesh geometry type
  ZE_RAYTRACING_GEOMETRY_TYPE_AABBS_FPTR = 2,  // procedural geometry with AABB bounds per primitive
  ZE_RAYTRACING_GEOMETRY_TYPE_INSTANCE = 3,    // instance geometry
} ze_raytracing_geometry_type_t;

/* The format of transformations supported. */
typedef enum ze_raytracing_transform_format_t : uint8_t
{
  ZE_RAYTRACING_TRANSFORM_FORMAT_FLOAT3X4_COLUMN_MAJOR = 0,    // 3x4 affine transformation in column major format (see ZE_RAYTRACING_TRANSFORM_FLOAT3X4_COLUMN_MAJOR layout)
  ZE_RAYTRACING_TRANSFORM_FORMAT_FLOAT4X4_COLUMN_MAJOR = 1,    // 4x4 affine transformation in column major format (see ZE_RAYTRACING_TRANSFORM_FLOAT4X4_COLUMN_MAJOR layout)
  ZE_RAYTRACING_TRANSFORM_FORMAT_FLOAT3X4_ROW_MAJOR = 2,       // 3x4 affine transformation in row    major format (see ZE_RAYTRACING_TRANSFORM_FLOAT3X4_ROW_MAJOR    layout)
} ze_raytracing_transform_format_t;


/* Instance flags supported (identical to DXR spec) */
typedef enum ze_raytracing_instance_flags_t : uint8_t
{
  ZE_RAYTRACING_INSTANCE_FLAG_NONE = 0,
  ZE_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE = 0x1,
  ZE_RAYTRACING_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE = 0x2,
  ZE_RAYTRACING_INSTANCE_FLAG_FORCE_OPAQUE = 0x4,
  ZE_RAYTRACING_INSTANCE_FLAG_FORCE_NON_OPAQUE = 0x8
} ze_raytracing_instance_flags_t;


/* Triangle mesh geometry descriptor. */
typedef struct ze_raytracing_geometry_triangles_desc_t  // 40 bytes
{
  ze_raytracing_geometry_type_t geometryType;           // must be ze_raytracing_geometry_type_triangles_t
  ze_raytracing_geometry_flags_t geometryFlags;         // geometry flags for all primitives of this geometry
  uint8_t geometryMask;                                 // 8-bit geometry mask for ray masking
  uint8_t reserved0;                                    // must be zero
  uint32_t reserved1;                                   // must be zero
  unsigned int triangleCount;                           // number of triangles in triangleBuffer
  unsigned int triangleStride;                          // stride in bytes of triangles in triangleBuffer
  unsigned int vertexCount;                             // number of vertices in vertexBuffer
  unsigned int vertexStride;                            // stride in bytes of vertices in vertexBuffer
  ze_raytracing_triangle_indices_t* triangleBuffer;     // pointer to array of triangle indices
  ze_raytracing_float3_t* vertexBuffer;                 // pointer to array of triangle vertices
  
} ze_raytracing_raytracing_geometry_triangles_desc_t;


/* Quad mesh geometry descriptor. A quad with vertices v0,v1,v2,v3 is
 * rendered as a pair of triangles (v0,v1,v3) and (v2,v3,v1), with
 * barycentric coordinates u/v of the second triangle corrected by
 * u=1-u, and v=1-v to make a u/v parametrization of the entire
 * [0,1]x[0,1] uv space. */
typedef struct ze_raytracing_geometry_quads_desc_t // 40 bytes
{
  ze_raytracing_geometry_type_t geometryType;      // must be ze_raytracing_geometry_type_quads_t
  ze_raytracing_geometry_flags_t geometryFlags;    // geometry flags for all primitives of this geometry
  uint8_t geometryMask;                            // 8-bit geometry mask for ray masking
  uint8_t reserved0;                               // must be zero
  uint32_t reserved1;                              // must be zero
  unsigned int quadCount;                          // number of quads in quadBuffer
  unsigned int quadStride;                         // stride in bytes of quads in quadBuffer
  unsigned int vertexCount;                        // number of vertices in vertexBuffer
  unsigned int vertexStride;                       // stride in bytes of vertices in vertexBuffer
  ze_raytracing_quad_indices_t* quadBuffer;        // pointer to array of quad indices
  ze_raytracing_float3_t* vertexBuffer;            // pointer to array of quad vertices
  
} ze_raytracing_raytracing_geometry_quads_desc_t;


/* Function pointer type to return AABBs for a range of procedural primitives. */
typedef void (*ze_raytracing_geometry_aabbs_fptr_t)(const uint32_t primID,            // first primitive to return bounds for
                                                    const uint32_t primIDCount,       // number of primitives to return bounds for
                                                    void* geomUserPtr,                // pointer provided through geometry descriptor
                                                    void* buildUserPtr,               // pointer provided through zeRaytracingBuildAccel function
                                                    ze_raytracing_aabb_t* pBoundsOut  // destination buffer to write AABB bounds to
  );

/* Geometry with procedural primitives bound by AABBs. */
typedef struct ze_raytracing_geometry_aabbs_fptr_desc_t // 24 bytes
{
  ze_raytracing_geometry_type_t geometryType;          // must be ze_raytracing_geometry_type_aabbs_fptr_t
  ze_raytracing_geometry_flags_t geometryFlags;        // geometry flags for all primitives of this geometry
  uint8_t geometryMask;                                // 8-bit geometry mask for ray masking
  uint8_t reserved;                                    // must be zero
  unsigned int primCount;                              // number of primitives in geometry
  ze_raytracing_geometry_aabbs_fptr_t getBounds;       // function pointer to return bounds for a range of primitives
  void* geomUserPtr;                                   // geometry user pointer passed to callback
  
} ze_raytracing_geometry_aabbs_fptr_desc_t;


/* Instance geometry descriptor. */
typedef struct ze_raytracing_geometry_instance_desc_t  // 32 bytes
{
  ze_raytracing_geometry_type_t geometryType;          // must be ze_raytracing_geometry_type_instance_t
  ze_raytracing_instance_flags_t instanceFlags;        // flags for the instance (see ze_raytracing_instance_flags_t)
  uint8_t geometryMask;                                // 8-bit geometry mask for ray masking
  ze_raytracing_transform_format_t transformFormat;    // format of the specified transformation
  unsigned int instanceUserID;                         // a user specified identifier for the instance
  float* transform;                                    // local to world instance transformation in specified format
  ze_raytracing_aabb_t* bounds;                        // AABB of the instanced acceleration structure
  void* accel;                                         // pointer to acceleration structure to instantiate
    
} ze_raytracing_geometry_instance_desc_t;


/* A geometry descriptor. */
typedef struct ze_raytracing_geometry_desc_t {
  ze_raytracing_geometry_type_t geometryType;           // the first byte of a geometry descriptor is always its type and user can case to geometry descriptor structs above
} ze_raytracing_geometry_desc_t;


/* Bitmask with features supported. */
typedef enum ze_raytracing_features_t {
  ZE_RAYTRACING_FEATURES_NONE = 0,
  ZE_RAYTRACING_FEATURES_GEOMETRY_TYPE_TRIANGLES  = 1 << 0,    // support for ZE_RAYTRACING_GEOMETRY_TYPE_TRIANGLES geometries 
  ZE_RAYTRACING_FEATURES_GEOMETRY_TYPE_QUADS      = 1 << 1,    // support for ZE_RAYTRACING_GEOMETRY_TYPE_QUADS geometries
  ZE_RAYTRACING_FEATURES_GEOMETRY_TYPE_AABBS_FPTR = 1 << 2,    // support for ZE_RAYTRACING_GEOMETRY_TYPE_AABBS_FPTR geometries
  ZE_RAYTRACING_FEATURES_GEOMETRY_TYPE_INSTANCE   = 1 << 3,    // support for ZE_RAYTRACING_GEOMETRY_TYPE_INSTANCE geometries
  
} ze_raytracing_features_t;


/* Additional ze_result_t enum fields. */
typedef enum ze_result_t
{
  ZE_RESULT_SUCCESS,
  ZE_RESULT_RAYTRACING_RETRY_BUILD_ACCEL,    // build ran out of memory, app should re-try with more memory
  ZE_RESULT_RAYTRACING_OPERATION_DEFERRED,   // operation is deferred into ray tracing parallel operation
  ZE_RESULT_ERROR_UNKNOWN,
  
} ze_raytracing_result_t;


/* Build quality hint for acceleration structure build. */
typedef enum ze_raytracing_build_quality_t
{
  ZE_RAYTRACING_BUILD_QUALITY_LOW    = 0,   // build low quality acceleration structure (fast)
  ZE_RAYTRACING_BUILD_QUALITY_MEDIUM = 1,   // build medium quality acceleration structure (slower)
  ZE_RAYTRACING_BUILD_QUALITY_HIGH   = 2,   // build high quality acceleration structure (slow)
  
} ze_raytracing_build_quality_t;


/* Some additional hints for acceleration structure build. */
typedef enum ze_raytracing_build_flags_t
{
  ZE_RAYTRACING_BUILD_FLAGS_NONE    = 0,
  ZE_RAYTRACING_BUILD_FLAGS_DYNAMIC = (1 << 0),  // optimize for dynamic content
  ZE_RAYTRACING_BUILD_FLAGS_COMPACT = (1 << 1),  // build more compact acceleration structure
  
} ze_raytracing_build_flags_t;


/* A handle of a parallel operation that can get joined with worker threads. */
typedef ze_raytracing_parallel_operation_opaque_t* ze_raytracing_parallel_operation_t;


/* Structure returned by zeRaytracingGetAccelSize that contains acceleration
 * structure size estimates. */
typedef struct ze_raytracing_accel_size_t
{
  /* The size of this structure in bytes. */
  size_t structBytes;

  /* The expected number of bytes required for the acceleration
   * structure. When using an acceleration structure buffer of that
   * size, the build is expected to succeed mostly, but it may fail
   * with ZE_RESULT_RAYTRACING_RETRY_BUILD_ACCEL. */
  size_t accelBufferExpectedBytes;

  /* The worst number of bytes required for the acceleration
   * structure. When using an acceleration structure buffer of that
   * size, the build is guaranteed to not run out of memory. */
  size_t accelBufferWorstCaseBytes;

  /* The scratch buffer bytes required for the acceleration structure
   * build. */
  size_t scratchBufferBytes;
  
} ze_raytracing_accel_size_t;


/* Argument structure passed to zeRaytracingBuildAccel function. */
typedef struct ze_raytracing_build_accel_args_t
{
  /* The size of this structure in bytes */
  size_t structBytes;

  /* Array of pointers to geometry descriptors. This array and the
   * geometry descriptors themselves can be standard host memory
   * allocations. A pointer to a geometry descriptor can be null, in
   * which case the geometry is treated as empty. */
  const ze_raytracing_geometry_desc_t** geometries;

  /* Number of geometries in geometry descriptor array. */
  uint32_t numGeometries;

  /* Destination buffer for acceleration structure. This has to be a
   * shared memory allocation aligned to
   * ZE_RAYTRACING_ACCELERATION_STRUCTURE_ALIGNMENT bytes and using the ray
   * tracing allocation descriptor
   * (ze_raytracing_mem_alloc_ext_desc_t) in the zeMemAllocShared
   * call. */
  void* accelBuffer;

  /* Number of allocated bytes of the acceleration structure
   * buffer. This can be 0 in which case the build implementation may
   * just return an improved accel buffer expected size estimate by
   * looking at the scene data. */
  size_t accelBufferBytes;

  /* Scratch space buffer to be used during accleration structure
   * construction. Can be a standard host allocation. */
  void* scratchBuffer;

  /* Number of allocated bytes of the scratch space buffer. */
  size_t scratchBufferBytes;

  /* Build quality to use (see ZE_RAYTRACING_BUILD_QUALITY) */
  ze_raytracing_build_quality_t quality;

  /* Some hints for acceleration structure build (see ze_raytracing_build_flags_t) */
  ze_raytracing_build_flags_t flags;

  /* When parallelOperation is NULL, the build is executed
   * sequentially on the current thread. If a parallelOperation is
   * specified, then the parallel operation gets attached to the
   * parallel build handle. This handle can then get joined with
   * worker threads to perform the parallel build operation. Only a
   * single build operation can be attached to a parallel build
   * operation at a given time. */
  ze_raytracing_parallel_operation_t parallelOperation;

  /* A pointer passed to callbacks. */
  void* buildUserPtr;
  
  /* When the pointer is NULL no data is returned. When the build
   * fails with ZE_RESULT_RAYTRACING_RETRY_BUILD_ACCEL, this returns new expected
   * acceleration structure bytes to be used for re-build. When build
   * succeeds this returns the number of bytes used in the
   * acceleration structure buffer. */
  size_t* accelBufferBytesOut;
  
  /* Destination address to write acceleration structure bounds to (can be NULL). */
  ze_raytracing_aabb_t* boundsOut;
  
} ze_raytracing_build_accel_args_t;


/*
 * Returns features supported by the implementation.
 */

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingGetSupportedFeatures( ze_device_handle_t deviceID, ze_raytracing_features_t* pRaytracingFeatures );

/*
 * The zeRaytracingGetAccelSize function calculates the size of buffers
 * required for the acceleration structure build.

   args: Specifies the build operation to estimate the buffer sizes
         for. The zeRaytracingGetAccelSize function may look at the number
         of geometries, the geometry descriptors (but not referenced
         data), the existence of an acceleration structure to link,
         the build quality, and build flags.

   sizeOut: A structure that contains the size information returned,
         see ZE_RAYTRACING_ACCEL_SIZE for more details.

*/

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingGetAccelSize( const ze_raytracing_build_accel_args_r* args, ze_raytracing_accel_size_t* pAccelSizeOut );


/*
 * The zeRaytracingBuildAccel function builds an acceleration structure of
 * the scene consisting of the specified geometry descriptors and
 * writes the acceleration structure to the provided destination
 * buffer. The build happens on the CPU and is synchronous, thus after
 * the function returns the acceleration structure can get used.
 *
 * It is the users responsibility to manage the acceleration structure
 * buffer allocation, deallocation, and potential prefetching to the
 * device memory. The required size of the accleration structure
 * buffer can be queried with the zeRaytracingGetAccelSize function. The
 * acceleration structure buffer must be a shared USM allocation and
 * should be present on the host at build time. The referenced scene
 * data (index and vertex buffers) can be standard host allocations,
 * and will not be referenced into by the build acceleration
 * structure.
 *
 * Before an accleration structure is build, the user has to query the
 * size of the acceleration structure and scratch space buffer to
 * allocate using the zeRaytracingGetAccelSize. When using the worst case
 * size to allocate the acceleration structure buffer, then the
 * zeRaytracingGetAccelSize function will never fail with an out of memory
 * error. When using the expected size for the acceleration structure
 * buffer, then the build may fail with ZE_RESULT_RAYTRACING_RETRY_BUILD_ACCEL and
 * should then get re-tried with a larger amount of data. Using the
 * AccelBufferBytesExpected pointer, the build itself returns an
 * improved size estimate which the user can use to re-try the build.
 *
 * The parallelization of the acceleration structure is abstracted,
 * such that the user can decide which tasking library to use,
 * e.g. Intel(R) oneAPI Threading Building Blocks. Therefore, the user
 * specifies a function pointer to the parallelFor implementation of
 * its tasking system. The parallelFor function pointer gets invoked
 * by the implementation using the parallel_for body to
 * parallelize. The tasking system has to support nested parallelism
 * (thus parallel_for can get invoked inside a parallel_for) and C++
 * exceptions have to get properly propagated to a parent task to be
 * potentially catched by the user.
 */

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingBuildAccel( const ze_raytracing_build_accel_args_t* args );

/*
 * Creates a new parallel operation that can get attached to a build
 * operation. Only a single build operation can be attached to a
 * parallel operation at a given time.
 */

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingParallelOperationCreate( ze_raytracing_parallel_operation_t* phParallelOperation );

/*
 * Destroys a parallel operation.
 */

ZE_APIEXPORT ze_result ZE_APICALL zeRaytracingParallelOperationDestroy( ze_raytracing_parallel_operation_t hParallelOperation );

/*
 * Returns the maximal number of threads that can join the parallel operation.
 */

ZE_APIEXPORT ze_result ZE_APICALL zeRaytracingParallelOperationGetMaxConcurrency( ze_raytracing_parallel_operation_t hParallelOperation, uint32_t* pMaxConcurency );


/* 
 * Called by worker threads to join a parallel build operation. When
 * the build finished, all worker threads return from this function
 * with the same error code for the build.
 */

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingParallelOperationJoin( ze_raytracing_parallel_operation_t hParallelOperation );
