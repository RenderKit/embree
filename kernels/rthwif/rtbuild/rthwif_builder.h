// Copyright 2009-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if !defined(EMBREE_LEVEL_ZERO)
struct _ze_device_handle_t {};
#else
struct _ze_device_handle_t;
#endif

typedef struct _ze_device_handle_t *ze_device_handle_t;

#if defined(__cplusplus)
#  define RTHWIF_API_EXTERN_C extern "C"
#else
#  define RTHWIF_API_EXTERN_C
#endif

#if defined(_WIN32)
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C //__declspec(dllimport)
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C //__declspec(dllexport)
#else
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C __attribute__ ((visibility ("default")))
#endif

#if defined(RTHWIF_EXPORT_API)
#  define RTHWIF_API RTHWIF_API_EXPORT
#else
#  define RTHWIF_API RTHWIF_API_IMPORT
#endif

/* Required alignment of acceleration structure buffers. */
#define RTHWIF_ACCELERATION_STRUCTURE_ALIGNMENT 128

/* Geometry flags supported (identical to DXR spec). */
typedef enum RTHWIF_GEOMETRY_FLAGS : uint8_t
{
  RTHWIF_GEOMETRY_FLAG_NONE = 0,
  RTHWIF_GEOMETRY_FLAG_OPAQUE = 0x1,                           // Opaque geometries do not invoke an anyhit shader
  RTHWIF_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION = 0x2    // Guarantees single anyhit shader invokation for primitives.
} RTHWIF_GEOMETRY_FLAGS;


/* A 3-component short vector type. */
typedef struct RTHWIF_FLOAT3 {
  float x, y, z;
} RTHWIF_FLOAT3;


/* A 3x4 affine transformation with column vectors vx, vy, vz, and p
 * transforming a point (x,y,z) to x*vx + y*vy + z*vz + p. */
typedef struct RTHWIF_TRANSFORM_FLOAT3X4_COLUMN_MAJOR {
  float vx_x, vx_y, vx_z; // column 0
  float vy_x, vy_y, vy_z; // column 1
  float vz_x, vz_y, vz_z; // column 2
  float  p_x,  p_y,  p_z; // column 3
} RTHWIF_TRANSFORM_FLOAT3X4_COLUMN_MAJOR;


/* Same as RTHWIF_TRANSFORM_FLOAT3X4_COLUMN_MAJOR, with ignored 4th
 * component of column vectors. */
typedef struct RTHWIF_TRANSFORM_FLOAT4X4_COLUMN_MAJOR {
  float vx_x, vx_y, vx_z, pad0; // column 0
  float vy_x, vy_y, vy_z, pad1; // column 1
  float vz_x, vz_y, vz_z, pad2; // column 2
  float  p_x,  p_y,  p_z, pad3; // column 3
} RTHWIF_TRANSFORM_FLOAT4X4_COLUMN_MAJOR;


/* Same as RTHWIF_TRANSFORM_FLOAT3X4_COLUMN_MAJOR, but using row major
 * layout. */
typedef struct RTHWIF_TRANSFORM_FLOAT3X4_ROW_MAJOR {
  float vx_x, vy_x, vz_x, p_x; // row 0
  float vx_y, vy_y, vz_y, p_y; // row 1
  float vx_z, vy_z, vz_z, p_z; // row 2
} RTHWIF_TRANSFORM_FLOAT3X4_ROW_MAJOR;


/* An axis aligned bounding box with lower and upper bounds in each
 * dimension. */
typedef struct RTHWIF_AABB
{
  RTHWIF_FLOAT3 lower; // lower bounds
  RTHWIF_FLOAT3 upper; // upper bounds
} RTHWIF_AABB;


/* A triangle represented using 3 vertex indices. */
typedef struct RTHWIF_TRIANGLE_INDICES {
  uint32_t v0, v1, v2;      // 3 triangle indices pointing into vertex array
} RTHWIF_TRIANGLE_INDICES;


/* A quad (triangle pair) represented using 4 vertex indices. */
typedef struct RTHWIF_QUAD_INDICES { 
  uint32_t v0, v1, v2, v3; // 4 quad indices pointing into vertex array
} RTHWIF_QUAD_INDICES;


/* The types of geometries supported. */
typedef enum RTHWIF_GEOMETRY_TYPE : uint8_t
{
  RTHWIF_GEOMETRY_TYPE_TRIANGLES = 0,   // triangle mesh geometry type
  RTHWIF_GEOMETRY_TYPE_QUADS = 1,       // quad mesh geometry type
  RTHWIF_GEOMETRY_TYPE_AABBS_FPTR = 2,  // procedural geometry with AABB bounds per primitive
  RTHWIF_GEOMETRY_TYPE_INSTANCE = 3,    // instance geometry
} RTHWIF_GEOMETRY_TYPE;

/* The format of transformations supported. */
typedef enum RTHWIF_TRANSFORM_FORMAT : uint8_t
{
  RTHWIF_TRANSFORM_FORMAT_FLOAT3X4_COLUMN_MAJOR = 0,          // 3x4 affine transformation in column major format (see RTHWIF_TRANSFORM_FLOAT3X4_COLUMN_MAJOR layout)
  RTHWIF_TRANSFORM_FORMAT_FLOAT4X4_COLUMN_MAJOR = 1,          // 4x4 affine transformation in column major format (see RTHWIF_TRANSFORM_FLOAT4X4_COLUMN_MAJOR layout)
  RTHWIF_TRANSFORM_FORMAT_FLOAT3X4_ROW_MAJOR = 2,             // 3x4 affine transformation in row    major format (see RTHWIF_TRANSFORM_FLOAT3X4_ROW_MAJOR    layout)

} RTHWIF_TRANSFORM_FORMAT;


/* Instance flags supported (identical to DXR spec) */
typedef enum RTHWIF_INSTANCE_FLAGS : uint8_t
{
  RTHWIF_INSTANCE_FLAG_NONE = 0,
  RTHWIF_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE = 0x1,
  RTHWIF_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE = 0x2,
  RTHWIF_INSTANCE_FLAG_FORCE_OPAQUE = 0x4,
  RTHWIF_INSTANCE_FLAG_FORCE_NON_OPAQUE = 0x8
} RTHWIF_INSTANCE_FLAGS;


/* Triangle mesh geometry descriptor. */
typedef struct RTHWIF_GEOMETRY_TRIANGLES_DESC  // 40 bytes
{
  RTHWIF_GEOMETRY_TYPE geometryType;       // must be RTHWIF_GEOMETRY_TYPE_TRIANGLES
  RTHWIF_GEOMETRY_FLAGS geometryFlags;     // geometry flags for all primitives of this geometry
  uint8_t geometryMask;                    // 8-bit geometry mask for ray masking
  uint8_t reserved0;                       // must be zero
  uint32_t reserved1;                      // must be zero
  unsigned int triangleCount;              // number of triangles in triangleBuffer
  unsigned int triangleStride;             // stride in bytes of triangles in triangleBuffer
  unsigned int vertexCount;                // number of vertices in vertexBuffer
  unsigned int vertexStride;               // stride in bytes of vertices in vertexBuffer
  RTHWIF_TRIANGLE_INDICES* triangleBuffer; // pointer to array of triangle indices
  RTHWIF_FLOAT3* vertexBuffer;             // pointer to array of triangle vertices
  
} RTHWIF_RAYTRACING_GEOMETRY_TRIANGLES_DESC;


/* Quad mesh geometry descriptor. A quad with vertices v0,v1,v2,v3 is
 * rendered as a pair of triangles (v0,v1,v3) and (v2,v3,v1), with
 * barycentric coordinates u/v of the second triangle corrected by
 * u=1-u, and v=1-v to make a u/v parametrization of the entire
 * [0,1]x[0,1] uv space. */
typedef struct RTHWIF_GEOMETRY_QUADS_DESC // 40 bytes
{
  RTHWIF_GEOMETRY_TYPE geometryType;     // must be RTHWIF_GEOMETRY_TYPE_QUADS
  RTHWIF_GEOMETRY_FLAGS geometryFlags;   // geometry flags for all primitives of this geometry
  uint8_t geometryMask;                  // 8-bit geometry mask for ray masking
  uint8_t reserved0;                     // must be zero
  uint32_t reserved1;                    // must be zero
  unsigned int quadCount;                // number of quads in quadBuffer
  unsigned int quadStride;               // stride in bytes of quads in quadBuffer
  unsigned int vertexCount;              // number of vertices in vertexBuffer
  unsigned int vertexStride;             // stride in bytes of vertices in vertexBuffer
  RTHWIF_QUAD_INDICES* quadBuffer;       // pointer to array of quad indices
  RTHWIF_FLOAT3* vertexBuffer;           // pointer to array of quad vertices
  
} RTHWIF_RAYTRACING_GEOMETRY_QUADS_DESC;


/* Function pointer type to return AABBs for a range of procedural primitives. */
typedef void (*RTHWIF_GEOMETRY_AABBS_FPTR)(const uint32_t primID,        // first primitive to return bounds for
                                           const uint32_t primIDCount,   // number of primitives to return bounds for
                                           void* geomUserPtr,            // pointer provided through geometry descriptor
                                           void* buildUserPtr,           // pointer provided through rthwifBuildAccel function
                                           RTHWIF_AABB* boundsOut        // destination buffer to write AABB bounds to
  );

/* Geometry with procedural primitives bound by AABBs. */
typedef struct RTHWIF_GEOMETRY_AABBS_FPTR_DESC // 24 bytes
{
  RTHWIF_GEOMETRY_TYPE geometryType;          // must be RTHWIF_GEOMETRY_TYPE_AABBS_FPTR
  RTHWIF_GEOMETRY_FLAGS geometryFlags;        // geometry flags for all primitives of this geometry
  uint8_t geometryMask;                       // 8-bit geometry mask for ray masking
  uint8_t reserved;                           // must be zero
  unsigned int primCount;                     // number of primitives in geometry
  RTHWIF_GEOMETRY_AABBS_FPTR getBounds;       // function pointer to return bounds for a range of primitives
  void* geomUserPtr;                          // geometry user pointer passed to callback
  
} RTHWIF_GEOMETRY_AABBS_DESC;


/* Instance geometry descriptor. */
typedef struct RTHWIF_GEOMETRY_INSTANCE_DESC // 32 bytes
{
  RTHWIF_GEOMETRY_TYPE geometryType;          // must be RTHWIF_GEOMETRY_TYPE_INSTANCE
  RTHWIF_INSTANCE_FLAGS instanceFlags;        // flags for the instance (see RTHWIF_INSTANCE_FLAGS)
  uint8_t geometryMask;                       // 8-bit geometry mask for ray masking
  RTHWIF_TRANSFORM_FORMAT transformFormat;    // format of the specified transformation
  unsigned int instanceUserID;                // a user specified identifier for the instance
  float* transform;                           // local to world instance transformation in specified format
  RTHWIF_AABB* bounds;                        // AABB of the instanced acceleration structure
  void* accel;                                // pointer to acceleration structure to instantiate
    
} RTHWIF_GEOMETRY_INSTANCE_DESC;


/* A geometry descriptor. */
typedef struct RTHWIF_GEOMETRY_DESC {
  RTHWIF_GEOMETRY_TYPE geometryType;           // the first byte of a geometry descriptor is always its type and user can case to geometry descriptor structs above
} RTHWIF_GEOMETRY_DESC;


/* Bitmask with features supported. */
typedef enum RTHWIF_FEATURES {
  RTHWIF_FEATURES_NONE = 0,
  RTHWIF_FEATURES_GEOMETRY_TYPE_TRIANGLES  = 1 << 0,    // support for RTHWIF_GEOMETRY_TYPE_TRIANGLES geometries 
  RTHWIF_FEATURES_GEOMETRY_TYPE_QUADS      = 1 << 1,    // support for RTHWIF_GEOMETRY_TYPE_QUADS geometries
  RTHWIF_FEATURES_GEOMETRY_TYPE_AABBS_FPTR = 1 << 2,    // support for RTHWIF_GEOMETRY_TYPE_AABBS_FPTR geometries
  RTHWIF_FEATURES_GEOMETRY_TYPE_INSTANCE   = 1 << 3,    // support for RTHWIF_GEOMETRY_TYPE_INSTANCE geometries
  
} RTHWIF_FEATURES;


/* Return errors from API functions. */
typedef enum RTHWIF_ERROR
{
  RTHWIF_ERROR_NONE  = 0x0,  // no error occured
  RTHWIF_ERROR_RETRY = 0x1,  // build ran out of memory, app should re-try with more memory
  RTHWIF_ERROR_OTHER = 0x2,  // some unspecified error occured
  RTHWIF_ERROR_PARALLEL_OPERATION = 0x3,  // task executing in parallel operation
  
} RTHWIF_ERROR;


/* Build quality hint for acceleration structure build. */
typedef enum RTHWIF_BUILD_QUALITY
{
  RTHWIF_BUILD_QUALITY_LOW    = 0,   // build low quality acceleration structure (fast)
  RTHWIF_BUILD_QUALITY_MEDIUM = 1,   // build medium quality acceleration structure (slower)
  RTHWIF_BUILD_QUALITY_HIGH   = 2,   // build high quality acceleration structure (slow)
  
} RTHWIF_BUILD_QUALITY;


/* Some additional hints for acceleration structure build. */
typedef enum RTHWIF_BUILD_FLAGS
{
  RTHWIF_BUILD_FLAG_NONE                    = 0,
  RTHWIF_BUILD_FLAG_DYNAMIC                 = (1 << 0),  // optimize for dynamic content
  RTHWIF_BUILD_FLAG_COMPACT                 = (1 << 1),  // build more compact acceleration structure
  
} RTHWIF_BUILD_FLAGS;


/* A handle of a parallel operation that can get joined with worker threads. */
typedef void* RTHWIF_PARALLEL_OPERATION;


/* Structure returned by rthwifGetAccelSize that contains acceleration
 * structure size estimates. */
typedef struct RTHWIF_ACCEL_SIZE
{
  /* The size of this structure in bytes. */
  size_t structBytes;

  /* The expected number of bytes required for the acceleration
   * structure. When using an acceleration structure buffer of that
   * size, the build is expected to succeed mostly, but it may fail
   * with RTHWIF_ERROR_RETRY. */
  size_t accelBufferExpectedBytes;

  /* The worst number of bytes required for the acceleration
   * structure. When using an acceleration structure buffer of that
   * size, the build is guaranteed to not run out of memory. */
  size_t accelBufferWorstCaseBytes;

  /* The scratch buffer bytes required for the acceleration structure
   * build. */
  size_t scratchBufferBytes;
  
} RTHWIF_ACCEL_SIZE;


/* Argument structure passed to rthwifBuildAccel function. */
typedef struct RTHWIF_BUILD_ACCEL_ARGS
{
  /* The size of this structure in bytes */
  size_t structBytes;

  /* Array of pointers to geometry descriptors. This array and the
   * geometry descriptors themselves can be standard host memory
   * allocations. A pointer to a geometry descriptor can be null, in
   * which case the geometry is treated as empty. */
  const RTHWIF_GEOMETRY_DESC** geometries;

  /* Number of geometries in geometry descriptor array. */
  uint32_t numGeometries;

  /* Destination buffer for acceleration structure. This has to be a
   * shared memory allocation aligned to
   * RTHWIF_ACCELERATION_STRUCTURE_ALIGNMENT bytes and using the ray
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

  /* Build quality to use (see RTHWIF_BUILD_QUALITY) */
  RTHWIF_BUILD_QUALITY quality;

  /* Some hints for acceleration structure build (see RTHWIF_BUILD_FLAGS) */
  RTHWIF_BUILD_FLAGS flags;

  /* When parallelOperation is NULL, the build is executed
   * sequentially on the current thread. If a parallelOperation is
   * specified, then the parallel operation gets attached to the
   * parallel build handle. This handle can then get joined with
   * worker threads to perform the parallel build operation. Only a
   * single build operation can be attached to a parallel build
   * operation at a given time. */
  RTHWIF_PARALLEL_OPERATION parallelOperation;

  /* A pointer passed to callbacks. */
  void* buildUserPtr;
  
  /* When the pointer is NULL no data is returned. When the build
   * fails with RTHWIF_ERROR_RETRY, this returns new expected
   * acceleration structure bytes to be used for re-build. When build
   * succeeds this returns the number of bytes used in the
   * acceleration structure buffer. */
  size_t* accelBufferBytesOut;
  
  /* Destination address to write acceleration structure bounds to (can be NULL). */
  RTHWIF_AABB* boundsOut;

  /* for debugging purposes use only */
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
  void* dispatchGlobalsPtr;
#endif
  
} RTHWIF_BUILD_ACCEL_ARGS;

/*
 * Initializes the library.
 */

RTHWIF_API void rthwifInit();

/*
 * Cleans up the library.
 */

RTHWIF_API void rthwifExit();

/*
 * Returns features supported by the implementation.
 */

RTHWIF_API RTHWIF_FEATURES rthwifGetSupportedFeatures(ze_device_handle_t hDevice);

/*
 * The rthwifGetAccelSize function calculates the size of buffers
 * required for the acceleration structure build.

   args: Specifies the build operation to estimate the buffer sizes
         for. The rthwifGetAccelSize function may look at the number
         of geometries, the geometry descriptors (but not referenced
         data), the existence of an acceleration structure to link,
         the build quality, and build flags.

   sizeOut: A structure that contains the size information returned,
         see RTHWIF_ACCEL_SIZE for more details.

*/

RTHWIF_API RTHWIF_ERROR rthwifGetAccelSize(const RTHWIF_BUILD_ACCEL_ARGS& args, RTHWIF_ACCEL_SIZE& sizeOut);


/*
 * The rthwifBuildAccel function builds an acceleration structure of
 * the scene consisting of the specified geometry descriptors and
 * writes the acceleration structure to the provided destination
 * buffer. The build happens on the CPU and is synchronous, thus after
 * the function returns the acceleration structure can get used.
 *
 * It is the users responsibility to manage the acceleration structure
 * buffer allocation, deallocation, and potential prefetching to the
 * device memory. The required size of the accleration structure
 * buffer can be queried with the rthwifGetAccelSize function. The
 * acceleration structure buffer must be a shared USM allocation and
 * should be present on the host at build time. The referenced scene
 * data (index and vertex buffers) can be standard host allocations,
 * and will not be referenced into by the build acceleration
 * structure.
 *
 * Before an accleration structure is build, the user has to query the
 * size of the acceleration structure and scratch space buffer to
 * allocate using the rthwifGetAccelSize. When using the worst case
 * size to allocate the acceleration structure buffer, then the
 * rthwifGetAccelSize function will never fail with an out of memory
 * error. When using the expected size for the acceleration structure
 * buffer, then the build may fail with RTHWIF_ERROR_RETRY and
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

RTHWIF_API RTHWIF_ERROR rthwifBuildAccel(const RTHWIF_BUILD_ACCEL_ARGS& args);

/*
 * Creates a new parallel operation that can get attached to a build
 * operation. Only a single build operation can be attached to a
 * parallel operation at a given time.
 */

RTHWIF_API RTHWIF_PARALLEL_OPERATION rthwifNewParallelOperation();

/*
 * Destroys a parallel operation.
 */

RTHWIF_API void rthwifDeleteParallelOperation( RTHWIF_PARALLEL_OPERATION  parallelOperation );

/*
 * Returns the maximal number of threads that can join the parallel operation.
 */

RTHWIF_API uint32_t rthwifGetParallelOperationMaxConcurrency( RTHWIF_PARALLEL_OPERATION  parallelOperation );


/* 
 * Called by worker threads to join a parallel build operation. When
 * the build finished, all worker threads return from this function
 * with the same error code for the build.
 */

RTHWIF_API RTHWIF_ERROR rthwifJoinParallelOperation( RTHWIF_PARALLEL_OPERATION parallelOperation );
