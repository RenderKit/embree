// Copyright 2009-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <level_zero/ze_api.h>


/**
   \file

   \brief This file contains a Level Zero API extension to build ray
   tracing acceleration structures to be used with an OpenCL ray
   tracing API extension
   (https://github.com/intel-innersource/libraries.graphics.renderkit.embree/blob/master/kernels/rthwif/rthwif_production_igc.h).

 */


/** Required alignment of acceleration structure buffers. */
#define ZE_RAYTRACING_ACCELERATION_STRUCTURE_ALIGNMENT_EXT 128

/**
 \brief Enumeration of geometry flags supported by the geometries.
*/
typedef enum ze_raytracing_geometry_ext_flags_t : uint8_t
{
  ZE_RAYTRACING_GEOMETRY_EXT_FLAG_NONE = 0,        ///< Default geometry flags
  ZE_RAYTRACING_GEOMETRY_EXT_FLAG_OPAQUE = 1,      ///< Opaque geometries do not invoke an anyhit shader
} ze_raytracing_geometry_ext_flags_t;


/**
 \brief A 3-component short vector type. 
*/
typedef struct ze_raytracing_float3_ext_t {
  float x; ///< x coordinate of float3 vector
  float y; ///< y coordinate of float3 vector
  float z; ///< z coordinate of float3 vector
} ze_raytracing_float3_ext_t;


/**
 \brief 3x4 affine transformation in column major layout
 
 A 3x4 affine transformation in column major layout, consisting of
 vectors vx=(vx_x,vx_y,vx_z), vy=(vy_x,vy_y,vy_z),
 vz=(vz_x,vz_y,vz_z), and p=(p_x,p_y,p_z). The transformation
 transforms a point (x,y,z) to x*vx + y*vy + z*vz + * p.

*/
typedef struct ze_raytracing_transform_float3x4_column_major_ext_t {
  float vx_x, vx_y, vx_z; ///< column 0 of 3x4 matrix
  float vy_x, vy_y, vy_z; ///< column 1 of 3x4 matrix
  float vz_x, vz_y, vz_z; ///< column 2 of 3x4 matrix
  float  p_x,  p_y,  p_z; ///< column 3 of 3x4 matrix
} ze_raytracing_transform_float3x4_column_major_ext_t;


/**
   \brief 3x4 affine transformation in column major layout with aligned column vectors

   A 3x4 affine transformation in column major layout, consisting of
   vectors vx=(vx_x,vx_y,vx_z), vy=(vy_x,vy_y,vy_z),
   vz=(vz_x,vz_y,vz_z), and p=(p_x,p_y,p_z). The transformation
   transforms a point (x,y,z) to x*vx + y*vy + z*vz + p. The column
   vectors are aligned to 16 bytes and pad members are ignored.
*/
typedef struct ze_raytracing_transform_float3x4_aligned_column_major_ext_t
{
  float vx_x, vx_y, vx_z, pad0; ///< column 0 of 3x4 matrix with ignored padding
  float vy_x, vy_y, vy_z, pad1; ///< column 1 of 3x4 matrix with ignored padding
  float vz_x, vz_y, vz_z, pad2; ///< column 2 of 3x4 matrix with ignored padding
  float  p_x,  p_y,  p_z, pad3; ///< column 3 of 3x4 matrix with ignored padding
} ze_raytracing_transform_float3x4_aligned_column_major_ext_t;


/** 

 \brief 3x4 affine transformation in row major layout

  A 3x4 affine transformation in row major layout, consisting of
  vectors vx=(vx_x,vx_y,vx_z), vy=(vy_x,vy_y,vy_z),
  vz=(vz_x,vz_y,vz_z), and p=(p_x,p_y,p_z). The transformation
  transforms a point (x,y,z) to x*vx + y*vy + z*vz + p.
*/
typedef struct ze_raytracing_transform_float3x4_row_major_ext_t
{
  float vx_x, vy_x, vz_x, p_x; ///< row 0 of 3x4 matrix
  float vx_y, vy_y, vz_y, p_y; ///< row 1 of 3x4 matrix
  float vx_z, vy_z, vz_z, p_z; ///< row 2 of 3x4 matrix
} ze_raytracing_transform_float3x4_row_major_ext_t;


/** 

  \brief Axis aligned 3-dimensional bounding box (AABB) with lower and
  upper bounds in each dimension.

*/
typedef struct ze_raytracing_aabb_ext_t
{
  ze_raytracing_float3_ext_t lower; ///< lower bounds of AABB
  ze_raytracing_float3_ext_t upper; ///< upper bounds of AABB
} ze_raytracing_aabb_ext_t;


/** 

  \brief Triangle represented using 3 vertex indices

  This strucure represents a triangle using 3 vertex indices that
  index into a vertex array that needs to be provided together with
  the index array.

  The linear barycentric u/v parametrization of the triangle is
  defined to be (u=0, v=0) at v0, (u=1, v=0) at v1, and (u=0, v=1) at
  v2.

*/
typedef struct ze_raytracing_triangle_indices_ext_t
{
  uint32_t v0;   ///< first index pointing to the first triangle vertex in vertex array
  uint32_t v1;   ///< second index pointing to the second triangle vertex in vertex array
  uint32_t v2;   ///< third index pointing to the third triangle vertex in vertex array
} ze_raytracing_triangle_indices_ext_t;


/** 

  \brief Quad represented using 4 vertex indices

  Represents a quad composed of 4 indices that index into a vertex
  array that needs to be provided together with the index array.

  A quad is a triangle pair represented using 4 vertex indices
  v0,v1,v2,v3. The first triangle is made out of indices v0,v1,v3 and
  the second triangle from indices v2,v3,v1. The piecewise linear
  barycentric u/v pametrization of the quad is defined to be (u=0,
  v=0) at v0, (u=1, v=0) at v1, (u=0, v=1) at v3, and (u=1, v=1) at
  v2. This is achieved by correcting the u'/v' coordinates of the
  second triangle by u = 1-u' and v = 1-v', thus the parametrization
  is piecewise linear.

*/
typedef struct ze_raytracing_quad_indices_ext_t
{ 
  uint32_t v0;  ///< first index pointing to the first quad vertex in vertex array
  uint32_t v1;  ///< second index pointing to the second quad vertex in vertex array
  uint32_t v2;  ///< third index pointing to the third quad vertex in vertex array
  uint32_t v3;  ///< forth index pointing to the forth quad vertex in vertex array
} ze_raytracing_quad_indices_ext_t;


/**

  \brief The types of geometries supported. 

  A type that identifies the various geometry descriptors supported by
  the API. Each geometry descriptor has this type stored as it first
  member at offset 0 to identify the type of the descriptor.

*/
typedef enum ze_raytracing_geometry_type_ext_t : uint8_t
{
  ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES = 0,   ///< triangle mesh geometry type identifying ze_raytracing_geometry_triangles_ext_desc_ext_t
  ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS = 1,       ///< quad mesh geometry type identifying ze_raytracing_geometry_quads_ext_desc_ext_t
  ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR = 2,  ///< procedural geometry type identifying ze_raytracing_geometry_aabbs_fptr_ext_desc_ext_t
  ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE = 3,    ///< instance geometry type identifying ze_raytracing_geometry_instance_ext_desc_ext_t
} ze_raytracing_geometry_type_ext_t;


/**


 \brief The format of transformations supported. 

 To specify instance transformations various formats of the
 transformation are supported.

*/
typedef enum ze_raytracing_transform_format_ext_t : uint8_t
{
  ZE_RAYTRACING_TRANSFORM_FORMAT_EXT_FLOAT3X4_COLUMN_MAJOR = 0,         ///< 3x4 affine transformation in column major format (see ze_raytracing_transform_float3x4_column_major_ext_t layout)
  ZE_RAYTRACING_TRANSFORM_FORMAT_EXT_FLOAT3X4_ALIGNED_COLUMN_MAJOR = 1, ///< 3x4 affine transformation in column major format (see ze_raytracing_transform_float3x4_aligned_column_major_ext_t layout)
  ZE_RAYTRACING_TRANSFORM_FORMAT_EXT_FLOAT3X4_ROW_MAJOR = 2,            ///< 3x4 affine transformation in row    major format (see ze_raytracing_transform_float3x4_row_major_ext_t layout)
} ze_raytracing_transform_format_ext_t;


/**

  \brief Instance flags supported

  This enumation lists flags to be used to specify instances.

*/
typedef enum ze_raytracing_instance_ext_flags_ext_t : uint8_t
{
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_NONE = 0,                               ///< default instance flag
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_TRIANGLE_CULL_DISABLE = 0x1,            ///< disables culling of front and backfacing triangles
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE = 0x2,  ///< reverses front and back face of triangles
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_FORCE_OPAQUE = 0x4,                     ///< forces instanced geometry to be opaque, unless ray flag forces it to be non-opaque
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_FORCE_NON_OPAQUE = 0x8                  ///< forces instanced geometry to be non-opaque, unless ray flag forces it to be opaque
} ze_raytracing_instance_ext_flags_ext_t;

/**

 \brief Base structure for all geometry descriptors.

 Each geometry descriptor has a geometry type as it's first argument
 that identifies which geometry is represented. See
 ze_raytracing_geometry_type_ext_t for details.

*/
typedef struct ze_raytracing_geometry_ext_desc_t {
  ze_raytracing_geometry_type_ext_t geometryType;           ///< type of geometry descriptor
} ze_raytracing_geometry_ext_desc_t;

/** 

  \brief Geometry descriptor of a triangle mesh 

  This geometry descriptor describes a triangle mesh consisting of an
  triangle buffer (index buffer), and a vertex buffer.

  The linear barycentric u/v parametrization of the triangle is
  defined to be (u=0, v=0) at v0, (u=1, v=0) at v1, and (u=0, v=1) at
  v2.

*/
typedef struct ze_raytracing_geometry_triangles_ext_desc_t  // 40 bytes
{
  ze_raytracing_geometry_type_ext_t geometryType;       ///< must be ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES
  ze_raytracing_geometry_ext_flags_t geometryFlags;     ///< geometry flags for all primitives of this geometry
  uint8_t geometryMask;                                 ///< 8-bit geometry mask for ray masking
  uint8_t reserved0;                                    ///< must be zero
  uint32_t reserved1;                                   ///< must be zero
  unsigned int triangleCount;                           ///< number of triangles in triangleBuffer
  unsigned int triangleStride;                          ///< stride in bytes of triangles in triangleBuffer
  unsigned int vertexCount;                             ///< number of vertices in vertexBuffer
  unsigned int vertexStride;                            ///< stride in bytes of vertices in vertexBuffer
  ze_raytracing_triangle_indices_ext_t* triangleBuffer; ///< pointer to array of triangle indices
  ze_raytracing_float3_ext_t* vertexBuffer;             ///< pointer to array of triangle vertices
  
} ze_raytracing_raytracing_geometry_triangles_ext_desc_t;


/** 

  \brief Geometry descriptor of a quad mesh

  This geometry descriptor describes a quad mesh consisting of a quad
  buffer (index buffer), and vertex buffer. The indices index
  respective vertices in the vertex array.

  A quad is a triangle pair represented using 4 vertex indices
  v0,v1,v2,v3. The first triangle is made out of indices v0,v1,v3 and
  the second triangle from indices v2,v3,v1. The piecewise linear
  barycentric u/v pametrization of the quad is defined to be (u=0,
  v=0) at v0, (u=1, v=0) at v1, (u=0, v=1) at v3, and (u=1, v=1) at
  v2. This is achieved by correcting the u'/v' coordinates of the
  second triangle by u = 1-u' and v = 1-v', thus the parametrization
  is piecewise linear.

*/
typedef struct ze_raytracing_geometry_quads_ext_desc_t // 40 bytes
{
  ze_raytracing_geometry_type_ext_t geometryType;   ///< must be ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS
  ze_raytracing_geometry_ext_flags_t geometryFlags; ///< geometry flags for all primitives of this geometry
  uint8_t geometryMask;                             ///< 8-bit geometry mask for ray masking
  uint8_t reserved0;                                ///< must be zero
  uint32_t reserved1;                               ///< must be zero
  unsigned int quadCount;                           ///< number of quads in quadBuffer
  unsigned int quadStride;                          ///< stride in bytes of quads in quadBuffer
  unsigned int vertexCount;                         ///< number of vertices in vertexBuffer
  unsigned int vertexStride;                        ///< stride in bytes of vertices in vertexBuffer
  ze_raytracing_quad_indices_ext_t* quadBuffer;     ///< pointer to an array of quad indices
  ze_raytracing_float3_ext_t* vertexBuffer;         ///< pointer to an array of quad vertices
  
} ze_raytracing_raytracing_geometry_quads_ext_desc_t;


/**
 \brief Function pointer type to return AABBs for a range of procedural primitives. 
*/
typedef void (*ze_raytracing_geometry_aabbs_fptr_ext_t)(const uint32_t primID,            ///< [in] first primitive to return bounds for
                                                        const uint32_t primIDCount,       ///< [in] number of primitives to return bounds for
                                                        void* geomUserPtr,                ///< [in] pointer provided through geometry descriptor
                                                        void* buildUserPtr,               ///< [in] pointer provided through zeRaytracingBuildAccelExt function
                                                        ze_raytracing_aabb_ext_t* pBoundsOut  ///< [out] destination buffer to write AABB bounds to
  );

/**

 \brief Geometry descriptor of procedural primitives

 Geometry with procedural primitives bound by AABBs. A host side
 bounds function is provided that is invoked by the acceleration
 structure builder to query bounds of procedural primitives on
 demand. The callback is passed some geomUserPtr provided with the
 geometry that can point to an application side representation of the
 procedural primitives. Futher, a second buildUserPtr pointer is
 passed to the callback, that can get set through the
 zeRaytracingBuildAccel function. This allows the build to change the
 bounds of the procedural geometry, e.g. to build a BVH only over a
 short time range to implement multi-segment motion blur.

*/
typedef struct ze_raytracing_geometry_aabbs_fptr_ext_desc_t // 24 bytes
{
  ze_raytracing_geometry_type_ext_t geometryType;      ///< must be ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR
  ze_raytracing_geometry_ext_flags_t geometryFlags;    ///< geometry flags for all primitives of this geometry
  uint8_t geometryMask;                                ///< 8-bit geometry mask for ray masking
  uint8_t reserved;                                    ///< must be zero
  unsigned int primCount;                              ///< number of primitives in geometry
  ze_raytracing_geometry_aabbs_fptr_ext_t getBounds;   ///< function pointer to return bounds for a range of primitives
  void* geomUserPtr;                                   ///< geometry user pointer passed to callback
  
} ze_raytracing_geometry_aabbs_fptr_ext_desc_t;


/**

  \brief Geometry descriptor of an instance

  Instance geometry descriptor which contains a pointer to an
  acceleration structure to instantiate, a pointer to the object to
  world transformation used for instancing, and a pointer to the
  object space bounding box of the instantiated acceleration
  structure.
*/
typedef struct ze_raytracing_geometry_instance_ext_desc_t  // 32 bytes
{
  ze_raytracing_geometry_type_ext_t geometryType;          ///< must be ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE
  ze_raytracing_instance_ext_flags_t instanceFlags;        ///< flags for the instance (see ze_raytracing_instance_ext_flags_t)
  uint8_t geometryMask;                                    ///< 8-bit geometry mask for ray masking
  ze_raytracing_transform_format_ext_t transformFormat;    ///< format of the specified transformation
  unsigned int instanceUserID;                             ///< a user specified identifier for the instance
  float* transform;                                        ///< object to world instance transformation in specified format
  ze_raytracing_aabb_ext_t* bounds;                        ///< AABB of the instanced acceleration structure
  void* accel;                                             ///< pointer to acceleration structure to instantiate
    
} ze_raytracing_geometry_instance_ext_desc_t;


/**
  \brief Bitmask with features supported. 
*/
#if 0
typedef enum ze_raytracing_features_ext_t {
  ZE_RAYTRACING_FEATURE_EXT_NONE = 0,
  ZE_RAYTRACING_FEATURE_EXT_GEOMETRY_TYPE_TRIANGLES  = 1 << 0,    ///< support for ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES geometries 
  ZE_RAYTRACING_FEATURE_EXT_GEOMETRY_TYPE_QUADS      = 1 << 1,    ///< support for ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS geometries
  ZE_RAYTRACING_FEATURE_EXT_GEOMETRY_TYPE_AABBS_FPTR = 1 << 2,    ///< support for ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR geometries
  ZE_RAYTRACING_FEATURE_EXT_GEOMETRY_TYPE_INSTANCE   = 1 << 3,    ///< support for ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE geometries
  
} ze_raytracing_features_ext_t;
#else
typedef enum ze_device_raytracing_ext_flag_t {
  ZE_DEVICE_RAYTRACING_EXT_FLAG_GEOMETRY_TYPE_TRIANGLES  = 1 << 0,    ///< support for ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES geometries 
  ZE_DEVICE_RAYTRACING_EXT_FLAG_GEOMETRY_TYPE_QUADS      = 1 << 1,    ///< support for ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS geometries
  ZE_DEVICE_RAYTRACING_EXT_FLAG_GEOMETRY_TYPE_AABBS_FPTR = 1 << 2,    ///< support for ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR geometries
  ZE_DEVICE_RAYTRACING_EXT_FLAG_GEOMETRY_TYPE_INSTANCE   = 1 << 3,    ///< support for ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE geometries
};
#endif

/**

 \brief Additional ze_result_t enum fields. 

*/
typedef enum ze_result_t
{
  ZE_RESULT_SUCCESS,                             ///< operation was successfull
  ZE_RESULT_ERROR_UNKNOWN,                       ///< unknown error occurred
  
  ZE_RESULT_RAYTRACING_EXT_RETRY_BUILD_ACCEL,    ///< acceleration structure build ran out of memory, app should re-try with more memory
  ZE_RESULT_RAYTRACING_EXT_OPERATION_DEFERRED,   ///< operation is deferred into ray tracing parallel operation
  
} ze_raytracing_result_t;


/**

   \brief Build quality hint for acceleration structure build. 

   Depending on use case different quality modes for acceleration structure build are supported.

   A low quality build builds an accelertion structure fast, but at
   some reduction in ray tracing performance. This mode is recommended
   for dynamic content, such as animated characters.

   A medium quality build uses a compromise between build quality and
   ray tracing performance. This mode should be used by default.

   Higher ray tracing performance can get achieved by using a high
   quality build, but acceleration structure build performance might
   be significantly reduced.

*/
typedef enum ze_raytracing_build_quality_ext_t
{
  ZE_RAYTRACING_BUILD_QUALITY_EXT_LOW    = 0,   ///< build low quality acceleration structure (fast)
  ZE_RAYTRACING_BUILD_QUALITY_EXT_MEDIUM = 1,   ///< build medium quality acceleration structure (slower)
  ZE_RAYTRACING_BUILD_QUALITY_EXT_HIGH   = 2,   ///< build high quality acceleration structure (slow)
  
} ze_raytracing_build_quality_ext_t;


/**

  \brief Flags for acceleration structure build. 

  These flags allow the application to tune the accelertion structure
  build to optimize for dynamic content
  (ZE_RAYTRACING_BUILD_EXT_FLAG_DYNAMIC) or to create more compact
  acceleration structures (ZE_RAYTRACING_BUILD_EXT_FLAG_COMPACT). Usage
  of any of these flags may reduce ray tracing performance.

  The acceleration structure build implementation might choose to use
  spatial splitting to split large or long primitives into smaller
  pieces. This resultsx in any-hit shaders being invoked multiple
  times for non-opaque primitives. If the application requires only a
  single any-hit shader invokation per primitive, the
  ZE_RAYTRACING_BUILD_EXT_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION build flag
  should get used.

*/
typedef enum ze_raytracing_build_ext_flags_t
{
  ZE_RAYTRACING_BUILD_EXT_FLAG_NONE    = 0,         ///< default build flags
  ZE_RAYTRACING_BUILD_EXT_FLAG_DYNAMIC = (1 << 0),  ///< optimize for dynamic content
  ZE_RAYTRACING_BUILD_EXT_FLAG_COMPACT = (1 << 1),  ///< build more compact acceleration structure
  ZE_RAYTRACING_BUILD_EXT_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION = (1 << 2), ///< guarantees single any hit shader invokation per primitive
  
} ze_raytracing_build_ext_flags_t;


/**

  \brief A handle of a parallel operation that can get joined with worker threads. 

  A parallel operation object can get used to perform acceleration
  structure build operations in parallel on multiple hardware
  thread. Therefore, a parallel build operation attached to an
  acceleration strucutre build can get joined by application manged
  threads to assist in building the acceleration structure.

*/
typedef ze_raytracing_parallel_operation_opaque_ext_handle_t* ze_raytracing_parallel_operation_ext_handle_t;


/**

   \brief Returns information about acceleration structure size estimates

   This structure is returned by zeRaytracingGetAccelSizeExt and
   contains acceleration structure size estimates.
*/
typedef struct ze_raytracing_accel_size_ext_t
{
  /** [in] type of this structure */
  ze_structure_type_t stype;

  /** [in,out][optional] must be null or a pointer to an extension-specific structure */
  void* pNext;                                    
  
  /** 
      [out] The expected number of bytes required for the acceleration
      structure. When using an acceleration structure buffer of that
      size, the build is expected to succeed mostly, but it may fail
      with ZE_RESULT_RAYTRACING_EXT_RETRY_BUILD_ACCEL. 
  */
  size_t accelBufferExpectedBytes;

  /**
     [out] The worst case number of bytes required for the acceleration
     structure. When using an acceleration structure buffer of that
     size, the build is guaranteed to not run out of memory.
  */
  size_t accelBufferWorstCaseBytes;

  /**
     [out] The scratch buffer bytes required for the acceleration
     structure build.
  */
  size_t scratchBufferBytes;
  
} ze_raytracing_accel_size_ext_t;


/** 
   \brief Argument structure passed to zeRaytracingBuildAccelExt function.
*/
typedef struct ze_raytracing_build_accel_args_ext_t
{
  /** [in] type of this structure */
  ze_structure_type_t stype;

  /** [in,out][optional] must be null or a pointer to an extension-specific structure */
  void* pNext;
  
  /** 
      Array of pointers to geometry descriptors. This array and the
      geometry descriptors themselves can be standard host memory
      allocations. A pointer to a geometry descriptor can be null, in
      which case the geometry is treated as empty. 
  */
  const ze_raytracing_geometry_ext_desc_t** geometries;

  /**
     Number of geometries in geometry descriptor array. 
  */
  uint32_t numGeometries;

  /**
     Destination buffer for acceleration structure. This has to be a
     shared memory allocation aligned to
     ZE_RAYTRACING_ACCELERATION_STRUCTURE_ALIGNMENT bytes and using the ray
     tracing allocation descriptor
     (ze_raytracing_mem_alloc_ext_ext_desc_t) in the zeMemAllocShared
     call. 
  */
  void* accelBuffer;

  /** 
      Number of allocated bytes of the acceleration structure
      buffer. This can be 0 in which case the build implementation may
      just return an improved accel buffer expected size estimate by
      looking at the scene data. 
  */
  size_t accelBufferBytes;

  /**
     Scratch space buffer to be used during accleration structure
     construction. Can be a standard host allocation. 
  */
  void* scratchBuffer;

  /**
     Number of allocated bytes of the scratch space buffer. 
  */
  size_t scratchBufferBytes;

  /** 
      Build quality to use (see ze_raytracing_build_quality_ext_t) 
  */
  ze_raytracing_build_quality_ext_t quality;

  /**
     Some hints for acceleration structure build (see ze_raytracing_build_ext_flags_t) 
  */
  ze_raytracing_build_ext_flags_t flags;

  /**
     [in][optional] When parallelOperation is NULL, the build is
     executed sequentially on the current thread. If a
     parallelOperation is specified, then the parallel operation gets
     attached to the parallel build handle. This handle can then get
     joined with worker threads to perform the parallel build
     operation. Only a single build operation can be attached to a
     parallel build operation at a given time.
  */
  ze_raytracing_parallel_operation_ext_handle_t parallelOperation;

  /**
     A pointer passed to callbacks. 
  */
  void* buildUserPtr;
  
  /**
     [out][optional] When the pointer is NULL no data is
     returned. When the build fails with
     ZE_RESULT_RAYTRACING_EXT_RETRY_BUILD_ACCEL, this returns new
     expected acceleration structure bytes to be used for
     re-build. When the build succeeds this returns the number of
     bytes used in the acceleration structure buffer.
  */
  size_t* accelBufferBytesOut;
  
  /**
     [out][optional] Destination address to write acceleration
     structure bounds to (can be NULL).
  */
  ze_raytracing_aabb_ext_t* boundsOut;
  
} ze_raytracing_build_accel_args_ext_t;


/*
 * Returns features supported by the implementation.
 */

//ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingGetSupportedFeaturesExt( ze_device_handle_t deviceID, ze_raytracing_features_ext_t* pRaytracingFeatures );

/**

  \brief Calculates size of buffers required for acceleration structure build.

  The zeRaytracingGetAccelSize function calculates the size of buffers
  required for the acceleration structure build. The function may look
  at the number of geometries, the geometry descriptors (but not
  referenced data), the existence of an acceleration structure to
  link, the build quality, and build flags of the build operation.

   \param args: Specifies the build operation to estimate the buffer sizes for. 
   \param pAccelSizeOut: A structure that is filled with the returned size information.

*/

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingGetAccelSizeExt( const ze_raytracing_build_accel_args_t* args, ze_raytracing_accel_size_ext_t* pAccelSizeOut );


/**
   
   \brief Builds a ray tracing acceleration structure

   The zeRaytracingBuildAccelExt function builds an acceleration
   structure of the scene consisting of the specified geometry
   descriptors and writes the acceleration structure to the provided
   destination buffer. All types of geometries can get freely mixed
   inside a scene. Please see ze_raytracing_build_accel_args_ext_t for
   a description of all input arguments.

   It is the users responsibility to manage the acceleration structure
   buffer allocation, deallocation, and potential prefetching to the
   device memory. The required size of the accleration structure
   buffer can be queried with the zeRaytracingGetAccelSizeExt
   function. The acceleration structure buffer must be a shared USM
   allocation and should be present on the host at build time. The
   referenced scene data (index and vertex buffers) can be standard
   host allocations, and will not be referenced into by the build
   acceleration structure.
   
   Before an accleration structure is build, the user has to use the
   zeRaytracingGetAccelSize function to query the size of the
   acceleration structure and scratch space buffer to allocate. When
   using the worst case size to allocate the acceleration structure
   buffer, then the zeRaytracingBuildAccelExt function will never fail
   with an out of memory error. When using the expected size for the
   acceleration structure buffer, then the build may fail with
   ZE_RESULT_RAYTRACING_EXT_RETRY_BUILD_ACCEL and should then get
   re-tried with a larger amount of data. Using the
   accelBufferBytesOut argument, the build itself returns an improved
   size estimate which the user can use to re-try the build.
   
   The acceleration structure build is executed on the host CPU and is
   synchronous, thus after the function returns with ZE_RESULT_SUCCESS
   the acceleration structure can get used. All provided data buffers
   have to be accessible from the host, thus no device memory can get
   passed. Unified shared memory has to get used for the acceleration
   structure buffer, for the builder to be able to write to it and the
   device to read from that buffer. 

   The constructed accleration structure is self contained, thus does
   not require the provided build data to be present after the build
   is done.

   The constructed acceleration structure cannot get copied into a
   different memory buffer as it may contain pointers.

   The acceleration build operation can get parallelized by passing a
   parallel operation and joining that parallel operation using the
   zeRaytracingParallelOperationJoinExt call with user provided worker
   threads. When a parallel operation is provided the
   zeRaytracingGetAccelSizeExt function returns with result
   ZE_RESULT_RAYTRACING_EXT_OPERATION_DEFERRED to indicate that the
   build operation is still in progress. The return value of the
   zeRaytracingParallelOperationJoinExt function determines the status
   of the parallel acceleration structure build.

 */

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingBuildAccelExt( const ze_raytracing_build_accel_args_ext_t* args );

/**

   \brief Create new parallel operation

   Creates a new parallel operation that can get attached to a build
   operation. Only a single build operation can be attached to a
   parallel operation at a given time.
*/

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingParallelOperationCreateExt( ze_raytracing_parallel_operation_ext_handle_t* phParallelOperation );

/**
  \brief Destroy parallel operation.
*/

ZE_APIEXPORT ze_result ZE_APICALL zeRaytracingParallelOperationDestroyExt( ze_raytracing_parallel_operation_ext_handle_t hParallelOperation );

/**
  \brief Returns the maximal number of threads that can join the parallel operation.
*/

ZE_APIEXPORT ze_result ZE_APICALL zeRaytracingParallelOperationGetMaxConcurrencyExt( ze_raytracing_parallel_operation_ext_handle_t hParallelOperation, uint32_t* pMaxConcurency );


/**
   \brief Join parallel operation

   Called by worker threads to join a parallel build operation. When
   the build finished, all worker threads return from this function
   with the same error code of the build.
*/

ZE_APIEXPORT ze_result_t ZE_APICALL zeRaytracingParallelOperationJoinExt( ze_raytracing_parallel_operation_ext_handle_t hParallelOperation );
