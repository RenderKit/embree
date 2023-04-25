// Copyright 2009-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stddef.h>
#include <stdint.h>

#if !defined(EMBREE_LEVEL_ZERO)
struct _ze_device_handle_t {};
#else
struct _ze_device_handle_t;
#endif

typedef struct _ze_device_handle_t *ze_device_handle_t;

#if !defined(EMBREE_LEVEL_ZERO)
struct _ze_driver_handle_t {};
#else
struct _ze_driver_handle_t;
#endif

typedef struct _ze_driver_handle_t *ze_driver_handle_t;

#if defined(__cplusplus)
#  define RTHWIF_API_EXTERN_C extern "C"
#else
#  define RTHWIF_API_EXTERN_C
#endif

#if defined(_WIN32)
#if defined(EMBREE_RTHWIF_STATIC_LIB)
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C
#else
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C __declspec(dllimport)
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C __declspec(dllexport)
#endif
#else
#  define RTHWIF_API_IMPORT RTHWIF_API_EXTERN_C
#  define RTHWIF_API_EXPORT RTHWIF_API_EXTERN_C __attribute__ ((visibility ("default")))
#endif

#if defined(RTHWIF_EXPORT_API)
#  define RTHWIF_API RTHWIF_API_EXPORT
#else
#  define RTHWIF_API RTHWIF_API_IMPORT
#endif

#define ZE_APICALL_

/**
   \file

   \brief This file contains a Level Zero API extension to build ray
   tracing acceleration structures to be used with an OpenCL ray
   tracing API extension.


   Ray Tracing Acceleration Structure Build API
   ============================================

   The L0 ray tracing acceleration structure build API provides
   functionality to build acceleration structures for 3D scenes on the
   host, which can get used for ray tracing on GPU devices.

   It is the users responsibility to manage the acceleration structure
   buffer allocation and deallocation. The required size of the
   accleration structure buffer can be queried with the
   zeRaytracingGetAccelSizeExt function and the acceleration structure
   an get build on the host using the zeRaytracingBuildAccelExt
   function.

   To build an acceleration structure one first has to setup a scene
   that consists of multiple geometry descriptors, such as
   ze_raytracing_geometry_triangles_ext_desc_t for triangle meshes,
   ze_raytracing_geometry_quads_ext_desc_t for quad meshes,
   ze_raytracing_geometry_aabbs_fptr_ext_desc_t for procedural
   primitives with attached axis aligned bounding box, and
   ze_raytracing_geometry_instance_ext_desc_t for instances of other
   acceleration structures.

   The followign example creates an
   ze_raytracing_geometry_triangles_ext_desc_t descriptor to specify
   a triangle mesh with triangles indices and vertices stored
   inside two vectors:

     ze_raytracing_geometry_triangles_ext_desc_t mesh;
     memset(&mesh,0,sizeof(mesh));
     mesh.geometryType = ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES;
     mesh.geometryFlags = ZE_RAYTRACING_GEOMETRY_EXT_FLAG_OPAQUE;
     mesh.geometryMask = 0xFF;

     mesh.triangleFormat = ZE_RAYTRACING_FORMAT_EXT_TRIANGLE_INDICES_UINT32;
     mesh.triangleCount = triangles.size();
     mesh.triangleStride = 12;
     mesh.triangleBuffer = triangles.data();

     mesh.vertexFormat = ZE_RAYTRACING_FORMAT_EXT_FLOAT3;
     mesh.vertexCount = vertices.size();
     mesh.vertexStride = 12;
     mesh.vertexBuffer = vertices.data();
   
  The specified geometry flag ZE_RAYTRACING_GEOMETRY_EXT_FLAG_OPAQUE
  enables a fast mode where traveral does not return to the caller of
  ray tracing for each hit. The proper data formats of the triangle
  and vertex buffer is specified, including the strides, and pointer
  to first elements in these buffers.
  
  To refer to multiple of these geometries that make a scene, pointers to
  these descriptors can be put into an array as follows:

    std::vector<ze_raytracing_geometry_ext_desc_t*> geometries;
    geometries.push_back((ze_raytracing_geometry_ext_desc_t*)&mesh);
    geometries.push_back((ze_raytracing_geometry_ext_desc_t*)&mesh1);
    ...

  This completes the definition of the geometry for the scene to
  construct the acceleration structure for.

  Next we need to query the format of the acceleration structure of
  the device we want to use for ray tracing:

    ze_raytracing_accel_format_ext_t accelFormat;
    ze_result_t result = zeRaytracingDeviceGetAccelFormatExt( hDevice, &accelFormat );

  To initiate the BVH build
  one first fills the ze_raytracing_build_accel_ext_desc_t structure
  with all arguments required for the acceleration structure build as
  in the following example:
    
    ze_raytracing_build_accel_ext_desc_t build_desc;
    memset(&build_desc,0,sizeof(build_desc));
    build_desc.stype = ZE_STRUCTURE_TYPE_RAYTRACING_BUILD_ACCEL_EXT_DESC;
    build_desc.pNext = nullptr;
    build_desc.accelFormat = accelFormat;
    build_desc.geometries = geometries.data(); 
    build_desc.numGeometries = geometries.size();
    build_desc.quality = ZE_RAYTRACING_BUILD_QUALITY_EXT_MEDIUM;
    build_desc.flags = ZE_RAYTRACING_BUILD_EXT_FLAG_NONE;

  Besides just passing a pointer to the geometries array this, passes
  the desired acceleration structure format, and sets some default
  build flags for a medium quality acceleration structure. Next the
  application has to query the buffer sizes required for the
  acceleration strucuture and scratch memory required for the build
  like in this example:

    ze_raytracing_accel_size_ext_properties_t accel_size;
    memset(&accel_size,0,sizeof(accel_size));
    accel_size.stype = ZE_STRUCTURE_TYPE_RAYTRACING_ACCEL_SIZE_EXT_PROPERTIES;
    accel_size.pNext = nullptr;

    ze_result_t result = zeRaytracingGetAccelSizeExt( &build_desc, &accel_size );
    assert(result == ZE_RESULT_SUCCESS);

  This queries the buffer sizes for the build operation specified in
  the provided build descriptor. Note that the sizes are only correct
  for the provided build descriptor, thus if parameters such as the
  build quality get changed for the actual build, calculated sizes are
  invalid.

  Now we allocate the scratch buffer:

    void* scratchBuffer = malloc(accel_size.scratchBufferBytes);

  and the acceleration structure buffer using the worst cast
  estimates:
  
    ze_raytracing_mem_alloc_ext_desc_t rt_desc;
    rt_desc.stype = ZE_STRUCTURE_TYPE_DEVICE_RAYTRACING_EXT_PROPERTIES;
    rt_desc.pNext = nullptr;
    rt_desc.flags = 0;

    ze_device_mem_alloc_desc_t device_desc;
    device_desc.stype = ZE_STRUCTURE_TYPE_DEVICE_MEM_ALLOC_DESC;
    device_desc.pNext = &rt_desc;
    device_desc.flags = ZE_DEVICE_MEM_ALLOC_FLAG_BIAS_CACHED;
    device_desc.ordinal = 0;
  
    ze_host_mem_alloc_desc_t host_desc;
    host_desc.stype = ZE_STRUCTURE_TYPE_HOST_MEM_ALLOC_DESC;
    host_desc.pNext = nullptr;
    host_desc.flags = ZE_HOST_MEM_ALLOC_FLAG_BIAS_CACHED;
    
    void* accelBuffer = nullptr;
    ze_result_t result = zeMemAllocShared(hContext,
                                          &device_desc, &host_desc,
                                          accel_size.accelBufferWorstCaseBytes, 
                                          ZE_RAYTRACING_ACCELERATION_STRUCTURE_ALIGNMENT_EXT,
                                          hDevice,&accelBuffer);
    assert(result == ZE_RESULT_SUCCESS);

    
  When using worst case estimate for the acceleration structure
  buffer, the build of the acceleration structure is guaranteed not to
  run out of memory.

  To initiate the build operation we need to additionally specify the
  just created buffers in the build descriptor:

    build_desc.accelBuffer = accelBuffer;
    build_desc.accelBufferBytes = accel_size.accelBufferWorstCaseBytes;

    build_desc.scratchBuffer = scratchBuffer;
    build_desc.scratchBufferBytes = accelsize.scratchBufferBytes;


  Now a single threaded build on the host CPU can get initiated using
  the zeRaytracingBuildAccelExt function:

    ze_result_t result = zeRaytracingBuildAccelExt( &build_desc );    
    assert(result == ZE_RESULT_SUCCESS);

  When the build completes successfully the acceleration structure
  buffer can get used as acceleration structure by the ray tracing
  API.


  Parallel Build
  --------------

  In order to speed up the build operation with multiple worker
  thread, some parallel operation object can get attached to the build
  and joined with application provided worker threads as in the
  following example:

     ze_raytracing_parallel_operation_ext_handle_t hParallelOperation;
     ze_result_t result = zeRaytracingParallelOperationCreateExt(&hParallelOperation);
     assert(result == ZE_RESULT_SUCCESS);

     build_desc.parallelOperation = hParallelOperation;

     result = zeRaytracingBuildAccelExt( &build_desc );    
     assert(result == ZE_RESULT_RAYTRACING_EXT_OPERATION_DEFERRED);

     uint32_t pMaxConcurrency = 0;
     result = zeRaytracingParallelOperationGetMaxConcurrencyExt( hParallelOperation, &MaxConcurrency );
     assert(result == ZE_RESULT_SUCCESS);

     tbb::parallel_for(0u,pMaxConcurrency,1u,[&](uint32_t i) {
       ze_result_t result = zeRaytracingParallelOperationJoinExt(hParallelOperation);
       assert(result == ZE_RESULT_SUCCESS);
     });
     
     result = zeRaytracingParallelOperationDestroyExt(hParallelOperation);
     assert(result == ZE_RESULT_SUCCESS);


   Conservative Acceleration Buffer Size
   -------------------------------------

   While using the conservative size estimate of the acceleration
   structure guarantees a successfull build, the memory requirements
   are larger than typically required. To reduce memory usage the
   application can also use the expected acceleration buffer size and
   re-try the build operation in case it ran out of memory. The
   following code illustrates this concept:

     void* accelBuffer = nullptr;
     size_t accel_bytes = accel_size.accelBufferExpectedBytes;
    
     while (true)
     {
       accelBuffer = allocate_accel_buffer(accel_bytes);
  
       build_desc.accelBuffer = accelBuffer;
       build_desc.accelBufferBytes = accel_bytes;
       build_desc.accelBufferBytesOut = &accel_bytes;
  
       ze_result_t result = zeRaytracingBuildAccelExt( &build_desc );    
       if (result == ZE_RESULT_SUCCESS) break;

       assert(result == ZE_RESULT_RAYTRACING_EXT_RETRY_BUILD_ACCEL);
       // accel_bytes got increased to a larger estimate for a next try

       free_accel_buffer(accelBuffer);
     }

  The loop starts with the expected acceleration buffer size, for
  which the build will mostly succeed. If the build runs out of memory
  the ZE_RESULT_RAYTRACING_EXT_RETRY_BUILD_ACCEL result is returned
  and the build can get re-tried with a larger acceleration structure
  buffer.

  The example above passes a pointer to the accel_bytes variable as
  accelBufferBytesOut input to the build operation. This causes the
  builder to write a larger acceleration structure size estimate to be
  used in the next try into the accel_bytes variable. Alternatively,
  the application can also increase the acceleration buffer size for
  the next try by some percentage, or just use the worst case size for
  a second try.

**/


/**

 \brief Additional ze_result_t enum fields. 

*/
typedef enum _ze_result_t_
{
  ZE_RESULT_SUCCESS_,                             ///< operation was successfull
  ZE_RESULT_ERROR_UNKNOWN_,                       ///< unknown error occurred
  ZE_RESULT_ERROR_INVALID_ARGUMENT_,              ///< generic error code for invalid arguments 
  
  ZE_RESULT_RAYTRACING_EXT_RETRY_BUILD_ACCEL,    ///< acceleration structure build ran out of memory, app should re-try with more memory
  ZE_RESULT_RAYTRACING_EXT_OPERATION_DEFERRED,   ///< operation is deferred to a parallel operation
  ZE_RESULT_RAYTRACING_EXT_ACCEL_INCOMPATIBLE,   ///< the tested devices have incompatible acceleration structures
  
} ze_result_t_;


/**

  \brief Additional ze_structure_type_t enum fields.

*/

typedef enum _ze_structure_type_t_
{
  ZE_STRUCTURE_TYPE_RAYTRACING_ACCEL_SIZE_EXT_PROPERTIES,  ///< ze_raytracing_accel_size_ext_properties_t
  ZE_STRUCTURE_TYPE_RAYTRACING_BUILD_ACCEL_EXT_DESC, ///< ze_raytracing_build_accel_ext_desc_t
  ZE_STRUCTURE_TYPE_RTAS_PARALLEL_OPERATION_EXP_PROPERTIES, ///< ze_rtas_parallel_operation_exp_properties_t
  
} ze_structure_type_t_;


/**
  \brief Feature bit for acceleration structure build API
*/
typedef enum _ze_device_raytracing_ext_flag_t_ {
  ZE_DEVICE_RAYTRACING_EXT_FLAG_ACCEL_BUILD,    ///< support for ray tracing acceleration structure build API
} ze_device_raytracing_ext_flag_t_;


typedef enum _ze_rtas_builder_exp_version_t
{
  ZE_RTAS_BUILDER_EXP_VERSION_1_0,
  
} ze_rtas_builder_exp_version_t;

typedef struct _ze_rtas_builder_exp_desc_t
{
  /** [in] type of this structure */
  ze_structure_type_t_ stype;

  /** [in,out][optional] must be null or a pointer to an extension-specific structure */
  const void* pNext;                                    

  ze_rtas_builder_exp_version_t builderVersion;
  
} ze_rtas_builder_exp_desc_t;

typedef struct _ze_rtas_builder_exp_handle_t *ze_rtas_builder_exp_handle_t;

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRTASBuilderCreateExp(ze_driver_handle_t hDriver, const ze_rtas_builder_exp_desc_t *pDescriptor, ze_rtas_builder_exp_handle_t *phBuilder);

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRTASBuilderDestroyExp(ze_rtas_builder_exp_handle_t hBuilder);
  
/**

  \brief A handle of a parallel operation that can get joined with worker threads. 

  A parallel operation object can get used to perform acceleration
  structure build operations in parallel on multiple hardware
  thread. Therefore, a parallel build operation attached to an
  acceleration strucutre build can get joined by application manged
  threads to assist in building the acceleration structure.

*/
typedef struct _ze_raytracing_parallel_operation_ext_handle_t* ze_raytracing_parallel_operation_ext_handle_t;


/**

   \brief Create new parallel operation

   Creates a new parallel operation that can get attached to a build
   operation. Only a single build operation can be attached to a
   parallel operation at a given time.
*/

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRaytracingParallelOperationCreateExt( ze_rtas_builder_exp_handle_t hBuilder, ze_raytracing_parallel_operation_ext_handle_t* phParallelOperation );


/**
  \brief Destroy parallel operation.
*/

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRaytracingParallelOperationDestroyExt( ze_raytracing_parallel_operation_ext_handle_t hParallelOperation );

typedef enum _ze_rtas_parallel_operation_exp_flags_t {
  ZE_RTAS_PARALLEL_OPERATION_EXP_FLAG_NONE = 0, 
} ze_rtas_parallel_operation_exp_flags_t;

typedef struct _ze_rtas_parallel_operation_exp_properties_t
{
  /** [in] type of this structure */
  ze_structure_type_t_ stype;

  /** [in,out][optional] must be null or a pointer to an extension-specific structure */
  const void* pNext;                                    

  ze_rtas_parallel_operation_exp_flags_t flags;

  uint32_t maxConcurrency;
  
} ze_rtas_parallel_operation_exp_properties_t;

/**
  \brief Returns the maximal number of threads that can join the parallel operation.
*/

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRaytracingParallelOperationGetMaxConcurrencyExt( ze_raytracing_parallel_operation_ext_handle_t hParallelOperation, ze_rtas_parallel_operation_exp_properties_t* pProperties );


/**
   \brief Join parallel operation

   Called by worker threads to join a parallel build operation. When
   the build finished, all worker threads return from this function
   with the same error code of the build.
*/

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRaytracingParallelOperationJoinExt( ze_raytracing_parallel_operation_ext_handle_t hParallelOperation );



/** Required alignment of acceleration structure buffer. */
#define ZE_RAYTRACING_ACCELERATION_STRUCTURE_ALIGNMENT_EXT 128


/**
 \brief Enumeration of geometry flags supported by the geometries.
*/
typedef uint8_t ze_raytracing_geometry_ext_flags_t;
typedef enum _ze_raytracing_geometry_ext_flag_t : uint8_t
{
  ZE_RAYTRACING_GEOMETRY_EXT_FLAG_NONE = 0,        ///< Default geometry flags
  ZE_RAYTRACING_GEOMETRY_EXT_FLAG_OPAQUE = 1,      ///< Opaque geometries do not invoke an anyhit shader
} ze_raytracing_geometry_ext_flag_t;

/**


 \brief Format of elements in data buffers. 

 The format is used to specify the format of elements of data buffer,
 such as the format of transformation used for instancing, or index
 format for triangles and quads.

*/
typedef enum _ze_raytracing_format_ext_t : uint8_t
{
  ZE_RAYTRACING_FORMAT_EXT_FLOAT3,                        ///< 3 component float vector (see ze_raytracing_float3_ext_t layout)
  ZE_RAYTRACING_FORMAT_EXT_FLOAT3X4_COLUMN_MAJOR,         ///< 3x4 affine transformation in column major format (see ze_raytracing_transform_float3x4_column_major_ext_t layout)
  ZE_RAYTRACING_FORMAT_EXT_FLOAT3X4_ALIGNED_COLUMN_MAJOR, ///< 3x4 affine transformation in column major format (see ze_raytracing_transform_float3x4_aligned_column_major_ext_t layout)
  ZE_RAYTRACING_FORMAT_EXT_FLOAT3X4_ROW_MAJOR,            ///< 3x4 affine transformation in row    major format (see ze_raytracing_transform_float3x4_row_major_ext_t layout)
  ZE_RAYTRACING_FORMAT_EXT_AABB,                          ///< 3 dimensional axis aligned bounding box (see ze_raytracing_aabb_ext_t layout)
  ZE_RAYTRACING_FORMAT_EXT_TRIANGLE_INDICES_UINT32,       ///< triangle indices of uint32 type (see ze_raytracing_triangle_indices_uint32_ext_t layout)
  ZE_RAYTRACING_FORMAT_EXT_QUAD_INDICES_UINT32,           ///< quad indices of uint32 type (see ze_raytracing_quad_indices_uint32_ext_t layout)
} ze_raytracing_format_ext_t;


/**
 \brief A 3-component short vector type. 
*/
typedef struct _ze_raytracing_float3_ext_t {
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
typedef struct _ze_raytracing_transform_float3x4_column_major_ext_t {
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
typedef struct _ze_raytracing_transform_float3x4_aligned_column_major_ext_t
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
typedef struct _ze_raytracing_transform_float3x4_row_major_ext_t
{
  float vx_x, vy_x, vz_x, p_x; ///< row 0 of 3x4 matrix
  float vx_y, vy_y, vz_y, p_y; ///< row 1 of 3x4 matrix
  float vx_z, vy_z, vz_z, p_z; ///< row 2 of 3x4 matrix
} ze_raytracing_transform_float3x4_row_major_ext_t;


/** 

  \brief Axis aligned 3-dimensional bounding box (AABB) with lower and
  upper bounds in each dimension.

*/
typedef struct _ze_raytracing_aabb_ext_t
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
typedef struct _ze_raytracing_triangle_indices_uint32_ext_t
{
  uint32_t v0;   ///< first index pointing to the first triangle vertex in vertex array
  uint32_t v1;   ///< second index pointing to the second triangle vertex in vertex array
  uint32_t v2;   ///< third index pointing to the third triangle vertex in vertex array
} ze_raytracing_triangle_indices_uint32_ext_t;


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
  second triangle by u = 1-u' and v = 1-v', yielding a piecewise
  linear parametrization.

*/
typedef struct _ze_raytracing_quad_indices_uint32_ext_t
{ 
  uint32_t v0;  ///< first index pointing to the first quad vertex in vertex array
  uint32_t v1;  ///< second index pointing to the second quad vertex in vertex array
  uint32_t v2;  ///< third index pointing to the third quad vertex in vertex array
  uint32_t v3;  ///< forth index pointing to the forth quad vertex in vertex array
} ze_raytracing_quad_indices_uint32_ext_t;


/**

  \brief The types of geometries supported. 

  A type that identifies the various geometry descriptors supported by
  the API. Each geometry descriptor has this type stored as it first
  member at offset 0 to identify the type of the descriptor.

*/
typedef enum _ze_raytracing_geometry_type_ext_t : uint8_t
{
  ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES = 0,   ///< triangle mesh geometry type identifying ze_raytracing_geometry_triangles_ext_desc_t
  ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS = 1,       ///< quad mesh geometry type identifying ze_raytracing_geometry_quads_ext_desc_t
  ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR = 2,  ///< procedural geometry type identifying ze_raytracing_geometry_aabbs_fptr_ext_desc_t
  ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE = 3,    ///< instance geometry type identifying ze_raytracing_geometry_instance_ext_desc_t
} ze_raytracing_geometry_type_ext_t;


/**

  \brief Instance flags supported

  This enumation lists flags to be used to specify instances.

*/
typedef uint8_t ze_raytracing_instance_ext_flags_t;
typedef enum _ze_raytracing_instance_ext_flag_t : uint8_t
{
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_NONE = 0,                               ///< default instance flag
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_TRIANGLE_CULL_DISABLE = 0x1,            ///< disables culling of front and backfacing triangles
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE = 0x2,  ///< reverses front and back face of triangles
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_FORCE_OPAQUE = 0x4,                     ///< forces instanced geometry to be opaque, unless ray flag forces it to be non-opaque
  ZE_RAYTRACING_INSTANCE_EXT_FLAG_FORCE_NON_OPAQUE = 0x8                  ///< forces instanced geometry to be non-opaque, unless ray flag forces it to be opaque
} ze_raytracing_instance_ext_flag_ext_t;

/**

 \brief Base structure for all geometry descriptors.

 Each geometry descriptor has a geometry type as it's first argument
 that identifies which geometry is represented. See
 ze_raytracing_geometry_type_ext_t for details.

*/
typedef struct _ze_raytracing_geometry_ext_desc_t {
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
typedef struct _ze_raytracing_geometry_triangles_ext_desc_t  // 40 bytes
{
  ze_raytracing_geometry_type_ext_t geometryType;       ///< must be ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES
  ze_raytracing_geometry_ext_flags_t geometryFlags;     ///< geometry flags for all primitives of this geometry
  uint8_t geometryMask;                                 ///< 8-bit geometry mask for ray masking
  uint8_t reserved0;                                    ///< must be zero
  uint8_t reserved1;                                    ///< must be zero
  uint8_t reserved2;                                    ///< must be zero
  ze_raytracing_format_ext_t triangleFormat;            ///< format of triangleBuffer (must be ZE_RAYTRACING_FORMAT_EXT_TRIANGLE_INDICES_UINT32)
  ze_raytracing_format_ext_t vertexFormat;              ///< format of vertexBuffer (must be ZE_RAYTRACING_FORMAT_EXT_FLOAT3)
  unsigned int triangleCount;                           ///< number of triangles in triangleBuffer
  unsigned int vertexCount;                             ///< number of vertices in vertexBuffer
  unsigned int triangleStride;                          ///< stride in bytes of triangles in triangleBuffer
  unsigned int vertexStride;                            ///< stride in bytes of vertices in vertexBuffer
  void* triangleBuffer;                                 ///< pointer to array of triangle indices in specified format
  void* vertexBuffer;                                   ///< pointer to array of triangle vertices in specified format
  
} ze_raytracing_geometry_triangles_ext_desc_t;


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
  second triangle by u = 1-u' and v = 1-v', yielding a piecewise
  linear parametrization.

*/
typedef struct _ze_raytracing_geometry_quads_ext_desc_t // 40 bytes
{
  ze_raytracing_geometry_type_ext_t geometryType;   ///< must be ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS
  ze_raytracing_geometry_ext_flags_t geometryFlags; ///< geometry flags for all primitives of this geometry
  uint8_t geometryMask;                             ///< 8-bit geometry mask for ray masking
  uint8_t reserved0;                                ///< must be zero
  uint8_t reserved1;                                ///< must be zero
  uint8_t reserved2;                                ///< must be zero
  ze_raytracing_format_ext_t quadFormat;            ///< format of quadBuffer (must be ZE_RAYTRACING_FORMAT_EXT_QUAD_INDICES_UINT32)
  ze_raytracing_format_ext_t vertexFormat;          ///< format of vertexBuffer (must be ZE_RAYTRACING_FORMAT_EXT_FLOAT3)
  unsigned int quadCount;                           ///< number of quads in quadBuffer
  unsigned int vertexCount;                         ///< number of vertices in vertexBuffer
  unsigned int quadStride;                          ///< stride in bytes of quads in quadBuffer
  unsigned int vertexStride;                        ///< stride in bytes of vertices in vertexBuffer
  void* quadBuffer;                                 ///< pointer to an array of quad indices in specified format
  void* vertexBuffer;                               ///< pointer to an array of quad vertices in specified format
  
} ze_raytracing_geometry_quads_ext_desc_t;


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
typedef struct _ze_raytracing_geometry_aabbs_fptr_ext_desc_t // 24 bytes
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
typedef struct _ze_raytracing_geometry_instance_ext_desc_t  // 32 bytes
{
  ze_raytracing_geometry_type_ext_t geometryType;          ///< must be ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE
  ze_raytracing_instance_ext_flags_t instanceFlags;        ///< flags for the instance (see ze_raytracing_instance_ext_flags_t)
  uint8_t geometryMask;                                    ///< 8-bit geometry mask for ray masking
  ze_raytracing_format_ext_t transformFormat;              ///< format of the specified transformation
  unsigned int instanceUserID;                             ///< a user specified identifier for the instance
  void* transform;                                         ///< object to world instance transformation in specified format
  ze_raytracing_aabb_ext_t* bounds;                        ///< AABB of the instanced acceleration structure
  void* accel;                                             ///< pointer to acceleration structure to instantiate
    
} ze_raytracing_geometry_instance_ext_desc_t;


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
typedef enum _ze_raytracing_build_quality_ext_t
{
  ZE_RAYTRACING_BUILD_QUALITY_EXT_LOW    = 0,   ///< build low quality acceleration structure (fast)
  ZE_RAYTRACING_BUILD_QUALITY_EXT_MEDIUM = 1,   ///< build medium quality acceleration structure (slower)
  ZE_RAYTRACING_BUILD_QUALITY_EXT_HIGH   = 2,   ///< build high quality acceleration structure (slow)
  
} ze_raytracing_build_quality_ext_t;


/**

  \brief Flags for acceleration structure build. 

  These flags allow the application to tune the accelertion structure
  build.

  Using the ZE_RAYTRACING_BUILD_EXT_FLAG_COMPACT flag causes the
  acceleration strucuture to create more compact acceleration
  structures.

  The acceleration structure build implementation might choose to use
  spatial splitting to split large or long primitives into smaller
  pieces. This results in any-hit shaders being invoked multiple
  times for non-opaque primitives. If the application requires only a
  single any-hit shader invokation per primitive, the
  ZE_RAYTRACING_BUILD_EXT_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION build flag
  should get used.

  Usage of any of these flags may reduce ray tracing performance.

*/
typedef uint32_t ze_raytracing_build_ext_flags_t;
typedef enum _ze_raytracing_build_ext_flag_t
{
  ZE_RAYTRACING_BUILD_EXT_FLAG_NONE    = 0,                               ///< default build flags
  ZE_RAYTRACING_BUILD_EXT_FLAG_COMPACT = (1 << 0),                        ///< build more compact acceleration structure
  ZE_RAYTRACING_BUILD_EXT_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION = (1 << 1), ///< guarantees single any hit shader invokation per primitive
  
} ze_raytracing_build_ext_flag_t;


/**

  \brief An opaque ray tracing acceleration structure format supported
  by some device.

*/
typedef enum _ze_raytracing_accel_format_ext_t {
  ZE_RAYTRACING_ACCEL_FORMAT_EXT_INVALID = 0      // invalid acceleration structure format
} ze_raytracing_accel_format_ext_t;


/**

  \brief Returns the acceleration structure format supported by the specified device.

  \param hDevice: device to query the acceleration structure format for
  \param pAccelFormat: points to destination of returned acceleration structure format

  If a ray tracing is supported by this device a format is returned to
  the memory location pointed by pAccelFormat and the result code
  ZE_RESULT_SUCCESS is returned.

*/

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRaytracingDeviceGetAccelFormatExt( const ze_device_handle_t hDevice, ze_raytracing_accel_format_ext_t* pAccelFormat );



/**

  \brief Checks if the acceleration structure build for hDevice can be used on hDeviceOther.

  \param accelFormat: format the acceleration structure is build for
  \param otherAccelFormat: the format to test compatibility with

  The function returns ZE_RESULT_SUCCESS if an acceleration structure
  build with format `accelFormat` can also get used on devices with
  acceleration structure format `otherAccelFormat` Otherwise
  ZE_RESULT_RAYTRACING_EXT_ACCEL_INCOMPATIBLE is returned.

*/

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRaytracingAccelFormatCompatibilityExt( ze_rtas_builder_exp_handle_t hBuilder, const ze_raytracing_accel_format_ext_t accelFormat, const ze_raytracing_accel_format_ext_t otherAccelFormat );



/**

   \brief Returns information about acceleration structure size estimates

   This structure is returned by zeRaytracingGetAccelSizeExt and
   contains acceleration structure size estimates.
*/
typedef struct _ze_raytracing_accel_size_ext_properties_t
{
  /** [in] type of this structure */
  ze_structure_type_t_ stype;

  /** [in,out][optional] must be null or a pointer to an extension-specific structure */
  const void* pNext;                                    

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
  
} ze_raytracing_accel_size_ext_properties_t;


/** 
   \brief Argument structure passed to zeRaytracingBuildAccelExt function.
*/
typedef struct _ze_raytracing_build_accel_ext_desc_t
{
  /** [in] type of this structure */
  ze_structure_type_t_ stype;

  /** [in,out][optional] must be null or a pointer to an extension-specific structure */
  const void* pNext;

  /** [in] The acceleration structure format of a device to use for
   * ray tracing. The acceleration structure can also get used on
   * other devices whose acceleration structure is compatible with the
   * device specified here (see
   * zeRaytracingAccelFormatCompatibilityExt function). */
  ze_raytracing_accel_format_ext_t accelFormat;

  /** 
      [in] Build quality to use (see ze_raytracing_build_quality_ext_t) 
  */
  ze_raytracing_build_quality_ext_t quality;

  /**
     [in] Some build flags for acceleration structure build (see ze_raytracing_build_ext_flags_t) 
  */
  ze_raytracing_build_ext_flags_t flags;
  
  /** 
      [in] Array of pointers to geometry descriptors. This array and
      the geometry descriptors themselves have to be standard host
      memory allocations. A pointer to a geometry descriptor can be
      NULL, in which case the geometry is treated as empty.
  */
  const ze_raytracing_geometry_ext_desc_t** geometries;

  /**
     [in] Number of geometries in geometry descriptor array. 
  */
  uint32_t numGeometries;

  /* for debugging purposes use only */
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
  void* dispatchGlobalsPtr;
#endif
  
} ze_raytracing_build_accel_ext_desc_t;


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

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRaytracingGetAccelSizeExt( ze_rtas_builder_exp_handle_t hBuilder, const ze_raytracing_build_accel_ext_desc_t* args, ze_raytracing_parallel_operation_ext_handle_t hParallelOperation, ze_raytracing_accel_size_ext_properties_t* pAccelSizeOut );


/**
   
   \brief Builds a ray tracing acceleration structure

   The zeRaytracingBuildAccelExt function builds an acceleration
   structure of the scene consisting of the specified geometry
   descriptors and writes the acceleration structure to the provided
   destination buffer. All types of geometries can get freely mixed
   inside a scene. Please see ze_raytracing_build_accel_ext_desc_t for
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

RTHWIF_API ze_result_t_ ZE_APICALL_ zeRaytracingBuildAccelExt( ze_rtas_builder_exp_handle_t hBuilder, const ze_raytracing_build_accel_ext_desc_t* args,
  void *pScratchBuffer, size_t scratchBufferSizeBytes, void *pRtasBuffer, size_t rtasBufferSizeBytes, ze_raytracing_parallel_operation_ext_handle_t hParallelOperation, void *pBuildUserPtr, ze_raytracing_aabb_ext_t *pBounds, size_t *pRtasBufferSizeBytes);


//////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Initializes the library.
 */

RTHWIF_API void zeRaytracingInitExt();

/*
 * Cleans up the library.
 */

RTHWIF_API void zeRaytracingExitExt();
