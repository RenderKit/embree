// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#define RTHWIF_EXPORT_API

#include "rthwif_builder.h"
#include "qbvh6_builder_sah.h"

#if defined(EMBREE_LEVEL_ZERO)
#include <level_zero/ze_api.h>
#endif

namespace embree
{
  using namespace embree::isa;

  static std::unique_ptr<tbb::task_arena> g_arena;
  
  inline ze_raytracing_triangle_indices_uint32_ext_t getPrimitive(const ze_raytracing_geometry_triangles_ext_desc_t* geom, uint32_t primID) {
    assert(primID < geom->triangleCount);
    return *(ze_raytracing_triangle_indices_uint32_ext_t*)((char*)geom->triangleBuffer + uint64_t(primID)*geom->triangleStride);
  }
  
  inline Vec3f getVertex(const ze_raytracing_geometry_triangles_ext_desc_t* geom, uint32_t vertexID) {
    assert(vertexID < geom->vertexCount);
    return *(Vec3f*)((char*)geom->vertexBuffer + uint64_t(vertexID)*geom->vertexStride);
  }
  
  inline ze_raytracing_quad_indices_uint32_ext_t getPrimitive(const ze_raytracing_geometry_quads_ext_desc_t* geom, uint32_t primID) {
    assert(primID < geom->quadCount);
    return *(ze_raytracing_quad_indices_uint32_ext_t*)((char*)geom->quadBuffer + uint64_t(primID)*geom->quadStride);
  }
  
  inline Vec3f getVertex(const ze_raytracing_geometry_quads_ext_desc_t* geom, uint32_t vertexID) {
    assert(vertexID < geom->vertexCount);
    return *(Vec3f*)((char*)geom->vertexBuffer + uint64_t(vertexID)*geom->vertexStride);
  }

  inline AffineSpace3fa getTransform(const ze_raytracing_geometry_instance_ext_desc_t* geom)
  {
    switch (geom->transformFormat)
    {
    case ZE_RAYTRACING_FORMAT_EXT_FLOAT3X4_COLUMN_MAJOR: {
      const ze_raytracing_transform_float3x4_column_major_ext_t* xfm = (const ze_raytracing_transform_float3x4_column_major_ext_t*) geom->transform;
      return {
        { xfm->vx_x, xfm->vx_y, xfm->vx_z },
        { xfm->vy_x, xfm->vy_y, xfm->vy_z },
        { xfm->vz_x, xfm->vz_y, xfm->vz_z },
        { xfm-> p_x, xfm-> p_y, xfm-> p_z }
      };
    }
    case ZE_RAYTRACING_FORMAT_EXT_FLOAT3X4_ALIGNED_COLUMN_MAJOR: {
      const ze_raytracing_transform_float3x4_aligned_column_major_ext_t* xfm = (const ze_raytracing_transform_float3x4_aligned_column_major_ext_t*) geom->transform;
      return {
        { xfm->vx_x, xfm->vx_y, xfm->vx_z },
        { xfm->vy_x, xfm->vy_y, xfm->vy_z },
        { xfm->vz_x, xfm->vz_y, xfm->vz_z },
        { xfm-> p_x, xfm-> p_y, xfm-> p_z }
      };
    }
    case ZE_RAYTRACING_FORMAT_EXT_FLOAT3X4_ROW_MAJOR: {
      const ze_raytracing_transform_float3x4_row_major_ext_t* xfm = (const ze_raytracing_transform_float3x4_row_major_ext_t*) geom->transform;
      return {
        { xfm->vx_x, xfm->vx_y, xfm->vx_z },
        { xfm->vy_x, xfm->vy_y, xfm->vy_z },
        { xfm->vz_x, xfm->vz_y, xfm->vz_z },
        { xfm-> p_x, xfm-> p_y, xfm-> p_z }
      };
    }
    default:
      throw std::runtime_error("invalid transform format");
    }
  }
  
  inline void verifyGeometryDesc(const ze_raytracing_geometry_triangles_ext_desc_t* geom)
  {
    if (geom->triangleFormat != ZE_RAYTRACING_FORMAT_EXT_TRIANGLE_INDICES_UINT32)
      throw std::runtime_error("triangle format must be ZE_RAYTRACING_FORMAT_EXT_TRIANGLE_INDICES_UINT32");
    
    if (geom->vertexFormat != ZE_RAYTRACING_FORMAT_EXT_FLOAT3)
      throw std::runtime_error("vertex format must be ZE_RAYTRACING_FORMAT_EXT_FLOAT3");
 
    if (geom->reserved0 != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->reserved1 != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->reserved2 != 0) throw std::runtime_error("reserved member must be 0");
    
    if (geom->triangleCount && geom->triangleBuffer == nullptr) throw std::runtime_error("no triangle buffer specified");
    if (geom->vertexCount   && geom->vertexBuffer   == nullptr) throw std::runtime_error("no vertex buffer specified");
  }

  inline void verifyGeometryDesc(const ze_raytracing_geometry_quads_ext_desc_t* geom)
  {
    if (geom->quadFormat != ZE_RAYTRACING_FORMAT_EXT_QUAD_INDICES_UINT32)
      throw std::runtime_error("quad format must be ZE_RAYTRACING_FORMAT_EXT_QUAD_INDICES_UINT32");
    
    if (geom->vertexFormat != ZE_RAYTRACING_FORMAT_EXT_FLOAT3)
      throw std::runtime_error("vertex format must be ZE_RAYTRACING_FORMAT_EXT_FLOAT3");
 
    if (geom->reserved0 != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->reserved1 != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->reserved2 != 0) throw std::runtime_error("reserved member must be 0");
    
    if (geom->quadCount   && geom->quadBuffer   == nullptr) throw std::runtime_error("no quad buffer specified");
    if (geom->vertexCount && geom->vertexBuffer == nullptr) throw std::runtime_error("no vertex buffer specified");
  }

  inline void verifyGeometryDesc(const ze_raytracing_geometry_aabbs_fptr_ext_desc_t* geom)
  {
    if (geom->reserved != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->primCount   && geom->getBounds == nullptr) throw std::runtime_error("no bounds function specified");
  }

  inline void verifyGeometryDesc(const ze_raytracing_geometry_instance_ext_desc_t* geom)
  {
    if (geom->transform == nullptr) throw std::runtime_error("no instance transformation specified");
    if (geom->bounds == nullptr) throw std::runtime_error("no acceleration structure bounds specified");
    if (geom->accel == nullptr) throw std::runtime_error("no acceleration structure to instanciate specified");
  }

  inline bool buildBounds(const ze_raytracing_geometry_triangles_ext_desc_t* geom, uint32_t primID, BBox3fa& bbox, void* buildUserPtr)
  {
    if (primID >= geom->triangleCount) return false;
    const ze_raytracing_triangle_indices_uint32_ext_t tri = getPrimitive(geom,primID);
    if (unlikely(tri.v0 >= geom->vertexCount)) return false;
    if (unlikely(tri.v1 >= geom->vertexCount)) return false;
    if (unlikely(tri.v2 >= geom->vertexCount)) return false;
    
    const Vec3f p0 = getVertex(geom,tri.v0);
    const Vec3f p1 = getVertex(geom,tri.v1);
    const Vec3f p2 = getVertex(geom,tri.v2);
    if (unlikely(!isvalid(p0))) return false;
    if (unlikely(!isvalid(p1))) return false;
    if (unlikely(!isvalid(p2))) return false;
    
    bbox = BBox3fa(min(p0,p1,p2),max(p0,p1,p2));
    return true;
  }

  inline bool buildBounds(const ze_raytracing_geometry_quads_ext_desc_t* geom, uint32_t primID, BBox3fa& bbox, void* buildUserPtr)
  {
    if (primID >= geom->quadCount) return false;
    const ze_raytracing_quad_indices_uint32_ext_t tri = getPrimitive(geom,primID);
    if (unlikely(tri.v0 >= geom->vertexCount)) return false;
    if (unlikely(tri.v1 >= geom->vertexCount)) return false;
    if (unlikely(tri.v2 >= geom->vertexCount)) return false;
    if (unlikely(tri.v3 >= geom->vertexCount)) return false;
    
    const Vec3f p0 = getVertex(geom,tri.v0);
    const Vec3f p1 = getVertex(geom,tri.v1);
    const Vec3f p2 = getVertex(geom,tri.v2);
    const Vec3f p3 = getVertex(geom,tri.v3);
    if (unlikely(!isvalid(p0))) return false;
    if (unlikely(!isvalid(p1))) return false;
    if (unlikely(!isvalid(p2))) return false;
    if (unlikely(!isvalid(p3))) return false;
    
    bbox = BBox3fa(min(p0,p1,p2,p3),max(p0,p1,p2,p3));
    return true;
  }

  inline bool buildBounds(const ze_raytracing_geometry_aabbs_fptr_ext_desc_t* geom, uint32_t primID, BBox3fa& bbox, void* buildUserPtr)
  {
    if (primID >= geom->primCount) return false;
    if (geom->getBounds == nullptr) return false;

    BBox3f bounds;
    (geom->getBounds)(primID,1,geom->geomUserPtr,buildUserPtr,(ze_raytracing_aabb_ext_t*)&bounds);
    if (unlikely(!isvalid(bounds.lower))) return false;
    if (unlikely(!isvalid(bounds.upper))) return false;
    if (unlikely(bounds.empty())) return false;
    
    bbox = (BBox3f&) bounds;
    return true;
  }

  inline bool buildBounds(const ze_raytracing_geometry_instance_ext_desc_t* geom, uint32_t primID, BBox3fa& bbox, void* buildUserPtr)
  {
    if (primID >= 1) return false;
    if (geom->accel == nullptr) return false;
    if (geom->transform == nullptr) return false;
    
    const AffineSpace3fa local2world = getTransform(geom);
    const Vec3fa lower(geom->bounds->lower.x,geom->bounds->lower.y,geom->bounds->lower.z);
    const Vec3fa upper(geom->bounds->upper.x,geom->bounds->upper.y,geom->bounds->upper.z);
    const BBox3fa bounds = xfmBounds(local2world,BBox3fa(lower,upper));
     
    if (unlikely(!isvalid(bounds.lower))) return false;
    if (unlikely(!isvalid(bounds.upper))) return false;
    if (unlikely(bounds.empty())) return false;
    
    bbox = bounds;
    return true;
  }

  template<typename GeometryType>
  PrimInfo createGeometryPrimRefArray(const GeometryType* geom, void* buildUserPtr, evector<PrimRef>& prims, const range<size_t>& r, size_t k, unsigned int geomID)
  {
    PrimInfo pinfo(empty);
    for (uint32_t primID=r.begin(); primID<r.end(); primID++)
    {
      BBox3fa bounds = empty;
      if (!buildBounds(geom,primID,bounds,buildUserPtr)) continue;
      const PrimRef prim(bounds,geomID,primID);
      pinfo.add_center2(prim);
      prims[k++] = prim;
    }
    return pinfo;
  }
  
  RTHWIF_API void zeRaytracingInitExt()
  {
    uint32_t numThreads = tbb::this_task_arena::max_concurrency();
    g_arena.reset(new tbb::task_arena(numThreads,numThreads));
  }
  
  RTHWIF_API void zeRaytracingExitExt()
  {
    g_arena.reset();
  }

  typedef struct _zet_base_desc_t
  {
    /** [in] type of this structure */
    ze_structure_type_t_ stype;
    
    /** [in,out][optional] must be null or a pointer to an extension-specific structure */
    const void* pNext;
    
  } zet_base_desc_t_;

  bool checkDescChain(zet_base_desc_t_* desc)
  {
    /* supporting maximal 1024 to also detect cycles */
    for (size_t i=0; i<1024; i++) {
      if (desc->pNext == nullptr) return true;
      desc = (zet_base_desc_t_*) desc->pNext;
    }
    return false;
  }

  struct ze_rtas_builder
  {
    ze_rtas_builder () {
    }
    
    ~ze_rtas_builder() {
      magick = 0x0;
    }

    bool verify() const {
      return magick == MAGICK;
    }
    
    enum { MAGICK = 0x45FE67E1 };
    uint32_t magick = MAGICK;
  };

  bool validate(ze_rtas_builder_exp_handle_t hBuilder)
  {
    if (hBuilder == nullptr) return false;
    return ((ze_rtas_builder*)hBuilder)->verify();
  }

  RTHWIF_API ze_result_t_ zeRTASBuilderCreateExp(ze_driver_handle_t hDriver, const ze_rtas_builder_exp_desc_t *pDescriptor, ze_rtas_builder_exp_handle_t *phBuilder)
  {
    if (hDriver == nullptr)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    if (pDescriptor == nullptr)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    if (phBuilder == nullptr)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    if (!checkDescChain((zet_base_desc_t_*)pDescriptor))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    if (pDescriptor->builderVersion != ZE_RTAS_BUILDER_EXP_VERSION_1_0)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    *phBuilder = (ze_rtas_builder_exp_handle_t) new ze_rtas_builder();
    return ZE_RESULT_SUCCESS_;
  }

  RTHWIF_API ze_result_t_ zeRTASBuilderDestroyExp(ze_rtas_builder_exp_handle_t hBuilder)
  {
    if (!validate(hBuilder))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    delete (ze_rtas_builder*) hBuilder;
    return ZE_RESULT_SUCCESS_;
  }

  typedef enum _ze_raytracing_accel_format_internal_t {
    ZE_RAYTRACING_ACCEL_FORMAT_EXT_INVALID = 0,      // invalid acceleration structure format
    ZE_RAYTRACING_ACCEL_FORMAT_EXT_VERSION_1 = 1, // acceleration structure format version 1
    ZE_RAYTRACING_ACCEL_FORMAT_EXT_VERSION_2 = 2, // acceleration structure format version 2
  } ze_raytracing_accel_format_internal_t;

  RTHWIF_API ze_result_t_ zeRaytracingDeviceGetAccelFormatExt( const ze_device_handle_t hDevice, ze_rtas_device_exp_properties_t* pProperties )
  {
    if (pProperties == nullptr)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check valid pNext chain */
    if (!checkDescChain((zet_base_desc_t_*)pProperties))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;
    
    /* check for proper property */
    if (pProperties->stype != ZE_STRUCTURE_TYPE_RTAS_DEVICE_EXP_PROPERTIES)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* fill properties */
    pProperties->flags = ZE_RTAS_DEVICE_EXP_FLAG_NONE;
    pProperties->rtasDeviceFormat = (ze_raytracing_accel_format_ext_t) ZE_RAYTRACING_ACCEL_FORMAT_EXT_INVALID;
    pProperties->rtasBufferAlignment = 128;

#if defined(EMBREE_LEVEL_ZERO)

    /* check for supported device ID */
    ze_device_properties_t device_props{ ZE_STRUCTURE_TYPE_DEVICE_PROPERTIES };
    ze_result_t status = zeDeviceGetProperties(hDevice, &device_props);
    if (status != ZE_RESULT_SUCCESS)
      return ZE_RESULT_ERROR_UNKNOWN_;

    /* check for Intel vendor */
    const uint32_t vendor_id = device_props.vendorId;
    const uint32_t device_id = device_props.deviceId;
    if (vendor_id != 0x8086) return ZE_RESULT_ERROR_UNKNOWN_;
    
    bool dg2 =
      (0x4F80 <= device_id && device_id <= 0x4F88) ||
      (0x5690 <= device_id && device_id <= 0x5698) ||
      (0x56A0 <= device_id && device_id <= 0x56A6) ||
      (0x56B0 <= device_id && device_id <= 0x56B3) ||
      (0x56C0 <= device_id && device_id <= 0x56C1);

    bool pvc =
      (0x0BD0 <= device_id && device_id <= 0x0BDB) ||
      (device_id == 0x0BE5                       );

    if (dg2 || pvc) {
      pProperties->rtasDeviceFormat = (ze_raytracing_accel_format_ext_t) ZE_RAYTRACING_ACCEL_FORMAT_EXT_VERSION_1;
      return ZE_RESULT_SUCCESS_;
    }        

    return ZE_RESULT_ERROR_UNKNOWN_;

#else

    pProperties->rtasDeviceFormat = (ze_raytracing_accel_format_ext_t) ZE_RAYTRACING_ACCEL_FORMAT_EXT_VERSION_1;
    return ZE_RESULT_SUCCESS_;
    
#endif
  }
  
  RTHWIF_API ze_result_t_ zeRaytracingAccelFormatCompatibilityExt( ze_rtas_builder_exp_handle_t hBuilder,
                                                                   const ze_raytracing_accel_format_ext_t accelFormat,
                                                                   const ze_raytracing_accel_format_ext_t otherAccelFormat )
  {
    /* check for valid rtas builder */
    if (!validate(hBuilder))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;
    
    if (accelFormat != otherAccelFormat)
      return ZE_RESULT_RAYTRACING_EXT_ACCEL_INCOMPATIBLE;

    return ZE_RESULT_SUCCESS_;
  }

  uint32_t getNumPrimitives(const ze_raytracing_geometry_ext_desc_t* geom)
  {
    switch (geom->geometryType) {
    case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES  : return ((ze_raytracing_geometry_triangles_ext_desc_t*) geom)->triangleCount;
    case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR : return ((ze_raytracing_geometry_aabbs_fptr_ext_desc_t*) geom)->primCount;
    case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS      : return ((ze_raytracing_geometry_quads_ext_desc_t*) geom)->quadCount;
    case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE   : return 1;
    default                              : return 0;
    };
  }
  
  RTHWIF_API ze_result_t_ zeRaytracingGetAccelSizeExt(ze_rtas_builder_exp_handle_t hBuilder, const ze_raytracing_build_accel_ext_desc_t* args, ze_raytracing_parallel_operation_ext_handle_t hParallelOperation, ze_raytracing_accel_size_ext_properties_t* size_o)
  {
    /* check for valid rtas builder */
    if (!validate(hBuilder))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;
    
    /* check for valid pointers */
    if (args == nullptr)
      return ZE_RESULT_ERROR_UNKNOWN_;

    /* check for valid pointers */
    if (size_o == nullptr)
      return ZE_RESULT_ERROR_UNKNOWN_;
    
    /* check if input descriptor has proper type */
    if (args->stype != ZE_STRUCTURE_TYPE_RAYTRACING_BUILD_ACCEL_EXT_DESC)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check valid pNext chain */
    if (!checkDescChain((zet_base_desc_t_*)args))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check if return property has proper type */
    if (size_o->stype != ZE_STRUCTURE_TYPE_RAYTRACING_ACCEL_SIZE_EXT_PROPERTIES)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check valid pNext chain */
    if (!checkDescChain((zet_base_desc_t_*)size_o))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check if acceleration structure format is supported */
    if (args->accelFormat != (ze_raytracing_accel_format_ext_t) ZE_RAYTRACING_ACCEL_FORMAT_EXT_VERSION_1)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;
    
    const ze_raytracing_geometry_ext_desc_t** geometries = args->geometries;
    const size_t numGeometries = args->numGeometries;

    auto getSize = [&](uint32_t geomID) -> size_t {
      const ze_raytracing_geometry_ext_desc_t* geom = geometries[geomID];
      if (geom == nullptr) return 0;
      return getNumPrimitives(geom);
    };
    
    auto getType = [&](unsigned int geomID)
    {
      const ze_raytracing_geometry_ext_desc_t* geom = geometries[geomID];
      assert(geom);
      switch (geom->geometryType) {
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES : return QBVH6BuilderSAH::TRIANGLE;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS: return QBVH6BuilderSAH::QUAD;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR: return QBVH6BuilderSAH::PROCEDURAL;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE: return QBVH6BuilderSAH::INSTANCE;
      default: throw std::runtime_error("invalid geometry type");
      };
    };

    /* query memory requirements from builder */
    size_t expectedBytes = 0;
    size_t worstCaseBytes = 0;
    size_t scratchBytes = 0;
    QBVH6BuilderSAH::estimateSize(numGeometries, getSize, getType, args->quality, args->flags, expectedBytes, worstCaseBytes, scratchBytes);
    
    /* fill return struct */
    size_o->accelBufferExpectedBytes = expectedBytes;
    size_o->accelBufferWorstCaseBytes = worstCaseBytes;
    size_o->scratchBufferBytes = scratchBytes;
    return ZE_RESULT_SUCCESS_;
  }
  
  RTHWIF_API ze_result_t_ zeRaytracingBuildAccelExtInternal(const ze_raytracing_build_accel_ext_desc_t* args,
                                                            void *pScratchBuffer, size_t scratchBufferSizeBytes,
                                                            void *pRtasBuffer, size_t rtasBufferSizeBytes,
                                                            void *pBuildUserPtr, ze_raytracing_aabb_ext_t *pBounds, size_t *pRtasBufferSizeBytes) try
  {
    const ze_raytracing_geometry_ext_desc_t** geometries = args->geometries;
    const uint32_t numGeometries = args->numGeometries;

    /* verify input descriptors */
    parallel_for(numGeometries,[&](uint32_t geomID) {
      const ze_raytracing_geometry_ext_desc_t* geom = geometries[geomID];
      if (geom == nullptr) return;
      
      switch (geom->geometryType) {
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES  : verifyGeometryDesc((ze_raytracing_geometry_triangles_ext_desc_t*)geom); break;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS      : verifyGeometryDesc((ze_raytracing_geometry_quads_ext_desc_t*    )geom); break;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR : verifyGeometryDesc((ze_raytracing_geometry_aabbs_fptr_ext_desc_t*)geom); break;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE   : verifyGeometryDesc((ze_raytracing_geometry_instance_ext_desc_t* )geom); break;
      default: throw std::runtime_error("invalid geometry type");
      };
    });
    
    auto getSize = [&](uint32_t geomID) -> size_t {
      const ze_raytracing_geometry_ext_desc_t* geom = geometries[geomID];
      if (geom == nullptr) return 0;
      return getNumPrimitives(geom);
    };
    
    auto getType = [&](unsigned int geomID)
    {
      const ze_raytracing_geometry_ext_desc_t* geom = geometries[geomID];
      assert(geom);
      switch (geom->geometryType) {
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES : return QBVH6BuilderSAH::TRIANGLE;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS: return QBVH6BuilderSAH::QUAD;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR: return QBVH6BuilderSAH::PROCEDURAL;
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE: return QBVH6BuilderSAH::INSTANCE;
      default: throw std::runtime_error("invalid geometry type");
      };
    };
    
    auto createPrimRefArray = [&] (evector<PrimRef>& prims, BBox1f time_range, const range<size_t>& r, size_t k, unsigned int geomID) -> PrimInfo
    {
      const ze_raytracing_geometry_ext_desc_t* geom = geometries[geomID];
      assert(geom);

      switch (geom->geometryType) {
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_TRIANGLES  : return createGeometryPrimRefArray((ze_raytracing_geometry_triangles_ext_desc_t*)geom,pBuildUserPtr,prims,r,k,geomID);
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_QUADS      : return createGeometryPrimRefArray((ze_raytracing_geometry_quads_ext_desc_t*    )geom,pBuildUserPtr,prims,r,k,geomID);
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_AABBS_FPTR: return createGeometryPrimRefArray((ze_raytracing_geometry_aabbs_fptr_ext_desc_t*)geom,pBuildUserPtr,prims,r,k,geomID);
      case ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE: return createGeometryPrimRefArray((ze_raytracing_geometry_instance_ext_desc_t* )geom,pBuildUserPtr,prims,r,k,geomID);
      default: throw std::runtime_error("invalid geometry type");
      };
    };
    
    auto getTriangle = [&](unsigned int geomID, unsigned int primID)
    {
      const ze_raytracing_geometry_triangles_ext_desc_t* geom = (const ze_raytracing_geometry_triangles_ext_desc_t*) geometries[geomID];
      assert(geom);
      
      const ze_raytracing_triangle_indices_uint32_ext_t tri = getPrimitive(geom,primID);
      if (unlikely(tri.v0 >= geom->vertexCount)) return QBVH6BuilderSAH::Triangle();
      if (unlikely(tri.v1 >= geom->vertexCount)) return QBVH6BuilderSAH::Triangle();
      if (unlikely(tri.v2 >= geom->vertexCount)) return QBVH6BuilderSAH::Triangle();
      
      const Vec3f p0 = getVertex(geom,tri.v0);
      const Vec3f p1 = getVertex(geom,tri.v1);
      const Vec3f p2 = getVertex(geom,tri.v2);
      if (unlikely(!isvalid(p0))) return QBVH6BuilderSAH::Triangle();
      if (unlikely(!isvalid(p1))) return QBVH6BuilderSAH::Triangle();
      if (unlikely(!isvalid(p2))) return QBVH6BuilderSAH::Triangle();

      const GeometryFlags gflags = (GeometryFlags) geom->geometryFlags;
      return QBVH6BuilderSAH::Triangle(tri.v0,tri.v1,tri.v2,p0,p1,p2,gflags,geom->geometryMask);
    };
    
    auto getTriangleIndices = [&] (uint32_t geomID, uint32_t primID) {
      const ze_raytracing_geometry_triangles_ext_desc_t* geom = (const ze_raytracing_geometry_triangles_ext_desc_t*) geometries[geomID];
      assert(geom);
      const ze_raytracing_triangle_indices_uint32_ext_t tri = getPrimitive(geom,primID);
      return Vec3<uint32_t>(tri.v0,tri.v1,tri.v2);
    };
    
    auto getQuad = [&](unsigned int geomID, unsigned int primID)
    {
      const ze_raytracing_geometry_quads_ext_desc_t* geom = (const ze_raytracing_geometry_quads_ext_desc_t*) geometries[geomID];
      assert(geom);
                     
      const ze_raytracing_quad_indices_uint32_ext_t quad = getPrimitive(geom,primID);
      const Vec3f p0 = getVertex(geom,quad.v0);
      const Vec3f p1 = getVertex(geom,quad.v1);
      const Vec3f p2 = getVertex(geom,quad.v2);
      const Vec3f p3 = getVertex(geom,quad.v3);

      const GeometryFlags gflags = (GeometryFlags) geom->geometryFlags;
      return QBVH6BuilderSAH::Quad(p0,p1,p2,p3,gflags,geom->geometryMask);
    };
    
    auto getProcedural = [&](unsigned int geomID, unsigned int primID) {
      const ze_raytracing_geometry_aabbs_fptr_ext_desc_t* geom = (const ze_raytracing_geometry_aabbs_fptr_ext_desc_t*) geometries[geomID];
      assert(geom);
      return QBVH6BuilderSAH::Procedural(geom->geometryMask); // FIXME: pass gflags
    };
    
    auto getInstance = [&](unsigned int geomID, unsigned int primID)
    {
      assert(geometries[geomID]);
      assert(geometries[geomID]->geometryType == ZE_RAYTRACING_GEOMETRY_TYPE_EXT_INSTANCE);
      const ze_raytracing_geometry_instance_ext_desc_t* geom = (const ze_raytracing_geometry_instance_ext_desc_t*) geometries[geomID];
      void* accel = geom->accel;
      const AffineSpace3fa local2world = getTransform(geom);
      return QBVH6BuilderSAH::Instance(local2world,accel,geom->geometryMask,geom->instanceUserID); // FIXME: pass instance flags
    };

    /* dispatch globals ptr for debugging purposes */
    void* dispatchGlobalsPtr = nullptr;
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    dispatchGlobalsPtr = args->dispatchGlobalsPtr;
#endif

    bool verbose = false;
    bool success = QBVH6BuilderSAH::build(numGeometries, nullptr, 
                           getSize, getType, 
                           createPrimRefArray, getTriangle, getTriangleIndices, getQuad, getProcedural, getInstance,
                           (char*)pRtasBuffer, rtasBufferSizeBytes,
                           pScratchBuffer, scratchBufferSizeBytes,
                           (BBox3f*) pBounds, pRtasBufferSizeBytes,
                           args->quality, args->flags, verbose, dispatchGlobalsPtr);
    if (!success) {
      return ZE_RESULT_RAYTRACING_EXT_RETRY_BUILD_ACCEL;
    }
    return ZE_RESULT_SUCCESS_;
  }
  catch (std::exception& e) {
    std::cerr << "caught exception during BVH build: " << e.what() << std::endl;
    return ZE_RESULT_ERROR_UNKNOWN_;
  }
  
  struct ze_raytracing_parallel_operation_ext_handle_t_IMPL
  {
    ze_raytracing_parallel_operation_ext_handle_t_IMPL(ze_rtas_builder_exp_handle_t hBuilder)
      : hBuilder(hBuilder) {}

    ~ze_raytracing_parallel_operation_ext_handle_t_IMPL() {
      magick = 0x0;
      hBuilder = nullptr;
    }

    bool verify() const {
      return (magick == MAGICK) && validate(hBuilder);
    }
    
    enum { MAGICK = 0xE84567E1 };
    uint32_t magick = MAGICK;
    ze_rtas_builder_exp_handle_t hBuilder = nullptr;
    ze_result_t_ errorCode = ZE_RESULT_SUCCESS_;
    tbb::task_group group;
  };

  bool validate(ze_raytracing_parallel_operation_ext_handle_t hParallelOperation)
  {
    if (hParallelOperation == nullptr) return false;
    return ((ze_raytracing_parallel_operation_ext_handle_t_IMPL*)hParallelOperation)->verify();
  }

  RTHWIF_API ze_result_t_ zeRaytracingBuildAccelExt(ze_rtas_builder_exp_handle_t hBuilder, const ze_raytracing_build_accel_ext_desc_t* args,
                                                    void *pScratchBuffer, size_t scratchBufferSizeBytes,
                                                    void *pRtasBuffer, size_t rtasBufferSizeBytes,
                                                    ze_raytracing_parallel_operation_ext_handle_t hParallelOperation,
                                                    void *pBuildUserPtr, ze_raytracing_aabb_ext_t *pBounds, size_t *pRtasBufferSizeBytes)
  {
    /* check for valid rtas builder */
    if (!validate(hBuilder))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;
    
    /* check for valid pointers */
    if (args == nullptr)
      return ZE_RESULT_ERROR_UNKNOWN_;
    
    /* check if input descriptor has proper type */
    if (args->stype != ZE_STRUCTURE_TYPE_RAYTRACING_BUILD_ACCEL_EXT_DESC)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check valid pNext chain */
    if (!checkDescChain((zet_base_desc_t_*)args))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check if acceleration structure format is supported */
    if (args->accelFormat != (ze_raytracing_accel_format_ext_t) ZE_RAYTRACING_ACCEL_FORMAT_EXT_VERSION_1)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check scratch buffer */
    if (pScratchBuffer == nullptr)
      return ZE_RESULT_ERROR_UNKNOWN_;

    /* check rtas buffer */
    if (pRtasBuffer == nullptr)
      return ZE_RESULT_ERROR_UNKNOWN_;
    
    /* if parallel operation is provided then execute using thread arena inside task group ... */
    if (hParallelOperation)
    {
      if (!validate(hParallelOperation))
        return ZE_RESULT_ERROR_INVALID_ARGUMENT_;
      
      ze_raytracing_parallel_operation_ext_handle_t_IMPL* op = (ze_raytracing_parallel_operation_ext_handle_t_IMPL*) hParallelOperation;
      if (op->hBuilder != hBuilder)
        return ZE_RESULT_ERROR_INVALID_ARGUMENT_;
      
      g_arena->execute([&](){ op->group.run([=](){
         op->errorCode = zeRaytracingBuildAccelExtInternal(args,
                                                           pScratchBuffer, scratchBufferSizeBytes,
                                                           pRtasBuffer, rtasBufferSizeBytes,
                                                           pBuildUserPtr, pBounds, pRtasBufferSizeBytes);
                                            });
                       });
      return ZE_RESULT_RAYTRACING_EXT_OPERATION_DEFERRED;
    }
    /* ... otherwise we just execute inside task arena to avoid spawning of TBB worker threads */
    else
    {
      ze_result_t_ errorCode = ZE_RESULT_SUCCESS_;
      g_arena->execute([&](){ errorCode = zeRaytracingBuildAccelExtInternal(args,
                                                                            pScratchBuffer, scratchBufferSizeBytes,
                                                                            pRtasBuffer, rtasBufferSizeBytes,
                                                                            pBuildUserPtr, pBounds, pRtasBufferSizeBytes);
                       });
      return errorCode;
    }
  }

  RTHWIF_API ze_result_t_ zeRaytracingParallelOperationCreateExt(ze_rtas_builder_exp_handle_t hBuilder, ze_raytracing_parallel_operation_ext_handle_t* phParallelOperation)
  {
    /* check for valid rtas builder */
    if (!validate(hBuilder))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;
    
    /* check for valid pointer */
    if (phParallelOperation == nullptr)
      return ZE_RESULT_ERROR_UNKNOWN_;

    /* create parallel operation object */
    *phParallelOperation = (ze_raytracing_parallel_operation_ext_handle_t) new ze_raytracing_parallel_operation_ext_handle_t_IMPL(hBuilder);
    return ZE_RESULT_SUCCESS_;
  }
  
  RTHWIF_API ze_result_t_ zeRaytracingParallelOperationDestroyExt( ze_raytracing_parallel_operation_ext_handle_t parallelOperation )
  {
    /* check for valid handle */
    if (!validate(parallelOperation))
      return ZE_RESULT_ERROR_UNKNOWN_;

    /* delete parallel operation */
    delete (ze_raytracing_parallel_operation_ext_handle_t_IMPL*) parallelOperation;
    return ZE_RESULT_SUCCESS_;
  }
  
  RTHWIF_API ze_result_t_ zeRaytracingParallelOperationGetMaxConcurrencyExt( ze_raytracing_parallel_operation_ext_handle_t parallelOperation, ze_rtas_parallel_operation_exp_properties_t* pProperties )
  {
    /* check for valid handle */
    if (!validate(parallelOperation))
      return ZE_RESULT_ERROR_UNKNOWN_;

    /* check for valid pointer */
    if (pProperties == nullptr)
      return ZE_RESULT_ERROR_UNKNOWN_;

    /* check for proper property */
    if (pProperties->stype != ZE_STRUCTURE_TYPE_RTAS_PARALLEL_OPERATION_EXP_PROPERTIES)
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* check valid pNext chain */
    if (!checkDescChain((zet_base_desc_t_*)pProperties))
      return ZE_RESULT_ERROR_INVALID_ARGUMENT_;

    /* return properties */
    pProperties->flags = ZE_RTAS_PARALLEL_OPERATION_EXP_FLAG_NONE;
    pProperties->maxConcurrency = tbb::this_task_arena::max_concurrency();
    return ZE_RESULT_SUCCESS_;
  }
  
  RTHWIF_API ze_result_t_ zeRaytracingParallelOperationJoinExt( ze_raytracing_parallel_operation_ext_handle_t parallelOperation)
  {
    /* check for valid handle */
    if (!validate(parallelOperation))
      return ZE_RESULT_ERROR_UNKNOWN_;
    
    ze_raytracing_parallel_operation_ext_handle_t_IMPL* op = (ze_raytracing_parallel_operation_ext_handle_t_IMPL*) parallelOperation;
    g_arena->execute([&](){ op->group.wait(); });
    return op->errorCode;
  }
}
