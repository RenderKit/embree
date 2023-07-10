// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#if !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "rthwif_embree.h"
#include "rthwif_embree_builder.h"
#include "../common/scene.h"
#include "../builders/primrefgen.h"
#include "../rthwif/rtbuild/rtbuild.h"

namespace embree
{

  using namespace embree::isa;

  enum Flags : uint32_t {
    NONE,
    DEPTH_TEST_LESS_EQUAL = 1 << 0  // when set we use <= for depth test, otherwise <
  };

  static void align(size_t& ofs, size_t alignment) {
    ofs = (ofs+(alignment-1))&(-alignment);
  }

  struct DispatchGlobals
  {
    uint64_t rtMemBasePtr;               // base address of the allocated stack memory
    uint64_t callStackHandlerKSP;        // this is the KSP of the continuation handler that is invoked by BTD when the read KSP is 0
    uint32_t asyncStackSize;             // async-RT stack size in 64 byte blocks
    uint32_t numDSSRTStacks : 16;        // number of stacks per DSS
    uint32_t syncRayQueryCount : 4;      // number of ray queries in the sync-RT stack: 0-15 mapped to: 1-16
    unsigned _reserved_mbz : 12;
    uint32_t maxBVHLevels;               // the maximal number of supported instancing levels (0->8, 1->1, 2->2, ...)
    Flags flags;                         // per context control flags
  };

  void* zeRTASInitExp(sycl::device device, sycl::context context)
  {
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)

#if !defined(EMBREE_LEVEL_ZERO)
#  error "Level Zero required to properly allocate RTDispatchGlobals"
#endif
    
    size_t maxBVHLevels = RTC_MAX_INSTANCE_LEVEL_COUNT+1;

    size_t rtstack_bytes = (64+maxBVHLevels*(64+32)+63)&-64;
    size_t num_rtstacks = 1<<17; // this is sufficiently large also for PVC
    size_t dispatchGlobalSize = 128+num_rtstacks*rtstack_bytes;
    
    void* dispatchGlobalsPtr = rthwifAllocAccelBuffer(dispatchGlobalSize,device,context);
    memset(dispatchGlobalsPtr, 0, dispatchGlobalSize);

    DispatchGlobals* dg = (DispatchGlobals*) dispatchGlobalsPtr;
    dg->rtMemBasePtr = (uint64_t) dispatchGlobalsPtr + dispatchGlobalSize;
    dg->callStackHandlerKSP = 0;
    dg->asyncStackSize = 0;
    dg->numDSSRTStacks = 0;
    dg->syncRayQueryCount = 0;
    dg->_reserved_mbz = 0;
    dg->maxBVHLevels = maxBVHLevels;
    dg->flags = DEPTH_TEST_LESS_EQUAL;

    return dispatchGlobalsPtr;

#else

    return nullptr;

#endif
  }

  void rthwifCleanup(void* dispatchGlobalsPtr, sycl::context context)
  {
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    rthwifFreeAccelBuffer(dispatchGlobalsPtr, context);
#endif
  }

#if defined(EMBREE_LEVEL_ZERO)

  int rthwifIsSYCLDeviceSupported(const sycl::device& sycl_device)
  {
    /* disabling of device check through env variable */
    const char* disable_device_check = getenv("EMBREE_DISABLE_DEVICEID_CHECK");
    if (disable_device_check && strcmp(disable_device_check,"1") == 0)
      return 1;

    sycl::platform platform = sycl_device.get_platform();
    ze_driver_handle_t hDriver = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(platform);

    uint32_t count = 0;
    std::vector<ze_driver_extension_properties_t> extensions;
    ze_result_t result = zeDriverGetExtensionProperties(hDriver,&count,extensions.data());
    if (result != ZE_RESULT_SUCCESS) return -1;

    extensions.resize(count);
    result = zeDriverGetExtensionProperties(hDriver,&count,extensions.data());
    if (result != ZE_RESULT_SUCCESS) return -1;

    bool ze_extension_ray_tracing = false;
    for (uint32_t i=0; i<extensions.size(); i++)
    {
      //std::cout << extensions[i].name << " version " << extensions[i].version << std::endl;
      
      if (strncmp("ZE_extension_raytracing",extensions[i].name,sizeof(extensions[i].name)))
        continue;
      
      ze_extension_ray_tracing = true; //extensions[i].version >= ZE_RAYTRACING_EXT_VERSION_1_0;
      break;
    }
    if (!ze_extension_ray_tracing)
      return -1;
  
#if 0

    /* check if GPU device is supported */
    ze_device_handle_t hDevice = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(sycl_device);

    /* check if ray tracing hardware is supported */
    ze_device_raytracing_ext_properties_t raytracing_properties;
    memset(&raytracing_properties,0,sizeof(raytracing_properties));
    raytracing_properties.stype = ZE_STRUCTURE_TYPE_DEVICE_RAYTRACING_EXT_PROPERTIES;
    raytracing_properties.pNext = nullptr;
    
    ze_device_module_properties_t module_properties;
    memset(&module_properties,0,sizeof(module_properties));
    module_properties.stype = ZE_STRUCTURE_TYPE_DEVICE_MODULE_PROPERTIES;
    module_properties.pNext = &raytracing_properties;
      
    ze_result_t result = zeDeviceGetModuleProperties(hDevice, &module_properties);
    if (result != ZE_RESULT_SUCCESS) return -1;

    const bool rayQuerySupported = raytracing_properties.flags & ZE_DEVICE_RAYTRACING_EXT_FLAG_RAYQUERY;
    if (!rayQuerySupported)
      return -1;
#endif

    return sycl_device.get_info<sycl::info::device::max_compute_units>();
  }

#else

  int rthwifIsSYCLDeviceSupported(const sycl::device& device)
  {
    // TODO: SYCL currently has no functionality to check if a GPU has RTHW
    // capabilities. Therefore, we return true when the device is a GPU,
    // the backend is level_zero, and the GPU has 8 threads per EU because
    // that indicates raytracing hardware.
    uint32_t threadsPerEU = 0;
    if (device.has(sycl::aspect::ext_intel_gpu_hw_threads_per_eu)) {
      threadsPerEU = device.get_info<sycl::ext::intel::info::device::gpu_hw_threads_per_eu>();
    }
    sycl::platform platform = device.get_platform();
    if(!device.is_gpu() || (threadsPerEU < 8) || (platform.get_info<sycl::info::platform::name>() != "Intel(R) Level-Zero"))
      return -1;
    else
      return device.get_info<sycl::info::device::max_compute_units>();
  }

#endif

#if defined(EMBREE_LEVEL_ZERO)

  void* rthwifAllocAccelBuffer(size_t bytes, sycl::device device, sycl::context context)
  {
    ze_context_handle_t hContext = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(context);
    ze_device_handle_t  hDevice  = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(device);

    ze_rtas_device_exp_properties_t rtasProp = { ZE_STRUCTURE_TYPE_RTAS_DEVICE_EXP_PROPERTIES };
    ze_result_t err = zeDeviceGetRTASPropertiesExp(hDevice, &rtasProp );
    if (err != ZE_RESULT_SUCCESS)
      throw std::runtime_error("get rtas device properties failed");

    ze_raytracing_mem_alloc_ext_desc_t rt_desc;
    rt_desc.stype = ZE_STRUCTURE_TYPE_RAYTRACING_MEM_ALLOC_EXT_DESC;
    rt_desc.pNext = nullptr;
    rt_desc.flags = 0;

    ze_relaxed_allocation_limits_exp_desc_t relaxed;
    relaxed.stype = ZE_STRUCTURE_TYPE_RELAXED_ALLOCATION_LIMITS_EXP_DESC;
    relaxed.pNext = &rt_desc;
    relaxed.flags = ZE_RELAXED_ALLOCATION_LIMITS_EXP_FLAG_MAX_SIZE;
    
    ze_device_mem_alloc_desc_t device_desc;
    device_desc.stype = ZE_STRUCTURE_TYPE_DEVICE_MEM_ALLOC_DESC;
    device_desc.pNext = &relaxed;
    device_desc.flags = ZE_DEVICE_MEM_ALLOC_FLAG_BIAS_CACHED;
    device_desc.ordinal = 0;
  
    ze_host_mem_alloc_desc_t host_desc;
    host_desc.stype = ZE_STRUCTURE_TYPE_HOST_MEM_ALLOC_DESC;
    host_desc.pNext = nullptr;
    host_desc.flags = ZE_HOST_MEM_ALLOC_FLAG_BIAS_CACHED;
    
    void* ptr = nullptr;
    ze_result_t result = zeMemAllocShared(hContext,&device_desc,&host_desc,bytes,rtasProp.rtasBufferAlignment,hDevice,&ptr);
    if (result != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_OUT_OF_MEMORY,"rtas memory allocation failed");

    return ptr;
  }
  
  void rthwifFreeAccelBuffer(void* ptr, sycl::context context)
  {
    if (ptr == nullptr) return;
    ze_context_handle_t hContext = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(context);
    ze_result_t result = zeMemFree(hContext,ptr);
    if (result != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_OUT_OF_MEMORY,"rtas memory free failed");
  }

#else

  void* rthwifAllocAccelBuffer(size_t bytes, sycl::device device, sycl::context context)
  {
    void* ptr = sycl::aligned_alloc_shared(128, bytes, device, context);
    
    if (ptr == nullptr)
      throw_RTCError(RTC_ERROR_OUT_OF_MEMORY,"rtas memory allocation failed");

    auto isAddrCanonical = [](uint64_t addr) {
      return ((addr & 0xFFFF000000000000LL) == 0x0) || ((addr & 0xFFFF800000000000LL) == 0xFFFF800000000000LL);
    };
    if (!isAddrCanonical((uint64_t)ptr))
      throw_RTCError(RTC_ERROR_OUT_OF_MEMORY,"rtas memory allocation out of 48 bit address range");
    return ptr;
  }

  void rthwifFreeAccelBuffer(void* ptr, sycl::context context)
  {
    if (ptr == nullptr) return;
    sycl::free(ptr, context);
  }

#endif

  struct GEOMETRY_INSTANCE_DESC : ze_rtas_builder_instance_geometry_info_exp_t
  {
    ze_rtas_transform_float3x4_aligned_column_major_exp_t xfmdata;
  };

  struct GEOMETRY_TYPE
  {
    GEOMETRY_TYPE(ze_rtas_builder_geometry_type_exp_t type, size_t extraBytes = 0)
      : type(type), extraBytes(extraBytes) {}
    
    ze_rtas_builder_geometry_type_exp_t type;
    size_t extraBytes;
  };

  size_t sizeof_RTHWIF_GEOMETRY(GEOMETRY_TYPE type)
  {
    switch (type.type) {
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_TRIANGLES  : return sizeof(ze_rtas_builder_triangles_geometry_info_exp_t)+type.extraBytes;
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_QUADS      : return sizeof(ze_rtas_builder_quads_geometry_info_exp_t)+type.extraBytes;
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL: return sizeof(ze_rtas_builder_procedural_geometry_info_exp_t)+type.extraBytes;
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE  : return sizeof(ze_rtas_builder_instance_geometry_info_exp_t)+type.extraBytes;
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_LOSSY_COMPRESSED_GEOMETRY  : return sizeof(ze_rtas_builder_lossy_compressed_geometry_info_exp_t)+type.extraBytes;
    default: assert(false); return 0;
    }
  }

  size_t alignof_RTHWIF_GEOMETRY(GEOMETRY_TYPE type)
  {
    switch (type.type) {
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_TRIANGLES  : return alignof(ze_rtas_builder_triangles_geometry_info_exp_t);
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_QUADS      : return alignof(ze_rtas_builder_quads_geometry_info_exp_t);
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL: return alignof(ze_rtas_builder_procedural_geometry_info_exp_t);
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE  : return alignof(ze_rtas_builder_instance_geometry_info_exp_t);
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_LOSSY_COMPRESSED_GEOMETRY  : return alignof(ze_rtas_builder_lossy_compressed_geometry_info_exp_t);
      
    default: assert(false); return 0;
    }
  }

  ze_rtas_builder_geometry_exp_flags_t getGeometryFlags(Scene* scene, Geometry* geom)
  {
    /* invoke any hit callback when Embree filter functions are present */
    ze_rtas_builder_geometry_exp_flags_t gflags = 0;
    if (geom->hasArgumentFilterFunctions() || geom->hasGeometryFilterFunctions())
      gflags = ZE_RTAS_BUILDER_GEOMETRY_EXP_FLAG_NON_OPAQUE;
    
#if defined(EMBREE_RAY_MASK)
    /* invoke any hit callback when high mask bits are enabled */
    if (geom->mask & 0xFFFFFF80)
      gflags = ZE_RTAS_BUILDER_GEOMETRY_EXP_FLAG_NON_OPAQUE;
#endif
    
    return gflags;
  }

  void createGeometryDesc(ze_rtas_builder_lossy_compressed_geometry_info_exp_t* out, Scene* scene, LossyCompressedGeometry* geom)
  {
    memset(out,0,sizeof(ze_rtas_builder_lossy_compressed_geometry_info_exp_t));
    out->geometryType = ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_LOSSY_COMPRESSED_GEOMETRY;
    out->geometryFlags = getGeometryFlags(scene,geom);
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->numLCGs = geom->numLCGs;//geom->numPrimitives;
    out->pLCGs = geom->pLCGs; //(void*)geom->userPtr;
    out->numLCMs = geom->numLCMs;    
    out->pLCMs = geom->pLCMs;
    out->pLCMIDs = geom->pLCMIDs;        
  }
  
  void createGeometryDesc(ze_rtas_builder_triangles_geometry_info_exp_t* out, Scene* scene, TriangleMesh* geom)
  {
    memset(out,0,sizeof(ze_rtas_builder_triangles_geometry_info_exp_t));
    out->geometryType = ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_TRIANGLES;
    out->geometryFlags = getGeometryFlags(scene,geom);
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->triangleFormat = ZE_RTAS_BUILDER_INPUT_DATA_FORMAT_EXP_TRIANGLE_INDICES_UINT32;
    out->vertexFormat = ZE_RTAS_BUILDER_INPUT_DATA_FORMAT_EXP_FLOAT3;
    out->pTriangleBuffer = (ze_rtas_triangle_indices_uint32_exp_t*) geom->triangles.getPtr();
    out->triangleCount = geom->triangles.size();
    out->triangleStride = geom->triangles.getStride();
    out->pVertexBuffer = (ze_rtas_float3_exp_t*) geom->vertices0.getPtr();
    out->vertexCount = geom->vertices0.size();
    out->vertexStride = geom->vertices0.getStride();
  }

  void createGeometryDesc(ze_rtas_builder_quads_geometry_info_exp_t* out, Scene* scene, QuadMesh* geom)
  {
    memset(out,0,sizeof(ze_rtas_builder_quads_geometry_info_exp_t));
    out->geometryType = ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_QUADS;
    out->geometryFlags = getGeometryFlags(scene,geom);
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->quadFormat = ZE_RTAS_BUILDER_INPUT_DATA_FORMAT_EXP_QUAD_INDICES_UINT32;
    out->vertexFormat = ZE_RTAS_BUILDER_INPUT_DATA_FORMAT_EXP_FLOAT3;
    out->pQuadBuffer = (ze_rtas_quad_indices_uint32_exp_t*) geom->quads.getPtr();
    out->quadCount = geom->quads.size();
    out->quadStride = geom->quads.getStride();
    out->pVertexBuffer = (ze_rtas_float3_exp_t*) geom->vertices0.getPtr();
    out->vertexCount = geom->vertices0.size();
    out->vertexStride = geom->vertices0.getStride();
  }

  void getProceduralAABB(ze_rtas_geometry_aabbs_exp_cb_params_t* params)
  {
    assert(params->stype == ZE_STRUCTURE_TYPE_RTAS_GEOMETRY_AABBS_EXP_CB_PARAMS);

    BBox1f time_range = * (BBox1f*) params->pBuildUserPtr;
    Geometry* geom = (Geometry*) params->pGeomUserPtr;
    ze_rtas_aabb_exp_t* boundsOut = params->pBoundsOut;
      
    for (uint32_t i=0; i<params->primIDCount; i++)
    {
      const uint32_t primID = params->primID+i;
      PrimRef prim;
      range<size_t> r(primID);
      size_t k = 0;
      uint32_t geomID = 0;

      PrimInfo pinfo = empty;
      if (geom->numTimeSegments() > 0)
        pinfo = geom->createPrimRefArrayMB(&prim,time_range,r,k,geomID);
      else
        pinfo = geom->createPrimRefArray(&prim,r,k,geomID);

      /* invalid primitive */
      if (pinfo.size() == 0) {
        boundsOut[i].lower.x = pos_inf;
        boundsOut[i].lower.y = pos_inf;
        boundsOut[i].lower.z = pos_inf;
        boundsOut[i].upper.x = neg_inf;
        boundsOut[i].upper.y = neg_inf;
        boundsOut[i].upper.z = neg_inf;
      }
      else
      {
        BBox3fa bounds = prim.bounds();
        boundsOut[i].lower.x = bounds.lower.x;
        boundsOut[i].lower.y = bounds.lower.y;
        boundsOut[i].lower.z = bounds.lower.z;
        boundsOut[i].upper.x = bounds.upper.x;
        boundsOut[i].upper.y = bounds.upper.y;
        boundsOut[i].upper.z = bounds.upper.z;
      }
    }
  };

  void createGeometryDescProcedural(ze_rtas_builder_procedural_geometry_info_exp_t* out, Scene* scene, Geometry* geom)
  {
    uint32_t numPrimitives = geom->size();
    if (GridMesh* mesh = dynamic_cast<GridMesh*>(geom))
      numPrimitives = mesh->getNumTotalQuads(); // FIXME: slow
    
    memset(out,0,sizeof(ze_rtas_builder_procedural_geometry_info_exp_t));
    out->geometryType = ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL;
    out->geometryFlags = 0;
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->primCount = numPrimitives;
    out->pfnGetBoundsCb = getProceduralAABB;
    out->pGeomUserPtr = geom;
  }
  
  void createGeometryDesc(GEOMETRY_INSTANCE_DESC* out, Scene* scene, Instance* geom)
  {
    assert(geom->gsubtype == AccelSet::GTY_SUBTYPE_INSTANCE_QUATERNION);
    memset(out,0,sizeof(GEOMETRY_INSTANCE_DESC));
    out->geometryType = ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE;
    out->instanceFlags = 0;
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->instanceUserID = 0;
    const AffineSpace3fa local2world = geom->getLocal2World();
    out->transformFormat = ZE_RTAS_BUILDER_INPUT_DATA_FORMAT_EXP_FLOAT3X4_ALIGNED_COLUMN_MAJOR;
    out->pTransform = (float*) &out->xfmdata;
    out->pBounds = (ze_rtas_aabb_exp_t*) &dynamic_cast<Scene*>(geom->object)->hwaccel_bounds;
    out->xfmdata = *(ze_rtas_transform_float3x4_aligned_column_major_exp_t*) &local2world;
    EmbreeHWAccel* hwaccel = (EmbreeHWAccel*) dynamic_cast<Scene*>(geom->object)->hwaccel.data();
    out->pAccelerationStructure = hwaccel->AccelTable[0];
  }

  void createGeometryDesc(ze_rtas_builder_instance_geometry_info_exp_t* out, Scene* scene, Instance* geom)
  {
    assert(geom->gsubtype == AccelSet::GTY_SUBTYPE_DEFAULT);
    memset(out,0,sizeof(ze_rtas_builder_instance_geometry_info_exp_t));
    out->geometryType = ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE;
    out->instanceFlags = 0;
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->instanceUserID = 0;
    out->transformFormat = ZE_RTAS_BUILDER_INPUT_DATA_FORMAT_EXP_FLOAT3X4_ALIGNED_COLUMN_MAJOR;
    out->pTransform = (float*) &geom->local2world[0];
    out->pBounds = (ze_rtas_aabb_exp_t*) &dynamic_cast<Scene*>(geom->object)->hwaccel_bounds;
    EmbreeHWAccel* hwaccel = (EmbreeHWAccel*) dynamic_cast<Scene*>(geom->object)->hwaccel.data();
    out->pAccelerationStructure = hwaccel->AccelTable[0];
  }

  void createGeometryDesc(char* out, Scene* scene, Geometry* geom, GEOMETRY_TYPE type)
  {
    switch (type.type) {
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_TRIANGLES  : return createGeometryDesc((ze_rtas_builder_triangles_geometry_info_exp_t*)out,scene,dynamic_cast<TriangleMesh*>(geom));
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_QUADS      : return createGeometryDesc((ze_rtas_builder_quads_geometry_info_exp_t*)out,scene,dynamic_cast<QuadMesh*>(geom));
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL: return createGeometryDescProcedural((ze_rtas_builder_procedural_geometry_info_exp_t*)out,scene,geom);
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE:
      if (type.extraBytes) return createGeometryDesc((GEOMETRY_INSTANCE_DESC*)out,scene,dynamic_cast<Instance*>(geom));
      else                 return createGeometryDesc((ze_rtas_builder_instance_geometry_info_exp_t*)out,scene,dynamic_cast<Instance*>(geom));
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_LOSSY_COMPRESSED_GEOMETRY: return createGeometryDesc((ze_rtas_builder_lossy_compressed_geometry_info_exp_t*)out,scene,dynamic_cast<LossyCompressedGeometry*>(geom));
      
    default: assert(false);
    }
  }

  ze_rtas_builder_build_quality_hint_exp_t convertBuildQuality(RTCBuildQuality quality_flags)
  {
    switch (quality_flags) {
    case RTC_BUILD_QUALITY_LOW    : return ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_LOW;
    case RTC_BUILD_QUALITY_MEDIUM : return ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_MEDIUM;
    case RTC_BUILD_QUALITY_HIGH   : return ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_HIGH;
    case RTC_BUILD_QUALITY_REFIT  : return ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_LOW;
    default                       : return ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_MEDIUM;
    }
  }

  ze_rtas_builder_build_op_exp_flags_t convertBuildFlags(RTCSceneFlags scene_flags, RTCBuildQuality quality_flags)
  {
    ze_rtas_builder_build_op_exp_flags_t result = 0;
    if (scene_flags & RTC_SCENE_FLAG_COMPACT) result |= ZE_RTAS_BUILDER_BUILD_OP_EXP_FLAG_COMPACT;

    /* only in high quality build mode spatial splits are allowed in Embree */
    if (quality_flags != RTC_BUILD_QUALITY_HIGH)
      result |= ZE_RTAS_BUILDER_BUILD_OP_EXP_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION;

    return result;
  }  

  
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
  
  void exception_handler(sycl::exception_list exceptions)
  {
    for (std::exception_ptr const& e : exceptions) {
      try {
        std::rethrow_exception(e);
      } catch(sycl::exception const& e) {
        std::cout << "Caught asynchronous SYCL exception: " << e.what() << std::endl;
      }
    }
  };

#endif
  
  
  BBox3f rthwifBuild(Scene* scene, AccelBuffer& accel)
  {
    DeviceGPU* gpuDevice = dynamic_cast<DeviceGPU*>(scene->device);
    if (gpuDevice == nullptr) throw std::runtime_error("internal error");
    ze_result_t err;
    if (scene->size() > 0x00FFFFFF)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "too many geometries inside scene");
    
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    DeviceGPU *gpu_device = gpuDevice;
    sycl::device &sycl_device = gpu_device->getGPUDevice();
    sycl::property_list PropList;
    if (gpu_device->verbose) PropList = { sycl::property::queue::enable_profiling() };
    sycl::queue sycl_queue(sycl_device, exception_handler, PropList );
#else    
    sycl::device device = gpuDevice->getGPUDevice();
    ze_device_handle_t hDevice = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(device);
    sycl::platform platform = device.get_platform();
    ze_driver_handle_t hDriver = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(platform);
    
    /* create L0 builder object */
    ze_rtas_builder_exp_desc_t builderDesc = { ZE_STRUCTURE_TYPE_RTAS_BUILDER_EXP_DESC };
    ze_rtas_builder_exp_handle_t hBuilder = nullptr;
    err = zeRTASBuilderCreateExp(hDriver, &builderDesc, &hBuilder);
    if (err != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_UNKNOWN, "ze_rtas_builder creation failed");      
#endif    
    
    auto getType = [&](unsigned int geomID) -> GEOMETRY_TYPE
    {
      /* no HW support for MB yet */
      if (scene->get(geomID)->numTimeSegments() > 0)
        return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL;

      switch (scene->get(geomID)->getType()) {
      case Geometry::GTY_FLAT_LINEAR_CURVE    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ROUND_LINEAR_CURVE   : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_LINEAR_CURVE: return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_CONE_LINEAR_CURVE    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      
      case Geometry::GTY_FLAT_BEZIER_CURVE    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ROUND_BEZIER_CURVE   : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_BEZIER_CURVE: return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      
      case Geometry::GTY_FLAT_BSPLINE_CURVE    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ROUND_BSPLINE_CURVE   : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_BSPLINE_CURVE: return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      
      case Geometry::GTY_FLAT_HERMITE_CURVE    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ROUND_HERMITE_CURVE   : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_HERMITE_CURVE: return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      
      case Geometry::GTY_FLAT_CATMULL_ROM_CURVE    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ROUND_CATMULL_ROM_CURVE   : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE: return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      
      case Geometry::GTY_TRIANGLE_MESH: return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_TRIANGLES; break;
      case Geometry::GTY_QUAD_MESH    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_QUADS; break;
      case Geometry::GTY_GRID_MESH    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_SUBDIV_MESH  : assert(false); return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      
      case Geometry::GTY_SPHERE_POINT       : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_DISC_POINT         : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_DISC_POINT: return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      
      case Geometry::GTY_USER_GEOMETRY     : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_LOSSY_COMPRESSED_GEOMETRY     : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_LOSSY_COMPRESSED_GEOMETRY; break;

#if RTC_MAX_INSTANCE_LEVEL_COUNT < 2
      case Geometry::GTY_INSTANCE_CHEAP    :
      case Geometry::GTY_INSTANCE_EXPENSIVE: {
        Instance* instance = scene->get<Instance>(geomID);
        EmbreeHWAccel* object = (EmbreeHWAccel*)((Scene*)instance->object)->hwaccel.data();
        if (object->numTimeSegments > 1) return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; // we need to handle instances in procedural mode if instanced scene has motion blur
        if (instance->mask & 0xFFFFFF80) return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; // we need to handle instances in procedural mode if high mask bits are set
        else if (instance->gsubtype == AccelSet::GTY_SUBTYPE_INSTANCE_QUATERNION)
          return GEOMETRY_TYPE(ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE,sizeof(GEOMETRY_INSTANCE_DESC)-sizeof(ze_rtas_builder_instance_geometry_info_exp_t));
        else return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE;
      }
#else
      case Geometry::GTY_INSTANCE_CHEAP    : return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
      case Geometry::GTY_INSTANCE_EXPENSIVE: return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL; break;
#endif

      default: assert(false); return ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL;
      }
    };

    /* calculate maximal number of motion blur time segments in scene */
    uint32_t maxTimeSegments = 1;
    for (size_t geomID=0; geomID<scene->size(); geomID++)
    {
      Geometry* geom = scene->get(geomID);
      if (geom == nullptr) continue;
      maxTimeSegments = std::max(maxTimeSegments, geom->numTimeSegments());
    }

    /* calculate size of geometry descriptor buffer */
    size_t totalBytes = 0;
    for (size_t geomID=0; geomID<scene->size(); geomID++)
    {
      Geometry* geom = scene->get(geomID);
      if (geom == nullptr) continue;
      
      const GEOMETRY_TYPE type = getType(geomID);
      align(totalBytes,alignof_RTHWIF_GEOMETRY(type));
      totalBytes += sizeof_RTHWIF_GEOMETRY(type);
    }

    const size_t numGeometries = scene->size();

    //auto alloc_mode = sycl::ext::oneapi::property::usm::device_read_only();    
    /* fill geomdesc buffers */    
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    const size_t geomDescrBytes = sizeof(ze_rtas_builder_geometry_info_exp_t*)*numGeometries;
    ze_rtas_builder_geometry_info_exp_t** geomDescr = (ze_rtas_builder_geometry_info_exp_t**)sycl::aligned_alloc(64,geomDescrBytes,gpu_device->getGPUDevice(),gpu_device->getGPUContext(),sycl::usm::alloc::host);
    assert(geomDescr);        
    char *geomDescrData = (char*)sycl::aligned_alloc_shared(64,totalBytes,gpu_device->getGPUDevice(),gpu_device->getGPUContext());
    assert(geomDescrData);
#else    
    /* fill geomdesc buffers */
    std::vector<ze_rtas_builder_geometry_info_exp_t*> geomDescr(scene->size());
    std::vector<char> geomDescrData(totalBytes);
#endif
    
    size_t offset = 0;
    for (size_t geomID=0; geomID<numGeometries; geomID++) // FIXME: slow if numGeometries is large
    {
      geomDescr[geomID] = nullptr;     
      Geometry* geom = scene->get(geomID);
      if (geom == nullptr) continue;
      
      const GEOMETRY_TYPE type = getType(geomID);
      align(offset,alignof_RTHWIF_GEOMETRY(type));
      createGeometryDesc(&geomDescrData[offset],scene,scene->get(geomID),type);
      geomDescr[geomID] = (ze_rtas_builder_geometry_info_exp_t*) &geomDescrData[offset];
      offset += sizeof_RTHWIF_GEOMETRY(type);
#if !defined(EMBREE_SYCL_GPU_BVH_BUILDER)      
      assert(offset <= geomDescrData.size());
#endif      
    }
      
#if !defined(EMBREE_SYCL_GPU_BVH_BUILDER)          
    ze_rtas_parallel_operation_exp_handle_t parallelOperation = nullptr;
    err = zeRTASParallelOperationCreateExp(hDriver, &parallelOperation);
    if (err != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_UNKNOWN, "parallel operation creation failed");

    ze_rtas_device_exp_properties_t rtasProp = { ZE_STRUCTURE_TYPE_RTAS_DEVICE_EXP_PROPERTIES };
    err = zeDeviceGetRTASPropertiesExp(hDevice, &rtasProp );
    if (err != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_UNKNOWN, "get rtas device properties failed");
#endif    

    /* estimate static accel size */
    BBox1f time_range(0,1);
    ze_rtas_aabb_exp_t bounds;
    ze_rtas_builder_build_op_exp_desc_t args;
    memset(&args,0,sizeof(args));

    args.stype = ZE_STRUCTURE_TYPE_RTAS_BUILDER_BUILD_OP_EXP_DESC;
    args.pNext = nullptr;
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    args.rtasFormat = ZE_RTAS_FORMAT_EXP_INVALID;
    args.buildQuality = gpu_device->quality_flags == RTC_BUILD_QUALITY_LOW ? ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_LOW : ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_MEDIUM;
#else
    args.rtasFormat = rtasProp.rtasFormat;
    args.buildQuality = convertBuildQuality(scene->quality_flags);
#endif    
    args.buildFlags = convertBuildFlags(scene->scene_flags,scene->quality_flags);
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    args.ppGeometries = (const ze_rtas_builder_geometry_info_exp_t**) geomDescr;
    args.numGeometries = numGeometries;    
#else    
    args.ppGeometries = (const ze_rtas_builder_geometry_info_exp_t**) geomDescr.data();
    args.numGeometries = geomDescr.size();
#endif    

#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    args.dispatchGlobalsPtr = dynamic_cast<DeviceGPU*>(scene->device)->dispatchGlobalsPtr;
#endif
        
    ze_rtas_builder_exp_properties_t sizeTotal = { ZE_STRUCTURE_TYPE_RTAS_BUILDER_EXP_PROPERTIES };
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    err = zeRTASGetAccelSizeGPUExp(&args,&sizeTotal,&sycl_queue,gpu_device->verbose);    
#else            
    err = zeRTASBuilderGetBuildPropertiesExp(hBuilder,&args,&sizeTotal);
    if (err != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_UNKNOWN,"BVH size estimate failed");
#endif
    
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)    
    char *scratchBuffer  = (char*)sycl::aligned_alloc(64,sizeTotal.scratchBufferSizeBytes,gpu_device->getGPUDevice(),gpu_device->getGPUContext(),gpu_device->verbose > 1 ? sycl::usm::alloc::shared : sycl::usm::alloc::device);    
#else    
    /* allocate scratch buffer */
    std::vector<char> scratchBuffer(sizeTotal.scratchBufferSizeBytes);
#endif
    
    size_t headerBytes = sizeof(EmbreeHWAccel) + std::max(1u,maxTimeSegments)*8;
    align(headerBytes,128);

    /* build BVH */
    BBox3f fullBounds = empty;
    while (true)
    {
      /* estimate size of all mblur BVHs */
      ze_rtas_builder_exp_properties_t size = { ZE_STRUCTURE_TYPE_RTAS_BUILDER_EXP_PROPERTIES };
      size.rtasBufferSizeBytesExpected  = maxTimeSegments*sizeTotal.rtasBufferSizeBytesExpected;
      size.rtasBufferSizeBytesMaxRequired = maxTimeSegments*sizeTotal.rtasBufferSizeBytesMaxRequired;
      size_t bytes = headerBytes+size.rtasBufferSizeBytesExpected;

      /* allocate BVH data */
      if (accel.size() < bytes)
	{
	  PRINT("RESIZING ACCEL BUFFER");
	  accel.resize(bytes);
	}
      
#if !defined(EMBREE_SYCL_GPU_BVH_BUILDER)      
      memset(accel.data(),0,accel.size()); // FIXME: not required
#endif
      
      /* build BVH for each time segment */
      for (uint32_t i=0; i<maxTimeSegments; i++)
      {
        const float t0 = float(i+0)/float(maxTimeSegments);
        const float t1 = float(i+1)/float(maxTimeSegments);
        time_range = BBox1f(t0,t1);
        
        void* accelBuffer = accel.data() + headerBytes + i*sizeTotal.rtasBufferSizeBytesExpected;
        size_t accelBufferBytes = sizeTotal.rtasBufferSizeBytesExpected;
        bounds = { { INFINITY, INFINITY, INFINITY }, { -INFINITY, -INFINITY, -INFINITY } };

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
        err = zeRTASBuildAccelGPUExp(&args,
                                     scratchBuffer,sizeTotal.scratchBufferSizeBytes,
                                     accelBuffer, accelBufferBytes,
                                     &time_range, &bounds, nullptr,&sycl_queue,gpu_device->verbose); //FIXME nullptr ????
#else        
        err = zeRTASBuilderBuildExp(hBuilder,&args,
                                    scratchBuffer.data(),scratchBuffer.size(),
                                    accelBuffer, accelBufferBytes,
                                    parallelOperation,
                                    &time_range, &bounds, nullptr);
        
        if (parallelOperation)
        {
          assert(err == ZE_RESULT_ERROR_HANDLE_OBJECT_IN_USE);

          ze_rtas_parallel_operation_exp_properties_t prop = { ZE_STRUCTURE_TYPE_RTAS_PARALLEL_OPERATION_EXP_PROPERTIES };
          err = zeRTASParallelOperationGetPropertiesExp(parallelOperation,&prop);
          if (err != ZE_RESULT_SUCCESS)
            throw_RTCError(RTC_ERROR_UNKNOWN, "get max concurrency failed");
          
          parallel_for(prop.maxConcurrency, [&](uint32_t) { err = zeRTASParallelOperationJoinExp(parallelOperation); });
        }
#endif
        if (err == ZE_RESULT_SUCCESS) // added if        
          fullBounds.extend(*(BBox3f*) &bounds);

        if (err == ZE_RESULT_EXP_ERROR_RETRY_RTAS_BUILD)
        {
          if (sizeTotal.rtasBufferSizeBytesExpected == sizeTotal.rtasBufferSizeBytesMaxRequired)
            throw_RTCError(RTC_ERROR_UNKNOWN,"build error");
          
          sizeTotal.rtasBufferSizeBytesExpected = std::min(sizeTotal.rtasBufferSizeBytesMaxRequired,(size_t(1.2*sizeTotal.rtasBufferSizeBytesExpected)+127)&-128);
          break;
        }
        
        if (err != ZE_RESULT_SUCCESS) break;
      }
      if (err != ZE_RESULT_EXP_ERROR_RETRY_RTAS_BUILD) break;
    }

    if (err != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_UNKNOWN,"build error");

    // === moving this to device code prevents USM down and up transfer of accel ===

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    char header[headerBytes];    
    EmbreeHWAccel *hwaccel = (EmbreeHWAccel *)header;
    hwaccel->numTimeSegments = maxTimeSegments;
    for (size_t i=0; i<maxTimeSegments; i++)
      hwaccel->AccelTable[i] = (char*)accel.data() + headerBytes + i*sizeTotal.rtasBufferSizeBytesExpected;    
    sycl::event queue_event =  sycl_queue.memcpy(accel.data(),hwaccel,headerBytes);
    try {
      queue_event.wait_and_throw();
     } catch (sycl::exception const& e) {
        std::cout << "Caught synchronous SYCL exception:\n"
                  << e.what() << std::endl;
        FATAL("SYCL Exception");     
    }     
#else    
    /* destroy parallel operation */
    err = zeRTASParallelOperationDestroyExp(parallelOperation);
    if (err != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_UNKNOWN, "parallel operation destruction failed");

    /* destroy rtas builder again */
    err = zeRTASBuilderDestroyExp(hBuilder);
    if (err != ZE_RESULT_SUCCESS)
      throw_RTCError(RTC_ERROR_UNKNOWN, "ze_rtas_builder destruction failed");
    
    EmbreeHWAccel* hwaccel = (EmbreeHWAccel*) accel.data();
    hwaccel->numTimeSegments = maxTimeSegments;

    for (size_t i=0; i<maxTimeSegments; i++)
      hwaccel->AccelTable[i] = (char*)hwaccel + headerBytes + i*sizeTotal.rtasBufferSizeBytesExpected;
#endif
    
    // =============================================================================
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    sycl::free(geomDescr        ,gpu_device->getGPUContext());
    sycl::free(geomDescrData    ,gpu_device->getGPUContext());    
    sycl::free(scratchBuffer    ,gpu_device->getGPUContext());
#endif
    return fullBounds;
  }

}
