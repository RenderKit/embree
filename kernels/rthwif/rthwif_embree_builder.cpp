// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "rthwif_embree.h"
#include "rthwif_embree_builder.h"
#include "../common/scene.h"
#include "builder/qbvh6_builder_sah.h"
#include "../builders/primrefgen.h"
#include "rthwif_internal.h"
#include "rthwif_builder.h"

#include <level_zero/ze_api.h>

namespace embree
{

  BBox3fa rthwifBuildPloc(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& buffer_o, const bool two_level=false);

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

  void* rthwifInit(sycl::device device, sycl::context context)
  {
    ::rthwifInit();
    
    size_t maxBVHLevels = RTC_MAX_INSTANCE_LEVEL_COUNT+1;

    size_t rtstack_bytes = (64+maxBVHLevels*(64+32)+63)&-64;
    size_t num_rtstacks = 1<<17; // this is sufficiently large also for PVC
    size_t dispatchGlobalSize = 128+num_rtstacks*rtstack_bytes;
    
    //dispatchGlobalsPtr = this->malloc(dispatchGlobalSize, 64);
    void* dispatchGlobalsPtr = sycl::aligned_alloc(64,dispatchGlobalSize,device,context,sycl::usm::alloc::shared);
    memset(dispatchGlobalsPtr, 0, dispatchGlobalSize);

    if (((size_t)dispatchGlobalsPtr & 0xFFFF000000000000ull) != 0)
      throw_RTCError(RTC_ERROR_UNKNOWN,"internal error in RTStack allocation");
  
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
  }

  void rthwifCleanup(void* dispatchGlobalsPtr, sycl::context context)
  {
    sycl::free(dispatchGlobalsPtr, context);

    rthwifExit();
  }

  bool rthwifIsSYCLDeviceSupported(const sycl::device& sycl_device)
  {
    /* disabling of device check through env variable */
    const char* disable_device_check = getenv("EMBREE_DISABLE_DEVICEID_CHECK");
    if (disable_device_check && strcmp(disable_device_check,"1") == 0)
      return true;

    /* check if GPU device is supported */
    ze_device_handle_t hDevice = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(sycl_device);
    
    const RTHWIF_FEATURES features = rthwifGetSupportedFeatures(hDevice);
    return features != RTHWIF_FEATURES_NONE;
  }
    
  void* rthwifAllocAccelBuffer(size_t bytes, sycl::device device, sycl::context context)
  {
    ze_context_handle_t hContext = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(context);
    ze_device_handle_t  hDevice  = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(device);
    
    ze_raytracing_mem_alloc_ext_desc_t rt_desc;
    rt_desc.stype = ZE_STRUCTURE_TYPE_DEVICE_RAYTRACING_EXT_PROPERTIES;
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
    
    void* ptr = nullptr;
    ze_result_t result = zeMemAllocShared(hContext,&device_desc,&host_desc,bytes,RTHWIF_ACCELERATION_STRUCTURE_ALIGNMENT,hDevice,&ptr);
    assert(result == ZE_RESULT_SUCCESS);
    return ptr;
  }
  
  void rthwifFreeAccelBuffer(void* ptr, sycl::context context)
  {
    if (ptr == nullptr) return;
    ze_context_handle_t hContext = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(context);
    ze_result_t result = zeMemFree(hContext,ptr);
    assert(result == ZE_RESULT_SUCCESS);
  }

  struct GEOMETRY_INSTANCE_DESC : RTHWIF_GEOMETRY_INSTANCE_DESC
  {
    RTHWIF_TRANSFORM_FLOAT4X4_COLUMN_MAJOR xfmdata;
  };

  struct GEOMETRY_TYPE
  {
    GEOMETRY_TYPE(RTHWIF_GEOMETRY_TYPE type, size_t extraBytes = 0)
      : type(type), extraBytes(extraBytes) {}
    
    RTHWIF_GEOMETRY_TYPE type;
    size_t extraBytes;
  };

  size_t sizeof_RTHWIF_GEOMETRY(GEOMETRY_TYPE type)
  {
    switch (type.type) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return sizeof(RTHWIF_GEOMETRY_TRIANGLES_DESC)+type.extraBytes;
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return sizeof(RTHWIF_GEOMETRY_QUADS_DESC)+type.extraBytes;
    case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR: return sizeof(RTHWIF_GEOMETRY_AABBS_FPTR_DESC)+type.extraBytes;
    case RTHWIF_GEOMETRY_TYPE_INSTANCE  : return sizeof(RTHWIF_GEOMETRY_INSTANCE_DESC)+type.extraBytes;
    default: assert(false); return 0;
    }
  }

  size_t alignof_RTHWIF_GEOMETRY(GEOMETRY_TYPE type)
  {
    switch (type.type) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return alignof(RTHWIF_GEOMETRY_TRIANGLES_DESC);
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return alignof(RTHWIF_GEOMETRY_QUADS_DESC);
    case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR: return alignof(RTHWIF_GEOMETRY_AABBS_FPTR_DESC);
    case RTHWIF_GEOMETRY_TYPE_INSTANCE  : return alignof(RTHWIF_GEOMETRY_INSTANCE_DESC);
    default: assert(false); return 0;
    }
  }

  RTHWIF_GEOMETRY_FLAGS getGeometryFlags(Scene* scene, Geometry* geom)
  {
    /* invoke any hit callback when Embree filter functions are present */
    RTHWIF_GEOMETRY_FLAGS gflags = RTHWIF_GEOMETRY_FLAG_OPAQUE;
    if (scene->hasContextFilterFunction() || geom->hasFilterFunctions())
      gflags = RTHWIF_GEOMETRY_FLAG_NONE;
    
    /* invoke any hit callback when high mask bits are enabled */
    if (geom->mask & 0xFFFFFF80)
      gflags = RTHWIF_GEOMETRY_FLAG_NONE;
    
    return gflags;
  }

  void createGeometryDesc(RTHWIF_GEOMETRY_TRIANGLES_DESC* out, Scene* scene, TriangleMesh* geom)
  {
    memset(out,0,sizeof(RTHWIF_GEOMETRY_TRIANGLES_DESC));
    out->geometryType = RTHWIF_GEOMETRY_TYPE_TRIANGLES;
    out->geometryFlags = getGeometryFlags(scene,geom);
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->triangleBuffer = (RTHWIF_TRIANGLE_INDICES*) geom->triangles.getPtr();
    out->triangleCount = geom->triangles.size();
    out->triangleStride = geom->triangles.getStride();
    out->vertexBuffer = (RTHWIF_FLOAT3*) geom->vertices0.getPtr();
    out->vertexCount = geom->vertices0.size();
    out->vertexStride = geom->vertices0.getStride();
  }

  void createGeometryDesc(RTHWIF_GEOMETRY_QUADS_DESC* out, Scene* scene, QuadMesh* geom)
  {
    memset(out,0,sizeof(RTHWIF_GEOMETRY_QUADS_DESC));
    out->geometryType = RTHWIF_GEOMETRY_TYPE_QUADS;
    out->geometryFlags = getGeometryFlags(scene,geom);
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->quadBuffer = (RTHWIF_QUAD_INDICES*) geom->quads.getPtr();
    out->quadCount = geom->quads.size();
    out->quadStride = geom->quads.getStride();
    out->vertexBuffer = (RTHWIF_FLOAT3*) geom->vertices0.getPtr();
    out->vertexCount = geom->vertices0.size();
    out->vertexStride = geom->vertices0.getStride();
  }

  void getProceduralAABB(const uint32_t primIDStart, const uint32_t primIDCount, void* geomUserPtr, void* buildUserPtr, RTHWIF_AABB* boundsOut)
  {
    BBox1f time_range = * (BBox1f*) buildUserPtr;
    Geometry* geom = (Geometry*) geomUserPtr;
      
    for (uint32_t i=0; i<primIDCount; i++)
    {
      const uint32_t primID = primIDStart+i;
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

  void createGeometryDescProcedural(RTHWIF_GEOMETRY_AABBS_FPTR_DESC* out, Scene* scene, Geometry* geom)
  {
    uint32_t numPrimitives = geom->size();
    if (GridMesh* mesh = dynamic_cast<GridMesh*>(geom))
      numPrimitives = mesh->getNumTotalQuads(); // FIXME: slow
    
    memset(out,0,sizeof(RTHWIF_GEOMETRY_AABBS_FPTR_DESC));
    out->geometryType = RTHWIF_GEOMETRY_TYPE_AABBS_FPTR;
    out->geometryFlags = RTHWIF_GEOMETRY_FLAG_NONE;
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->primCount = numPrimitives;
    out->getBounds = getProceduralAABB;
    out->geomUserPtr = geom;
  }
  
  void createGeometryDesc(GEOMETRY_INSTANCE_DESC* out, Scene* scene, Instance* geom)
  {
    assert(geom->gsubtype == AccelSet::GTY_SUBTYPE_INSTANCE_QUATERNION);
    memset(out,0,sizeof(GEOMETRY_INSTANCE_DESC));
    out->geometryType = RTHWIF_GEOMETRY_TYPE_INSTANCE;
    out->instanceFlags = RTHWIF_INSTANCE_FLAG_NONE;
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->instanceUserID = 0;
    const AffineSpace3fa local2world = geom->getLocal2World();
    out->transformFormat = RTHWIF_TRANSFORM_FORMAT_FLOAT4X4_COLUMN_MAJOR;
    out->transform = (float*) &out->xfmdata;
    out->bounds = (RTHWIF_AABB*) &dynamic_cast<Scene*>(geom->object)->hwaccel_bounds;
    out->xfmdata = *(RTHWIF_TRANSFORM_FLOAT4X4_COLUMN_MAJOR*) &local2world;
    
    EmbreeHWAccel* hwaccel = (EmbreeHWAccel*) dynamic_cast<Scene*>(geom->object)->hwaccel.data();
    out->accel = hwaccel->AccelTable[0];
  }

  void createGeometryDesc(RTHWIF_GEOMETRY_INSTANCE_DESC* out, Scene* scene, Instance* geom)
  {
    assert(geom->gsubtype == AccelSet::GTY_SUBTYPE_DEFAULT);
    memset(out,0,sizeof(RTHWIF_GEOMETRY_INSTANCE_DESC));
    out->geometryType = RTHWIF_GEOMETRY_TYPE_INSTANCE;
    out->instanceFlags = RTHWIF_INSTANCE_FLAG_NONE;
    out->geometryMask = mask32_to_mask8(geom->mask);
    out->instanceUserID = 0;
    out->transformFormat = RTHWIF_TRANSFORM_FORMAT_FLOAT4X4_COLUMN_MAJOR;
    out->transform = (float*) &geom->local2world[0];
    out->bounds = (RTHWIF_AABB*) &dynamic_cast<Scene*>(geom->object)->hwaccel_bounds;
    EmbreeHWAccel* hwaccel = (EmbreeHWAccel*) dynamic_cast<Scene*>(geom->object)->hwaccel.data();
    out->accel = hwaccel->AccelTable[0];
  }

  void createGeometryDesc(char* out, Scene* scene, Geometry* geom, GEOMETRY_TYPE type)
  {
    switch (type.type) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return createGeometryDesc((RTHWIF_GEOMETRY_TRIANGLES_DESC*)out,scene,dynamic_cast<TriangleMesh*>(geom));
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return createGeometryDesc((RTHWIF_GEOMETRY_QUADS_DESC*)out,scene,dynamic_cast<QuadMesh*>(geom));
    case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR: return createGeometryDescProcedural((RTHWIF_GEOMETRY_AABBS_FPTR_DESC*)out,scene,geom);
    case RTHWIF_GEOMETRY_TYPE_INSTANCE:
      if (type.extraBytes) return createGeometryDesc((GEOMETRY_INSTANCE_DESC*)out,scene,dynamic_cast<Instance*>(geom));
      else                 return createGeometryDesc((RTHWIF_GEOMETRY_INSTANCE_DESC*)out,scene,dynamic_cast<Instance*>(geom));
    default: assert(false);
    }
  }

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)      
  RTHWIF_ERROR rthwifGetAccelSizeGPU(const RTHWIF_BUILD_ACCEL_ARGS& args_i, RTHWIF_ACCEL_SIZE& size_o);
  RTHWIF_ERROR rthwifBuildAccelGPU(const RTHWIF_BUILD_ACCEL_ARGS& args);
#endif
  
  BBox3f rthwifBuild(Scene* scene, RTCBuildQuality quality_flags, AccelBuffer& accel, int gpu_build)
  {
    DeviceGPU *gpu_device = dynamic_cast<DeviceGPU*>(scene->device);
    
    auto getType = [&](unsigned int geomID) -> GEOMETRY_TYPE
    {
      /* no HW support for MB yet */
      if (scene->get(geomID)->numTimeSegments() > 0)
        return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR;

      switch (scene->get(geomID)->getType()) {
      case Geometry::GTY_FLAT_LINEAR_CURVE    : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ROUND_LINEAR_CURVE   : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ORIENTED_LINEAR_CURVE: return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_CONE_LINEAR_CURVE    : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      
      case Geometry::GTY_FLAT_BEZIER_CURVE    : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ROUND_BEZIER_CURVE   : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ORIENTED_BEZIER_CURVE: return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      
      case Geometry::GTY_FLAT_BSPLINE_CURVE    : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ROUND_BSPLINE_CURVE   : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ORIENTED_BSPLINE_CURVE: return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      
      case Geometry::GTY_FLAT_HERMITE_CURVE    : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ROUND_HERMITE_CURVE   : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ORIENTED_HERMITE_CURVE: return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      
      case Geometry::GTY_FLAT_CATMULL_ROM_CURVE    : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ROUND_CATMULL_ROM_CURVE   : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE: return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      
      case Geometry::GTY_TRIANGLE_MESH: return RTHWIF_GEOMETRY_TYPE_TRIANGLES; break;
      case Geometry::GTY_QUAD_MESH    : return RTHWIF_GEOMETRY_TYPE_QUADS; break;
      case Geometry::GTY_GRID_MESH    : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_SUBDIV_MESH  : assert(false); return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      
      case Geometry::GTY_SPHERE_POINT       : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_DISC_POINT         : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_ORIENTED_DISC_POINT: return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      
      case Geometry::GTY_USER_GEOMETRY     : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;

#if RTC_MAX_INSTANCE_LEVEL_COUNT < 2
      case Geometry::GTY_INSTANCE_CHEAP    :
      case Geometry::GTY_INSTANCE_EXPENSIVE: {
        Instance* instance = scene->get<Instance>(geomID);
        EmbreeHWAccel* object = (EmbreeHWAccel*)((Scene*)instance->object)->hwaccel.data();
        if (object->numTimeSegments > 1) return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; // we need to handle instances in procedural mode if instanced scene has motion blur
        if (instance->mask & 0xFFFFFF80) return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; // we need to handle instances in procedural mode if high mask bits are set
        else if (instance->gsubtype == AccelSet::GTY_SUBTYPE_INSTANCE_QUATERNION)
          return GEOMETRY_TYPE(RTHWIF_GEOMETRY_TYPE_INSTANCE,sizeof(GEOMETRY_INSTANCE_DESC)-sizeof(RTHWIF_GEOMETRY_INSTANCE_DESC));
        else return RTHWIF_GEOMETRY_TYPE_INSTANCE;
      }
#else
      case Geometry::GTY_INSTANCE_CHEAP    : return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
      case Geometry::GTY_INSTANCE_EXPENSIVE: return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR; break;
#endif

      default: assert(false); return RTHWIF_GEOMETRY_TYPE_AABBS_FPTR;
      }
    };

    // TODO: prefetch in GPU mode

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
    
    /* fill geomdesc buffers */
    const size_t geomDescrBytes = sizeof(RTHWIF_GEOMETRY_DESC*)*numGeometries;
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)    
    RTHWIF_GEOMETRY_DESC** geomDescr = (RTHWIF_GEOMETRY_DESC**)sycl::aligned_alloc(64,geomDescrBytes,gpu_device->getGPUDevice(),gpu_device->getGPUContext(),sycl::usm::alloc::shared);
    assert(geomDescr);        
    char *geomDescrData = (char*)sycl::aligned_alloc(64,    totalBytes,gpu_device->getGPUDevice(),gpu_device->getGPUContext(),sycl::usm::alloc::shared);
    assert(geomDescrData);
#else    
    std::vector<RTHWIF_GEOMETRY_DESC*> geomDescr(numGeometries);
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
      geomDescr[geomID] = (RTHWIF_GEOMETRY_DESC*) &geomDescrData[offset];
      offset += sizeof_RTHWIF_GEOMETRY(type);
      assert(offset <= geomDescrData.size());
    }

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    RTHWIF_PARALLEL_OPERATION parallelOperation = nullptr;    
#else
    RTHWIF_PARALLEL_OPERATION parallelOperation = rthwifNewParallelOperation();    
#endif    
    /* estimate static accel size */
    BBox1f time_range(0,1);
    RTHWIF_AABB bounds;
    RTHWIF_BUILD_ACCEL_ARGS args;
    memset(&args,0,sizeof(args));
    args.structBytes = sizeof(args);
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    args.geometries = (const RTHWIF_GEOMETRY_DESC**) geomDescr;    
#else    
    args.geometries = (const RTHWIF_GEOMETRY_DESC**) geomDescr.data();
#endif    
    args.numGeometries = numGeometries;
    args.accelBuffer = nullptr;
    args.accelBufferBytes = 0;
    args.scratchBuffer = nullptr;
    args.scratchBufferBytes = 0;
    args.quality = RTHWIF_BUILD_QUALITY_MEDIUM;
    args.flags = RTHWIF_BUILD_FLAG_NONE;
    args.parallelOperation = parallelOperation;
    args.boundsOut = &bounds;
    args.buildUserPtr = &time_range;
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    args.dispatchGlobalsPtr = dynamic_cast<DeviceGPU*>(scene->device)->dispatchGlobalsPtr;
#endif

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    args.deviceGPU = dynamic_cast<DeviceGPU*>(scene->device);
#endif

    
    RTHWIF_ACCEL_SIZE sizeTotal;
    memset(&sizeTotal,0,sizeof(RTHWIF_ACCEL_SIZE));
    sizeTotal.structBytes = sizeof(RTHWIF_ACCEL_SIZE);
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    RTHWIF_ERROR err = rthwifGetAccelSizeGPU(args,sizeTotal);    
#else    
    RTHWIF_ERROR err = rthwifGetAccelSize(args,sizeTotal);
#endif    
    if (err != RTHWIF_ERROR_NONE)
      throw_RTCError(RTC_ERROR_UNKNOWN,"BVH size estimate failed");

    /* allocate scratch buffer */

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    // === scratch buffer === 
    char *scratchBuffer  = (char*)sycl::aligned_alloc(64,sizeTotal.scratchBufferBytes,gpu_device->getGPUDevice(),gpu_device->getGPUContext(),sycl::usm::alloc::shared);
    assert(scratchBuffer);
    args.scratchBuffer = scratchBuffer;
    args.scratchBufferBytes = sizeTotal.scratchBufferBytes;
    
    // === host device communication buffer ===
    char *hostDeviceCommPtr = (char*)sycl::aligned_alloc(64,sizeof(uint)*4,gpu_device->getGPUDevice(),gpu_device->getGPUContext(),sycl::usm::alloc::host); // FIXME
    assert(host_device_tasks);
    args.hostDeviceCommPtr = hostDeviceCommPtr;
#else        
    std::vector<char> scratchBuffer(sizeTotal.scratchBufferBytes);
    args.scratchBuffer = scratchBuffer.data();
    args.scratchBufferBytes = scratchBuffer.size();
#endif
    
    size_t headerBytes = sizeof(EmbreeHWAccel) + std::max(1u,maxTimeSegments)*8;
    align(headerBytes,128);

    /* build BVH */
    BBox3f fullBounds = empty;
    while (true)
    {
      /* estimate size of all mblur BVHs */
      RTHWIF_ACCEL_SIZE size;
      size.accelBufferExpectedBytes  = maxTimeSegments*sizeTotal.accelBufferExpectedBytes;
      size.accelBufferWorstCaseBytes = maxTimeSegments*sizeTotal.accelBufferWorstCaseBytes;
      size_t bytes = headerBytes+size.accelBufferExpectedBytes;
      
      /* allocate BVH data */
      if (accel.size() < bytes) accel.resize(bytes);
#if !defined(EMBREE_SYCL_GPU_BVH_BUILDER)      
      memset(accel.data(),0,accel.size()); // FIXME: not required
#endif
      
      /* build BVH for each time segment */
      for (uint32_t i=0; i<maxTimeSegments; i++)
      {
        const float t0 = float(i+0)/float(maxTimeSegments);
        const float t1 = float(i+1)/float(maxTimeSegments);
        time_range = BBox1f(t0,t1);

        // why again?
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
        args.geometries = (const RTHWIF_GEOMETRY_DESC**) geomDescr;
        size_t new_accelBufferBytes = 0;
        args.accelBufferBytesOut = &new_accelBufferBytes;
#else        
        args.geometries = (const RTHWIF_GEOMETRY_DESC**) geomDescr.data();
#endif        
        args.numGeometries = numGeometries; 
        args.accelBuffer = accel.data() + headerBytes + i*sizeTotal.accelBufferExpectedBytes; // why + headerBytes?
        args.accelBufferBytes = sizeTotal.accelBufferExpectedBytes;
        bounds = { { INFINITY, INFINITY, INFINITY }, { -INFINITY, -INFINITY, -INFINITY } };  // why does the host initializes the bounds

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
        err = rthwifBuildAccelGPU(args);
        if (err == RTHWIF_ERROR_NONE && gpu_device->verbosity(2))
        {
          QBVH6* qbvh   = (QBVH6*)args.accelBuffer;
          qbvh->print(std::cout,qbvh->root(),0,6);
          BVHStatistics stats = qbvh->computeStatistics();      
          stats.print(std::cout);
          stats.print_raw(std::cout);
          PRINT("VERBOSE STATS DONE");
        }        
        
#else
        err = rthwifBuildAccel(args);        
#endif        
        
        if (args.parallelOperation) {
          assert(err == RTHWIF_ERROR_PARALLEL_OPERATION);
          uint32_t maxThreads = rthwifGetParallelOperationMaxConcurrency(parallelOperation);
          parallel_for(maxThreads, [&](uint32_t) { err = rthwifJoinParallelOperation(parallelOperation); });
        }

        if (err == RTHWIF_ERROR_NONE) // added if
          fullBounds.extend(*(BBox3f*) args.boundsOut);

        if (err == RTHWIF_ERROR_RETRY)
        {
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
          sizeTotal.accelBufferExpectedBytes  = new_accelBufferBytes;
          sizeTotal.accelBufferWorstCaseBytes = new_accelBufferBytes;
#else          
          if (sizeTotal.accelBufferExpectedBytes == sizeTotal.accelBufferWorstCaseBytes) // why?
            throw_RTCError(RTC_ERROR_UNKNOWN,"build error");
          
          sizeTotal.accelBufferExpectedBytes = std::min(sizeTotal.accelBufferWorstCaseBytes,(size_t(1.2*sizeTotal.accelBufferExpectedBytes)+127)&-128);
#endif          
          
          break;
        }
        
        if (err != RTHWIF_ERROR_NONE) break;
      }
      if (err != RTHWIF_ERROR_RETRY) break;
    }

    if (err != RTHWIF_ERROR_NONE)
      throw_RTCError(RTC_ERROR_UNKNOWN,"build error");

    EmbreeHWAccel* hwaccel = (EmbreeHWAccel*) accel.data();
    hwaccel->numTimeSegments = maxTimeSegments;

    for (size_t i=0; i<maxTimeSegments; i++)
      hwaccel->AccelTable[i] = (char*)hwaccel + headerBytes + i*sizeTotal.accelBufferExpectedBytes;

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
    sycl::free(geomDescr        ,gpu_device->getGPUContext());
    sycl::free(geomDescrData    ,gpu_device->getGPUContext());    
    sycl::free(scratchBuffer    ,gpu_device->getGPUContext());
    sycl::free(hostDeviceCommPtr,gpu_device->getGPUContext());
#endif    
    return fullBounds;
  }

}
