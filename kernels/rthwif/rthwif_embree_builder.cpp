// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "rthwif_embree.h"
#include "rthwif_embree_builder.h"
#include "builder/qbvh6_builder_sah.h"
#include "../builders/primrefgen.h"
#include "rthwif_internal.h"
#include "rthwif_builder.h"

#include <level_zero/ze_api.h>

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
    uint64_t callStackHandlerKSP;             // this is the KSP of the continuation handler that is invoked by BTD when the read KSP is 0
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
    /* check for Intel vendor */
    const uint32_t vendor_id = sycl_device.get_info<sycl::info::device::vendor_id>();
    if (vendor_id != 0x8086) return false;

    /* check for supported device ID */
    auto native_device = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(sycl_device);
    ze_device_properties_t device_props{ ZE_STRUCTURE_TYPE_DEVICE_PROPERTIES };
    ze_result_t status = zeDeviceGetProperties(native_device, &device_props);
    if (status != ZE_RESULT_SUCCESS)
      return false;

    const uint32_t device_id = device_props.deviceId;
    
    // DG2
    if (0x4F80 <= device_id && device_id <= 0x4F88) return true;
    if (0x5690 <= device_id && device_id <= 0x5698) return true;
    if (0x56A0 <= device_id && device_id <= 0x56A6) return true;
    if (0x56B0 <= device_id && device_id <= 0x56B3) return true;
    if (0x56C0 <= device_id && device_id <= 0x56C1) return true;
       
    // ATS-M
    if (0x0201 <= device_id && device_id <= 0x0210) return true;

    // PVC
    if (0x0BD0 <= device_id && device_id <= 0x0BDB) return true;
    if (device_id == 0x0BE5) return true;

    return false;
  }

  struct GEOMETRY_INSTANCE_DESC : RTHWIF_GEOMETRY_INSTANCE_DESC
  {
    RTHWIF_TRANSFORM4X4 xfmdata;
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
    out->transform = &out->xfmdata;
    out->xfmdata = *(RTHWIF_TRANSFORM4X4*) &local2world;
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
    out->transform = (RTHWIF_TRANSFORM4X4*) &geom->local2world[0];
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

  BBox3fa rthwifBuildDriver(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& accel)
  {
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

    /* calculate maximal number of motion blur time segments in scene */
    uint32_t maxTimeSegments = 0;
    for (size_t geomID=0; geomID<scene->size(); geomID++)
    {
      Geometry* geom = scene->get(geomID);
      if (geom == nullptr) continue;
      maxTimeSegments = std::max(maxTimeSegments, geom->numTimeSegments());
    }

    /* calculate size of geometry descriptor buffer */
    size_t totalPrimitives = 0;
    size_t bytesStatic = 0, bytesMBlur = 0;
    for (size_t geomID=0; geomID<scene->size(); geomID++)
    {
      Geometry* geom = scene->get(geomID);
      if (geom == nullptr) continue;
      totalPrimitives += geom->size();
      
      const GEOMETRY_TYPE type = getType(geomID);
      if (geom->numTimeSegments() == 0) {
        align(bytesStatic,alignof_RTHWIF_GEOMETRY(type));
        bytesStatic += sizeof_RTHWIF_GEOMETRY(type);
      } else {
        align(bytesMBlur,alignof_RTHWIF_GEOMETRY(type));
        bytesMBlur += sizeof_RTHWIF_GEOMETRY(type);
      }
    }

    /* fill geomdesc buffers */
    std::vector<RTHWIF_GEOMETRY_DESC*> geomStatic(scene->size());
    std::vector<RTHWIF_GEOMETRY_DESC*> geomMBlur(scene->size());
    std::vector<char> geomDescStatic(bytesStatic);
    std::vector<char> geomDescMBlur(bytesMBlur);

    size_t offsetStatic = 0, offsetMBlur = 0;
    for (size_t geomID=0; geomID<scene->size(); geomID++)
    {
      geomStatic[geomID] = nullptr;
      geomMBlur [geomID] = nullptr;
      
      Geometry* geom = scene->get(geomID);
      if (geom == nullptr) continue;
      
      const GEOMETRY_TYPE type = getType(geomID);
      
      if (geom->numTimeSegments() == 0) {
        align(offsetStatic,alignof_RTHWIF_GEOMETRY(type));
        createGeometryDesc(&geomDescStatic[offsetStatic],scene,scene->get(geomID),type);
        geomStatic[geomID] = (RTHWIF_GEOMETRY_DESC*) &geomDescStatic[offsetStatic];
        offsetStatic += sizeof_RTHWIF_GEOMETRY(type);
        assert(offsetStatic <= geomDescStatic.size());
      } else {
        align(offsetMBlur,alignof_RTHWIF_GEOMETRY(type));
        createGeometryDesc(&geomDescMBlur[offsetMBlur],scene,scene->get(geomID),type);
        geomMBlur[geomID] = (RTHWIF_GEOMETRY_DESC*) &geomDescMBlur[offsetMBlur];
        offsetMBlur += sizeof_RTHWIF_GEOMETRY(type);
        assert(offsetMBlur <= geomDescMBlur.size());
      }
    }

    RTHWIF_PARALLEL_OPERATION parallelOperation = rthwifNewParallelOperation();

    /* estimate static accel size */
    BBox1f time_range(0,1);
    RTHWIF_AABB bounds;
    RTHWIF_BUILD_ACCEL_ARGS args;
    memset(&args,0,sizeof(args));
    args.structBytes = sizeof(args);
    args.dispatchGlobalsPtr = dynamic_cast<DeviceGPU*>(scene->device)->dispatchGlobalsPtr;
    args.geometries = (const RTHWIF_GEOMETRY_DESC**) geomStatic.data();
    args.numGeometries = geomStatic.size();
    args.accelBuffer = nullptr;
    args.accelBufferBytes = 0;
    args.quality = RTHWIF_BUILD_QUALITY_MEDIUM;
    args.flags = RTHWIF_BUILD_FLAG_NONE;
    args.parallelOperation = parallelOperation;
    args.boundsOut = &bounds;
    args.buildUserPtr = &time_range;
    
    RTHWIF_ACCEL_SIZE sizeStatic;
    memset(&sizeStatic,0,sizeof(RTHWIF_ACCEL_SIZE));
    sizeStatic.structBytes = sizeof(RTHWIF_ACCEL_SIZE);
    RTHWIF_ERROR err = rthwifGetAccelSize(args,sizeStatic);
    if (err != RTHWIF_ERROR_NONE)
      throw_RTCError(RTC_ERROR_UNKNOWN,"BVH size estimate failed");

    /* estimate MBlur accel size */
    args.geometries = (const RTHWIF_GEOMETRY_DESC**) geomMBlur.data();
    args.numGeometries = geomMBlur.size();
    
    RTHWIF_ACCEL_SIZE sizeMBlur;
    memset(&sizeMBlur,0,sizeof(RTHWIF_ACCEL_SIZE));
    sizeMBlur.structBytes = sizeof(RTHWIF_ACCEL_SIZE);
    err = rthwifGetAccelSize(args,sizeMBlur);
    if (err != RTHWIF_ERROR_NONE)
      throw_RTCError(RTC_ERROR_UNKNOWN,"BVH size estimate failed");

    size_t headerBytes = sizeof(EmbreeHWAccel) + std::max(1u,maxTimeSegments)*8;
    align(headerBytes,128);

    /* build BVH */
    BBox3f fullBounds = empty;
    //for (size_t bytes = size.accelBufferExpectedBytes; bytes < size.accelBufferWorstCaseBytes; bytes*=1.2)
    while (true)
    {
      /* estimate size of static and mblur BVHs */
      RTHWIF_ACCEL_SIZE size;
      size.accelBufferExpectedBytes  = sizeStatic.accelBufferExpectedBytes  + maxTimeSegments*sizeMBlur.accelBufferExpectedBytes;
      size.accelBufferWorstCaseBytes = sizeStatic.accelBufferWorstCaseBytes + maxTimeSegments*sizeMBlur.accelBufferWorstCaseBytes;
      size_t bytes = headerBytes+size.accelBufferExpectedBytes;
        
      /* allocate BVH data */
      if (accel.size() < bytes) accel = std::move(Device::avector<char,64>(scene->device,bytes));
      memset(accel.data(),0,accel.size()); // FIXME: not required

      /*uint32_t N = 10;
      double dt_avg = 0.0f;
      for (size_t i=0; i<N; i++)
      {
      double time0 = getSeconds();*/
        
      /* build static BVH */
      args.geometries = (const RTHWIF_GEOMETRY_DESC**) geomStatic.data();
      args.numGeometries = geomStatic.size();
      args.accelBuffer = accel.data() + headerBytes;
      args.accelBufferBytes = sizeStatic.accelBufferExpectedBytes;
      bounds = { { INFINITY, INFINITY, INFINITY }, { -INFINITY, -INFINITY, -INFINITY } };
      
      err = rthwifBuildAccel(args);
      if (args.parallelOperation) {
        assert(err == RTHWIF_ERROR_PARALLEL_OPERATION);
        uint32_t maxThreads = rthwifGetParallelOperationMaxConcurrency(parallelOperation);
        parallel_for(maxThreads, [&](uint32_t) { err = rthwifJoinParallelOperation(parallelOperation); });
      }

      /*double time1 = getSeconds();
      dt_avg += time1-time0;
      }
      dt_avg /= (double) N;
      std::cout << "build performance " << double(totalPrimitives)/dt_avg*1E-6 << " Mprims/s, " << dt_avg*1000.0 << " ms" << std::endl;*/
      
      const BBox3f staticBounds = *(BBox3f*) args.boundsOut;
      fullBounds.extend(staticBounds);
      
      if (err == RTHWIF_ERROR_RETRY)
      {
        if (sizeStatic.accelBufferExpectedBytes == sizeStatic.accelBufferWorstCaseBytes)
          throw_RTCError(RTC_ERROR_UNKNOWN,"build error");
        
        sizeStatic.accelBufferExpectedBytes = std::min(sizeStatic.accelBufferWorstCaseBytes,(size_t(1.2*sizeStatic.accelBufferExpectedBytes)+127)&-128);
        continue;
      }

      /* build BVH for each time segment */
      for (uint32_t i=0; i<maxTimeSegments; i++)
      {
        const float t0 = float(i+0)/float(maxTimeSegments);
        const float t1 = float(i+1)/float(maxTimeSegments);
        time_range = BBox1f(t0,t1);
        
        args.geometries = (const RTHWIF_GEOMETRY_DESC**) geomMBlur.data();
        args.numGeometries = geomMBlur.size();
        args.accelBuffer = accel.data() + headerBytes + sizeStatic.accelBufferExpectedBytes + i*sizeMBlur.accelBufferExpectedBytes;
        args.accelBufferBytes = sizeMBlur.accelBufferExpectedBytes;
        args.linkAccel.Accel = nullptr;
        if (!staticBounds.empty()) {
          args.linkAccel.Accel = accel.data() + headerBytes;
          args.linkAccel.bounds = (RTHWIF_AABB&) staticBounds;
        }
        bounds = { { INFINITY, INFINITY, INFINITY }, { -INFINITY, -INFINITY, -INFINITY } };
        
        err = rthwifBuildAccel(args);
        if (args.parallelOperation) {
          assert(err == RTHWIF_ERROR_PARALLEL_OPERATION);
          uint32_t maxThreads = rthwifGetParallelOperationMaxConcurrency(parallelOperation);
          parallel_for(maxThreads, [&](uint32_t) { err = rthwifJoinParallelOperation(parallelOperation); });
        }
        
        fullBounds.extend(*(BBox3f*) args.boundsOut);

        if (err == RTHWIF_ERROR_RETRY)
        {
          if (sizeMBlur.accelBufferExpectedBytes == sizeMBlur.accelBufferWorstCaseBytes)
            throw_RTCError(RTC_ERROR_UNKNOWN,"build error");
          
          sizeMBlur.accelBufferExpectedBytes = std::min(sizeMBlur.accelBufferWorstCaseBytes,(size_t(1.2*sizeMBlur.accelBufferExpectedBytes)+127)&-128);
          break;
        }
        
        if (err != RTHWIF_ERROR_NONE) break;
      }
      if (err != RTHWIF_ERROR_RETRY) break;
    }

    if (err != RTHWIF_ERROR_NONE)
      throw_RTCError(RTC_ERROR_UNKNOWN,"build error");

    EmbreeHWAccel* hwaccel = (EmbreeHWAccel*) accel.data();
    hwaccel->bounds[0][0] = fullBounds.lower.x;
    hwaccel->bounds[0][1] = fullBounds.lower.y;
    hwaccel->bounds[0][2] = fullBounds.lower.z;
    hwaccel->bounds[1][0] = fullBounds.upper.x;
    hwaccel->bounds[1][1] = fullBounds.upper.y;
    hwaccel->bounds[1][2] = fullBounds.upper.z;
    hwaccel->numTimeSegments = maxTimeSegments;

    for (size_t i=0; i<maxTimeSegments; i++)
      hwaccel->AccelTable[i] = (char*)hwaccel + headerBytes + sizeStatic.accelBufferExpectedBytes + i*sizeMBlur.accelBufferExpectedBytes;
    
    if (maxTimeSegments == 0)
      hwaccel->AccelTable[0] = (char*)hwaccel + headerBytes;

    return fullBounds;
  }

  BBox3fa rthwifBuild(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& accel)
  {
    return rthwifBuildDriver(scene,quality_flags,accel);
  }
}
