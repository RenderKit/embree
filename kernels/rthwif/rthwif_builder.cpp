// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#define RTHWIF_EXPORT_API
#include "../../common/tasking/taskscheduler.h"

#include "rthwif_builder.h"
#include "builder/qbvh6_builder_sah.h"
#include "rthwif_internal.h"

#if defined(EMBREE_LEVEL_ZERO)
#include <level_zero/ze_api.h>
#endif

namespace embree
{
  using namespace embree::isa;

  static std::unique_ptr<tbb::task_arena> g_arena;
  
  inline RTHWIF_TRIANGLE_INDICES getPrimitive(const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom, uint32_t primID) {
    assert(primID < geom->triangleCount);
    return *(RTHWIF_TRIANGLE_INDICES*)((char*)geom->triangleBuffer + primID*geom->triangleStride);
  }
  
  inline Vec3f getVertex(const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom, uint32_t vertexID) {
    assert(vertexID < geom->vertexCount);
    return *(Vec3f*)((char*)geom->vertexBuffer + vertexID*geom->vertexStride);
  }
  
  inline RTHWIF_QUAD_INDICES getPrimitive(const RTHWIF_GEOMETRY_QUADS_DESC* geom, uint32_t primID) {
    assert(primID < geom->quadCount);
    return *(RTHWIF_QUAD_INDICES*)((char*)geom->quadBuffer + primID*geom->quadStride);
  }
  
  inline Vec3f getVertex(const RTHWIF_GEOMETRY_QUADS_DESC* geom, uint32_t vertexID) {
    assert(vertexID < geom->vertexCount);
    return *(Vec3f*)((char*)geom->vertexBuffer + vertexID*geom->vertexStride);
  }

  inline AffineSpace3fa getTransform(const RTHWIF_GEOMETRY_INSTANCE_DESC* geom)
  {
    switch (geom->transformFormat)
    {
    case RTHWIF_TRANSFORM_FORMAT_FLOAT3X4_COLUMN_MAJOR: {
      const RTHWIF_TRANSFORM_FLOAT3X4_COLUMN_MAJOR* xfm = (const RTHWIF_TRANSFORM_FLOAT3X4_COLUMN_MAJOR*) geom->transform;
      return {
        { xfm->vx_x, xfm->vx_y, xfm->vx_z },
        { xfm->vy_x, xfm->vy_y, xfm->vy_z },
        { xfm->vz_x, xfm->vz_y, xfm->vz_z },
        { xfm-> p_x, xfm-> p_y, xfm-> p_z }
      };
    }
    case RTHWIF_TRANSFORM_FORMAT_FLOAT4X4_COLUMN_MAJOR: {
      const RTHWIF_TRANSFORM_FLOAT4X4_COLUMN_MAJOR* xfm = (const RTHWIF_TRANSFORM_FLOAT4X4_COLUMN_MAJOR*) geom->transform;
      return {
        { xfm->vx_x, xfm->vx_y, xfm->vx_z },
        { xfm->vy_x, xfm->vy_y, xfm->vy_z },
        { xfm->vz_x, xfm->vz_y, xfm->vz_z },
        { xfm-> p_x, xfm-> p_y, xfm-> p_z }
      };
    }
    case RTHWIF_TRANSFORM_FORMAT_FLOAT3X4_ROW_MAJOR: {
      const RTHWIF_TRANSFORM_FLOAT3X4_ROW_MAJOR* xfm = (const RTHWIF_TRANSFORM_FLOAT3X4_ROW_MAJOR*) geom->transform;
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
  
  inline void verifyGeometryDesc(const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom)
  {
    if (geom->reserved0 != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->reserved1 != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->triangleCount && geom->triangleBuffer == nullptr) throw std::runtime_error("no triangle buffer specified");
    if (geom->vertexCount   && geom->vertexBuffer   == nullptr) throw std::runtime_error("no vertex buffer specified");
  }

  inline void verifyGeometryDesc(const RTHWIF_GEOMETRY_QUADS_DESC* geom)
  {
    if (geom->reserved0 != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->reserved1 != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->quadCount   && geom->quadBuffer   == nullptr) throw std::runtime_error("no quad buffer specified");
    if (geom->vertexCount && geom->vertexBuffer == nullptr) throw std::runtime_error("no vertex buffer specified");
  }

  inline void verifyGeometryDesc(const RTHWIF_GEOMETRY_AABBS_FPTR_DESC* geom)
  {
    if (geom->reserved != 0) throw std::runtime_error("reserved member must be 0");
    if (geom->primCount   && geom->getBounds == nullptr) throw std::runtime_error("no bounds function specified");
  }

  inline void verifyGeometryDesc(const RTHWIF_GEOMETRY_INSTANCE_DESC* geom)
  {
    if (geom->transform == nullptr) throw std::runtime_error("no instance transformation specified");
    if (geom->bounds == nullptr) throw std::runtime_error("no acceleration structure bounds specified");
    if (geom->accel == nullptr) throw std::runtime_error("no acceleration structure to instanciate specified");
  }

  inline bool buildBounds(const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom, uint32_t primID, BBox3fa& bbox, void* buildUserPtr)
  {
    if (primID >= geom->triangleCount) return false;
    const RTHWIF_TRIANGLE_INDICES tri = getPrimitive(geom,primID);
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

  inline bool buildBounds(const RTHWIF_GEOMETRY_QUADS_DESC* geom, uint32_t primID, BBox3fa& bbox, void* buildUserPtr)
  {
    if (primID >= geom->quadCount) return false;
    const RTHWIF_QUAD_INDICES tri = getPrimitive(geom,primID);
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

  inline bool buildBounds(const RTHWIF_GEOMETRY_AABBS_FPTR_DESC* geom, uint32_t primID, BBox3fa& bbox, void* buildUserPtr)
  {
    if (primID >= geom->primCount) return false;
    if (geom->getBounds == nullptr) return false;

    BBox3f bounds;
    (geom->getBounds)(primID,1,geom->geomUserPtr,buildUserPtr,(RTHWIF_AABB*)&bounds);
    if (unlikely(!isvalid(bounds.lower))) return false;
    if (unlikely(!isvalid(bounds.upper))) return false;
    if (unlikely(bounds.empty())) return false;
    
    bbox = (BBox3f&) bounds;
    return true;
  }

  inline bool buildBounds(const RTHWIF_GEOMETRY_INSTANCE_DESC* geom, uint32_t primID, BBox3fa& bbox, void* buildUserPtr)
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
  
  RTHWIF_API void rthwifInit()
  {
    uint32_t numThreads = tbb::this_task_arena::max_concurrency();
    g_arena.reset(new tbb::task_arena(numThreads,numThreads));
  }
  
  RTHWIF_API void rthwifExit()
  {
    g_arena.reset();
  }

#if defined(EMBREE_LEVEL_ZERO)
  
  RTHWIF_API RTHWIF_FEATURES rthwifGetSupportedFeatures(ze_device_handle_t hDevice)
  {
    /* check for supported device ID */
    ze_device_properties_t device_props{ ZE_STRUCTURE_TYPE_DEVICE_PROPERTIES };
    ze_result_t status = zeDeviceGetProperties(hDevice, &device_props);
    if (status != ZE_RESULT_SUCCESS)
      return RTHWIF_FEATURES_NONE;

    /* check for Intel vendor */
    const uint32_t vendor_id = device_props.vendorId;
    const uint32_t device_id = device_props.deviceId;
    if (vendor_id != 0x8086) return RTHWIF_FEATURES_NONE;
    
    /* return supported features */
    uint32_t features_xe = RTHWIF_FEATURES_NONE;
    features_xe |= RTHWIF_FEATURES_GEOMETRY_TYPE_TRIANGLES;
    features_xe |= RTHWIF_FEATURES_GEOMETRY_TYPE_QUADS;
    features_xe |= RTHWIF_FEATURES_GEOMETRY_TYPE_AABBS_FPTR;
    features_xe |= RTHWIF_FEATURES_GEOMETRY_TYPE_INSTANCE;
    
    // DG2
    if (0x4F80 <= device_id && device_id <= 0x4F88) return (RTHWIF_FEATURES) features_xe;
    if (0x5690 <= device_id && device_id <= 0x5698) return (RTHWIF_FEATURES) features_xe;
    if (0x56A0 <= device_id && device_id <= 0x56A6) return (RTHWIF_FEATURES) features_xe;
    if (0x56B0 <= device_id && device_id <= 0x56B3) return (RTHWIF_FEATURES) features_xe;
    if (0x56C0 <= device_id && device_id <= 0x56C1) return (RTHWIF_FEATURES) features_xe;
       
    // PVC
    if (0x0BD0 <= device_id && device_id <= 0x0BDB) return (RTHWIF_FEATURES) features_xe;
    if (device_id == 0x0BE5                       ) return (RTHWIF_FEATURES) features_xe;

    return RTHWIF_FEATURES_NONE;
  }
  
#else

  RTHWIF_API RTHWIF_FEATURES rthwifGetSupportedFeatures(ze_device_handle_t hDevice)
  {
    uint32_t features_xe = RTHWIF_FEATURES_NONE;
    features_xe |= RTHWIF_FEATURES_GEOMETRY_TYPE_TRIANGLES;
    features_xe |= RTHWIF_FEATURES_GEOMETRY_TYPE_QUADS;
    features_xe |= RTHWIF_FEATURES_GEOMETRY_TYPE_AABBS_FPTR;
    features_xe |= RTHWIF_FEATURES_GEOMETRY_TYPE_INSTANCE;
    return RTHWIF_FEATURES(features_xe);
  }

#endif

  uint32_t getNumPrimitives(const RTHWIF_GEOMETRY_DESC* geom)
  {
    switch (geom->geometryType) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return ((RTHWIF_GEOMETRY_TRIANGLES_DESC*) geom)->triangleCount;
    case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR : return ((RTHWIF_GEOMETRY_AABBS_FPTR_DESC*) geom)->primCount;
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return ((RTHWIF_GEOMETRY_QUADS_DESC*) geom)->quadCount;
    case RTHWIF_GEOMETRY_TYPE_INSTANCE   : return 1;
    default                              : return 0;
    };
  }
  
  /* fill all arg members that app did not know of yet */
  RTHWIF_BUILD_ACCEL_ARGS rthwifPrepareBuildAccelArgs(const RTHWIF_BUILD_ACCEL_ARGS& args_i)
  {
    RTHWIF_BUILD_ACCEL_ARGS args;
    memset(&args,0,sizeof(RTHWIF_BUILD_ACCEL_ARGS));
    memcpy(&args,&args_i,std::min(sizeof(RTHWIF_BUILD_ACCEL_ARGS),args_i.structBytes));
    args.structBytes = sizeof(RTHWIF_BUILD_ACCEL_ARGS);
    return args;
  }
  
  RTHWIF_API RTHWIF_ERROR rthwifGetAccelSize(const RTHWIF_BUILD_ACCEL_ARGS& args_i, RTHWIF_ACCEL_SIZE& size_o)
  {
    RTHWIF_BUILD_ACCEL_ARGS args = rthwifPrepareBuildAccelArgs(args_i);
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const size_t numGeometries = args.numGeometries;

    auto getSize = [&](uint32_t geomID) -> size_t {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      if (geom == nullptr) return 0;
      return getNumPrimitives(geom);
    };
    
    auto getType = [&](unsigned int geomID)
    {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      assert(geom);
      switch (geom->geometryType) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES : return QBVH6BuilderSAH::TRIANGLE;
      case RTHWIF_GEOMETRY_TYPE_QUADS: return QBVH6BuilderSAH::QUAD;
      case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR: return QBVH6BuilderSAH::PROCEDURAL;
      case RTHWIF_GEOMETRY_TYPE_INSTANCE: return QBVH6BuilderSAH::INSTANCE;
      default: throw std::runtime_error("invalid geometry type");
      };
    };

    /* query memory requirements from builder */
    size_t expectedBytes = 0;
    size_t worstCaseBytes = 0;
    size_t scratchBytes = 0;
    QBVH6BuilderSAH::estimateSize(numGeometries, getSize, getType, args.quality, expectedBytes, worstCaseBytes, scratchBytes);
                            
    /* return size to user */
    RTHWIF_ACCEL_SIZE size;
    memset(&size,0,sizeof(RTHWIF_ACCEL_SIZE));
    size.accelBufferExpectedBytes = expectedBytes;
    size.accelBufferWorstCaseBytes = worstCaseBytes;
    size.scratchBufferBytes = scratchBytes;
    size_t bytes_o = size_o.structBytes;
    memset(&size_o,0,bytes_o);
    memcpy(&size_o,&size,bytes_o);
    size_o.structBytes = bytes_o;
    return RTHWIF_ERROR_NONE;
  }
  
  RTHWIF_API RTHWIF_ERROR rthwifBuildAccelInternal(const RTHWIF_BUILD_ACCEL_ARGS& args_i) try
  {
    /* prepare input arguments */
    const RTHWIF_BUILD_ACCEL_ARGS args = rthwifPrepareBuildAccelArgs(args_i);
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const uint32_t numGeometries = args.numGeometries;

    if (args.accelBuffer == nullptr) return RTHWIF_ERROR_OTHER;

    /* verify input descriptors */
    parallel_for(numGeometries,[&](uint32_t geomID) {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      if (geom == nullptr) return;
      
      switch (geom->geometryType) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : verifyGeometryDesc((RTHWIF_GEOMETRY_TRIANGLES_DESC*)geom); break;
      case RTHWIF_GEOMETRY_TYPE_QUADS      : verifyGeometryDesc((RTHWIF_GEOMETRY_QUADS_DESC*    )geom); break;
      case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR : verifyGeometryDesc((RTHWIF_GEOMETRY_AABBS_FPTR_DESC*)geom); break;
      case RTHWIF_GEOMETRY_TYPE_INSTANCE   : verifyGeometryDesc((RTHWIF_GEOMETRY_INSTANCE_DESC* )geom); break;
      default: throw std::runtime_error("invalid geometry type");
      };
    });
    
    auto getSize = [&](uint32_t geomID) -> size_t {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      if (geom == nullptr) return 0;
      return getNumPrimitives(geom);
    };
    
    auto getType = [&](unsigned int geomID)
    {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      assert(geom);
      switch (geom->geometryType) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES : return QBVH6BuilderSAH::TRIANGLE;
      case RTHWIF_GEOMETRY_TYPE_QUADS: return QBVH6BuilderSAH::QUAD;
      case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR: return QBVH6BuilderSAH::PROCEDURAL;
      case RTHWIF_GEOMETRY_TYPE_INSTANCE: return QBVH6BuilderSAH::INSTANCE;
      default: throw std::runtime_error("invalid geometry type");
      };
    };
    
    auto createPrimRefArray = [&] (evector<PrimRef>& prims, BBox1f time_range, const range<size_t>& r, size_t k, unsigned int geomID) -> PrimInfo
    {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      assert(geom);

      switch (geom->geometryType) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return createGeometryPrimRefArray((RTHWIF_GEOMETRY_TRIANGLES_DESC*)geom,args.buildUserPtr,prims,r,k,geomID);
      case RTHWIF_GEOMETRY_TYPE_QUADS      : return createGeometryPrimRefArray((RTHWIF_GEOMETRY_QUADS_DESC*    )geom,args.buildUserPtr,prims,r,k,geomID);
      case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR: return createGeometryPrimRefArray((RTHWIF_GEOMETRY_AABBS_FPTR_DESC*)geom,args.buildUserPtr,prims,r,k,geomID);
      case RTHWIF_GEOMETRY_TYPE_INSTANCE: return createGeometryPrimRefArray((RTHWIF_GEOMETRY_INSTANCE_DESC* )geom,args.buildUserPtr,prims,r,k,geomID);
      default: throw std::runtime_error("invalid geometry type");
      };
    };
    
    auto getTriangle = [&](unsigned int geomID, unsigned int primID)
    {
      const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom = (const RTHWIF_GEOMETRY_TRIANGLES_DESC*) geometries[geomID];
      assert(geom);
      
      const RTHWIF_TRIANGLE_INDICES tri = getPrimitive(geom,primID);
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
      const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom = (const RTHWIF_GEOMETRY_TRIANGLES_DESC*) geometries[geomID];
      assert(geom);
      const RTHWIF_TRIANGLE_INDICES tri = getPrimitive(geom,primID);
      return Vec3<uint32_t>(tri.v0,tri.v1,tri.v2);
    };
    
    auto getQuad = [&](unsigned int geomID, unsigned int primID)
    {
      const RTHWIF_GEOMETRY_QUADS_DESC* geom = (const RTHWIF_GEOMETRY_QUADS_DESC*) geometries[geomID];
      assert(geom);
                     
      const RTHWIF_QUAD_INDICES quad = getPrimitive(geom,primID);
      const Vec3f p0 = getVertex(geom,quad.v0);
      const Vec3f p1 = getVertex(geom,quad.v1);
      const Vec3f p2 = getVertex(geom,quad.v2);
      const Vec3f p3 = getVertex(geom,quad.v3);

      const GeometryFlags gflags = (GeometryFlags) geom->geometryFlags;
      return QBVH6BuilderSAH::Quad(p0,p1,p2,p3,gflags,geom->geometryMask);
    };
    
    auto getProcedural = [&](unsigned int geomID, unsigned int primID) {
      const RTHWIF_GEOMETRY_AABBS_FPTR_DESC* geom = (const RTHWIF_GEOMETRY_AABBS_FPTR_DESC*) geometries[geomID];
      assert(geom);
      return QBVH6BuilderSAH::Procedural(geom->geometryMask); // FIXME: pass gflags
    };
    
    auto getInstance = [&](unsigned int geomID, unsigned int primID)
    {
      assert(geometries[geomID]);
      assert(geometries[geomID]->geometryType == RTHWIF_GEOMETRY_TYPE_INSTANCE);
      const RTHWIF_GEOMETRY_INSTANCE_DESC* geom = (const RTHWIF_GEOMETRY_INSTANCE_DESC*) geometries[geomID];
      void* accel = geom->accel;
      const AffineSpace3fa local2world = getTransform(geom);
      return QBVH6BuilderSAH::Instance(local2world,accel,geom->geometryMask,geom->instanceUserID); // FIXME: pass instance flags
    };

    /* dispatch globals ptr for debugging purposes */
    void* dispatchGlobalsPtr = nullptr;
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    dispatchGlobalsPtr = args.dispatchGlobalsPtr;
#endif

    bool verbose = false;
    QBVH6BuilderSAH::build(numGeometries, nullptr, 
                           getSize, getType, 
                           createPrimRefArray, getTriangle, getTriangleIndices, getQuad, getProcedural, getInstance,
                           (char*)args.accelBuffer, args.accelBufferBytes,
                           args.scratchBuffer, args.scratchBufferBytes,
                           (BBox3f*) args.boundsOut, args.accelBufferBytesOut,
                           args.quality, verbose, dispatchGlobalsPtr);
    return RTHWIF_ERROR_NONE;
  }
  
  catch (std::bad_alloc&) {
    return RTHWIF_ERROR_RETRY;
  }
  
  catch (...) {
    return RTHWIF_ERROR_OTHER;
  }

  struct RTHWIF_PARALLEL_OPERATION_IMPL
  {
    RTHWIF_ERROR errorCode = RTHWIF_ERROR_NONE;
    tbb::task_group group;
  };

  RTHWIF_API RTHWIF_ERROR rthwifBuildAccel(const RTHWIF_BUILD_ACCEL_ARGS& args_i)
  {
    /* if parallel operation is provided then execute using thread arena inside task group ... */
    if (args_i.parallelOperation)
    {
      RTHWIF_PARALLEL_OPERATION_IMPL* op = (RTHWIF_PARALLEL_OPERATION_IMPL*) args_i.parallelOperation;
      g_arena->execute([&](){ op->group.run([=](){ op->errorCode = rthwifBuildAccelInternal(args_i); }); });
      return RTHWIF_ERROR_PARALLEL_OPERATION;
    }
    /* ... otherwise we just execute inside task arena to avoid spawning of TBB worker threads */
    else
    {
      RTHWIF_ERROR errorCode = RTHWIF_ERROR_NONE;
      g_arena->execute([&](){ errorCode = rthwifBuildAccelInternal(args_i); });
      return errorCode;
    }
  }

  RTHWIF_API RTHWIF_PARALLEL_OPERATION rthwifNewParallelOperation()
  {
    return (RTHWIF_PARALLEL_OPERATION) new RTHWIF_PARALLEL_OPERATION_IMPL;
  }
  
  RTHWIF_API void rthwifDeleteParallelOperation( RTHWIF_PARALLEL_OPERATION  parallelOperation )
  {
    delete (RTHWIF_PARALLEL_OPERATION_IMPL*) parallelOperation;
  }
  
  RTHWIF_API uint32_t rthwifGetParallelOperationMaxConcurrency( RTHWIF_PARALLEL_OPERATION  parallelOperation )
  {
    return tbb::this_task_arena::max_concurrency();
  }
  
  RTHWIF_API RTHWIF_ERROR rthwifJoinParallelOperation( RTHWIF_PARALLEL_OPERATION parallelOperation)
  {
    RTHWIF_PARALLEL_OPERATION_IMPL* op = (RTHWIF_PARALLEL_OPERATION_IMPL*) parallelOperation;
    g_arena->execute([&](){ op->group.wait(); });
    return op->errorCode;
  }
}
