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

  BBox3fa rthwifBuildDirect(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& accel)
  {
    auto getSize = [&](uint32_t geomID) -> size_t {
      Geometry* geom = scene->geometries[geomID].ptr;
      if (geom == nullptr) return 0;
      if (!geom->isEnabled()) return 0;
      if (!(geom->getTypeMask() & Geometry::MTY_ALL)) return 0;
      if (GridMesh* mesh = scene->getSafe<GridMesh>(geomID))
        return mesh->getNumTotalQuads(); // FIXME: slow
      else
        return geom->size();
    };

    auto getType = [&](unsigned int geomID)
    {
      /* no HW support for MB yet */
      if (scene->get(geomID)->numTimeSegments() > 0)
        return QBVH6BuilderSAH::PROCEDURAL;

      switch (scene->get(geomID)->getType()) {
      case Geometry::GTY_FLAT_LINEAR_CURVE    : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ROUND_LINEAR_CURVE   : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_LINEAR_CURVE: return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_CONE_LINEAR_CURVE    : return QBVH6BuilderSAH::PROCEDURAL; break;
      
      case Geometry::GTY_FLAT_BEZIER_CURVE    : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ROUND_BEZIER_CURVE   : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_BEZIER_CURVE: return QBVH6BuilderSAH::PROCEDURAL; break;
      
      case Geometry::GTY_FLAT_BSPLINE_CURVE    : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ROUND_BSPLINE_CURVE   : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_BSPLINE_CURVE: return QBVH6BuilderSAH::PROCEDURAL; break;
      
      case Geometry::GTY_FLAT_HERMITE_CURVE    : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ROUND_HERMITE_CURVE   : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_HERMITE_CURVE: return QBVH6BuilderSAH::PROCEDURAL; break;
      
      case Geometry::GTY_FLAT_CATMULL_ROM_CURVE    : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ROUND_CATMULL_ROM_CURVE   : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE: return QBVH6BuilderSAH::PROCEDURAL; break;
      
      case Geometry::GTY_TRIANGLE_MESH: return QBVH6BuilderSAH::TRIANGLE; break;
      case Geometry::GTY_QUAD_MESH    : return QBVH6BuilderSAH::QUAD; break;
      case Geometry::GTY_GRID_MESH    : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_SUBDIV_MESH  : assert(false); return QBVH6BuilderSAH::UNKNOWN; break;
      
      case Geometry::GTY_SPHERE_POINT       : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_DISC_POINT         : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_ORIENTED_DISC_POINT: return QBVH6BuilderSAH::PROCEDURAL; break;
      
      case Geometry::GTY_USER_GEOMETRY     : return QBVH6BuilderSAH::PROCEDURAL; break;

#if RTC_MAX_INSTANCE_LEVEL_COUNT < 2
      case Geometry::GTY_INSTANCE_CHEAP    :
      case Geometry::GTY_INSTANCE_EXPENSIVE: {
        Instance* instance = scene->get<Instance>(geomID);
        QBVH6* object = (QBVH6*)((Scene*)instance->object)->hwaccel.data();
        if (object->numTimeSegments > 1) return QBVH6BuilderSAH::PROCEDURAL; // we need to handle instances in procedural mode if instanced scene has motion blur
        if (instance->mask & 0xFFFFFF80) return QBVH6BuilderSAH::PROCEDURAL; // we need to handle instances in procedural mode if high mask bits are set
        else                             return QBVH6BuilderSAH::INSTANCE;
      }
#else
      case Geometry::GTY_INSTANCE_CHEAP    : return QBVH6BuilderSAH::PROCEDURAL; break;
      case Geometry::GTY_INSTANCE_EXPENSIVE: return QBVH6BuilderSAH::PROCEDURAL; break;
#endif

      default: assert(false); return QBVH6BuilderSAH::UNKNOWN;
      }
    };

    auto getNumTimeSegments = [&] (unsigned int geomID) {
      Geometry* geom = scene->geometries[geomID].ptr;
      if (geom == nullptr) return 0u;
      return geom->numTimeSegments();
    };
    
    auto createPrimRefArray = [&] (mvector<PrimRef>& prims, BBox1f time_range, const range<size_t>& r, size_t k, unsigned int geomID)
    {
      const Geometry* geom = scene->get(geomID);
      PrimInfo primInfo = geom->numTimeSegments() > 0
        ? geom->createPrimRefArrayMB(prims,time_range,r,k,geomID)
        : geom->createPrimRefArray  (prims,r,k,geomID);
      return primInfo;
    };

    auto getTriangle = [&](unsigned int geomID, unsigned int primID)
    {
      /* invoke any hit callback when Embree filter functions are present */
      GeometryFlags gflags = GeometryFlags::OPAQUE;
      if (scene->hasContextFilterFunction() || scene->get(geomID)->hasFilterFunctions())
        gflags = GeometryFlags::NONE;

      /* invoke any hit callback when high mask bits are enabled */
      if (scene->get(geomID)->mask & 0xFFFFFF80)
        gflags = GeometryFlags::NONE;
      
      TriangleMesh* mesh = scene->get<TriangleMesh>(geomID);
      const TriangleMesh::Triangle tri = mesh->triangle(primID);
      if (unlikely(tri.v[0] >= mesh->numVertices())) return QBVH6BuilderSAH::Triangle();
      if (unlikely(tri.v[1] >= mesh->numVertices())) return QBVH6BuilderSAH::Triangle();
      if (unlikely(tri.v[2] >= mesh->numVertices())) return QBVH6BuilderSAH::Triangle();

      const Vec3f p0 = mesh->vertices[0][tri.v[0]];
      const Vec3f p1 = mesh->vertices[0][tri.v[1]];
      const Vec3f p2 = mesh->vertices[0][tri.v[2]];
      if (unlikely(!isvalid(p0))) return QBVH6BuilderSAH::Triangle();
      if (unlikely(!isvalid(p1))) return QBVH6BuilderSAH::Triangle();
      if (unlikely(!isvalid(p2))) return QBVH6BuilderSAH::Triangle();

      return QBVH6BuilderSAH::Triangle(tri.v[0],tri.v[1],tri.v[2],p0,p1,p2,gflags,mask32_to_mask8(mesh->mask));
    };

    auto getTriangleIndices = [&] (uint32_t geomID, uint32_t primID) {
       const TriangleMesh::Triangle tri = scene->get<TriangleMesh>(geomID)->triangles[primID];
       return Vec3<uint32_t>(tri.v[0],tri.v[1],tri.v[2]);
    };

    auto getQuad = [&](unsigned int geomID, unsigned int primID)
    {
      /* invoke any hit callback when Embree filter functions are present */
      GeometryFlags gflags = GeometryFlags::OPAQUE;
      if (scene->hasContextFilterFunction() || scene->get(geomID)->hasFilterFunctions())
        gflags = GeometryFlags::NONE;

      /* invoke any hit callback when high mask bits are enabled */
      if (scene->get(geomID)->mask & 0xFFFFFF80)
        gflags = GeometryFlags::NONE;
          
      QuadMesh* mesh = scene->get<QuadMesh>(geomID);
      const QuadMesh::Quad quad = mesh->quad(primID);
      const Vec3f p0 = mesh->vertices[0][quad.v[0]];
      const Vec3f p1 = mesh->vertices[0][quad.v[1]];
      const Vec3f p2 = mesh->vertices[0][quad.v[2]];
      const Vec3f p3 = mesh->vertices[0][quad.v[3]];
      return QBVH6BuilderSAH::Quad(p0,p1,p2,p3,gflags,mask32_to_mask8(mesh->mask));
    };

    auto getProcedural = [&](unsigned int geomID, unsigned int primID) {
      return QBVH6BuilderSAH::Procedural(mask32_to_mask8(scene->get(geomID)->mask));
    };

    auto getInstance = [&](unsigned int geomID, unsigned int primID)
    {
      Instance* instance = scene->get<Instance>(geomID);
      void* accel = dynamic_cast<Scene*>(instance->object)->hwaccel.data();
      const AffineSpace3fa local2world = instance->getLocal2World();
      return QBVH6BuilderSAH::Instance(local2world,accel,mask32_to_mask8(instance->mask));
    };

    void* dispatchGlobalsPtr = dynamic_cast<DeviceGPU*>(scene->device)->dispatchGlobalsPtr;
    return QBVH6BuilderSAH::build(scene->size(), scene->device,
                                  getSize, getType, getNumTimeSegments,
                                  createPrimRefArray, getTriangle, getTriangleIndices, getQuad, getProcedural, getInstance,
                                  accel, scene->device->verbosity(1), dispatchGlobalsPtr);
  }

  static void align(size_t& ofs, size_t alignment) {
    ofs = (ofs+(alignment-1))&(-alignment);
  }

  size_t sizeof_RTHWIF_GEOMETRY(RTHWIF_GEOMETRY_TYPE type)
  {
    switch (type) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return sizeof(RTHWIF_GEOMETRY_TRIANGLES_DESC);
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return sizeof(RTHWIF_GEOMETRY_QUADS_DESC);
    case RTHWIF_GEOMETRY_TYPE_PROCEDURALS: return sizeof(RTHWIF_GEOMETRY_AABBS_DESC);
    case RTHWIF_GEOMETRY_TYPE_INSTANCES  : return sizeof(RTHWIF_GEOMETRY_INSTANCE_DESC);
    case RTHWIF_GEOMETRY_TYPE_INSTANCEREF  : return sizeof(RTHWIF_GEOMETRY_INSTANCEREF_DESC);
    default: assert(false); return 0;
    }
  }

  size_t alignof_RTHWIF_GEOMETRY(RTHWIF_GEOMETRY_TYPE type)
  {
    switch (type) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return alignof(RTHWIF_GEOMETRY_TRIANGLES_DESC);
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return alignof(RTHWIF_GEOMETRY_QUADS_DESC);
    case RTHWIF_GEOMETRY_TYPE_PROCEDURALS: return alignof(RTHWIF_GEOMETRY_AABBS_DESC);
    case RTHWIF_GEOMETRY_TYPE_INSTANCES  : return alignof(RTHWIF_GEOMETRY_INSTANCE_DESC);
    case RTHWIF_GEOMETRY_TYPE_INSTANCEREF  : return alignof(RTHWIF_GEOMETRY_INSTANCEREF_DESC);
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
    out->GeometryType = RTHWIF_GEOMETRY_TYPE_TRIANGLES;
    out->GeometryFlags = getGeometryFlags(scene,geom);
    out->GeometryMask = mask32_to_mask8(geom->mask);
    out->IndexBuffer = (RTHWIF_UINT3*) geom->triangles.getPtr();
    out->TriangleCount = geom->triangles.size();
    out->TriangleStride = geom->triangles.getStride();
    out->VertexBuffer = (RTHWIF_FLOAT3*) geom->vertices0.getPtr();
    out->VertexCount = geom->vertices0.size();
    out->VertexStride = geom->vertices0.getStride();
  }

  void createGeometryDesc(RTHWIF_GEOMETRY_QUADS_DESC* out, Scene* scene, QuadMesh* geom)
  {
    memset(out,0,sizeof(RTHWIF_GEOMETRY_QUADS_DESC));
    out->GeometryType = RTHWIF_GEOMETRY_TYPE_QUADS;
    out->GeometryFlags = getGeometryFlags(scene,geom);
    out->GeometryMask = mask32_to_mask8(geom->mask);
    out->IndexBuffer = (RTHWIF_UINT4*) geom->quads.getPtr();
    out->QuadCount = geom->quads.size();
    out->QuadStride = geom->quads.getStride();
    out->VertexBuffer = (RTHWIF_FLOAT3*) geom->vertices0.getPtr();
    out->VertexCount = geom->vertices0.size();
    out->VertexStride = geom->vertices0.getStride();
  }

  RTHWIF_AABB getProceduralAABB(const uint32_t primID, void* userPtr)
  {
    Geometry* geom = (Geometry*) userPtr;
    mvector<PrimRef> prim(geom->device,1); // FIXME: not efficient!
    geom->createPrimRefArray(prim,range<size_t>(primID),0,0);
    BBox3fa bounds = prim[0].bounds();
    
    RTHWIF_AABB obounds;
    obounds.lower.x = bounds.lower.x;
    obounds.lower.y = bounds.lower.y;
    obounds.lower.z = bounds.lower.z;
    obounds.upper.x = bounds.upper.x;
    obounds.upper.y = bounds.upper.y;
    obounds.upper.z = bounds.upper.z;
    return obounds;
  };

  void createGeometryDescProcedural(RTHWIF_GEOMETRY_AABBS_DESC* out, Scene* scene, Geometry* geom)
  {
    uint32_t numPrimitives = geom->size();
    if (GridMesh* mesh = dynamic_cast<GridMesh*>(geom))
      numPrimitives = mesh->getNumTotalQuads(); // FIXME: slow
    
    memset(out,0,sizeof(RTHWIF_GEOMETRY_AABBS_DESC));
    out->GeometryType = RTHWIF_GEOMETRY_TYPE_PROCEDURALS;
    out->GeometryFlags = RTHWIF_GEOMETRY_FLAG_NONE;
    out->GeometryMask = mask32_to_mask8(geom->mask);
    out->AABBCount = numPrimitives;
    out->AABBs = getProceduralAABB;
    out->userPtr = geom;
  }

  void createGeometryDesc(RTHWIF_GEOMETRY_INSTANCE_DESC* out, Scene* scene, Instance* geom)
  {
    memset(out,0,sizeof(RTHWIF_GEOMETRY_INSTANCE_DESC));
    out->GeometryType = RTHWIF_GEOMETRY_TYPE_INSTANCES;
    out->InstanceFlags = RTHWIF_INSTANCE_FLAG_NONE;
    out->GeometryMask = mask32_to_mask8(geom->mask);
    out->InstanceID = 0;
    const AffineSpace3fa local2world = geom->getLocal2World();
    out->Transform = *(RTHWIF_TRANSFORM4X4*) &local2world;
    out->Accel = dynamic_cast<Scene*>(geom->object)->hwaccel.data();
  }

  void createGeometryDesc(RTHWIF_GEOMETRY_INSTANCEREF_DESC* out, Scene* scene, Instance* geom)
  {
    assert(geom->gsubtype == AccelSet::GTY_SUBTYPE_DEFAULT);
    memset(out,0,sizeof(RTHWIF_GEOMETRY_INSTANCEREF_DESC));
    out->GeometryType = RTHWIF_GEOMETRY_TYPE_INSTANCEREF;
    out->InstanceFlags = RTHWIF_INSTANCE_FLAG_NONE;
    out->GeometryMask = mask32_to_mask8(geom->mask);
    out->InstanceID = 0;
    out->Transform = (RTHWIF_TRANSFORM4X4*) &geom->local2world[0];
    out->Accel = dynamic_cast<Scene*>(geom->object)->hwaccel.data();
  }

  void createGeometryDesc(char* out, Scene* scene, Geometry* geom, RTHWIF_GEOMETRY_TYPE type)
  {
    switch (type) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return createGeometryDesc((RTHWIF_GEOMETRY_TRIANGLES_DESC*)out,scene,dynamic_cast<TriangleMesh*>(geom));
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return createGeometryDesc((RTHWIF_GEOMETRY_QUADS_DESC*)out,scene,dynamic_cast<QuadMesh*>(geom));
    case RTHWIF_GEOMETRY_TYPE_PROCEDURALS: return createGeometryDescProcedural((RTHWIF_GEOMETRY_AABBS_DESC*)out,scene,geom);
    case RTHWIF_GEOMETRY_TYPE_INSTANCES  : return createGeometryDesc((RTHWIF_GEOMETRY_INSTANCE_DESC*)out,scene,dynamic_cast<Instance*>(geom));
    case RTHWIF_GEOMETRY_TYPE_INSTANCEREF: return createGeometryDesc((RTHWIF_GEOMETRY_INSTANCEREF_DESC*)out,scene,dynamic_cast<Instance*>(geom));
    default: assert(false);
    }
  }

  BBox3fa rthwifBuildDriver(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& accel)
  {
    std::vector<RTHWIF_GEOMETRY_DESC*> geom;
    geom.resize(scene->size());

    auto getType = [&](unsigned int geomID)
    {
      /* no HW support for MB yet */
      if (scene->get(geomID)->numTimeSegments() > 0)
        return RTHWIF_GEOMETRY_TYPE_PROCEDURALS;

      switch (scene->get(geomID)->getType()) {
      case Geometry::GTY_FLAT_LINEAR_CURVE    : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ROUND_LINEAR_CURVE   : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ORIENTED_LINEAR_CURVE: return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_CONE_LINEAR_CURVE    : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      
      case Geometry::GTY_FLAT_BEZIER_CURVE    : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ROUND_BEZIER_CURVE   : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ORIENTED_BEZIER_CURVE: return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      
      case Geometry::GTY_FLAT_BSPLINE_CURVE    : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ROUND_BSPLINE_CURVE   : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ORIENTED_BSPLINE_CURVE: return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      
      case Geometry::GTY_FLAT_HERMITE_CURVE    : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ROUND_HERMITE_CURVE   : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ORIENTED_HERMITE_CURVE: return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      
      case Geometry::GTY_FLAT_CATMULL_ROM_CURVE    : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ROUND_CATMULL_ROM_CURVE   : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE: return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      
      case Geometry::GTY_TRIANGLE_MESH: return RTHWIF_GEOMETRY_TYPE_TRIANGLES; break;
      case Geometry::GTY_QUAD_MESH    : return RTHWIF_GEOMETRY_TYPE_QUADS; break;
      case Geometry::GTY_GRID_MESH    : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_SUBDIV_MESH  : assert(false); return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      
      case Geometry::GTY_SPHERE_POINT       : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_DISC_POINT         : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      case Geometry::GTY_ORIENTED_DISC_POINT: return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;
      
      case Geometry::GTY_USER_GEOMETRY     : return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; break;

#if RTC_MAX_INSTANCE_LEVEL_COUNT < 2
      case Geometry::GTY_INSTANCE_CHEAP    :
      case Geometry::GTY_INSTANCE_EXPENSIVE: {
        Instance* instance = scene->get<Instance>(geomID);
        QBVH6* object = (QBVH6*)((Scene*)instance->object)->hwaccel.data();
        if (object->numTimeSegments > 1) return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; // we need to handle instances in procedural mode if instanced scene has motion blur
        if (instance->mask & 0xFFFFFF80) return RTHWIF_GEOMETRY_TYPE_PROCEDURALS; // we need to handle instances in procedural mode if high mask bits are set
        else if (instance->gsubtype == AccelSet::GTY_SUBTYPE_INSTANCE_QUATERNION) return RTHWIF_GEOMETRY_TYPE_INSTANCES;
        else return RTHWIF_GEOMETRY_TYPE_INSTANCEREF;
      }
#else
      case Geometry::GTY_INSTANCE_CHEAP    : return RTHWIF_GEOMETRY_TYPE_PROCEDURAL; break;
      case Geometry::GTY_INSTANCE_EXPENSIVE: return RTHWIF_GEOMETRY_TYPE_PROCEDURAL; break;
#endif

      default: assert(false); return RTHWIF_GEOMETRY_TYPE_PROCEDURALS;
      }
    };

    /* calculate size of geometry descriptor buffer */
    size_t bytes = 0;
    for (size_t geomID=0; geomID<geom.size(); geomID++) {
      const RTHWIF_GEOMETRY_TYPE type = getType(geomID);
      align(bytes,alignof_RTHWIF_GEOMETRY(type));
      bytes += sizeof_RTHWIF_GEOMETRY(type);
    }

    /* fill geomdesc buffer */
    std::vector<char> geomdesc(bytes);
    
    size_t offset = 0;
    for (size_t geomID=0; geomID<geom.size(); geomID++)
    {
      const RTHWIF_GEOMETRY_TYPE type = getType(geomID);
      align(offset,alignof_RTHWIF_GEOMETRY(type));
      createGeometryDesc(&geomdesc[offset],scene,scene->get(geomID),type);
      geom[geomID] = (RTHWIF_GEOMETRY_DESC*) &geomdesc[offset];
      offset += sizeof_RTHWIF_GEOMETRY(type);
      assert(offset <= geomdesc.size());
    }

    /* estimate accel size */
    RTHWIF_AABB bounds;
    RTHWIF_BUILD_ACCEL_ARGS args;
    memset(&args,0,sizeof(args));
    args.bytes = sizeof(args);
    args.device = nullptr;
    args.embree_device = scene->device;
    args.geometries = (const RTHWIF_GEOMETRY_DESC**) geom.data();
    args.numGeometries = geom.size();
    args.accel = nullptr;
    args.numBytes = 0;
    args.quality = RTHWIF_BUILD_QUALITY_MEDIUM;
    args.flags = RTHWIF_BUILD_FLAG_NONE;
    args.bounds = &bounds;
    RTHWIF_ACCEL_SIZE size = rthwifGetAccelSize(args);

    /* build BVH */
    RTHWIF_ERROR err = RTHWIF_ERROR_NONE;
    for (size_t bytes = size.expectedBytes; bytes < size.worstCaseBytes; bytes*=1.2)
    {
      if (accel.size() < bytes) accel = std::move(Device::avector<char,64>(scene->device,bytes));
      memset(accel.data(),0,accel.size()); // FIXME: not required
      args.accel=accel.data();
      args.numBytes = accel.size();

      RTHWIF_ERROR err = rthwifBuildAccel(args);
      if (err != RTHWIF_ERROR_OUT_OF_MEMORY) break;
    }

    if (err != RTHWIF_ERROR_NONE)
      throw std::runtime_error("build error");

    return *(BBox3f*) args.bounds;
  }

  BBox3fa rthwifBuild(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& accel)
  {
    return rthwifBuildDirect(scene,quality_flags,accel);
    //return rthwifBuildDriver(scene,quality_flags,accel);
  }
}
