// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "rthwif_embree.h"
#include "rthwif_embree_builder.h"
#include "builder/qbvh6_builder_sah.h"
#include "../builders/primrefgen.h"
#include "rthwif_internal.h"

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

  BBox3fa rthwifBuild(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& accel)
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
}
