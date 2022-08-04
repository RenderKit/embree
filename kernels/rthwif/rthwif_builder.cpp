// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#define RTHWIF_EXPORT_API

#include "rthwif_builder.h"
#include "builder/qbvh6_builder_sah.h"
#include "rthwif_internal.h"

#include <level_zero/ze_api.h>

namespace embree
{
  using namespace embree::isa;
  
  inline RTHWIF_UINT3 getPrimitive(const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom, uint32_t primID) {
    assert(primID < geom->TriangleCount);
    return *(RTHWIF_UINT3*)((char*)geom->IndexBuffer + primID*geom->TriangleStride);
  }
  
  inline Vec3f getVertex(const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom, uint32_t vertexID) {
    assert(vertexID < geom->VertexCount);
    return *(Vec3f*)((char*)geom->VertexBuffer + vertexID*geom->VertexStride);
  }
  
  inline RTHWIF_UINT4 getPrimitive(const RTHWIF_GEOMETRY_QUADS_DESC* geom, uint32_t primID) {
    assert(primID < geom->QuadCount);
    return *(RTHWIF_UINT4*)((char*)geom->IndexBuffer + primID*geom->QuadStride);
  }
  
  inline Vec3f getVertex(const RTHWIF_GEOMETRY_QUADS_DESC* geom, uint32_t vertexID) {
    assert(vertexID < geom->VertexCount);
    return *(Vec3f*)((char*)geom->VertexBuffer + vertexID*geom->VertexStride);
  }

  inline bool buildBounds(const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom, uint32_t primID, BBox3fa& bbox, void* userPtr)
  {
    if (primID >= geom->TriangleCount) return false;
    const RTHWIF_UINT3 tri = getPrimitive(geom,primID);
    if (unlikely(tri.v0 >= geom->VertexCount)) return false;
    if (unlikely(tri.v1 >= geom->VertexCount)) return false;
    if (unlikely(tri.v2 >= geom->VertexCount)) return false;
    
    const Vec3f p0 = getVertex(geom,tri.v0);
    const Vec3f p1 = getVertex(geom,tri.v1);
    const Vec3f p2 = getVertex(geom,tri.v2);
    if (unlikely(!isvalid(p0))) return false;
    if (unlikely(!isvalid(p1))) return false;
    if (unlikely(!isvalid(p2))) return false;
    
    bbox = BBox3fa(min(p0,p1,p2),max(p0,p1,p2));
    return true;
  }

  inline bool buildBounds(const RTHWIF_GEOMETRY_QUADS_DESC* geom, uint32_t primID, BBox3fa& bbox, void* userPtr)
  {
    if (primID >= geom->QuadCount) return false;
    const RTHWIF_UINT4 tri = getPrimitive(geom,primID);
    if (unlikely(tri.v0 >= geom->VertexCount)) return false;
    if (unlikely(tri.v1 >= geom->VertexCount)) return false;
    if (unlikely(tri.v2 >= geom->VertexCount)) return false;
    if (unlikely(tri.v3 >= geom->VertexCount)) return false;
    
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

  inline bool buildBounds(const RTHWIF_GEOMETRY_AABBS_DESC* geom, uint32_t primID, BBox3fa& bbox, void* userPtr)
  {
    if (primID >= geom->AABBCount) return false;
    if (geom->AABBs == nullptr) return false;
    
    const RTHWIF_AABB rthwif_bounds = (geom->AABBs)(primID,geom->userPtr,userPtr);
    const BBox3f bounds = (BBox3f&) rthwif_bounds;
    if (unlikely(!isvalid(bounds.lower))) return false;
    if (unlikely(!isvalid(bounds.upper))) return false;
    if (unlikely(bounds.empty())) return false;
    
    bbox = (BBox3f&) bounds;
    return true;
  }

  inline bool buildBounds(const RTHWIF_GEOMETRY_INSTANCE_DESC* geom, uint32_t primID, BBox3fa& bbox, void* userPtr)
  {
    if (primID >= 1) return false;
    if (geom->Accel == nullptr) return false;

    const Vec3fa vx = *(Vec3f*) &geom->Transform.vx;
    const Vec3fa vy = *(Vec3f*) &geom->Transform.vy;
    const Vec3fa vz = *(Vec3f*) &geom->Transform.vz;
    const Vec3fa p  = *(Vec3f*) &geom->Transform.p;
    const AffineSpace3fa local2world(vx,vy,vz,p);

    HWAccel* accel = (HWAccel*) geom->Accel;
    const Vec3fa lower(accel->bounds[0][0],accel->bounds[0][1],accel->bounds[0][2]);
    const Vec3fa upper(accel->bounds[1][0],accel->bounds[1][1],accel->bounds[1][2]);
    const BBox3fa bounds = xfmBounds(local2world,BBox3fa(lower,upper));
     
    if (unlikely(!isvalid(bounds.lower))) return false;
    if (unlikely(!isvalid(bounds.upper))) return false;
    if (unlikely(bounds.empty())) return false;
    
    bbox = bounds;
    return true;
  }

  inline bool buildBounds(const RTHWIF_GEOMETRY_INSTANCEREF_DESC* geom, uint32_t primID, BBox3fa& bbox, void* userPtr)
  {
    if (primID >= 1) return false;
    if (geom->Accel == nullptr) return false;
    if (geom->Transform == nullptr) return false;
    
    const Vec3fa vx = *(Vec3f*) &geom->Transform->vx;
    const Vec3fa vy = *(Vec3f*) &geom->Transform->vy;
    const Vec3fa vz = *(Vec3f*) &geom->Transform->vz;
    const Vec3fa p  = *(Vec3f*) &geom->Transform->p;
    const AffineSpace3fa local2world(vx,vy,vz,p);

    HWAccel* accel = (HWAccel*) geom->Accel;
    const Vec3fa lower(accel->bounds[0][0],accel->bounds[0][1],accel->bounds[0][2]);
    const Vec3fa upper(accel->bounds[1][0],accel->bounds[1][1],accel->bounds[1][2]);
    const BBox3fa bounds = xfmBounds(local2world,BBox3fa(lower,upper));
     
    if (unlikely(!isvalid(bounds.lower))) return false;
    if (unlikely(!isvalid(bounds.upper))) return false;
    if (unlikely(bounds.empty())) return false;
    
    bbox = bounds;
    return true;
  }

  template<typename GeometryType>
  PrimInfo createGeometryPrimRefArray(const GeometryType* geom, void* userPtr, mvector<PrimRef>& prims, const range<size_t>& r, size_t k, unsigned int geomID)
  {
    PrimInfo pinfo(empty);
    for (uint32_t primID=r.begin(); primID<r.end(); primID++)
    {
      BBox3fa bounds = empty;
      if (!buildBounds(geom,primID,bounds,userPtr)) continue;
      const PrimRef prim(bounds,geomID,primID);
      pinfo.add_center2(prim);
      prims[k++] = prim;
    }
    return pinfo;
  }
  
  RTHWIF_API void rthwifInit()
  {
  }
  
  RTHWIF_API void rthwifExit()
  {
  }
  
  RTHWIF_API RTHWIF_FEATURES rthwifGetSupportedFeatures(sycl::device device)
  {
    /* check for Intel vendor */
    const uint32_t vendor_id = device.get_info<sycl::info::device::vendor_id>();
    if (vendor_id != 0x8086) return RTHWIF_FEATURES_NONE;

    /* check for supported device ID */
    auto native_device = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(device);
    ze_device_properties_t device_props{ ZE_STRUCTURE_TYPE_DEVICE_PROPERTIES };
    ze_result_t status = zeDeviceGetProperties(native_device, &device_props);
    if (status != ZE_RESULT_SUCCESS)
      return RTHWIF_FEATURES_NONE;

    const uint32_t device_id = device_props.deviceId;

    /* return supported features */
    uint32_t features_xe = RTHWIF_FEATURES_NONE;
    features_xe |= RTHWIF_FEATURES_TRIANGLES;
    features_xe |= RTHWIF_GEOMETRY_PROCEDURALS;
    features_xe |= RTHWIF_GEOMETRY_QUADS;
    features_xe |= RTHWIF_GEOMETRY_INSTANCE;
    
    // DG2
    if (0x4F80 <= device_id && device_id <= 0x4F88) return (RTHWIF_FEATURES) features_xe;
    if (0x5690 <= device_id && device_id <= 0x5698) return (RTHWIF_FEATURES) features_xe;
    if (0x56A0 <= device_id && device_id <= 0x56A6) return (RTHWIF_FEATURES) features_xe;
    if (0x56B0 <= device_id && device_id <= 0x56B3) return (RTHWIF_FEATURES) features_xe;
    if (0x56C0 <= device_id && device_id <= 0x56C1) return (RTHWIF_FEATURES) features_xe;
       
    // ATS-M
    if (0x0201 <= device_id && device_id <= 0x0210) return (RTHWIF_FEATURES) features_xe;

    // PVC
    if (0x0BD0 <= device_id && device_id <= 0x0BDB) return (RTHWIF_FEATURES) features_xe;
    if (device_id == 0x0BE5                       ) return (RTHWIF_FEATURES) features_xe;

    return RTHWIF_FEATURES_NONE;
  }
  
  uint32_t getNumPrimitives(const RTHWIF_GEOMETRY_DESC* geom)
  {
    switch (geom->GeometryType) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return ((RTHWIF_GEOMETRY_TRIANGLES_DESC*) geom)->TriangleCount;
    case RTHWIF_GEOMETRY_TYPE_PROCEDURALS: return ((RTHWIF_GEOMETRY_AABBS_DESC*) geom)->AABBCount;
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return ((RTHWIF_GEOMETRY_QUADS_DESC*) geom)->QuadCount;
    case RTHWIF_GEOMETRY_TYPE_INSTANCES  : return 1;
    case RTHWIF_GEOMETRY_TYPE_INSTANCEREF: return 1;
    default                            : return 0;
    };
  }
  
  /* fill all arg members that app did not know of yet */
  RTHWIF_BUILD_ACCEL_ARGS rthwifPrepareBuildAccelArgs(const RTHWIF_BUILD_ACCEL_ARGS& args_i)
  {
    RTHWIF_BUILD_ACCEL_ARGS args;
    memset(&args,0,sizeof(RTHWIF_BUILD_ACCEL_ARGS));
    memcpy(&args,&args_i,std::min(sizeof(RTHWIF_BUILD_ACCEL_ARGS),args_i.bytes));
    args.bytes = sizeof(RTHWIF_BUILD_ACCEL_ARGS);
    return args;
  }
  
  RTHWIF_API RTHWIF_ERROR rthwifGetAccelSize(const RTHWIF_BUILD_ACCEL_ARGS& args_i, RTHWIF_ACCEL_SIZE& size_o)
  {
    RTHWIF_BUILD_ACCEL_ARGS args = rthwifPrepareBuildAccelArgs(args_i);
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const size_t numGeometries = args.numGeometries;
    
    struct Stats
    {
      size_t numTriangles = 0;
      size_t numQuads = 0;
      size_t numProcedurals = 0;
      size_t numInstances = 0;
      
      /* assume some reasonable quadification rate */
      void estimate_quadification()
      {
        numQuads += (numTriangles+1)/2 + numTriangles/8;
        numTriangles = 0;
      }
      
      void estimate_presplits( double factor )
      {
        numTriangles = max(numTriangles, size_t(numTriangles*factor));
        numQuads     = max(numQuads    , size_t(numQuads*factor));
        numInstances = max(numInstances, size_t(numInstances*factor));
      }
      
      size_t size() {
        return numTriangles+numQuads+numProcedurals+numInstances;
      }
      
      size_t expected_bvh_bytes()
      {
        const size_t blocks = (size()+5)/6;
        const size_t expected_bytes   = 128 + 64*size_t(1+1.5*blocks) + numTriangles*64 + numQuads*64 + numProcedurals*8 + numInstances*128;
        const size_t bytes = 2*4096 + size_t(1.1*expected_bytes); // FIXME: FastAllocator wastes memory and always allocates 4kB per thread
        return (bytes+127)&-128;
      }
      
      size_t worst_case_bvh_bytes()
      {
        const size_t numPrimitives = size();
        const size_t blocks = (numPrimitives+5)/6;
        const size_t worst_case_bytes = 128 + 64*(1+blocks + numPrimitives) + numTriangles*64 + numQuads*64 + numProcedurals*64 + numInstances*128;
        const size_t bytes = 2*4096 + size_t(1.1*worst_case_bytes); // FIXME: FastAllocator wastes memory and always allocates 4kB per thread
        return (bytes+127)&-128;
      }
      
    } stats;
    
    for (size_t geomID=0; geomID<numGeometries; geomID++)
    {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      if (geom == nullptr) continue;
      
      switch (geom->GeometryType) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : stats.numTriangles += ((RTHWIF_GEOMETRY_TRIANGLES_DESC*) geom)->TriangleCount;
      case RTHWIF_GEOMETRY_TYPE_QUADS      : stats.numQuads += ((RTHWIF_GEOMETRY_QUADS_DESC*) geom)->QuadCount;
      case RTHWIF_GEOMETRY_TYPE_PROCEDURALS: stats.numProcedurals += ((RTHWIF_GEOMETRY_AABBS_DESC*) geom)->AABBCount;
      case RTHWIF_GEOMETRY_TYPE_INSTANCES  : stats.numInstances += 1;
      case RTHWIF_GEOMETRY_TYPE_INSTANCEREF: stats.numInstances += 1;
      };
    }
    
    stats.estimate_quadification();
    stats.estimate_presplits(1.2);

    /* return size to user */
    RTHWIF_ACCEL_SIZE size;
    memset(&size,0,sizeof(RTHWIF_ACCEL_SIZE));
    size.expectedBytes = stats.expected_bvh_bytes();
    size.worstCaseBytes = stats.worst_case_bvh_bytes();
    size_t bytes_o = size_o.bytes;
    memset(&size_o,0,bytes_o);
    memcpy(&size_o,&size,bytes_o);
    size_o.bytes = bytes_o;
    return RTHWIF_ERROR_NONE;
  }
  
  RTHWIF_API RTHWIF_ERROR rthwifBuildAccel(const RTHWIF_BUILD_ACCEL_ARGS& args_i) try
  {
    /* prepare input arguments */
    const RTHWIF_BUILD_ACCEL_ARGS args = rthwifPrepareBuildAccelArgs(args_i);
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const size_t numGeometries = args.numGeometries;

    if (args.accel == nullptr) return RTHWIF_ERROR_OTHER;
    
    auto getSize = [&](uint32_t geomID) -> size_t {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      if (geom == nullptr) return 0;
      return getNumPrimitives(geom);
    };
    
    auto getType = [&](unsigned int geomID)
    {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      switch (geom->GeometryType) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES : return QBVH6BuilderSAH::TRIANGLE;
      case RTHWIF_GEOMETRY_TYPE_QUADS: return QBVH6BuilderSAH::QUAD;
      case RTHWIF_GEOMETRY_TYPE_PROCEDURALS: return QBVH6BuilderSAH::PROCEDURAL;
      case RTHWIF_GEOMETRY_TYPE_INSTANCES: return QBVH6BuilderSAH::INSTANCE;
      case RTHWIF_GEOMETRY_TYPE_INSTANCEREF: return QBVH6BuilderSAH::INSTANCE;
      default: throw std::runtime_error("invalid geometry type");
      };
    };
    
    auto getNumTimeSegments = [&] (unsigned int geomID) {
      return 0;
    };
    
    auto createPrimRefArray = [&] (mvector<PrimRef>& prims, BBox1f time_range, const range<size_t>& r, size_t k, unsigned int geomID) -> PrimInfo
    {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      if (geom == nullptr) return PrimInfo(empty);
      
      switch (geom->GeometryType) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return createGeometryPrimRefArray((RTHWIF_GEOMETRY_TRIANGLES_DESC*)geom,args.userPtr,prims,r,k,geomID);
      case RTHWIF_GEOMETRY_TYPE_QUADS      : return createGeometryPrimRefArray((RTHWIF_GEOMETRY_QUADS_DESC*    )geom,args.userPtr,prims,r,k,geomID);
      case RTHWIF_GEOMETRY_TYPE_PROCEDURALS: return createGeometryPrimRefArray((RTHWIF_GEOMETRY_AABBS_DESC*    )geom,args.userPtr,prims,r,k,geomID);
      case RTHWIF_GEOMETRY_TYPE_INSTANCES  : return createGeometryPrimRefArray((RTHWIF_GEOMETRY_INSTANCE_DESC* )geom,args.userPtr,prims,r,k,geomID);
      case RTHWIF_GEOMETRY_TYPE_INSTANCEREF: return createGeometryPrimRefArray((RTHWIF_GEOMETRY_INSTANCEREF_DESC* )geom,args.userPtr,prims,r,k,geomID);
      default: throw std::runtime_error("invalid geometry type");
      };
    };
    
    auto getTriangle = [&](unsigned int geomID, unsigned int primID)
    {
      const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom = (const RTHWIF_GEOMETRY_TRIANGLES_DESC*) geometries[geomID];
      assert(geom);
      
      const RTHWIF_UINT3 tri = getPrimitive(geom,primID);
      if (unlikely(tri.v0 >= geom->VertexCount)) return QBVH6BuilderSAH::Triangle();
      if (unlikely(tri.v1 >= geom->VertexCount)) return QBVH6BuilderSAH::Triangle();
      if (unlikely(tri.v2 >= geom->VertexCount)) return QBVH6BuilderSAH::Triangle();
      
      const Vec3f p0 = getVertex(geom,tri.v0);
      const Vec3f p1 = getVertex(geom,tri.v1);
      const Vec3f p2 = getVertex(geom,tri.v2);
      if (unlikely(!isvalid(p0))) return QBVH6BuilderSAH::Triangle();
      if (unlikely(!isvalid(p1))) return QBVH6BuilderSAH::Triangle();
      if (unlikely(!isvalid(p2))) return QBVH6BuilderSAH::Triangle();

      const GeometryFlags gflags = (GeometryFlags) geom->GeometryFlags;
      return QBVH6BuilderSAH::Triangle(tri.v0,tri.v1,tri.v2,p0,p1,p2,gflags,geom->GeometryMask);
    };
    
    auto getTriangleIndices = [&] (uint32_t geomID, uint32_t primID) {
      const RTHWIF_GEOMETRY_TRIANGLES_DESC* geom = (const RTHWIF_GEOMETRY_TRIANGLES_DESC*) geometries[geomID];
      assert(geom);
      const RTHWIF_UINT3 tri = getPrimitive(geom,primID);
      return Vec3<uint32_t>(tri.v0,tri.v1,tri.v2);
    };
    
    auto getQuad = [&](unsigned int geomID, unsigned int primID)
    {
      const RTHWIF_GEOMETRY_QUADS_DESC* geom = (const RTHWIF_GEOMETRY_QUADS_DESC*) geometries[geomID];
      assert(geom);
                     
      const RTHWIF_UINT4 quad = getPrimitive(geom,primID);
      const Vec3f p0 = getVertex(geom,quad.v0);
      const Vec3f p1 = getVertex(geom,quad.v1);
      const Vec3f p2 = getVertex(geom,quad.v2);
      const Vec3f p3 = getVertex(geom,quad.v3);

      const GeometryFlags gflags = (GeometryFlags) geom->GeometryFlags;
      return QBVH6BuilderSAH::Quad(p0,p1,p2,p3,gflags,geom->GeometryMask);
    };
    
    auto getProcedural = [&](unsigned int geomID, unsigned int primID) {
      const RTHWIF_GEOMETRY_AABBS_DESC* geom = (const RTHWIF_GEOMETRY_AABBS_DESC*) geometries[geomID];
      assert(geom);
      return QBVH6BuilderSAH::Procedural(geom->GeometryMask); // FIXME: pass gflags
    };
    
    auto getInstance = [&](unsigned int geomID, unsigned int primID)
    {
      assert(geometries[geomID]);
      if (geometries[geomID]->GeometryType == RTHWIF_GEOMETRY_TYPE_INSTANCES) {
        const RTHWIF_GEOMETRY_INSTANCE_DESC* geom = (const RTHWIF_GEOMETRY_INSTANCE_DESC*) geometries[geomID];
        void* accel = geom->Accel;
        RTHWIF_TRANSFORM4X4 local2world = geom->Transform;
        return QBVH6BuilderSAH::Instance((AffineSpace3fa&)local2world,accel,geom->GeometryMask); // FIXME: pass instance flags
      } else {
        const RTHWIF_GEOMETRY_INSTANCEREF_DESC* geom = (const RTHWIF_GEOMETRY_INSTANCEREF_DESC*) geometries[geomID];
        void* accel = geom->Accel;
        RTHWIF_TRANSFORM4X4 local2world = *geom->Transform;
        return QBVH6BuilderSAH::Instance((AffineSpace3fa&)local2world,accel,geom->GeometryMask); // FIXME: pass instance flags
      }
    };

    void* dispatchGlobalsPtr = nullptr;
    bool verbose = false;
    BBox3f bounds = QBVH6BuilderSAH::build2(numGeometries, (Device*) args.embree_device, 
                                            getSize, getType, getNumTimeSegments,
                                            createPrimRefArray, getTriangle, getTriangleIndices, getQuad, getProcedural, getInstance,
                                            (char*)args.accel, args.numBytes, args.AddAccel, verbose, dispatchGlobalsPtr);

    if (args.bounds) *(BBox3f*)args.bounds = bounds;
    
    return RTHWIF_ERROR_NONE;
  }
  
  catch (std::bad_alloc&) {
    return RTHWIF_ERROR_OUT_OF_MEMORY;
  }
  
  catch (...) {
    return RTHWIF_ERROR_OTHER;
  }
}
