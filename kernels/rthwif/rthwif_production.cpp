// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/default.h"
#include "../common/device.h"
#include "../common/scene.h"
#include "../common/context.h"
#include "../../include/embree4/rtcore_ray.h"

#include "rthwif_production.h"
#include "rthwif_internal.h"
using namespace embree;

#define sizeof_QBVH6_InternalNode6 64
#define QBVH6_rootNodeOffset 128

 /*struct rayquery_impl_t {
    rtfence_t fence;
    rtglobals_t dispatchGlobalsPtr;
    struct RTStack* rtStack;
    TraceRayCtrl ctrl;
    uint32_t bvh_level;
 };*/

void use_rthwif_production()
{
}

SYCL_EXTERNAL rayquery_t intel_ray_query_init( unsigned int bvh_level, RayDescINTEL ray, rtas_t* _accel_i, uint32_t bvh_id )
{
  rtas_t* accel_i = sycl::global_ptr<rtas_t>(_accel_i).get();
  HWAccel* accel = (HWAccel*)accel_i;
#if defined(EMBREE_DPCPP_IMPLICIT_DISPATCH_GLOBALS)
  rtglobals_t dispatchGlobalsPtr = (rtglobals_t) intel_get_implicit_dispatch_globals();
#else
  rtglobals_t dispatchGlobalsPtr = (rtglobals_t) accel->dispatchGlobalsPtr;
#endif
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)intel_get_rt_stack( (rtglobals_t)dispatchGlobalsPtr )).get();
    
  /* init ray */
  rtStack->ray[bvh_level].org[0] = ray.O.x();
  rtStack->ray[bvh_level].org[1] = ray.O.y();
  rtStack->ray[bvh_level].org[2] = ray.O.z();
  rtStack->ray[bvh_level].dir[0] = ray.D.x();
  rtStack->ray[bvh_level].dir[1] = ray.D.y();
  rtStack->ray[bvh_level].dir[2] = ray.D.z();
  rtStack->ray[bvh_level].tnear  = ray.tmin;
  rtStack->ray[bvh_level].tfar   = ray.tmax;
  rtStack->ray[bvh_level].rootNodePtr = (uint64_t)accel + QBVH6_rootNodeOffset + sizeof_QBVH6_InternalNode6*bvh_id;
  rtStack->ray[bvh_level].rayFlags = ray.flags;
  rtStack->ray[bvh_level].hitGroupSRBasePtr = 0;
  rtStack->ray[bvh_level].hitGroupSRStride = 0;
  rtStack->ray[bvh_level].missSRPtr = 0;
  rtStack->ray[bvh_level].pad0 = 0;
  rtStack->ray[bvh_level].shaderIndexMultiplier = 0;
  rtStack->ray[bvh_level].instLeafPtr = 0;
  rtStack->ray[bvh_level].rayMask = ray.mask;
  rtStack->ray[bvh_level].pad1 = 0;
  
  rtStack->committedHit.t  = INFINITY;
  rtStack->committedHit.u  = 0.0f;
  rtStack->committedHit.v  = 0.0f;
  rtStack->committedHit.data = 0;

  rtStack->potentialHit.t  = INFINITY;
  rtStack->potentialHit.u  = 0.0f;
  rtStack->potentialHit.v  = 0.0f;
  rtStack->potentialHit.data = 0;
  rtStack->potentialHit.done = 1;
  rtStack->potentialHit.valid = 1;
  
  return { nullptr, (void*) dispatchGlobalsPtr, rtStack, TRACE_RAY_INITIAL, bvh_level };
}

SYCL_EXTERNAL void intel_ray_query_forward_ray( rayquery_t* query, unsigned int bvh_level, RayDescINTEL ray, rtas_t* accel_i, uint32_t bvh_id )
{
  HWAccel* accel = (HWAccel*)accel_i;
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query->opaque2).get();
  
  /* init ray */
  rtStack->ray[bvh_level].org[0] = ray.O.x();
  rtStack->ray[bvh_level].org[1] = ray.O.y();
  rtStack->ray[bvh_level].org[2] = ray.O.z();
  rtStack->ray[bvh_level].dir[0] = ray.D.x();
  rtStack->ray[bvh_level].dir[1] = ray.D.y();
  rtStack->ray[bvh_level].dir[2] = ray.D.z();
  rtStack->ray[bvh_level].tnear  = ray.tmin;
  rtStack->ray[bvh_level].tfar   = ray.tmax;
  rtStack->ray[bvh_level].rootNodePtr = (uint64_t)accel + QBVH6_rootNodeOffset + sizeof_QBVH6_InternalNode6*bvh_id;
  rtStack->ray[bvh_level].rayFlags = ray.flags;
  rtStack->ray[bvh_level].hitGroupSRBasePtr = 0;
  rtStack->ray[bvh_level].hitGroupSRStride = 0;
  rtStack->ray[bvh_level].missSRPtr = 0;
  rtStack->ray[bvh_level].pad0 = 0;
  rtStack->ray[bvh_level].shaderIndexMultiplier = 0;
  rtStack->ray[bvh_level].instLeafPtr = 0;
  rtStack->ray[bvh_level].rayMask = ray.mask;
  rtStack->ray[bvh_level].pad1 = 0;
  *query = { nullptr, query->opaque1, query->opaque2, TRACE_RAY_INSTANCE, bvh_level };
}

SYCL_EXTERNAL void intel_ray_query_commit_potential_hit( rayquery_t* query )
{
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query->opaque2).get();
  
  uint32_t bvh_level = query->bvh_level;
  uint32_t rflags = rtStack->ray[bvh_level].rayFlags;
  if (rflags & ACCEPT_FIRST_HIT_AND_END_SEARCH) {
    rtStack->committedHit = rtStack->potentialHit;
    rtStack->committedHit.valid = 1;
    *query = { nullptr, query->opaque1, query->opaque2, TRACE_RAY_DONE, bvh_level };
  } else {
    rtStack->potentialHit.valid = 1; // FIXME: is this required?
    *query = { nullptr, query->opaque1, query->opaque2, TRACE_RAY_COMMIT, bvh_level };
  }
}

SYCL_EXTERNAL void intel_ray_query_commit_potential_hit( rayquery_t* query, float override_hit_distance, float2 override_uv )
{
  //struct RTStack* rtStack = (struct RTStack*) query.opaque2;  
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query->opaque2).get();
  
  rtStack->potentialHit.t = override_hit_distance;
  rtStack->potentialHit.u = override_uv.x();
  rtStack->potentialHit.v = override_uv.y();
  intel_ray_query_commit_potential_hit(query);
}

SYCL_EXTERNAL void intel_ray_query_start_traversal( rayquery_t* query )
{
  rtglobals_t dispatchGlobalsPtr = (rtglobals_t) query->opaque1;
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query->opaque2).get();

  rtStack->potentialHit.done = 1;
  rtStack->potentialHit.valid = 1;
  
  if (query->ctrl == TRACE_RAY_DONE) return;
  rtfence_t fence = intel_dispatch_trace_ray_query(dispatchGlobalsPtr,query->bvh_level,query->ctrl);
  *query = { (void*) fence, query->opaque1, query->opaque2, TRACE_RAY_INITIAL, 0 }; 
}

SYCL_EXTERNAL void intel_ray_query_sync( rayquery_t* query )
{
  intel_rt_sync((rtfence_t)query->opaque0);
  
  /* continue is default behaviour */
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query->opaque2).get();
  
  uint32_t bvh_level = rtStack->potentialHit.bvhLevel;
  *query = { query->opaque0, query->opaque1, query->opaque2, TRACE_RAY_CONTINUE, bvh_level };
}

SYCL_EXTERNAL void intel_ray_query_abandon( rayquery_t* query )
{
  intel_ray_query_sync(query);
  *query = { nullptr, nullptr, nullptr, TRACE_RAY_INITIAL, 0 };
}

SYCL_EXTERNAL unsigned int intel_get_hit_bvh_level( rayquery_t query, HitType hit_type ) {
  return query.hit(hit_type).bvhLevel;
}

SYCL_EXTERNAL float intel_get_hit_distance( rayquery_t query, HitType hit_type ) {
  return query.hit(hit_type).t;
}

SYCL_EXTERNAL float2 intel_get_hit_barys( rayquery_t query, HitType hit_type ) {
  return float2(query.hit(hit_type).u, query.hit(hit_type).v);
}

SYCL_EXTERNAL bool intel_hit_is_front_face( rayquery_t query, HitType hit_type ) {
  return query.hit(hit_type).frontFace;
}

SYCL_EXTERNAL uint32_t intel_get_hit_geomID(rayquery_t query, HitType hit_type )
{
  struct PrimLeafDesc* __restrict leaf = (struct PrimLeafDesc*)query.hit(hit_type).getPrimLeafPtr();
  return leaf->geomIndex;
}

SYCL_EXTERNAL uint32_t intel_get_hit_primID( rayquery_t query, HitType hit_type )
{
  MemHit& hit = query.hit(hit_type);
  void* __restrict leaf = hit.getPrimLeafPtr();
  
  if (hit.leafType == NODE_TYPE_QUAD)
    return ((QuadLeaf*)leaf)->primIndex0 + hit.primIndexDelta;
  else
     return ((ProceduralLeaf*)leaf)->_primIndex[hit.primLeafIndex];
}

SYCL_EXTERNAL uint32_t intel_get_hit_primID_triangle( rayquery_t query, HitType hit_type )
{
  MemHit& hit = query.hit(hit_type);
  QuadLeaf* __restrict leaf = (QuadLeaf*) hit.getPrimLeafPtr();
  
  return leaf->primIndex0 + hit.primIndexDelta;
}

SYCL_EXTERNAL uint32_t intel_get_hit_primID_procedural( rayquery_t query, HitType hit_type )
{
  MemHit& hit = query.hit(hit_type);
  ProceduralLeaf* __restrict leaf = (ProceduralLeaf*) hit.getPrimLeafPtr();
  return leaf->_primIndex[hit.primLeafIndex];
}

SYCL_EXTERNAL uint32_t intel_get_hit_instID( rayquery_t query, HitType hit_type )
{
  MemHit& hit = query.hit(hit_type);
  InstanceLeaf* __restrict leaf = (InstanceLeaf*) hit.getInstanceLeafPtr();
  if (leaf == nullptr) return -1;
  return leaf->part1.instanceIndex;
}

SYCL_EXTERNAL uint32_t intel_get_hit_instUserID( rayquery_t query, HitType hit_type )
{
  MemHit& hit = query.hit(hit_type);
  InstanceLeaf* __restrict leaf = (InstanceLeaf*) hit.getInstanceLeafPtr();
  if (leaf == nullptr) return -1;
  return leaf->part1.instanceID;
}

SYCL_EXTERNAL float4x3_INTEL intel_get_hit_world_to_object( rayquery_t query, HitType hit_type )
{
  MemHit& hit = query.hit(hit_type);
  InstanceLeaf* __restrict leaf = (InstanceLeaf*) hit.getInstanceLeafPtr();
  if (leaf == nullptr) return { { 1,0,0 }, { 0,1,0 }, { 0,0,1 }, { 0,0,0 } };
  return {
    { leaf->part0.world2obj_vx[0], leaf->part0.world2obj_vx[1], leaf->part0.world2obj_vx[2] },
    { leaf->part0.world2obj_vy[0], leaf->part0.world2obj_vy[1], leaf->part0.world2obj_vy[2] },
    { leaf->part0.world2obj_vz[0], leaf->part0.world2obj_vz[1], leaf->part0.world2obj_vz[2] },
    { leaf->part1.world2obj_p [0], leaf->part1.world2obj_p [1], leaf->part1.world2obj_p [2] }
  };
}

SYCL_EXTERNAL float4x3_INTEL intel_get_hit_object_to_world( rayquery_t query, HitType hit_type )
{
  MemHit& hit = query.hit(hit_type);
  InstanceLeaf* __restrict leaf = (InstanceLeaf*) hit.getInstanceLeafPtr();
  if (leaf == nullptr) return { { 1,0,0 }, { 0,1,0 }, { 0,0,1 }, { 0,0,0 } };
  return {
    { leaf->part1.obj2world_vx[0], leaf->part1.obj2world_vx[1], leaf->part1.obj2world_vx[2] },
    { leaf->part1.obj2world_vy[0], leaf->part1.obj2world_vy[1], leaf->part1.obj2world_vy[2] },
    { leaf->part1.obj2world_vz[0], leaf->part1.obj2world_vz[1], leaf->part1.obj2world_vz[2] },
    { leaf->part0.obj2world_p [0], leaf->part0.obj2world_p [1], leaf->part0.obj2world_p [2] }
  };
}

SYCL_EXTERNAL void intel_get_hit_triangle_verts( rayquery_t query, float3 verts_out[3], HitType hit_type )
{
  const QuadLeaf* __restrict leaf = (const QuadLeaf*) query.hit(hit_type).getPrimLeafPtr();
  
  unsigned int j0 = 0, j1 = 1, j2 = 2;
  if (query.hit(hit_type).primLeafIndex != 0)
  {
    j0 = leaf->j0;
    j1 = leaf->j1;
    j2 = leaf->j2;
  }

  verts_out[0] = float3(leaf->v[j0][0], leaf->v[j0][1], leaf->v[j0][2]);
  verts_out[1] = float3(leaf->v[j1][0], leaf->v[j1][1], leaf->v[j1][2]);
  verts_out[2] = float3(leaf->v[j2][0], leaf->v[j2][1], leaf->v[j2][2]);
}

SYCL_EXTERNAL float3 intel_get_ray_origin( rayquery_t query, unsigned int bvh_level)
{
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query.opaque2).get();
  
  MemRay& ray = rtStack->ray[bvh_level];
  return float3(ray.org[0], ray.org[1], ray.org[2]);
}

SYCL_EXTERNAL float3 intel_get_ray_direction( rayquery_t query, unsigned int bvh_level)
{
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query.opaque2).get();
  MemRay& ray = rtStack->ray[bvh_level];
  return float3(ray.dir[0], ray.dir[1], ray.dir[2]);
}

SYCL_EXTERNAL float intel_get_ray_tmin( rayquery_t query, unsigned int bvh_level)
{
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query.opaque2).get();
  return rtStack->ray[bvh_level].tnear;
}

SYCL_EXTERNAL int intel_get_ray_flags( rayquery_t query, unsigned int bvh_level)
{
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query.opaque2).get();
  return rtStack->ray[bvh_level].rayFlags;
}

SYCL_EXTERNAL int intel_get_ray_mask( rayquery_t query, unsigned int bvh_level)
{
  struct RTStack* __restrict rtStack = sycl::global_ptr<RTStack>((struct RTStack*)query.opaque2).get();
  return rtStack->ray[bvh_level].rayMask;
}

SYCL_EXTERNAL bool intel_is_traversal_done( rayquery_t query ) {
  return query.hit(POTENTIAL_HIT).done;
}

SYCL_EXTERNAL CandidateType intel_get_hit_candidate( rayquery_t query, HitType hit_type) {
  return query.hit(hit_type).leafType == NODE_TYPE_QUAD ? TRIANGLE : PROCEDURAL;
}

SYCL_EXTERNAL bool intel_has_committed_hit( rayquery_t query ) {
  return query.hit(COMMITTED_HIT).valid;
}
