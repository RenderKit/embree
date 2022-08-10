// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "rthwif_internal.h"

enum RayFlagsIntel 
{
  // Flags directly exposed in API
  NONE = 0x00,
  FORCE_OPAQUE = 0x01,                      // forces geometry to be opaque (no anyhit shader invokation)
  FORCE_NON_OPAQUE = 0x02,                  // forces geometry to be non-opqaue (invoke anyhit shader)
  ACCEPT_FIRST_HIT_AND_END_SEARCH = 0x04,   // terminates traversal on the first hit found (shadow rays)
  SKIP_CLOSEST_HIT_SHADER = 0x08,           // skip execution of the closest hit shader
  CULL_BACK_FACING_TRIANGLES = 0x10,        // back facing triangles to not produce a hit
  CULL_FRONT_FACING_TRIANGLES = 0x20,       // front facing triangles do not produce a hit
  CULL_OPAQUE = 0x40,                       // opaque geometry does not produce a hit
  CULL_NON_OPAQUE = 0x80,                   // non-opaque geometry does not produce a hit
  SKIP_TRIANGLES = 0x100,                   // treat all triangle intersections as misses.
  SKIP_PROCEDURAL_PRIMITIVES = 0x200,       // skip execution of intersection shaders
  
  // Flags below are "internal use" flags
  TRIANGLE_FRONT_COUNTERCLOCKWISE = 0x4000, // switched front and backface of triangles
  LEVEL_ASCEND_DISABLED = 0x8000            // disables the automatic level ascend for this level, thus traversal will terminate when BVH at this level is done
};

enum HitType
{
  COMMITTED_HIT = 0,
  POTENTIAL_HIT = 1,
};

enum TraceRayCtrl
{
  TRACE_RAY_INITIAL = 0,              // Initializes hit and initializes traversal state
  TRACE_RAY_INSTANCE = 1,             // Loads committed hit and initializes traversal state
  TRACE_RAY_COMMIT = 2,               // Loads potential hit and loads traversal state 
  TRACE_RAY_CONTINUE = 3,             // Loads committed hit and loads traversal state
  TRACE_RAY_DONE = -1,                // for internal use only 
};

// opaque types
struct rayquery_t {
  void* opaque0; void* opaque1; void* opaque2; TraceRayCtrl ctrl; uint32_t bvh_level;
  MemHit& hit(HitType ty) {
    struct RTStack* rtStack = (struct RTStack*) opaque2;
    return rtStack->hit[ty];
  }
};
struct rtas_t;

struct RayDescINTEL
{
  sycl::float3 O;
  sycl::float3 D;
  float tmin; // FIXME: tmin or tnear?
  float tmax;
  uint32_t mask;
  uint32_t flags;
};

// initializes a ray query
SYCL_EXTERNAL rayquery_t intel_ray_query_init(
  unsigned int bvh_level,
  RayDescINTEL ray,
  rtas_t* accel,
  unsigned int bvh_id // FIXME: remove bvh_id
);

// setup for instance traversal using a transformed ray and bottom-level AS
SYCL_EXTERNAL void intel_ray_query_forward_ray(
  rayquery_t& query,
  unsigned int new_bvh_level,
  RayDescINTEL Ray,
  rtas_t* accel,
  unsigned int bvh_id); // FIXME: remove bvh_id

// commit the potential hit
SYCL_EXTERNAL void intel_ray_query_commit_potential_hit(
    rayquery_t& query
);

// commit the potential hit and override hit distance and UVs
SYCL_EXTERNAL void intel_ray_query_commit_potential_hit(
    rayquery_t& query,
    float override_hit_distance,
    sycl::float2 override_uv
);

// start traversal of a ray query
SYCL_EXTERNAL void intel_ray_query_start_traversal( rayquery_t& query );

// synchronize rayquery execution.  If a ray was dispatched, 
//  This must be called prior to calling any of the accessors below.
SYCL_EXTERNAL void intel_sync_ray_query( rayquery_t& query );

// signal that a ray query will not be used further.  This is the moral equaivalent of a delete
// this function does an implicit sync
SYCL_EXTERNAL void intel_ray_query_abandon( rayquery_t& query );

// read hit information during shader execution

SYCL_EXTERNAL unsigned int intel_get_hit_bvh_level( rayquery_t query, HitType hit_type );
SYCL_EXTERNAL float intel_get_hit_distance( rayquery_t query, HitType hit_type );
SYCL_EXTERNAL sycl::float2 intel_get_hit_barys( rayquery_t query, HitType hit_type );
SYCL_EXTERNAL bool intel_hit_is_front_face( rayquery_t query, HitType hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_geomID(rayquery_t query, HitType hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_primID( rayquery_t query, HitType hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_primID_triangle( rayquery_t query, HitType hit_type );  // fast path for quad leaves
SYCL_EXTERNAL uint32_t intel_get_hit_primID_procedural( rayquery_t query, HitType hit_type ); // fast path for procedural leaves
SYCL_EXTERNAL uint32_t intel_get_hit_instanceID( rayquery_t query, HitType hit_type );

// fetch triangle vertices for a hit
SYCL_EXTERNAL void intel_get_hit_triangle_verts( rayquery_t query, sycl::float3 verts_out[3], HitType hit_type );

//
// read ray-data
//  This is used to read transformed rays produced by HW instancing pipeline
//  during any-hit or intersection shader execution
//
SYCL_EXTERNAL sycl::float3 intel_get_ray_origin( rayquery_t query, unsigned int bvh_level );
SYCL_EXTERNAL sycl::float3 intel_get_ray_direction( rayquery_t query, unsigned int bvh_level );
SYCL_EXTERNAL float intel_get_ray_tnear( rayquery_t query, unsigned int bvh_level );
SYCL_EXTERNAL int intel_get_ray_flags( rayquery_t query, unsigned int bvh_level ); // FIXME: uint32_t?
SYCL_EXTERNAL int intel_get_ray_mask( rayquery_t query, unsigned int bvh_level ); // FIXME: uint32_t?

// test whether traversal has terminated.  If false, the ray has reached
//  a procedural leaf or a non-opaque triangle leaf, and requires shader processing
SYCL_EXTERNAL bool intel_is_traversal_done( rayquery_t query );

// if traversal is not done one can test if a triangle is hit (anyhit
// shader needs to get invoked) or a procedural (intersection shader
// should get invoked)

enum CandidateType { TRIANGLE, PROCEDURAL };
SYCL_EXTERNAL CandidateType intel_get_hit_candidate( rayquery_t query, HitType hit_type );

// if traversal is done one can test for the presence of a committed hit to either invoke miss or closest hit shader
SYCL_EXTERNAL bool intel_has_committed_hit( rayquery_t query );
