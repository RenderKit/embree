// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "rthwif_internal.h"

enum RayFlagsINTEL
{
  // Flags directly exposed in API
  RAY_FLAGS_INTEL_NONE = 0x00,
  RAY_FLAGS_INTEL_FORCE_OPAQUE = 0x01,                      // forces geometry to be opaque (no anyhit shader invokation)
  RAY_FLAGS_INTEL_FORCE_NON_OPAQUE = 0x02,                  // forces geometry to be non-opqaue (invoke anyhit shader)
  RAY_FLAGS_INTEL_ACCEPT_FIRST_HIT_AND_END_SEARCH = 0x04,   // terminates traversal on the first hit found (shadow rays)
  RAY_FLAGS_INTEL_SKIP_CLOSEST_HIT_SHADER = 0x08,           // skip execution of the closest hit shader
  RAY_FLAGS_INTEL_CULL_BACK_FACING_TRIANGLES = 0x10,        // back facing triangles to not produce a hit
  RAY_FLAGS_INTEL_CULL_FRONT_FACING_TRIANGLES = 0x20,       // front facing triangles do not produce a hit
  RAY_FLAGS_INTEL_CULL_OPAQUE = 0x40,                       // opaque geometry does not produce a hit
  RAY_FLAGS_INTEL_CULL_NON_OPAQUE = 0x80,                   // non-opaque geometry does not produce a hit
  RAY_FLAGS_INTEL_SKIP_TRIANGLES = 0x100,                   // treat all triangle intersections as misses.
  RAY_FLAGS_INTEL_SKIP_PROCEDURAL_PRIMITIVES = 0x200,       // skip execution of intersection shaders
};

enum HitType
{
  COMMITTED_HIT = 0,
  POTENTIAL_HIT = 1,
};

// opaque types
struct ray_query_t {
  void* opaque0; void* opaque1; void* opaque2; uint32_t ctrl; uint32_t bvh_level;
  MemHit& hit(HitType ty) {
    struct RTStack* rtStack = (struct RTStack*) opaque2;
    return rtStack->hit[ty];
  }
};
struct rtas_t;

struct float4x3_INTEL {
  sycl::float3 vx, vy, vz, p;
};

struct RayDescINTEL
{
  sycl::float3 O;
  sycl::float3 D;
  float tmin;
  float tmax;
  uint32_t mask;
  uint32_t flags;
};

// initializes a ray query
SYCL_EXTERNAL ray_query_t intel_ray_query_init(
  unsigned int bvh_level,
  RayDescINTEL ray,
  rtas_t* accel,
  unsigned int bvh_id // FIXME: remove bvh_id
);

// setup for instance traversal using a transformed ray and bottom-level AS
SYCL_EXTERNAL void intel_ray_query_forward_ray(
  ray_query_t* query,
  unsigned int new_bvh_level,
  RayDescINTEL Ray,
  rtas_t* accel,
  unsigned int bvh_id // FIXME: remove bvh_id
);

// commit the potential hit
SYCL_EXTERNAL void intel_ray_query_commit_potential_hit(
    ray_query_t* query
);

// commit the potential hit and override hit distance and UVs
SYCL_EXTERNAL void intel_ray_query_commit_potential_hit(
    ray_query_t* query,
    float override_hit_distance,
    sycl::float2 override_uv
);

// start traversal of a ray query
SYCL_EXTERNAL void intel_ray_query_start_traversal( ray_query_t* query );

// synchronize rayquery execution.  If a ray was dispatched, 
//  This must be called prior to calling any of the accessors below.
SYCL_EXTERNAL void intel_ray_query_sync( ray_query_t* query );

// signal that a ray query will not be used further.  This is the moral equaivalent of a delete
// this function does an implicit sync
SYCL_EXTERNAL void intel_ray_query_abandon( ray_query_t* query );

// read hit information during shader execution
SYCL_EXTERNAL unsigned int intel_get_hit_bvh_level( ray_query_t query, HitType hit_type );
SYCL_EXTERNAL float intel_get_hit_distance( ray_query_t query, HitType hit_type );
SYCL_EXTERNAL sycl::float2 intel_get_hit_barys( ray_query_t query, HitType hit_type );
SYCL_EXTERNAL bool intel_hit_is_front_face( ray_query_t query, HitType hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_geomID(ray_query_t query, HitType hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_primID( ray_query_t query, HitType hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_primID_triangle( ray_query_t query, HitType hit_type );  // fast path for quad leaves
SYCL_EXTERNAL uint32_t intel_get_hit_primID_procedural( ray_query_t query, HitType hit_type ); // fast path for procedural leaves

SYCL_EXTERNAL uint32_t intel_get_hit_instID( ray_query_t query, HitType hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_instUserID( ray_query_t query, HitType hit_type );
SYCL_EXTERNAL float4x3_INTEL intel_get_hit_world_to_object( ray_query_t query, HitType hit_type );
SYCL_EXTERNAL float4x3_INTEL intel_get_hit_object_to_world( ray_query_t query, HitType hit_type );

// fetch triangle vertices for a hit
SYCL_EXTERNAL void intel_get_hit_triangle_verts( ray_query_t query, sycl::float3 verts_out[3], HitType hit_type );

//
// read ray-data
//  This is used to read transformed rays produced by HW instancing pipeline
//  during any-hit or intersection shader execution
//
SYCL_EXTERNAL sycl::float3 intel_get_ray_origin( ray_query_t query, unsigned int bvh_level );
SYCL_EXTERNAL sycl::float3 intel_get_ray_direction( ray_query_t query, unsigned int bvh_level );
SYCL_EXTERNAL float intel_get_ray_tmin( ray_query_t query, unsigned int bvh_level );
SYCL_EXTERNAL int intel_get_ray_flags( ray_query_t query, unsigned int bvh_level ); // FIXME: uint32_t?
SYCL_EXTERNAL int intel_get_ray_mask( ray_query_t query, unsigned int bvh_level ); // FIXME: uint32_t?

// test whether traversal has terminated.  If false, the ray has reached
//  a procedural leaf or a non-opaque triangle leaf, and requires shader processing
SYCL_EXTERNAL bool intel_is_traversal_done( ray_query_t query );

// if traversal is not done one can test if a triangle is hit (anyhit
// shader needs to get invoked) or a procedural (intersection shader
// should get invoked)

enum CandidateType { TRIANGLE, PROCEDURAL };
SYCL_EXTERNAL CandidateType intel_get_hit_candidate( ray_query_t query, HitType hit_type );

// if traversal is done one can test for the presence of a committed hit to either invoke miss or closest hit shader
SYCL_EXTERNAL bool intel_has_committed_hit( ray_query_t query );
