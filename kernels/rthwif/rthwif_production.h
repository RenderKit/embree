// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "rthwif_internal.h"

enum intel_ray_flags_t
{
  // Flags directly exposed in API
  intel_ray_flags_none = 0x00,
  intel_ray_flags_force_opaque = 0x01,                      // forces geometry to be opaque (no anyhit shader invokation)
  intel_ray_flags_force_non_opaque = 0x02,                  // forces geometry to be non-opqaue (invoke anyhit shader)
  intel_ray_flags_accept_first_hit_and_end_search = 0x04,   // terminates traversal on the first hit found (shadow rays)
  intel_ray_flags_skip_closest_hit_shader = 0x08,           // skip execution of the closest hit shader
  intel_ray_flags_cull_back_facing_triangles = 0x10,        // back facing triangles to not produce a hit
  intel_ray_flags_cull_front_facing_triangles = 0x20,       // front facing triangles do not produce a hit
  intel_ray_flags_cull_opaque = 0x40,                       // opaque geometry does not produce a hit
  intel_ray_flags_cull_non_opaque = 0x80,                   // non-opaque geometry does not produce a hit
  intel_ray_flags_skip_triangles = 0x100,                   // treat all triangle intersections as misses.
  intel_ray_flags_skip_procedural_primitives = 0x200,       // skip execution of intersection shaders
};

enum intel_hit_type_t
{
  intel_hit_type_committed_hit = 0,
  intel_hit_type_potential_hit = 1,
};

// opaque types
struct intel_ray_query_t {
  void* opaque0; void* opaque1; void* opaque2; uint32_t ctrl; uint32_t bvh_level;
  MemHit& hit(intel_hit_type_t ty) {
    struct RTStack* rtStack = (struct RTStack*) opaque2;
    return rtStack->hit[ty];
  }
};
struct intel_raytracing_acceleration_structure_t;

struct intel_float4x3 {
  sycl::float3 vx, vy, vz, p;
};

struct intel_ray_desc_t
{
  sycl::float3 origin;
  sycl::float3 direction;
  float tmin;
  float tmax;
  uint32_t mask;
  uint32_t flags;
  float time;
};

// initializes a ray query
SYCL_EXTERNAL intel_ray_query_t intel_ray_query_init(
  intel_ray_desc_t ray,
  intel_raytracing_acceleration_structure_t* accel,
  unsigned int bvh_id // FIXME: remove bvh_id
);

// setup for instance traversal using a transformed ray and bottom-level AS
SYCL_EXTERNAL void intel_ray_query_forward_ray(
  intel_ray_query_t* query,
  unsigned int new_bvh_level,
  intel_ray_desc_t Ray,
  intel_raytracing_acceleration_structure_t* accel,
  unsigned int bvh_id // FIXME: remove bvh_id
);

// commit the potential hit
SYCL_EXTERNAL void intel_ray_query_commit_potential_hit(
    intel_ray_query_t* query
);

// commit the potential hit and override hit distance and UVs
SYCL_EXTERNAL void intel_ray_query_commit_potential_hit(
    intel_ray_query_t* query,
    float override_hit_distance,
    sycl::float2 override_uv
);

// start traversal of a ray query
SYCL_EXTERNAL void intel_ray_query_start_traversal( intel_ray_query_t* query );

// synchronize rayquery execution.  If a ray was dispatched, 
//  This must be called prior to calling any of the accessors below.
SYCL_EXTERNAL void intel_ray_query_sync( intel_ray_query_t* query );

// signal that a ray query will not be used further.  This is the moral equaivalent of a delete
// this function does an implicit sync
SYCL_EXTERNAL void intel_ray_query_abandon( intel_ray_query_t* query );

// read hit information during shader execution
SYCL_EXTERNAL unsigned int intel_get_hit_bvh_level( intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL float intel_get_hit_distance( intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL sycl::float2 intel_get_hit_barycentrics( intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL bool intel_hit_is_front_face( intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_geometry_id(intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_primitive_id( intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_triangle_primitive_id( intel_ray_query_t* query, intel_hit_type_t hit_type );  // fast path for quad leaves
SYCL_EXTERNAL uint32_t intel_get_hit_procedural_primitive_id( intel_ray_query_t* query, intel_hit_type_t hit_type ); // fast path for procedural leaves

SYCL_EXTERNAL uint32_t intel_get_hit_instance_id( intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL uint32_t intel_get_hit_instance_user_id( intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL intel_float4x3 intel_get_hit_world_to_object( intel_ray_query_t* query, intel_hit_type_t hit_type );
SYCL_EXTERNAL intel_float4x3 intel_get_hit_object_to_world( intel_ray_query_t* query, intel_hit_type_t hit_type );

// fetch triangle vertices for a hit
SYCL_EXTERNAL void intel_get_hit_triangle_vertices( intel_ray_query_t* query, sycl::float3 verts_out[3], intel_hit_type_t hit_type );

//
// read ray-data
//  This is used to read transformed rays produced by HW instancing pipeline
//  during any-hit or intersection shader execution
//
SYCL_EXTERNAL sycl::float3 intel_get_ray_origin( intel_ray_query_t* query, unsigned int bvh_level );
SYCL_EXTERNAL sycl::float3 intel_get_ray_direction( intel_ray_query_t* query, unsigned int bvh_level );
SYCL_EXTERNAL float intel_get_ray_tmin( intel_ray_query_t* query, unsigned int bvh_level );
SYCL_EXTERNAL int intel_get_ray_flags( intel_ray_query_t* query, unsigned int bvh_level ); // FIXME: uint32_t?
SYCL_EXTERNAL int intel_get_ray_mask( intel_ray_query_t* query, unsigned int bvh_level ); // FIXME: uint32_t?

// test whether traversal has terminated.  If false, the ray has reached
//  a procedural leaf or a non-opaque triangle leaf, and requires shader processing
SYCL_EXTERNAL bool intel_is_traversal_done( intel_ray_query_t* query );

// if traversal is not done one can test if a triangle is hit (anyhit
// shader needs to get invoked) or a procedural (intersection shader
// should get invoked)

enum intel_candidate_type_t
{
  intel_candidate_type_triangle,
  intel_candidate_type_procedural
};

SYCL_EXTERNAL intel_candidate_type_t intel_get_hit_candidate( intel_ray_query_t* query, intel_hit_type_t hit_type );

// if traversal is done one can test for the presence of a committed hit to either invoke miss or closest hit shader
SYCL_EXTERNAL bool intel_has_committed_hit( intel_ray_query_t* query );
