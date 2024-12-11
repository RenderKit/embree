// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/default.h"
#include "../common/device.h"
#include "../common/scene.h"
#include "../common/context.h"
#include "../../include/embree4/rtcore.h"

#include "../geometry/disc_intersector.h"
#include "../geometry/sphere_intersector.h"
#include "../geometry/roundline_intersector.h"
#include "../geometry/curve_intersector_ribbon.h"
#include "../geometry/curve_intersector_oriented.h"
#include "../geometry/curve_intersector_sweep.h"
#include "../geometry/curve_intersector_distance.h"
#include "../geometry/curve_intersector_ribbon.h"
#include "../geometry/curveNi_intersector.h"
#include "../geometry/instance_intersector.h"
#include "../geometry/instance_array_intersector.h"
#include "../geometry/intersector_epilog_sycl.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"

#include "rthwif_embree.h"
#include "../rthwif/rttrace/rttrace.h"

using namespace embree;

intel_ray_flags_t operator |(intel_ray_flags_t a, intel_ray_flags_t b) {
  return (intel_ray_flags_t) ((uint32_t)a | (uint32_t)b);
}

intel_ray_flags_t operator |= (intel_ray_flags_t& a, intel_ray_flags_t b) {
  return a = a | b;
}

RTC_NAMESPACE_BEGIN;

//#if defined(EMBREE_SYCL_ROBUST)
#define TriangleIntersector isa::PlueckerIntersector1<1,true>
#define ROBUST_MODE true
//#else
//#define TriangleIntersector isa::MoellerTrumboreIntersector1<1,true>
//#define ROBUST_MODE false
//#endif

const constexpr uint32_t TRAV_LOOP_FEATURES =
#if (RTC_MAX_INSTANCE_LEVEL_COUNT > 1)
  RTC_FEATURE_FLAG_TRIANGLE |   // filter function enforced for triangles and quads in this case
  RTC_FEATURE_FLAG_QUAD |
#endif
#if defined(EMBREE_RAY_MASK)
  RTC_FEATURE_FLAG_32_BIT_RAY_MASK |
#endif
  RTC_FEATURE_FLAG_MOTION_BLUR |
  RTC_FEATURE_FLAG_CURVES |
  RTC_FEATURE_FLAG_GRID |
  RTC_FEATURE_FLAG_POINT |
  RTC_FEATURE_FLAG_USER_GEOMETRY |
  RTC_FEATURE_FLAG_INSTANCE |
  RTC_FEATURE_FLAG_INSTANCE_ARRAY |
  RTC_FEATURE_FLAG_FILTER_FUNCTION;

/*
  low-level optimized array of scenes for (multi-level) instancing. Our compiler
  really does not like const sized arrays ...
*/
#if (RTC_MAX_INSTANCE_LEVEL_COUNT > 2)
struct Scenes {
  Scenes(Scene* scene) : scene0(scene) {}

  __forceinline void set(unsigned int level, Scene* scene) {
    if (level == 0) scene0 = scene;
    else            scenes[level-1] = scene;
  }

  __forceinline Scene* get(unsigned int level) {
    return level ? scenes[level-1] : scene0;
  }

  Scene* scene0;
  Scene* scenes[RTC_MAX_INSTANCE_LEVEL_COUNT];
};
#elif (RTC_MAX_INSTANCE_LEVEL_COUNT == 2)
struct Scenes {
  Scenes(Scene* scene) : scene0(scene) {}

  __forceinline void set(unsigned int level, Scene* scene) {
    if      (level == 0) scene0 = scene;
    else if (level == 1) scene1 = scene;
    else                 scene2 = scene;
  }

  __forceinline Scene* get(unsigned int level) {
    if      (level == 0) return scene0;
    else if (level == 1) return scene1;
    else                 return scene2;
  }

  Scene* scene0;
  Scene* scene1;
  Scene* scene2;
};
#else
struct Scenes {
  Scenes(Scene* scene) : scene0(scene) {}

  __forceinline void set(unsigned int level, Scene* scene) {
    if (level == 0) scene0 = scene;
    else            scene1 = scene;
  }

  __forceinline Scene* get(unsigned int level) {
    return level ? scene1 : scene0;
  }

  Scene* scene0;
  Scene* scene1;
};
#endif

void use_rthwif_embree() {
}

__forceinline Vec3f intel_get_hit_triangle_normal(intel_ray_query_t& query, intel_hit_type_t hit_type)
{
  intel_float3 v[3]; intel_get_hit_triangle_vertices(query, v, hit_type);
  const Vec3f v0(v[0].x, v[0].y, v[0].z);
  const Vec3f v1(v[1].x, v[1].y, v[1].z);
  const Vec3f v2(v[2].x, v[2].y, v[2].z);
  return cross(v1-v0, v2-v0);
}

__forceinline bool intersect_user_geometry(intel_ray_query_t& query, RayHit& ray, UserGeometry* geom, Scenes& scenes, sycl::private_ptr<RayQueryContext> context, uint32_t geomID, uint32_t primID)
{
  /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
  if ((ray.mask & geom->mask) == 0)
    return false;
#endif

  RTCScene forward_scene = nullptr;
  const bool ishit = geom->intersect(ray,geomID,primID,context,forward_scene);
  if (!forward_scene) return ishit;

  /* forward ray to instanced scene */
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  unsigned int bvh_level = intel_get_hit_bvh_level( query, intel_hit_type_potential_hit );
#else
  constexpr unsigned int bvh_level = 0;
#endif
  Scene* scene = (Scene*) forward_scene;
  scenes.set(bvh_level+1, scene);

  intel_ray_desc_t raydesc;
  raydesc.origin = float3(ray.org.x, ray.org.y, ray.org.z);
  raydesc.direction = float3(ray.dir.x, ray.dir.y, ray.dir.z);
  raydesc.tmin = ray.tnear();
  raydesc.tmax = inf; // unused
  raydesc.mask = mask32_to_mask8(ray.mask);
  raydesc.flags = intel_ray_flags_force_non_opaque;

  //if (context.enforceArgumentFilterFunction())
  //  raydesc.flags |= intel_ray_flags_force_non_opaque;

#if defined(EMBREE_BACKFACE_CULLING)
  raydesc.flags |= intel_ray_flags_cull_back_facing_triangles;
#endif

  intel_raytracing_acceleration_structure_t hwaccel_ptr = (intel_raytracing_acceleration_structure_t) scene->accelBuffer.getHWAccel(0);

  intel_ray_query_forward_ray(query, raydesc, hwaccel_ptr);
  return false;
}

__forceinline bool intersect_user_geometry(intel_ray_query_t& query, Ray& ray, UserGeometry* geom, Scenes& scenes, sycl::private_ptr<RayQueryContext> context, uint32_t geomID, uint32_t primID)
{
  /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
  if ((ray.mask & geom->mask) == 0) 
    return false;
#endif

  RTCScene forward_scene = nullptr;
  const bool ishit = geom->occluded(ray,geomID,primID,context,forward_scene);
  if (!forward_scene) return ishit;

  /* forward ray to instanced scene */
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  unsigned int bvh_level = intel_get_hit_bvh_level( query, intel_hit_type_potential_hit );
#else
  constexpr unsigned int bvh_level = 0;
#endif
  Scene* scene = (Scene*) forward_scene;
  scenes.set(bvh_level+1, scene);

  intel_ray_desc_t raydesc;
  raydesc.origin = float3(ray.org.x, ray.org.y, ray.org.z);
  raydesc.direction = float3(ray.dir.x, ray.dir.y, ray.dir.z);
  raydesc.tmin = ray.tnear();
  raydesc.tmax = inf; // unused
  raydesc.mask = mask32_to_mask8(ray.mask);
  raydesc.flags = intel_ray_flags_force_non_opaque | intel_ray_flags_accept_first_hit_and_end_search;

  //if (context.enforceArgumentFilterFunction())
  //  raydesc.flags |= intel_ray_flags_force_non_opaque;

#if defined(EMBREE_BACKFACE_CULLING)
  raydesc.flags |= intel_ray_flags_cull_back_facing_triangles;
#endif

  intel_raytracing_acceleration_structure_t hwaccel_ptr = (intel_raytracing_acceleration_structure_t) scene->accelBuffer.getHWAccel(0);

  intel_ray_query_forward_ray(query, raydesc, hwaccel_ptr);
  return false;
}

template<typename Ray>
__forceinline bool intersect_instance(intel_ray_query_t& query, Ray& ray, Geometry* geom, Scenes& scenes, sycl::private_ptr<RayQueryContext> context, uint32_t geomID, uint32_t primID, bool instance_array);

template<>
__forceinline bool intersect_instance(intel_ray_query_t& query, RayHit& ray, Geometry* geom, Scenes& scenes, sycl::private_ptr<RayQueryContext> context, uint32_t geomID, uint32_t primID, bool instance_array)
{
  /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
  if ((ray.mask & geom->mask) == 0)
    return false;
#endif

  if (!instance_id_stack::push(context->user, geomID, primID))
    return false;

#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  unsigned int bvh_level = intel_get_hit_bvh_level( query, intel_hit_type_potential_hit );
#else
  constexpr unsigned int bvh_level = 0;
#endif

  Scene* object = instance_array
    ? (Scene*) ((InstanceArray*)geom)->getObject(primID)
    : (Scene*) ((Instance*)geom)->object;
  if (!object) return false;

  const AffineSpace3fa world2local = instance_array
    ? ((InstanceArray*)geom)->getWorld2Local(primID, ray.time())
    : ((Instance*)geom)->getWorld2Local(ray.time());

  const Vec3fa ray_org = xfmPoint (world2local, (Vec3f) ray.org);
  const Vec3fa ray_dir = xfmVector(world2local, (Vec3f) ray.dir);
  scenes.set(bvh_level+1, object);
  
  intel_ray_desc_t raydesc;
  raydesc.origin = float3(ray_org.x, ray_org.y, ray_org.z);
  raydesc.direction = float3(ray_dir.x, ray_dir.y, ray_dir.z);
  raydesc.tmin = ray.tnear();
  raydesc.tmax = inf; // unused
  raydesc.mask = mask32_to_mask8(ray.mask);
  raydesc.flags = intel_ray_flags_force_non_opaque;

#if defined(EMBREE_BACKFACE_CULLING)
  raydesc.flags |= intel_ray_flags_cull_back_facing_triangles;
#endif

  uint32_t bvh_id = 0;
  if (context->args->feature_mask & RTC_FEATURE_FLAG_MOTION_BLUR) {
    float time = clamp(ray.time(),0.0f,1.0f);
    uint32_t numTimeSegments = object->getMaxTimeSegments();
    bvh_id = (uint32_t) clamp(uint32_t(numTimeSegments*time), 0u, numTimeSegments-1);
  }

  intel_raytracing_acceleration_structure_t hwaccel_ptr = (intel_raytracing_acceleration_structure_t) object->accelBuffer.getHWAccel(bvh_id);
  
  intel_ray_query_forward_ray(query, raydesc, hwaccel_ptr);

  return false;
}

template<>
__forceinline bool intersect_instance(intel_ray_query_t& query, Ray& ray, Geometry* geom, Scenes& scenes, sycl::private_ptr<RayQueryContext> context, uint32_t geomID, uint32_t primID, bool instance_array)
{
  /* perform ray mask test */
#if defined(EMBREE_RAY_MASK)
  if ((ray.mask & geom->mask) == 0) 
    return false;
#endif

  if (!instance_id_stack::push(context->user, geomID, primID))
    return false;

#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  unsigned int bvh_level = intel_get_hit_bvh_level( query, intel_hit_type_potential_hit );
#else
  constexpr unsigned int bvh_level = 0;
#endif

  Scene* object = instance_array
    ? (Scene*) ((InstanceArray*)geom)->getObject(primID)
    : (Scene*) ((Instance*)geom)->object;
  if (!object) return false;

  const AffineSpace3fa world2local = instance_array
    ? ((InstanceArray*)geom)->getWorld2Local(primID, ray.time())
    : ((Instance*)geom)->getWorld2Local(ray.time());

  const Vec3fa ray_org = xfmPoint (world2local, (Vec3f) ray.org);
  const Vec3fa ray_dir = xfmVector(world2local, (Vec3f) ray.dir);
  scenes.set(bvh_level+1, object);
  
  intel_ray_desc_t raydesc;
  raydesc.origin = float3(ray_org.x, ray_org.y, ray_org.z);
  raydesc.direction = float3(ray_dir.x, ray_dir.y, ray_dir.z);
  raydesc.tmin = ray.tnear();
  raydesc.tmax = inf; // unused
  raydesc.mask = mask32_to_mask8(ray.mask);
  raydesc.flags = intel_ray_flags_accept_first_hit_and_end_search;

  //if (context->enforceArgumentFilterFunction())
  //  raydesc.flags |= intel_ray_flags_force_non_opaque;
  
#if defined(EMBREE_BACKFACE_CULLING)
  raydesc.flags |= intel_ray_flags_cull_back_facing_triangles;
#endif

  uint32_t bvh_id = 0;
  if (context->args->feature_mask & RTC_FEATURE_FLAG_MOTION_BLUR) {
    float time = clamp(ray.time(),0.0f,1.0f);
    uint32_t numTimeSegments = object->getMaxTimeSegments();
    bvh_id = (uint32_t) clamp(uint32_t(numTimeSegments*time), 0u, numTimeSegments-1);
  }

  intel_raytracing_acceleration_structure_t hwaccel_ptr = (intel_raytracing_acceleration_structure_t) object->accelBuffer.getHWAccel(bvh_id);

  intel_ray_query_forward_ray(query, raydesc, hwaccel_ptr);

  return false;
}

template<typename Ray>
__forceinline bool intersect_primitive(intel_ray_query_t& query, Ray& ray, Scenes& scenes, Geometry* geom, sycl::private_ptr<RayQueryContext> context, uint32_t geomID, uint32_t primID, const RTCFeatureFlags feature_mask)
{
#if defined(EMBREE_SYCL_SUPPORT) && defined(__SYCL_DEVICE_ONLY__)
  bool filter = feature_mask & (RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS | RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY);
  if (feature_mask & RTC_FEATURE_FLAG_MOTION_BLUR) {
    if (ray.time() < geom->time_range.lower || geom->time_range.upper < ray.time())
      return false;
  }

#if defined(EMBREE_GEOMETRY_USER)
  if ((feature_mask & RTC_FEATURE_FLAG_USER_GEOMETRY) && (geom->getType() == Geometry::GTY_USER_GEOMETRY)) {
    return intersect_user_geometry(query,ray,(UserGeometry*)geom, scenes, context, geomID, primID);
  }
#endif

#if defined(EMBREE_GEOMETRY_INSTANCE) || defined(EMBREE_GEOMETRY_INSTANCE_ARRAY)
  if ((feature_mask & RTC_FEATURE_FLAG_INSTANCE) && (geom->getTypeMask() & Geometry::MTY_INSTANCE) ||
      (feature_mask & RTC_FEATURE_FLAG_INSTANCE_ARRAY) && (geom->getTypeMask() & Geometry::MTY_INSTANCE_ARRAY)) {
    return intersect_instance(query,ray,(Instance*)geom, scenes, context, geomID, primID, geom->getTypeMask() & Geometry::MTY_INSTANCE_ARRAY);
  }
#endif

  isa::CurvePrecalculations1 pre(ray,context->scene);
  
  const Geometry::GType gtype MAYBE_UNUSED = geom->getType();
  const Geometry::GType stype MAYBE_UNUSED = (Geometry::GType)(gtype & Geometry::GTY_SUBTYPE_MASK);
  const Geometry::GType basis MAYBE_UNUSED = (Geometry::GType)(gtype & Geometry::GTY_BASIS_MASK);
  
#if defined(EMBREE_GEOMETRY_TRIANGLE)
  if (gtype == Geometry::GTY_TRIANGLE_MESH && (feature_mask & RTC_FEATURE_FLAG_TRIANGLE) && (feature_mask & RTC_FEATURE_FLAG_MOTION_BLUR))
  {
    const TriangleMesh* geom = (const TriangleMesh*)context->scene->get(geomID);
    const TriangleMesh::Triangle triangle = geom->triangle(primID);
    Vec3fa v0 = geom->vertex(triangle.v[0], ray.time());
    Vec3fa v1 = geom->vertex(triangle.v[1], ray.time());
    Vec3fa v2 = geom->vertex(triangle.v[2], ray.time());
    return TriangleIntersector().intersect(ray,v0,v1,v2,Intersect1Epilog1_HWIF<Ray>(ray, context, geomID, primID, filter));
  } else
#endif

#if defined(EMBREE_GEOMETRY_QUAD)
  if (gtype == Geometry::GTY_QUAD_MESH && (feature_mask & RTC_FEATURE_FLAG_QUAD) && (feature_mask & RTC_FEATURE_FLAG_MOTION_BLUR))
  {
    const QuadMesh* geom = context->scene->get<QuadMesh>(geomID);
    const QuadMesh::Quad quad = geom->quad(primID);
    Vec3fa v0 = geom->vertex(quad.v[0], ray.time());
    Vec3fa v1 = geom->vertex(quad.v[1], ray.time());
    Vec3fa v2 = geom->vertex(quad.v[2], ray.time());
    Vec3fa v3 = geom->vertex(quad.v[3], ray.time());
    bool ishit0 = TriangleIntersector().intersect(ray,v0,v1,v3,Intersect1Epilog1_HWIF<Ray>(ray, context, geomID, primID, filter));
    bool ishit1 = TriangleIntersector().intersect(ray,v2,v3,v1,[&](float &u, float &v, Vec3f& Ng) { u = 1.f - u; v = 1.f - v; }, Intersect1Epilog1_HWIF<Ray>(ray, context, geomID, primID, filter));
    return ishit0 || ishit1;
  } else
#endif

#if defined(EMBREE_GEOMETRY_GRID)
  if (gtype == Geometry::GTY_GRID_MESH && (feature_mask & RTC_FEATURE_FLAG_GRID))
  {
    const GridMesh* mesh = context->scene->get<GridMesh>(geomID);
    const GridMesh::PrimID_XY c = mesh->quadID_to_primID_xy[primID];
    const GridMesh::Grid& g = mesh->grid(c.primID);

    auto map_uv0 = [&](float &u, float &v, Vec3f& Ng) {
      const Vec2f uv(u,v);
      u = (c.x + uv.x)/(g.resX-1);
      v = (c.y + uv.y)/(g.resY-1);
    };

    auto map_uv1 = [&](float &u, float &v, Vec3f& Ng) {
      const Vec2f uv(1.0f-u, 1.0f-v);
      u = (c.x + uv.x)/(g.resX-1);
      v = (c.y + uv.y)/(g.resY-1);
    };

    Vec3fa v0,v1,v2,v3;
    mesh->gather_quad_vertices_safe(v0,v1,v2,v3,g,c.x,c.y,ray.time());

    bool ishit0 = TriangleIntersector().intersect(ray,v0,v1,v3,map_uv0,Intersect1Epilog1_HWIF<Ray>(ray, context, geomID, c.primID, filter));
    bool ishit1 = TriangleIntersector().intersect(ray,v2,v3,v1,map_uv1,Intersect1Epilog1_HWIF<Ray>(ray, context, geomID, c.primID, filter));
    return ishit0 || ishit1;
  } else
#endif

#if defined(EMBREE_GEOMETRY_POINT)
  
  if (gtype == Geometry::GTY_SPHERE_POINT && (feature_mask & RTC_FEATURE_FLAG_SPHERE_POINT))
  {
    const Points* geom = context->scene->get<Points>(geomID);
    const Vec3ff xyzr = geom->vertex_safe(primID, ray.time());
    const Vec4f vr(xyzr.x,xyzr.y,xyzr.z,xyzr.w);
    return isa::SphereIntersector1<1>::intersect(true, ray, pre, vr, Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
  }
  else if (gtype == Geometry::GTY_DISC_POINT && (feature_mask & RTC_FEATURE_FLAG_DISC_POINT))
  {
    const Points* geom = context->scene->get<Points>(geomID);
    const Vec3ff xyzr = geom->vertex_safe(primID, ray.time());
    const Vec4f vr(xyzr.x,xyzr.y,xyzr.z,xyzr.w);
    return isa::DiscIntersector1<1>::intersect(true, ray, nullptr, nullptr, pre, vr, Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
  }
  else if (gtype == Geometry::GTY_ORIENTED_DISC_POINT && (feature_mask & RTC_FEATURE_FLAG_ORIENTED_DISC_POINT))
  {
    const Points* geom = context->scene->get<Points>(geomID);
    const Vec3ff xyzr = geom->vertex_safe(primID, ray.time());
    const Vec4f vr(xyzr.x,xyzr.y,xyzr.z,xyzr.w);
    const Vec3f n = geom->normal_safe(primID, ray.time());
    return isa::DiscIntersector1<1>::intersect(true, ray, nullptr, nullptr, pre, vr, n, Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
  } else
    
#endif

#if defined(EMBREE_GEOMETRY_CURVE)

  if (geom->getTypeMask() & Geometry::MTY_CURVES)
  {
    if (gtype == Geometry::GTY_FLAT_LINEAR_CURVE && (feature_mask & RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE))
    {
      LineSegments* geom = context->scene->get<LineSegments>(geomID);
      Vec3ff v0, v1; geom->gather_safe(v0,v1,geom->segment(primID),ray.time());
      return isa::FlatLinearCurveIntersector1<1>::intersect(true,ray,context,geom,pre,v0,v1,Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
    }
    else if (gtype == Geometry::GTY_ROUND_LINEAR_CURVE && (feature_mask & RTC_FEATURE_FLAG_ROUND_LINEAR_CURVE))
    {
      LineSegments* geom = context->scene->get<LineSegments>(geomID);
      Vec3ff v0,v1,v2,v3; geom->gather_safe(v0,v1,v2,v3,primID,geom->segment(primID),ray.time());
      return isa::RoundLinearCurveIntersector1<1>().intersect(true,ray,context,geom,pre,v0,v1,v2,v3,Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
    }
    else if (gtype == Geometry::GTY_CONE_LINEAR_CURVE && (feature_mask & RTC_FEATURE_FLAG_CONE_LINEAR_CURVE))
    {
      LineSegments* geom = context->scene->get<LineSegments>(geomID);
      Vec3ff v0 = zero, v1 = zero; bool cL = false, cR = false;
      geom->gather_safe(v0,v1,cL,cR,primID,geom->segment(primID),ray.time());
      return isa::ConeCurveIntersector1<1>().intersect(true,ray,context,geom,pre,v0,v1,cL,cR,Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
    }
    else
    {
      CurveGeometry* geom = context->scene->get<CurveGeometry>(geomID);
      if (stype == Geometry::GTY_SUBTYPE_ORIENTED_CURVE && (feature_mask & RTC_FEATURE_FLAG_NORMAL_ORIENTED_CURVES))
      {
        using Intersector = isa::OrientedCurve1Intersector1<CubicBezierCurve,8,1>;
        using Curve = isa::TensorLinearCubicBezierSurface3fa;
        if (geom->numTimeSegments() > 0 && (feature_mask & RTC_FEATURE_FLAG_MOTION_BLUR))
        {
          Curve curve;
          if (basis == Geometry::GTY_BASIS_HERMITE && (feature_mask & RTC_FEATURE_FLAG_NORMAL_ORIENTED_HERMITE_CURVE)) {
            curve = geom->getNormalOrientedHermiteCurveSafe<HermiteCurveT<Vec3ff>, HermiteCurveT<Vec3fa>, Curve>(context,ray.org,primID,ray.time());
          }
          else if (basis == Geometry::GTY_BASIS_BSPLINE && (feature_mask & RTC_FEATURE_FLAG_NORMAL_ORIENTED_BSPLINE_CURVE)) {
            curve = geom->getNormalOrientedCurveSafe<BSplineCurveT<Vec3ff>, BSplineCurveT<Vec3fa>, Curve>(context,ray.org,primID,ray.time());
          }
          else if (basis == Geometry::GTY_BASIS_CATMULL_ROM && (feature_mask & RTC_FEATURE_FLAG_NORMAL_ORIENTED_CATMULL_ROM_CURVE)) {
            curve = geom->getNormalOrientedCurveSafe<CatmullRomCurveT<Vec3ff>, CatmullRomCurveT<Vec3fa>, Curve>(context,ray.org,primID,ray.time());
          }
          else {
            curve = geom->getNormalOrientedCurveSafe<Intersector::SourceCurve3ff, Intersector::SourceCurve3fa, Curve>(context,ray.org,primID,ray.time());
          }
          return Intersector().intersect(pre,ray,context,geom,primID,curve,Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
        }
        else
        {
          Vec3ff v0,v1,v2,v3;
          Vec3fa n0,n1,n2,n3;
          if (basis == Geometry::GTY_BASIS_HERMITE && (feature_mask & RTC_FEATURE_FLAG_NORMAL_ORIENTED_HERMITE_CURVE))
            geom->gather_hermite_safe(v0,v1,n0,n1,v2,v3,n2,n3,geom->curve(primID),ray.time());
          else
            geom->gather_safe(v0,v1,v2,v3,n0,n1,n2,n3,geom->curve(primID),ray.time());
          isa::convert_to_bezier(gtype, v0,v1,v2,v3, n0,n1,n2,n3);
          return Intersector().intersect(pre,ray,context,geom,primID,v0,v1,v2,v3,n0,n1,n2,n3,Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
        }
      }
      else if (feature_mask & (RTC_FEATURE_FLAG_FLAT_CURVES | RTC_FEATURE_FLAG_ROUND_CURVES)) {
        Vec3ff v0,v1,v2,v3;
        if (basis == Geometry::GTY_BASIS_HERMITE && (feature_mask & (RTC_FEATURE_FLAG_ROUND_HERMITE_CURVE | RTC_FEATURE_FLAG_FLAT_HERMITE_CURVE)))
          geom->gather_hermite_safe(v0,v1,v2,v3,geom->curve(primID),ray.time());
        else
          geom->gather_safe(v0,v1,v2,v3,geom->curve(primID),ray.time());
        
        isa::convert_to_bezier(gtype, v0,v1,v2,v3);

        if (stype == Geometry::GTY_SUBTYPE_FLAT_CURVE && (feature_mask & RTC_FEATURE_FLAG_FLAT_CURVES))
        {
          isa::RibbonCurve1Intersector1<CubicBezierCurve,1> intersector;
          return intersector.intersect(pre,ray,context,geom,primID,v0,v1,v2,v3,Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
        }
        else if (stype == Geometry::GTY_SUBTYPE_ROUND_CURVE && (feature_mask & RTC_FEATURE_FLAG_ROUND_CURVES))
        {
          isa::SweepCurve1Intersector1<CubicBezierCurve> intersector;
          return intersector.intersect(pre,ray,context,geom,primID,v0,v1,v2,v3,Intersect1Epilog1_HWIF<Ray>(ray,context,geomID,primID,filter));
        }
        return false;
      }
      return false;
    }
  } else
    
#endif

#endif
  
  return false;
}

__forceinline bool invokeTriangleIntersectionFilter(intel_ray_query_t& query, Geometry* geom, uint32_t bvh_level, RayHit& ray, Hit& hit, sycl::private_ptr<RayQueryContext> context, const RTCFeatureFlags feature_mask)
{
#if defined(EMBREE_FILTER_FUNCTION)
  if (!(feature_mask & RTC_FEATURE_FLAG_FILTER_FUNCTION) || runIntersectionFilter1SYCL(geom, ray, context, hit))
#endif
  {
    intel_ray_query_commit_potential_hit_override (query, ray.tfar, float2(hit.u, hit.v));
    
    for (unsigned l = 0; l < RTC_MAX_INSTANCE_LEVEL_COUNT; ++l) {
      ray.instID[l] = hit.instID[l];
#if defined(EMBREE_GEOMETRY_INSTANCE_ARRAY)
      ray.instPrimID[l] = hit.instPrimID[l];
#endif
    }
  }
  return false;
}

__forceinline bool invokeTriangleIntersectionFilter(intel_ray_query_t& query, Geometry* geom, uint32_t bvh_level, Ray& ray, Hit& hit, sycl::private_ptr<RayQueryContext> context, const RTCFeatureFlags feature_mask)
{
  bool ishit = true;
#if defined(EMBREE_FILTER_FUNCTION)
  ishit = !(feature_mask & RTC_FEATURE_FLAG_FILTER_FUNCTION) || runIntersectionFilter1SYCL(geom, ray, context, hit);
  if (ishit)
#endif
  {
    intel_ray_query_commit_potential_hit_override (query, ray.tfar, float2(hit.u, hit.v));
  }
  return ishit;
}

__forceinline bool commit_potential_hit(intel_ray_query_t& query, RayHit& ray) {
  intel_ray_query_commit_potential_hit_override (query, ray.tfar, float2(ray.u, ray.v));
  return false;
}

__forceinline bool commit_potential_hit(intel_ray_query_t& query, Ray& ray) {
  intel_ray_query_commit_potential_hit_override (query, ray.tfar, float2(0.0f, 0.0f));
  return true;
}

template<typename Ray>
__forceinline void trav_loop(intel_ray_query_t& query, Ray& ray, Scene* scene, sycl::private_ptr<RayQueryContext> context, const RTCFeatureFlags feature_mask)
{
  Scenes scenes(scene);
  while (!intel_is_traversal_done(query))
  {
    intel_candidate_type_t candidate = intel_get_hit_candidate(query, intel_hit_type_potential_hit);

    const unsigned int bvh_level = intel_get_hit_bvh_level( query, intel_hit_type_potential_hit );
    const float3 org = intel_get_ray_origin   ( query, bvh_level );
    const float3 dir = intel_get_ray_direction( query, bvh_level );
    const float t = intel_get_hit_distance(query, intel_hit_type_potential_hit);
    const float2 uv = intel_get_hit_barycentrics (query, intel_hit_type_potential_hit);
    const unsigned int geomID = intel_get_hit_geometry_id(query, intel_hit_type_potential_hit);
    const unsigned int primID = intel_get_hit_primitive_id(query, intel_hit_type_potential_hit);

    ray.org = Vec3ff(org.x(), org.y(), org.z(), ray.tnear());
    ray.dir = Vec3ff(dir.x(), dir.y(), dir.z(), ray.time ());
    ray.tfar = intel_get_hit_distance(query, intel_hit_type_committed_hit);

#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
    context->user->instStackSize = bvh_level;
    Scene* scene = scenes.get(bvh_level);
#else
    const unsigned int instID = intel_get_hit_instance_id(query, intel_hit_type_potential_hit);

    /* assume software instancing mode by default (required for rtcForwardRay) */
    Scene* scene = scenes.get(bvh_level);

    /* if we are in hardware instancing mode and we need to read the scene from the instance */
    if (bvh_level > 0 && instID != RTC_INVALID_GEOMETRY_ID) {
      Instance* inst = scenes.get(0)->get<Instance>(instID);
      scene = (Scene*) inst->object;
      context->user->instID[0] = instID;
    }
    else if (bvh_level == 0)
      context->user->instID[0] = RTC_INVALID_GEOMETRY_ID;

#endif
    context->scene = scene;
    Geometry* geom = scene->get(geomID);

    /* perform ray masking */
#if defined(EMBREE_RAY_MASK)
    if (ray.mask & geom->mask)
#endif
    {
      if (candidate == intel_candidate_type_procedural)
      {
        if (intersect_primitive(query,ray,scenes,geom,context,geomID,primID,feature_mask))
          if (commit_potential_hit (query, ray))
            break; // shadow rays break at first hit

      }
      else // if (candidate == TRIANGLE)
      {
        ray.tfar = t;
        Vec3f Ng = intel_get_hit_triangle_normal(query, intel_hit_type_potential_hit);
        Hit hit(context->user,geomID,primID,Vec2f(uv.x(),uv.y()),Ng);
        if (invokeTriangleIntersectionFilter(query, geom, bvh_level, ray, hit, context, feature_mask))
          break; // shadow rays break at first hit
      }
    }

    intel_ray_query_start_traversal(query);
    intel_ray_query_sync(query);
  }
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcIntersectRTHW(sycl::global_ptr<RTCTraversableTy> htraversable, sycl::private_ptr<RTCRayQueryContext> ucontext, sycl::private_ptr<RTCRayHit> rayhit_i, sycl::private_ptr<RTCIntersectArguments> args)
{
#if 0
  Scene* scene = (Scene*) htraversable.get();

  RayQueryContext context(scene, ucontext, args);

  RayHit ray;
  ray.org = Vec3ff(rayhit_i->ray.org_x, rayhit_i->ray.org_y, rayhit_i->ray.org_z, rayhit_i->ray.tnear);
  ray.dir = Vec3ff(rayhit_i->ray.dir_x, rayhit_i->ray.dir_y, rayhit_i->ray.dir_z, rayhit_i->ray.time);
  ray.tfar = rayhit_i->ray.tfar;
  ray.mask = rayhit_i->ray.mask;
  ray.id = rayhit_i->ray.id;
  ray.flags = rayhit_i->ray.flags;
  ray.Ng = Vec3f(0,0,0);
  ray.u = 0;
  ray.v = 0;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  for (uint32_t l=0; l<RTC_MAX_INSTANCE_LEVEL_COUNT; l++) {
    ray.instID[l] = RTC_INVALID_GEOMETRY_ID;
  }
#else
  ray.instID[0] = RTC_INVALID_GEOMETRY_ID;
#endif
  
  intel_ray_desc_t raydesc;
  raydesc.origin = float3(ray.org.x, ray.org.y, ray.org.z);
  raydesc.direction = float3(ray.dir.x, ray.dir.y, ray.dir.z);
  raydesc.tmin = ray.tnear();
  raydesc.tmax = ray.tfar;
  raydesc.mask = mask32_to_mask8(ray.mask);
  raydesc.flags = intel_ray_flags_none;
  
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  raydesc.flags |= intel_ray_flags_force_non_opaque;
#endif

#if defined(EMBREE_BACKFACE_CULLING)
  raydesc.flags |= intel_ray_flags_cull_back_facing_triangles;
#endif

  if (context.enforceArgumentFilterFunction())
     raydesc.flags |= intel_ray_flags_force_non_opaque;

  uint32_t bvh_id = 0;
  if (args->feature_mask & RTC_FEATURE_FLAG_MOTION_BLUR) {
    float time = clamp(ray.time(),0.0f,1.0f);
    uint32_t numTimeSegments = scene->getMaxTimeSegments();
    bvh_id = (uint32_t) clamp(uint32_t(numTimeSegments*time), 0u, numTimeSegments-1);
  }

  intel_raytracing_acceleration_structure_t hwaccel_ptr = (intel_raytracing_acceleration_structure_t) scene->accelBuffer.getHWAccel(bvh_id);
  
  intel_ray_query_t query = intel_ray_query_init(raydesc, hwaccel_ptr);
  intel_ray_query_start_traversal(query);
  intel_ray_query_sync(query);
  
  if (args->feature_mask & TRAV_LOOP_FEATURES) {
    trav_loop(query,ray,scene,&context,args->feature_mask);
  }

  bool valid = intel_has_committed_hit(query);

  if (valid)
  {
    float t = intel_get_hit_distance(query, intel_hit_type_committed_hit);
    float2 uv = intel_get_hit_barycentrics (query, intel_hit_type_committed_hit);
    unsigned int geomID = intel_get_hit_geometry_id(query, intel_hit_type_committed_hit);

    unsigned int primID = ray.primID;
    if (intel_get_hit_candidate(query, intel_hit_type_committed_hit) == intel_candidate_type_triangle)
      primID = intel_get_hit_triangle_primitive_id(query, intel_hit_type_committed_hit);
      
    rayhit_i->ray.tfar = t;
    rayhit_i->hit.geomID = geomID;
    rayhit_i->hit.primID = primID;
    rayhit_i->hit.u = uv.x();
    rayhit_i->hit.v = uv.y();
    
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
    for (uint32_t l=0; l<RTC_MAX_INSTANCE_LEVEL_COUNT; l++) {
      rayhit_i->hit.instID[l] = ray.instID[l];
#if defined(EMBREE_GEOMETRY_INSTANCE_ARRAY)
      rayhit_i->hit.instPrimID[l] = ray.instPrimID[l];
#endif
    }
#else
    unsigned int bvh_level = intel_get_hit_bvh_level(query, intel_hit_type_committed_hit);
    unsigned int instID = intel_get_hit_instance_id(query, intel_hit_type_committed_hit);
    /* when rtcForwardRay was used then we are in software instancing mode */
    if (bvh_level > 0 && instID == RTC_INVALID_GEOMETRY_ID)
      instID = ray.instID[0];
    
    rayhit_i->hit.instID[0] = instID;
#if defined(EMBREE_GEOMETRY_INSTANCE_ARRAY)
    rayhit_i->hit.instPrimID[0] = ray.instPrimID[0];
  #endif
#endif

    /* calculate geometry normal for hardware accelerated triangles */
    if (intel_get_hit_candidate(query, intel_hit_type_committed_hit) == intel_candidate_type_triangle)
      ray.Ng = intel_get_hit_triangle_normal(query, intel_hit_type_committed_hit);

    rayhit_i->hit.Ng_x = ray.Ng.x;
    rayhit_i->hit.Ng_y = ray.Ng.y;
    rayhit_i->hit.Ng_z = ray.Ng.z;
  }
  else
  {
    rayhit_i->hit.geomID = -1;
  }
  
  intel_ray_query_abandon(query);
#endif
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcOccludedRTHW(sycl::global_ptr<RTCTraversableTy> htraversable, sycl::private_ptr<RTCRayQueryContext> ucontext, sycl::private_ptr<RTCRay> ray_i, sycl::private_ptr<RTCOccludedArguments> args)
{
#if 0
  Scene* scene = (Scene*) htraversable.get();
  
  RayQueryContext context(scene, ucontext, args);
  
  Ray ray;
  ray.org = Vec3ff(ray_i->org_x, ray_i->org_y, ray_i->org_z, ray_i->tnear);
  ray.dir = Vec3ff(ray_i->dir_x, ray_i->dir_y, ray_i->dir_z, ray_i->time);
  ray.tfar = ray_i->tfar;
  ray.mask = ray_i->mask;
  ray.id = ray_i->id;
  ray.flags = ray_i->flags;
  
  intel_ray_desc_t raydesc;
  raydesc.origin = float3(ray.org.x, ray.org.y, ray.org.z);
  raydesc.direction = float3(ray.dir.x, ray.dir.y, ray.dir.z);
  raydesc.tmin = ray.tnear();
  raydesc.tmax = ray.tfar;
  raydesc.mask = mask32_to_mask8(ray.mask);
  raydesc.flags = intel_ray_flags_accept_first_hit_and_end_search;
  
#if defined(EMBREE_BACKFACE_CULLING)
  raydesc.flags |= intel_ray_flags_cull_back_facing_triangles;
#endif

  if (context.enforceArgumentFilterFunction())
     raydesc.flags |= intel_ray_flags_force_non_opaque;

  uint32_t bvh_id = 0;
  if (args->feature_mask & RTC_FEATURE_FLAG_MOTION_BLUR) {
    float time = clamp(ray.time(),0.0f,1.0f);
    uint32_t numTimeSegments = scene->getMaxTimeSegments();
    bvh_id = (uint32_t) clamp(uint32_t(numTimeSegments*time), 0u, numTimeSegments-1);
  }

  intel_raytracing_acceleration_structure_t hwaccel_ptr = (intel_raytracing_acceleration_structure_t) scene->accelBuffer.getHWAccel(bvh_id);

  intel_ray_query_t query = intel_ray_query_init(raydesc, hwaccel_ptr);
  intel_ray_query_start_traversal(query);
  intel_ray_query_sync(query);

  if (args->feature_mask & TRAV_LOOP_FEATURES) {
    trav_loop(query,ray,scene,&context,args->feature_mask);
  }
  
  if (intel_has_committed_hit(query))
    ray_i->tfar = -INFINITY;

  intel_ray_query_abandon(query);
#endif
}

RTC_NAMESPACE_END;
