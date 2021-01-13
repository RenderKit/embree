// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.isph"
#include "../common/math/closest_point.isph"

struct TriangleMesh
{
  ALIGNED_STRUCT_(16)
  uniform Vertex* vertices;
  uniform Triangle* triangles;
  uniform unsigned int num_vertices;
  uniform unsigned int num_triangles;

  uniform RTCGeometry geom;
  uniform unsigned int geomID;
};

struct Instance
{
  ALIGNED_STRUCT_(16)
  RTCGeometry geometry;
  RTCScene object;
  uniform int userID;
  AffineSpace3f local2world;
  AffineSpace3f world2local;
  LinearSpace3f normal2world;
  Vec3f lower;
  Vec3f upper;
};

/* for animation */
uniform float g_last_time = 0.f;
uniform float g_animate_time = 0.f;
extern uniform bool g_animate;
extern uniform bool g_userDefinedInstancing;

/* scene data */
RTCScene g_scene0 = NULL; 
RTCScene g_sceneEmbreeInstance = NULL;
RTCScene g_sceneUserDefinedInstance = NULL;

// scene containing all instances. will point to g_sceneEmbreeInstance or
// g_sceneUserDefinedInstance depending on g_userDefinedInstancing
RTCScene g_scene = NULL;  

uniform Instance* uniform g_instance[3] = { NULL, NULL, NULL };
uniform RTCGeometry g_instanceEmbree[3] = { NULL, NULL, NULL };

uniform TriangleMesh g_triangle_meshes[4];

/* data for visualization */
struct VisualizationData
{
  RTCGeometry spheres_geom;
  RTCGeometry lines_geom;
  uniform unsigned int num_point_queries;
  uniform unsigned int spheres_geomID;
  uniform unsigned int lines_geomID;
  uniform Vec4f* uniform sphere_vertex_buffer;
  uniform Vec4f* uniform line_vertex_buffer;
  uniform unsigned int* uniform line_index_buffer;
};

void init_VisualizationData(uniform VisualizationData * uniform visData)
{
  visData->num_point_queries = 10;
  visData->spheres_geomID = 1111111;
  visData->lines_geomID = 1111112;
  visData->sphere_vertex_buffer = NULL;
  visData->line_vertex_buffer = NULL;
  visData->line_index_buffer = NULL;
}

uniform VisualizationData g_visDataEmbreeInstance;
uniform VisualizationData g_visDataUserDefinedInstance;

inline void pushInstanceId(uniform RTCIntersectContext* uniform ctx, uniform unsigned int id)
{
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  ctx->instID[ctx->instStackSize++] = id;
#else
  ctx->instID[0] = id;
#endif
}

inline void popInstanceId(uniform RTCIntersectContext* uniform ctx)
{
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  ctx->instID[--ctx->instStackSize] = RTC_INVALID_GEOMETRY_ID;
#else
  ctx->instID[0] = RTC_INVALID_GEOMETRY_ID;
#endif
}

inline void copyInstanceIdStack(uniform const RTCIntersectContext* uniform ctx, varying unsigned* uniform tgt)
{
  tgt[0] = ctx->instID[0];
#if (RTC_MAX_INSTANCE_LEVEL_COUNT > 1)
  for (unsigned l = 1; l < RTC_MAX_INSTANCE_LEVEL_COUNT && l < ctx->instStackSize; ++l)
    tgt[l] = ctx->instID[l];
#endif
}

// ======================================================================== //
//              everything needed for closest point query                   //
// ======================================================================== //
struct ClosestPointResult
{
  Vec3f p;
  unsigned int primID;
  unsigned int geomID;
};

inline uniform AffineSpace3f from_raw(uniform float* uniform raw)
{
  return make_AffineSpace3f(*((uniform AffineSpace3fa* uniform)((uniform float* uniform)raw)));
}

inline void to_raw(uniform float* uniform raw, uniform AffineSpace3f const& m)
{
  raw[ 0] = m.l.vx.x; raw[ 1] = m.l.vx.y; raw[ 2] = m.l.vx.z; raw[ 3] = 0.f;
  raw[ 4] = m.l.vy.x; raw[ 5] = m.l.vy.y; raw[ 6] = m.l.vy.z; raw[ 7] = 0.f;
  raw[ 8] = m.l.vz.x; raw[ 9] = m.l.vz.y; raw[10] = m.l.vz.z; raw[11] = 0.f;
  raw[12] = m.p.x;    raw[13] = m.p.y;    raw[14] = m.p.z;    raw[15] = 1.f;
}

unmasked bool closestPointFunc(RTCPointQueryFunctionArguments* uniform args)
{
  assert(args->userPtr);
  uniform const unsigned int geomID = args->geomID;
  uniform const unsigned int primID = args->primID;

  uniform RTCPointQueryContext* uniform context = args->context;
  uniform const unsigned int stackSize = args->context->instStackSize;
  uniform const unsigned int stackPtr = stackSize-1;

  uniform AffineSpace3f inst2world = stackSize > 0
    ? from_raw(context->inst2world[stackPtr])
    : make_AffineSpace3f_scale(make_Vec3f(1.f, 1.f, 1.f));

  // query position in world space
  uniform Vec3f q = make_Vec3f(args->query->x, args->query->y, args->query->z);
  
  /*
   * Get triangle information in local space
   */
  uniform TriangleMesh const& triangle_mesh = g_triangle_meshes[geomID];
  uniform Triangle const& t = triangle_mesh.triangles[primID];
  uniform Vertex const& V0 = triangle_mesh.vertices[t.v0];
  uniform Vertex const& V1 = triangle_mesh.vertices[t.v1];
  uniform Vertex const& V2 = triangle_mesh.vertices[t.v2];
  uniform Vec3f v0 = make_Vec3f(V0.x, V0.y, V0.z);
  uniform Vec3f v1 = make_Vec3f(V1.x, V1.y, V1.z);
  uniform Vec3f v2 = make_Vec3f(V2.x, V2.y, V2.z);

  /*
   * Bring query and primitive data in the same space if necessary.
   */
  if (stackSize > 0 && args->similarityScale > 0)
  {
    // Instance transform is a similarity transform, therefore we 
    // can comute distance insformation in instance space. Therefore,
    // transform query position into local instance space.
    uniform AffineSpace3f m = from_raw(context->world2inst[stackPtr]);
    q = xfmPoint(m, q);
  }
  else if (stackSize > 0)
  {
    // Instance transform is not a similarity tranform. We have to transform the
    // primitive data into world space and perform distance computations in
    // world space to ensure correctness.
    v0 = xfmPoint(inst2world, v0);
    v1 = xfmPoint(inst2world, v1);
    v2 = xfmPoint(inst2world, v2);
  }
  else {
    // Primitive is not instanced, therefore point query and primitive are
    // already in the same space.
  }

  /*
   * Determine distance to closest point on triangle (implemented in
   * common/math/closest_point.h), and transform in world space if necessary.
   */
  uniform Vec3f p = closestPointTriangle(q, v0, v1, v2);
  uniform float d = distance(q, p);
  if (args->similarityScale > 0)
    d = d / args->similarityScale;

  /*
   * Store result in userPtr and update the query radius if we found a point
   * closer to the query position. This is optional but allows for faster
   * traversal (due to better culling). 
   */
  if (d < args->query->radius)
  {
    args->query->radius = d;
    ClosestPointResult* result = (ClosestPointResult*)args->userPtr;
    result->p = args->similarityScale > 0 ? xfmPoint(inst2world, p) : p;
    result->primID = primID;
    result->geomID = geomID;
    return true; // Return true to indicate that the query radius changed.
  }

  return false;
}

inline void pushInstanceIdAndTransform(uniform RTCPointQueryContext* uniform context,
                                       uniform unsigned int id, 
                                       uniform AffineSpace3f const& w2i_in, 
                                       uniform AffineSpace3f const& i2w_in)
{
  context->instID[context->instStackSize] = id;

  // local copies of const references to fullfill alignment constraints
  uniform AffineSpace3f w2i = w2i_in;
  uniform AffineSpace3f i2w = i2w_in;

  const uniform unsigned int stackSize = context->instStackSize;
  if (stackSize > 0) {
    w2i = from_raw(context->world2inst[stackSize-1]) * w2i;
    i2w = i2w * from_raw(context->inst2world[stackSize-1]);
  }

  to_raw(context->world2inst[stackSize], w2i);
  to_raw(context->inst2world[stackSize], i2w);

  context->instStackSize++;
}

inline void popInstanceIdAndTransform(uniform RTCPointQueryContext* uniform context)
{
  context->instID[--context->instStackSize] = RTC_INVALID_GEOMETRY_ID;
}

unmasked bool instanceClosestPointFunc(RTCPointQueryFunctionArguments* uniform args)
{
  // convert geomID in the scene to instance idx (-4)
  uniform Instance* uniform instance = g_instance[args->geomID - 4];
  
  pushInstanceIdAndTransform(args->context, instance->userID, instance->world2local, instance->local2world);
  bool changed = rtcPointQuery(instance->object, args->query, args->context, 0, args->userPtr);
  popInstanceIdAndTransform(args->context);
  return changed;
}

// ======================================================================== //
//                         User defined instancing                          //
// ======================================================================== //

unmasked void instanceBoundsFunc(const struct RTCBoundsFunctionArguments* uniform args)
{
  const uniform Instance* uniform instance = (const uniform Instance* uniform) args->geometryUserPtr;
  uniform RTCBounds* uniform bounds_o = args->bounds_o;
  uniform Vec3f l = instance->lower;
  uniform Vec3f u = instance->upper;
  uniform Vec3f p000 = xfmPoint(instance->local2world,make_Vec3f(l.x,l.y,l.z));
  uniform Vec3f p001 = xfmPoint(instance->local2world,make_Vec3f(l.x,l.y,u.z));
  uniform Vec3f p010 = xfmPoint(instance->local2world,make_Vec3f(l.x,u.y,l.z));
  uniform Vec3f p011 = xfmPoint(instance->local2world,make_Vec3f(l.x,u.y,u.z));
  uniform Vec3f p100 = xfmPoint(instance->local2world,make_Vec3f(u.x,l.y,l.z));
  uniform Vec3f p101 = xfmPoint(instance->local2world,make_Vec3f(u.x,l.y,u.z));
  uniform Vec3f p110 = xfmPoint(instance->local2world,make_Vec3f(u.x,u.y,l.z));
  uniform Vec3f p111 = xfmPoint(instance->local2world,make_Vec3f(u.x,u.y,u.z));
  uniform Vec3f lower = min(min(min(p000,p001),min(p010,p011)),min(min(p100,p101),min(p110,p111)));
  uniform Vec3f upper = max(max(max(p000,p001),max(p010,p011)),max(max(p100,p101),max(p110,p111)));
  bounds_o->lower_x = lower.x;
  bounds_o->lower_y = lower.y;
  bounds_o->lower_z = lower.z;
  bounds_o->upper_x = upper.x;
  bounds_o->upper_y = upper.y;
  bounds_o->upper_z = upper.z;
}

unmasked void instanceIntersectFunc(const RTCIntersectFunctionNArguments* uniform args)
{
  
  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  uniform RTCIntersectContext* uniform context = args->context;
  RTCRayHitN* uniform rays = (RTCRayHitN* uniform)args->rayhit;
                                    
  assert(args->N == programCount);
  if (!valid[programIndex])
    return;
  
  varying Ray *uniform ray = (varying Ray*uniform)rays;
  const uniform Instance* uniform instance = (const uniform Instance* uniform)ptr;
  const Vec3f ray_org = ray->org;
  const Vec3f ray_dir = ray->dir;
  const float ray_tnear = ray->tnear;
  const float ray_tfar  = ray->tfar;
  ray->org = xfmPoint (instance->world2local,ray_org);
  ray->dir = xfmVector(instance->world2local,ray_dir);
  ray->tnear = ray_tnear;
  ray->tfar  = ray_tfar;
  pushInstanceId(context, instance->userID);
  rtcIntersectV(instance->object,context,RTCRayHit_(*ray));
  popInstanceId(context);
  const float updated_tfar = ray->tfar;
  ray->org = ray_org;
  ray->dir = ray_dir;
  ray->tfar = updated_tfar;
}

uniform Instance* uniform createInstance (RTCScene scene, RTCScene object, uniform int userID, const uniform Vec3f& lower, const uniform Vec3f& upper)
{
  uniform Instance* uniform instance = uniform new uniform Instance;
  instance->object = object;
  instance->userID = userID;
  instance->lower = lower;
  instance->upper = upper;
  instance->local2world.l.vx = make_Vec3f(1,0,0);
  instance->local2world.l.vy = make_Vec3f(0,1,0);
  instance->local2world.l.vz = make_Vec3f(0,0,1);
  instance->local2world.p    = make_Vec3f(0,0,0);
  instance->geometry = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  rtcSetGeometryUserPrimitiveCount(instance->geometry,1);
  rtcSetGeometryUserData(instance->geometry,instance);
  rtcSetGeometryBoundsFunction(instance->geometry,instanceBoundsFunc,NULL);
  rtcSetGeometryIntersectFunction(instance->geometry,instanceIntersectFunc);
  rtcSetGeometryPointQueryFunction(instance->geometry, instanceClosestPointFunc);
  rtcCommitGeometry(instance->geometry);
  rtcAttachGeometry(scene,instance->geometry);
  rtcReleaseGeometry(instance->geometry);
  return instance;
}

void updateInstance (RTCScene scene, uniform Instance* uniform instance)
{
  instance->world2local = rcp(instance->local2world);
  instance->normal2world = transposed(rcp(instance->local2world.l));
  rtcCommitGeometry(instance->geometry);
}

// ======================================================================== //
//                      triangle mesh geometry                              //
// ======================================================================== //

uniform TriangleMesh createTriangulatedSphere (const uniform Vec3f& p, uniform float r, uniform unsigned int geomID)
{
  const uniform int numPhi = 10;
  const uniform int numTheta = 4*numPhi;

  /* create triangle mesh */
  uniform TriangleMesh mesh;
  mesh.geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  mesh.geomID = geomID;
  mesh.num_vertices = numTheta*(numPhi+1);
  mesh.num_triangles = 2*numTheta*(numPhi-1);
  mesh.vertices  = (uniform Vertex* uniform)   rtcSetNewGeometryBuffer(mesh.geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vertex),   mesh.num_vertices);
  mesh.triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(mesh.geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT3,  sizeof(uniform Triangle),mesh.num_triangles);

  /* create sphere */
  uniform int tri = 0;
  const uniform float rcpNumTheta = rcp((uniform float)numTheta);
  const uniform float rcpNumPhi   = rcp((uniform float)numPhi);
  for (uniform int phi=0; phi<=numPhi; phi++)
  {
    for (uniform int theta=0; theta<numTheta; theta++)
    {
      const uniform float phif   = phi*pi*rcpNumPhi;
      const uniform float thetaf = theta*2.0f*pi*rcpNumTheta;

      uniform Vertex& v = mesh.vertices[phi*numTheta+theta];
      v.x = p.x + r*sin(phif)*sin(thetaf);
      v.y = p.y + r*cos(phif);
      v.z = p.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (uniform int theta=1; theta<=numTheta; theta++)
    {
      uniform int p00 = (phi-1)*numTheta+theta-1;
      uniform int p01 = (phi-1)*numTheta+theta%numTheta;
      uniform int p10 = phi*numTheta+theta-1;
      uniform int p11 = phi*numTheta+theta%numTheta;

      if (phi > 1) {
        mesh.triangles[tri].v0 = p10;
        mesh.triangles[tri].v1 = p00;
        mesh.triangles[tri].v2 = p01;
        tri++;
      }

      if (phi < numPhi) {
        mesh.triangles[tri].v0 = p11;
        mesh.triangles[tri].v1 = p10;
        mesh.triangles[tri].v2 = p01;
        tri++;
      }
    }
  }
  return mesh;
}

uniform TriangleMesh createCube (uniform Vec3f const& p, uniform float r, uniform unsigned int geomID)
{
  uniform TriangleMesh mesh;
  mesh.geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  mesh.geomID = geomID;
  mesh.num_vertices  = 8;
  mesh.num_triangles = 12;
  mesh.vertices  = (uniform Vertex* uniform)  rtcSetNewGeometryBuffer(mesh.geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vertex),   mesh.num_vertices);
  mesh.triangles = (uniform Triangle* uniform)rtcSetNewGeometryBuffer(mesh.geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT3,  sizeof(uniform Triangle), mesh.num_triangles);

  /* set vertices and vertex colors */
  mesh.vertices[0].x = -r+p.x; mesh.vertices[0].y = -r+p.y; mesh.vertices[0].z = -r+p.z;
  mesh.vertices[1].x = -r+p.x; mesh.vertices[1].y = -r+p.y; mesh.vertices[1].z = +r+p.z;
  mesh.vertices[2].x = -r+p.x; mesh.vertices[2].y = +r+p.y; mesh.vertices[2].z = -r+p.z;
  mesh.vertices[3].x = -r+p.x; mesh.vertices[3].y = +r+p.y; mesh.vertices[3].z = +r+p.z;
  mesh.vertices[4].x = +r+p.x; mesh.vertices[4].y = -r+p.y; mesh.vertices[4].z = -r+p.z;
  mesh.vertices[5].x = +r+p.x; mesh.vertices[5].y = -r+p.y; mesh.vertices[5].z = +r+p.z;
  mesh.vertices[6].x = +r+p.x; mesh.vertices[6].y = +r+p.y; mesh.vertices[6].z = -r+p.z;
  mesh.vertices[7].x = +r+p.x; mesh.vertices[7].y = +r+p.y; mesh.vertices[7].z = +r+p.z;

  /* set triangles and face colors */
  uniform int tri = 0;

  // left side
  mesh.triangles[tri].v0 = 0; mesh.triangles[tri].v1 = 1; mesh.triangles[tri].v2 = 2; tri++;
  mesh.triangles[tri].v0 = 1; mesh.triangles[tri].v1 = 3; mesh.triangles[tri].v2 = 2; tri++;

  // right side
  mesh.triangles[tri].v0 = 4; mesh.triangles[tri].v1 = 6; mesh.triangles[tri].v2 = 5; tri++;
  mesh.triangles[tri].v0 = 5; mesh.triangles[tri].v1 = 6; mesh.triangles[tri].v2 = 7; tri++;

  // bottom side
  mesh.triangles[tri].v0 = 0; mesh.triangles[tri].v1 = 4; mesh.triangles[tri].v2 = 1; tri++;
  mesh.triangles[tri].v0 = 1; mesh.triangles[tri].v1 = 4; mesh.triangles[tri].v2 = 5; tri++;

  // top side
  mesh.triangles[tri].v0 = 2; mesh.triangles[tri].v1 = 3; mesh.triangles[tri].v2 = 6; tri++;
  mesh.triangles[tri].v0 = 3; mesh.triangles[tri].v1 = 7; mesh.triangles[tri].v2 = 6; tri++;

  // front side
  mesh.triangles[tri].v0 = 0; mesh.triangles[tri].v1 = 2; mesh.triangles[tri].v2 = 4; tri++;
  mesh.triangles[tri].v0 = 2; mesh.triangles[tri].v1 = 6; mesh.triangles[tri].v2 = 4; tri++;

  // back side
  mesh.triangles[tri].v0 = 1; mesh.triangles[tri].v1 = 5; mesh.triangles[tri].v2 = 3; tri++;
  mesh.triangles[tri].v0 = 3; mesh.triangles[tri].v1 = 5; mesh.triangles[tri].v2 = 7; tri++;

  return mesh;
}

uniform TriangleMesh createPlane (uniform AffineSpace3f M, uniform unsigned int R, uniform unsigned int geomID)
{
  uniform TriangleMesh mesh;
  mesh.geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  mesh.geomID = geomID;
  mesh.num_vertices  = (R+1)*(R+1);
  mesh.num_triangles = 2*R*R;
  mesh.vertices  = (uniform Vertex* uniform)  rtcSetNewGeometryBuffer(mesh.geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vertex),   mesh.num_vertices);
  mesh.triangles = (uniform Triangle* uniform)rtcSetNewGeometryBuffer(mesh.geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT3,  sizeof(uniform Triangle), mesh.num_triangles);

  /* set vertices and vertex colors */
  for (uniform unsigned int y = 0; y <= R; ++y)
  for (uniform unsigned int x = 0; x <= R; ++x)
  {
    uniform Vec3f p = make_Vec3f((float)x/R, (float)y/R, 0.f);
    uniform Vec3f pt = xfmPoint(M, p);
    mesh.vertices[y*(R+1)+x].x = pt.x;
    mesh.vertices[y*(R+1)+x].y = pt.y;
    mesh.vertices[y*(R+1)+x].z = pt.z;
  }

  /* set triangles and face colors */
  for (uniform unsigned int j = 0; j < R; ++j)
  for (uniform unsigned int i = 0; i < R; ++i)
  {
    mesh.triangles[2*(j*R+i)+0].v0 = (j*(R+1)+i);
    mesh.triangles[2*(j*R+i)+0].v1 = (j*(R+1)+i) + (R + 1) + 1;
    mesh.triangles[2*(j*R+i)+0].v2 = (j*(R+1)+i) + (R + 1);
    mesh.triangles[2*(j*R+i)+1].v0 = (j*(R+1)+i);
    mesh.triangles[2*(j*R+i)+1].v1 = (j*(R+1)+i) + 1;
    mesh.triangles[2*(j*R+i)+1].v2 = (j*(R+1)+i) + (R + 1) + 1;
  }
  return mesh;
}

void addTriangleMeshToScene(uniform RTCScene scene, uniform TriangleMesh const& mesh)
{
  uniform RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, (const void* uniform)mesh.vertices,  0, sizeof(uniform Vertex),   mesh.num_vertices);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT3,  (const void* uniform)mesh.triangles, 0, sizeof(uniform Triangle), mesh.num_triangles);
  rtcSetGeometryPointQueryFunction(geom, closestPointFunc);
  rtcCommitGeometry(geom);
  rtcAttachGeometryByID(scene, geom, mesh.geomID);
  rtcReleaseGeometry(geom);
}

void addVisualizationSpheres(uniform RTCScene scene, uniform VisualizationData * uniform visData)
{
  visData->spheres_geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SPHERE_POINT);
  visData->sphere_vertex_buffer = (Vec4f*)rtcSetNewGeometryBuffer(visData->spheres_geom, RTC_BUFFER_TYPE_VERTEX, 0, 
                                                                 RTC_FORMAT_FLOAT4, sizeof(uniform Vec4f), 
                                                                 2*visData->num_point_queries);
  visData->sphere_vertex_buffer[ 0] = make_Vec4f( 0.00f, -0.50f,  -0.25f, 0.2f);
  visData->sphere_vertex_buffer[ 2] = make_Vec4f(-8.25f, -0.50f,  -1.25f, 0.2f); 
  visData->sphere_vertex_buffer[ 4] = make_Vec4f(-8.00f, -2.00f,  -7.75f, 0.2f); 
  visData->sphere_vertex_buffer[ 6] = make_Vec4f(-0.50f,  1.75f,  -7.25f, 0.2f); 
  visData->sphere_vertex_buffer[ 8] = make_Vec4f( 0.00f,  1.75f, -13.00f, 0.2f); 
  visData->sphere_vertex_buffer[10] = make_Vec4f( 6.75f,  1.00f, -12.25f, 0.2f); 
  visData->sphere_vertex_buffer[12] = make_Vec4f( 5.75f,  1.00f, -12.25f, 0.2f); 
  visData->sphere_vertex_buffer[14] = make_Vec4f( 5.50f,  0.50f,  -6.50f, 0.2f); 
  visData->sphere_vertex_buffer[16] = make_Vec4f( 7.25f, -3.00f,  -1.00f, 0.2f); 
  visData->sphere_vertex_buffer[18] = make_Vec4f(-0.25f, -0.50f,  -4.25f, 0.2f); 
  for (uniform int i = 0; i < visData->num_point_queries; ++i) 
    visData->sphere_vertex_buffer[2*i+1] = make_Vec4f(0.f);

  rtcCommitGeometry(visData->spheres_geom);
  rtcAttachGeometryByID(scene, visData->spheres_geom, visData->spheres_geomID);
  rtcReleaseGeometry(visData->spheres_geom);
      
  visData->lines_geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
  visData->line_vertex_buffer = (Vec4f*)rtcSetNewGeometryBuffer(visData->lines_geom, RTC_BUFFER_TYPE_VERTEX, 0, 
                                                               RTC_FORMAT_FLOAT4, sizeof(uniform Vec4f), 
                                                               2*visData->num_point_queries); 
  visData->line_index_buffer = (unsigned int*)rtcSetNewGeometryBuffer(visData->lines_geom, RTC_BUFFER_TYPE_INDEX, 0, 
                                                                     RTC_FORMAT_UINT, sizeof(uniform unsigned int), 
                                                                     visData->num_point_queries);
  for (uniform int i = 0; i < 2*visData->num_point_queries; ++i)
    visData->line_vertex_buffer[i] = make_Vec4f(make_Vec3f(visData->sphere_vertex_buffer[i]), 0.05f);
  for (uniform int i = 0; i < visData->num_point_queries; ++i) 
    visData->line_index_buffer[i] = 2*i;

  rtcCommitGeometry(visData->lines_geom);
  rtcAttachGeometryByID(scene, visData->lines_geom, visData->lines_geomID);
  rtcReleaseGeometry(visData->lines_geom);
  rtcCommitScene(scene);
}

void updateGeometryAndQueries(uniform float time)
{
  uniform const float delta_time = time - g_last_time;
  if (g_animate) {
    g_animate_time += delta_time;
  }
  g_last_time = time;

  g_instance[0]->local2world 
    = make_AffineSpace3f_translate(make_Vec3f(7.5f, 0.f, -8.f))
    * make_AffineSpace3f_rotate(make_Vec3f(0.f), make_Vec3f(0.f, 1.f, 0.f), M_PI/2.f)
    * make_AffineSpace3f_rotate(make_Vec3f(0.f), make_Vec3f(1.f, 0.f, 0.f), 0.2f * sin(g_animate_time));

  g_instance[1]->local2world 
    = make_AffineSpace3f_translate(make_Vec3f(0.f, 3.f + 1.5f*sin(g_animate_time), -9.f)) 
    * make_AffineSpace3f_scale(make_Vec3f(1.f, 2.f, 3.f))
    * make_AffineSpace3f_rotate(make_Vec3f(0.f), make_Vec3f(0.f, 1.f, 0.f), M_PI);
  
  uniform AffineSpace3f sheer = make_AffineSpace3f_scale(make_Vec3f(1.2f));
  sheer.l.vz.x = cos(g_animate_time*0.75) * 1.0f + 0.5f;
  g_instance[2]->local2world 
    = make_AffineSpace3f_translate(make_Vec3f(-8.5f, 0.f, -7.f)) 
    * make_AffineSpace3f_rotate(make_Vec3f(0.f), make_Vec3f(0.f, 1.f, 0.f), -M_PI/2.f)
    * sheer;

  /* update scene */
  updateInstance(g_sceneUserDefinedInstance, g_instance[0]);
  updateInstance(g_sceneUserDefinedInstance, g_instance[1]);
  updateInstance(g_sceneUserDefinedInstance, g_instance[2]);

  /* set instance transformations */
  rtcSetGeometryTransform(g_instanceEmbree[0], 0, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR, (float*)&g_instance[0]->local2world);
  rtcSetGeometryTransform(g_instanceEmbree[1], 0, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR, (float*)&g_instance[1]->local2world);
  rtcSetGeometryTransform(g_instanceEmbree[2], 0, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR, (float*)&g_instance[2]->local2world);

  rtcCommitGeometry(g_instanceEmbree[0]);
  rtcCommitGeometry(g_instanceEmbree[1]);
  rtcCommitGeometry(g_instanceEmbree[2]);

  rtcCommitScene(g_sceneEmbreeInstance);
  rtcCommitScene(g_sceneUserDefinedInstance);

  g_scene = g_userDefinedInstancing ? g_sceneUserDefinedInstance : g_sceneEmbreeInstance;
  uniform VisualizationData * uniform visData = g_userDefinedInstancing ? &g_visDataUserDefinedInstance : &g_visDataEmbreeInstance;

  for (int i = 0; i < visData->num_point_queries; ++i)
  {
    RTCPointQuery query;
    query.x = visData->sphere_vertex_buffer[2*i+0].x; 
    query.y = visData->sphere_vertex_buffer[2*i+0].y; 
    query.z = visData->sphere_vertex_buffer[2*i+0].z; 
    query.radius = inf;
    query.time = 0.f;
    
    uniform ClosestPointResult result[programCount];
    result[programIndex].primID = RTC_INVALID_GEOMETRY_ID;
    result[programIndex].geomID = RTC_INVALID_GEOMETRY_ID;
    void* userPtr = (void*)&result[programIndex];
    
    uniform RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQueryV(g_scene,
                  (varying RTCPointQuery* uniform)&query,
                  &context,
                  NULL,
                  &userPtr);

    unsigned int primID = result[programIndex].primID;
    unsigned int geomID = result[programIndex].geomID;
    Vec3f p = result[programIndex].p;
    
    assert(primID != RTC_INVALID_GEOMETRY_ID || geomID != RTC_INVALID_GEOMETRY_ID);
    visData->sphere_vertex_buffer[2*i+1].x = p.x;
    visData->sphere_vertex_buffer[2*i+1].y = p.y;
    visData->sphere_vertex_buffer[2*i+1].z = p.z;
    visData->sphere_vertex_buffer[2*i+1].w = 0.2f;
    visData->line_vertex_buffer[2*i+1].x = p.x;
    visData->line_vertex_buffer[2*i+1].y = p.y;
    visData->line_vertex_buffer[2*i+1].z = p.z;
  }

  rtcCommitScene(g_scene);
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  init_VisualizationData(&g_visDataEmbreeInstance);
  init_VisualizationData(&g_visDataUserDefinedInstance);

  /* create scene data */
  g_triangle_meshes[0] = createPlane(
    make_AffineSpace3f_translate(make_Vec3f(0.f, -3.f, 0.f)) *
    make_AffineSpace3f_scale(make_Vec3f(10.f, 4.f, 4.f)) * 
    make_AffineSpace3f_rotate(make_Vec3f(0.f), make_Vec3f(1.f, 0.f, 0.f), M_PI/2) *
    make_AffineSpace3f_translate(make_Vec3f(-0.5f, -0.5f, 0.f)),
    1, 0);
  g_triangle_meshes[1] = createPlane(
    make_AffineSpace3f_translate(make_Vec3f(0.f, -1.f, 2.f)) *
    make_AffineSpace3f_scale(make_Vec3f(10.f, 4.f, 4.f)) * 
    make_AffineSpace3f_translate(make_Vec3f(-0.5f, -0.5f, 0.f)),
    8, 1);
  g_triangle_meshes[2] = createTriangulatedSphere(make_Vec3f(3.f, -2.f, 0.0f), 1.f, 2);
  g_triangle_meshes[3] = createCube(make_Vec3f(-3.f, -2.f, 0), 1.0f, 3);
  
  /* compute bounding box of the scene that will be instanced */
  uniform Vec3f bbmin = make_Vec3f(inf);
  uniform Vec3f bbmax = make_Vec3f(neg_inf);
  for (uniform int i = 0; i < 4; ++i) {
    uniform TriangleMesh const& mesh = g_triangle_meshes[i];
    for (uniform int v = 0; v < mesh.num_vertices; ++v) {
      uniform Vertex const& vert = mesh.vertices[v];
      bbmin = min(bbmin, make_Vec3f(vert.x, vert.y, vert.z));
      bbmax = max(bbmax, make_Vec3f(vert.x, vert.y, vert.z));
    }
  }

  g_sceneEmbreeInstance = rtcNewScene(g_device);
  g_sceneUserDefinedInstance = rtcNewScene(g_device);
  g_scene0 = rtcNewScene(g_device);

  for (uniform int i = 0; i < 4; ++i)
  {
    addTriangleMeshToScene(g_scene0, g_triangle_meshes[i]);
    addTriangleMeshToScene(g_sceneEmbreeInstance, g_triangle_meshes[i]);
    addTriangleMeshToScene(g_sceneUserDefinedInstance, g_triangle_meshes[i]);
  }
  rtcCommitScene(g_scene0);

  /* instantiate geometry */
  for (uniform int i = 0; i < 3; ++i)
  {
    g_instance[i] = createInstance(g_sceneUserDefinedInstance, g_scene0, i, bbmin, bbmax);
    g_instanceEmbree[i] = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
    rtcSetGeometryInstancedScene(g_instanceEmbree[i], g_scene0);
    rtcSetGeometryTimeStepCount(g_instanceEmbree[i], 1);
    rtcAttachGeometryByID(g_sceneEmbreeInstance, g_instanceEmbree[i], 4+i);
    rtcReleaseGeometry(g_instanceEmbree[i]);
    rtcCommitGeometry(g_instanceEmbree[i]);
  }

  addVisualizationSpheres(g_sceneEmbreeInstance, &g_visDataEmbreeInstance);
  addVisualizationSpheres(g_sceneUserDefinedInstance, &g_visDataUserDefinedInstance);

  updateGeometryAndQueries(0.f);
}

inline Vec3f face_forward(const Vec3f& dir, const Vec3f& _Ng) {
  const Vec3f Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), 
                     make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 
                     0.0f, inf, 0.0f, -1,
                     RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);

  /* intersect ray with scene */
  rtcIntersectV(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  uniform VisualizationData * uniform visData = g_userDefinedInstancing 
                                              ? &g_visDataUserDefinedInstance 
                                              : &g_visDataEmbreeInstance;
  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    if (ray.geomID == visData->spheres_geomID)
    {
      if (ray.primID % 2 == 0) return make_Vec3f(0.8f, 0.2f, 0.2f);
      else                     return make_Vec3f(1.0f, 1.0f, 1.0f);
    }
    if (ray.geomID == visData->lines_geomID)
    {
      return make_Vec3f(0.7f, 0.3f, 0.7f);
    }

    /* calculate shading normal in world space */
    Vec3f Ns = ray.Ng;
    if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID)
    {
      if (g_userDefinedInstancing)
        Ns = xfmVector(g_instance[ray.instID[0]]->normal2world, make_Vec3f(Ns));
      else
        // convert geomID (ray.instID) in the scene to instance idx (-4)
        Ns = xfmVector(g_instance[ray.instID[0]-4]->normal2world, make_Vec3f(Ns));
    }

    Ns = face_forward(ray.dir,normalize(Ns));
    color = 0.5f * Ns + make_Vec3f(0.5f);
  }
  return color;
}

/* renders a single screen tile */
void renderTileStandard(uniform int taskIndex,
                        uniform int threadIndex,
                        uniform int* uniform pixels,
                        const uniform unsigned int width,
                        const uniform unsigned int height,
                        const uniform float time,
                        const uniform ISPCCamera& camera,
                        const uniform int numTilesX,
                        const uniform int numTilesY)
{
  const uniform unsigned int tileY = taskIndex / numTilesX;
  const uniform unsigned int tileX = taskIndex - tileY * numTilesX;
  const uniform unsigned int x0 = tileX * TILE_SIZE_X;
  const uniform unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const uniform unsigned int y0 = tileY * TILE_SIZE_Y;
  const uniform unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    if (all(__mask == 0)) continue;

    /* calculate pixel color */
    Vec3f color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
task void renderTileTask(uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const uniform float time,
                         const uniform ISPCCamera& camera,
                         const uniform int numTilesX,
                         const uniform int numTilesY)
{
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
{
  /* render all pixels */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  updateGeometryAndQueries(time);
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  for (uniform int i = 0; i < 4; ++i)
    rtcReleaseGeometry(g_triangle_meshes[i].geom);
  rtcReleaseScene (g_scene0); g_scene0 = NULL;
  rtcReleaseScene (g_sceneEmbreeInstance); g_scene0 = NULL;
  rtcReleaseScene (g_sceneUserDefinedInstance); g_scene0 = NULL;
  rtcReleaseDevice(g_device); g_device = NULL;
}
