// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/closest_point.h"

namespace embree {

struct TriangleMesh;
struct Instance;

/* for animation */
float g_last_time = 0.f;
float g_animate_time = 0.f;
extern "C" bool g_animate;
extern "C" bool g_userDefinedInstancing;

/* scene data */
RTCScene g_scene1 = nullptr; 
RTCScene g_sceneEmbreeInstance = nullptr;
RTCScene g_sceneUserDefinedInstance = nullptr;

// scene containing all instances. will point to g_sceneEmbreeInstance or
// g_sceneUserDefinedInstance depending on g_userDefinedInstancing
RTCScene g_scene = nullptr;  

RTCGeometry g_instanceEmbree[3]    = { nullptr, nullptr, nullptr };
Instance* g_instanceUserDefined[3] = { nullptr, nullptr, nullptr };

TriangleMesh* g_triangle_meshes[4] = { 
  nullptr, nullptr, nullptr, nullptr
};

AffineSpace3fa g_instance_xfm[3];
LinearSpace3fa g_normal_xfm[3];

/* data for visualization */
const unsigned int g_num_point_queries = 10;
Vec3f g_sphere_locations[2*g_num_point_queries] = {
  Vec3f( 0.00f, -0.50f,  -0.25f), Vec3f(0.0f),
  Vec3f(-8.25f, -0.50f,  -1.25f), Vec3f(0.0f),
  Vec3f(-8.00f, -2.00f,  -7.75f), Vec3f(0.0f),
  Vec3f(-0.50f,  1.75f,  -7.25f), Vec3f(0.0f),
  Vec3f( 0.00f,  1.75f, -13.00f), Vec3f(0.0f),
  Vec3f( 6.75f,  1.00f, -12.25f), Vec3f(0.0f),
  Vec3f( 5.75f,  1.00f, -12.25f), Vec3f(0.0f),
  Vec3f( 5.50f,  0.50f,  -6.50f), Vec3f(0.0f),
  Vec3f( 7.25f, -3.00f,  -1.00f), Vec3f(0.0f),
  Vec3f(-0.25f, -0.50f,  -4.25f), Vec3f(0.0f),
}; // consecutive pairs of (query point, closest point)
RTCGeometry g_spheres = nullptr;
RTCGeometry g_lines = nullptr;
unsigned int g_spheres_geomID = 111111;
unsigned int g_lines_geomID = 111112;

Vec4f g_sphere_vertex_buffer[2*g_num_point_queries];
Vec4f g_line_vertex_buffer[2*g_num_point_queries];
unsigned int g_line_index_buffer[g_num_point_queries] = {
  0, 2, 4, 6, 8, 10, 12, 14, 16, 18
};

// ======================================================================== //
//                         User defined instancing                          //
// ======================================================================== //
struct Instance
{
  ALIGNED_STRUCT_(16)
  RTCGeometry geometry;
  RTCScene object;
  int userID;
  AffineSpace3fa local2world;
  AffineSpace3fa world2local;
  LinearSpace3fa normal2world;
  Vec3fa lower;
  Vec3fa upper;
};

void instanceBoundsFunc(const struct RTCBoundsFunctionArguments* args)
{
  const Instance* instance = (const Instance*) args->geometryUserPtr;
  RTCBounds* bounds_o = args->bounds_o;
  Vec3fa l = instance->lower;
  Vec3fa u = instance->upper;
  Vec3fa p000 = xfmPoint(instance->local2world,Vec3fa(l.x,l.y,l.z));
  Vec3fa p001 = xfmPoint(instance->local2world,Vec3fa(l.x,l.y,u.z));
  Vec3fa p010 = xfmPoint(instance->local2world,Vec3fa(l.x,u.y,l.z));
  Vec3fa p011 = xfmPoint(instance->local2world,Vec3fa(l.x,u.y,u.z));
  Vec3fa p100 = xfmPoint(instance->local2world,Vec3fa(u.x,l.y,l.z));
  Vec3fa p101 = xfmPoint(instance->local2world,Vec3fa(u.x,l.y,u.z));
  Vec3fa p110 = xfmPoint(instance->local2world,Vec3fa(u.x,u.y,l.z));
  Vec3fa p111 = xfmPoint(instance->local2world,Vec3fa(u.x,u.y,u.z));
  Vec3fa lower = min(min(min(p000,p001),min(p010,p011)),min(min(p100,p101),min(p110,p111)));
  Vec3fa upper = max(max(max(p000,p001),max(p010,p011)),max(max(p100,p101),max(p110,p111)));
  bounds_o->lower_x = lower.x;
  bounds_o->lower_y = lower.y;
  bounds_o->lower_z = lower.z;
  bounds_o->upper_x = upper.x;
  bounds_o->upper_y = upper.y;
  bounds_o->upper_z = upper.z;
}

inline void pushInstanceId(RTCIntersectContext* ctx, unsigned int id)
{
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  ctx->instID[ctx->instStackSize++] = id;
#else
  ctx->instID[0] = id;
#endif
}

inline void popInstanceId(RTCIntersectContext* ctx)
{
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  ctx->instID[--ctx->instStackSize] = RTC_INVALID_GEOMETRY_ID;
#else
  ctx->instID[0] = RTC_INVALID_GEOMETRY_ID;
#endif
}

void instanceIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  const int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCIntersectContext* context = args->context;
  RTCRayHitN* rays = (RTCRayHitN*)args->rayhit;
                                    
  assert(args->N == 1);
  if (!valid[0])
    return;
  
  Ray *ray = (Ray*)rays;
  const Instance* instance = (const Instance*)ptr;
  const Vec3ff ray_org = ray->org;
  const Vec3ff ray_dir = ray->dir;
  const float ray_tnear = ray->tnear();
  const float ray_tfar  = ray->tfar;
  ray->org = (Vec3ff) xfmPoint (instance->world2local,ray_org);
  ray->dir = (Vec3ff) xfmVector(instance->world2local,ray_dir);
  ray->tnear() = ray_tnear;
  ray->tfar  = ray_tfar;
  pushInstanceId(context, instance->userID);
  rtcIntersect1(instance->object,context,RTCRayHit_(*ray));
  popInstanceId(context);
  const float updated_tfar = ray->tfar;
  ray->org = ray_org;
  ray->dir = ray_dir;
  ray->tfar = updated_tfar;
}

inline void pushInstanceIdAndTransform(RTCPointQueryContext* context,
                                       unsigned int id, 
                                       AffineSpace3fa const& w2i_in, 
                                       AffineSpace3fa const& i2w_in)
{
  context->instID[context->instStackSize] = id;

  // local copies of const references to fulfill alignment constraints
  AffineSpace3fa w2i = w2i_in;
  AffineSpace3fa i2w = i2w_in;

  const unsigned int stackSize = context->instStackSize;
  if (stackSize > 0) {
    w2i = (*(AffineSpace3fa*)context->world2inst[stackSize-1]) * w2i;
    i2w = i2w * (*(AffineSpace3fa*)context->inst2world[stackSize-1]);
  }

  (*(AffineSpace3fa*)context->world2inst[stackSize]) = w2i;
  (*(AffineSpace3fa*)context->inst2world[stackSize]) = i2w;

  context->instStackSize++;
}

inline void popInstanceIdAndTransform(RTCPointQueryContext* context)
{
  context->instID[--context->instStackSize] = RTC_INVALID_GEOMETRY_ID;
}

bool instanceClosestPointFunc(RTCPointQueryFunctionArguments* args)
{
  // convert geomID in the scene to instance idx (-4)
  Instance* instance = g_instanceUserDefined[args->geomID - 4];

  pushInstanceIdAndTransform(args->context, instance->userID, instance->world2local, instance->local2world);
  bool changed = rtcPointQuery(instance->object, args->query, args->context, 0, args->userPtr);
  popInstanceIdAndTransform(args->context);
  return changed;
}

Instance* createInstance (RTCScene scene, RTCScene object, int userID, const Vec3fa& lower, const Vec3fa& upper)
{
  Instance* instance = (Instance*) alignedMalloc(sizeof(Instance),16);
  instance->object = object;
  instance->userID = userID;
  instance->lower = lower;
  instance->upper = upper;
  instance->local2world.l.vx = Vec3fa(1,0,0);
  instance->local2world.l.vy = Vec3fa(0,1,0);
  instance->local2world.l.vz = Vec3fa(0,0,1);
  instance->local2world.p    = Vec3fa(0,0,0);
  instance->geometry = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  rtcSetGeometryUserPrimitiveCount(instance->geometry,1);
  rtcSetGeometryUserData(instance->geometry,instance);
  rtcSetGeometryBoundsFunction(instance->geometry,instanceBoundsFunc,nullptr);
  rtcSetGeometryIntersectFunction(instance->geometry,instanceIntersectFunc);
  rtcSetGeometryPointQueryFunction(instance->geometry, instanceClosestPointFunc);
  rtcCommitGeometry(instance->geometry);
  rtcAttachGeometry(scene,instance->geometry);
  rtcReleaseGeometry(instance->geometry);
  return instance;
}

void updateInstance (RTCScene scene, Instance* instance)
{
  instance->world2local = rcp(instance->local2world);
  instance->normal2world = transposed(rcp(instance->local2world.l));
  rtcCommitGeometry(instance->geometry);
}


// ======================================================================== //
//                      triangle mesh geometry                              //
// ======================================================================== //

struct TriangleMesh
{
  ALIGNED_STRUCT_(16)
  Vertex* vertices;
  Triangle* triangles;
  unsigned int num_vertices;
  unsigned int num_triangles;
  
  TriangleMesh()
    : vertices(nullptr), triangles(nullptr) {}

  ~TriangleMesh() {
    if(vertices) alignedFree(vertices);
    if(triangles) alignedFree(triangles);
  }

private:
  TriangleMesh (const TriangleMesh& other) DELETED; // do not implement
  TriangleMesh& operator= (const TriangleMesh& other) DELETED; // do not implement
};


// ======================================================================== //
//              everything needed for closest point query                   //
// ======================================================================== //
struct ClosestPointResult
{
  ClosestPointResult() 
    : primID(RTC_INVALID_GEOMETRY_ID)
    , geomID(RTC_INVALID_GEOMETRY_ID)
  {}

  Vec3f p;
  unsigned int primID;
  unsigned int geomID;
};

bool closestPointFunc(RTCPointQueryFunctionArguments* args)
{
  assert(args->userPtr);
  const unsigned int geomID = args->geomID;
  const unsigned int primID = args->primID;

  RTCPointQueryContext* context = args->context;
  const unsigned int stackSize = args->context->instStackSize;
  const unsigned int stackPtr = stackSize-1;

  AffineSpace3fa inst2world = stackSize > 0
                            ? (*(AffineSpace3fa*)context->inst2world[stackPtr])
                            : one;
  

  // query position in world space
  Vec3fa q(args->query->x, args->query->y, args->query->z);
  
  /*
   * Get triangle information in local space
   */
  const TriangleMesh *const triangle_mesh = g_triangle_meshes[geomID];
  Triangle const& t = triangle_mesh->triangles[primID];
  Vertex const& V0 = triangle_mesh->vertices[t.v0];
  Vertex const& V1 = triangle_mesh->vertices[t.v1];
  Vertex const& V2 = triangle_mesh->vertices[t.v2];
  Vec3fa v0(V0.x, V0.y, V0.z);
  Vec3fa v1(V1.x, V1.y, V1.z);
  Vec3fa v2(V2.x, V2.y, V2.z);

  /*
   * Bring query and primitive data in the same space if necessary.
   */
  if (stackSize > 0 && args->similarityScale > 0)
  {
    // Instance transform is a similarity transform, therefore we 
    // can compute distance insformation in instance space. Therefore,
    // transform query position into local instance space.
    AffineSpace3fa const& m = (*(AffineSpace3fa*)context->world2inst[stackPtr]);
    q = xfmPoint(m, q);
  }
  else if (stackSize > 0)
  {
    // Instance transform is not a similarity transform. We have to transform the
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
  const Vec3fa p = closestPointTriangle(q, v0, v1, v2);
  float d = distance(q, p);
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


// ======================================================================== //
//              helpers to create scene geometry                            //
// ======================================================================== //

TriangleMesh* createTriangulatedSphere (const Vec3fa& p, float r)
{
  // resolution of triangulated spheres
  const int numPhi = 10;
  const int numTheta = 4*numPhi;

  /* create triangle mesh */
  TriangleMesh* sphere = new TriangleMesh();
  sphere->num_vertices = numTheta*(numPhi+1);
  sphere->num_triangles = 2*numTheta*(numPhi-1);
  sphere->vertices = (Vertex*) alignedMalloc(sizeof(Vertex)*sphere->num_vertices+1, 16);
  sphere->triangles = (Triangle*) alignedMalloc(sizeof(Triangle)*sphere->num_triangles, 16);

  /* create sphere */
  int tri = 0;
  const float rcpNumTheta = rcp((float)numTheta);
  const float rcpNumPhi   = rcp((float)numPhi);
  for (int phi=0; phi<=numPhi; phi++)
  {
    for (int theta=0; theta<numTheta; theta++)
    {
      const float phif   = phi*float(pi)*rcpNumPhi;
      const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;

      Vertex& v = sphere->vertices[phi*numTheta+theta];
      v.x = p.x + r*sin(phif)*sin(thetaf);
      v.y = p.y + r*cos(phif);
      v.z = p.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (int theta=1; theta<=numTheta; theta++)
    {
      int p00 = (phi-1)*numTheta+theta-1;
      int p01 = (phi-1)*numTheta+theta%numTheta;
      int p10 = phi*numTheta+theta-1;
      int p11 = phi*numTheta+theta%numTheta;

      if (phi > 1) {
        sphere->triangles[tri].v0 = p10;
        sphere->triangles[tri].v1 = p00;
        sphere->triangles[tri].v2 = p01;
        tri++;
      }

      if (phi < numPhi) {
        sphere->triangles[tri].v0 = p11;
        sphere->triangles[tri].v1 = p10;
        sphere->triangles[tri].v2 = p01;
        tri++;
      }
    }
  }

  return sphere;
}

TriangleMesh* createCube (const Vec3fa& p, float r)
{
  /* create triangle mesh */
  TriangleMesh* cube = new TriangleMesh();
  cube->num_vertices = 8;
  cube->num_triangles = 12;
  cube->vertices = (Vertex*) alignedMalloc(sizeof(Vertex)*cube->num_vertices+1, 16);
  cube->triangles = (Triangle*) alignedMalloc(sizeof(Triangle)*cube->num_triangles, 16);

  /* set vertices and vertex colors */
  cube->vertices[0].x = -r+p.x; cube->vertices[0].y = -r+p.y; cube->vertices[0].z = -r+p.z;
  cube->vertices[1].x = -r+p.x; cube->vertices[1].y = -r+p.y; cube->vertices[1].z = +r+p.z;
  cube->vertices[2].x = -r+p.x; cube->vertices[2].y = +r+p.y; cube->vertices[2].z = -r+p.z;
  cube->vertices[3].x = -r+p.x; cube->vertices[3].y = +r+p.y; cube->vertices[3].z = +r+p.z;
  cube->vertices[4].x = +r+p.x; cube->vertices[4].y = -r+p.y; cube->vertices[4].z = -r+p.z;
  cube->vertices[5].x = +r+p.x; cube->vertices[5].y = -r+p.y; cube->vertices[5].z = +r+p.z;
  cube->vertices[6].x = +r+p.x; cube->vertices[6].y = +r+p.y; cube->vertices[6].z = -r+p.z;
  cube->vertices[7].x = +r+p.x; cube->vertices[7].y = +r+p.y; cube->vertices[7].z = +r+p.z;

  /* set triangles and face colors */
  int tri = 0;

  // left side
  cube->triangles[tri].v0 = 0; cube->triangles[tri].v1 = 1; cube->triangles[tri].v2 = 2; tri++;
  cube->triangles[tri].v0 = 1; cube->triangles[tri].v1 = 3; cube->triangles[tri].v2 = 2; tri++;

  // right side
  cube->triangles[tri].v0 = 4; cube->triangles[tri].v1 = 6; cube->triangles[tri].v2 = 5; tri++;
  cube->triangles[tri].v0 = 5; cube->triangles[tri].v1 = 6; cube->triangles[tri].v2 = 7; tri++;

  // bottom side
  cube->triangles[tri].v0 = 0; cube->triangles[tri].v1 = 4; cube->triangles[tri].v2 = 1; tri++;
  cube->triangles[tri].v0 = 1; cube->triangles[tri].v1 = 4; cube->triangles[tri].v2 = 5; tri++;

  // top side
  cube->triangles[tri].v0 = 2; cube->triangles[tri].v1 = 3; cube->triangles[tri].v2 = 6; tri++;
  cube->triangles[tri].v0 = 3; cube->triangles[tri].v1 = 7; cube->triangles[tri].v2 = 6; tri++;

  // front side
  cube->triangles[tri].v0 = 0; cube->triangles[tri].v1 = 2; cube->triangles[tri].v2 = 4; tri++;
  cube->triangles[tri].v0 = 2; cube->triangles[tri].v1 = 6; cube->triangles[tri].v2 = 4; tri++;

  // back side
  cube->triangles[tri].v0 = 1; cube->triangles[tri].v1 = 5; cube->triangles[tri].v2 = 3; tri++;
  cube->triangles[tri].v0 = 3; cube->triangles[tri].v1 = 5; cube->triangles[tri].v2 = 7; tri++;

  return cube;
}

TriangleMesh* createPlane (
  AffineSpace3fa const& M, // transformation
  unsigned int R)          // resolution
{
  /* create triangle mesh */
  TriangleMesh* plane = new TriangleMesh();
  plane->num_vertices = (R+1)*(R+1);
  plane->num_triangles = 2*R*R;
  plane->vertices = (Vertex*) alignedMalloc(sizeof(Vertex)*plane->num_vertices+1, 16);
  plane->triangles = (Triangle*) alignedMalloc(sizeof(Triangle)*plane->num_triangles, 16);

  /* set vertices and vertex colors */
  for (unsigned int y = 0; y <= R; ++y)
  for (unsigned int x = 0; x <= R; ++x)
  {
    Vec3fa p((float)x/R, (float)y/R, 0.f);
    Vec3fa pt = xfmPoint(M, p);
    plane->vertices[y*(R+1)+x].x = pt.x;
    plane->vertices[y*(R+1)+x].y = pt.y;
    plane->vertices[y*(R+1)+x].z = pt.z;
  }

  /* set triangles and face colors */
  for (unsigned int j = 0; j < R; ++j)
  for (unsigned int i = 0; i < R; ++i)
  {
    plane->triangles[2*(j*R+i)+0].v0 = (j*(R+1)+i);
    plane->triangles[2*(j*R+i)+0].v1 = (j*(R+1)+i) + (R + 1) + 1;
    plane->triangles[2*(j*R+i)+0].v2 = (j*(R+1)+i) + (R + 1);
    plane->triangles[2*(j*R+i)+1].v0 = (j*(R+1)+i);
    plane->triangles[2*(j*R+i)+1].v1 = (j*(R+1)+i) + 1;
    plane->triangles[2*(j*R+i)+1].v2 = (j*(R+1)+i) + (R + 1) + 1;
  }

  return plane;
}


// ======================================================================== //
//     update instance transforms and perform all closest point queries     //
// ======================================================================== //

void updateGeometryAndQueries(float time)
{
  const float delta_time = time - g_last_time;
  if (g_animate) {
    g_animate_time += delta_time;
  }
  g_last_time = time;

  g_instance_xfm[0] = AffineSpace3fa::translate(Vec3f(7.5f, 0.f, -8.f))
                    * AffineSpace3fa::rotate(Vec3f(0.f, 1.f, 0.f), float(pi)/2.f)
                    * AffineSpace3fa::rotate(Vec3f(1.f, 0.f, 0.f), 0.2f * sin(g_animate_time));

  g_instance_xfm[1] = AffineSpace3fa::translate(Vec3f(0.f, 3.f + 1.5f*sin(g_animate_time), -9.f)) 
                    * AffineSpace3fa::scale(Vec3fa(1.f, 2.f, 3.f))
                    * AffineSpace3fa::rotate(Vec3f(0.f, 1.f, 0.f), float(pi));
  
  AffineSpace3fa sheer = AffineSpace3fa::scale(Vec3f(1.2f));
  sheer.l.vz.x = cos(g_animate_time*0.75f) * 1.0f + 0.5f;
  g_instance_xfm[2] = AffineSpace3fa::translate(Vec3f(-8.5f, 0.f, -7.f)) 
                    * AffineSpace3fa::rotate(Vec3fa(0.f, 1.f, 0.f), -float(pi)/2.f)
                    * sheer;
  
  assert( similarityTransform(g_instance_xfm[0], 0));
  assert(!similarityTransform(g_instance_xfm[1], 0));
  assert(!similarityTransform(g_instance_xfm[2], 0));

  g_normal_xfm[0] = transposed(rcp(g_instance_xfm[0].l));
  g_normal_xfm[1] = transposed(rcp(g_instance_xfm[1].l));
  g_normal_xfm[2] = transposed(rcp(g_instance_xfm[2].l));

  /* set instance transformations */
  rtcSetGeometryTransform(g_instanceEmbree[0],0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&g_instance_xfm[0]);
  rtcSetGeometryTransform(g_instanceEmbree[1],0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&g_instance_xfm[1]);
  rtcSetGeometryTransform(g_instanceEmbree[2],0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&g_instance_xfm[2]);

  /* update scene */
  rtcCommitGeometry(g_instanceEmbree[0]);
  rtcCommitGeometry(g_instanceEmbree[1]);
  rtcCommitGeometry(g_instanceEmbree[2]);
  rtcCommitScene(g_sceneEmbreeInstance);
  
  g_instanceUserDefined[0]->local2world = g_instance_xfm[0];
  g_instanceUserDefined[1]->local2world = g_instance_xfm[1];
  g_instanceUserDefined[2]->local2world = g_instance_xfm[2];

  /* update scene */
  updateInstance(g_sceneUserDefinedInstance,g_instanceUserDefined[0]);
  updateInstance(g_sceneUserDefinedInstance,g_instanceUserDefined[1]);
  updateInstance(g_sceneUserDefinedInstance,g_instanceUserDefined[2]);
  rtcCommitScene(g_sceneUserDefinedInstance);

  g_scene = g_userDefinedInstancing ? g_sceneUserDefinedInstance : g_sceneEmbreeInstance;

  for (int i = 0; i < g_num_point_queries; ++i)
  {
    RTCPointQuery query;
    query.x = g_sphere_locations[2*i+0].x; 
    query.y = g_sphere_locations[2*i+0].y;
    query.z = g_sphere_locations[2*i+0].z;
    query.radius = inf;
    query.time = 0.f;

    ClosestPointResult result;
    RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQuery(g_scene, &query, &context, nullptr, (void*)&result);
    assert(result.primID != RTC_INVALID_GEOMETRY_ID || result.geomID != RTC_INVALID_GEOMETRY_ID);
    g_sphere_locations[2*i+1] = result.p;

    g_sphere_vertex_buffer[2*i+0] = Vec4f(g_sphere_locations[2*i+0], 0.2f);
    g_sphere_vertex_buffer[2*i+1] = Vec4f(g_sphere_locations[2*i+1], 0.2f);
    g_line_vertex_buffer[2*i+0]   = Vec4f(g_sphere_locations[2*i+0], 0.05f);
    g_line_vertex_buffer[2*i+1]   = Vec4f(g_sphere_locations[2*i+1], 0.05f);
  }
  
  rtcCommitGeometry(g_spheres);
  rtcCommitGeometry(g_lines);
  rtcCommitScene(g_scene);
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene data */
  g_triangle_meshes[0] = createPlane(
    AffineSpace3fa::translate(Vec3fa(0.f, -3.f, 0.f)) *
    AffineSpace3fa::scale(Vec3fa(10.f, 4.f, 4.f)) * 
    AffineSpace3fa::rotate(Vec3fa(1.f, 0.f, 0.f), float(pi)/2) *
    AffineSpace3fa::translate(Vec3fa(-0.5f, -0.5f, 0.f)),
    1);
  g_triangle_meshes[1] = createPlane(
    AffineSpace3fa::translate(Vec3fa(0.f, -1.f, 2.f)) *
    AffineSpace3fa::scale(Vec3fa(10.f, 4.f, 4.f)) * 
    AffineSpace3fa::translate(Vec3fa(-0.5f, -0.5f, 0.f)),
    8);
  g_triangle_meshes[2] = createTriangulatedSphere(Vec3fa(3.f, -2.f, 0.0f), 1.f);
  g_triangle_meshes[3] = createCube(Vec3fa(-3.f, -2.f, 0), 1.0f);

  /* create embree scene */
  g_sceneEmbreeInstance = rtcNewScene(g_device);
  g_sceneUserDefinedInstance = rtcNewScene(g_device);

  /* create scene that will be instanced */
  g_scene1 = rtcNewScene(g_device);

  // add the four objects to all three scenes
  {
    RTCScene scenes[3] = { g_scene1, g_sceneEmbreeInstance, g_sceneUserDefinedInstance };
    for (int m = 0; m < 4; ++m)
    for (int s = 0; s < 3; ++s) 
    {
      RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
      rtcSetGeometryPointQueryFunction(geom, closestPointFunc);
      rtcCommitGeometry(geom);
      rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,g_triangle_meshes[m]->vertices, 0,sizeof(Vertex),  g_triangle_meshes[m]->num_vertices);
      rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX, 0,RTC_FORMAT_UINT3, g_triangle_meshes[m]->triangles,0,sizeof(Triangle),g_triangle_meshes[m]->num_triangles);
      rtcAttachGeometryByID(scenes[s], geom, m);
      rtcReleaseGeometry(geom);
      rtcCommitGeometry(geom);
    }
    rtcCommitScene(g_scene1);
  }

  /* compute bounding box of the scene that will be instanced */
  Vec3f bbmin(inf);
  Vec3f bbmax(neg_inf);
  for (int i = 0; i < 4; ++i) {
    TriangleMesh* mesh = g_triangle_meshes[i];
    for (unsigned int v = 0; v < mesh->num_vertices; ++v) {
      Vertex* vert = mesh->vertices+v;
      bbmin = min(bbmin, Vec3f(vert->x, vert->y, vert->z));
      bbmax = max(bbmax, Vec3f(vert->x, vert->y, vert->z));
    }
  }
  
  /* instantiate geometry */
  for (unsigned int i = 0; i < 3; ++i) {
    g_instanceEmbree[i] = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
    rtcSetGeometryInstancedScene(g_instanceEmbree[i], g_scene1);
    rtcSetGeometryTimeStepCount(g_instanceEmbree[i], 1);
    rtcAttachGeometryByID(g_sceneEmbreeInstance, g_instanceEmbree[i], 4+i);
    rtcReleaseGeometry(g_instanceEmbree[i]);
    rtcCommitGeometry(g_instanceEmbree[i]);
    
    g_instanceUserDefined[i] = createInstance(g_sceneUserDefinedInstance, g_scene1, i, bbmin, bbmax);
  }

  {
    // add visualization spheres to both scenes
    RTCScene scenes[2] = { g_sceneEmbreeInstance, g_sceneUserDefinedInstance };
    for (unsigned int s = 0; s < 2; ++s)
    {
      g_spheres = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SPHERE_POINT);
      rtcSetSharedGeometryBuffer(g_spheres, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, g_sphere_vertex_buffer, 0, sizeof(Vec4f), 2*g_num_point_queries);
      rtcAttachGeometryByID(scenes[s], g_spheres, g_spheres_geomID);
      rtcReleaseGeometry(g_spheres);
      rtcCommitGeometry(g_spheres);
      
      g_lines = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
      rtcSetSharedGeometryBuffer(g_lines, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, g_line_vertex_buffer, 0, sizeof(Vec4f), 2*g_num_point_queries);
      rtcSetSharedGeometryBuffer(g_lines, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT, g_line_index_buffer, 0, sizeof(unsigned int), g_num_point_queries);
      rtcAttachGeometryByID(scenes[s], g_lines, g_lines_geomID);
      rtcReleaseGeometry(g_lines);
      rtcCommitGeometry(g_lines);
    }
  }

  updateGeometryAndQueries(0.f);
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), 
                     Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 
                     0.0f, inf, 0.0f, -1,
                     RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);

  /* intersect ray with scene */
  rtcIntersect1(g_scene, &context, RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    if (ray.geomID == g_spheres_geomID)
    {
      if (ray.primID % 2 == 0) return Vec3fa(0.8f, 0.2f, 0.2f);
      else                     return Vec3fa(1.0f, 1.0f, 1.0f);
    }
    if (ray.geomID == g_lines_geomID)
    {
      return Vec3fa(0.7f, 0.3f, 0.7f);
    }

    /* calculate shading normal in world space */
    Vec3fa Ns = ray.Ng;
    if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID)
    {
      if (g_userDefinedInstancing)
        Ns = xfmVector(g_instanceUserDefined[ray.instID[0]]->normal2world, Vec3fa(Ns));
      else
        // convert geomID (ray.instID) in the scene to instance idx (-4)
        Ns = xfmVector(g_normal_xfm[ray.instID[0]-4], Vec3fa(Ns));
    }

    Ns = face_forward(ray.dir,normalize(Ns));
    return 0.5f * Ns + Vec3fa(0.5f, 0.5f, 0.5f);
  }
  return Vec3fa(0.0f);
}

/* renders a single screen tile */
void renderTileStandard(int taskIndex,
                        int threadIndex,
                        int* pixels,
                        const unsigned int width,
                        const unsigned int height,
                        const float time,
                        const ISPCCamera& camera,
                        const int numTilesX,
                        const int numTilesY)
{
  const unsigned int tileY = taskIndex / numTilesX;
  const unsigned int tileX = taskIndex - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* calculate pixel color */
    Vec3fa color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int threadIndex, int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         const int numTilesX,
                         const int numTilesY)
{
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

extern "C" void renderFrameStandard (int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera)
{
  /* render all pixels */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
}


/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  updateGeometryAndQueries(time);
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene1); g_scene1 = nullptr;
  rtcReleaseScene (g_sceneEmbreeInstance); g_sceneEmbreeInstance = nullptr;
  rtcReleaseScene (g_sceneUserDefinedInstance); g_sceneUserDefinedInstance = nullptr;
  for (int i = 0; i < 4; ++i) 
  { 
    if (g_triangle_meshes[i]) 
      delete g_triangle_meshes[i]; 
  }
  for (int i = 0; i < 3; ++i)
  {
    if (g_instanceUserDefined[i])
      delete g_instanceUserDefined[i];
  }
  rtcReleaseDevice(g_device); g_device = nullptr;
}

} // namespace embree
