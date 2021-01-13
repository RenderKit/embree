// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"
#include <set>
#include "pbd.h"
#include "clothModel.h"
#include "constraints.h"

namespace embree {

extern RTCDevice g_device;
extern RTCScene g_scene;
size_t cur_time = 0;
extern std::vector<std::pair<std::pair<unsigned,unsigned>, std::pair<unsigned,unsigned>>> sim_collisions;
// extern bool use_user_geometry;
extern std::vector<std::unique_ptr<collide2::Mesh>> meshes;
extern unsigned int clothID;
extern bool benchmark;
double total_collision_time = 0.0f;
  
extern void triangle_bounds_func(const struct RTCBoundsFunctionArguments* args);
extern void triangle_intersect_func(const RTCIntersectFunctionNArguments* args);
extern void CollideFunc (void* userPtr, RTCCollision* collisions, unsigned int num_collisions);

extern int numPhi;
extern int numTheta;

extern size_t NX; 
extern size_t NZ;
const float width = 5.f;
const float height = 5.f;
const float ks = 4000.f;
const float damping = .2f;
const float m = 1.f;
const float nsub = 5.f;
const float h = 1.f / (nsub * 24.f);
const size_t nIters = 20;
const float collDelta = 1.e-6f;

extern bool pause;

/* creates a ground plane */
unsigned int createGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  std::unique_ptr<collide2::Mesh> plane (new collide2::Mesh());
  plane->x_.resize (4);
  plane->tris_.resize (2);

  /* set plane->x_ */
  plane->x_[0].x = -10; plane->x_[0].y = -2; plane->x_[0].z = -10;
  plane->x_[1].x = -10; plane->x_[1].y = -2; plane->x_[1].z = +10;
  plane->x_[2].x = +10; plane->x_[2].y = -2; plane->x_[2].z = -10;
  plane->x_[3].x = +10; plane->x_[3].y = -2; plane->x_[3].z = +10;

  /* set plane->tris_ */
  plane->tris_[0].v0 = 0; plane->tris_[0].v1 = 1; plane->tris_[0].v2 = 2;
  plane->tris_[1].v0 = 1; plane->tris_[1].v1 = 3; plane->tris_[1].v2 = 2;

  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,plane->x_.data(),0,sizeof(collide2::vec_t),plane->x_.size());
  rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,plane->tris_.data(),0,sizeof(Triangle),plane->tris_.size ());
  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  meshes.push_back (std::move (plane));
  return geomID;
}

unsigned int createTriangulatedSphere (RTCScene scene, const Vec3fa& p, float r)
{
  /* create triangle mesh */
  std::unique_ptr<collide2::Mesh> sphere (new collide2::Mesh());
  sphere->x_.resize (numTheta*(numPhi-1)+2);
  sphere->tris_.resize (2*numTheta*(numPhi-1));

  /* create sphere */
  int tri = 0;
  const float rcpNumTheta = rcp((float)numTheta);
  const float rcpNumPhi   = rcp((float)numPhi);
  sphere->x_[0].x = p.x;
  sphere->x_[0].y = p.y + r;
  sphere->x_[0].z = p.z;

  for (int phi=0; phi<numPhi-1; phi++)
  {
    const float phif   = (phi+1)*float(pi)*rcpNumPhi;
    const float sinp   = sin(phif);
    const float cosp   = cos(phif);
    for (int theta=0; theta<numTheta; theta++)
    {
      const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;

      auto& v = sphere->x_[phi*(numTheta)+theta+1];
      v.x = p.x + r*sinp*sin(thetaf);
      v.y = p.y + r*cosp;
      v.z = p.z + r*sinp*cos(thetaf);
    }
  }
  sphere->x_[numTheta*(numPhi-1)+1].x = p.x;
  sphere->x_[numTheta*(numPhi-1)+1].y = p.y - r;
  sphere->x_[numTheta*(numPhi-1)+1].z = p.z;

  for (int theta=0; theta<numTheta; theta++) {
    sphere->tris_[tri].v0 = theta+1;
    sphere->tris_[tri].v1 = 0;
    sphere->tris_[theta].v2 = (theta+1)%numTheta + 1;
    tri++;
  }

  for (int phi=0; phi<numPhi-2; phi++)
  {
    for (int theta=0; theta<numTheta; theta++)
    {
      int p00 = phi*numTheta + 1 + theta;
      int p01 = phi*numTheta + 1 + (theta+1)%numTheta;
      int p10 = (phi+1)*numTheta + 1 + theta;
      int p11 = (phi+1)*numTheta + 1 + (theta+1)%numTheta;

      sphere->tris_[tri].v0 = p10;
      sphere->tris_[tri].v1 = p01;
      sphere->tris_[tri].v2 = p00;
      tri++;

      sphere->tris_[tri].v0 = p11;
      sphere->tris_[tri].v1 = p01;
      sphere->tris_[tri].v2 = p10;
      tri++;
    }
  }

  for (int theta=0; theta<numTheta; theta++) {
    sphere->tris_[tri].v0 = (theta+1)%numTheta + numTheta*(numPhi-2) + 1;
    sphere->tris_[tri].v1 = numTheta*(numPhi-1)+1;
    sphere->tris_[tri].v2 = theta + numTheta*(numPhi-2) + 1;
    tri++;
  }

  // if (use_user_geometry) {
    RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_USER);
    unsigned int geomID = rtcAttachGeometry(scene,geom);
    rtcSetGeometryUserPrimitiveCount(geom, sphere->tris_.size());
    rtcSetGeometryUserData(geom,(void*)(size_t)geomID);
    rtcSetGeometryBoundsFunction   (geom, triangle_bounds_func, nullptr);
    rtcSetGeometryIntersectFunction(geom, triangle_intersect_func);
    rtcCommitGeometry(geom);
    rtcReleaseGeometry(geom);
    meshes.push_back (std::move (sphere));
    return geomID;
  // } else {
  //   RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  //   rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sphere->x_.data(),0,sizeof(collide2::vec_t),sphere->x_.size());
  //   rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sphere->tris_.data(),0,sizeof(Triangle),sphere->tris_.size ());
  //   rtcCommitGeometry(geom);
  //   unsigned int geomID = rtcAttachGeometry(scene,geom);
  //   rtcReleaseGeometry(geom);
  //   meshes.push_back (std::move (sphere));
  //   return geomID;
  // }
}

void initializeClothPositions (collide2::ClothModel & cloth) {

  sim_collisions.clear();

  for (size_t i=0; i<NX; ++i) {
    for (size_t j=0; j<NZ; ++j) {
      cloth.x_0_[NX*i+j].x = -(.5f*width) + (float)i*(width/(float)(NX-1));
      cloth.x_0_[NX*i+j].y = +1.5f;
      cloth.x_0_[NX*i+j].z = -(.5f*height) + (float)j*(height/(float)(NZ-1));
    }
  }
  cloth.x_ = cloth.x_0_;
  cloth.x_old_ = cloth.x_0_;
  cloth.x_last_ = cloth.x_0_;
  collide2::vec_t nullvec(0.f, 0.f, 0.f);
  std::fill (cloth.v_.begin (), cloth.v_.end (), nullvec);

  cur_time = 0;
  cloth.clearCollisionConstraints();
}

unsigned int createClothSheet (RTCScene scene)
{
  std::unique_ptr<collide2::ClothModel> cloth (new collide2::ClothModel());
  cloth->x_0_.resize (NX*NZ);
  cloth->x_.resize (NX*NZ);
  cloth->x_old_.resize (NX*NZ);
  cloth->x_last_.resize (NX*NZ);
  collide2::vec_t nullvec(0.f, 0.f, 0.f);
  collide2::vec_t gravity(0.f, -9.8f, 0.f);
  cloth->v_.resize (NX*NZ, nullvec);
  cloth->a_.resize (NX*NZ, gravity);
  cloth->m_.resize (NX*NZ, m);
  cloth->m_inv_.resize (NX*NZ, 1.f / m);
  cloth->tris_.resize (2*(NX-1)*(NZ-1));

  initializeClothPositions (*cloth);

  // init topology
  for (size_t i=0; i<NX-1; ++i) {
    for (size_t j=0; j<NZ-1; ++j) {
      cloth->tris_[2*(i*(NZ-1)+j)].v0 = i*NZ+j;
      cloth->tris_[2*(i*(NZ-1)+j)].v1 = i*NZ+j+1;
      cloth->tris_[2*(i*(NZ-1)+j)].v2 = (i+1)*NZ+j+1;

      cloth->tris_[2*(i*(NZ-1)+j)+1].v0 = (i+1)*NZ+j+1;
      cloth->tris_[2*(i*(NZ-1)+j)+1].v1 = (i+1)*NZ+j;
      cloth->tris_[2*(i*(NZ-1)+j)+1].v2 = i*NZ+j;
    }
  }

  cloth->k_stretch_ = ks;
  cloth->k_damp_ = damping;

  // set distance constraints
  for (size_t vID=0; vID<NX*NZ; ++vID) {
    
    size_t i = vID/NZ;
    size_t j = vID%NZ;

    std::vector<size_t> sIDs;

    if (i<NX-1) sIDs.push_back (vID+NZ);
    if (j<NZ-1) sIDs.push_back (vID+1);

    if (i<NX-1) {
      if (j<NZ-1) {
        sIDs.push_back (vID+NZ+1);
      }
    }

    if (i>0) {
      if (j<NZ-1) {
        sIDs.push_back (vID-NZ+1);
      }
    }

    for (auto id : sIDs) {
      auto c = new collide2::DistanceConstraint ();
      c->initConstraint (*cloth, vID, id);
      cloth->m_constraints_.push_back (c);
    }
  }

  // fix corners
  // cloth->m_[0] = 0.f;
  // cloth->m_[(NX-1)*NZ] = 0.f;
  // cloth->m_[(NX-1)*NZ + NZ-1] = 0.f;
  // cloth->m_[NZ-1] = 0.f;
  // cloth->m_inv_[0] = 0.f;
  // cloth->m_inv_[(NX-1)*NZ] = 0.f;
  // cloth->m_inv_[(NX-1)*NZ + NZ-1] = 0.f;
  // cloth->m_inv_[NZ-1] = 0.f;
  // cloth->a_[0].y = 0.f;
  // cloth->a_[(NX-1)*NZ].y = 0.f;
  // cloth->a_[(NX-1)*NZ + NZ-1].y = 0.f;
  // cloth->a_[NZ-1].y = 0.f;

  // if (use_user_geometry) {
    RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_USER);
    unsigned int geomID = rtcAttachGeometry(scene,geom);
    rtcSetGeometryUserPrimitiveCount(geom, cloth->tris_.size());
    rtcSetGeometryUserData(geom,(void*)(size_t)geomID);
    rtcSetGeometryBoundsFunction   (geom, triangle_bounds_func, nullptr);
    rtcSetGeometryIntersectFunction(geom, triangle_intersect_func);
    rtcCommitGeometry(geom);
    rtcReleaseGeometry(geom);
    meshes.push_back (std::move (cloth));
    return geomID;
  // } else {
  //   RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  //   rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, cloth->x_.data(), 0, sizeof(collide2::vec_t), cloth->x_.size());
  //   rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX , 0, RTC_FORMAT_UINT3,  cloth->tris_.data(), 0, sizeof(Triangle), cloth->tris_.size());
  //   rtcCommitGeometry(geom);
  //   unsigned int geomID = rtcAttachGeometry(scene,geom);
  //   rtcReleaseGeometry(geom);
  //   meshes.push_back (std::move (cloth));
  //   return geomID;
  // }
}

collide2::CollisionConstraint * makeCollisionConstraint (RTCScene scene, unsigned qID, unsigned cGeomID, unsigned cPrimID) 
{
  auto x0 = meshes[cGeomID]->x_[meshes[cGeomID]->tris_[cPrimID].v0];
  auto x1 = meshes[cGeomID]->x_[meshes[cGeomID]->tris_[cPrimID].v1];
  auto x2 = meshes[cGeomID]->x_[meshes[cGeomID]->tris_[cPrimID].v2];
  collide2::vec_t e0, e1;
  e0.x = x1.x - x0.x;
  e0.y = x1.y - x0.y;
  e0.z = x1.z - x0.z;
  e1.x = x2.x - x1.x;
  e1.y = x2.y - x1.y;
  e1.z = x2.z - x1.z;
  auto collNorm = normalize (cross (e0, e1));
  auto c = new collide2::CollisionConstraint ();
  c->initConstraint (qID, x0, collNorm, collDelta);
  return c;
}

void addCollisionConstraints (RTCScene scene)
{
  auto & cloth = (collide2::ClothModel &) (*meshes[clothID]);
  cloth.clearCollisionConstraints();
  
  for (auto const & coll : sim_collisions) {
    auto & c0 = coll.first;
    auto & c1 = coll.second;

    // throw out self collisions for now
    if (clothID == c0.first && clothID == c1.first) continue;

    // push back cloth vertex constraints if not fixed
    if (clothID == c1.first) {
      if (cloth.m_inv_[cloth.tris_[c1.second].v0] != 0) {
        auto c = makeCollisionConstraint (scene, cloth.tris_[c1.second].v0, c0.first, c0.second);
        cloth.c_constraints_.push_back (c);
      }
      if (cloth.m_inv_[cloth.tris_[c1.second].v1] != 0) {
        auto c = makeCollisionConstraint (scene, cloth.tris_[c1.second].v1, c0.first, c0.second);
        cloth.c_constraints_.push_back (c);
      }
      if (cloth.m_inv_[cloth.tris_[c1.second].v2] != 0) {
        auto c = makeCollisionConstraint (scene, cloth.tris_[c1.second].v2, c0.first, c0.second);
        cloth.c_constraints_.push_back (c);
      }
    }
  }
}

void updateScene () 
{
  auto & cloth = (collide2::ClothModel &) (*meshes[clothID]);
  collide2::updatePositions (cloth, h);

  rtcUpdateGeometryBuffer(rtcGetGeometry(g_scene, clothID), RTC_BUFFER_TYPE_VERTEX, 0);
  rtcCommitGeometry(rtcGetGeometry(g_scene, clothID));
  rtcCommitScene(g_scene);

  // sim_collisions.clear();
  double t0 = getSeconds();
  rtcCollide(g_scene,g_scene,CollideFunc,&sim_collisions);
  double t1 = getSeconds();
  total_collision_time += t1-t0;
  addCollisionConstraints (g_scene);

  collide2::constrainPositions (cloth, h, nIters);
  collide2::updateVelocities (cloth, h);

  rtcUpdateGeometryBuffer(rtcGetGeometry(g_scene, clothID), RTC_BUFFER_TYPE_VERTEX, 0);
  rtcCommitGeometry(rtcGetGeometry(g_scene, clothID));
  rtcCommitScene(g_scene);

  ++cur_time;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera)
{
  /* initialize ray */
  Ray ray;
  ray.org = Vec3ff(camera.xfm.p);
  ray.dir = Vec3ff(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear() = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time() = 0.0f;

  /* intersect ray with scene */
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  rtcIntersect1(g_scene,&context,RTCRayHit_(ray));

  /* shade background black */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
    return Vec3fa(0.0f);
  }

  /* shade all rays that hit something */
  Vec3fa color(0,0,0);
  color = Vec3fa(1.0f,0.0f,0.0f);
  if (ray.geomID == clothID) {
    color = Vec3fa(0.0f,1.0f,0.0f);
  }

  return color*abs(dot(neg(ray.dir),normalize(ray.Ng)));
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
  const int t = taskIndex;
  const unsigned int tileY = t / numTilesX;
  const unsigned int tileX = t - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    Vec3fa color = renderPixelStandard((float)x,(float)y,camera);

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

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{

  g_scene = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(g_scene,RTC_BUILD_QUALITY_LOW);

  // createGroundPlane (g_scene);
  createTriangulatedSphere(g_scene,Vec3fa(0, 0., 0),1.f);
  clothID = createClothSheet (g_scene);
  rtcCommitScene (g_scene);

  /* set error handler */
  rtcSetDeviceErrorFunction(g_device,error_handler,nullptr);
}

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
  /* render image */
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

  if (!pause) updateScene();
  //else PRINT(cur_time);

  if (benchmark && cur_time == 128) {
    std::cout << "collision time = " << 1000.0f*total_collision_time << " ms" << std::endl;
    exit(0);
  }
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = nullptr;
  rtcReleaseDevice(g_device); g_device = nullptr;
}

} // namespace embree
