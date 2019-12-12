// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"
#include <set>

namespace embree {

extern RTCDevice g_device;
extern RTCScene g_scene;
extern std::set<std::pair<unsigned,unsigned>> collision_candidates;

const int numPhi = 45;
const int numTheta = 2*numPhi;

const size_t NX = 20; 
const size_t NZ = 20;
const float strl = 4.f/(float)(NX-1);
const float shrl = 1.41421356f * strl;
const float brl = 2.f * strl;
const float ks = 10000.f;
const float ksh = 100.f;
const float kb = 100.f;
const float m = 1.f;
const float cd = 100.f;
const float h = 0.01*(1.f/24.f);
unsigned int clothID;

Vertex prevPos[NX*NZ];

unsigned int createTriangulatedSphere (RTCScene scene, const Vec3fa& p, float r)
{
  /* create triangle mesh */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* map triangle and vertex buffers */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),numTheta*(numPhi+1));
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2*numTheta*(numPhi-1));

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

      Vertex& v = vertices[phi*numTheta+theta];
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
        triangles[tri].v0 = p10;
        triangles[tri].v1 = p01;
        triangles[tri].v2 = p00;
        tri++;
      }

      if (phi < numPhi) {
        triangles[tri].v0 = p11;
        triangles[tri].v1 = p01;
        triangles[tri].v2 = p10;
        tri++;
      }
    }
  }

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

unsigned int createClothSheet (RTCScene scene)
{
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),NX*NZ);
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2*(NX-1)*(NZ-1));

  for (size_t i=0; i<NX; ++i) {
    for (size_t j=0; j<NZ; ++j) {
      vertices[NX*i+j].x = -2.f + (float)i*(4.f/(float)(NX-1));
      vertices[NX*i+j].y = +2.f;
      vertices[NX*i+j].z = -2.f + (float)j*(4.f/(float)(NZ-1));
    }
  }
  for (size_t i=0; i<NX-1; ++i) {
    for (size_t j=0; j<NZ-1; ++j) {
      triangles[2*(i*(NZ-1)+j)].v0 = i*NZ+j;
      triangles[2*(i*(NZ-1)+j)].v1 = i*NZ+j+1;
      triangles[2*(i*(NZ-1)+j)].v2 = (i+1)*NZ+j+1;

      triangles[2*(i*(NZ-1)+j)+1].v0 = (i+1)*NZ+j+1;
      triangles[2*(i*(NZ-1)+j)+1].v1 = (i+1)*NZ+j;
      triangles[2*(i*(NZ-1)+j)+1].v2 = i*NZ+j;
    }
  }

  memcpy (prevPos, vertices, NX*NZ*sizeof(Vertex));

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

struct force {
  float x,y,z = 0.f;
};

float norm (Vertex const & v1, Vertex const & v2) {
  return std::sqrt ((v1.x-v2.x)*(v1.x-v2.x)+(v1.y-v2.y)*(v1.y-v2.y)+(v1.z-v2.z)*(v1.z-v2.z));
}

force computeVertexAccels (size_t vID) 
{
  size_t i = vID/NZ;
  size_t j = vID%NZ;

  // std::cout << "i: " << i << std::endl;
  // std::cout << "j: " << j << std::endl;

  force out; out.x = 0.f; out.y = 0.f; out.z = 0.f;

  if (0==i /*&& 0==j*/) return out;
  if (0==i && NZ-1==j) return out;

  // std::cout << "vID: " << vID << std::endl;

  Vertex* vertices = (Vertex*) rtcGetGeometryBufferData (rtcGetGeometry(g_scene,clothID),RTC_BUFFER_TYPE_VERTEX,0);
  Vertex& v0 = vertices[vID];

  std::vector<size_t> sIDs;
  if (i<NX-1)  sIDs.push_back (vID+NZ);
  if (i>0) sIDs.push_back (vID-NZ);
  if (j>0) sIDs.push_back (vID-1);
  if (j<NZ-1) sIDs.push_back (vID+1);

  for (auto id : sIDs) {
    // std::cout << "id: " << id << std::endl;
    Vertex& vs = vertices[id];
    float k = ks*((strl/norm(v0,vs))-1.f);
    out.x += k*(v0.x-vs.x);
    out.y += k*(v0.y-vs.y);
    out.z += k*(v0.z-vs.z);

    // std::cout << "out: " << out.x << " " << out.y << " " << out.z << " " << std::endl;
  }

  std::vector<size_t> shIDs;
  if (i<NX-1) {
    if (j>0) shIDs.push_back (vID+NZ-1);
    if (j<NZ-1) shIDs.push_back (vID+NZ+1);
  }
  if (i>0) {
    if (j>0) shIDs.push_back (vID-NZ-1);
    if (j<NZ-1) shIDs.push_back (vID-NZ+1);
  }

  for (auto id : shIDs) {
    // std::cout << "id: " << id << std::endl;
    Vertex& vs = vertices[id];
    float k = ksh*((shrl/norm(v0,vs))-1.f);
    out.x += k*(v0.x-vs.x);
    out.y += k*(v0.y-vs.y);
    out.z += k*(v0.z-vs.z);

    // std::cout << "out: " << out.x << " " << out.y << " " << out.z << " " << std::endl;
  }

  std::vector<size_t> bIDs;
  if (i<NX-2)  bIDs.push_back (vID+2*NZ);
  if (i>1) bIDs.push_back (vID-2*NZ);
  if (j>1) bIDs.push_back (vID-2);
  if (j<NZ-2) bIDs.push_back (vID+2);

  for (auto id : bIDs) {
    // std::cout << "id: " << id << std::endl;
    Vertex& vs = vertices[id];
    float k = kb*((brl/norm(v0,vs))-1.f);
    out.x += k*(v0.x-vs.x);
    out.y += k*(v0.y-vs.y);
    out.z += k*(v0.z-vs.z);

    // std::cout << "out: " << out.x << " " << out.y << " " << out.z << " " << std::endl;
  }

  out.y -= 9.8f;

  out.x -= cd*(vertices[vID].x - prevPos[vID].x);
  out.y -= cd*(vertices[vID].y - prevPos[vID].y);
  out.z -= cd*(vertices[vID].z - prevPos[vID].z);

  // std::cout << "out: " << out.x << " " << out.y << " " << out.z << " " << std::endl;

  return out;
}

/* creates a ground plane */
unsigned int createGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera)
{
  /* initialize ray */
  Ray ray;
  ray.org = Vec3fa(camera.xfm.p);
  ray.dir = Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
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
  if (collision_candidates.find(std::make_pair(ray.geomID,ray.primID)) != collision_candidates.end())
    color = Vec3fa(1.0f,1.0f,0.0f);

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
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{

  g_scene = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(g_scene,RTC_BUILD_QUALITY_LOW);

  createTriangulatedSphere(g_scene,Vec3fa(0, -1., 0),1.f);
  createGroundPlane (g_scene);
  clothID = createClothSheet (g_scene);
  rtcCommitScene (g_scene);

  for (size_t i=0; i<NX*NZ; ++i) {
    computeVertexAccels (i);
  }

  /* set error handler */
  rtcSetDeviceErrorFunction(g_device,error_handler,nullptr);

  /* set start render mode */
  renderTile = renderTileStandard;
}

void updateScene () 
{
  Vertex* vertices = (Vertex*) rtcGetGeometryBufferData (rtcGetGeometry(g_scene,clothID),RTC_BUFFER_TYPE_VERTEX,0);
  for (size_t i=0; i<NX*NZ; ++i) {
    // std::cout << "id: " << i << std::endl;
    auto a = computeVertexAccels (i);
    // std::cout << "a: " << a.x << " " << a.y << " " << a.z << std::endl;
    Vertex newPos;
    newPos.x = 2*vertices[i].x - prevPos[i].x + h*h*a.x;
    newPos.y = 2*vertices[i].y - prevPos[i].y + h*h*a.y;
    newPos.z = 2*vertices[i].z - prevPos[i].z + h*h*a.z;
    // std::cout << "prev: " << prevPos[i].x << " " << prevPos[i].y << " " << prevPos[i].z << std::endl;
    // std::cout << "curr: " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << std::endl;
    // std::cout << "new: " << newPos.x << " " << newPos.y << " " << newPos.z << std::endl;
    prevPos[i] = vertices[i];
    vertices[i] = newPos;
  }

  rtcUpdateGeometryBuffer(rtcGetGeometry(g_scene, clothID), RTC_BUFFER_TYPE_VERTEX, 0);
  rtcCommitGeometry(rtcGetGeometry(g_scene, clothID));
  rtcCommitScene(g_scene);
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{

  /*if (!pause)*/ updateScene();
  //else PRINT(cur_time);
  //collision_candidates.clear();
  //rtcCollide(g_scene,g_scene,CollideFunc,nullptr);

  /* render image */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = nullptr;
  rtcReleaseDevice(g_device); g_device = nullptr;
}

} // namespace embree
