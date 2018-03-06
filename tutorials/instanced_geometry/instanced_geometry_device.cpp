// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "../common/tutorial/tutorial_device.h"

namespace embree {

const int numPhi = 5;
const int numTheta = 2*numPhi;

void renderTileStandardStream(int taskIndex,
                              int threadIndex,
                              int* pixels,
                              const unsigned int width,
                              const unsigned int height,
                              const float time,
                              const ISPCCamera& camera,
                              const int numTilesX,
                              const int numTilesY);

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

/* scene data */
RTCScene g_scene  = nullptr;
RTCScene g_scene1 = nullptr;

RTCGeometry g_instance0 = nullptr;
RTCGeometry g_instance1 = nullptr;
RTCGeometry g_instance2 = nullptr;
RTCGeometry g_instance3 = nullptr;
AffineSpace3fa instance_xfm[4];
LinearSpace3fa normal_xfm[4];

Vec3fa colors[4][4];

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(g_scene,RTC_BUILD_QUALITY_LOW);
  rtcSetSceneFlags(g_scene,RTC_SCENE_FLAG_DYNAMIC);

  /* create scene with 4 triangulated spheres */
  g_scene1 = rtcNewScene(g_device);
  createTriangulatedSphere(g_scene1,Vec3fa( 0, 0,+1),0.5f);
  createTriangulatedSphere(g_scene1,Vec3fa(+1, 0, 0),0.5f);
  createTriangulatedSphere(g_scene1,Vec3fa( 0, 0,-1),0.5f);
  createTriangulatedSphere(g_scene1,Vec3fa(-1, 0, 0),0.5f);
  rtcCommitScene (g_scene1);

  /* instantiate geometry */
  g_instance0 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance0,g_scene1);
  rtcSetGeometryTimeStepCount(g_instance0,1);
  g_instance1 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance1,g_scene1);
  rtcSetGeometryTimeStepCount(g_instance1,1);
  g_instance2 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance2,g_scene1);
  rtcSetGeometryTimeStepCount(g_instance2,1);
  g_instance3 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance3,g_scene1);
  rtcSetGeometryTimeStepCount(g_instance3,1);
  rtcAttachGeometry(g_scene,g_instance0);
  rtcAttachGeometry(g_scene,g_instance1);
  rtcAttachGeometry(g_scene,g_instance2);
  rtcAttachGeometry(g_scene,g_instance3);
  rtcReleaseGeometry(g_instance0);
  rtcReleaseGeometry(g_instance1);
  rtcReleaseGeometry(g_instance2);
  rtcReleaseGeometry(g_instance3);
  createGroundPlane(g_scene);

  /* set all colors */
  colors[0][0] = Vec3fa(0.25f, 0.f, 0.f);
  colors[0][1] = Vec3fa(0.50f, 0.f, 0.f);
  colors[0][2] = Vec3fa(0.75f, 0.f, 0.f);
  colors[0][3] = Vec3fa(1.00f, 0.f, 0.f);

  colors[1][0] = Vec3fa(0.f, 0.25f, 0.f);
  colors[1][1] = Vec3fa(0.f, 0.50f, 0.f);
  colors[1][2] = Vec3fa(0.f, 0.75f, 0.f);
  colors[1][3] = Vec3fa(0.f, 1.00f, 0.f);

  colors[2][0] = Vec3fa(0.f, 0.f, 0.25f);
  colors[2][1] = Vec3fa(0.f, 0.f, 0.50f);
  colors[2][2] = Vec3fa(0.f, 0.f, 0.75f);
  colors[2][3] = Vec3fa(0.f, 0.f, 1.00f);

  colors[3][0] = Vec3fa(0.25f, 0.25f, 0.f);
  colors[3][1] = Vec3fa(0.50f, 0.50f, 0.f);
  colors[3][2] = Vec3fa(0.75f, 0.75f, 0.f);
  colors[3][3] = Vec3fa(1.00f, 1.00f, 0.f);

  /* set start render mode */
  if (g_mode == MODE_NORMAL) renderTile = renderTileStandard;
  else                       renderTile = renderTileStandardStream;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  rtcIntersect1(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* calculate shading normal in world space */
    Vec3fa Ns = ray.Ng;
    if (ray.instID != RTC_INVALID_GEOMETRY_ID)
      Ns = xfmVector(normal_xfm[ray.instID],Ns);
    Ns = normalize(Ns);

    /* calculate diffuse color of geometries */
    Vec3fa diffuse = Vec3fa(1,1,1);
    if (ray.instID != RTC_INVALID_GEOMETRY_ID)
      diffuse = colors[ray.instID][ray.geomID];
    color = color + diffuse*0.5;

    /* initialize shadow ray */
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    rtcOccluded1(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
  }
  return color;
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

/* renders a single screen tile */
void renderTileStandardStream(int taskIndex,
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

  RayStats& stats = g_stats[threadIndex];

  Ray primary_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Ray shadow_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Vec3fa color_stream[TILE_SIZE_X*TILE_SIZE_Y];
  bool valid_stream[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  int N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    

    /* initialize variables */
    color_stream[N] = Vec3fa(0.0f);
    bool mask = 1; { valid_stream[N] = mask; }

    /* initialize ray */
    Ray& primary = primary_stream[N];
    mask = 1; { // invalidates inactive rays
      primary.tnear() = mask ? 0.0f         : (float)(pos_inf);
      primary.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(primary, Vec3fa(camera.xfm.p), Vec3fa(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), primary.tnear(), primary.tfar);

    N++;
    RayStats_addRay(stats);
  }

  Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

  /* trace rays */
  RTCIntersectContext primary_context;
  rtcInitIntersectContext(&primary_context);
  primary_context.flags = g_iflags_coherent;
  rtcIntersect1M(g_scene,&primary_context,(RTCRayHit*)&primary_stream,N,sizeof(Ray));

  /* terminate rays and update color */
  N = -1;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    N++;
    /* ISPC workaround for mask == 0 */
    

    /* invalidate shadow rays by default */
    Ray& shadow = shadow_stream[N];
    {
      shadow.tnear() = (float)(pos_inf);
      shadow.tfar  = (float)(neg_inf);
    }

    /* ignore invalid rays */
    if (valid_stream[N] == false) continue;

    /* terminate rays that hit nothing */
    if (primary_stream[N].geomID == RTC_INVALID_GEOMETRY_ID) {
      valid_stream[N] = false;
      continue;
    }

    /* calculate shading normal in world space */
    Ray& primary = primary_stream[N];
    Vec3fa Ns = primary.Ng;
    if (primary.instID != RTC_INVALID_GEOMETRY_ID)
      Ns = xfmVector(normal_xfm[primary.instID],Ns);
    Ns = normalize(Ns);

    /* calculate diffuse color of geometries */
    Vec3fa diffuse = Vec3fa(1,1,1);
    if (primary.instID != RTC_INVALID_GEOMETRY_ID)
      diffuse = colors[primary.instID][primary.geomID];
    color_stream[N] = color_stream[N] + diffuse*0.5;

    /* initialize shadow ray tnear/tfar */
    bool mask = 1; {
      shadow.tnear() = mask ? 0.001f       : (float)(pos_inf);
      shadow.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(shadow, primary.org + primary.tfar*primary.dir, neg(lightDir), shadow.tnear(), shadow.tfar);

    RayStats_addShadowRay(stats);
  }
  N++;

  /* trace shadow rays */
  RTCIntersectContext shadow_context;
  rtcInitIntersectContext(&shadow_context);
  shadow_context.flags = g_iflags_coherent;
  rtcOccluded1M(g_scene,&shadow_context,(RTCRay*)&shadow_stream,N,sizeof(Ray));

  /* add light contribution */
  N = -1;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    N++;
    /* ISPC workaround for mask == 0 */
    

    /* ignore invalid rays */
    if (valid_stream[N] == false) continue;

    /* calculate shading normal in world space */
    Ray& primary = primary_stream[N];
    Vec3fa Ns = primary.Ng;
    if (primary.instID != RTC_INVALID_GEOMETRY_ID)
      Ns = xfmVector(normal_xfm[primary.instID],Ns);
    Ns = normalize(Ns);

    /* calculate diffuse color of geometries */
    Vec3fa diffuse = Vec3fa(1,1,1);
    if (primary.instID != RTC_INVALID_GEOMETRY_ID)
      diffuse = colors[primary.instID][primary.geomID];

    /* add light contrinution */
    Ray& shadow = shadow_stream[N];
    if (shadow.tfar >= 0.0f) {
      color_stream[N] = color_stream[N] + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
    }
  }
  N++;

  /* framebuffer writeback */
  N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color_stream[N].x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color_stream[N].y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color_stream[N].z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
    N++;
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

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  float t0 = 0.7f*time;
  float t1 = 1.5f*time;

  /* rotate instances around themselves */
  LinearSpace3fa xfm;
  xfm.vx = Vec3fa(cos(t1),0,sin(t1));
  xfm.vy = Vec3fa(0,1,0);
  xfm.vz = Vec3fa(-sin(t1),0,cos(t1));

  /* calculate transformations to move instances in cirle */
  for (int i=0; i<4; i++) {
    float t = t0+i*2.0f*float(pi)/4.0f;
    instance_xfm[i] = AffineSpace3fa(xfm,2.2f*Vec3fa(+cos(t),0.0f,+sin(t)));
  }

  /* calculate transformations to properly transform normals */
  for (int i=0; i<4; i++)
    normal_xfm[i] = transposed(rcp(instance_xfm[i].l));

  /* set instance transformations */
  rtcSetGeometryTransform(g_instance0,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&instance_xfm[0]);
  rtcSetGeometryTransform(g_instance1,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&instance_xfm[1]);
  rtcSetGeometryTransform(g_instance2,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&instance_xfm[2]);
  rtcSetGeometryTransform(g_instance3,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&instance_xfm[3]);

  /* update scene */
  rtcCommitGeometry(g_instance0);
  rtcCommitGeometry(g_instance1);
  rtcCommitGeometry(g_instance2);
  rtcCommitGeometry(g_instance3);
  rtcCommitScene (g_scene);

  /* render all pixels */
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
  rtcReleaseScene (g_scene1); g_scene1 = nullptr;
}

} // namespace embree
