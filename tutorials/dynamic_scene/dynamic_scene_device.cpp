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

#if 0
const int numSpheres = 1000;
const int numPhi = 5;
#else
const int numSpheres = 20;
const int numPhi = 120;
//const int numPhi = 256;
#endif
const int numTheta = 2*numPhi;

/* scene data */
RTCScene g_scene = nullptr;
Vec3fa position[numSpheres];
Vec3fa colors[numSpheres+1];
float radius[numSpheres];
int disabledID = -1;

/* adds a sphere to the scene */
unsigned int createSphere (RTCBuildQuality quality, const Vec3fa& pos, const float r)
{
  /* create a triangulated sphere */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetGeometryBuildQuality(geom, quality);

  /* map triangle and vertex buffer */
  Vertex*   vertices  = (Vertex*  ) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),numTheta*(numPhi+1));
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2*numTheta*(numPhi-1));

  /* create sphere geometry */
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
      v.x = pos.x + r*sin(phif)*sin(thetaf);
      v.y = pos.y + r*cos(phif);
      v.z = pos.z + r*sin(phif)*cos(thetaf);
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
  unsigned int geomID = rtcAttachGeometry(g_scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
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
  unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);
  rtcSetSceneFlags(g_scene,RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
  rtcSetSceneBuildQuality(g_scene,RTC_BUILD_QUALITY_LOW);

  /* create some triangulated spheres */
  for (int i=0; i<numSpheres; i++)
  {
    const float phi = i*2.0f*float(pi)/numSpheres;
    const float r = 2.0f*float(pi)/numSpheres;
    const Vec3fa p = 2.0f*Vec3fa(sin(phi),0.0f,-cos(phi));
    //RTCBuildQuality quality = i%3 == 0 ? RTC_BUILD_QUALITY_MEDIUM : i%3 == 1 ? RTC_BUILD_QUALITY_REFIT : RTC_BUILD_QUALITY_LOW;
    RTCBuildQuality quality = i%2 ? RTC_BUILD_QUALITY_REFIT : RTC_BUILD_QUALITY_LOW;
    //RTCBuildQuality quality = RTC_BUILD_QUALITY_REFIT;
    int id = createSphere(quality,p,r);
    position[id] = p;
    radius[id] = r;
    colors[id].x = (i%16+1)/17.0f;
    colors[id].y = (i%8+1)/9.0f;
    colors[id].z = (i%4+1)/5.0f;
  }

  /* add ground plane to scene */
  int id = addGroundPlane(g_scene);
  colors[id] = Vec3fa(1.0f,1.0f,1.0f);

  /* commit changes to scene */
  rtcCommitScene (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* animates the sphere */
void animateSphere (int taskIndex, int threadIndex, Vertex* vertices,
                         const float rcpNumTheta,
                         const float rcpNumPhi,
                         const Vec3fa& pos,
                         const float r,
                         const float f)
{
  int phi = taskIndex;
  for (unsigned int theta=0; theta<numTheta; theta++)
  {
    Vertex* v = &vertices[phi*numTheta+theta];
    const float phif   = phi*float(pi)*rcpNumPhi;
    const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
    v->x = pos.x + r*sin(f*phif)*sin(thetaf);
    v->y = pos.y + r*cos(phif);
    v->z = pos.z + r*sin(f*phif)*cos(thetaf);
  }
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  rtcIntersect1(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3fa diffuse = colors[ray.geomID];
    color = color + diffuse*0.1f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    rtcOccluded1(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
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

/* animates a sphere */
void animateSphere (int id, float time)
{
  /* animate vertices */
  RTCGeometry geom = rtcGetGeometry(g_scene,id);
  Vertex* vertices = (Vertex*) rtcGetGeometryBufferData(geom,RTC_BUFFER_TYPE_VERTEX,0);
  const float rcpNumTheta = rcp((float)numTheta);
  const float rcpNumPhi   = rcp((float)numPhi);
  const Vec3fa pos = position[id];
  const float r = radius[id];
  const float f = 2.0f*(1.0f+0.5f*sin(time));

  /* loop over all vertices */
#if 1 // enables parallel execution
  parallel_for(size_t(0),size_t(numPhi+1),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      animateSphere((int)i,threadIndex,vertices,rcpNumTheta,rcpNumPhi,pos,r,f);
  }); 
#else
  for (unsigned int phi=0; phi<numPhi+1; phi++) for (int theta=0; theta<numTheta; theta++)
  {
    Vertex* v = &vertices[phi*numTheta+theta];
    const float phif   = phi*float(pi)*rcpNumPhi;
    const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
    v->x = pos.x+r*sin(f*phif)*sin(thetaf);
    v->y = pos.y+r*cos(phif);
    v->z = pos.z+r*sin(f*phif)*cos(thetaf);
  }
#endif

  /* commit mesh */
  rtcUpdateGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0);
  rtcCommitGeometry(geom);
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  /* animate sphere */
  for (int i=0; i<numSpheres; i++)
    animateSphere(i,time+i);

  /* commit changes to scene */
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
}

} // namespace embree
