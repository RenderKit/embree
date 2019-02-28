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

#include "../math/random_sampler.h"
#include "../math/sampling.h"
#include "tutorial_device.h"
#include "scene_device.h"

namespace embree {

/* the scene to render */
extern RTCScene g_scene;
extern "C" ISPCScene* g_ispc_scene;

/* intensity scaling for traversal cost visualization */
extern "C" float scale;
extern "C" bool g_changed;

extern "C" float g_debug;

/* stores pointer to currently used rendePixel function */
renderTileFunc renderTile;

/* renders a single pixel with eyelight shading */
Vec3fa renderPixelEyeLight(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
    return Vec3fa(0.0f);
  else if (dot(ray.dir,ray.Ng) < 0.0f)
    return Vec3fa(0.0f,abs(dot(ray.dir,normalize(ray.Ng))),0.0f);
  else
    return Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))),0.0f,0.0f);
}

void renderTileEyeLight(int taskIndex,
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
    Vec3fa color = renderPixelEyeLight((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* renders a single pixel with occlusion shading */
Vec3fa renderPixelOcclusion(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcOccluded1(g_scene,&context.context,RTCRay_(ray));
  RayStats_addRay(stats);

  /* return black if nothing hit */
  if (ray.tfar >= 0.0f) 
    return Vec3fa(0.0f,0.0f,0.0f);
  else 
    return Vec3fa(1.0f,1.0f,1.0f);
}

void renderTileOcclusion(int taskIndex,
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
    Vec3fa color = renderPixelOcclusion((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* renders a single pixel with UV shading */
Vec3fa renderPixelUV(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f,0.0f,1.0f);
  else return Vec3fa(ray.u,ray.v,1.0f-ray.u-ray.v);
}

void renderTileUV(int taskIndex,
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
    Vec3fa color = renderPixelUV((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

unsigned int render_texcoords_mode = 0;

/* renders a single pixel with TexCoords shading */
Vec3fa renderPixelTexCoords(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
    return Vec3fa(0.0f,0.0f,1.0f);

  else if (g_ispc_scene)
  {
    Vec2f st = Vec2f(0,0);
    unsigned int geomID = ray.geomID; {
      RTCGeometry geometry = rtcGetGeometry(g_scene,geomID);
      rtcInterpolate0(geometry,ray.primID,ray.u,ray.v,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,2,&st.x,2);
    }
    if (render_texcoords_mode%2 == 0)
      return Vec3fa(st.x,st.y,0.0f);
    else if (render_texcoords_mode%2 == 1)
      return ((int)(10.0f*st.x)+(int)(10.0f*st.y)) % 2 == 0 ? Vec3fa(1,0,0) : Vec3fa(0,1,0);
  }

  return Vec3fa(1.0f);
}

void renderTileTexCoords(int taskIndex,
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
    Vec3fa color = renderPixelTexCoords((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* renders a single pixel with geometry normal shading */
Vec3fa renderPixelNg(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f,0.0f,1.0f);
  else return abs(normalize(Vec3fa(ray.Ng.x,ray.Ng.y,ray.Ng.z)));
  //else return normalize(Vec3fa(ray.Ng.x,ray.Ng.y,ray.Ng.z));
}

void renderTileNg(int taskIndex,
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
    Vec3fa color = renderPixelNg((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

Vec3fa randomColor(const int ID)
{
  int r = ((ID+13)*17*23) & 255;
  int g = ((ID+15)*11*13) & 255;
  int b = ((ID+17)* 7*19) & 255;
  const float oneOver255f = 1.f/255.f;
  return Vec3fa(r*oneOver255f,g*oneOver255f,b*oneOver255f);
}

/* geometry ID shading */
Vec3fa renderPixelGeomID(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
  else return randomColor(ray.geomID);
}

void renderTileGeomID(int taskIndex,
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
    Vec3fa color = renderPixelGeomID((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* geometry ID and primitive ID shading */
Vec3fa renderPixelGeomIDPrimID(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
  else return randomColor(ray.geomID ^ ray.primID)*Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))));
}

void renderTileGeomIDPrimID(int taskIndex,
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
    Vec3fa color = renderPixelGeomIDPrimID((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* vizualizes the traversal cost of a pixel */
Vec3fa renderPixelCycles(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  int64_t c0 = get_tsc();
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  int64_t c1 = get_tsc();
  RayStats_addRay(stats);

  /* shade pixel */
  return Vec3fa((float)(c1-c0)*scale,0.0f,0.0f);
}

void renderTileCycles(int taskIndex,
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
    Vec3fa color = renderPixelCycles((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* renders a single pixel with ambient occlusion */
Vec3fa renderPixelAmbientOcclusion(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);

  Vec3fa Ng = normalize(ray.Ng);
  Vec3fa Nf = faceforward(Ng,ray.dir,Ng);
  Vec3fa col = Vec3fa(min(1.f,.3f+.8f*abs(dot(Ng,normalize(ray.dir)))));

  /* calculate hit point */
  float intensity = 0;
  Vec3fa hitPos = ray.org + ray.tfar * ray.dir;

#define AMBIENT_OCCLUSION_SAMPLES 64
  /* trace some ambient occlusion rays */
  RandomSampler sampler;
  RandomSampler_init(sampler, (int)x, (int)y, 0);
  for (int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
  {
    Vec2f sample = RandomSampler_get2D(sampler);
    Sample3f dir = cosineSampleHemisphere(sample.x,sample.y,Nf);

    /* initialize shadow ray */
    Ray shadow;
    shadow.org = hitPos;
    shadow.dir = dir.v;
    shadow.tnear() = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time() = g_debug;

    /* trace shadow ray */
    IntersectContext context;
    InitIntersectionContext(&context);
    rtcOccluded1(g_scene,&context.context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      intensity += 1.0f;
  }
  intensity *= 1.0f/AMBIENT_OCCLUSION_SAMPLES;

  /* shade pixel */
  return col * intensity;
}

void renderTileAmbientOcclusion(int taskIndex,
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
    Vec3fa color = renderPixelAmbientOcclusion((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* differential visualization */
static int differentialMode = 0;
Vec3fa renderPixelDifferentials(float x, float y, const ISPCCamera& camera, RayStats& stats)
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
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);

  /* calculate differentials */
  float eps = 0.001f/16.0f;
  Vec3fa P00, P01, P10, P11;
  Vec3fa dP00du, dP01du, dP10du, dP11du;
  Vec3fa dP00dv, dP01dv, dP10dv, dP11dv;
  Vec3fa dPdu1, dPdv1, ddPdudu1, ddPdvdv1, ddPdudv1;
  unsigned int geomID = ray.geomID; {
    RTCGeometry geometry = rtcGetGeometry(g_scene,geomID);
    rtcInterpolate1(geometry,ray.primID,ray.u+0.f,ray.v+0.f,RTC_BUFFER_TYPE_VERTEX,0,&P00.x,&dP00du.x,&dP00dv.x,3);
    rtcInterpolate1(geometry,ray.primID,ray.u+0.f,ray.v+eps,RTC_BUFFER_TYPE_VERTEX,0,&P01.x,&dP01du.x,&dP01dv.x,3);
    rtcInterpolate1(geometry,ray.primID,ray.u+eps,ray.v+0.f,RTC_BUFFER_TYPE_VERTEX,0,&P10.x,&dP10du.x,&dP10dv.x,3);
    rtcInterpolate1(geometry,ray.primID,ray.u+eps,ray.v+eps,RTC_BUFFER_TYPE_VERTEX,0,&P11.x,&dP11du.x,&dP11dv.x,3);
    rtcInterpolate2(geometry,ray.primID,ray.u,ray.v,RTC_BUFFER_TYPE_VERTEX,0,nullptr,&dPdu1.x,&dPdv1.x,&ddPdudu1.x,&ddPdvdv1.x,&ddPdudv1.x,3);
  }
  Vec3fa dPdu0 = (P10-P00)/eps;
  Vec3fa dPdv0 = (P01-P00)/eps;
  Vec3fa ddPdudu0 = (dP10du-dP00du)/eps;
  Vec3fa ddPdvdv0 = (dP01dv-dP00dv)/eps;
  Vec3fa ddPdudv0 = (dP01du-dP00du)/eps;

  Vec3fa color = Vec3fa(0.0f);
  switch (differentialMode)
  {
  case  0: color = dPdu0; break;
  case  1: color = dPdu1; break;
  case  2: color = 10.0f*(dPdu1-dPdu0); break;

  case  3: color = dPdv0; break;
  case  4: color = dPdv1; break;
  case  5: color = 10.0f*(dPdv1-dPdv0); break;

  case  6: color = ddPdudu0; break;
  case  7: color = ddPdudu1; break;
  case  8: color = 10.0f*(ddPdudu1-ddPdudu0); break;

  case  9: color = ddPdvdv0; break;
  case 10: color = ddPdvdv1; break;
  case 11: color = 10.0f*(ddPdvdv1-ddPdvdv0); break;

  case 12: color = ddPdudv0; break;
  case 13: color = ddPdudv1; break;
  case 14: color = 10.0f*(ddPdudv1-ddPdudv0); break;

  case 15: {
    color.x = length(dnormalize(cross(dPdu1,dPdv1),cross(ddPdudu1,dPdv1)+cross(dPdu1,ddPdudv1)))/length(dPdu1);
    color.y = length(dnormalize(cross(dPdu1,dPdv1),cross(ddPdudv1,dPdv1)+cross(dPdu1,ddPdvdv1)))/length(dPdv1);
    color.z = 0.0f;
    break;
  }
  case 16: {
    float Cu = length(dnormalize(cross(dPdu1,dPdv1),cross(ddPdudu1,dPdv1)+cross(dPdu1,ddPdudv1)))/length(dPdu1);
    float Cv = length(dnormalize(cross(dPdu1,dPdv1),cross(ddPdudv1,dPdv1)+cross(dPdu1,ddPdvdv1)))/length(dPdv1);
    color = Vec3fa(sqrt(Cu*Cu + Cv*Cv));
    break;
  }
  }
  return clamp(color,Vec3fa(0.0f),Vec3fa(1.0f));
}

void renderTileDifferentials(int taskIndex,
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
    Vec3fa color = renderPixelDifferentials((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* returns the point seen through specified pixel */
extern "C" bool device_pick(const float x,
                                const float y,
                                const ISPCCamera& camera,
                                Vec3fa& hitPos)
{
  /* initialize ray */
  Ray1 ray;
  ray.org = Vec3fa(camera.xfm.p);
  ray.dir = Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear() = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time() = g_debug;

  /* intersect ray with scene */
  IntersectContext context;
  InitIntersectionContext(&context);
  rtcIntersect1(g_scene,&context.context,RTCRayHit1_(ray));

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
    hitPos = Vec3fa(0.0f,0.0f,0.0f);
    return false;
  }
  else {
    hitPos = ray.org + ray.tfar*ray.dir;
    return true;
  }
}

/* called when a key is pressed */
extern "C" void device_key_pressed_default(int key)
{
  if (key == GLFW_KEY_F1) {
    renderTile = renderTileStandard;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F2) {
    renderTile = renderTileEyeLight;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F3) {
    renderTile = renderTileOcclusion;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F4) {
    renderTile = renderTileUV;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F5) {
    renderTile = renderTileNg;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F6) {
    renderTile = renderTileGeomID;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F7) {
    renderTile = renderTileGeomIDPrimID;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F8) {
    if (renderTile == renderTileTexCoords) render_texcoords_mode++;
    renderTile = renderTileTexCoords;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F9) {
    if (renderTile == renderTileCycles) scale *= 2.0f;
    renderTile = renderTileCycles;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F10) {
    if (renderTile == renderTileCycles) scale *= 0.5f;
    renderTile = renderTileCycles;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F11) {
    renderTile = renderTileAmbientOcclusion;
    g_changed = true;
  }
  else if (key == GLFW_KEY_F12) {
    if (renderTile == renderTileDifferentials) {
      differentialMode = (differentialMode+1)%17;
    } else {
      renderTile = renderTileDifferentials;
      differentialMode = 0;
    }
    g_changed = true;
  }
}

/* called when a key is pressed */
void (* key_pressed_handler)(int key) = nullptr;

extern "C" void device_key_pressed(int key) {
  if (key_pressed_handler) key_pressed_handler(key);
}

Vec2f getTextureCoordinatesSubdivMesh(void* _mesh, const unsigned int primID, const float u, const float v)
{
  ISPCSubdivMesh *mesh = (ISPCSubdivMesh *)_mesh;
  Vec2f st;
  st.x = u;
  st.y = v;
  if (mesh && mesh->texcoord_indices)
    {
      assert(primID < mesh->numFaces);
      const unsigned int face_offset = mesh->face_offsets[primID];
      if (mesh->verticesPerFace[primID] == 3)
	{
	  const unsigned int t0 = mesh->texcoord_indices[face_offset+0];
	  const unsigned int t1 = mesh->texcoord_indices[face_offset+1];
	  const unsigned int t2 = mesh->texcoord_indices[face_offset+2];
	  const Vec2f txt0 = mesh->texcoords[t0];
	  const Vec2f txt1 = mesh->texcoords[t1];
	  const Vec2f txt2 = mesh->texcoords[t2];
	  const float w = 1.0f - u - v;
	  st = w * txt0 + u * txt1 + v * txt2;
	}
      else if (mesh->verticesPerFace[primID] == 4)
	{
	  const unsigned int t0 = mesh->texcoord_indices[face_offset+0];
	  const unsigned int t1 = mesh->texcoord_indices[face_offset+1];
	  const unsigned int t2 = mesh->texcoord_indices[face_offset+2];
	  const unsigned int t3 = mesh->texcoord_indices[face_offset+3];
	  const Vec2f txt0 = mesh->texcoords[t0];
	  const Vec2f txt1 = mesh->texcoords[t1];
	  const Vec2f txt2 = mesh->texcoords[t2];
	  const Vec2f txt3 = mesh->texcoords[t3];
	  const float u0 = u;
	  const float v0 = v;
	  const float u1 = 1.0f - u;
	  const float v1 = 1.0f - v;
	  st = u1*v1 * txt0 + u0*v1* txt1 + u0*v0 * txt2 + u1*v0* txt3;
	}
#if defined(_DEBUG)
      else
	PRINT("not supported");
#endif
    }
  return st;
}

float getTextureTexel1f(const Texture* texture, float s, float t)
{
  if (!texture) return 0.0f;

  int iu = (int)floor(s * (float)(texture->width));
  iu = iu % texture->width; if (iu < 0) iu += texture->width;
  int iv = (int)floor(t * (float)(texture->height));
  iv = iv % texture->height; if (iv < 0) iv += texture->height;

  if (texture->format == Texture::FLOAT32)
  {
    float *data = (float *)texture->data;
    return data[iv*texture->width + iu];
  }
  else if (texture->format == Texture::RGBA8)
  {
    const int offset = (iv * texture->width + iu) * 4;
    unsigned char * t = (unsigned char*)texture->data;
    return t[offset+0]*(1.0f/255.0f);
  }
  return 0.0f;
}

Vec3fa getTextureTexel3f(const Texture* texture, float s, float t)
{
  if (!texture) return Vec3fa(0.0f,0.0f,0.0f);

  int iu = (int)floor(s * (float)(texture->width));
  iu = iu % texture->width; if (iu < 0) iu += texture->width;
  int iv = (int)floor(t * (float)(texture->height));
  iv = iv % texture->height; if (iv < 0) iv += texture->height;

  if (texture->format == Texture::RGBA8)
  {
    const int offset = (iv * texture->width + iu) * 4;
    unsigned char * t = (unsigned char*)texture->data;
    const unsigned char  r = t[offset+0];
    const unsigned char  g = t[offset+1];
    const unsigned char  b = t[offset+2];
    return Vec3fa(  (float)r * 1.0f/255.0f, (float)g * 1.0f/255.0f, (float)b * 1.0f/255.0f );
  }
  return Vec3fa(0.0f,0.0f,0.0f);
}

} // namespace embree
