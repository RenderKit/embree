// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "tutorial_device.h"

/* the scene to render */
extern RTCScene g_scene;

/* global subdivision level for subdivision geometry */
unsigned int g_subdivision_levels = 0;

/* intensity scaling for traversal cost visualization */
float scale = 0.001f;
extern "C" bool g_changed = false;

/* stores pointer to currently used rendePixel function */
extern renderPixelFunc renderPixel;

extern float g_debug;

/* standard rendering function for each tutorial */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p);

/* renders a single pixel with eyelight shading */
Vec3fa renderPixelEyeLight(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
  else return Vec3fa(embree::abs(dot(ray.dir,normalize(ray.Ng))));
}

__noinline void setray(RTCRay& ray)
{
  ray.u = ray.v = 0.001f;
  ray.Ng = Vec3fa(0,1,0);
  ray.geomID = 0;
  ray.primID = 0;
}

/* renders a single pixel with wireframe shading */
Vec3fa renderPixelWireframe(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* return black if nothing hit */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);

  /* calculate wireframe around triangles */
  const float border = 0.05f;
  Vec3fa color = Vec3fa(1.0f);
  if (ray.u < border) color = Vec3fa(0.0f);
  if (ray.v < border) color = Vec3fa(0.0f);
  if (1.0f-ray.u-ray.v < border) color = Vec3fa(0.0f);

  /* perform eyelight shading */
  return color*Vec3fa(embree::abs(dot(ray.dir,normalize(ray.Ng))));
}

/* renders a single pixel with UV shading */
Vec3fa renderPixelUV(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
  else return Vec3fa(ray.u,ray.v,1.0f-ray.u-ray.v);
}

/* renders a single pixel with geometry normal shading */
Vec3fa renderPixelNg(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
  else return abs(normalize(Vec3fa(ray.Ng.x,ray.Ng.y,ray.Ng.z)));
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
Vec3fa renderPixelGeomID(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
  else return randomColor(ray.geomID);
}

/* geometry ID and primitive ID shading */
Vec3fa renderPixelGeomIDPrimID(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
  else return randomColor(ray.geomID ^ ray.primID);
}

/* vizualizes the traversal cost of a pixel */
Vec3fa renderPixelCycles(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  int64 c0 = get_tsc();
  rtcIntersect(g_scene,ray);
  int64 c1 = get_tsc();
  
  /* shade pixel */
  return Vec3fa((float)(c1-c0)*scale,0.0f,0.0f);
}

/* renders a single pixel with UV shading */
Vec3fa renderPixelUV16(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  for (int i=0; i<16; i++) {
    ray.tfar = inf;
    rtcIntersect(g_scene,ray);
  }

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
  else return Vec3fa(ray.u,ray.v,1.0f-ray.u-ray.v);
}

/* returns the point seen through specified pixel */
extern "C" bool device_pick(const float x,
                            const float y, 
                            const Vec3fa& vx, 
                            const Vec3fa& vy, 
                            const Vec3fa& vz, 
                            const Vec3fa& p,
                            Vec3fa& hitPos)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

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
extern "C" void device_key_pressed(int key)
{
  if (key == GLUT_KEY_F1) {
    renderPixel = renderPixelStandard;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F2) {
    renderPixel = renderPixelEyeLight;
    g_changed = true;
  }    
  else if (key == GLUT_KEY_F3) {
    renderPixel = renderPixelWireframe;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F4) {
    renderPixel = renderPixelUV;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F5) {
    renderPixel = renderPixelNg;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F6) {
    renderPixel = renderPixelGeomID;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F7) {
    renderPixel = renderPixelGeomIDPrimID;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F8) {
    renderPixel = renderPixelUV16;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F9) {
    if (renderPixel == renderPixelCycles) scale *= 1.1f;
    renderPixel = renderPixelCycles;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F10) {
    if (renderPixel == renderPixelCycles) scale *= 0.9f;
    renderPixel = renderPixelCycles;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F11) {
  if (g_subdivision_levels > 0)	
    g_subdivision_levels--;
    g_changed = true;
  }
  else if (key == GLUT_KEY_F12) {
      g_subdivision_levels++;
    g_changed = true;
  }

}

void renderTile(int taskIndex,
                int* pixels,
                const int width,
                const int height, 
                const float time,
                const Vec3fa& vx, 
                const Vec3fa& vy, 
                const Vec3fa& vz, 
                const Vec3fa& p,
                const int numTilesX, 
                const int numTilesY);

struct RenderTileTask
{
  RenderTileTask (int* pixels, const int width, const int height, const float time, 
                  const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p, const int numTilesX, const int numTilesY)
    : pixels(pixels), width(width), height(height), time(time), vx(vx), vy(vy), vz(vz), p(p), numTilesX(numTilesX), numTilesY(numTilesY) {}

public:
  int* pixels;
  const int width;
  const int height;
  const float time;
  const Vec3fa vx;
  const Vec3fa vy;
  const Vec3fa vz;
  const Vec3fa p;
  const int numTilesX;
  const int numTilesY;
};

void renderTile_parallel(RenderTileTask* task, size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) {
  renderTile(taskIndex,task->pixels,task->width,task->height,task->time,task->vx,task->vy,task->vz,task->p,task->numTilesX,task->numTilesY);
}

void launch_renderTile (int numTiles, 
                        int* pixels, const int width, const int height, const float time, 
                        const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p, const int numTilesX, const int numTilesY)
{
  TaskScheduler::EventSync event;
  RenderTileTask parms(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY);
  TaskScheduler::Task task(&event,(TaskScheduler::runFunction)renderTile_parallel,&parms,numTiles,NULL,NULL,"render");
  TaskScheduler::addTask(-1,TaskScheduler::GLOBAL_FRONT,&task);
  event.sync();
}

typedef void (*animateSphereFunc) (int taskIndex, Vertex* vertices, 
				   const float rcpNumTheta,
				   const float rcpNumPhi,
				   const Vec3fa& pos, 
				   const float r,
				   const float f);

struct AnimateSphereTask
{
  AnimateSphereTask (animateSphereFunc func,
		     Vertex* vertices, 
		     const float rcpNumTheta,
		     const float rcpNumPhi,
		     const Vec3fa& pos, 
		     const float r,
		     const float f)
    : func(func), vertices(vertices), rcpNumTheta(rcpNumTheta), rcpNumPhi(rcpNumPhi), pos(pos), r(r), f(f) {}

public:
  animateSphereFunc func;
  Vertex* vertices;
  const float rcpNumTheta;
  const float rcpNumPhi;
  const Vec3fa pos;
  const float r;
  const float f;
};

void animateSphere_parallel(AnimateSphereTask* task, size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) {
  task->func(taskIndex,task->vertices,task->rcpNumTheta,task->rcpNumPhi,task->pos,task->r,task->f);
}

void launch_animateSphere(animateSphereFunc func,
			  int taskSize,
			  Vertex* vertices, 
			  const float rcpNumTheta,
			  const float rcpNumPhi,
			  const Vec3fa& pos, 
			  const float r,
			  const float f)
{
  TaskScheduler::EventSync event;
  AnimateSphereTask parms(func,vertices,rcpNumTheta,rcpNumPhi,pos,r,f);
  TaskScheduler::Task task(&event,(TaskScheduler::runFunction)animateSphere_parallel,&parms,taskSize,NULL,NULL,"render");
  TaskScheduler::addTask(-1,TaskScheduler::GLOBAL_FRONT,&task);
  event.sync();
}
