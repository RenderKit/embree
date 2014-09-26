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

#include "tutorial_device.isph"

/* the scene to render */
extern RTCScene g_scene;

uniform unsigned int g_numThreads = 0;
uniform unsigned int g_subdivision_levels = 0;

/* intensity scaling for traversal cost visualization */
uniform float scale = 0.0001f;
uniform bool g_changed = false;

/* stores pointer to currently used rendePixel function */
extern renderPixelFunc renderPixel;

/* standard rendering function for each tutorial */
Vec3f renderPixelStandard(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p);

/* renders a single pixel with eyelight shading */
Vec3f renderPixelEyeLight(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
  else return make_Vec3f(abs(dot(ray.dir,normalize(ray.Ng))));
  //else return make_Vec3f(clamp(dot(ray.dir,normalize(ray.Ng))));
}

/* renders a single pixel with wireframe shading */
Vec3f renderPixelWireframe(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* return black if nothing hit */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);

  /* calculate wireframe around triangles */
  const float border = 0.05f;
  Vec3f color = make_Vec3f(1.0f);
  if (ray.u < border) color = make_Vec3f(0.0f);
  if (ray.v < border) color = make_Vec3f(0.0f);
  if (1.0f-ray.u-ray.v < border) color = make_Vec3f(0.0f);

  /* perform eyelight shading */
  return color*make_Vec3f(abs(dot(ray.dir,normalize(ray.Ng))));
}

/* renders a single pixel with UV shading */
Vec3f renderPixelUV(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
  else return make_Vec3f(ray.u,ray.v,1.0f-ray.u-ray.v);
}

/* renders a single pixel with geometry normal shading */
Vec3f renderPixelNg(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
  else return abs(normalize(make_Vec3f(ray.Ng.x,ray.Ng.y,ray.Ng.z)));
}

Vec3f randomColor(const int ID) 
{
  int r = ((ID+13)*17*23) & 255;
  int g = ((ID+15)*11*13) & 255;
  int b = ((ID+17)* 7*19) & 255;
  const float oneOver255f = 1.f/255.f;
  return make_Vec3f(r*oneOver255f,g*oneOver255f,b*oneOver255f);
}

/* geometry ID shading */
Vec3f renderPixelGeomID(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
  else return randomColor(ray.geomID);
}

/* geometry ID and primitive ID shading */
Vec3f renderPixelGeomIDPrimID(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
  else return randomColor(ray.geomID ^ ray.primID);
}

/* vizualizes the traversal cost of a pixel */
Vec3f renderPixelCycles(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  ray.time = 0;

  /* intersect ray with scene */
  uniform int64 c0 = get_tsc();
  rtcIntersect(g_scene,ray);
  uniform int64 c1 = get_tsc();
  
  /* shade pixel */
  return make_Vec3f((uniform float)(c1-c0)*scale,0.0f,0.0f);
}

/* renders a single pixel with UV shading and shoot ray 16 times for measurements */
Vec3f renderPixelUV16(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  ray.time = 0;

  /* intersect ray with scene */
  for (uniform int i=0; i<16; i++) {
    ray.tfar = inf;
    rtcIntersect(g_scene,ray);
  }

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
  else return make_Vec3f(ray.u,ray.v,1.0f-ray.u-ray.v);
}

/* returns the point seen through specified pixel */
export uniform bool device_pick(const uniform float x,
                                const uniform float y, 
                                const uniform Vec3f& vx, 
                                const uniform Vec3f& vy, 
                                const uniform Vec3f& vz, 
                                const uniform Vec3f& p,
                                uniform Vec3f& hitPos)
{
  /* initialize ray */
  uniform RTCRay1 ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect1(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
    hitPos = make_Vec3f(0.0f,0.0f,0.0f);
    return false;
  }
  else {
    hitPos = ray.org + ray.tfar*ray.dir;
    return true;
  }
}

/* called when a key is pressed */
export void device_key_pressed(uniform int key)
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


task void task_getNumHWThreads()
{
 g_numThreads = threadCount;
}

uniform unsigned int getNumHWThreads()
{
 launch[ 1 ] task_getNumHWThreads(); sync;
 return g_numThreads;
}
