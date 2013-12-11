// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "tutorial/tutorial_device.h"
#include "sys/taskscheduler.h"
using namespace embree;

/* scene data */
RTCScene scene = NULL;
Vec3fa* colors = NULL;

/* adds a cube to the scene */
unsigned addCube (RTCScene scene_i)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  unsigned mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 12, 8);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -1; vertices[0].y = -1; vertices[0].z = -1; 
  vertices[1].x = -1; vertices[1].y = -1; vertices[1].z = +1; 
  vertices[2].x = -1; vertices[2].y = +1; vertices[2].z = -1; 
  vertices[3].x = -1; vertices[3].y = +1; vertices[3].z = +1; 
  vertices[4].x = +1; vertices[4].y = -1; vertices[4].z = -1; 
  vertices[5].x = +1; vertices[5].y = -1; vertices[5].z = +1; 
  vertices[6].x = +1; vertices[6].y = +1; vertices[6].z = -1; 
  vertices[7].x = +1; vertices[7].y = +1; vertices[7].z = +1; 
  rtcUnmapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER);

  /* create triangle color array */
  colors = (Vec3fa*) alignedMalloc(12*sizeof(Vec3fa[12]));

  /* set triangles and colors */
  int tri = 0;
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  
  // left side
  colors[tri] = Vec3fa(1,0,0); triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 1; tri++;
  colors[tri] = Vec3fa(1,0,0); triangles[tri].v0 = 1; triangles[tri].v1 = 2; triangles[tri].v2 = 3; tri++;

  // right side
  colors[tri] = Vec3fa(0,1,0); triangles[tri].v0 = 4; triangles[tri].v1 = 5; triangles[tri].v2 = 6; tri++;
  colors[tri] = Vec3fa(0,1,0); triangles[tri].v0 = 5; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;

  // bottom side
  colors[tri] = Vec3fa(0.5f);  triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 4; tri++;
  colors[tri] = Vec3fa(0.5f);  triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 4; tri++;

  // top side
  colors[tri] = Vec3fa(1.0f);  triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 3; tri++;
  colors[tri] = Vec3fa(1.0f);  triangles[tri].v0 = 3; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;

  // front side
  colors[tri] = Vec3fa(0,0,1); triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 2; tri++;
  colors[tri] = Vec3fa(0,0,1); triangles[tri].v0 = 2; triangles[tri].v1 = 4; triangles[tri].v2 = 6; tri++;

  // back side
  colors[tri] = Vec3fa(1,1,0); triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 5; tri++;
  colors[tri] = Vec3fa(1,1,0); triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 5; tri++;

  rtcUnmapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  return mesh;
}

/* adds a ground plane to the scene */
unsigned addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  unsigned mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10; 
  rtcUnmapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
extern "C" void device_init (const char* cfg)
{
  /* initialize ray tracing core */
  rtcInit(cfg);

  /* create scene */
  scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);

  /* add cube */
  addCube(scene);

  /* add ground plane */
  addGroundPlane(scene);

  /* commit changes to scene */
  rtcCommit (scene);
}

/* renders a single pixel with simple shading and shadows */
Vec3fa renderPixelNormal(size_t x, size_t y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;
      
  /* intersect ray with scene */
  rtcIntersect(scene,ray);
  
  /* shade pixels */
  Vec3fa color = 0.0f;

  if (ray.geomID != -1) 
  {
    Vec3fa diffuse = colors[ray.primID];
    color += 0.5f*diffuse;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    
    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = ray.org + ray.tfar * ray.dir;
    shadow.dir = -lightDir;
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = 1;
    shadow.primID = 0;
    shadow.mask = -1;
    shadow.time = 0;
    
    /* trace shadow ray */
    rtcOccluded(scene,shadow);
    
    /* add light contribution */
    if (shadow.geomID)
      color += diffuse * clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }
  return color;
}

/* renders a single pixel with UV shading */
Vec3fa renderPixelUV(size_t x, size_t y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(scene,ray);

  /* shade pixel */
  if (ray.geomID == -1) return zero;
  else return Vec3fa(ray.u,ray.v,1.0f-ray.u-ray.v);
}

/* vizualizes the traversal cost of a pixel */
float scale = 0.001f;

Vec3fa renderPixelCycles(size_t x, size_t y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  int64 c0 = get_tsc();
  rtcIntersect(scene,ray);
  int64 c1 = get_tsc();

  /* shade pixel */
  return Vec3fa((c1-c0)*scale,0.0f,0.0f);
}

/* called when a key is pressed */
Vec3fa (*renderPixel)(size_t x, size_t y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p) = renderPixelNormal;

extern "C" void device_key_pressed(int key)
{
  if      (key == GLUT_KEY_F1) renderPixel = renderPixelNormal;
  else if (key == GLUT_KEY_F2) renderPixel = renderPixelUV;
  else if (key == GLUT_KEY_F3) {
    if (renderPixel == renderPixelCycles) scale *= 1.1f;
    renderPixel = renderPixelCycles;
  }
  else if (key == GLUT_KEY_F4) {
    if (renderPixel == renderPixelCycles) scale *= 0.9f;
    renderPixel = renderPixelCycles;
  }
}

/* returns the point seen through specified pixel */
extern "C" bool device_pick(const float x,
                            const float y, 
                            const Vec3f& vx, 
                            const Vec3f& vy, 
                            const Vec3f& vz, 
                            const Vec3f& p,
                            Vec3f& hitPos)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(scene,ray);

  /* shade pixel */
  if (ray.geomID == -1) {
    hitPos = Vec3f(0.0f,0.0f,0.0f);
    return false;
  }
  else {
    hitPos = ray.org+ray.tfar*ray.dir;
    return true;
  }
}

/* task that renders a single screen tile */
void renderTile(int taskIndex,
                int* pixels,
                const int width,
                const int height, 
                const float time,
                const Vec3f& vx, 
                const Vec3f& vy, 
                const Vec3f& vz, 
                const Vec3f& p,
                const int numTilesX, 
                const int numTilesY)
{
  const size_t tileY = taskIndex / numTilesX;
  const size_t tileX = taskIndex - tileY * numTilesX;
  const size_t x0 = tileX * TILE_SIZE_X;
  const size_t x1 = min(x0+TILE_SIZE_X,size_t(width));
  const size_t y0 = tileY * TILE_SIZE_Y;
  const size_t y1 = min(y0+TILE_SIZE_Y,size_t(height));

  for (size_t y = y0; y<y1; y++)
  {
    for (size_t x = x0; x<x1; x++)
    {
      /* calculate pixel color */
      Vec3fa color = renderPixel(x,y,vx,vy,vz,p);

      /* write color to framebuffer */
      unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
      unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
      unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
      pixels[y*width+x] = (b << 16) + (g << 8) + r;
    }
  }
}

struct RenderTileTask
{
  RenderTileTask (int* pixels, const int width, const int height, const float time, 
                  const Vec3f& vx, const Vec3f& vy, const Vec3f& vz, const Vec3f& p, const int numTilesX, const int numTilesY)
    : pixels(pixels), width(width), height(height), time(time), vx(vx), vy(vy), vz(vz), p(p), numTilesX(numTilesX), numTilesY(numTilesY) {}

public:
  int* pixels;
  const int width;
  const int height;
  const float time;
  const Vec3f vx;
  const Vec3f vy;
  const Vec3f vz;
  const Vec3f p;
  const int numTilesX;
  const int numTilesY;
};

void renderTile_parallel(RenderTileTask* task, size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) {
  renderTile(taskIndex,task->pixels,task->width,task->height,task->time,task->vx,task->vy,task->vz,task->p,task->numTilesX,task->numTilesY);
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                               const int width,
                               const int height,
                               const float time,
                               const Vec3f& vx, 
                               const Vec3f& vy, 
                               const Vec3f& vz, 
                               const Vec3f& p)
{
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  const int numTiles = numTilesX * numTilesY;

  TaskScheduler::EventSync event;
  RenderTileTask parms(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY);
  TaskScheduler::Task task(&event,(TaskScheduler::runFunction)renderTile_parallel,&parms,numTiles,NULL,NULL,"render");
  TaskScheduler::addTask(-1,TaskScheduler::GLOBAL_FRONT,&task);
  event.sync();

  rtcDebug();
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (scene);
  alignedFree(colors);
  rtcExit();
}
