// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

/* scene data */
RTCDevice g_device = nullptr;
RTCScene g_scene = nullptr;
Vec3f* vertex_colors = nullptr;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const RTCError code, const char* str = nullptr)
{
  if (code == RTC_NO_ERROR) 
    return;

  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : printf("RTC_CANCELLED"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  exit(1);
}

/* adds points to the scene */
unsigned int addPoints (RTCScene scene_i, unsigned int numPoints)
{
  /* create a Lorenz attractor */
  unsigned int mesh = rtcNewPoints (scene_i, RTC_GEOMETRY_STATIC, numPoints);

  /* create vertex color arrays */
  vertex_colors = (Vec3f*)alignedMalloc(numPoints*sizeof(Vec3f));

  /* set vertices and vertex colors */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER);

  float x0, y0, z0, x1, y1, z1;
  const float dt = 0.001f;
  const float a = 10.0f;
  const float b = 28.0f;
  const float c = 8.0f / 3.0f;

  x0 = 0.1f;
  y0 = 0.0f;
  z0 = 0.0f;

  for (unsigned int i = 0; i < numPoints; i++) {
    vertex_colors[i] = Vec3fa(1,(float)i/numPoints,0); 
	x1 = x0 + dt * a * (y0 - x0);
	y1 = y0 + dt * (x0 * (b - z0) - y0);
	z1 = z0 + dt * (x0 * y0 - c * z0);
	x0 = x1;
	y0 = y1;
	z0 = z1;
	vertices[i].x = x0; vertices[i].y = y0; vertices[i].z = z0; vertices[i].r = 0.05f;
  }

  rtcUnmapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 

  return mesh;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  unsigned int mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -80; vertices[0].y = -30; vertices[0].z = -50; 
  vertices[1].x = -80; vertices[1].y = -30; vertices[1].z = +50; 
  vertices[2].x = +20; vertices[2].y = -30; vertices[2].z = -50; 
  vertices[3].x = +20; vertices[3].y = -30; vertices[3].z = +50;
  rtcUnmapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);
 
  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT1);

  /* add points */
  addPoints(g_scene, 1000000);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  renderPixel = renderPixelStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
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
  
  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
  {
	  Vec3fa diffuse;
	if (ray.geomID == 0) 
		diffuse = vertex_colors[ray.primID];
	else if (ray.geomID == 1) 
		diffuse = Vec3fa(1.0f, 1.0f, 1.0f);
	else 
		diffuse = Vec3fa(0.5f, 0.5f, 0.5f);
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    
    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = ray.org + ray.tfar*ray.dir;
    shadow.dir = neg(lightDir);
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = 1;
    shadow.primID = 0;
    shadow.mask = -1;
    shadow.time = 0;
    
    /* trace shadow ray */
    rtcOccluded(g_scene,shadow);
    
    /* add light contribution */
    if (shadow.geomID)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }
  return color;
}

/* task that renders a single screen tile */
void renderTile(int taskIndex, int* pixels,
                     const int width,
                     const int height, 
                     const float time,
                     const Vec3fa& vx, 
                     const Vec3fa& vy, 
                     const Vec3fa& vz, 
                     const Vec3fa& p,
                     const int numTilesX, 
                     const int numTilesY)
{
  const int tileY = taskIndex / numTilesX;
  const int tileX = taskIndex - tileY * numTilesX;
  const int x0 = tileX * TILE_SIZE_X;
  const int x1 = min(x0+TILE_SIZE_X,width);
  const int y0 = tileY * TILE_SIZE_Y;
  const int y1 = min(y0+TILE_SIZE_Y,height);

  for (int y = y0; y<y1; y++) for (int x = x0; x<x1; x++)
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

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                    const int width,
                    const int height,
                    const float time,
                    const Vec3fa& vx, 
                    const Vec3fa& vy, 
                    const Vec3fa& vz, 
                    const Vec3fa& p)
{
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
  alignedFree(vertex_colors);
}

