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

#include "../common/tutorial/tutorial_device.isph"

/* scene data */
RTCDevice g_device = NULL;
RTCScene g_scene = NULL;
uniform Vec3f* uniform vertex_colors = NULL;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const uniform RTCError code, const uniform int8* uniform str = NULL)
{
  if (code == RTC_NO_ERROR) 
    return;

  print("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : print("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : print("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: print("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : print("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : print("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : print("RTC_CANCELLED"); break;
  default                   : print("invalid error code"); break;
  }
  if (str) { 
    print(" ("); 
    while (*str) putchar(*str++); 
    print(")\n"); 
  }
  abort();
}

/* adds points to the scene */
uniform unsigned int addPoints (RTCScene scene_i, uniform unsigned int numPoints)
{
  /* create a Lorenz attractor */
  uniform unsigned int mesh = rtcNewPoints (scene_i, RTC_GEOMETRY_STATIC, numPoints);

  /* create vertex color arrays */
  vertex_colors = uniform new uniform Vec3f[numPoints];

  /* set vertices and vertex colors */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER);

  uniform float x0, y0, z0, x1, y1, z1;
  uniform const float dt = 0.001f;
  uniform const float a = 10.0f;
  uniform const float b = 28.0f;
  uniform const float c = 8.0f / 3.0f;

  x0 = 0.1f;
  y0 = 0.0f;
  z0 = 0.0f;

  for (unsigned int i = 0; i < numPoints; i++) {
    vertex_colors[i] = make_Vec3f(1,(float)i/numPoints,0); 
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
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -80; vertices[0].y = -30; vertices[0].z = -50; 
  vertices[1].x = -80; vertices[1].y = -30; vertices[1].z = +50; 
  vertices[2].x = +20; vertices[2].y = -30; vertices[2].z = -50; 
  vertices[3].x = +20; vertices[3].y = -30; vertices[3].z = +50;
  rtcUnmapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);
 
  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

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
Vec3f renderPixelStandard(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
  {
	 Vec3f diffuse;
	if (ray.geomID == 0) 
		diffuse = vertex_colors[ray.primID];
	else if (ray.geomID == 1) 
		diffuse = make_Vec3f(1.0f, 1.0f, 1.0f);
	else 
		diffuse = make_Vec3f(0.5f, 0.5f, 0.5f);
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));
    
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
task void renderTile(uniform int* uniform pixels,
                     const uniform int width,
                     const uniform int height, 
                     const uniform float time,
                     const uniform Vec3f& vx, 
                     const uniform Vec3f& vy, 
                     const uniform Vec3f& vz, 
                     const uniform Vec3f& p,
                     const uniform int numTilesX, 
                     const uniform int numTilesY)
{
  const uniform int tileY = taskIndex / numTilesX;
  const uniform int tileX = taskIndex - tileY * numTilesX;
  const uniform int x0 = tileX * TILE_SIZE_X;
  const uniform int x1 = min(x0+TILE_SIZE_X,width);
  const uniform int y0 = tileY * TILE_SIZE_Y;
  const uniform int y1 = min(y0+TILE_SIZE_Y,height);

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* calculate pixel color */
    Vec3f color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;  
  }
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                    const uniform int width,
                    const uniform int height,
                    const uniform float time,
                    const uniform Vec3f& vx, 
                    const uniform Vec3f& vy, 
                    const uniform Vec3f& vz, 
                    const uniform Vec3f& p)
{
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
  delete[] vertex_colors;
}

