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
uniform Vec3f* uniform face_colors = NULL;

/* accumulation buffer */
uniform Vec3fa* uniform g_accu = NULL;
uniform unsigned int g_accu_width = 0;
uniform unsigned int g_accu_height = 0;
uniform unsigned int g_accu_count = 0;
uniform Vec3f g_accu_vx;
uniform Vec3f g_accu_vy;
uniform Vec3f g_accu_vz;
uniform Vec3f g_accu_p;
extern uniform bool g_changed;

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

__aligned(16) uniform float cube_vertices[8][4] = 
{
  { -1.0f, -1.0f, -1.0f, 0.0f },
  { -1.0f, -1.0f, +1.0f, 0.0f },
  { -1.0f, +1.0f, -1.0f, 0.0f },
  { -1.0f, +1.0f, +1.0f, 0.0f },
  { +1.0f, -1.0f, -1.0f, 0.0f },
  { +1.0f, -1.0f, +1.0f, 0.0f },
  { +1.0f, +1.0f, -1.0f, 0.0f },
  { +1.0f, +1.0f, +1.0f, 0.0f }
};

uniform unsigned int cube_indices[36] = { 
  0, 2, 1,  1, 2, 3,
  4, 5, 6,  5, 7, 6,
  0, 1, 4,  1, 5, 4,
  2, 6, 3,  3, 6, 7, 
  0, 4, 2,  2, 4, 6, 
  1, 3, 5,  3, 7, 5
};

unsigned int cube_quad_indices[24] = { 
  0, 1, 3, 6,
  4, 5, 7, 6, 
  0, 1, 5, 4, 
  2, 3, 7, 6, 
  0, 2, 6, 4, 
  1, 3, 7, 5,
};

#define USE_QUADS 0

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene)
{
#if USE_QUADS == 1
  /* create a quad cube with 6 quads and 8 vertices */
  uniform unsigned int geomID = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 6, 8, 2);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, 4*sizeof(unsigned int));
#else
  /* create a triangulated cube with 12 triangles and 8 vertices */
  uniform unsigned int geomID = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 12, 8, 2);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_indices , 0, 3*sizeof(uniform unsigned int));
#endif

  uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,3,0),make_Vec3f(1,1,0),0.44f);
  uniform Vec3fa* uniform vertex0 = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER0);
  uniform Vec3fa* uniform vertex1 = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER1);
  for (uniform int i=0; i<8; i++) 
  {
    uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2])+make_Vec3f(0,3,0);
    vertex0[i] = make_Vec3fa(v);
    vertex1[i] = make_Vec3fa(xfmPoint(rotation,v));
  }
  rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER0);
  rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER1);
  
  /* create face color array */
#if USE_QUADS == 1
  face_colors = uniform new uniform Vec3f[6];
  face_colors[0] = make_Vec3f(1,0,0);
  face_colors[1] = make_Vec3f(0,1,0);
  face_colors[2] = make_Vec3f(0.5f);
  face_colors[3] = make_Vec3f(1.0f);
  face_colors[4] = make_Vec3f(0,0,1);
  face_colors[5] = make_Vec3f(1,1,0);
#else
  face_colors = uniform new uniform Vec3f[12];
  face_colors[0] = make_Vec3f(1,0,0);
  face_colors[1] = make_Vec3f(1,0,0);
  face_colors[2] = make_Vec3f(0,1,0);
  face_colors[3] = make_Vec3f(0,1,0);
  face_colors[4] = make_Vec3f(0.5f);
  face_colors[5] = make_Vec3f(0.5f);
  face_colors[6] = make_Vec3f(1.0f);
  face_colors[7] = make_Vec3f(1.0f);
  face_colors[8] = make_Vec3f(0,0,1);
  face_colors[9] = make_Vec3f(0,0,1);
  face_colors[10] = make_Vec3f(1,1,0);
  face_colors[11] = make_Vec3f(1,1,0);
#endif
  return geomID;
}

/* add hair geometry */
uniform unsigned int addHair (RTCScene scene)
{
  uniform unsigned int geomID = rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, 16, 4*16, 2);

  uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,3,0),make_Vec3f(1,1,0),0.44f);
  uniform Vec3fa* uniform vertices0 = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER0);
  uniform Vec3fa* uniform vertices1 = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER1);
  for (uniform size_t i=0; i<16; i++) 
  {
    uniform Vec3f org = make_Vec3f((i%4+0.5f)*0.5f-1.0f,2.0f,(i/4+0.5f)*0.5f-1.0f);
    uniform Vec3f p0 = org + make_Vec3f(0.0f,+0.0f,0.0f);
    uniform Vec3f p1 = org + make_Vec3f(0.3f,+0.0f,0.0f);
    uniform Vec3f p2 = org + make_Vec3f(0.3f,-3.0f,0.3f);
    uniform Vec3f p3 = org + make_Vec3f(0.0f,-3.0f,0.0f);
    vertices0[4*i+0] = make_Vec3fa(p0,0.02f);
    vertices0[4*i+1] = make_Vec3fa(p1,0.02f);
    vertices0[4*i+2] = make_Vec3fa(p2,0.02f);
    vertices0[4*i+3] = make_Vec3fa(p3,0.02f);
    vertices1[4*i+0] = make_Vec3fa(xfmPoint(rotation,p0),0.02f);
    vertices1[4*i+1] = make_Vec3fa(xfmPoint(rotation,p1),0.02f);
    vertices1[4*i+2] = make_Vec3fa(p2,0.02f);
    vertices1[4*i+3] = make_Vec3fa(p3,0.02f);
  }
  rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER0);
  rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER1); 
  
  uniform int* uniform indices = (uniform int* uniform) rtcMapBuffer(scene,geomID,RTC_INDEX_BUFFER);
  for (uniform size_t i=0; i<16; i++) {
    indices[i] = 4*i;
  }
  rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);

  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize last seen camera */
  g_accu_vx = make_Vec3f(0.0f);
  g_accu_vy = make_Vec3f(0.0f);
  g_accu_vz = make_Vec3f(0.0f);
  g_accu_p  = make_Vec3f(0.0f);

  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);
 
  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* add cube */
  addCube(g_scene);

  /* add hair */
  addHair(g_scene);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  renderPixel = renderPixelStandard;
  key_pressed_handler = device_key_pressed_default;
}

uniform int frameID = 50;

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
{
  uniform float time = abs((uniform int)(0.01f*frameID) - 0.01f*frameID);

  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = time;
  
  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  
  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
  {
    Vec3f diffuse;
    if (ray.geomID == 0) diffuse = face_colors[ray.primID];
    else if (ray.geomID == 1) diffuse = make_Vec3f(0.0f,1.0f,0.0f);
    else diffuse = make_Vec3f(0.5f,0.5f,0.5f);
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-4,-1));
    
    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = ray.org + ray.tfar*ray.dir;
    shadow.dir = neg(lightDir);
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = 1;
    shadow.primID = 0;
    shadow.mask = -1;
    shadow.time = time;
    
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
    Vec3fa accu_color = g_accu[y*width+x] + make_Vec3fa(color.x,color.y,color.z,1.0f); g_accu[y*width+x] = accu_color;
    float f = rcp(max(0.001f,accu_color.w));
    unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
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
  uniform Vec3fa cam_org = make_Vec3fa(p.x,p.y,p.z);

  /* create accumulator */
  if (g_accu_width != width || g_accu_height != height) {
    delete[] g_accu;
    g_accu = uniform new uniform Vec3fa[width*height];
    g_accu_width = width;
    g_accu_height = height;
    memset(g_accu,0,width*height*sizeof(uniform Vec3fa));
  }

  /* reset accumulator */
  uniform bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(g_accu_vx,vx); g_accu_vx = vx;
  camera_changed |= ne(g_accu_vy,vy); g_accu_vy = vy;
  camera_changed |= ne(g_accu_vz,vz); g_accu_vz = vz;
  camera_changed |= ne(g_accu_p,  p); g_accu_p  = p;
  //camera_changed = true;
  if (camera_changed) {
    g_accu_count=0;
    memset(g_accu,0,width*height*sizeof(uniform Vec3fa));
  }
  
  /* render next frame */
  frameID++;
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
  delete[] face_colors;
  delete[] g_accu;
}

