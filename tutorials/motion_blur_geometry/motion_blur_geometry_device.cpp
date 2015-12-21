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
Vec3fa* face_colors = nullptr;

/* accumulation buffer */
Vec3fa* g_accu = nullptr;
unsigned int g_accu_width = 0;
unsigned int g_accu_height = 0;
unsigned int g_accu_count = 0;
Vec3fa g_accu_vx;
Vec3fa g_accu_vy;
Vec3fa g_accu_vz;
Vec3fa g_accu_p;
extern "C" bool g_changed;

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

__aligned(16) float cube_vertices[8][4] = 
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

unsigned int cube_indices[36] = { 
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
unsigned int addCube (RTCScene scene)
{
#if USE_QUADS == 1
  /* create a quad cube with 6 quads and 8 vertices */
  unsigned int geomID = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 6, 8, 2);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, 4*sizeof(unsigned int));
#else
  /* create a triangulated cube with 12 triangles and 8 vertices */
  unsigned int geomID = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 12, 8, 2);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_indices , 0, 3*sizeof(unsigned int));
#endif

  AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,3,0),Vec3fa(1,1,0),0.44f);
  Vec3fa* vertex0 = (Vec3fa*) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER0);
  Vec3fa* vertex1 = (Vec3fa*) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER1);
  for (int i=0; i<8; i++) 
  {
    Vec3fa v = Vec3fa(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2])+Vec3fa(0,3,0);
    vertex0[i] = Vec3fa(v);
    vertex1[i] = Vec3fa(xfmPoint(rotation,v));
  }
  rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER0);
  rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER1);
  
  /* create face color array */
#if USE_QUADS == 1
  face_colors = (Vec3fa*) alignedMalloc(6*sizeof(Vec3fa));
  face_colors[0] = Vec3fa(1,0,0);
  face_colors[1] = Vec3fa(0,1,0);
  face_colors[2] = Vec3fa(0.5f);
  face_colors[3] = Vec3fa(1.0f);
  face_colors[4] = Vec3fa(0,0,1);
  face_colors[5] = Vec3fa(1,1,0);
#else
  face_colors = (Vec3fa*) alignedMalloc(12*sizeof(Vec3fa));
  face_colors[0] = Vec3fa(1,0,0);
  face_colors[1] = Vec3fa(1,0,0);
  face_colors[2] = Vec3fa(0,1,0);
  face_colors[3] = Vec3fa(0,1,0);
  face_colors[4] = Vec3fa(0.5f);
  face_colors[5] = Vec3fa(0.5f);
  face_colors[6] = Vec3fa(1.0f);
  face_colors[7] = Vec3fa(1.0f);
  face_colors[8] = Vec3fa(0,0,1);
  face_colors[9] = Vec3fa(0,0,1);
  face_colors[10] = Vec3fa(1,1,0);
  face_colors[11] = Vec3fa(1,1,0);
#endif
  return geomID;
}

/* add hair geometry */
unsigned int addHair (RTCScene scene)
{
  unsigned int geomID = rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, 16, 4*16, 2);

  AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,3,0),Vec3fa(1,1,0),0.44f);
  Vec3fa* vertices0 = (Vec3fa*) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER0);
  Vec3fa* vertices1 = (Vec3fa*) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER1);
  for (size_t i=0; i<16; i++) 
  {
    Vec3fa org = Vec3fa((i%4+0.5f)*0.5f-1.0f,2.0f,(i/4+0.5f)*0.5f-1.0f);
    Vec3fa p0 = org + Vec3fa(0.0f,+0.0f,0.0f);
    Vec3fa p1 = org + Vec3fa(0.3f,+0.0f,0.0f);
    Vec3fa p2 = org + Vec3fa(0.3f,-3.0f,0.3f);
    Vec3fa p3 = org + Vec3fa(0.0f,-3.0f,0.0f);
    vertices0[4*i+0] = Vec3fa(p0,0.02f);
    vertices0[4*i+1] = Vec3fa(p1,0.02f);
    vertices0[4*i+2] = Vec3fa(p2,0.02f);
    vertices0[4*i+3] = Vec3fa(p3,0.02f);
    vertices1[4*i+0] = Vec3fa(xfmPoint(rotation,p0),0.02f);
    vertices1[4*i+1] = Vec3fa(xfmPoint(rotation,p1),0.02f);
    vertices1[4*i+2] = Vec3fa(p2,0.02f);
    vertices1[4*i+3] = Vec3fa(p3,0.02f);
  }
  rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER0);
  rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER1); 
  
  int* indices = (int*) rtcMapBuffer(scene,geomID,RTC_INDEX_BUFFER);
  for (size_t i=0; i<16; i++) {
    indices[i] = 4*i;
  }
  rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);

  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  unsigned int mesh = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* initialize last seen camera */
  g_accu_vx = Vec3fa(0.0f);
  g_accu_vy = Vec3fa(0.0f);
  g_accu_vz = Vec3fa(0.0f);
  g_accu_p  = Vec3fa(0.0f);

  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);
 
  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT1);

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

int frameID = 50;

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  float time = abs((int)(0.01f*frameID) - 0.01f*frameID);

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
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
  {
    Vec3fa diffuse;
    if (ray.geomID == 0) diffuse = face_colors[ray.primID];
    else if (ray.geomID == 1) diffuse = Vec3fa(0.0f,1.0f,0.0f);
    else diffuse = Vec3fa(0.5f,0.5f,0.5f);
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-4,-1));
    
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
    Vec3fa accu_color = g_accu[y*width+x] + Vec3fa(color.x,color.y,color.z,1.0f); g_accu[y*width+x] = accu_color;
    float f = rcp(max(0.001f,accu_color.w));
    unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
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
  Vec3fa cam_org = Vec3fa(p.x,p.y,p.z);

  /* create accumulator */
  if (g_accu_width != width || g_accu_height != height) {
    alignedFree(g_accu);
    g_accu = (Vec3fa*) alignedMalloc(width*height*sizeof(Vec3fa));
    g_accu_width = width;
    g_accu_height = height;
    memset(g_accu,0,width*height*sizeof(Vec3fa));
  }

  /* reset accumulator */
  bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(g_accu_vx,vx); g_accu_vx = vx;
  camera_changed |= ne(g_accu_vy,vy); g_accu_vy = vy;
  camera_changed |= ne(g_accu_vz,vz); g_accu_vz = vz;
  camera_changed |= ne(g_accu_p,  p); g_accu_p  = p;
  //camera_changed = true;
  if (camera_changed) {
    g_accu_count=0;
    memset(g_accu,0,width*height*sizeof(Vec3fa));
  }
  
  /* render next frame */
  frameID++;
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
  alignedFree(face_colors);
  alignedFree(g_accu);
}

