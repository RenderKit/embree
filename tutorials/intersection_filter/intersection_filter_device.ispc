// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
extern uniform bool g_ray_stream_mode;
uniform Vec3f* uniform colors = NULL;

void renderTileStandardStream(uniform int taskIndex, 
                              uniform int* uniform pixels,
                              const uniform int width,
                              const uniform int height, 
                              const uniform float time,
                              const uniform ISPCCamera& camera,
                              const uniform int numTilesX, 
                              const uniform int numTilesY);

/* error reporting function */
unmasked void error_handler(const uniform RTCError code, const uniform int8* uniform str = NULL)
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

#define NUM_VERTICES 8
#define NUM_QUAD_INDICES 24
#define NUM_TRI_INDICES 36
#define NUM_QUAD_FACES 6
#define NUM_TRI_FACES 12

__aligned(16) uniform float cube_vertices[NUM_VERTICES][4] = 
{
  { -1, -1, -1, 0 }, 
  { -1, -1, +1, 0 }, 
  { -1, +1, -1, 0 }, 
  { -1, +1, +1, 0 }, 
  { +1, -1, -1, 0 }, 
  { +1, -1, +1, 0 }, 
  { +1, +1, -1, 0 }, 
  { +1, +1, +1, 0 }, 
};

uniform unsigned int cube_quad_indices[NUM_QUAD_INDICES] = { 
  0, 2, 3, 1, 
  5, 7, 6, 4, 
  0, 1, 5, 4, 
  6, 7, 3, 2, 
  0, 4, 6, 2,
  3, 7, 5, 1
};

uniform unsigned int cube_tri_indices[NUM_TRI_INDICES] = { 
  0, 2, 1,  2, 3, 1, 
  5, 7, 4,  7, 6, 4, 
  0, 1, 4,  1, 5, 4, 
  6, 7, 2,  7, 3, 2, 
  0, 4, 2,  4, 6, 2,
  3, 7, 1,  7, 5, 1
};

uniform unsigned int cube_quad_faces[NUM_QUAD_FACES] = { 
  4, 4, 4, 4, 4, 4 
};

/* extended ray structure that includes total transparency along the ray */
struct RTCRay2
{
  Vec3f org;     //!< Ray origin
  Vec3f dir;     //!< Ray direction
  float tnear;   //!< Start of ray segment
  float tfar;    //!< End of ray segment
  float time;    //!< Time of this ray for motion blur.
  int mask;      //!< used to mask out objects during traversal
  Vec3f Ng;      //!< Geometric normal.
  float u;       //!< Barycentric u coordinate of hit
  float v;       //!< Barycentric v coordinate of hit
  int geomID;    //!< geometry ID
  int primID;    //!< primitive ID
  int instID;    //!< instance ID

  // ray extensions
  float transparency; //!< accumulated transparency value
};

/* 3D procedural transparency */
inline float transparencyFunction(Vec3f h)
{
  float v = abs(sin(4.0f*h.x)*cos(4.0f*h.y)*sin(4.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);
  return T;
}

/* intersection filter function */
void intersectionFilter(void* uniform ptr, RTCRay2& ray)
{
  Vec3f h = ray.org + ray.dir*ray.tfar;
  float T = transparencyFunction(h);
  if (T >= 1.0f) ray.geomID = RTC_INVALID_GEOMETRY_ID;
  else ray.transparency = T;
}

/* occlusion filter function */
void occlusionFilter(void* uniform ptr, RTCRay2& ray)
{
  Vec3f h = ray.org + ray.dir*ray.tfar;
  float T = transparencyFunction(h);
  T *= ray.transparency;
  ray.transparency = T;
  if (T != 0.0f) ray.geomID = RTC_INVALID_GEOMETRY_ID;
}

void scatter(varying float& ptr, const uniform size_t stride, const size_t pid, const size_t rid, float v)
{
  ((uniform float*)((uniform char* uniform)&ptr + pid*stride))[rid];
  //foreach_unique(p in pid) { 
  //  float* uniform ptr_pid = (float* uniform)((uniform char* uniform)&ptr + p*stride);
  //  *ptr_pid = insert(*ptr_pid,rid,v); 
  //}
}

/* intersection filter function */
unmasked void intersectionFilterN(uniform int* uniform valid,
                                  void* uniform ptr, 
                                  const uniform RTCIntersectContext* uniform context,
                                  struct RTCRayN* uniform ray,
                                  const struct RTCHitN* uniform potentialHit,
                                  uniform const size_t N)
{
  /* iterate over all rays in ray packet */
  foreach(i=0 ... N)
  {
    /* ignore inactive rays */
    if (valid[i] != -1) continue;
    
    /* read ray from ray structure */
    Vec3f ray_org = make_Vec3f(RTCRayN_org_x(ray,N,i),RTCRayN_org_y(ray,N,i),RTCRayN_org_z(ray,N,i));
    Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(ray,N,i),RTCRayN_dir_y(ray,N,i),RTCRayN_dir_z(ray,N,i));
    float ray_tfar = RTCRayN_tfar(ray,N,i);
    int   ray_mask = RTCRayN_mask(ray,N,i);

    /* get extended ray data */
    int pid = ray_mask >> (sizeof(float)/4);
    int rid = ray_mask & (sizeof(float)/4-1);
    varying RTCRay2* uniform eray = (varying RTCRay2* uniform) potentialHit->userRayExt;
    
    /* calculate transparency and either skip intersection or remember transparency value */
    Vec3f h = ray_org + ray_dir*ray_tfar;
    float T = transparencyFunction(ray);
    if (T >= 1.0f) valid[i] = 0;
    else scatter(&eray->transparency,sizeof(RTCRay2),pid,rid,transparency); 
    }
  }
}

/* occlusion filter function */
unmasked void occlusionFilterN(uniform int* uniform valid,
                               void* uniform ptr, 
                               const uniform RTCIntersectContext* uniform context,
                               struct RTCRayN* uniform ray,
                               const struct RTCHitN* uniform potentialHit,
                               uniform const size_t N)
{
  /* iterate over all rays in ray packet */
  foreach(i=0 ... N)
  {
    /* ignore inactive rays */
    if (valid[i] != -1) continue;
    
    /* read ray from ray structure */
    Vec3f ray_org = make_Vec3f(RTCRayN_org_x(ray,N,i),RTCRayN_org_y(ray,N,i),RTCRayN_org_z(ray,N,i));
    Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(ray,N,i),RTCRayN_dir_y(ray,N,i),RTCRayN_dir_z(ray,N,i));
    float ray_tfar = RTCRayN_tfar(ray,N,i);

    /* get extended ray data */
    int rid = RTCRayN_mask(ray,N,i);
    RTCRay2* uniform eray = (RTCRay2* uniform)potentialHit->userRayExt;
    float ray_transparency = at(&eray->transparency,rid);
    
    /* calculate transparency and either skip intersection or remember transparency value */
    Vec3f h = ray_org + ray_dir*ray_tfar;
    float T = transparencyFunction(ray);
    T *= ray_transparency;
    at(&eray->transparency,rid) = T;
    if (T != 0.0f) valid[i] = 0;
  }
}

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene_i)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  uniform unsigned int geomID = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, NUM_TRI_FACES, NUM_VERTICES);
  rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER, cube_vertices,     0, sizeof(uniform Vec3fa  ));
  rtcSetBuffer(scene_i, geomID, RTC_INDEX_BUFFER,  cube_tri_indices , 0, 3*sizeof(uniform unsigned int));

  /* create per-triangle color array */
  colors = uniform new uniform Vec3f[12];
  colors[0] = make_Vec3f(1,0,0); // left side
  colors[1] = make_Vec3f(1,0,0);
  colors[2] = make_Vec3f(0,1,0); // right side
  colors[3] = make_Vec3f(0,1,0);
  colors[4] = make_Vec3f(0.5f);  // bottom side
  colors[5] = make_Vec3f(0.5f); 
  colors[6] = make_Vec3f(1.0f);  // top side
  colors[7] = make_Vec3f(1.0f); 
  colors[8] = make_Vec3f(0,0,1); // front side
  colors[9] = make_Vec3f(0,0,1);
  colors[10] = make_Vec3f(1,1,0); // back side
  colors[11] = make_Vec3f(1,1,0);

  /* set intersection filter for the cube */
  if (g_ray_stream_mode) {
    rtcSetIntersectionFilterFunctionN(scene_i,geomID,(RTCFilterFuncN)&intersectionFilterN);
    rtcSetOcclusionFilterFunctionN   (scene_i,geomID,(RTCFilterFuncN)&occlusionFilterN);
  }
  else {
    rtcSetIntersectionFilterFunction(scene_i,geomID,(RTCFilterFuncVarying)&intersectionFilter);
    rtcSetOcclusionFilterFunction   (scene_i,geomID,(RTCFilterFuncVarying)&occlusionFilter);
  }

  return geomID;
}

/* adds a cube to the scene */
uniform unsigned int addSubdivCube (RTCScene scene_i)
{
  uniform unsigned int geomID = rtcNewSubdivisionMesh(scene_i, RTC_GEOMETRY_STATIC, NUM_QUAD_FACES, NUM_QUAD_INDICES, NUM_VERTICES, 0, 0, 0);
  rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER, cube_vertices,      0, sizeof(uniform Vec3fa  ));
  rtcSetBuffer(scene_i, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_FACE_BUFFER,   cube_quad_faces,    0, sizeof(uniform unsigned int));

  uniform float* level = (uniform float*) rtcMapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
  for (size_t i=0; i<NUM_QUAD_INDICES; i++) level[i] = 4;
  rtcUnmapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
  
  /* create face color array */
  colors = uniform new uniform Vec3f[6];
  colors[0] = make_Vec3f(1,0,0); // left side
  colors[1] = make_Vec3f(0,1,0); // right side
  colors[2] = make_Vec3f(0.5f);  // bottom side
  colors[3] = make_Vec3f(1.0f);  // top side
  colors[4] = make_Vec3f(0,0,1); // front side
  colors[5] = make_Vec3f(1,1,0); // back side

  /* set intersection filter for the cube */
  if (g_ray_stream_mode) {
    rtcSetIntersectionFilterFunctionN(scene_i,geomID,(RTCFilterFuncN)&intersectionFilterN);
    rtcSetOcclusionFilterFunctionN   (scene_i,geomID,(RTCFilterFuncN)&occlusionFilterN);
  }
  else {
    rtcSetIntersectionFilterFunction(scene_i,geomID,(RTCFilterFuncVarying)&intersectionFilter);
    rtcSetOcclusionFilterFunction   (scene_i,geomID,(RTCFilterFuncVarying)&occlusionFilter);
  }

  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
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
  uniform RTCAlgorithmFlags aflags;
  if (g_ray_stream_mode) aflags = RTC_INTERSECT_STREAM;
  else                   aflags = RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING;
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,aflags);

  /* add cube */
  addCube(g_scene);
  //addSubdivCube(g_scene);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  if (g_ray_stream_mode) renderTile = renderTileStandardStream;
  else                   renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera)
{
  float weight = 1.0f;
  Vec3f color = make_Vec3f(0.0f);

  /* initialize ray */
  RTCRay2 primary;
  primary.org = make_Vec3f(camera.xfm.p);
  primary.dir = make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  primary.tnear = 0.0f;
  primary.tfar = inf;
  primary.geomID = RTC_INVALID_GEOMETRY_ID;
  primary.primID = RTC_INVALID_GEOMETRY_ID;
  primary.mask = -1;
  primary.time = 0;
  primary.transparency = 0.0f;

  while (true)
  {
    /* intersect ray with scene */
    rtcIntersect(g_scene,*((varying RTCRay*)&primary));
    
    /* shade pixels */
    if (primary.geomID == RTC_INVALID_GEOMETRY_ID) 
      break;

    float opacity = 1.0f-primary.transparency;
    Vec3f diffuse = colors[primary.primID];
    Vec3f La = diffuse*0.5f;
    color = color + weight*opacity*La;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));
      
    /* initialize shadow ray */
    RTCRay2 shadow;
    shadow.org = primary.org + primary.tfar*primary.dir;
    shadow.dir = neg(lightDir);
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;
    shadow.transparency = 1.0f;
    
    /* trace shadow ray */
    rtcOccluded(g_scene,*((varying RTCRay*)&shadow));
    
    /* add light contribution */
    if (shadow.geomID) {
      Vec3f Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.Ng)),0.0f,1.0f);
      color = color + weight*opacity*Ll;
    }

    /* shoot transmission ray */
    weight *= primary.transparency;
    primary.tnear = 1.001f*primary.tfar;
    primary.tfar = inf;
    primary.geomID = RTC_INVALID_GEOMETRY_ID;
    primary.primID = RTC_INVALID_GEOMETRY_ID;
    primary.transparency = 0.0f;
  }
  return color;
}
  
/* renders a single screen tile */
void renderTileStandard(uniform int taskIndex, 
                        uniform int* uniform pixels,
                        const uniform int width,
                        const uniform int height, 
                        const uniform float time,
                        const uniform ISPCCamera& camera,
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
    Vec3f color = renderPixelStandard(x,y,camera);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* renders a single screen tile */
void renderTileStandardStream(uniform int taskIndex, 
                              uniform int* uniform pixels,
                              const uniform int width,
                              const uniform int height, 
                              const uniform float time,
                              const uniform ISPCCamera& camera,
                              const uniform int numTilesX, 
                              const uniform int numTilesY)
{
  const uniform int tileY = taskIndex / numTilesX;
  const uniform int tileX = taskIndex - tileY * numTilesX;
  const uniform int x0 = tileX * TILE_SIZE_X;
  const uniform int x1 = min(x0+TILE_SIZE_X,width);
  const uniform int y0 = tileY * TILE_SIZE_Y;
  const uniform int y1 = min(y0+TILE_SIZE_Y,height);

  RTCRay rays[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  uniform int N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;
    
    /* initialize ray */
    RTCRay& ray = rays[N++];
    ray.org = make_Vec3f(camera.xfm.p); // FIXME: make invalid rays empty
    ray.dir = make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
    unmasked { ray.tnear = pos_inf; ray.tfar = neg_inf; } // invalidates inactive rays
    ray.tnear = 0.0f;
    ray.tfar = inf;
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.primID = RTC_INVALID_GEOMETRY_ID;
    ray.mask = -1;
    ray.time = 0.0f;
  }

  /* trace primary rays */
  uniform RTCIntersectContext context;
  context.flags = RTC_INTERSECT_INCOHERENT;
  rtcIntersectVM(g_scene,&context,rays,N,sizeof(RTCRay));

  /* shade stream of rays */
  N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;
    RTCRay& ray = rays[N++];

    /* eyelight shading */
    Vec3f color = make_Vec3f(0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
      color = make_Vec3f(abs(dot(ray.dir,normalize(ray.Ng))));
      //color = ambientOcclusionShading(x,y,ray);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
task void renderTileTask(uniform int* uniform pixels,
                         const uniform int width,
                         const uniform int height, 
                         const uniform float time,
                         const uniform ISPCCamera& camera,
                         const uniform int numTilesX, 
                         const uniform int numTilesY)
{
  renderTile(taskIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform int width,
                           const uniform int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene); g_scene = NULL;
  rtcDeleteDevice(g_device); g_device = NULL;
  delete[] colors; colors = NULL;
}

