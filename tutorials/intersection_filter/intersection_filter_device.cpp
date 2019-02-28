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

/* scene data */
RTCScene g_scene = nullptr;
Vec3fa* colors = nullptr;

// FIXME: fast path for occlusionFilter

/******************************************************************************************/
/*                             Standard Mode                                              */
/******************************************************************************************/

#define HIT_LIST_LENGTH 16

/* extended ray structure that includes total transparency along the ray */
struct Ray2
{
  Ray ray;

  // ray extensions
  float transparency; //!< accumulated transparency value

  // we remember up to 16 hits to ignore duplicate hits
  unsigned int firstHit, lastHit;
  unsigned int hit_geomIDs[HIT_LIST_LENGTH];
  unsigned int hit_primIDs[HIT_LIST_LENGTH];
};

inline RTCRayHit* RTCRayHit_(Ray2& ray)
{
  RTCRayHit* ray_ptr = (RTCRayHit*)&ray;
  return ray_ptr;
}

inline RTCRay* RTCRay_(Ray2& ray)
{
  RTCRay* ray_ptr = (RTCRay*)&ray;
  return ray_ptr;
}

/* 3D procedural transparency */
inline float transparencyFunction(Vec3fa& h)
{
  float v = abs(sin(4.0f*h.x)*cos(4.0f*h.y)*sin(4.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);
  return T;
  //return 0.5f;
}


/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  float weight = 1.0f;
  Vec3fa color = Vec3fa(0.0f);

  IntersectContext context;
  InitIntersectionContext(&context);
  
  /* initialize ray */
  Ray2 primary;
  init_Ray(primary.ray,Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);
  primary.ray.id = 0; // needs to encode rayID for filter
  primary.transparency = 0.0f;


  while (true)
  {
    context.userRayExt = &primary;

    /* intersect ray with scene */
    rtcIntersect1(g_scene,&context.context,RTCRayHit_(primary));
    RayStats_addRay(stats);

    /* shade pixels */
    if (primary.ray.geomID == RTC_INVALID_GEOMETRY_ID)
      break;

    float opacity = 1.0f-primary.transparency;
    Vec3fa diffuse = colors[primary.ray.primID];
    Vec3fa La = diffuse*0.5f;
    color = color + weight*opacity*La;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    Ray2 shadow;
    init_Ray(shadow.ray, primary.ray.org + primary.ray.tfar*primary.ray.dir, neg(lightDir), 0.001f, inf);
    shadow.ray.id = 0; // needs to encode rayID for filter
    shadow.transparency = 1.0f;
    shadow.firstHit = 0;
    shadow.lastHit = 0;
    context.userRayExt = &shadow;

    /* trace shadow ray */
    rtcOccluded1(g_scene,&context.context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.ray.tfar >= 0.0f) {
      Vec3fa Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.ray.Ng)),0.0f,1.0f);
      color = color + weight*opacity*Ll;
    }

    /* shoot transmission ray */
    weight *= primary.transparency;
    primary.ray.tnear() = 1.001f*primary.ray.tfar;
    primary.ray.tfar = (float)(inf);
    primary.ray.geomID = RTC_INVALID_GEOMETRY_ID;
    primary.ray.primID = RTC_INVALID_GEOMETRY_ID;
    primary.transparency = 0.0f;
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

/******************************************************************************************/
/*                               Stream Mode                                              */
/******************************************************************************************/

inline float gather(float& ptr, const unsigned int stride, const unsigned int pid, const unsigned int rid)
{
  float* uptr = (float*) (((char*)&ptr) + pid*stride);
  return uptr[rid];
}

inline unsigned int gather(unsigned int& ptr, const unsigned int stride, const unsigned int pid, const unsigned int rid)
{
  unsigned int* uptr = (unsigned int*) (((char*)&ptr) + pid*stride);
  return uptr[rid];
}

inline unsigned int gather(unsigned int& ptr, const unsigned int idx, const unsigned int stride, const unsigned int pid, const unsigned int rid)
{
  unsigned int* uptr = (unsigned int*) (((char*)&ptr) + pid*stride);
  return uptr[rid + 1*idx];
}

inline void scatter(float& ptr, const unsigned int stride, const unsigned int pid, const unsigned int rid, float v) {
  ((float*)(((char*)&ptr) + pid*stride))[rid] = v;
}

inline void scatter(unsigned int& ptr, const unsigned int stride, const unsigned int pid, const unsigned int rid, unsigned int v) {
  ((unsigned int*)(((char*)&ptr) + pid*stride))[rid] = v;
}

inline void scatter(unsigned int& ptr, const unsigned int idx, const unsigned int stride, const unsigned int pid, const unsigned int rid, unsigned int v) {
  ((unsigned int*)(((char*)&ptr) + pid*stride))[rid+1*idx] = v;
}


/* intersection filter function for single rays and packets */
void intersectionFilter(const RTCFilterFunctionNArguments* args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == nullptr) return;

  assert(args->N == 1);
  int* valid = args->valid;
  const IntersectContext* context = (const IntersectContext*) args->context;
  Ray* ray = (Ray*)args->ray;
  //RTCHit* hit = (RTCHit*)args->hit;

  /* ignore inactive rays */
  if (valid[0] != -1) return;

  /* calculate transparency */
  Vec3fa h = ray->org + ray->dir  * ray->tfar;
  float T = transparencyFunction(h);

  /* ignore hit if completely transparent */
  if (T >= 1.0f) 
    valid[0] = 0;
  /* otherwise accept hit and remember transparency */
  else
  {
    Ray2* eray = (Ray2*) context->userRayExt;
    eray->transparency = T;
  }
}

/* intersection filter function for streams of general packets */
void intersectionFilterN(const RTCFilterFunctionNArguments* args)
{
  int* valid = args->valid;
  const IntersectContext* context = (const IntersectContext*) args->context;
  struct RTCRayHitN* rayN = (struct RTCRayHitN*)args->ray;
  //struct RTCHitN* hitN = args->hit;
  const unsigned int N = args->N;
                                  
  /* avoid crashing when debug visualizations are used */
  if (context == nullptr) return;

  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* read ray/hit from ray structure */
    RTCRayHit rtc_ray = rtcGetRayHitFromRayHitN(rayN,N,ui);
    Ray* ray = (Ray*)&rtc_ray;

    /* calculate transparency */
    Vec3fa h = ray->org + ray->dir  * ray->tfar;
    float T = transparencyFunction(h);

    /* ignore hit if completely transparent */
    if (T >= 1.0f) 
      valid[vi] = 0;
    /* otherwise accept hit and remember transparency */
    else
    {
      /* decode ray IDs */
      const unsigned int pid = ray->id / 1;
      const unsigned int rid = ray->id % 1;
      Ray2* ray2 = (Ray2*) context->userRayExt;
      assert(ray2);
      scatter(ray2->transparency,sizeof(Ray2),pid,rid,T);
    }
  }
}

/* occlusion filter function for single rays and packets */
void occlusionFilter(const RTCFilterFunctionNArguments* args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == nullptr) return;

  assert(args->N == 1);
  int* valid = args->valid;
  const IntersectContext* context = (const IntersectContext*) args->context;
  Ray* ray = (Ray*)args->ray;
  RTCHit* hit = (RTCHit*)args->hit;

  /* ignore inactive rays */
  if (valid[0] != -1) return;

  Ray2* ray2 = (Ray2*) context->userRayExt;
  assert(ray2);

  for (unsigned int i=ray2->firstHit; i<ray2->lastHit; i++) {
    unsigned slot= i%HIT_LIST_LENGTH;
    if (ray2->hit_geomIDs[slot] == hit->geomID && ray2->hit_primIDs[slot] == hit->primID) {
      valid[0] = 0; return; // ignore duplicate intersections
    }
  }
  /* store hit in hit list */
  unsigned int slot = ray2->lastHit%HIT_LIST_LENGTH;
  ray2->hit_geomIDs[slot] = hit->geomID;
  ray2->hit_primIDs[slot] = hit->primID;
  ray2->lastHit++;
  if (ray2->lastHit - ray2->firstHit >= HIT_LIST_LENGTH)
    ray2->firstHit++;

  Vec3fa h = ray->org + ray->dir * ray->tfar;

  /* calculate and accumulate transparency */
  float T = transparencyFunction(h);
  T *= ray2->transparency;
  ray2->transparency = T;
  if (T != 0.0f) 
    valid[0] = 0;
}

/* intersection filter function for streams of general packets */
void occlusionFilterN(const RTCFilterFunctionNArguments* args)
{
  int* valid = args->valid;
  const IntersectContext* context = (const IntersectContext*) args->context;
  struct RTCRayHitN* rayN = (struct RTCRayHitN*)args->ray;
  struct RTCHitN* hitN = args->hit;
  const unsigned int N = args->N;
                                  
  /* avoid crashing when debug visualizations are used */
  if (context == nullptr) return;

  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* read ray/hit from ray structure */
    RTCRayHit rtc_ray = rtcGetRayHitFromRayHitN(rayN,N,ui);
    Ray* ray = (Ray*)&rtc_ray;

    RTCHit hit = rtcGetHitFromHitN(hitN,N,ui);
    const unsigned int hit_geomID = hit.geomID;
    const unsigned int hit_primID = hit.primID;

    /* decode ray IDs */
    const unsigned int pid = ray->id / 1;
    const unsigned int rid = ray->id % 1;
    Ray2* ray2 = (Ray2*) context->userRayExt;
    assert(ray2);

    /* The occlusion filter function may be called multiple times with
     * the same hit. We remember the last N hits, and skip duplicates. */
    unsigned int ray2_firstHit = gather(ray2->firstHit,sizeof(Ray2),pid,rid);
    unsigned int ray2_lastHit =  gather(ray2->lastHit ,sizeof(Ray2),pid,rid);
    for (unsigned int i=ray2_firstHit; i<ray2_lastHit; i++)
    {
      unsigned int slot= i%HIT_LIST_LENGTH;
      unsigned int last_geomID = gather(ray2->hit_geomIDs[0],slot,sizeof(Ray2),pid,rid);
      unsigned int last_primID = gather(ray2->hit_primIDs[0],slot,sizeof(Ray2),pid,rid);
      if (last_geomID == hit_geomID && last_primID == hit_primID) {
        valid[vi] = 0; break; // ignore duplicate intersections
      }
    }
    if (!valid[vi]) continue;

    /* store hit in hit list */
    unsigned int slot = ray2_lastHit%HIT_LIST_LENGTH;
    scatter(ray2->hit_geomIDs[0],slot,sizeof(Ray2),pid,rid,hit_geomID);
    scatter(ray2->hit_primIDs[0],slot,sizeof(Ray2),pid,rid,hit_primID);
    ray2_lastHit++;
    scatter(ray2->lastHit,sizeof(Ray2),pid,rid,ray2_lastHit);
    if (ray2_lastHit - ray2_firstHit >= HIT_LIST_LENGTH)
      scatter(ray2->firstHit,sizeof(Ray2),pid,rid,ray2_firstHit+1);

    /* calculate transparency */
    Vec3fa h = ray->org + ray->dir * ray->tfar;
    float T = transparencyFunction(h);

    /* accumulate transparency and store inside ray extensions */
    T *= gather(ray2->transparency,sizeof(Ray2),pid,rid);
    scatter(ray2->transparency,sizeof(Ray2),pid,rid,T);

    /* reject a hit if not fully opqaue */
    if (T != 0.0f) 
      valid[vi] = 0;
  }
}

/* renders a single screen tile */
void renderTileStandardStream(int taskIndex,
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

  RayStats& stats = g_stats[threadIndex];

  Ray2 primary_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Ray2 shadow_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Vec3fa color_stream[TILE_SIZE_X*TILE_SIZE_Y];
  float weight_stream[TILE_SIZE_X*TILE_SIZE_Y];
  bool valid_stream[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  int N = 0;
  int numActive = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    

    /* initialize variables */
    numActive++;
    color_stream[N] = Vec3fa(0.0f);
    weight_stream[N] = 1.0f;
    bool mask = 1; { valid_stream[N] = mask; }

    /* initialize ray */
    Ray2& primary = primary_stream[N];
    mask = 1; { // invalidates inactive rays
      primary.ray.tnear() = mask ? 0.0f         : (float)(pos_inf);
      primary.ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(primary.ray, Vec3fa(camera.xfm.p), Vec3fa(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), primary.ray.tnear(), primary.ray.tfar);
 
    primary.ray.id = N*1 + 0;
    primary.transparency = 0.0f;

    N++;
    RayStats_addRay(stats);
  }

  Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

  while (numActive)
  {
    /* trace rays */
    IntersectContext primary_context;
    InitIntersectionContext(&primary_context);
    primary_context.context.flags = g_iflags_coherent;
    primary_context.userRayExt = &primary_stream;
    rtcIntersect1M(g_scene,&primary_context.context,(RTCRayHit*)&primary_stream,N,sizeof(Ray2));

    /* terminate rays and update color */
    N = -1;
    for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
    {
      N++;
      /* ISPC workaround for mask == 0 */
      

      /* invalidate shadow rays by default */
      Ray2& shadow = shadow_stream[N];
      {
        shadow.ray.tnear() = (float)(pos_inf);
        shadow.ray.tfar  = (float)(neg_inf);
      }

      /* ignore invalid rays */
      if (valid_stream[N] == false) continue;

      /* terminate rays that hit nothing */
      if (primary_stream[N].ray.geomID == RTC_INVALID_GEOMETRY_ID) {
        valid_stream[N] = false;
        continue;
      }

      /* update color */
      Ray2& primary = primary_stream[N];
      float opacity = 1.0f-primary.transparency;
      Vec3fa diffuse = colors[primary.ray.primID];
      Vec3fa La = diffuse*0.5f;
      color_stream[N] = color_stream[N] + weight_stream[N]*opacity*La;

      /* initialize shadow ray */
      bool mask = 1; {
        shadow.ray.tnear() = mask ? 0.001f       : (float)(pos_inf);
        shadow.ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
      }
      init_Ray(shadow.ray, primary.ray.org + primary.ray.tfar*primary.ray.dir, neg(lightDir), shadow.ray.tnear(), shadow.ray.tfar);
      shadow.ray.id = N*1 + 0;
      shadow.transparency = 1.0f;
      shadow.firstHit = 0;
      shadow.lastHit = 0;
      RayStats_addShadowRay(stats);
    }
    N++;

    /* trace shadow rays */
    IntersectContext shadow_context;
    InitIntersectionContext(&shadow_context);
    shadow_context.context.flags = g_iflags_coherent;
    shadow_context.userRayExt = &shadow_stream;
    rtcOccluded1M(g_scene,&shadow_context.context,(RTCRay*)&shadow_stream,N,sizeof(Ray2));

    /* add light contribution and generate transmission ray */
    N = -1;
    numActive = 0;
    for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
    {
      N++;
      /* ISPC workaround for mask == 0 */
      

      /* invalidate rays by default */
      Ray2& primary = primary_stream[N];
      float primary_tfar = primary.ray.tfar;
      {
        primary.ray.tnear() = (float)(pos_inf);
        primary.ray.tfar  = (float)(neg_inf);
      }

      /* ignore invalid rays */
      if (valid_stream[N] == false) continue;
      numActive++;

      /* add light contrinution */
      float opacity = 1.0f-primary.transparency;
      Vec3fa diffuse = colors[primary.ray.primID];
      Ray2& shadow = shadow_stream[N];
      if (shadow.ray.tfar >= 0.0f) {
        Vec3fa Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.ray.Ng)),0.0f,1.0f);
        color_stream[N] = color_stream[N] + weight_stream[N]*opacity*Ll;
      }

      /* initialize transmission ray */
      weight_stream[N] *= primary.transparency;
      bool mask = 1; {
        primary.ray.tnear() = mask ? 1.001f*primary_tfar : (float)(pos_inf);
        primary.ray.tfar  = mask ? (float)(inf)        : (float)(neg_inf);
      }
      primary.ray.geomID = RTC_INVALID_GEOMETRY_ID;
      primary.ray.primID = RTC_INVALID_GEOMETRY_ID;
      primary.transparency = 0.0f;
      RayStats_addRay(stats);
    }
    N++;
  }

  /* framebuffer writeback */
  N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color_stream[N].x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color_stream[N].y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color_stream[N].z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
    N++;
  }
}

/******************************************************************************************/
/*                              Scene Creation                                            */
/******************************************************************************************/

#define NUM_VERTICES 8
#define NUM_QUAD_INDICES 24
#define NUM_TRI_INDICES 36
#define NUM_QUAD_FACES 6
#define NUM_TRI_FACES 12

__aligned(16) float cube_vertices[NUM_VERTICES][4] =
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

unsigned int cube_quad_indices[NUM_QUAD_INDICES] = {
  0, 1, 3, 2,
  5, 4, 6, 7,
  0, 4, 5, 1,
  6, 2, 3, 7,
  0, 2, 6, 4,
  3, 1, 5, 7
};

unsigned int cube_tri_indices[NUM_TRI_INDICES] = {
  0, 1, 2,  2, 1, 3,
  5, 4, 7,  7, 4, 6,
  0, 4, 1,  1, 4, 5,
  6, 2, 7,  7, 2, 3,
  0, 2, 4,  4, 2, 6,
  3, 1, 7,  7, 1, 5
};

unsigned int cube_quad_faces[NUM_QUAD_FACES] = {
  4, 4, 4, 4, 4, 4
};

/* adds a cube to the scene */
unsigned int addCube (RTCScene scene_i, const Vec3fa& offset, const Vec3fa& scale, float rotation)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  //rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, cube_vertices,     0, sizeof(Vec3fa  ), NUM_VERTICES);
  Vec3fa* ptr = (Vec3fa*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), NUM_VERTICES);
  for (unsigned int i=0; i<NUM_VERTICES; i++) {
    float x = cube_vertices[i][0];
    float y = cube_vertices[i][1];
    float z = cube_vertices[i][2];
    Vec3fa vtx = Vec3fa(x,y,z);
    ptr[i] = Vec3fa(offset+LinearSpace3fa::rotate(Vec3fa(0,1,0),rotation)*LinearSpace3fa::scale(scale)*vtx);
  }
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, cube_tri_indices, 0, 3*sizeof(unsigned int), NUM_TRI_FACES);

  /* create per-triangle color array */
  colors = (Vec3fa*) alignedMalloc(12*sizeof(Vec3fa),16);
  colors[0] = Vec3fa(1,0,0); // left side
  colors[1] = Vec3fa(1,0,0);
  colors[2] = Vec3fa(0,1,0); // right side
  colors[3] = Vec3fa(0,1,0);
  colors[4] = Vec3fa(0.5f);  // bottom side
  colors[5] = Vec3fa(0.5f);
  colors[6] = Vec3fa(1.0f);  // top side
  colors[7] = Vec3fa(1.0f);
  colors[8] = Vec3fa(0,0,1); // front side
  colors[9] = Vec3fa(0,0,1);
  colors[10] = Vec3fa(1,1,0); // back side
  colors[11] = Vec3fa(1,1,0);

  /* set intersection filter for the cube */
  if (g_mode == MODE_NORMAL && nativePacketSupported(g_device))
  {
    rtcSetGeometryIntersectFilterFunction(geom,intersectionFilter);
    rtcSetGeometryOccludedFilterFunction(geom,occlusionFilter);
  }
  else
  {
    rtcSetGeometryIntersectFilterFunction(geom,intersectionFilterN);
    rtcSetGeometryOccludedFilterFunction(geom,occlusionFilterN);
  }

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a cube to the scene */
unsigned int addSubdivCube (RTCScene scene_i)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, cube_vertices,      0, sizeof(Vec3fa),       NUM_VERTICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT,   cube_quad_indices,  0, sizeof(unsigned int), NUM_QUAD_INDICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,   0, RTC_FORMAT_UINT,   cube_quad_faces,    0, sizeof(unsigned int), NUM_QUAD_FACES);

  float* level = (float*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, sizeof(float), NUM_QUAD_INDICES);
  for (unsigned int i=0; i<NUM_QUAD_INDICES; i++) level[i] = 4;

  /* create face color array */
  colors = (Vec3fa*) alignedMalloc(6*sizeof(Vec3fa),16);
  colors[0] = Vec3fa(1,0,0); // left side
  colors[1] = Vec3fa(0,1,0); // right side
  colors[2] = Vec3fa(0.5f);  // bottom side
  colors[3] = Vec3fa(1.0f);  // top side
  colors[4] = Vec3fa(0,0,1); // front side
  colors[5] = Vec3fa(1,1,0); // back side

  /* set intersection filter for the cube */
  if (g_mode == MODE_NORMAL && nativePacketSupported(g_device))
  {
    rtcSetGeometryIntersectFilterFunction(geom,intersectionFilter);
    rtcSetGeometryOccludedFilterFunction(geom,occlusionFilter);
  }
  else
  {
    rtcSetGeometryIntersectFilterFunction(geom,intersectionFilterN);
    rtcSetGeometryOccludedFilterFunction(geom,occlusionFilterN);
  }

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene_i,geom);
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
  rtcSetSceneBuildQuality(g_scene, RTC_BUILD_QUALITY_HIGH); // high quality mode to test if we filter out duplicated intersections

  /* add cube */
  addCube(g_scene,Vec3fa(0.0f,0.0f,0.0f),Vec3fa(10.0f,1.0f,1.0f),45.0f);
  //addSubdivCube(g_scene);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommitScene (g_scene);

  /* set start render mode */
  if (g_mode == MODE_NORMAL) renderTile = renderTileStandard;
  else                       renderTile = renderTileStandardStream;
  key_pressed_handler = device_key_pressed_default;
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

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
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
  alignedFree(colors); colors = nullptr;
}

} // namespace embree
