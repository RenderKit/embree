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
RTCDevice g_device = nullptr;
RTCScene g_scene = nullptr;
Vec3fa* colors = nullptr;

/******************************************************************************************/
/*                             Standard Mode                                              */
/******************************************************************************************/

#define HIT_LIST_LENGTH 16

/* extended ray structure that includes total transparency along the ray */
struct RTCRay2
{
  Vec3fa org;     //!< Ray origin
  Vec3fa dir;     //!< Ray direction
  float tnear;   //!< Start of ray segment
  float tfar;    //!< End of ray segment
  float time;    //!< Time of this ray for motion blur.
  unsigned int mask; //!< used to mask out objects during traversal
  Vec3fa Ng;      //!< Geometric normal.
  float u;       //!< Barycentric u coordinate of hit
  float v;       //!< Barycentric v coordinate of hit
  unsigned int geomID; //!< geometry ID
  unsigned int primID; //!< primitive ID
  unsigned int instID; //!< instance ID

  // ray extensions
  float transparency; //!< accumulated transparency value

  // we remember up to 16 hits to ignore duplicate hits
  unsigned int hit_geomIDs[HIT_LIST_LENGTH];
  unsigned int hit_primIDs[HIT_LIST_LENGTH];
  unsigned int firstHit, lastHit;
};

/* 3D procedural transparency */
inline float transparencyFunction(Vec3fa& h)
{
  float v = abs(sin(4.0f*h.x)*cos(4.0f*h.y)*sin(4.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);
  return T;
  //return 0.5f;
}

/* intersection filter function */
void intersectionFilter(void* ptr, RTCRay& ray_i)
{
  RTCRay2& ray = (RTCRay2&) ray_i;
  Vec3fa h = ray.org + ray.dir*ray.tfar;
  float T = transparencyFunction(h);
  if (T >= 1.0f) ray.geomID = RTC_INVALID_GEOMETRY_ID;
  else ray.transparency = T;
}

/* occlusion filter function */
void occlusionFilter(void* ptr, RTCRay& ray_i)
{
  RTCRay2& ray = (RTCRay2&) ray_i;
  /* The occlusion filter function may be called multiple times with
   * the same hit. We remember the last N hits, and skip duplicates. */
  for (size_t i=ray.firstHit; i<ray.lastHit; i++) {
    unsigned slot= i%HIT_LIST_LENGTH;
    if (ray.hit_geomIDs[slot] == ray.geomID && ray.hit_primIDs[slot] == ray.primID) {
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return;
    }
  }

  /* store hit in hit list */
  unsigned int slot = ray.lastHit%HIT_LIST_LENGTH;
  ray.hit_geomIDs[slot] = ray.geomID;
  ray.hit_primIDs[slot] = ray.primID;
  ray.lastHit++;
  if (ray.lastHit - ray.firstHit >= HIT_LIST_LENGTH)
    ray.firstHit++;

  /* calculate and accumulate transparency */
  Vec3fa h = ray.org + ray.dir*ray.tfar;
  float T = transparencyFunction(h);
  T *= ray.transparency;
  ray.transparency = T;
  if (T != 0.0f) ray.geomID = RTC_INVALID_GEOMETRY_ID;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  float weight = 1.0f;
  Vec3fa color = Vec3fa(0.0f);

  /* initialize ray */
  RTCRay2 primary;
  primary.org = Vec3fa(camera.xfm.p);
  primary.dir = Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  primary.tnear = 0.0f;
  primary.tfar = (float)(inf);
  primary.geomID = RTC_INVALID_GEOMETRY_ID;
  primary.primID = RTC_INVALID_GEOMETRY_ID;
  primary.mask = -1;
  primary.time = 0;
  primary.transparency = 0.0f;

  while (true)
  {
    /* intersect ray with scene */
    rtcIntersect(g_scene,*((RTCRay*)&primary));
    RayStats_addRay(stats);

    /* shade pixels */
    if (primary.geomID == RTC_INVALID_GEOMETRY_ID)
      break;

    float opacity = 1.0f-primary.transparency;
    Vec3fa diffuse = colors[primary.primID];
    Vec3fa La = diffuse*0.5f;
    color = color + weight*opacity*La;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    RTCRay2 shadow;
    shadow.org = primary.org + primary.tfar*primary.dir;
    shadow.dir = neg(lightDir);
    shadow.tnear = 0.001f;
    shadow.tfar = (float)(inf);
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;
    shadow.transparency = 1.0f;
    shadow.firstHit = 0;
    shadow.lastHit = 0;

    /* trace shadow ray */
    rtcOccluded(g_scene,*((RTCRay*)&shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.geomID) {
      Vec3fa Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.Ng)),0.0f,1.0f);
      color = color + weight*opacity*Ll;
    }

    /* shoot transmission ray */
    weight *= primary.transparency;
    primary.tnear = 1.001f*primary.tfar;
    primary.tfar = (float)(inf);
    primary.geomID = RTC_INVALID_GEOMETRY_ID;
    primary.primID = RTC_INVALID_GEOMETRY_ID;
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

inline float gather(float& ptr, const size_t stride, const size_t pid, const size_t rid)
{
  float* uptr = (float*) (((char*)&ptr) + pid*stride);
  return uptr[rid];
}

inline unsigned int gather(unsigned int& ptr, const unsigned int idx, const size_t stride, const size_t pid, const size_t rid)
{
  unsigned int* uptr = (unsigned int*) (((char*)&ptr) + pid*stride);
  return uptr[rid + 1*idx];
}

inline void scatter(float& ptr, const size_t stride, const size_t pid, const size_t rid, float v) {
  ((float*)(((char*)&ptr) + pid*stride))[rid] = v;
}

inline void scatter(unsigned int& ptr, const unsigned int idx, const size_t stride, const size_t pid, const size_t rid, unsigned int v) {
  ((unsigned int*)(((char*)&ptr) + pid*stride))[rid+1*idx] = v;
}

/* intersection filter function */
void intersectionFilterN(int* valid,
                                  void* ptr,
                                  const RTCIntersectContext* context,
                                  RTCRayN* ray,
                                  const RTCHitN* potentialHit,
                                  const size_t N)
{
  /* avoid crashing when debug visualizations are used */
  if (context == nullptr)
    return;

  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* read ray from ray structure */
    Vec3fa ray_org = Vec3fa(RTCRayN_org_x(ray,N,ui),RTCRayN_org_y(ray,N,ui),RTCRayN_org_z(ray,N,ui));
    Vec3fa ray_dir = Vec3fa(RTCRayN_dir_x(ray,N,ui),RTCRayN_dir_y(ray,N,ui),RTCRayN_dir_z(ray,N,ui));
    unsigned ray_mask = RTCRayN_mask(ray,N,ui);
    float hit_t = RTCHitN_t(potentialHit,N,ui);

    /* decode ray IDs */
    int pid = (ray_mask & 0xFFFF) / 1;
    int rid = (ray_mask & 0xFFFF) % 1;

    /* calculate transparency */
    Vec3fa h = ray_org + ray_dir*hit_t;
    float T = transparencyFunction(h);

    /* ignore hit if completely transparent */
    if (T >= 1.0f) valid[vi] = 0;

    /* otherwise accept hit and remember transparency */
    else
    {
      RTCRayN_instID(ray,N,ui) = RTCHitN_instID(potentialHit,N,ui);
      RTCRayN_geomID(ray,N,ui) = RTCHitN_geomID(potentialHit,N,ui);
      RTCRayN_primID(ray,N,ui) = RTCHitN_primID(potentialHit,N,ui);

      RTCRayN_u(ray,N,ui) = RTCHitN_u(potentialHit,N,ui);
      RTCRayN_v(ray,N,ui) = RTCHitN_v(potentialHit,N,ui);
      RTCRayN_tfar(ray,N,ui) = RTCHitN_t(potentialHit,N,ui);

      RTCRayN_Ng_x(ray,N,ui) = RTCHitN_Ng_x(potentialHit,N,ui);
      RTCRayN_Ng_y(ray,N,ui) = RTCHitN_Ng_y(potentialHit,N,ui);
      RTCRayN_Ng_z(ray,N,ui) = RTCHitN_Ng_z(potentialHit,N,ui);

      if (context) {
        RTCRay2* eray = (RTCRay2*) context->userRayExt;
        scatter(eray->transparency,sizeof(RTCRay2),pid,rid,T);
      }
    }
  }
}

/* occlusion filter function */
void occlusionFilterN(int* valid,
                               void* ptr,
                               const RTCIntersectContext* context,
                               RTCRayN* ray,
                               const RTCHitN* potentialHit,
                               const size_t N)
{
  /* avoid crashing when debug visualizations are used */
  if (context == nullptr)
    return;

  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* read ray from ray structure */
    Vec3fa ray_org = Vec3fa(RTCRayN_org_x(ray,N,ui),RTCRayN_org_y(ray,N,ui),RTCRayN_org_z(ray,N,ui));
    Vec3fa ray_dir = Vec3fa(RTCRayN_dir_x(ray,N,ui),RTCRayN_dir_y(ray,N,ui),RTCRayN_dir_z(ray,N,ui));
    unsigned ray_mask= RTCRayN_mask(ray,N,ui);
    unsigned hit_geomID = RTCHitN_geomID(potentialHit,N,ui);
    unsigned hit_primID = RTCHitN_primID(potentialHit,N,ui);
    float hit_t   = RTCHitN_t(potentialHit,N,ui);

    /* decode ray IDs */
    int pid = (ray_mask & 0xFFFF) / 1;
    int rid = (ray_mask & 0xFFFF) % 1;
    RTCRay2* eray = (RTCRay2*) context->userRayExt;

    /* The occlusion filter function may be called multiple times with
     * the same hit. We remember the last N hits, and skip duplicates. */
    bool already_hit = false;
    unsigned int eray_firstHit = gather(eray->firstHit,0,sizeof(RTCRay2),pid,rid);
    unsigned int eray_lastHit =  gather(eray->lastHit,0,sizeof(RTCRay2),pid,rid);
    for (unsigned int i=eray_firstHit; i<eray_lastHit; i++)
    {
      unsigned int slot= i%HIT_LIST_LENGTH;
      unsigned int last_geomID = gather(eray->hit_geomIDs[0],slot,sizeof(RTCRay2),pid,rid);
      unsigned int last_primID = gather(eray->hit_primIDs[0],slot,sizeof(RTCRay2),pid,rid);
      if (last_geomID == hit_geomID && last_primID == hit_primID) {
        already_hit = true;
        break;
      }
    }
    if (already_hit) {
      valid[vi] = 0;
      continue;
    }

    /* store hit in hit list */
    unsigned int slot = eray_lastHit%HIT_LIST_LENGTH;
    scatter(eray->hit_geomIDs[0],slot,sizeof(RTCRay2),pid,rid,hit_geomID);
    scatter(eray->hit_primIDs[0],slot,sizeof(RTCRay2),pid,rid,hit_primID);
    eray_lastHit++;
    scatter(eray->lastHit,0,sizeof(RTCRay2),pid,rid,eray_lastHit);
    if (eray_lastHit - eray_firstHit >= HIT_LIST_LENGTH)
      scatter(eray->firstHit,0,sizeof(RTCRay2),pid,rid,eray_firstHit+1);

    /* calculate transparency */
    Vec3fa h = ray_org + ray_dir*hit_t;
    float T = transparencyFunction(h);

    /* accumulate transparency and store inside ray extensions */
    T *= gather(eray->transparency,sizeof(RTCRay2),pid,rid);
    scatter(eray->transparency,sizeof(RTCRay2),pid,rid,T);

    /* reject a hit if not fully opqaue */
    if (T != 0.0f) valid[vi] = 0;

    /* otherwise accept the hit */
    else RTCRayN_geomID(ray,N,ui) = 0;
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

  RTCRay2 primary_stream[TILE_SIZE_X*TILE_SIZE_Y];
  RTCRay2 shadow_stream[TILE_SIZE_X*TILE_SIZE_Y];
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
    RTCRay2& primary = primary_stream[N];
    primary.org = Vec3fa(camera.xfm.p);
    primary.dir = Vec3fa(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz));
    mask = 1; { // invalidates inactive rays
      primary.tnear = mask ? 0.0f         : (float)(pos_inf);
      primary.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    primary.geomID = RTC_INVALID_GEOMETRY_ID;
    primary.primID = RTC_INVALID_GEOMETRY_ID;
    primary.mask = 0xFFFF0000 + N*1 + 0;
    primary.time = 0.0f;
    primary.transparency = 0.0f;
    N++;
    RayStats_addRay(stats);
  }

  Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

  while (numActive)
  {
    /* trace rays */
    RTCIntersectContext primary_context;
    primary_context.flags = g_iflags_coherent;
    primary_context.userRayExt = &primary_stream;
    rtcIntersect1M(g_scene,&primary_context,(RTCRay*)&primary_stream,N,sizeof(RTCRay2));

    /* terminate rays and update color */
    N = -1;
    for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
    {
      N++;
      /* ISPC workaround for mask == 0 */
      

      /* invalidate shadow rays by default */
      RTCRay2& shadow = shadow_stream[N];
      {
        shadow.tnear = (float)(pos_inf);
        shadow.tfar  = (float)(neg_inf);
      }

      /* ignore invalid rays */
      if (valid_stream[N] == false) continue;

      /* terminate rays that hit nothing */
      if (primary_stream[N].geomID == RTC_INVALID_GEOMETRY_ID) {
        valid_stream[N] = false;
        continue;
      }

      /* update color */
      RTCRay2& primary = primary_stream[N];
      float opacity = 1.0f-primary.transparency;
      Vec3fa diffuse = colors[primary.primID];
      Vec3fa La = diffuse*0.5f;
      color_stream[N] = color_stream[N] + weight_stream[N]*opacity*La;

      /* initialize shadow ray */
      shadow.org = primary.org + primary.tfar*primary.dir;
      shadow.dir = neg(lightDir);
      bool mask = 1; {
        shadow.tnear = mask ? 0.001f       : (float)(pos_inf);
        shadow.tfar  = mask ? (float)(inf) : (float)(neg_inf);
      }
      shadow.geomID = RTC_INVALID_GEOMETRY_ID;
      shadow.primID = RTC_INVALID_GEOMETRY_ID;
      shadow.mask = 0xFFFF0000 + N*1 + 0;
      shadow.time = 0;
      shadow.transparency = 1.0f;
      shadow.firstHit = 0;
      shadow.lastHit = 0;
      RayStats_addShadowRay(stats);
    }
    N++;

    /* trace shadow rays */
    RTCIntersectContext shadow_context;
    shadow_context.flags = g_iflags_coherent;
    shadow_context.userRayExt = &shadow_stream;
    rtcOccluded1M(g_scene,&shadow_context,(RTCRay*)&shadow_stream,N,sizeof(RTCRay2));

    /* add light contribution and generate transmission ray */
    N = -1;
    numActive = 0;
    for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
    {
      N++;
      /* ISPC workaround for mask == 0 */
      

      /* invalidate rays by default */
      RTCRay2& primary = primary_stream[N];
      float primary_tfar = primary.tfar;
      {
        primary.tnear = (float)(pos_inf);
        primary.tfar  = (float)(neg_inf);
      }

      /* ignore invalid rays */
      if (valid_stream[N] == false) continue;
      numActive++;

      /* add light contrinution */
      float opacity = 1.0f-primary.transparency;
      Vec3fa diffuse = colors[primary.primID];
      RTCRay2& shadow = shadow_stream[N];
      if (shadow.geomID) {
        Vec3fa Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.Ng)),0.0f,1.0f);
        color_stream[N] = color_stream[N] + weight_stream[N]*opacity*Ll;
      }

      /* initialize transmission ray */
      weight_stream[N] *= primary.transparency;
      bool mask = 1; {
        primary.tnear = mask ? 1.001f*primary_tfar : (float)(pos_inf);
        primary.tfar  = mask ? (float)(inf)        : (float)(neg_inf);
      }
      primary.geomID = RTC_INVALID_GEOMETRY_ID;
      primary.primID = RTC_INVALID_GEOMETRY_ID;
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
  0, 2, 3, 1,
  5, 7, 6, 4,
  0, 1, 5, 4,
  6, 7, 3, 2,
  0, 4, 6, 2,
  3, 7, 5, 1
};

unsigned int cube_tri_indices[NUM_TRI_INDICES] = {
  0, 2, 1,  2, 3, 1,
  5, 7, 4,  7, 6, 4,
  0, 1, 4,  1, 5, 4,
  6, 7, 2,  7, 3, 2,
  0, 4, 2,  4, 6, 2,
  3, 7, 1,  7, 5, 1
};

unsigned int cube_quad_faces[NUM_QUAD_FACES] = {
  4, 4, 4, 4, 4, 4
};

/* adds a cube to the scene */
unsigned int addCube (RTCScene scene_i, const Vec3fa& offset, const Vec3fa& scale, float rotation)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  unsigned int geomID = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, NUM_TRI_FACES, NUM_VERTICES);
  //rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER, cube_vertices,     0, sizeof(Vec3fa  ));
  Vec3fa* ptr = (Vec3fa*) rtcMapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  for (size_t i=0; i<NUM_VERTICES; i++) {
    float x = cube_vertices[i][0];
    float y = cube_vertices[i][1];
    float z = cube_vertices[i][2];
    Vec3fa vtx = Vec3fa(x,y,z);
    ptr[i] = Vec3fa(offset+LinearSpace3fa::rotate(Vec3fa(0,1,0),rotation)*LinearSpace3fa::scale(scale)*vtx);
  }
  rtcUnmapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER);
  rtcSetBuffer(scene_i, geomID, RTC_INDEX_BUFFER,  cube_tri_indices , 0, 3*sizeof(unsigned int));

  /* create per-triangle color array */
  colors = (Vec3fa*) alignedMalloc(12*sizeof(Vec3fa));
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
  if (g_mode != MODE_NORMAL) {
    rtcSetIntersectionFilterFunctionN(scene_i,geomID,intersectionFilterN);
    rtcSetOcclusionFilterFunctionN   (scene_i,geomID,occlusionFilterN);
  }
  else {
    rtcSetIntersectionFilterFunction(scene_i,geomID,intersectionFilter);
    rtcSetOcclusionFilterFunction   (scene_i,geomID,occlusionFilter);
  }

  return geomID;
}

/* adds a cube to the scene */
unsigned int addSubdivCube (RTCScene scene_i)
{
  unsigned int geomID = rtcNewSubdivisionMesh(scene_i, RTC_GEOMETRY_STATIC, NUM_QUAD_FACES, NUM_QUAD_INDICES, NUM_VERTICES, 0, 0, 0);
  rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER, cube_vertices,      0, sizeof(Vec3fa  ));
  rtcSetBuffer(scene_i, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, sizeof(unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_FACE_BUFFER,   cube_quad_faces,    0, sizeof(unsigned int));

  float* level = (float*) rtcMapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
  for (size_t i=0; i<NUM_QUAD_INDICES; i++) level[i] = 4;
  rtcUnmapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);

  /* create face color array */
  colors = (Vec3fa*) alignedMalloc(6*sizeof(Vec3fa));
  colors[0] = Vec3fa(1,0,0); // left side
  colors[1] = Vec3fa(0,1,0); // right side
  colors[2] = Vec3fa(0.5f);  // bottom side
  colors[3] = Vec3fa(1.0f);  // top side
  colors[4] = Vec3fa(0,0,1); // front side
  colors[5] = Vec3fa(1,1,0); // back side

  /* set intersection filter for the cube */
  if (g_mode != MODE_NORMAL) {
    rtcSetIntersectionFilterFunctionN(scene_i,geomID,intersectionFilterN);
    rtcSetOcclusionFilterFunctionN   (scene_i,geomID,occlusionFilterN);
  }
  else {
    rtcSetIntersectionFilterFunction(scene_i,geomID,intersectionFilter);
    rtcSetOcclusionFilterFunction   (scene_i,geomID,occlusionFilter);
  }

  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  unsigned int mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 2, 4);

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
extern "C" void device_init (char* cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(nullptr,rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction2(g_device,error_handler,nullptr);

  /* create scene */
  RTCAlgorithmFlags aflags;
  if (g_mode == MODE_NORMAL) aflags = RTC_INTERSECT1;
  else                       aflags = RTC_INTERSECT1 | RTC_INTERSECT_STREAM;
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC | RTC_SCENE_HIGH_QUALITY,aflags);

  /* add cube */
  addCube(g_scene,Vec3fa(0.0f,0.0f,0.0f),Vec3fa(10.0f,1.0f,1.0f),45.0f);
  //addSubdivCube(g_scene);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);

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
  rtcDeleteScene (g_scene); g_scene = nullptr;
  rtcDeleteDevice(g_device); g_device = nullptr;
  alignedFree(colors); colors = nullptr;
}

} // namespace embree
