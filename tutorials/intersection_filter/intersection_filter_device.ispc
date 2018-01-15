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

#include "../common/tutorial/tutorial_device.isph"

/* scene data */
RTCDevice g_device = NULL;
RTCScene g_scene = NULL;
uniform Vec3f* uniform colors = NULL;

/******************************************************************************************/
/*                             Standard Mode                                              */
/******************************************************************************************/

#define HIT_LIST_LENGTH 16

/* extended ray structure that includes total transparency along the ray */
struct RTCRay2
{
  Vec3f org;     //!< Ray origin
  Vec3f dir;     //!< Ray direction
  float tnear;   //!< Start of ray segment
  float tfar;    //!< End of ray segment
  float time;    //!< Time of this ray for motion blur.
  unsigned int mask; //!< used to mask out objects during traversal
  Vec3f Ng;      //!< Geometric normal.
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
inline float transparencyFunction(Vec3f& h)
{
  float v = abs(sin(4.0f*h.x)*cos(4.0f*h.y)*sin(4.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);
  return T;
  //return 0.5f;
}

/* intersection filter function */
void intersectionFilter(void* uniform ptr, RTCRay& ray_i)
{
  RTCRay2& ray = (RTCRay2&) ray_i;
  Vec3f h = ray.org + ray.dir*ray.tfar;
  float T = transparencyFunction(h);
  if (T >= 1.0f) ray.geomID = RTC_INVALID_GEOMETRY_ID;
  else ray.transparency = T;
}

/* occlusion filter function */
void occlusionFilter(void* uniform ptr, RTCRay& ray_i)
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
  Vec3f h = ray.org + ray.dir*ray.tfar;
  float T = transparencyFunction(h);
  T *= ray.transparency;
  ray.transparency = T;
  if (T != 0.0f) ray.geomID = RTC_INVALID_GEOMETRY_ID;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  float weight = 1.0f;
  Vec3f color = make_Vec3f(0.0f);

  /* initialize ray */
  RTCRay2 primary;
  primary.org = make_Vec3f(camera.xfm.p);
  primary.dir = make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
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
    rtcIntersect(g_scene,*((varying RTCRay*)&primary));
    RayStats_addRay(stats);

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
    shadow.tfar = (float)(inf);
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;
    shadow.transparency = 1.0f;
    shadow.firstHit = 0;
    shadow.lastHit = 0;

    /* trace shadow ray */
    rtcOccluded(g_scene,*((varying RTCRay*)&shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.geomID) {
      Vec3f Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.Ng)),0.0f,1.0f);
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
void renderTileStandard(uniform int taskIndex,
                        uniform int threadIndex,
                        uniform int* uniform pixels,
                        const uniform unsigned int width,
                        const uniform unsigned int height,
                        const uniform float time,
                        const uniform ISPCCamera& camera,
                        const uniform int numTilesX,
                        const uniform int numTilesY)
{
  const uniform unsigned int tileY = taskIndex / numTilesX;
  const uniform unsigned int tileX = taskIndex - tileY * numTilesX;
  const uniform unsigned int x0 = tileX * TILE_SIZE_X;
  const uniform unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const uniform unsigned int y0 = tileY * TILE_SIZE_Y;
  const uniform unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* calculate pixel color */
    Vec3f color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

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

inline float gather(varying float& ptr, const uniform size_t stride, const size_t pid, const size_t rid)
{
  uniform float* uptr = (uniform float*) (((uniform int8* uniform)&ptr) + pid*stride);
  return uptr[rid];
}

inline unsigned int gather(varying unsigned int& ptr, const varying unsigned int idx, const uniform size_t stride, const size_t pid, const size_t rid)
{
  uniform unsigned int* uptr = (uniform unsigned int*) (((uniform int8* uniform)&ptr) + pid*stride);
  return uptr[rid + programCount*idx];
}

inline void scatter(varying float& ptr, const uniform size_t stride, const size_t pid, const size_t rid, float v) {
  ((uniform float*)(((uniform int8* uniform)&ptr) + pid*stride))[rid] = v;
}

inline void scatter(varying unsigned int& ptr, const varying unsigned int idx, const uniform size_t stride, const size_t pid, const size_t rid, unsigned int v) {
  ((uniform unsigned int*)(((uniform int8* uniform)&ptr) + pid*stride))[rid+programCount*idx] = v;
}

/* intersection filter function */
unmasked void intersectionFilterN(uniform int* uniform valid,
                                  void* uniform ptr,
                                  const uniform RTCIntersectContext* uniform context,
                                  RTCRayN* uniform ray,
                                  const RTCHitN* uniform potentialHit,
                                  uniform const uintptr_t N)
{
  /* avoid crashing when debug visualizations are used */
  if (context == NULL)
    return;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* read ray from ray structure */
    Vec3f ray_org = make_Vec3f(RTCRayN_org_x(ray,N,ui),RTCRayN_org_y(ray,N,ui),RTCRayN_org_z(ray,N,ui));
    Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(ray,N,ui),RTCRayN_dir_y(ray,N,ui),RTCRayN_dir_z(ray,N,ui));
    unsigned ray_mask = RTCRayN_mask(ray,N,ui);
    float hit_t = RTCHitN_t(potentialHit,N,ui);

    /* decode ray IDs */
    int pid = (ray_mask & 0xFFFF) / programCount;
    int rid = (ray_mask & 0xFFFF) % programCount;

    /* calculate transparency */
    Vec3f h = ray_org + ray_dir*hit_t;
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
        varying RTCRay2* uniform eray = (varying RTCRay2* uniform) context->userRayExt;
        scatter(eray->transparency,sizeof(RTCRay2),pid,rid,T);
      }
    }
  }
}

/* occlusion filter function */
unmasked void occlusionFilterN(uniform int* uniform valid,
                               void* uniform ptr,
                               const uniform RTCIntersectContext* uniform context,
                               RTCRayN* uniform ray,
                               const RTCHitN* uniform potentialHit,
                               uniform const uintptr_t N)
{
  /* avoid crashing when debug visualizations are used */
  if (context == NULL)
    return;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* read ray from ray structure */
    Vec3f ray_org = make_Vec3f(RTCRayN_org_x(ray,N,ui),RTCRayN_org_y(ray,N,ui),RTCRayN_org_z(ray,N,ui));
    Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(ray,N,ui),RTCRayN_dir_y(ray,N,ui),RTCRayN_dir_z(ray,N,ui));
    unsigned ray_mask= RTCRayN_mask(ray,N,ui);
    unsigned hit_geomID = RTCHitN_geomID(potentialHit,N,ui);
    unsigned hit_primID = RTCHitN_primID(potentialHit,N,ui);
    float hit_t   = RTCHitN_t(potentialHit,N,ui);

    /* decode ray IDs */
    int pid = (ray_mask & 0xFFFF) / programCount;
    int rid = (ray_mask & 0xFFFF) % programCount;
    varying RTCRay2* uniform eray = (varying RTCRay2* uniform) context->userRayExt;

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
    Vec3f h = ray_org + ray_dir*hit_t;
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
void renderTileStandardStream(uniform int taskIndex,
                              uniform int threadIndex,
                              uniform int* uniform pixels,
                              const uniform unsigned int width,
                              const uniform unsigned int height,
                              const uniform float time,
                              const uniform ISPCCamera& camera,
                              const uniform int numTilesX,
                              const uniform int numTilesY)
{
  const uniform unsigned int tileY = taskIndex / numTilesX;
  const uniform unsigned int tileX = taskIndex - tileY * numTilesX;
  const uniform unsigned int x0 = tileX * TILE_SIZE_X;
  const uniform unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const uniform unsigned int y0 = tileY * TILE_SIZE_Y;
  const uniform unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  uniform RayStats& stats = g_stats[threadIndex];

  RTCRay2 primary_stream[TILE_SIZE_X*TILE_SIZE_Y];
  RTCRay2 shadow_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Vec3f color_stream[TILE_SIZE_X*TILE_SIZE_Y];
  float weight_stream[TILE_SIZE_X*TILE_SIZE_Y];
  bool valid_stream[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  uniform int N = 0;
  uniform int numActive = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* initialize variables */
    numActive++;
    color_stream[N] = make_Vec3f(0.0f);
    weight_stream[N] = 1.0f;
    bool mask = __mask; unmasked { valid_stream[N] = mask; }

    /* initialize ray */
    RTCRay2& primary = primary_stream[N];
    primary.org = make_Vec3f(camera.xfm.p);
    primary.dir = make_Vec3f(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz));
    mask = __mask; unmasked { // invalidates inactive rays
      primary.tnear = mask ? 0.0f         : (float)(pos_inf);
      primary.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    primary.geomID = RTC_INVALID_GEOMETRY_ID;
    primary.primID = RTC_INVALID_GEOMETRY_ID;
    primary.mask = 0xFFFF0000 + N*programCount + programIndex;
    primary.time = 0.0f;
    primary.transparency = 0.0f;
    N++;
    RayStats_addRay(stats);
  }

  Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

  while (numActive)
  {
    /* trace rays */
    uniform RTCIntersectContext primary_context;
    primary_context.flags = g_iflags_coherent;
    primary_context.userRayExt = &primary_stream;
    rtcIntersectVM(g_scene,&primary_context,(varying RTCRay* uniform)&primary_stream,N,sizeof(RTCRay2));

    /* terminate rays and update color */
    N = -1;
    foreach_tiled (y = y0 ... y1, x = x0 ... x1)
    {
      N++;
      /* ISPC workaround for mask == 0 */
      if (all(__mask == 0)) continue;

      /* invalidate shadow rays by default */
      RTCRay2& shadow = shadow_stream[N];
      unmasked {
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
      Vec3f diffuse = colors[primary.primID];
      Vec3f La = diffuse*0.5f;
      color_stream[N] = color_stream[N] + weight_stream[N]*opacity*La;

      /* initialize shadow ray */
      shadow.org = primary.org + primary.tfar*primary.dir;
      shadow.dir = neg(lightDir);
      bool mask = __mask; unmasked {
        shadow.tnear = mask ? 0.001f       : (float)(pos_inf);
        shadow.tfar  = mask ? (float)(inf) : (float)(neg_inf);
      }
      shadow.geomID = RTC_INVALID_GEOMETRY_ID;
      shadow.primID = RTC_INVALID_GEOMETRY_ID;
      shadow.mask = 0xFFFF0000 + N*programCount + programIndex;
      shadow.time = 0;
      shadow.transparency = 1.0f;
      shadow.firstHit = 0;
      shadow.lastHit = 0;
      RayStats_addShadowRay(stats);
    }
    N++;

    /* trace shadow rays */
    uniform RTCIntersectContext shadow_context;
    shadow_context.flags = g_iflags_coherent;
    shadow_context.userRayExt = &shadow_stream;
    rtcOccludedVM(g_scene,&shadow_context,(varying RTCRay* uniform)&shadow_stream,N,sizeof(RTCRay2));

    /* add light contribution and generate transmission ray */
    N = -1;
    numActive = 0;
    foreach_tiled (y = y0 ... y1, x = x0 ... x1)
    {
      N++;
      /* ISPC workaround for mask == 0 */
      if (all(__mask == 0)) continue;

      /* invalidate rays by default */
      RTCRay2& primary = primary_stream[N];
      float primary_tfar = primary.tfar;
      unmasked {
        primary.tnear = (float)(pos_inf);
        primary.tfar  = (float)(neg_inf);
      }

      /* ignore invalid rays */
      if (valid_stream[N] == false) continue;
      numActive++;

      /* add light contrinution */
      float opacity = 1.0f-primary.transparency;
      Vec3f diffuse = colors[primary.primID];
      RTCRay2& shadow = shadow_stream[N];
      if (shadow.geomID) {
        Vec3f Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.Ng)),0.0f,1.0f);
        color_stream[N] = color_stream[N] + weight_stream[N]*opacity*Ll;
      }

      /* initialize transmission ray */
      weight_stream[N] *= primary.transparency;
      bool mask = __mask; unmasked {
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
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

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

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene_i, const uniform Vec3f& offset, const uniform Vec3f& scale, uniform float rotation)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  uniform unsigned int geomID = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, NUM_TRI_FACES, NUM_VERTICES);
  //rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER, cube_vertices,     0, sizeof(uniform Vec3fa  ));
  Vec3fa* uniform ptr = (Vec3fa* uniform) rtcMapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  for (uniform size_t i=0; i<NUM_VERTICES; i++) {
    uniform float x = cube_vertices[i][0];
    uniform float y = cube_vertices[i][1];
    uniform float z = cube_vertices[i][2];
    uniform Vec3f vtx = make_Vec3f(x,y,z);
    ptr[i] = make_Vec3fa(offset+make_LinearSpace3f_rotate(make_Vec3f(0,1,0),rotation)*make_LinearSpace3f_scale(scale)*vtx);
  }
  rtcUnmapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER);
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
  error_handler(NULL,rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction2(g_device,error_handler,NULL);

  /* create scene */
  uniform RTCAlgorithmFlags aflags;
  if (g_mode == MODE_NORMAL) aflags = RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING;
  else                       aflags = RTC_INTERSECT_UNIFORM | RTC_INTERSECT_STREAM;
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC | RTC_SCENE_HIGH_QUALITY,aflags);

  /* add cube */
  addCube(g_scene,make_Vec3f(0.0f,0.0f,0.0f),make_Vec3f(10.0f,1.0f,1.0f),45.0f);
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
task void renderTileTask(uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const uniform float time,
                         const uniform ISPCCamera& camera,
                         const uniform int numTilesX,
                         const uniform int numTilesY)
{
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
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
