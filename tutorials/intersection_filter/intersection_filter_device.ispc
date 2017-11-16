// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
  unsigned int hit_geomIDs[HIT_LIST_LENGTH];
  unsigned int hit_primIDs[HIT_LIST_LENGTH];
  unsigned int firstHit, lastHit;
};

inline varying RTCRay* uniform RTCRay_(varying Ray2& ray)
{
  varying RTCRay* uniform ray_ptr = (varying RTCRay* uniform)&ray;
  return ray_ptr;
}

/* 3D procedural transparency */
inline float transparencyFunction(Vec3f& h)
{
  float v = abs(sin(4.0f*h.x)*cos(4.0f*h.y)*sin(4.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);
  return T;
  //return 0.5f;
}


/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  float weight = 1.0f;
  Vec3f color = make_Vec3f(0.0f);

  uniform IntersectContext context;
  InitIntersectionContext(&context);
  
  /* initialize ray */
  Ray2 primary;
  init_Ray(primary.ray,make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);
  primary.ray.mask = programIndex; // needs to encode rayID for filter
  primary.transparency = 0.0f;


  while (true)
  {
    context.userRayExt = &primary;

    /* intersect ray with scene */
    rtcIntersect(g_scene,&context.context,RTCRay_(primary));
    RayStats_addRay(stats);

    /* shade pixels */
    if (primary.ray.geomID == RTC_INVALID_GEOMETRY_ID)
      break;

    float opacity = 1.0f-primary.transparency;
    Vec3f diffuse = colors[primary.ray.primID];
    Vec3f La = diffuse*0.5f;
    color = color + weight*opacity*La;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    Ray2 shadow;
    init_Ray(shadow.ray, primary.ray.org + primary.ray.tfar*primary.ray.dir, neg(lightDir), 0.001f, inf);
    shadow.ray.mask = programIndex; // needs to encode rayID for filter
    shadow.transparency = 1.0f;
    shadow.firstHit = 0;
    shadow.lastHit = 0;
    context.userRayExt = &shadow;

    /* trace shadow ray */
    rtcOccluded(g_scene,&context.context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.ray.geomID) {
      Vec3f Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.ray.Ng)),0.0f,1.0f);
      color = color + weight*opacity*Ll;
    }

    /* shoot transmission ray */
    weight *= primary.transparency;
    primary.ray.tnear = 1.001f*primary.ray.tfar;
    primary.ray.tfar = (float)(inf);
    primary.ray.geomID = RTC_INVALID_GEOMETRY_ID;
    primary.ray.primID = RTC_INVALID_GEOMETRY_ID;
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
unmasked void intersectionFilterN(const RTCFilterFunctionNArguments* uniform const args)
{
  uniform int* uniform valid = args->valid;
  const uniform IntersectContext* uniform context = (const uniform IntersectContext* uniform) args->context;
  struct RTCRayN* uniform ray = args->ray;
  struct RTCHitN* uniform potentialHit = args->potentialHit;
  const uniform unsigned int N = args->N;
                                  
  /* avoid crashing when debug visualizations are used */
  if (context == NULL)
    return;

  /* fast path for SIMD width == N and only a single input packet */
  unsigned mask = RTCRayN_mask(ray,N,0);
  if (programCount == N && all((mask & 0x80000000) == 0))
  {
    /* ignore inactive rays */
    if (valid[programIndex] != -1) return;
  
    const uniform unsigned int rayID = 0;
    const float tfar   = RTCHitN_t(potentialHit,N,rayID);
    varying Ray2 *uniform ray2 = (varying Ray2* uniform) context->userRayExt;

    Vec3f h = ray2->ray.org + ray2->ray.dir*tfar;

    float T = transparencyFunction(h);
    if (T < 1.0f) 
    {
      ray2->transparency = T;
      // ray2->ray.instID = RTCHitN_instID(potentialHit,N,rayID);
      // ray2->ray.geomID = RTCHitN_geomID(potentialHit,N,rayID);
      // ray2->ray.primID = RTCHitN_primID(potentialHit,N,rayID);
      // ray2->ray.u      = RTCHitN_u(potentialHit,N,rayID);
      // ray2->ray.v      = RTCHitN_v(potentialHit,N,rayID);
      // ray2->ray.tfar   = tfar;
      // ray2->ray.Ng.x   = RTCHitN_Ng_x(potentialHit,N,rayID);
      // ray2->ray.Ng.y   = RTCHitN_Ng_y(potentialHit,N,rayID);
      // ray2->ray.Ng.z   = RTCHitN_Ng_z(potentialHit,N,rayID);
    }
    else
      valid[programIndex] = 0;

    return;    
  }

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
    if (T >= 1.0f) 
      valid[vi] = 0;

    /* otherwise accept hit and remember transparency */
    else
    {
      // RTCRayN_instID(ray,N,ui) = RTCHitN_instID(potentialHit,N,ui);
      // RTCRayN_geomID(ray,N,ui) = RTCHitN_geomID(potentialHit,N,ui);
      // RTCRayN_primID(ray,N,ui) = RTCHitN_primID(potentialHit,N,ui);

      // RTCRayN_u(ray,N,ui) = RTCHitN_u(potentialHit,N,ui);
      // RTCRayN_v(ray,N,ui) = RTCHitN_v(potentialHit,N,ui);
      // RTCRayN_tfar(ray,N,ui) = RTCHitN_t(potentialHit,N,ui);

      // RTCRayN_Ng_x(ray,N,ui) = RTCHitN_Ng_x(potentialHit,N,ui);
      // RTCRayN_Ng_y(ray,N,ui) = RTCHitN_Ng_y(potentialHit,N,ui);
      // RTCRayN_Ng_z(ray,N,ui) = RTCHitN_Ng_z(potentialHit,N,ui);

      if (context) {
        varying Ray2* uniform eray = (varying Ray2* uniform) context->userRayExt;
        assert(eray);
        scatter(eray->transparency,sizeof(Ray2),pid,rid,T);
      }
    }
  }
}

/* occlusion filter function */
unmasked void occlusionFilterN(const RTCFilterFunctionNArguments* uniform const args)
{
  uniform int* uniform valid = args->valid;
  const uniform IntersectContext* uniform context = (const uniform IntersectContext* uniform) args->context;
  struct RTCRayN* uniform ray = args->ray;
  struct RTCHitN* uniform potentialHit = args->potentialHit;
  const uniform unsigned int N = args->N;
                                  
  /* avoid crashing when debug visualizations are used */
  if (context == NULL)
    return;

  /* fast path for SIMD width == N and only a single input packet */
  unsigned mask = RTCRayN_mask(ray,N,0);
  if (programCount == N && all((mask & 0x80000000) == 0))
  {
    /* ignore inactive rays */
    if (valid[programIndex] != -1) return;
  
    const uniform unsigned int rayID = 0;
    const unsigned int geomID = RTCHitN_geomID(potentialHit,N,rayID);
    const unsigned int primID = RTCHitN_primID(potentialHit,N,rayID);
    const float tfar          = RTCHitN_t(potentialHit,N,rayID);
    varying Ray2 *uniform ray2 = (varying Ray2* uniform) context->userRayExt;
    assert(ray2);

    for (size_t i=ray2->firstHit; i<ray2->lastHit; i++) {
      unsigned slot= i%HIT_LIST_LENGTH;
      if (ray2->hit_geomIDs[slot] == geomID && ray2->hit_primIDs[slot] == primID) {
        //valid[programIndex] = 0;
        return;
      }
    }

    /* store hit in hit list */
    unsigned int slot = ray2->lastHit%HIT_LIST_LENGTH;
    ray2->hit_geomIDs[slot] = geomID;
    ray2->hit_primIDs[slot] = primID;
    ray2->lastHit++;
    if (ray2->lastHit - ray2->firstHit >= HIT_LIST_LENGTH)
      ray2->firstHit++;

    Vec3f h = ray2->ray.org + ray2->ray.dir*tfar;

    /* calculate and accumulate transparency */
    float T = transparencyFunction(h);
    T *= ray2->transparency;
    ray2->transparency = T;
    if (T != 0.0f) 
      valid[programIndex] = 0;
    return;    
  }

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
    varying Ray2* uniform eray = (varying Ray2* uniform) context->userRayExt;
    assert(eray);

    /* The occlusion filter function may be called multiple times with
     * the same hit. We remember the last N hits, and skip duplicates. */
    bool already_hit = false;
    unsigned int eray_firstHit = gather(eray->firstHit,0,sizeof(Ray2),pid,rid);
    unsigned int eray_lastHit =  gather(eray->lastHit,0,sizeof(Ray2),pid,rid);
    for (unsigned int i=eray_firstHit; i<eray_lastHit; i++)
    {
      unsigned int slot= i%HIT_LIST_LENGTH;
      unsigned int last_geomID = gather(eray->hit_geomIDs[0],slot,sizeof(Ray2),pid,rid);
      unsigned int last_primID = gather(eray->hit_primIDs[0],slot,sizeof(Ray2),pid,rid);
      if (last_geomID == hit_geomID && last_primID == hit_primID) {
        already_hit = true;
        break;
      }
    }
    if (already_hit) {
      //valid[vi] = 0;
      continue;
    }

    /* store hit in hit list */
    unsigned int slot = eray_lastHit%HIT_LIST_LENGTH;
    scatter(eray->hit_geomIDs[0],slot,sizeof(Ray2),pid,rid,hit_geomID);
    scatter(eray->hit_primIDs[0],slot,sizeof(Ray2),pid,rid,hit_primID);
    eray_lastHit++;
    scatter(eray->lastHit,0,sizeof(Ray2),pid,rid,eray_lastHit);
    if (eray_lastHit - eray_firstHit >= HIT_LIST_LENGTH)
      scatter(eray->firstHit,0,sizeof(Ray2),pid,rid,eray_firstHit+1);

    /* calculate transparency */
    Vec3f h = ray_org + ray_dir*hit_t;
    float T = transparencyFunction(h);

    /* accumulate transparency and store inside ray extensions */
    T *= gather(eray->transparency,sizeof(Ray2),pid,rid);
    scatter(eray->transparency,sizeof(Ray2),pid,rid,T);

    /* reject a hit if not fully opqaue */
    if (T != 0.0f) 
      valid[vi] = 0;
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

  Ray2 primary_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Ray2 shadow_stream[TILE_SIZE_X*TILE_SIZE_Y];
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
    Ray2& primary = primary_stream[N];
    mask = __mask; unmasked { // invalidates inactive rays
      primary.ray.tnear = mask ? 0.0f         : (float)(pos_inf);
      primary.ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(primary.ray, make_Vec3f(camera.xfm.p), make_Vec3f(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), primary.ray.tnear, primary.ray.tfar);
 
    primary.ray.mask = 0xFFFF0000 + N*programCount + programIndex;
    primary.transparency = 0.0f;

    N++;
    RayStats_addRay(stats);
  }

  Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

  while (numActive)
  {
    /* trace rays */
    uniform IntersectContext primary_context;
    InitIntersectionContext(&primary_context);
    primary_context.context.flags = g_iflags_coherent;
    primary_context.userRayExt = &primary_stream;
    rtcIntersectVM(g_scene,&primary_context.context,(varying RTCRay* uniform)&primary_stream,N,sizeof(Ray2));

    /* terminate rays and update color */
    N = -1;
    foreach_tiled (y = y0 ... y1, x = x0 ... x1)
    {
      N++;
      /* ISPC workaround for mask == 0 */
      if (all(__mask == 0)) continue;

      /* invalidate shadow rays by default */
      Ray2& shadow = shadow_stream[N];
      unmasked {
        shadow.ray.tnear = (float)(pos_inf);
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
      Vec3f diffuse = colors[primary.ray.primID];
      Vec3f La = diffuse*0.5f;
      color_stream[N] = color_stream[N] + weight_stream[N]*opacity*La;

      /* initialize shadow ray */
      bool mask = __mask; unmasked {
        shadow.ray.tnear = mask ? 0.001f       : (float)(pos_inf);
        shadow.ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
      }
      init_Ray(shadow.ray, primary.ray.org + primary.ray.tfar*primary.ray.dir, neg(lightDir), shadow.ray.tnear, shadow.ray.tfar);
      shadow.ray.mask = 0xFFFF0000 + N*programCount + programIndex;
      shadow.transparency = 1.0f;
      shadow.firstHit = 0;
      shadow.lastHit = 0;
      RayStats_addShadowRay(stats);
    }
    N++;

    /* trace shadow rays */
    uniform IntersectContext shadow_context;
    InitIntersectionContext(&shadow_context);
    shadow_context.context.flags = g_iflags_coherent;
    shadow_context.userRayExt = &shadow_stream;
    rtcOccludedVM(g_scene,&shadow_context.context,(varying RTCRay* uniform)&shadow_stream,N,sizeof(Ray2));

    /* add light contribution and generate transmission ray */
    N = -1;
    numActive = 0;
    foreach_tiled (y = y0 ... y1, x = x0 ... x1)
    {
      N++;
      /* ISPC workaround for mask == 0 */
      if (all(__mask == 0)) continue;

      /* invalidate rays by default */
      Ray2& primary = primary_stream[N];
      float primary_tfar = primary.ray.tfar;
      unmasked {
        primary.ray.tnear = (float)(pos_inf);
        primary.ray.tfar  = (float)(neg_inf);
      }

      /* ignore invalid rays */
      if (valid_stream[N] == false) continue;
      numActive++;

      /* add light contrinution */
      float opacity = 1.0f-primary.transparency;
      Vec3f diffuse = colors[primary.ray.primID];
      Ray2& shadow = shadow_stream[N];
      if (shadow.ray.geomID) {
        Vec3f Ll = diffuse*shadow.transparency*clamp(-dot(lightDir,normalize(primary.ray.Ng)),0.0f,1.0f);
        color_stream[N] = color_stream[N] + weight_stream[N]*opacity*Ll;
      }

      /* initialize transmission ray */
      weight_stream[N] *= primary.transparency;
      bool mask = __mask; unmasked {
        primary.ray.tnear = mask ? 1.001f*primary_tfar : (float)(pos_inf);
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
  0, 1, 3, 2,
  5, 4, 6, 7,
  0, 4, 5, 1,
  6, 2, 3, 7,
  0, 2, 6, 4,
  3, 1, 5, 7
};

uniform unsigned int cube_tri_indices[NUM_TRI_INDICES] = {
  0, 1, 2,  2, 1, 3,
  5, 4, 7,  7, 4, 6,
  0, 4, 1,  1, 4, 5,
  6, 2, 7,  7, 2, 3,
  0, 2, 4,  4, 2, 6,
  3, 1, 7,  7, 1, 5
};

uniform unsigned int cube_quad_faces[NUM_QUAD_FACES] = {
  4, 4, 4, 4, 4, 4
};

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene_i, const uniform Vec3f& offset, const uniform Vec3f& scale, uniform float rotation)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry geom = rtcNewTriangleMesh (g_device);
  //rtcSetBuffer(geom, RTC_VERTEX_BUFFER, cube_vertices,     0, sizeof(uniform Vec3fa  ), NUM_VERTICES);
  Vec3fa* uniform ptr = (Vec3fa* uniform) rtcNewBuffer(geom, RTC_VERTEX_BUFFER, sizeof(uniform Vec3fa), NUM_VERTICES);
  for (uniform size_t i=0; i<NUM_VERTICES; i++) {
    uniform float x = cube_vertices[i][0];
    uniform float y = cube_vertices[i][1];
    uniform float z = cube_vertices[i][2];
    uniform Vec3f vtx = make_Vec3f(x,y,z);
    ptr[i] = make_Vec3fa(offset+make_LinearSpace3f_rotate(make_Vec3f(0,1,0),rotation)*make_LinearSpace3f_scale(scale)*vtx);
  }
  rtcSetBuffer(geom,RTC_INDEX_BUFFER,  cube_tri_indices , 0, 3*sizeof(uniform unsigned int), NUM_TRI_FACES);

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
  rtcSetIntersectionFilterFunction(geom,intersectionFilterN);
  rtcSetOcclusionFilterFunction   (geom,occlusionFilterN);

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a cube to the scene */
uniform unsigned int addSubdivCube (RTCScene scene_i)
{
  RTCGeometry geom = rtcNewSubdivisionMesh(g_device);
  rtcSetBuffer(geom, RTC_VERTEX_BUFFER, cube_vertices,      0, sizeof(uniform Vec3fa  ), NUM_VERTICES);
  rtcSetBuffer(geom, RTC_INDEX_BUFFER,  cube_quad_indices , 0, sizeof(uniform unsigned int), NUM_QUAD_INDICES);
  rtcSetBuffer(geom, RTC_FACE_BUFFER,   cube_quad_faces,    0, sizeof(uniform unsigned int), NUM_QUAD_FACES);

  uniform float* level = (uniform float*) rtcNewBuffer(geom, RTC_LEVEL_BUFFER, sizeof(uniform float), NUM_QUAD_INDICES);
  for (size_t i=0; i<NUM_QUAD_INDICES; i++) level[i] = 4;

  /* create face color array */
  colors = uniform new uniform Vec3f[6];
  colors[0] = make_Vec3f(1,0,0); // left side
  colors[1] = make_Vec3f(0,1,0); // right side
  colors[2] = make_Vec3f(0.5f);  // bottom side
  colors[3] = make_Vec3f(1.0f);  // top side
  colors[4] = make_Vec3f(0,0,1); // front side
  colors[5] = make_Vec3f(1,1,0); // back side

  /* set intersection filter for the cube */
  rtcSetIntersectionFilterFunction(geom,intersectionFilterN);
  rtcSetOcclusionFilterFunction   (geom,occlusionFilterN);

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewTriangleMesh (g_device);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcNewBuffer(geom,RTC_VERTEX_BUFFER,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcNewBuffer(geom,RTC_INDEX_BUFFER,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(NULL,rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler,NULL);

  /* create scene */
  g_scene = rtcDeviceNewScene(g_device);
  rtcSetBuildQuality(g_scene, RTC_BUILD_QUALITY_HIGH); // high quality mode to test if we filter out duplicated intersections

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
  rtcReleaseScene (g_scene); g_scene = NULL;
  rtcReleaseDevice(g_device); g_device = NULL;
  delete[] colors; colors = NULL;
}
