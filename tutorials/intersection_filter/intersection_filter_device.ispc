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
  unsigned int firstHit, lastHit;
  unsigned int hit_geomIDs[HIT_LIST_LENGTH];
  unsigned int hit_primIDs[HIT_LIST_LENGTH];
};

inline varying RTCRayHit* uniform RTCRayHit_(varying Ray2& ray)
{
  varying RTCRayHit* uniform ray_ptr = (varying RTCRayHit* uniform)&ray;
  return ray_ptr;
}

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
  primary.ray.id = programIndex; // needs to encode rayID for filter
  primary.transparency = 0.0f;


  while (true)
  {
    context.userRayExt = &primary;

    /* intersect ray with scene */
    rtcIntersectV(g_scene,&context.context,RTCRayHit_(primary));
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
    shadow.ray.id = programIndex; // needs to encode rayID for filter
    shadow.transparency = 1.0f;
    shadow.firstHit = 0;
    shadow.lastHit = 0;
    context.userRayExt = &shadow;

    /* trace shadow ray */
    rtcOccludedV(g_scene,&context.context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.ray.tfar >= 0.0f) {
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

inline float gather(varying float& ptr, const uniform unsigned int stride, const unsigned int pid, const unsigned int rid)
{
  uniform float* uptr = (uniform float*) (((uniform int8* uniform)&ptr) + pid*stride);
  return uptr[rid];
}

inline unsigned int gather(varying unsigned int& ptr, const uniform unsigned int stride, const unsigned int pid, const unsigned int rid)
{
  uniform unsigned int* uptr = (uniform unsigned int*) (((uniform int8* uniform)&ptr) + pid*stride);
  return uptr[rid];
}

inline unsigned int gather(varying unsigned int& ptr, const varying unsigned int idx, const uniform unsigned int stride, const unsigned int pid, const unsigned int rid)
{
  uniform unsigned int* uptr = (uniform unsigned int*) (((uniform int8* uniform)&ptr) + pid*stride);
  return uptr[rid + programCount*idx];
}

inline void scatter(varying float& ptr, const uniform unsigned int stride, const unsigned int pid, const unsigned int rid, float v) {
  ((uniform float*)(((uniform int8* uniform)&ptr) + pid*stride))[rid] = v;
}

inline void scatter(varying unsigned int& ptr, const uniform unsigned int stride, const unsigned int pid, const unsigned int rid, unsigned int v) {
  ((uniform unsigned int*)(((uniform int8* uniform)&ptr) + pid*stride))[rid] = v;
}

inline void scatter(varying unsigned int& ptr, const varying unsigned int idx, const uniform unsigned int stride, const unsigned int pid, const unsigned int rid, unsigned int v) {
  ((uniform unsigned int*)(((uniform int8* uniform)&ptr) + pid*stride))[rid+programCount*idx] = v;
}


/* intersection filter function for single rays and packets */
unmasked void intersectionFilter(const RTCFilterFunctionNArguments* uniform args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == NULL) return;

  assert(args->N == programCount);
  uniform int* uniform valid = args->valid;
  const uniform IntersectContext* uniform context = (const uniform IntersectContext* uniform) args->context;
  varying Ray* uniform ray = (varying Ray* uniform)args->ray;
  //varying RTCHit* uniform hit = (varying RTCHit* uniform)args->hit;

  /* ignore inactive rays */
  if (valid[programIndex] != -1) return;

  /* calculate transparency */
  Vec3f h = ray->org + ray->dir  * ray->tfar;
  float T = transparencyFunction(h);

  /* ignore hit if completely transparent */
  if (T >= 1.0f) 
    valid[programIndex] = 0;
  /* otherwise accept hit and remember transparency */
  else
  {
    varying Ray2* uniform eray = (varying Ray2* uniform) context->userRayExt;
    eray->transparency = T;
  }
}

/* intersection filter function for streams of general packets */
unmasked void intersectionFilterN(const RTCFilterFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  const uniform IntersectContext* uniform context = (const uniform IntersectContext* uniform) args->context;
  struct RTCRayHitN* uniform rayN = (struct RTCRayHitN* uniform)args->ray;
  //struct RTCHitN* uniform hitN = args->hit;
  const uniform unsigned int N = args->N;
                                  
  /* avoid crashing when debug visualizations are used */
  if (context == NULL) return;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* read ray/hit from ray structure */
    RTCRayHit rtc_ray = rtcGetRayHitFromRayHitN(rayN,N,ui);
    varying Ray* uniform ray = (varying Ray* uniform)&rtc_ray;

    /* calculate transparency */
    Vec3f h = ray->org + ray->dir  * ray->tfar;
    float T = transparencyFunction(h);

    /* ignore hit if completely transparent */
    if (T >= 1.0f) 
      valid[vi] = 0;
    /* otherwise accept hit and remember transparency */
    else
    {
      /* decode ray IDs */
      const unsigned int pid = ray->id / programCount;
      const unsigned int rid = ray->id % programCount;
      varying Ray2* uniform ray2 = (varying Ray2* uniform) context->userRayExt;
      assert(ray2);
      scatter(ray2->transparency,sizeof(Ray2),pid,rid,T);
    }
  }
}

/* occlusion filter function for single rays and packets */
unmasked void occlusionFilter(const RTCFilterFunctionNArguments* uniform args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == NULL) return;

  assert(args->N == programCount);
  uniform int* uniform valid = args->valid;
  const uniform IntersectContext* uniform context = (const uniform IntersectContext* uniform) args->context;
  varying Ray* uniform ray = (varying Ray* uniform)args->ray;
  varying RTCHit* uniform hit = (varying RTCHit* uniform)args->hit;

  /* ignore inactive rays */
  if (valid[programIndex] != -1) return;

  varying Ray2* uniform ray2 = (varying Ray2* uniform) context->userRayExt;
  assert(ray2);

  for (unsigned int i=ray2->firstHit; i<ray2->lastHit; i++) {
    unsigned slot= i%HIT_LIST_LENGTH;
    if (ray2->hit_geomIDs[slot] == hit->geomID && ray2->hit_primIDs[slot] == hit->primID) {
      valid[programIndex] = 0; return; // ignore duplicate intersections
    }
  }
  /* store hit in hit list */
  unsigned int slot = ray2->lastHit%HIT_LIST_LENGTH;
  ray2->hit_geomIDs[slot] = hit->geomID;
  ray2->hit_primIDs[slot] = hit->primID;
  ray2->lastHit++;
  if (ray2->lastHit - ray2->firstHit >= HIT_LIST_LENGTH)
    ray2->firstHit++;

  Vec3f h = ray->org + ray->dir * ray->tfar;

  /* calculate and accumulate transparency */
  float T = transparencyFunction(h);
  T *= ray2->transparency;
  ray2->transparency = T;
  if (T != 0.0f) 
    valid[programIndex] = 0;
}

/* intersection filter function for streams of general packets */
unmasked void occlusionFilterN(const RTCFilterFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  const uniform IntersectContext* uniform context = (const uniform IntersectContext* uniform) args->context;
  struct RTCRayHitN* uniform rayN = (struct RTCRayHitN* uniform)args->ray;
  struct RTCHitN* uniform hitN = args->hit;
  const uniform unsigned int N = args->N;
                                  
  /* avoid crashing when debug visualizations are used */
  if (context == NULL) return;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* read ray/hit from ray structure */
    RTCRayHit rtc_ray = rtcGetRayHitFromRayHitN(rayN,N,ui);
    varying Ray* uniform ray = (varying Ray* uniform)&rtc_ray;

    RTCHit hit = rtcGetHitFromHitN(hitN,N,ui);
    const unsigned int hit_geomID = hit.geomID;
    const unsigned int hit_primID = hit.primID;

    /* decode ray IDs */
    const unsigned int pid = ray->id / programCount;
    const unsigned int rid = ray->id % programCount;
    varying Ray2* uniform ray2 = (varying Ray2* uniform) context->userRayExt;
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
    Vec3f h = ray->org + ray->dir * ray->tfar;
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
 
    primary.ray.id = N*programCount + programIndex;
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
    rtcIntersectVM(g_scene,&primary_context.context,(varying RTCRayHit* uniform)&primary_stream,N,sizeof(Ray2));

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
      shadow.ray.id = N*programCount + programIndex;
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
      if (shadow.ray.tfar >= 0.0f) {
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
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  //rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, cube_vertices,     0, sizeof(uniform Vec3fa  ), NUM_VERTICES);
  Vec3fa* uniform ptr = (Vec3fa* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vec3fa), NUM_VERTICES);
  for (uniform unsigned int i=0; i<NUM_VERTICES; i++) {
    uniform float x = cube_vertices[i][0];
    uniform float y = cube_vertices[i][1];
    uniform float z = cube_vertices[i][2];
    uniform Vec3f vtx = make_Vec3f(x,y,z);
    ptr[i] = make_Vec3fa(offset+make_LinearSpace3f_rotate(make_Vec3f(0,1,0),rotation)*make_LinearSpace3f_scale(scale)*vtx);
  }
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, cube_tri_indices, 0, 3*sizeof(uniform unsigned int), NUM_TRI_FACES);

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
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a cube to the scene */
uniform unsigned int addSubdivCube (RTCScene scene_i)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, cube_vertices,      0, sizeof(uniform Vec3fa),       NUM_VERTICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT,   cube_quad_indices,  0, sizeof(uniform unsigned int), NUM_QUAD_INDICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,   0, RTC_FORMAT_UINT,   cube_quad_faces,    0, sizeof(uniform unsigned int), NUM_QUAD_FACES);

  uniform float* level = (uniform float*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, sizeof(uniform float), NUM_QUAD_INDICES);
  for (unsigned int i=0; i<NUM_QUAD_INDICES; i++) level[i] = 4;

  /* create face color array */
  colors = uniform new uniform Vec3f[6];
  colors[0] = make_Vec3f(1,0,0); // left side
  colors[1] = make_Vec3f(0,1,0); // right side
  colors[2] = make_Vec3f(0.5f);  // bottom side
  colors[3] = make_Vec3f(1.0f);  // top side
  colors[4] = make_Vec3f(0,0,1); // front side
  colors[5] = make_Vec3f(1,1,0); // back side

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
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2);
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
  /* create scene */
  g_scene = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(g_scene, RTC_BUILD_QUALITY_HIGH); // high quality mode to test if we filter out duplicated intersections

  /* add cube */
  addCube(g_scene,make_Vec3f(0.0f,0.0f,0.0f),make_Vec3f(10.0f,1.0f,1.0f),45.0f);
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
  delete[] colors; colors = NULL;
}
