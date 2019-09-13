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

#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

extern "C" ISPCScene* g_ispc_scene;
extern "C" int g_instancing_mode;
extern "C" int g_num_hits;
extern "C" bool g_verify;

#define MAX_HITS 16*1024

/* extended ray structure that gathers all hits along the ray */
struct RayExt
{
  RayExt ()
    : begin(0), end(0) {}

  unsigned int size() const {
    return end-begin;
  }

  unsigned int begin;
  unsigned int end;
  struct Hit
  {
    Hit() {}

    Hit (float t, unsigned int primID = 0xFFFFFFFF, unsigned int geomID = 0xFFFFFFFF)
      : t(t), primID(primID), geomID(geomID) {}

    /* lexicographical order (t,geomID,primID) */
    __forceinline friend bool operator < (Hit& a, Hit& b)
    {
      if (a.t == b.t) {
        if (a.geomID == b.geomID) return a.primID < b.primID;
        else                      return a.geomID < b.geomID;
      }
      return a.t < b.t;
    }

    __forceinline friend bool operator ==(Hit& a, Hit& b) {
      return a.t == b.t && a.primID == b.primID && a.geomID == b.geomID;
    }

    __forceinline friend bool operator <= (Hit& a, Hit& b)
    {
      if (a == b) return true;
      else return a < b;
    }

    __forceinline friend bool operator !=(Hit& a, Hit& b) {
      return !(a == b);
    }

    friend std::ostream& operator<<(std::ostream& cout, const Hit& hit) {
      return cout << "Hit { t = " << hit.t << ", geomID = " << hit.geomID << ", primID = " << hit.primID << " }";
    }
    
    float t;
    unsigned int primID;
    unsigned int geomID;
  };
  Hit hits[MAX_HITS];
};
  
struct IntersectContext
{
  IntersectContext(RayExt& rayext)
    : rayext(rayext) {}
  
  RTCIntersectContext context;
  RayExt& rayext;
};

/* scene data */
RTCScene g_scene = nullptr;

void device_key_pressed_handler(int key)
{
  //if (key == 110 /*n*/) g_use_smooth_normals = !g_use_smooth_normals;
  //else
  device_key_pressed_default(key);
}

RTCScene convertScene(ISPCScene* scene_in)
{
  RTCScene scene_out = ConvertScene(g_device, g_ispc_scene, RTC_BUILD_QUALITY_MEDIUM);
  rtcSetSceneFlags(scene_out, rtcGetSceneFlags(scene_out) | RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION | RTC_SCENE_FLAG_ROBUST);

  /* commit individual objects in case of instancing */
  if (g_instancing_mode != ISPC_INSTANCING_NONE)
  {
    for (unsigned int i=0; i<scene_in->numGeometries; i++) {
      ISPCGeometry* geometry = g_ispc_scene->geometries[i];
      if (geometry->type == GROUP) {
        rtcSetSceneFlags(geometry->scene, rtcGetSceneFlags(geometry->scene) | RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION);
        rtcCommitScene(geometry->scene);
      }
    }
  }

  /* commit changes to scene */
  return scene_out;
}

/* Filter callback function that gathers all hits */
void gatherAllHits(const struct RTCFilterFunctionNArguments* args)
{
  assert(*args->valid == -1);
  IntersectContext* context = (IntersectContext*) args->context;
  RayExt& rayext = context->rayext;
  RTCRay* ray = (RTCRay*) args->ray;
  RTCHit* hit = (RTCHit*) args->hit;
  assert(args->N == 1);
  args->valid[0] = 0; // ignore all hits
    
  if (rayext.end > MAX_HITS) return;

  /* add hit to list */
  rayext.hits[rayext.end++] = RayExt::Hit(ray->tfar,hit->primID,hit->geomID);
}

/* Filter callback function that gathers first N hits */
void gatherNHits(const struct RTCFilterFunctionNArguments* args)
{
  assert(*args->valid == -1);
  IntersectContext* context = (IntersectContext*) args->context;
  RayExt& rayext = context->rayext;
  RTCRay* ray = (RTCRay*) args->ray;
  RTCHit* hit = (RTCHit*) args->hit;
  assert(args->N == 1);
  args->valid[0] = 0; // ignore all hits
    
  if (rayext.end > MAX_HITS) return;

  RayExt::Hit nhit(ray->tfar,hit->primID,hit->geomID);

  if (rayext.begin > 0 && nhit <= rayext.hits[rayext.begin-1])
    return;

  for (size_t i=rayext.begin; i<rayext.end; i++)
    if (nhit < rayext.hits[i])
      std::swap(nhit,rayext.hits[i]);

  if (rayext.size() < g_num_hits)
    rayext.hits[rayext.end++] = nhit;
  else {
    ray->tfar = rayext.hits[rayext.end-1].t;
    args->valid[0] = -1; // accept hit
  }
}

void single_pass(Ray ray, RayExt& rayext_o, RayStats& stats)
{
  IntersectContext context(rayext_o);
  rtcInitIntersectContext(&context.context);
  
  context.context.filter = gatherAllHits;
  rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
  RayStats_addRay(stats);
  std::sort(&context.rayext.hits[context.rayext.begin],&context.rayext.hits[context.rayext.end]);
}

void multi_pass(Ray ray, RayExt& rayext_o, RayStats& stats)
{
  IntersectContext context(rayext_o);
  rtcInitIntersectContext(&context.context);
  
  context.context.filter = gatherNHits;
  
  int iter = 0;
  do {
    
    if (context.rayext.end)
      ray.tnear() = context.rayext.hits[context.rayext.end-1].t;
    
    ray.tfar = inf;
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    context.rayext.begin = context.rayext.end;
    
    for (size_t i=0; i<g_num_hits; i++)
      if (context.rayext.begin+i < MAX_HITS)
        context.rayext.hits[context.rayext.begin+i] = RayExt::Hit(neg_inf);
    
    rtcIntersect1(g_scene,&context.context,RTCRayHit_(ray));
    RayStats_addRay(stats);
    iter++;
    
    /*PRINT(iter);
      for (size_t i=0; i<context.rayext.end; i++)
      {
      PRINT2(i,context.rayext.hits[i]);
      }*/
    
  } while (context.rayext.size() != 0);
  
  context.rayext.begin = 0;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  RayExt rayext;
  
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf, 0.0f);

  /* either gather hits in single pass or using multiple passes */
  if (g_num_hits == 0) single_pass(ray,rayext,stats);
  else                 multi_pass(ray,rayext,stats);

  /* verify result with gathering all hits */
  if (g_verify)
  {
    RayExt verify_rayext;
    single_pass(ray,verify_rayext,stats);
    
    /*for (size_t i=verify_rayext.begin; i<verify_rayext.end; i++)
      PRINT2(i,verify_rayext.hits[i]);
    
    for (size_t i=rayext.begin; i<rayext.end; i++)
    PRINT2(i,rayext.hits[i]);*/
    
    if (verify_rayext.size() != rayext.size())
      throw std::runtime_error("different number of hits found");
    
    for (size_t i=verify_rayext.begin; i<verify_rayext.end; i++)
    {
      if (verify_rayext.hits[i] != rayext.hits[i])
        throw std::runtime_error("hits differ");
    }
  }
  
  /* calculate random sequence based on hit geomIDs and primIDs */
  RandomSampler sampler = { 0 };
  for (size_t i=rayext.begin; i<rayext.end; i++) {
    sampler.s = MurmurHash3_mix(sampler.s, rayext.hits[i].geomID);
    sampler.s = MurmurHash3_mix(sampler.s, rayext.hits[i].primID);
  }
  sampler.s = MurmurHash3_finalize(sampler.s);

  /* map geomID/primID sequence to color */
  Vec3fa color;
  color.x = RandomSampler_getFloat(sampler);
  color.y = RandomSampler_getFloat(sampler);
  color.z = RandomSampler_getFloat(sampler);
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
  const int t = taskIndex;
  const unsigned int tileY = t / numTilesX;
  const unsigned int tileX = t - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    Vec3fa color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
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

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_handler;
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  /* create scene */
  if (g_scene == nullptr) {
    g_scene = convertScene(g_ispc_scene);
    rtcCommitScene (g_scene);
  }

  /* render image */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
  //rtcDebug();
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = nullptr;
}

} // namespace embree
