// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "intersection_filter_device.h"

namespace embree {

#if EMBREE_SYCL_TUTORIAL
#define USE_ARGUMENT_CALLBACKS 1
#else
#define USE_ARGUMENT_CALLBACKS 0
#endif

/* all features required by this tutorial */
#if USE_ARGUMENT_CALLBACKS
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS
#else
#define FEATURE_MASK     \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY
#endif

RTC_SYCL_INDIRECTLY_CALLABLE void intersectionFilter(const RTCFilterFunctionNArguments* args);
RTC_SYCL_INDIRECTLY_CALLABLE void occlusionFilter(const RTCFilterFunctionNArguments* args);

RTCScene g_scene = nullptr;
TutorialData data;

// FIXME: fast path for occlusionFilter

/******************************************************************************************/
/*                             Standard Mode                                              */
/******************************************************************************************/

/* 3D procedural transparency */
inline float transparencyFunction(Vec3fa& h)
{
  float v = abs(sin(4.0f*h.x)*cos(4.0f*h.y)*sin(4.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);
  return T;
  //return 0.5f;
}


/* task that renders a single screen tile */
void renderPixelStandard(const TutorialData& data,
                         int x, int y, 
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera, RayStats& stats)
{
  float weight = 1.0f;
  Vec3fa color = Vec3fa(0.0f);

  RayQueryContext context;
  InitIntersectionContext(&context);

  /* initialize ray */
  Ray primary;
  init_Ray(primary,Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);
  float primary_transparency = 0.0f;

  while (true)
  {
    context.userRayExt = &primary_transparency;

    /* intersect ray with scene */
    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.context = &context.context;
    iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
    iargs.filter = intersectionFilter;
#endif
    rtcIntersect1(data.g_scene,RTCRayHit_(primary),&iargs);
    RayStats_addRay(stats);

    /* shade pixels */
    if (primary.geomID == RTC_INVALID_GEOMETRY_ID)
      break;

    float opacity = 1.0f-primary_transparency;
    Vec3fa diffuse = data.colors[primary.primID];
    Vec3fa La = diffuse*0.5f;
    color = color + weight*opacity*La;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow;
    init_Ray(shadow, primary.org + primary.tfar*primary.dir, neg(lightDir), 0.001f, inf);
    float shadow_transparency = 1.0f;
    context.userRayExt = &shadow_transparency;

    /* trace shadow ray */
    RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.context = &context.context;
    sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
    sargs.filter = occlusionFilter;
#endif
    rtcOccluded1(data.g_scene,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f) {
      Vec3fa Ll = diffuse*shadow_transparency*clamp(-dot(lightDir,normalize(primary.Ng)),0.0f,1.0f);
      color = color + weight*opacity*Ll;
    }

    /* shoot transmission ray */
    weight *= primary_transparency;
    primary.tnear() = 1.001f*primary.tfar;
    primary.tfar = (float)(inf);
    primary.geomID = RTC_INVALID_GEOMETRY_ID;
    primary.primID = RTC_INVALID_GEOMETRY_ID;
    primary_transparency = 0.0f;
  }

  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
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
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex]);
  }
}

/* intersection filter function for single rays and packets */
RTC_SYCL_INDIRECTLY_CALLABLE void intersectionFilter(const RTCFilterFunctionNArguments* args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == nullptr) return;

  assert(args->N == 1);
  int* valid = args->valid;
  const RayQueryContext* context = (const RayQueryContext*) args->context;
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
    float* transparency = (float*) context->userRayExt;
    *transparency = T;
  }
}

/* occlusion filter function for single rays and packets */
RTC_SYCL_INDIRECTLY_CALLABLE void occlusionFilter(const RTCFilterFunctionNArguments* args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == nullptr) return;

  assert(args->N == 1);
  int* valid = args->valid;
  const RayQueryContext* context = (const RayQueryContext*) args->context;
  Ray* ray = (Ray*)args->ray;

  /* ignore inactive rays */
  if (valid[0] != -1) return;

  float* transparency = (float*) context->userRayExt;
  assert(transparency);

  Vec3fa h = ray->org + ray->dir * ray->tfar;

  /* calculate and accumulate transparency */
  float T = transparencyFunction(h);
  T *= *transparency;
  *transparency = T;
  if (T != 0.0f) 
    valid[0] = 0;
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
  //rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, cube_tri_indices, 0, 3*sizeof(unsigned int), NUM_TRI_FACES);
  Vec3i* index = (Vec3i*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(unsigned int), NUM_TRI_FACES);
  memcpy(index, cube_tri_indices, 3*sizeof(unsigned int) * NUM_TRI_FACES);

  /* create per-triangle color array */
  data.colors = (Vec3fa*) alignedUSMMalloc((12)*sizeof(Vec3fa),16);
  data.colors[0] = Vec3fa(1,0,0); // left side
  data.colors[1] = Vec3fa(1,0,0);
  data.colors[2] = Vec3fa(0,1,0); // right side
  data.colors[3] = Vec3fa(0,1,0);
  data.colors[4] = Vec3fa(0.5f);  // bottom side
  data.colors[5] = Vec3fa(0.5f);
  data.colors[6] = Vec3fa(1.0f);  // top side
  data.colors[7] = Vec3fa(1.0f);
  data.colors[8] = Vec3fa(0,0,1); // front side
  data.colors[9] = Vec3fa(0,0,1);
  data.colors[10] = Vec3fa(1,1,0); // back side
  data.colors[11] = Vec3fa(1,1,0);

  /* set intersection filter for the cube */
#if USE_ARGUMENT_CALLBACKS
  rtcSetGeometryEnableFilterFunctionFromArguments(geom,true);
#else
  rtcSetGeometryIntersectFilterFunction(geom,data.intersectionFilter);
  rtcSetGeometryOccludedFilterFunction(geom,data.occlusionFilter);
#endif

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
  data.colors = (Vec3fa*) alignedUSMMalloc((6)*sizeof(Vec3fa),16);
  data.colors[0] = Vec3fa(1,0,0); // left side
  data.colors[1] = Vec3fa(0,1,0); // right side
  data.colors[2] = Vec3fa(0.5f);  // bottom side
  data.colors[3] = Vec3fa(1.0f);  // top side
  data.colors[4] = Vec3fa(0,0,1); // front side
  data.colors[5] = Vec3fa(1,1,0); // back side

  /* set intersection filter for the cube */
#if USE_ARGUMENT_CALLBACKS
  rtcSetGeometryEnableFilterFunctionFromArguments(geom,true);
#else
  rtcSetGeometryIntersectFilterFunction(geom,data.intersectionFilter);
  rtcSetGeometryOccludedFilterFunction(geom,data.occlusionFilter);
#endif

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
#if !USE_ARGUMENT_CALLBACKS
  data.intersectionFilter = GET_FUNCTION_POINTER(intersectionFilter);
  data.occlusionFilter    = GET_FUNCTION_POINTER(occlusionFilter   );
#endif
  
  /* create scene */
  g_scene = data.g_scene = rtcNewScene(g_device);
  rtcSetSceneFlags(data.g_scene, RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS);
  rtcSetSceneBuildQuality(data.g_scene, RTC_BUILD_QUALITY_MEDIUM);

  /* add cube */
  addCube(data.g_scene,Vec3fa(0.0f,0.0f,0.0f),Vec3fa(10.0f,1.0f,1.0f),45.0f);
  //addSubdivCube(data.g_scene);

  /* add ground plane */
  addGroundPlane(data.g_scene);

  /* commit changes to scene */
  rtcCommitScene (data.g_scene);
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
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  TutorialData ldata = data;
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
      const unsigned int x = item.get_global_id(1); if (x >= width ) return;
      const unsigned int y = item.get_global_id(0); if (y >= height) return;
      RayStats stats;
      renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats);
    });
  });
  global_gpu_queue->wait_and_throw();

  const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
  const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
  const double dt = (t1-t0)*1E-9;
  ((ISPCCamera*)&camera)->render_time = dt;
  
#else
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
#endif
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
