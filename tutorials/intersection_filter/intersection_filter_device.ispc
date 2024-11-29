// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "intersection_filter_device.isph"

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

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void intersectionFilter(const RTCFilterFunctionNArguments* uniform args);
RTC_SYCL_INDIRECTLY_CALLABLE unmasked void occlusionFilter(const RTCFilterFunctionNArguments* uniform args);

RTCScene g_scene = NULL;
uniform TutorialData data;

// FIXME: fast path for occlusionFilter

/******************************************************************************************/
/*                             Standard Mode                                              */
/******************************************************************************************/

/* 3D procedural transparency */
inline float transparencyFunction(Vec3f& h)
{
  float v = abs(sin(4.0f*h.x)*cos(4.0f*h.y)*sin(4.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);
  return T;
  //return 0.5f;
}


/* task that renders a single screen tile */
void renderPixelStandard(const uniform TutorialData& data,
                         int x, int y, 
                         uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const float time,
                         const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  float weight = 1.0f;
  Vec3f color = make_Vec3f(0.0f);

  uniform RayQueryContext context;
  InitIntersectionContext(&context);

  /* initialize ray */
  Ray primary;
  init_Ray(primary,make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);
  float primary_transparency = 0.0f;

  while (true)
  {
    context.userRayExt = &primary_transparency;

    /* intersect ray with scene */
    uniform RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.context = &context.context;
    iargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
    iargs.filter = intersectionFilter;
#endif
    rtcTraversableIntersectV(data.g_traversable,RTCRayHit_(primary),&iargs);
    RayStats_addRay(stats);

    /* shade pixels */
    if (primary.geomID == RTC_INVALID_GEOMETRY_ID)
      break;

    float opacity = 1.0f-primary_transparency;
    Vec3f diffuse = data.colors[primary.primID];
    Vec3f La = diffuse*0.5f;
    color = color + weight*opacity*La;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow;
    init_Ray(shadow, primary.org + primary.tfar*primary.dir, neg(lightDir), 0.001f, inf);
    float shadow_transparency = 1.0f;
    context.userRayExt = &shadow_transparency;

    /* trace shadow ray */
    uniform RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.context = &context.context;
    sargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
    sargs.filter = occlusionFilter;
#endif
    rtcTraversableOccludedV(data.g_traversable,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f) {
      Vec3f Ll = diffuse*shadow_transparency*clamp(-dot(lightDir,normalize(primary.Ng)),0.0f,1.0f);
      color = color + weight*opacity*Ll;
    }

    /* shoot transmission ray */
    weight *= primary_transparency;
    primary.tnear = 1.001f*primary.tfar;
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
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex]);
  }
}

/* intersection filter function for single rays and packets */
RTC_SYCL_INDIRECTLY_CALLABLE unmasked void intersectionFilter(const RTCFilterFunctionNArguments* uniform args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == NULL) return;

  assert(args->N == programCount);
  uniform int* uniform valid = args->valid;
  const uniform RayQueryContext* uniform context = (const uniform RayQueryContext* uniform) args->context;
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
    varying float* uniform transparency = (varying float* uniform) context->userRayExt;
    *transparency = T;
  }
}

/* occlusion filter function for single rays and packets */
RTC_SYCL_INDIRECTLY_CALLABLE unmasked void occlusionFilter(const RTCFilterFunctionNArguments* uniform args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == NULL) return;

  assert(args->N == programCount);
  uniform int* uniform valid = args->valid;
  const uniform RayQueryContext* uniform context = (const uniform RayQueryContext* uniform) args->context;
  varying Ray* uniform ray = (varying Ray* uniform)args->ray;

  /* ignore inactive rays */
  if (valid[programIndex] != -1) return;

  varying float* uniform transparency = (varying float* uniform) context->userRayExt;
  assert(transparency);

  Vec3f h = ray->org + ray->dir * ray->tfar;

  /* calculate and accumulate transparency */
  float T = transparencyFunction(h);
  T *= *transparency;
  *transparency = T;
  if (T != 0.0f) 
    valid[programIndex] = 0;
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
  //rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, cube_tri_indices, 0, 3*sizeof(uniform unsigned int), NUM_TRI_FACES);
  uniform Vec3i* uniform index = (uniform Vec3i* uniform)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(uniform unsigned int), NUM_TRI_FACES);
  memcpy(index, cube_tri_indices, 3*sizeof(uniform unsigned int) * NUM_TRI_FACES);

  /* create per-triangle color array */
  data.colors = uniform new uniform Vec3f[12];
  data.colors[0] = make_Vec3f(1,0,0); // left side
  data.colors[1] = make_Vec3f(1,0,0);
  data.colors[2] = make_Vec3f(0,1,0); // right side
  data.colors[3] = make_Vec3f(0,1,0);
  data.colors[4] = make_Vec3f(0.5f);  // bottom side
  data.colors[5] = make_Vec3f(0.5f);
  data.colors[6] = make_Vec3f(1.0f);  // top side
  data.colors[7] = make_Vec3f(1.0f);
  data.colors[8] = make_Vec3f(0,0,1); // front side
  data.colors[9] = make_Vec3f(0,0,1);
  data.colors[10] = make_Vec3f(1,1,0); // back side
  data.colors[11] = make_Vec3f(1,1,0);

  /* set intersection filter for the cube */
#if USE_ARGUMENT_CALLBACKS
  rtcSetGeometryEnableFilterFunctionFromArguments(geom,true);
#else
  rtcSetGeometryIntersectFilterFunction(geom,data.intersectionFilter);
  rtcSetGeometryOccludedFilterFunction(geom,data.occlusionFilter);
#endif

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
  data.colors = uniform new uniform Vec3f[6];
  data.colors[0] = make_Vec3f(1,0,0); // left side
  data.colors[1] = make_Vec3f(0,1,0); // right side
  data.colors[2] = make_Vec3f(0.5f);  // bottom side
  data.colors[3] = make_Vec3f(1.0f);  // top side
  data.colors[4] = make_Vec3f(0,0,1); // front side
  data.colors[5] = make_Vec3f(1,1,0); // back side

  /* set intersection filter for the cube */
#if USE_ARGUMENT_CALLBACKS
  rtcSetGeometryEnableFilterFunctionFromArguments(geom,true);
#else
  rtcSetGeometryIntersectFilterFunction(geom,data.intersectionFilter);
  rtcSetGeometryOccludedFilterFunction(geom,data.occlusionFilter);
#endif

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
#if !USE_ARGUMENT_CALLBACKS
  data.intersectionFilter = GET_FUNCTION_POINTER(intersectionFilter);
  data.occlusionFilter    = GET_FUNCTION_POINTER(occlusionFilter   );
#endif
  
  /* create scene */
  g_scene = data.g_scene = rtcNewScene(g_device);
  rtcSetSceneFlags(data.g_scene, RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS);
  rtcSetSceneBuildQuality(data.g_scene, RTC_BUILD_QUALITY_MEDIUM);

  /* add cube */
  addCube(data.g_scene,make_Vec3f(0.0f,0.0f,0.0f),make_Vec3f(10.0f,1.0f,1.0f),45.0f);
  //addSubdivCube(data.g_scene);

  /* add ground plane */
  addGroundPlane(data.g_scene);

  /* commit changes to scene */
  rtcCommitScene (data.g_scene);
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
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
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
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
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
#endif
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  TutorialData_Destructor(&data);
}
