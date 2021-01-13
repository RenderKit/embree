// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.isph"
#include "../common/math/random_sampler.isph"

struct Point
{
  ALIGNED_STRUCT_(16)
  Vec3f p;                      //!< position
};

struct PointQueryResult
{
  ALIGNED_STRUCT_(16)
  unsigned int primID;
};

struct PointQuery
{
  Vec3f p;
  float time;
  float radius;
};

/* scene data */
RTCScene g_scene  = NULL;
uniform Point* uniform g_points = NULL;
const uniform int g_num_colors = 27;
uniform Vec3fa g_colors[g_num_colors];

extern uniform int g_num_points;
uniform int g_num_points_current;

void renderTileStandardStream(uniform int taskIndex,
                              uniform int threadIndex,
                              uniform int* uniform pixels,
                              const uniform unsigned int width,
                              const uniform unsigned int height,
                              const uniform float time,
                              const uniform ISPCCamera& camera,
                              const uniform int numTilesX,
                              const uniform int numTilesY);

// ======================================================================== //
//                     User defined point geometry                         //
// ======================================================================== //

unmasked void pointBoundsFunc(const struct RTCBoundsFunctionArguments* uniform args)
{
  const uniform Point* uniform points = (const uniform Point* uniform) args->geometryUserPtr;
  uniform RTCBounds* uniform bounds_o = args->bounds_o;
  const uniform Point& point = points[args->primID];
  bounds_o->lower_x = point.p.x;
  bounds_o->lower_y = point.p.y;
  bounds_o->lower_z = point.p.z;
  bounds_o->upper_x = point.p.x;
  bounds_o->upper_y = point.p.y;
  bounds_o->upper_z = point.p.z;
}

unmasked bool pointQueryFunc(RTCPointQueryFunctionArguments* uniform args)
{
  assert(args->userPtr);

  RTCPointQuery* query = (RTCPointQuery*)args->query;
  unsigned int primID = args->primID;
  Vec3f q = make_Vec3f(query->x, query->y, query->z);
  const Point point = g_points[primID];
  const float d = distance(point.p, q);

  if(d < query->radius)
  {
    PointQueryResult* result = (PointQueryResult*)args->userPtr;
    result->primID = primID;
    query->radius = d;
    return true;
  }
  return false;
}

uniform Point* uniform createPoints (RTCScene scene, uniform unsigned int N)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  g_points = uniform new uniform Point[N];
  uniform unsigned int geomID = rtcAttachGeometry(scene, geom);
  rtcSetGeometryUserPrimitiveCount(geom, N);
  rtcSetGeometryUserData(geom, g_points);
  rtcSetGeometryBoundsFunction(geom, pointBoundsFunc, NULL);
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);

  RandomSampler rs;
  RandomSampler_init(rs, 42);
  for (unsigned int i = 0; i < N; ++i) 
  {
    float xi1 = RandomSampler_getFloat(rs);
    float xi2 = RandomSampler_getFloat(rs);
    g_points[i].p = make_Vec3f(xi1, 0.f, xi2);
  }

  g_num_points_current = N;
  return g_points;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);
  g_points = createPoints(g_scene, g_num_points);
  rtcCommitScene(g_scene);

  /* set all colors */
  for (uniform int r = 0; r < 3; ++r) 
    for (uniform int g = 0; g < 3; ++g) 
      for (uniform int b = 0; b < 3; ++b) 
        g_colors[r * 9 + g * 3 + b] = make_Vec3fa(0.2f + 0.3f * r, 0.2f + 0.3f * g, 0.2f + 0.3f * b); 

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
    if (all(__mask == 0)) continue;

    /* calculate pixel color */
    Vec3fa color = make_Vec3fa(0.f);
    Vec3f q = make_Vec3f(((float)x + 0.5f) / width, 0.f, ((float)y + 0.5f) / height);

    PointQuery query;
    query.p = q;
    query.time = 0.f;
    query.radius = inf;

    uniform PointQueryResult result[programCount];
    result[programIndex].primID = RTC_INVALID_GEOMETRY_ID;
    void* userPtr = (void*)&result[programIndex];
    
    uniform RTCPointQueryContext context;
    rtcInitPointQueryContext(&context);
    rtcPointQueryV(g_scene,
                  (varying RTCPointQuery* uniform)&query,
                  &context,
                  pointQueryFunc,
                  &userPtr);

    unsigned int primID = result[programIndex].primID;

    if (primID != RTC_INVALID_GEOMETRY_ID)
      color = g_colors[primID % 27];

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
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

void splat_color(uniform int* pixels, 
                 const uniform unsigned int width, 
                 const uniform unsigned int height, 
                 const uniform int kernel_size,
                 float x, float y, Vec3fa const& color)
{
  for (int dy = -kernel_size; dy <= kernel_size; ++dy)
  for (int dx = -kernel_size; dx <= kernel_size; ++dx)
  {
    unsigned int r = (unsigned int)(255.0f * clamp(color.x, 0.0f, 1.0f));
    unsigned int g = (unsigned int)(255.0f * clamp(color.y, 0.0f, 1.0f));
    unsigned int b = (unsigned int)(255.0f * clamp(color.z, 0.0f, 1.0f));
    const unsigned int px = (unsigned int)min(width  - 1.f, max(0.f, x * width  + dx));
    const unsigned int py = (unsigned int)min(height - 1.f, max(0.f, y * height + dy));
    pixels[py*width + px] = (b << 16) + (g << 8) + r;
  }
}


task void drawPoints(uniform int* uniform pixels,
                const uniform unsigned int width,
                const uniform unsigned int height)
{
  Point p = g_points[taskIndex];
  Vec3fa color = g_colors[taskIndex % g_num_colors] / 0.8f;
  splat_color(pixels, width, height, 2, p.p.x, p.p.z, color);
}

export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
{
  /* render all pixels */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;

  launch[g_num_points] drawPoints(pixels, width, height); sync;
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  if (g_num_points != g_num_points_current)
  {
    rtcReleaseScene (g_scene); 
    g_scene = rtcNewScene(g_device);
    delete[] g_points; g_points = NULL;
    g_points = createPoints(g_scene, g_num_points);
    rtcCommitScene(g_scene);
  }
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = NULL;
  rtcReleaseDevice(g_device); g_device = NULL;
  delete[] g_points; g_points = NULL;
}
