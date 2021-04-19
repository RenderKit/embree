// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

#include <functional>
#include <queue>

namespace embree {

struct Point;

/* scene data */
RTCScene g_scene = nullptr;
Point* g_points = nullptr;
Point* g_points_tmp = nullptr;
Vec3fa* g_colors = nullptr;
extern "C" Vec3fa g_query_point;
const int g_num_colors = 27;

typedef void (*DrawGUI)(void);

int g_num_points_current;
extern "C" int g_num_points;
extern "C" int g_num_knn;
extern "C" bool g_show_voronoi;
extern "C" bool g_point_repulsion;
extern "C" float g_tmax;

struct Neighbour
{
  unsigned int primID;
  float d;

  bool operator<(Neighbour const& n1) const { return d < n1.d; }
};

struct KNNResult
{
  KNNResult() 
  {
    visited.reserve(2 * g_num_knn);
  }

  unsigned int k;
  std::priority_queue<Neighbour, std::vector<Neighbour>> knn;
  std::vector<unsigned int> visited; // primIDs of all visited points
};

// ======================================================================== //
//                     User defined point geometry                         //
// ======================================================================== //

struct Point
{
  ALIGNED_STRUCT_(16)
  Vec3fa p;                      //!< position
  RTCGeometry geometry;
  unsigned int geomID;
};

void pointBoundsFunc(const struct RTCBoundsFunctionArguments* args)
{
  const Point* points = (const Point*) args->geometryUserPtr;
  RTCBounds* bounds_o = args->bounds_o;
  const Point& point = points[args->primID];
  bounds_o->lower_x = point.p.x;
  bounds_o->lower_y = point.p.y;
  bounds_o->lower_z = point.p.z;
  bounds_o->upper_x = point.p.x;
  bounds_o->upper_y = point.p.y;
  bounds_o->upper_z = point.p.z;
}

bool pointQueryFunc(struct RTCPointQueryFunctionArguments* args)
{
  RTCPointQuery* query = (RTCPointQuery*)args->query;
  const unsigned int primID = args->primID;
  const Vec3f q(query->x, query->y, query->z);
  const Point& point = g_points[primID];
  const float d = distance(point.p, q);
    
  assert(args->query);
  KNNResult* result = (KNNResult*)args->userPtr;
  result->visited.push_back(primID);

  if (d < query->radius && (result->knn.size() < result->k || d < result->knn.top().d))
  {
    Neighbour neighbour;
    neighbour.primID = primID;
    neighbour.d = d;
  
    if (result->knn.size() == result->k)
      result->knn.pop();
      
    result->knn.push(neighbour);
  
    if (result->knn.size() == result->k)
    {
      const float R = result->knn.top().d;
      query->radius = R;
      return true;
    }
  }
  return false;
}

void knnQuery(Vec3f const& q, float radius, KNNResult* result)
{
  RTCPointQuery query;
  query.x = q.x;
  query.y = q.y;
  query.z = q.z;
  query.radius = radius;
  query.time = 0.f;
  RTCPointQueryContext context;
  rtcInitPointQueryContext(&context);
  rtcPointQuery(g_scene, &query, &context, pointQueryFunc, (void*)result);
}

Point* createPoints (RTCScene scene, unsigned int N)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  g_points = (Point*) alignedMalloc(N*sizeof(Point), 16);
  g_points_tmp = (Point*) alignedMalloc(N*sizeof(Point), 16);
  unsigned int geomID = rtcAttachGeometry(scene, geom);
  for (unsigned int i=0; i<N; i++) {
    g_points[i].geometry = geom;
    g_points[i].geomID = geomID;
    g_points_tmp[i].geometry = geom;
    g_points_tmp[i].geomID = geomID;
  }
  rtcSetGeometryUserPrimitiveCount(geom, N);
  rtcSetGeometryUserData(geom, g_points);
  rtcSetGeometryBoundsFunction(geom, pointBoundsFunc, nullptr);
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);

  RandomSampler rs;
  RandomSampler_init(rs, 42);
  for (unsigned int i = 0; i < N; ++i) 
  {
    float xi1 = RandomSampler_getFloat(rs);
    float xi2 = RandomSampler_getFloat(rs);
    g_points[i].p = Vec3f(xi1, 0.f, xi2);
  }

  g_num_points_current = N;
  return g_points;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);
  g_points = createPoints(g_scene, g_num_points);
  rtcCommitScene(g_scene);

  g_colors = (Vec3fa*) alignedMalloc(g_num_colors*sizeof(Point), 16);
  for (int r = 0; r < 3; ++r) for (int g = 0; g < 3; ++g) for (int b = 0; b < 3; ++b) 
    g_colors[r * 9 + g * 3 + b] = Vec3fa(0.2f + 0.3f * r, 0.2f + 0.3f * g, 0.2f + 0.3f * b); 
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
    Vec3fa color = Vec3fa(0.f);
    if (g_show_voronoi)
    {
      Vec3fa q = Vec3fa((float(x) + 0.5f) / width, 0.f, (float(y) + 0.5f) / height);

      KNNResult result;
      result.k = 1;
      knnQuery(q, g_tmax, &result);
      unsigned int primID = result.knn.empty() ? RTC_INVALID_GEOMETRY_ID : result.knn.top().primID;

      if (primID != RTC_INVALID_GEOMETRY_ID)
        color = g_colors[primID % 27];
    }

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
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

void splat_color(int* pixels, 
                 unsigned int width, 
                 unsigned int height, 
                 int kernel_size,
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

void rebuild_bvh()
{
  rtcReleaseScene (g_scene); 
  g_scene = rtcNewScene(g_device);
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  rtcAttachGeometry(g_scene, geom);
  rtcSetGeometryUserPrimitiveCount(geom, g_num_points);
  rtcSetGeometryUserData(geom, g_points);
  rtcSetGeometryBoundsFunction(geom, pointBoundsFunc, nullptr);
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  rtcCommitScene(g_scene);
}

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  if (g_num_points != g_num_points_current)
  {
    rtcReleaseScene (g_scene); 
    g_scene = rtcNewScene(g_device);
    alignedFree(g_points);
    alignedFree(g_points_tmp);
    g_points = createPoints(g_scene, g_num_points);
    rtcCommitScene(g_scene);
  }

  if (g_point_repulsion)
  {
    parallel_for(size_t(0), size_t(g_num_points), [&](const range<size_t>& range) {
      for (size_t i = range.begin(); i < range.end(); i++)
      {
        // perform nearest neighbour search for point 
        KNNResult result;
        result.k = g_num_knn + 1;
        knnQuery(g_points[i].p, g_tmax, &result);
        if (result.knn.empty())
          continue;

        const float D = result.knn.top().d;
        result.knn.pop();

        // store number of nearest neighbours for normalization later
        const size_t K = result.knn.size();
        assert(K >= 1);

        // tusk point repulsion formula
        Vec3fa dx(0.f);
        while (!result.knn.empty())
        {
          Point const& q = g_points[result.knn.top().primID];
          dx += (g_points[i].p - q.p) * (D / (result.knn.top().d + 1e-4f) - 1.f);
          result.knn.pop();
        }
        g_points_tmp[i].p = min(Vec3fa(1.f), max(Vec3fa(0.f), g_points[i].p + (1.f / K) * dx));
      }
    });

    // copy new point locations and rebuild bvh
    for (int i = 0; i < g_num_points; ++i)
      g_points[i] = g_points_tmp[i];
    rebuild_bvh();
  }

  // clear image and draw voronoi regions if enabled
  const int numTilesX = (width + TILE_SIZE_X - 1) / TILE_SIZE_X;
  const int numTilesY = (height + TILE_SIZE_Y - 1) / TILE_SIZE_Y;
  parallel_for(size_t(0), size_t(numTilesX*numTilesY), [&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i = range.begin(); i < range.end(); i++)
      renderTileTask((int)i, threadIndex, pixels, width, height, time, camera, numTilesX, numTilesY);
  });

  // draw points
  parallel_for(size_t(0), size_t(g_num_points), [&](const range<size_t>& range) {
    for (size_t i = range.begin(); i < range.end(); i++)
    {
      Point const& p = g_points[i];
      Vec3fa color = Vec3fa(0.9f);
      if (g_show_voronoi) color = g_colors[i%g_num_colors] / 0.8f;
      splat_color(pixels, width, height, 2, p.p.x, p.p.z, color);
    }
  });

  if (!g_show_voronoi)
  {
    // perform nearest neighbour query for query point
    KNNResult result;
    result.k = g_num_knn;
    knnQuery(g_query_point, g_tmax, &result);

    if (!result.knn.empty())
    {
      // draw search radius
      parallel_for(size_t(0), size_t(numTilesX*numTilesY), [&](const range<size_t>& range) {
        for (size_t i = range.begin(); i < range.end(); i++)
        {
          const unsigned int tileY = (unsigned int)i / numTilesX;
          const unsigned int tileX = (unsigned int)i - tileY * numTilesX;
          const unsigned int x0 = tileX * TILE_SIZE_X;
          const unsigned int x1 = min(x0 + TILE_SIZE_X, width);
          const unsigned int y0 = tileY * TILE_SIZE_Y;
          const unsigned int y1 = min(y0 + TILE_SIZE_Y, height);

          for (unsigned int y = y0; y < y1; y++)
            for (unsigned int x = x0; x < x1; x++)
            {
              Vec3fa q = Vec3fa((float(x) + 0.5f) / width, 0.f, (float(y) + 0.5f) / height);
              if ( pixels[y*width + x] > 0) 
                continue;
              Vec3fa color(0.0f, 0.0f, 0.0f);
              if (distance(q, g_query_point) < result.knn.top().d)
                color = Vec3fa(0.2f, 0.2f, 0.8f);
              else if (distance(q, g_query_point) < g_tmax)
                  color = Vec3fa(0.2f, 0.2f, 0.2f);
              else {
                continue;
              }
              unsigned int r = (unsigned int)(255.0f * clamp(color.x, 0.0f, 1.0f));
              unsigned int g = (unsigned int)(255.0f * clamp(color.y, 0.0f, 1.0f));
              unsigned int b = (unsigned int)(255.0f * clamp(color.z, 0.0f, 1.0f));
              pixels[y*width + x] = (b << 16) + (g << 8) + r;
            }
        }
      });
    }
    
    // draw all visited points
    for (unsigned int primID : result.visited)
    {
      Point const& p = g_points[primID];
      splat_color(pixels, width, height, 2, p.p.x, p.p.z, Vec3fa(0.8f, 0.2f, 0.2f));
    }

    // draw nearest neighbours
    while (!result.knn.empty())
    {
      Point const& p = g_points[result.knn.top().primID];
      splat_color(pixels, width, height, 2, p.p.x, p.p.z, Vec3fa(0.2f, 0.8f, 0.2f));
      result.knn.pop();
    }

    // draw query point
    splat_color(pixels, width, height, 2, g_query_point.x, g_query_point.z, Vec3fa(0.8f, 0.8f, 0.2f));
  }
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = nullptr;
  rtcReleaseDevice(g_device); g_device = nullptr;
  alignedFree(g_points); g_points = nullptr;
  alignedFree(g_points_tmp); g_points_tmp = nullptr;
  alignedFree(g_colors); g_colors = nullptr;
}

} // namespace embree
