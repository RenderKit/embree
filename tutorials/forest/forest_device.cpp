// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/math/random_sampler.h"
#include "forest_device.h"

#include "trees.h"

namespace embree {

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_INSTANCE | \
  RTC_FEATURE_FLAG_INSTANCE_ARRAY

RTCScene g_scene = nullptr;
TutorialData data;

unsigned int num_trees_sqrt;
unsigned int num_trees;
RTCScene scene_terrain;
RTCScene scene_trees[6];
RTCScene scene_trees_selected[6];
float time_last_frame = 0;
float time_total = 0;

extern "C" bool g_use_instance_array;
extern "C" bool g_rebuild;
extern "C" int g_complexity;
extern "C" int g_build_quality;
extern "C" int g_spp;
extern "C" bool g_trees_changed;
extern "C" bool g_animate;
extern "C" size_t g_memory_consumed;
extern "C" size_t g_cycles_cleanup;
extern "C" size_t g_cycles_objects;
extern "C" size_t g_cycles_embree_objects;
extern "C" size_t g_cycles_embree_bvh_build;
extern "C" size_t g_cycles_total;
extern "C" int g_trees[6];

extern "C" int64_t get_clock();

RTCGeometry instance_array;
RTCGeometry* instances = nullptr;

unsigned int addTree(RTCScene scene_i, unsigned int tree_idx)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  const float* vertices = tree_vertices[tree_idx];
  const float* colors = tree_colors[tree_idx];
  const unsigned int* indices = tree_indices[tree_idx];
  unsigned int num_vertices = tree_num_vertices[tree_idx];
  unsigned int num_colors = tree_num_colors[tree_idx];
  unsigned int num_triangles = tree_num_triangles[tree_idx];

  /* set vertices and vertex colors */
  RTCBuffer vertex_buffer = rtcNewSharedBufferHostDevice(g_device, (void *)vertices, 3*num_vertices*sizeof(float));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, vertex_buffer, 0, 3*sizeof(float), num_vertices);
  rtcCommitBuffer(vertex_buffer);
  rtcReleaseBuffer(vertex_buffer);

  RTCBuffer index_buffer = rtcNewSharedBufferHostDevice(g_device, (void *)indices, 3*num_triangles*sizeof(unsigned int));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, index_buffer, 0, 3*sizeof(unsigned int), num_triangles);
  rtcCommitBuffer(index_buffer);
  rtcReleaseBuffer(index_buffer);
  data.tree_triangles[tree_idx] = (Triangle*)rtcGetBufferDataDevice(index_buffer);

  /* create vertex color array */
  RTCBuffer color_buffer = rtcNewSharedBufferHostDevice(g_device, (void*)colors, 3*num_colors*sizeof(float));
  rtcSetGeometryVertexAttributeCount(mesh,1);
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3, color_buffer, 0, 3*sizeof(float), num_colors);
  rtcCommitBuffer(color_buffer);
  rtcReleaseBuffer(color_buffer);
  data.tree_vertex_colors[tree_idx] = (Vec3f*)rtcGetBufferDataDevice(color_buffer);

  g_memory_consumed += num_vertices * 3 * sizeof(float);
  g_memory_consumed += num_triangles * 3 * sizeof(unsigned int);
  g_memory_consumed += num_colors * 3 * sizeof(float);

  rtcCommitGeometry(mesh);
  unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addTerrain(RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry mesh = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  RTCBuffer vertex_buffer = rtcNewSharedBufferHostDevice(g_device, (void *)terrain_vertices, 3*terrain_num_vertices*sizeof(float));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, vertex_buffer, 0, 3*sizeof(float), terrain_num_vertices);
  rtcCommitBuffer(vertex_buffer);
  rtcReleaseBuffer(vertex_buffer);

  /* set triangles */
  RTCBuffer index_buffer = rtcNewSharedBufferHostDevice(g_device, (void *)terrain_indices, 3*terrain_num_triangles*sizeof(unsigned int));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, index_buffer, 0, 3*sizeof(unsigned int), terrain_num_triangles);
  rtcCommitBuffer(index_buffer);
  rtcReleaseBuffer(index_buffer);
  data.terrain_triangles = (Triangle*)rtcGetBufferDataDevice(index_buffer);

  g_memory_consumed += terrain_num_vertices * 3 * sizeof(float);
  g_memory_consumed += terrain_num_triangles * 3 * sizeof(unsigned int);

  rtcCommitGeometry(mesh);
  unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

bool monitorMemoryFunction(void* ptr, ssize_t bytes, bool post)
{
  g_memory_consumed += bytes;
  return true;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  rtcSetDeviceMemoryMonitorFunction(g_device, monitorMemoryFunction, nullptr);

  TutorialData_Constructor(&data);

  for (unsigned int i = 0; i < 6; ++i) {
    scene_trees[i] = rtcNewScene(g_device);
    addTree(scene_trees[i], i);
    rtcCommitScene(scene_trees[i]);
  }

  /* add ground plane */
  scene_terrain = rtcNewScene(g_device);
  addTerrain(scene_terrain);
  rtcCommitScene(scene_terrain);
}

void update_trees(float time)
{
  RTCBounds bounds;
  rtcGetSceneBounds(scene_terrain, &bounds);

  RTCTraversable traversable = rtcGetSceneTraversable(scene_terrain);
  unsigned int lnum_trees_sqrt = num_trees_sqrt;

#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  unsigned int* tree_ids = data.tree_ids_device;
  AffineSpace3fa* tree_transforms = data.tree_transforms_device;
#else
  unsigned int* tree_ids = data.tree_ids_host;
  AffineSpace3fa* tree_transforms = data.tree_transforms_host;
#endif

#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  unsigned int lnum_trees = num_trees;
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh) {
  const sycl::range range(num_trees);
  cgh.parallel_for(range,[=](auto t) {
    if (t >= lnum_trees) return;
#else
  parallel_for(size_t(0),size_t(num_trees),[&](const range<size_t>& range) {
  for (size_t t=range.begin(); t<range.end(); t++) {
#endif
    RandomSampler rng;
    RandomSampler_init(rng, t);
    tree_ids[t] = min(5, (int)(6 * RandomSampler_getFloat(rng)));

    unsigned int j = t / lnum_trees_sqrt;
    unsigned int i = t % lnum_trees_sqrt;

    float px = bounds.lower_x + ((float)i + RandomSampler_getFloat(rng))/((float)lnum_trees_sqrt) * (bounds.upper_x - bounds.lower_x);
    float pz = bounds.lower_z + ((float)j + RandomSampler_getFloat(rng))/((float)lnum_trees_sqrt) * (bounds.upper_z - bounds.lower_z);
    float py = bounds.upper_y;

    float dx = bounds.upper_x - bounds.lower_x;
    float dz = bounds.upper_z - bounds.lower_z;

    float phi = 2*float(M_PI)*RandomSampler_getFloat(rng);
    float mx = sin(phi);
    float mz = cos(phi);

    px = px + time * mx;
    if (px < bounds.lower_x) {
      float f = ceil((bounds.lower_x - px) / dx);
      px += f * dx;
    }
    if (px > bounds.upper_x) {
      float f = ceil((bounds.upper_x - px) / dx);
      px += f * dx;
    }
    pz = pz + time * mz;
    if (pz < bounds.lower_z) {
      float f = ceil((bounds.lower_z - pz) / dz);
      pz += f * dz;
    }
    if (pz > bounds.upper_z) {
      float f = ceil((bounds.upper_z - pz) / dz);
      pz += f * dz;
    }

    Ray ray(Vec3fa(px, py, pz), Vec3fa(0.f, -1.f, 0.f), 0.0f, inf);
    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
    rtcTraversableIntersect1(traversable,RTCRayHit_(ray),&iargs);

    Vec3fa treePos;
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID) {
      py = py - ray.tfar;
      treePos = Vec3fa(px, py, pz);
    } else {
      treePos = Vec3fa(inf, inf, inf);
    }

    tree_transforms[t] = AffineSpace3fa::translate(treePos);
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
    });
  });
  global_gpu_queue->memcpy(data.tree_ids_host, data.tree_ids_device, sizeof(uint32_t)*num_trees);
  global_gpu_queue->memcpy(data.tree_transforms_host, data.tree_transforms_device, sizeof(AffineSpace3fa)*num_trees);
  global_gpu_queue->wait_and_throw();
#else
  }});
#endif
}

void rebuild_trees(size_t old_num_trees, float time)
{
  if (data.tree_ids_host) {
    TutorialData_FreeTreeData((void*)data.tree_ids_host, (void*)data.tree_ids_device);
    g_memory_consumed -= old_num_trees * sizeof(uint32_t);
  }
  data.tree_ids_host = (uint32_t*) alignedMalloc((num_trees)*sizeof(uint32_t),16);
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  data.tree_ids_device = sycl::aligned_alloc_device<uint32_t>(16, num_trees, *global_gpu_device, *global_gpu_context);
#else
  data.tree_ids_device = data.tree_ids_host;
#endif
  g_memory_consumed += num_trees * sizeof(uint32_t);

  if (data.tree_transforms_host) {
    TutorialData_FreeTreeData((void*)data.tree_transforms_host, (void*)data.tree_transforms_device);
    g_memory_consumed -= old_num_trees * sizeof(AffineSpace3fa);
  }
  data.tree_transforms_host = (AffineSpace3fa*) alignedMalloc((num_trees)*sizeof(AffineSpace3fa),16);
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  data.tree_transforms_device = sycl::aligned_alloc_device<AffineSpace3fa>(16, num_trees, *global_gpu_device, *global_gpu_context);
#else
  data.tree_transforms_device = data.tree_transforms_host;
#endif
  g_memory_consumed += num_trees * sizeof(AffineSpace3fa);

  update_trees(time);
}

void update_instance_scenes()
{
  if (g_use_instance_array)
  {
    rtcSetGeometryInstancedScenes(instance_array,(RTCScene*)scene_trees_selected,6);
    rtcCommitGeometry(instance_array);
  }
  else
  {
    for (unsigned int i = 0; i < num_trees; ++i) {
      rtcSetGeometryInstancedScene(instances[i],scene_trees_selected[data.tree_ids_host[i]]);
      rtcCommitGeometry(instances[i]);
    }
  }
}

void update_instance_transforms()
{
  if (g_use_instance_array)
  {
    rtcUpdateGeometryBuffer(instance_array, RTC_BUFFER_TYPE_TRANSFORM, 0);
    rtcCommitGeometry(instance_array);
  }
  else
  {
    for (unsigned int i = 0; i < num_trees; ++i) {
      rtcSetGeometryTransform(instances[i],0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&data.tree_transforms_host[i]);
      rtcCommitGeometry(instances[i]);
    }
  }
}

void rebuild_instances(size_t old_num_trees)
{
  if (instances) {
    alignedFree(instances);
    instances = nullptr;
    g_memory_consumed -= old_num_trees * sizeof(RTCGeometry);
  }

  if (g_use_instance_array)
  {
#if defined(EMBREE_SYCL_TUTORIAL)
    void* tree_ids_device = data.tree_ids_device;
    void* tree_transforms_device = data.tree_transforms_device;
#else
    void* tree_ids_device = nullptr;
    void* tree_transforms_device = nullptr;
#endif
    instance_array = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE_ARRAY);
    rtcSetGeometryInstancedScenes(instance_array,(RTCScene*)scene_trees_selected,6);
    rtcSetSharedGeometryBufferHostDevice(instance_array, RTC_BUFFER_TYPE_INDEX,     0, RTC_FORMAT_UINT,                  (void*)data.tree_ids_host, tree_ids_device, 0, sizeof(unsigned int),  num_trees);
    rtcSetSharedGeometryBufferHostDevice(instance_array, RTC_BUFFER_TYPE_TRANSFORM, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, (void*)data.tree_transforms_host, tree_transforms_device, 0, sizeof(AffineSpace3fa), num_trees);
    rtcAttachGeometry(data.g_scene,instance_array);
    rtcReleaseGeometry(instance_array);
    rtcCommitGeometry(instance_array);
  }
  else
  {
    instances = (RTCGeometry*) alignedMalloc((num_trees)*sizeof(RTCGeometry),16);
    g_memory_consumed += num_trees * sizeof(RTCGeometry);
    for (unsigned int i = 0; i < num_trees; ++i) {
      instances[i] = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
      rtcSetGeometryInstancedScene(instances[i],scene_trees_selected[data.tree_ids_host[i]]);
      rtcSetGeometryTransform(instances[i],0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&data.tree_transforms_host[i]);
      rtcAttachGeometry(data.g_scene,instances[i]);
      rtcReleaseGeometry(instances[i]);
      rtcCommitGeometry(instances[i]);
    }
  }
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
  Vec3fa color_accum = Vec3fa(0.0f);

  // multiple samples per pixel because otherwise it looks very
  // bad due to geometric noise/aliasing in the far-field
  for (int j = 0; j < data.spp; ++j)
  for (int i = 0; i < data.spp; ++i)
  {
    float fx = (float) x + ((float)i + 0.5f) / 3;
    float fy = (float) y + ((float)j + 0.5f) / 3;
    /* initialize ray */
    Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(fx*camera.xfm.l.vx + fy*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

    /* intersect ray with scene */
    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
    rtcTraversableIntersect1(data.g_traversable,RTCRayHit_(ray),&iargs);
    RayStats_addRay(stats);

    /* shade pixels */
    Vec3fa color = Vec3fa(0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
    {
      Vec3fa diffuse = Vec3fa(1.0f);
      if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID) {
        unsigned int tree_idx = 0;
        if (data.use_instance_array && ray.instPrimID[0] != RTC_INVALID_GEOMETRY_ID) {
          tree_idx = ray.instPrimID[0];
        } else {
          tree_idx = ray.instID[0] - 1;
        }

        unsigned int tree_id = data.trees_selected[data.tree_ids_device[tree_idx]];
        Triangle* tree_triangles = data.tree_triangles[tree_id];
        Triangle triangle = tree_triangles[ray.primID];

        Vec3f* tree_colors = data.tree_vertex_colors[tree_id];
        Vec3f c0 = tree_colors[triangle.v0];
        Vec3f c1 = tree_colors[triangle.v1];
        Vec3f c2 = tree_colors[triangle.v2];
        float u = ray.u, v = ray.v, w = 1.0f-ray.u-ray.v;
        Vec3f c = w*c0 + u*c1 + v*c2;
        diffuse = Vec3fa(c);
      }
      else if (ray.geomID == 0) {
        // ground
        diffuse = Vec3fa(0.5f, 0.8f, 0.0f);
      }

      //color = Vec3fa(0.5f) + 0.5 * normal;
      color = color + diffuse*0.5f;
      Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

      /* initialize shadow ray */
      Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

      /* trace shadow ray */
      RTCOccludedArguments sargs;
      rtcInitOccludedArguments(&sargs);
      sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
      rtcTraversableOccluded1(data.g_traversable,RTCRay_(shadow),&sargs);
      RayStats_addShadowRay(stats);

      /* add light contribution */
      if (shadow.tfar >= 0.0f)
        color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
    } else {
      color = Vec3fa(0.5f, 0.8f, 0.9f);
    }

    color_accum = color_accum + color;
  }

  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color_accum.x/(data.spp*data.spp),0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color_accum.y/(data.spp*data.spp),0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color_accum.z/(data.spp*data.spp),0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
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

/* called by the C++ code to render */
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
  data.spp = g_spp;
  if (g_animate) {
    time_total += (time - time_last_frame);
  }
  time_last_frame = time;

  if (g_rebuild || g_trees_changed || g_animate) {
    int64_t total_start = get_clock();

    data.use_instance_array = g_use_instance_array;
    unsigned int old_num_trees = num_trees;
    if      (g_complexity == 0) { num_trees_sqrt = 250;  }
    else if (g_complexity == 1) { num_trees_sqrt = 500;  }
    else if (g_complexity == 2) { num_trees_sqrt = 1000; }
    else                        { num_trees_sqrt = 2000; }

    num_trees = num_trees_sqrt * num_trees_sqrt;

    if (g_rebuild)
    {
      int64_t start_cleanup = get_clock();
      if (data.g_scene) {
        rtcReleaseScene(data.g_scene);
        data.g_scene = nullptr;
      }
      int64_t start_objects = get_clock();
      g_cycles_cleanup = start_objects - start_cleanup;
      for (int i = 0; i < 6; ++i) {
        data.trees_selected[i] = g_trees[i];
        scene_trees_selected[i] = scene_trees[g_trees[i]];
      }
      rebuild_trees(old_num_trees, time_total);
      int64_t start_embree_objects = get_clock();
      g_cycles_objects = start_embree_objects - start_objects;
      g_scene = data.g_scene = rtcNewScene(g_device);
      if (g_animate)
        rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC);
      if      (g_build_quality == 0) rtcSetSceneBuildQuality(data.g_scene, RTC_BUILD_QUALITY_LOW);
      else if (g_build_quality == 1) rtcSetSceneBuildQuality(data.g_scene, RTC_BUILD_QUALITY_MEDIUM);
      else                           rtcSetSceneBuildQuality(data.g_scene, RTC_BUILD_QUALITY_HIGH);
      addTerrain(data.g_scene);
      rebuild_instances(old_num_trees);
      int64_t start_embree_bvh_build = get_clock();
      g_cycles_embree_objects = start_embree_bvh_build - start_embree_objects;
      rtcCommitScene(data.g_scene);
      data.g_traversable = rtcGetSceneTraversable(data.g_scene);
      g_cycles_embree_bvh_build = get_clock() - start_embree_bvh_build;
      g_rebuild = false;
    }
    else if (g_trees_changed)
    {
      g_cycles_cleanup = 0;
      int64_t start_objects = get_clock();
      for (int i = 0; i < 6; ++i) {
        data.trees_selected[i] = g_trees[i];
        scene_trees_selected[i] = scene_trees[g_trees[i]];
      }
      int64_t start_embree_objects = get_clock();
      g_cycles_objects = start_embree_objects - start_objects;
      update_instance_scenes();
      int64_t start_embree_bvh_build = get_clock();
      g_cycles_embree_objects = start_embree_bvh_build - start_embree_objects;
      rtcCommitScene (data.g_scene);
      data.g_traversable = rtcGetSceneTraversable(data.g_scene);
      g_cycles_embree_bvh_build = get_clock() - start_embree_bvh_build;
      g_trees_changed = false;
    } else if (g_animate) {
      g_cycles_cleanup = 0;
      int64_t start_objects = get_clock();
      update_trees(time_total);
      int64_t start_embree_objects = get_clock();
      g_cycles_objects = start_embree_objects - start_objects;
      update_instance_transforms();
      int64_t start_embree_bvh_build = get_clock();
      g_cycles_embree_objects = start_embree_bvh_build - start_embree_objects;
      rtcCommitScene (data.g_scene);
      data.g_traversable = rtcGetSceneTraversable(data.g_scene);
      g_cycles_embree_bvh_build = get_clock() - start_embree_bvh_build;
    }

    g_cycles_total = get_clock() - total_start;
  }
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  for (unsigned int i = 0; i < 6; ++i) {
    if (scene_trees[i]) {
      rtcReleaseScene(scene_trees[i]);
    }
  }
  TutorialData_Destructor(&data);
}

} // namespace embree
