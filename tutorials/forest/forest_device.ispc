// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/math/random_sampler.isph"
#include "forest_device.isph"

#include "trees.h"

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_INSTANCE | \
  RTC_FEATURE_FLAG_INSTANCE_ARRAY

RTCScene g_scene = NULL;
uniform TutorialData data;

uniform unsigned int num_trees_sqrt;
uniform unsigned int num_trees;
uniform RTCScene scene_terrain;
uniform RTCScene scene_trees[6];
uniform RTCScene scene_trees_selected[6];
uniform float time_last_frame = 0;
uniform float time_total = 0;

extern uniform bool g_use_instance_array;
extern uniform bool g_rebuild;
extern uniform int g_complexity;
extern uniform int g_build_quality;
extern uniform int g_spp;
extern uniform bool g_trees_changed;
extern uniform bool g_animate;
extern uniform size_t g_memory_consumed;
extern uniform size_t g_cycles_cleanup;
extern uniform size_t g_cycles_objects;
extern uniform size_t g_cycles_embree_objects;
extern uniform size_t g_cycles_embree_bvh_build;
extern uniform size_t g_cycles_total;
extern uniform int g_trees[6];

extern "C" uniform int64 get_clock();

uniform RTCGeometry instance_array;
uniform RTCGeometry* uniform instances = NULL;

uniform unsigned int addTree(RTCScene scene_i, uniform unsigned int tree_idx)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  uniform const float* uniform vertices = tree_vertices[tree_idx];
  uniform const float* uniform colors = tree_colors[tree_idx];
  uniform const unsigned int* uniform indices = tree_indices[tree_idx];
  uniform const unsigned int num_vertices = tree_num_vertices[tree_idx];
  uniform const unsigned int num_colors = tree_num_colors[tree_idx];
  uniform const unsigned int num_triangles = tree_num_triangles[tree_idx];

  /* set vertices and vertex colors */
  uniform Vertex* uniform vertex_buffer = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),num_vertices);
  for (uniform unsigned int i = 0; i < num_vertices; ++i) {
    vertex_buffer[i].x = vertices[3 * i + 0];
    vertex_buffer[i].y = vertices[3 * i + 1];
    vertex_buffer[i].z = vertices[3 * i + 2];
    vertex_buffer[i].r = 0.f;
  }

  /* set triangles and face colors */
  uniform Triangle* uniform index_buffer = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),num_triangles);
  data.tree_triangles[tree_idx] = index_buffer;
  for (uniform unsigned int i = 0; i < num_triangles; ++i) {
    index_buffer[i].v0 = indices[3 * i + 0];
    index_buffer[i].v1 = indices[3 * i + 1];
    index_buffer[i].v2 = indices[3 * i + 2];
  }

  /* create vertex color array */
  uniform Vec3fa* uniform color_buffer = uniform new uniform Vec3fa[num_colors];
  g_memory_consumed += num_colors * sizeof(uniform Vec3fa);
  data.tree_vertex_colors[tree_idx] = color_buffer;
  for (uniform unsigned int i = 0; i < num_colors; ++i) {
    color_buffer[i] = make_Vec3fa(colors[3 * i + 0], colors[3 * i + 1], colors[3 * i + 2]);
  }

  rtcSetGeometryVertexAttributeCount(mesh,1);
  rtcSetSharedGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,0,RTC_FORMAT_FLOAT3,color_buffer,0,sizeof(uniform Vec3fa),num_colors);

  rtcCommitGeometry(mesh);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addTerrain(RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry mesh = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */

  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),terrain_num_vertices);
  for (uniform unsigned int i = 0; i < terrain_num_vertices; ++i) {
    vertices[i].x = terrain_vertices[3 * i + 0];
    vertices[i].y = terrain_vertices[3 * i + 1];
    vertices[i].z = terrain_vertices[3 * i + 2];
  }

  /* set triangles */
  data.terrain_triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),terrain_num_triangles);
  for (uniform unsigned int i = 0; i < terrain_num_triangles; ++i) {
    data.terrain_triangles[i].v0 = terrain_indices[3 * i + 0];
    data.terrain_triangles[i].v1 = terrain_indices[3 * i + 1];
    data.terrain_triangles[i].v2 = terrain_indices[3 * i + 2];
  }

  rtcCommitGeometry(mesh);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

unmasked uniform bool monitorMemoryFunction(void* uniform ptr, uniform int64 bytes, uniform bool post)
{
  g_memory_consumed += bytes;
  return true;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  rtcSetDeviceMemoryMonitorFunction(g_device, monitorMemoryFunction, NULL);

  TutorialData_Constructor(&data);

  for (uniform unsigned int i = 0; i < 6; ++i) {
    scene_trees[i] = rtcNewScene(g_device);
    addTree(scene_trees[i], i);
    rtcCommitScene(scene_trees[i]);
  }

  /* add ground plane */
  scene_terrain = rtcNewScene(g_device);
  addTerrain(scene_terrain);
  rtcCommitScene(scene_terrain);
}

task void update_tree(uniform float time, uniform RTCBounds bounds)
{
  const uniform unsigned int T = taskIndex;
  RTCScene scene = scene_terrain;
  RandomSampler rng;

  foreach (tt = 0 ... num_trees_sqrt) {
    unsigned int t = T * num_trees_sqrt + tt;
    RandomSampler_init(rng, t);
    data.tree_ids[t] = min(5, (int)(6 * RandomSampler_getFloat(rng)));

    unsigned int j = t / num_trees_sqrt;
    unsigned int i = t % num_trees_sqrt;

    float px = bounds.lower_x + ((float)i + RandomSampler_getFloat(rng))/((float)num_trees_sqrt) * (bounds.upper_x - bounds.lower_x);
    float pz = bounds.lower_z + ((float)j + RandomSampler_getFloat(rng))/((float)num_trees_sqrt) * (bounds.upper_z - bounds.lower_z);
    float py = bounds.upper_y;

    float dx = bounds.upper_x - bounds.lower_x;
    float dz = bounds.upper_z - bounds.lower_z;

    float phi = 2*M_PI*RandomSampler_getFloat(rng);
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

    Ray ray = make_Ray(make_Vec3f(px, py, pz), make_Vec3f(0.f, -1.f, 0.f), 0.0f, inf);
    uniform RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
    rtcIntersectV(scene,RTCRayHit_(ray),&iargs);

    Vec3f treePos;
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID) {
      py = py - ray.tfar;
      treePos = make_Vec3f(px, py, pz);
    } else {
      treePos = make_Vec3f(inf, inf, inf);
    }

    data.tree_transforms[t] = make_AffineSpace3f_translate(treePos);
  }
}

void update_trees(uniform float time)
{
  uniform RTCBounds bounds;
  rtcGetSceneBounds(scene_terrain, &bounds);

  launch[num_trees/num_trees_sqrt] update_tree(time, bounds); sync;
}

void rebuild_trees(uniform size_t old_num_trees, uniform float time)
{
  if (data.tree_ids) {
    delete[] data.tree_ids;
    g_memory_consumed -= old_num_trees * sizeof(uniform uint32);
  }
  data.tree_ids = uniform new uniform uint32[num_trees];
  g_memory_consumed += num_trees * sizeof(uniform uint32);

  if (data.tree_transforms) {
    delete[] data.tree_transforms;
    g_memory_consumed -= old_num_trees * sizeof(uniform AffineSpace3f);
  }
  data.tree_transforms = uniform new uniform AffineSpace3f[num_trees];
  g_memory_consumed += num_trees * sizeof(uniform AffineSpace3f);

  update_trees(time);
}

void update_instance_scenes()
{
  if (g_use_instance_array)
  {
    rtcSetGeometryInstancedScenes(instance_array,(uniform RTCScene* uniform)scene_trees_selected,6);
    rtcCommitGeometry(instance_array);
  }
  else
  {
    for (uniform unsigned int i = 0; i < num_trees; ++i) {
      rtcSetGeometryInstancedScene(instances[i],scene_trees_selected[data.tree_ids[i]]);
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
    for (uniform unsigned int i = 0; i < num_trees; ++i) {
      rtcSetGeometryTransform(instances[i],0,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&data.tree_transforms[i]);
      rtcCommitGeometry(instances[i]);
    }
  }
}

void rebuild_instances(uniform size_t old_num_trees)
{
  if (instances) {
    delete[] instances;
    instances = NULL;
    g_memory_consumed -= old_num_trees * sizeof(uniform RTCGeometry);
  }

  if (g_use_instance_array)
  {
    instance_array = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE_ARRAY);
    rtcSetGeometryInstancedScenes(instance_array,(uniform RTCScene* uniform)scene_trees_selected,6);
    rtcSetSharedGeometryBuffer(instance_array, RTC_BUFFER_TYPE_INDEX,     0, RTC_FORMAT_UINT,                  (void* uniform)data.tree_ids, 0, sizeof(uniform unsigned int),  num_trees);
    rtcSetSharedGeometryBuffer(instance_array, RTC_BUFFER_TYPE_TRANSFORM, 0, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR, (void* uniform)data.tree_transforms, 0, sizeof(uniform AffineSpace3f), num_trees);
    rtcAttachGeometry(data.g_scene,instance_array);
    rtcReleaseGeometry(instance_array);
    rtcCommitGeometry(instance_array);
  }
  else
  {
    instances = uniform new uniform RTCGeometry[num_trees];
    g_memory_consumed += num_trees * sizeof(uniform RTCGeometry);
    for (uniform unsigned int i = 0; i < num_trees; ++i) {
      instances[i] = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
      rtcSetGeometryInstancedScene(instances[i],scene_trees_selected[data.tree_ids[i]]);
      rtcSetGeometryTransform(instances[i],0,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&data.tree_transforms[i]);
      rtcAttachGeometry(data.g_scene,instances[i]);
      rtcReleaseGeometry(instances[i]);
      rtcCommitGeometry(instances[i]);
    }
  }
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
  Vec3f color_accum = make_Vec3f(0.0f);

  // multiple samples per pixel because otherwise it looks very
  // bad due to geometric noise/aliasing in the far-field
  for (int j = 0; j < g_spp; ++j)
  for (int i = 0; i < g_spp; ++i)
  {
    float fx = (float) x + ((float)i + 0.5f) / 3;
    float fy = (float) y + ((float)j + 0.5f) / 3;
    /* initialize ray */
    Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(fx*camera.xfm.l.vx + fy*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

    /* intersect ray with scene */
    uniform RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
    rtcIntersectV(data.g_scene,RTCRayHit_(ray),&iargs);
    RayStats_addRay(stats);

    /* shade pixels */
    Vec3f color = make_Vec3f(0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
    {
      Vec3f diffuse = make_Vec3f(1.0f);
      if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID) {
        unsigned int tree_idx = 0;
        if (data.use_instance_array && ray.instPrimID[0] != RTC_INVALID_GEOMETRY_ID) {
          tree_idx = ray.instPrimID[0];
        } else {
          tree_idx = ray.instID[0] - 1;
        }

        unsigned int tree_id = data.trees_selected[data.tree_ids[tree_idx]];
        uniform Triangle* varying tree_triangles = data.tree_triangles[tree_id];
        Triangle triangle = tree_triangles[ray.primID];

        uniform Vec3fa* varying tree_colors = data.tree_vertex_colors[tree_id];
        Vec3fa c0 = tree_colors[triangle.v0];
        Vec3fa c1 = tree_colors[triangle.v1];
        Vec3fa c2 = tree_colors[triangle.v2];
        float u = ray.u, v = ray.v, w = 1.0f-ray.u-ray.v;
        Vec3fa c = w*c0 + u*c1 + v*c2;
        diffuse = make_Vec3f(c);
      }
      else if (ray.geomID == 0) {
        // ground
        diffuse = make_Vec3f(0.5f, 0.8f, 0.0f);
      }

      color = color + diffuse*0.5f;
      Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

      /* initialize shadow ray */
      Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

      /* trace shadow ray */
      uniform RTCOccludedArguments sargs;
      rtcInitOccludedArguments(&sargs);
      sargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
      rtcOccludedV(data.g_scene,RTCRay_(shadow),&sargs);
      RayStats_addShadowRay(stats);

      /* add light contribution */
      if (shadow.tfar >= 0.0f)
        color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
    } else {
      color = make_Vec3f(0.5f, 0.8f, 0.9f);
    }

    color_accum = color_accum + color;
  }

  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color_accum.x/(g_spp*g_spp),0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color_accum.y/(g_spp*g_spp),0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color_accum.z/(g_spp*g_spp),0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
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

/* called by the C++ code to render */
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
  if (g_animate) {
    time_total += (time - time_last_frame);
  }
  time_last_frame = time;

  if (g_rebuild || g_trees_changed || g_animate) {
    uniform int64 total_start = get_clock();

    data.use_instance_array = g_use_instance_array;
    uniform unsigned int old_num_trees = num_trees;
    if      (g_complexity == 0) { num_trees_sqrt = 250;  }
    else if (g_complexity == 1) { num_trees_sqrt = 500;  }
    else if (g_complexity == 2) { num_trees_sqrt = 1000; }
    else                        { num_trees_sqrt = 2000; }

    num_trees = num_trees_sqrt * num_trees_sqrt;

    if (g_rebuild)
    {
      uniform int64 start_cleanup = get_clock();
      if (data.g_scene) {
        rtcReleaseScene(data.g_scene);
        data.g_scene = NULL;
      }
      uniform int64 start_objects = get_clock();
      g_cycles_cleanup = start_objects - start_cleanup;
      for (uniform int i = 0; i < 6; ++i) {
        data.trees_selected[i] = g_trees[i];
        scene_trees_selected[i] = scene_trees[g_trees[i]];
      }
      rebuild_trees(old_num_trees, time_total);
      uniform int64 start_embree_objects = get_clock();
      g_cycles_objects = start_embree_objects - start_objects;
      g_scene = data.g_scene = rtcNewScene(g_device);
      if (g_animate)
        rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC);
      if      (g_build_quality == 0) rtcSetSceneBuildQuality(data.g_scene, RTC_BUILD_QUALITY_LOW);
      else if (g_build_quality == 1) rtcSetSceneBuildQuality(data.g_scene, RTC_BUILD_QUALITY_MEDIUM);
      else                           rtcSetSceneBuildQuality(data.g_scene, RTC_BUILD_QUALITY_HIGH);
      addTerrain(data.g_scene);
      rebuild_instances(old_num_trees);
      uniform int64 start_embree_bvh_build = get_clock();
      g_cycles_embree_objects = start_embree_bvh_build - start_embree_objects;
      rtcCommitScene(data.g_scene);
      g_cycles_embree_bvh_build = get_clock() - start_embree_bvh_build;
      g_rebuild = false;
    }
    else if (g_trees_changed)
    {
      g_cycles_cleanup = 0;
      uniform int64 start_objects = get_clock();
      for (uniform int i = 0; i < 6; ++i) {
        data.trees_selected[i] = g_trees[i];
        scene_trees_selected[i] = scene_trees[g_trees[i]];
      }
      uniform int64 start_embree_objects = get_clock();
      g_cycles_objects = start_embree_objects - start_objects;
      update_instance_scenes();
      uniform int64 start_embree_bvh_build = get_clock();
      g_cycles_embree_objects = start_embree_bvh_build - start_embree_objects;
      rtcCommitScene (data.g_scene);
      g_cycles_embree_bvh_build = get_clock() - start_embree_bvh_build;
      g_trees_changed = false;
    } else if (g_animate) {
      g_cycles_cleanup = 0;
      uniform int64 start_objects = get_clock();
      update_trees(time_total);
      uniform int64 start_embree_objects = get_clock();
      g_cycles_objects = start_embree_objects - start_objects;
      update_instance_transforms();
      uniform int64 start_embree_bvh_build = get_clock();
      g_cycles_embree_objects = start_embree_bvh_build - start_embree_objects;
      rtcCommitScene (data.g_scene);
      g_cycles_embree_bvh_build = get_clock() - start_embree_bvh_build;
    }

    g_cycles_total = get_clock() - total_start;
  }
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  for (uniform unsigned int i = 0; i < 6; ++i) {
    if (scene_trees[i]) {
      rtcReleaseScene(scene_trees[i]);
    }
  }
  TutorialData_Destructor(&data);
}
