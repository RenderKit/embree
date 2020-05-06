// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "buildbench.h"

#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"

#ifdef USE_GOOGLE_BENCHMARK
#include <benchmark/benchmark.h>
#endif

#include <thread>

namespace embree {

  extern uint32_t g_num_user_threads;
  
  RTCScene g_scene; // do not use!

  // for legacy reasons
  static const MAYBE_UNUSED size_t iterations_dynamic_deformable = 200;
  static const MAYBE_UNUSED size_t iterations_dynamic_dynamic    = 200;
  static const MAYBE_UNUSED size_t iterations_dynamic_static     = 50;
  static const MAYBE_UNUSED size_t iterations_static_static      = 30;
  
  void convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out, RTCBuildQuality quality)
  {
    RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryBuildQuality(geom, quality);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, (unsigned int)t, RTC_FORMAT_FLOAT3, mesh->positions[t], 0, sizeof(Vec3fa), (size_t)mesh->numVertices);
    }
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, mesh->triangles, 0, sizeof(ISPCTriangle), mesh->numTriangles);
    rtcCommitGeometry(geom);
    mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
    rtcReleaseGeometry(geom);
  }

  void convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out, RTCBuildQuality quality)
  {
    RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_QUAD);
    rtcSetGeometryTimeStepCount(geom, mesh->numTimeSteps);
    rtcSetGeometryBuildQuality(geom, quality);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, (unsigned int)t, RTC_FORMAT_FLOAT3, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, mesh->quads, 0, sizeof(ISPCQuad), mesh->numQuads);
    rtcCommitGeometry(geom);
    mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
    rtcReleaseGeometry(geom);
  }

  void convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out, RTCBuildQuality quality)
  {
    RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
    rtcSetGeometryTimeStepCount(geom, mesh->numTimeSteps);
    rtcSetGeometryBuildQuality(geom, quality);
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 16.0f;
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, (unsigned int)t, RTC_FORMAT_FLOAT3, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, mesh->subdivlevel,      0, sizeof(float),        mesh->numEdges);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT,  mesh->position_indices, 0, sizeof(unsigned int), mesh->numEdges);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,  0, RTC_FORMAT_UINT,  mesh->verticesPerFace,  0, sizeof(unsigned int), mesh->numFaces);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_HOLE,  0, RTC_FORMAT_UINT,  mesh->holes,            0, sizeof(unsigned int), mesh->numHoles);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_INDEX,    0, RTC_FORMAT_UINT2, mesh->edge_creases,          0, 2*sizeof(unsigned int), mesh->numEdgeCreases);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT,   0, RTC_FORMAT_FLOAT, mesh->edge_crease_weights,   0, sizeof(float),          mesh->numEdgeCreases);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX,  0, RTC_FORMAT_UINT,  mesh->vertex_creases,        0, sizeof(unsigned int),   mesh->numVertexCreases);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT, mesh->vertex_crease_weights, 0, sizeof(float),          mesh->numVertexCreases);
    rtcSetGeometrySubdivisionMode(geom, 0, mesh->position_subdiv_mode);
    rtcCommitGeometry(geom);
    mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
    rtcReleaseGeometry(geom);
  }

  void convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out, RTCBuildQuality quality)
  {
    RTCGeometry geom = rtcNewGeometry (g_device, hair->type);
    rtcSetGeometryTimeStepCount(geom, hair->numTimeSteps);
    rtcSetGeometryBuildQuality(geom, quality);

    for (size_t t=0; t<hair->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, (unsigned int)t, RTC_FORMAT_FLOAT4, hair->positions[t], 0, sizeof(Vertex), hair->numVertices);
    }
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, hair->hairs, 0, sizeof(ISPCHair), hair->numHairs);
    if (hair->type != RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE)
      rtcSetGeometryTessellationRate(geom,(float)hair->tessellation_rate);
    rtcCommitGeometry(geom);
    hair->geom.geomID = rtcAttachGeometry(scene_out,geom);
    rtcReleaseGeometry(geom);
  }

  RTCScene createScene(RTCSceneFlags sflags, RTCBuildQuality qflags)
  {
    RTCScene scene_out = rtcNewScene(g_device);
    rtcSetSceneFlags(scene_out,sflags);
    rtcSetSceneBuildQuality(scene_out,qflags);
    return scene_out;
  }

  void convertScene(RTCScene scene_out, ISPCScene* scene_in, RTCBuildQuality quality)
  {
    size_t numGeometries = scene_in->numGeometries;

    for (size_t i=0; i<numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out, quality);
      }
      else if (geometry->type == TRIANGLE_MESH) {
        convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out, quality);
      }
      else if (geometry->type == QUAD_MESH) {
        convertQuadMesh((ISPCQuadMesh*) geometry, scene_out, quality);
      }
      else if (geometry->type == CURVES) {
        convertCurveGeometry((ISPCHairSet*) geometry, scene_out, quality);
      }
      else
        assert(false);
    }
  }
  
  size_t getNumPrimitives(ISPCScene* scene_in)
  {
    size_t numPrimitives = 0;
    size_t numGeometries = scene_in->numGeometries;
    for (size_t i=0; i<numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        numPrimitives += ((ISPCSubdivMesh*)geometry)->numFaces;
      }
      else if (geometry->type == TRIANGLE_MESH) {
        numPrimitives += ((ISPCTriangleMesh*)geometry)->numTriangles;
      }
      else if (geometry->type == QUAD_MESH) {
        numPrimitives += ((ISPCQuadMesh*)geometry)->numQuads;
      }
      else if (geometry->type == CURVES) {
        numPrimitives += ((ISPCHairSet*)geometry)->numHairs;
      }
      else
        assert(false);
    }
    return numPrimitives;
  }

  size_t getNumObjects(ISPCScene* scene_in)
  {
    return scene_in->numGeometries;
  }

  void updateObjects(ISPCScene* scene_in, RTCScene scene_out)  // FIXME: geomID can be accessed easier now
  {
    size_t numGeometries = scene_in->numGeometries;
    for (size_t i=0; i<numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      RTCGeometry geom = rtcGetGeometry(scene_out,geometry->geomID);
      rtcUpdateGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX, 0);
      rtcCommitGeometry(geom);
    }
  }

  void deleteObjects(ISPCScene* scene_in, RTCScene scene_out) // FIXME: geomID can be accessed easier now
  {
    size_t numGeometries = scene_in->numGeometries;
    for (size_t i=0; i<numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      rtcDetachGeometry(scene_out,geometry->geomID);
    }
  }
  
#ifdef USE_GOOGLE_BENCHMARK
  inline void addCounter(BenchState& state, size_t numPrims, size_t numObjects)
  {
    state.state->SetItemsProcessed(state.state->iterations() * numPrims);
    state.state->counters["Prims"] = ::benchmark::Counter(numPrims);
    state.state->counters["Objects"] = ::benchmark::Counter(numObjects);
  }
#endif
  
  void Benchmark_Dynamic_Update_Legacy(ISPCScene* scene_in, BenchParams& params, RTCBuildQuality quality = RTC_BUILD_QUALITY_LOW)
  {
    size_t benchmark_iterations = params.minTimeOrIterations;
    if (benchmark_iterations <= 0) {
      benchmark_iterations = (quality == RTC_BUILD_QUALITY_MEDIUM) 
                           ? iterations_dynamic_static 
                           : iterations_dynamic_dynamic;
    }

    RTCScene scene = createScene(RTC_SCENE_FLAG_DYNAMIC, RTC_BUILD_QUALITY_LOW);
    convertScene(scene, scene_in, quality);
    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+params.skipIterations;i++)
    {
      updateObjects(scene_in,scene);
      double t0 = getSeconds();
      rtcCommitScene (scene);
      double t1 = getSeconds();
      if (i >= params.skipIterations)
      {
        time += t1 - t0;
        iterations++;
      }
    }

    if (quality == RTC_BUILD_QUALITY_MEDIUM)
      std::cout << "BENCHMARK_UPDATE_DYNAMIC_STATIC ";
    else if (quality == RTC_BUILD_QUALITY_LOW)
      std::cout << "BENCHMARK_UPDATE_DYNAMIC_DYNAMIC ";
    else if (quality == RTC_BUILD_QUALITY_REFIT)
      std::cout << "BENCHMARK_UPDATE_DYNAMIC_DEFORMABLE ";
    else
      FATAL("unknown flags");

    if (iterations == 0) iterations = 1;
    std::cout << iterations << " iterations, " << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    rtcReleaseScene (scene);
  }
  
  void Benchmark_Dynamic_Update(
    BenchState& state, 
    BenchParams& params, 
    BuildBenchParams& buildParams, 
    ISPCScene* ispc_scene, 
    RTCBuildQuality quality)
  {
#ifdef USE_GOOGLE_BENCHMARK
    if (params.legacy) {
      Benchmark_Dynamic_Update_Legacy(ispc_scene, params, quality);
      return;
    }

    RTCScene scene = createScene(RTC_SCENE_FLAG_DYNAMIC, RTC_BUILD_QUALITY_LOW);
    convertScene(scene, ispc_scene, quality);
    const size_t primitives = getNumPrimitives(ispc_scene);
    const size_t objects = getNumObjects(ispc_scene);

    // warmup
    for (int i = 0; i < params.minTimeOrIterations; ++i) {
      updateObjects(ispc_scene,scene);
      rtcCommitScene(scene);
    }

    for (auto _ : *state.state) {
      state.state->PauseTiming();

      updateObjects(ispc_scene,scene);

      state.state->ResumeTiming();

      rtcCommitScene (scene);
    }
    
    addCounter(state, primitives, objects);
    
    rtcReleaseScene (scene);
#else
    Benchmark_Dynamic_Update_Legacy(ispc_scene, params, quality);
#endif
  }
  
  void Benchmark_Dynamic_Create_Legacy(ISPCScene* scene_in, BenchParams& params, RTCBuildQuality quality = RTC_BUILD_QUALITY_MEDIUM)
  {
    size_t benchmark_iterations = params.minTimeOrIterations;
    if (benchmark_iterations <= 0) {
      benchmark_iterations = (quality == RTC_BUILD_QUALITY_MEDIUM) 
                           ? iterations_dynamic_static 
                           : iterations_dynamic_dynamic;
    }

    RTCScene scene = createScene(RTC_SCENE_FLAG_DYNAMIC, RTC_BUILD_QUALITY_LOW);
    convertScene(scene, scene_in,quality);
    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+params.skipIterations;i++)
    {
      deleteObjects(scene_in,scene);
      convertScene(scene, scene_in,quality);
      double t0 = getSeconds();
      rtcCommitScene (scene);
      double t1 = getSeconds();
      if (i >= params.skipIterations)
      {
        time += t1 - t0;
        iterations++;
      }
    }

    if (quality == RTC_BUILD_QUALITY_MEDIUM)
      std::cout << "BENCHMARK_CREATE_DYNAMIC_STATIC ";
    else if (quality == RTC_BUILD_QUALITY_LOW)
      std::cout << "BENCHMARK_CREATE_DYNAMIC_DYNAMIC ";
    else if (quality == RTC_BUILD_QUALITY_REFIT)
      std::cout << "BENCHMARK_CREATE_DYNAMIC_DEFORMABLE ";
    else
      FATAL("unknown flags");

    if (iterations == 0) iterations = 1;
    std::cout << iterations << " iterations, " << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    rtcReleaseScene (scene);
  }

  void Benchmark_Dynamic_Create(
    BenchState& state, 
    BenchParams& params, 
    BuildBenchParams& buildParams, 
    ISPCScene* ispc_scene, 
    RTCBuildQuality quality)
  {
#ifdef USE_GOOGLE_BENCHMARK
    if (params.legacy) {
      Benchmark_Dynamic_Create_Legacy(ispc_scene, params, quality);
      return;
    }

    RTCScene scene = createScene(RTC_SCENE_FLAG_DYNAMIC, RTC_BUILD_QUALITY_LOW);
    convertScene(scene, ispc_scene, quality);
    const size_t primitives = getNumPrimitives(ispc_scene);
    const size_t objects = getNumObjects(ispc_scene);
    
    // warmup
    for (int i = 0; i < params.minTimeOrIterations; ++i) {
      deleteObjects(ispc_scene, scene);
      convertScene(scene, ispc_scene, quality);
      rtcCommitScene(scene);
    }
    
    for (auto _ : *state.state) {
      state.state->PauseTiming(); 

      deleteObjects(ispc_scene, scene);
      convertScene(scene, ispc_scene, quality);

      state.state->ResumeTiming(); 

      rtcCommitScene(scene);
    }
    
    addCounter(state, primitives, objects);

    rtcReleaseScene (scene);
#else
    Benchmark_Dynamic_Create_Legacy(ispc_scene, params, quality);
#endif
  }

  void Benchmark_Static_Create_Legacy(ISPCScene* scene_in, BenchParams& params, RTCBuildQuality quality, RTCBuildQuality qflags)
  {
    size_t benchmark_iterations = params.minTimeOrIterations;
    if (benchmark_iterations <= 0)
      benchmark_iterations = iterations_static_static;

    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;

    for(size_t i=0;i<benchmark_iterations+params.skipIterations;i++)
    {
      RTCScene scene = createScene(RTC_SCENE_FLAG_NONE,qflags);
      convertScene(scene,scene_in,quality);

      const size_t numThreads = TaskScheduler::threadCount();
      std::vector<std::thread> threads;
      threads.reserve(numThreads);

      double t0 = getSeconds();
      rtcCommitScene (scene);
      double t1 = getSeconds();
      if (i >= params.skipIterations)
      {
        time += t1 - t0;
        iterations++;
      }
      rtcReleaseScene (scene);
    }

    if (qflags == RTC_BUILD_QUALITY_HIGH)
      std::cout << "BENCHMARK_CREATE_HQ_STATIC_";
    else
      std::cout << "BENCHMARK_CREATE_STATIC_";

    if (quality == RTC_BUILD_QUALITY_MEDIUM)
      std::cout << "STATIC ";
    else if (quality == RTC_BUILD_QUALITY_LOW)
      std::cout << "DYNAMIC ";
    else if (quality == RTC_BUILD_QUALITY_REFIT)
      std::cout << "DEFORMABLE ";
    else
      FATAL("unknown flags");

    if (iterations == 0) iterations = 1;
    std::cout << iterations << " iterations, " << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;
  }

  void Benchmark_Static_Create(
    BenchState& state, 
    BenchParams& params, 
    BuildBenchParams& buildParams, 
    ISPCScene* ispc_scene, 
    RTCBuildQuality quality, 
    RTCBuildQuality qflags)
  {
#ifdef USE_GOOGLE_BENCHMARK
    if (params.legacy) {
      Benchmark_Static_Create_Legacy(ispc_scene, params, quality, qflags);
      return;
    }

    const size_t primitives = getNumPrimitives(ispc_scene);
    const size_t objects = getNumObjects(ispc_scene);
    
    // warmup
    for (int i = 0; i < params.minTimeOrIterations; ++i) {
      RTCScene scene = createScene(RTC_SCENE_FLAG_NONE, qflags);
      convertScene(scene, ispc_scene, quality);
      const size_t numThreads = TaskScheduler::threadCount();
      std::vector<std::thread> threads;
      threads.reserve(numThreads);
      rtcCommitScene(scene);
      rtcReleaseScene(scene);
    }

    for(auto _ : *state.state) {
      state.state->PauseTiming();

      RTCScene scene = createScene(RTC_SCENE_FLAG_NONE, qflags);
      convertScene(scene, ispc_scene, quality);

      const size_t numThreads = TaskScheduler::threadCount();
      std::vector<std::thread> threads;
      threads.reserve(numThreads);

      state.state->ResumeTiming();

      rtcCommitScene(scene);

      state.state->PauseTiming();

      rtcReleaseScene(scene);
      
      state.state->ResumeTiming();
    }

    addCounter(state, primitives, objects);
#else
    Benchmark_Static_Create_Legacy(ispc_scene, params, quality, qflags);
#endif
  }
  struct Helper {
    BarrierSys barrier;
    volatile bool term = false;
    RTCScene scene;

    void perform_work(size_t threadID) {
      setAffinity(threadID);
      while (true) {
        barrier.wait();
        if (term)
          return;
        rtcJoinCommitScene(scene);
        barrier.wait();
      }
    }
  } helper;

  void Benchmark_Static_Create_UserThreads_Legacy(ISPCScene* scene_in, BenchParams& params, RTCBuildQuality quality, RTCBuildQuality qflags)
  {
    size_t benchmark_iterations = params.minTimeOrIterations;
    if (benchmark_iterations <= 0)
      benchmark_iterations = iterations_static_static;

    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    const size_t numThreads = g_num_user_threads;

    Helper helper;
    helper.barrier.init(numThreads);

    std::vector<std::thread> threads;
    threads.reserve(numThreads);
    
    /* ramp up threads */
    setAffinity(0); 
    for (size_t i=1; i<numThreads; i++) 
      threads.push_back(std::thread(&Helper::perform_work, &helper, i));
    
    for (size_t i=0; i<benchmark_iterations+params.skipIterations; i++)
    {
      helper.scene = createScene(RTC_SCENE_FLAG_NONE,qflags);
      convertScene(helper.scene,scene_in,quality);

      double t0 = getSeconds();
      
      helper.barrier.wait();
      rtcJoinCommitScene(helper.scene);
      helper.barrier.wait();
      
      double t1 = getSeconds();

      if (i >= params.skipIterations)
      {
        time += t1 - t0;
        iterations++;
      }

      if (iterations == 0) iterations = 1;
      std::cout << primitives << " primitives, " << objects << " objects, "
                << time/iterations << " s, "
                << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

      rtcReleaseScene(helper.scene);
    }

    /* terminate task loop */
    helper.term = true;
    helper.barrier.wait();
    for (auto& thread: threads)
      thread.join();

    if (qflags == RTC_BUILD_QUALITY_HIGH)
      std::cout << "BENCHMARK_CREATE_HQ_STATIC_";
    else
      std::cout << "BENCHMARK_CREATE_STATIC_";

    if (quality == RTC_BUILD_QUALITY_MEDIUM)
      std::cout << "STATIC ";
    else if (quality == RTC_BUILD_QUALITY_LOW)
      std::cout << "DYNAMIC ";
    else if (quality == RTC_BUILD_QUALITY_REFIT)
      std::cout << "DEFORMABLE ";
    else
      FATAL("unknown flags");

    if (iterations == 0) iterations = 1;
    std::cout << iterations << " iterations, " << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;
  }

  void Benchmark_Static_Create_UserThreads(
    BenchState& state, 
    BenchParams& params, 
    BuildBenchParams& buildParams, 
    ISPCScene* ispc_scene, 
    RTCBuildQuality quality, 
    RTCBuildQuality qflags)
  {
#ifdef USE_GOOGLE_BENCHMARK
    if (params.legacy) {
      Benchmark_Static_Create_UserThreads_Legacy(ispc_scene, params, quality, qflags);
      return;
    }
    
    size_t primitives = getNumPrimitives(ispc_scene);
    const size_t objects = getNumObjects(ispc_scene);
    const size_t numThreads = g_num_user_threads;

    Helper helper;
    helper.barrier.init(numThreads);

    std::vector<std::thread> threads;
    threads.reserve(numThreads);
    
    /* ramp up threads */
    setAffinity(0); 
    for (size_t i=1; i<numThreads; i++) 
      threads.push_back(std::thread(&Helper::perform_work, &helper, i));
    
    // warmup
    for (int i = 0; i < params.minTimeOrIterations; ++i) {
      helper.scene = createScene(RTC_SCENE_FLAG_NONE,qflags);
      convertScene(helper.scene,ispc_scene,quality);
      helper.barrier.wait();
      rtcJoinCommitScene(helper.scene);
      helper.barrier.wait();
      rtcReleaseScene(helper.scene);
    }

    for (auto _ : *state.state) {
      state.state->PauseTiming();

      helper.scene = createScene(RTC_SCENE_FLAG_NONE,qflags);
      convertScene(helper.scene,ispc_scene,quality);

      state.state->ResumeTiming();
      
      helper.barrier.wait();
      rtcJoinCommitScene(helper.scene);
      helper.barrier.wait();
      
      state.state->PauseTiming();

      rtcReleaseScene(helper.scene);
      
      state.state->ResumeTiming();
    }

    /* terminate task loop */
    helper.term = true;
    helper.barrier.wait();
    for (auto& thread: threads)
      thread.join();

    addCounter(state, primitives, objects);
#else
    Benchmark_Static_Create_UserThreads_Legacy(ispc_scene, params, quality, qflags);
#endif
  }

  extern "C" void device_init (char* cfg)
  {
  }

  void renderFrameStandard (int* pixels,
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
  }

  /* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
  }
} // namespace embree
