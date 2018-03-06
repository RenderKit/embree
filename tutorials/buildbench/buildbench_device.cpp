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

#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

  static const MAYBE_UNUSED size_t skip_iterations               = 5;
  static const MAYBE_UNUSED size_t iterations_dynamic_deformable = 200;
  static const MAYBE_UNUSED size_t iterations_dynamic_dynamic    = 200;
  static const MAYBE_UNUSED size_t iterations_dynamic_static     = 50;
  static const MAYBE_UNUSED size_t iterations_static_static      = 30;

  extern "C" ISPCScene* g_ispc_scene;

  /* scene data */
  RTCScene g_scene = nullptr;

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

  void Benchmark_Dynamic_Update(ISPCScene* scene_in, size_t benchmark_iterations, RTCBuildQuality quality = RTC_BUILD_QUALITY_LOW)
  {
    assert(g_scene == nullptr);
    g_scene = createScene(RTC_SCENE_FLAG_DYNAMIC, RTC_BUILD_QUALITY_LOW);
    convertScene(g_scene, scene_in, quality);
    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      updateObjects(scene_in,g_scene);
      double t0 = getSeconds();
      rtcCommitScene (g_scene);
      double t1 = getSeconds();
      if (i >= skip_iterations)
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

    std::cout << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    rtcReleaseScene (g_scene);
    g_scene = nullptr;
  }

  void Benchmark_Dynamic_Create(ISPCScene* scene_in, size_t benchmark_iterations, RTCBuildQuality quality = RTC_BUILD_QUALITY_MEDIUM)
  {
    assert(g_scene == nullptr);
    g_scene = createScene(RTC_SCENE_FLAG_DYNAMIC, RTC_BUILD_QUALITY_LOW);
    convertScene(g_scene, scene_in,quality);
    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      deleteObjects(scene_in,g_scene);
      convertScene(g_scene, scene_in,quality);
      double t0 = getSeconds();
      rtcCommitScene (g_scene);
      double t1 = getSeconds();
      if (i >= skip_iterations)
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

    std::cout << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    rtcReleaseScene (g_scene);
    g_scene = nullptr;
  }

  void Benchmark_Static_Create(ISPCScene* scene_in, size_t benchmark_iterations, RTCBuildQuality quality, RTCBuildQuality qflags)
  {
    assert(g_scene == nullptr);
    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      g_scene = createScene(RTC_SCENE_FLAG_NONE,qflags);
      convertScene(g_scene,scene_in,quality);

      double t0 = getSeconds();
      rtcCommitScene (g_scene);
      double t1 = getSeconds();
      if (i >= skip_iterations)
      {
        time += t1 - t0;
        iterations++;
      }
      rtcReleaseScene (g_scene);
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

    std::cout << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    g_scene = nullptr;
  }

  void Pause()
  {
    std::cout << "sleeping..." << std::flush;
    sleepSeconds(3);
    std::cout << "done" << std::endl;
  }


  /* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    /* set error handler */
    Benchmark_Dynamic_Update(g_ispc_scene,iterations_dynamic_dynamic,RTC_BUILD_QUALITY_REFIT);
    Pause();
    Benchmark_Dynamic_Update(g_ispc_scene,iterations_dynamic_dynamic,RTC_BUILD_QUALITY_LOW);
    Pause();
    Benchmark_Dynamic_Update(g_ispc_scene,iterations_dynamic_static ,RTC_BUILD_QUALITY_MEDIUM);
    Pause();
    Benchmark_Dynamic_Create(g_ispc_scene,iterations_dynamic_dynamic,RTC_BUILD_QUALITY_REFIT);
    Pause();
    Benchmark_Dynamic_Create(g_ispc_scene,iterations_dynamic_dynamic,RTC_BUILD_QUALITY_LOW);
    Pause();
    Benchmark_Dynamic_Create(g_ispc_scene,iterations_dynamic_static ,RTC_BUILD_QUALITY_MEDIUM);
    Pause();
    Benchmark_Static_Create(g_ispc_scene,iterations_static_static,RTC_BUILD_QUALITY_MEDIUM,RTC_BUILD_QUALITY_MEDIUM);
    Pause();
    Benchmark_Static_Create(g_ispc_scene,iterations_static_static,RTC_BUILD_QUALITY_MEDIUM,RTC_BUILD_QUALITY_HIGH);
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
