// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
  RTCDevice g_device = nullptr;
  RTCScene g_scene = nullptr;

  void convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out, RTCGeometryFlags flags)
  {
    RTCGeometry geom = rtcNewTriangleMesh (g_device, flags, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(geom, RTC_VERTEX_BUFFER_(t),mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetBuffer(geom, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle), mesh->numTriangles);
    mesh->geom.geomID = rtcAttachAndReleaseGeometry(scene_out,geom);
  }

  void convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out, RTCGeometryFlags flags)
  {
    RTCGeometry geom = rtcNewQuadMesh (g_device, flags, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(geom, RTC_VERTEX_BUFFER_(t),mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetBuffer(geom, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad), mesh->numQuads);
    mesh->geom.geomID = rtcAttachAndReleaseGeometry(scene_out,geom);
  }

  void convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out, RTCGeometryFlags flags)
  {
    RTCGeometry geom = rtcNewSubdivisionMesh(g_device,flags, mesh->numTimeSteps);
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 16.0f;
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(geom, RTC_VERTEX_BUFFER_(t),mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetBuffer(geom, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float), mesh->numEdges);
    rtcSetBuffer(geom, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int), mesh->numEdges);
    rtcSetBuffer(geom, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int), mesh->numFaces);
    rtcSetBuffer(geom, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int), mesh->numHoles);
    rtcSetBuffer(geom, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(unsigned int), mesh->numEdgeCreases);
    rtcSetBuffer(geom, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float), mesh->numEdgeCreases);
    rtcSetBuffer(geom, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(unsigned int), mesh->numVertexCreases);
    rtcSetBuffer(geom, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float), mesh->numVertexCreases);
    rtcSetSubdivisionMode(geom, 0, mesh->position_subdiv_mode);
    mesh->geom.geomID = rtcAttachAndReleaseGeometry(scene_out,geom);
  }

  void convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out, RTCGeometryFlags flags)
  {
    RTCGeometry geom = rtcNewCurveGeometry (g_device, flags, hair->type, hair->basis, hair->numTimeSteps);

    for (size_t t=0; t<hair->numTimeSteps; t++) {
      rtcSetBuffer(geom,RTC_VERTEX_BUFFER_(t),hair->positions[t],0,sizeof(Vertex),hair->numVertices);
    }
    rtcSetBuffer(geom,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair),hair->numHairs);
    if (hair->basis != RTC_BASIS_LINEAR)
      rtcSetTessellationRate(geom,(float)hair->tessellation_rate);
    hair->geom.geomID = rtcAttachAndReleaseGeometry(scene_out,geom);
  }

  RTCScene createScene(RTCSceneFlags sflags)
  {
    RTCSceneFlags scene_flags = sflags | RTC_SCENE_INCOHERENT;
    int scene_aflags = RTC_INTERSECT1 | RTC_INTERSECT_STREAM | RTC_INTERPOLATE;
    RTCScene scene_out = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
    return scene_out;
  }

  void convertScene(RTCScene scene_out, ISPCScene* scene_in, RTCGeometryFlags gflags)
  {
    size_t numGeometries = scene_in->numGeometries;

    for (size_t i=0; i<numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out, gflags);
      }
      else if (geometry->type == TRIANGLE_MESH) {
        convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out, gflags);
      }
      else if (geometry->type == QUAD_MESH) {
        convertQuadMesh((ISPCQuadMesh*) geometry, scene_out, gflags);
      }
      else if (geometry->type == CURVES) {
        convertCurveGeometry((ISPCHairSet*) geometry, scene_out, gflags);
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
      if (geometry->type == SUBDIV_MESH) {
        rtcUpdate(rtcGetGeometry(scene_out,((ISPCSubdivMesh*)geometry)->geom.geomID));
      }
      else if (geometry->type == TRIANGLE_MESH) {
        rtcUpdate(rtcGetGeometry(scene_out,((ISPCTriangleMesh*)geometry)->geom.geomID));
      }
      else if (geometry->type == QUAD_MESH) {
        rtcUpdate(rtcGetGeometry(scene_out,((ISPCQuadMesh*)geometry)->geom.geomID));
      }
      else if (geometry->type == CURVES) {
        rtcUpdate(rtcGetGeometry(scene_out,((ISPCHairSet*)geometry)->geom.geomID));
      }
      else
        assert(false);
    }
  }

  void deleteObjects(ISPCScene* scene_in, RTCScene scene_out) // FIXME: geomID can be accessed easier now
  {
    size_t numGeometries = scene_in->numGeometries;
    for (size_t i=0; i<numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        rtcDetachGeometry(scene_out,((ISPCSubdivMesh*)geometry)->geom.geomID);
      }
      else if (geometry->type == TRIANGLE_MESH) {
        rtcDetachGeometry(scene_out,((ISPCTriangleMesh*)geometry)->geom.geomID);
      }
      else if (geometry->type == QUAD_MESH) {
        rtcDetachGeometry(scene_out,((ISPCQuadMesh*)geometry)->geom.geomID);
      }
      else if (geometry->type == CURVES) {
        rtcDetachGeometry(scene_out,((ISPCHairSet*)geometry)->geom.geomID);
      }
      else
        assert(false);
    }
  }

  void Benchmark_Dynamic_Update(ISPCScene* scene_in, size_t benchmark_iterations, RTCGeometryFlags gflags = RTC_GEOMETRY_DYNAMIC)
  {
    assert(g_scene == nullptr);
    g_scene = createScene(RTC_SCENE_DYNAMIC);
    convertScene(g_scene, scene_in, gflags);
    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      updateObjects(scene_in,g_scene);
      double t0 = getSeconds();
      rtcCommit (g_scene);
      double t1 = getSeconds();
      if (i >= skip_iterations)
      {
        time += t1 - t0;
        iterations++;
      }
    }

    if (gflags == RTC_GEOMETRY_STATIC)
      std::cout << "BENCHMARK_UPDATE_DYNAMIC_STATIC ";
    else if (gflags == RTC_GEOMETRY_DYNAMIC)
      std::cout << "BENCHMARK_UPDATE_DYNAMIC_DYNAMIC ";
    else if (gflags == RTC_GEOMETRY_DEFORMABLE)
      std::cout << "BENCHMARK_UPDATE_DYNAMIC_DEFORMABLE ";
    else
      FATAL("unknown flags");

    std::cout << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    rtcDeleteScene (g_scene);
    g_scene = nullptr;
  }

  void Benchmark_Dynamic_Create(ISPCScene* scene_in, size_t benchmark_iterations, RTCGeometryFlags gflags = RTC_GEOMETRY_STATIC)
  {
    assert(g_scene == nullptr);
    g_scene = createScene(RTC_SCENE_DYNAMIC);
    convertScene(g_scene, scene_in,gflags);
    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      deleteObjects(scene_in,g_scene);
      convertScene(g_scene, scene_in,gflags);
      double t0 = getSeconds();
      rtcCommit (g_scene);
      double t1 = getSeconds();
      if (i >= skip_iterations)
      {
        time += t1 - t0;
        iterations++;
      }
    }

    if (gflags == RTC_GEOMETRY_STATIC)
      std::cout << "BENCHMARK_CREATE_DYNAMIC_STATIC ";
    else if (gflags == RTC_GEOMETRY_DYNAMIC)
      std::cout << "BENCHMARK_CREATE_DYNAMIC_DYNAMIC ";
    else if (gflags == RTC_GEOMETRY_DEFORMABLE)
      std::cout << "BENCHMARK_CREATE_DYNAMIC_DEFORMABLE ";
    else
      FATAL("unknown flags");

    std::cout << primitives << " primitives, " << objects << " objects, "
              << time/iterations << " s, "
              << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    rtcDeleteScene (g_scene);
    g_scene = nullptr;
  }

  void Benchmark_Static_Create(ISPCScene* scene_in, size_t benchmark_iterations, RTCGeometryFlags gflags = RTC_GEOMETRY_STATIC, RTCSceneFlags sflags = RTC_SCENE_STATIC)
  {
    assert(g_scene == nullptr);
    size_t primitives = getNumPrimitives(scene_in);
    size_t objects = getNumObjects(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      g_scene = createScene(sflags);
      convertScene(g_scene,scene_in,gflags);

      double t0 = getSeconds();
      rtcCommit (g_scene);
      double t1 = getSeconds();
      if (i >= skip_iterations)
      {
        time += t1 - t0;
        iterations++;
      }
      rtcDeleteScene (g_scene);
    }

    if (sflags & RTC_SCENE_HIGH_QUALITY)
      std::cout << "BENCHMARK_CREATE_HQ_STATIC_";
    else
      std::cout << "BENCHMARK_CREATE_STATIC_";

    if (gflags == RTC_GEOMETRY_STATIC)
      std::cout << "STATIC ";
    else if (gflags == RTC_GEOMETRY_DYNAMIC)
      std::cout << "DYNAMIC ";
    else if (gflags == RTC_GEOMETRY_DEFORMABLE)
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
    std::string init("start_threads=1,affinity=1");
    if (cfg)
    {
      init += ",";
      init += cfg;
    }
    /* create new Embree device */
    g_device = rtcNewDevice(init.c_str());
    error_handler(nullptr,rtcDeviceGetError(g_device));

    /* set error handler */
    rtcDeviceSetErrorFunction(g_device,error_handler,nullptr);
    Benchmark_Dynamic_Update(g_ispc_scene,iterations_dynamic_dynamic,RTC_GEOMETRY_DEFORMABLE);
    Pause();
    Benchmark_Dynamic_Update(g_ispc_scene,iterations_dynamic_dynamic,RTC_GEOMETRY_DYNAMIC);
    Pause();
    Benchmark_Dynamic_Update(g_ispc_scene,iterations_dynamic_static ,RTC_GEOMETRY_STATIC);
    Pause();
    Benchmark_Dynamic_Create(g_ispc_scene,iterations_dynamic_dynamic,RTC_GEOMETRY_DEFORMABLE);
    Pause();
    Benchmark_Dynamic_Create(g_ispc_scene,iterations_dynamic_dynamic,RTC_GEOMETRY_DYNAMIC);
    Pause();
    Benchmark_Dynamic_Create(g_ispc_scene,iterations_dynamic_static ,RTC_GEOMETRY_STATIC);
    Pause();
    Benchmark_Static_Create(g_ispc_scene,iterations_static_static,RTC_GEOMETRY_STATIC,RTC_SCENE_STATIC);
    Pause();
    Benchmark_Static_Create(g_ispc_scene,iterations_static_static,RTC_GEOMETRY_STATIC,RTC_SCENE_HIGH_QUALITY);
    rtcDeleteDevice(g_device); g_device = nullptr;
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
