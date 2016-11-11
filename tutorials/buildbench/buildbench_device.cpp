// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

  static const size_t skip_iterations            = 5;
  static const size_t iterations_dynamic_dynamic = 200;
  static const size_t iterations_dynamic_static  = 50;
  static const size_t iterations_static_static   = 30;

  extern "C" ISPCScene* g_ispc_scene;

/* scene data */
  RTCDevice g_device = nullptr;
  RTCScene g_scene = nullptr;

  unsigned int convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out, RTCGeometryFlags flags)
  {
    unsigned int geomID = rtcNewTriangleMesh (scene_out, flags, mesh->numTriangles, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
    mesh->geomID = geomID;
    return geomID;
  }

  unsigned int convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out, RTCGeometryFlags flags)
  {
    unsigned int geomID = rtcNewQuadMesh (scene_out, flags, mesh->numQuads, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
    mesh->geomID = geomID;
    return geomID;
  }

  unsigned int convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out, RTCGeometryFlags flags)
  {
    unsigned int geomID = rtcNewSubdivisionMesh(scene_out,flags, mesh->numFaces, mesh->numEdges, mesh->numVertices,
                                                mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles, mesh->numTimeSteps);
    mesh->geomID = geomID;
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 16.0f;
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa  ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float));
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float));
    return geomID;
  }

  unsigned int convertLineSegments(ISPCLineSegments* mesh, RTCScene scene_out, RTCGeometryFlags flags)
  {
    unsigned int geomID = rtcNewLineSegments (scene_out, flags, mesh->numSegments, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
    return geomID;
  }

  unsigned int convertHairSet(ISPCHairSet* hair, RTCScene scene_out, RTCGeometryFlags flags)
  {
    unsigned int geomID = rtcNewHairGeometry (scene_out, flags, hair->numHairs, hair->numVertices, hair->numTimeSteps);
    for (size_t t=0; t<hair->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),hair->positions+t*hair->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
    return geomID;
  }

  unsigned int convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out, RTCGeometryFlags flags)
  {
    unsigned int geomID = rtcNewCurveGeometry (scene_out, flags, hair->numHairs, hair->numVertices, hair->numTimeSteps);
    for (size_t t=0; t<hair->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),hair->positions+t*hair->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
    return geomID;
  }

  RTCScene convertScene(ISPCScene* scene_in, RTCSceneFlags sflags, RTCGeometryFlags gflags)
  {
    size_t numGeometries = scene_in->numGeometries;
    RTCSceneFlags scene_flags = sflags | RTC_SCENE_INCOHERENT;
    int scene_aflags = RTC_INTERSECT1 | RTC_INTERSECT_STREAM | RTC_INTERPOLATE;
    RTCScene scene_out = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
    for (size_t i=0; i<numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        unsigned int geomID MAYBE_UNUSED = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out, gflags);
        ((ISPCSubdivMesh*)geometry)->geomID = geomID;
        assert(geomID == i);
      }
      else if (geometry->type == TRIANGLE_MESH) {
        unsigned int geomID MAYBE_UNUSED = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out, gflags);
        ((ISPCTriangleMesh*)geometry)->geomID = geomID;
        assert(geomID == i);
      }
      else if (geometry->type == QUAD_MESH) {
        unsigned int geomID MAYBE_UNUSED = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out, gflags);
        ((ISPCQuadMesh*)geometry)->geomID = geomID;
        assert(geomID == i);
      }
      else if (geometry->type == LINE_SEGMENTS) {
        unsigned int geomID MAYBE_UNUSED = convertLineSegments((ISPCLineSegments*) geometry, scene_out, gflags);
        ((ISPCLineSegments*)geometry)->geomID = geomID;
        assert(geomID == i);
      }
      else if (geometry->type == HAIR_SET) {
        unsigned int geomID MAYBE_UNUSED = convertHairSet((ISPCHairSet*) geometry, scene_out, gflags);
        ((ISPCHairSet*)geometry)->geomID = geomID;
        assert(geomID == i);
      }
      else if (geometry->type == CURVES) {
        unsigned int geomID MAYBE_UNUSED = convertCurveGeometry((ISPCHairSet*) geometry, scene_out, gflags);
        ((ISPCHairSet*)geometry)->geomID = geomID;
        assert(geomID == i);
      }
      else
        assert(false);
    }
    return scene_out;
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
      else if (geometry->type == LINE_SEGMENTS) {
        numPrimitives += ((ISPCLineSegments*)geometry)->numSegments;
      }
      else if (geometry->type == HAIR_SET) {
        numPrimitives += ((ISPCHairSet*)geometry)->numHairs;
      }
      else if (geometry->type == CURVES) {
        numPrimitives += ((ISPCHairSet*)geometry)->numHairs;
      }
      else
        assert(false);
    }
    return numPrimitives;
  }

  void updateScene(ISPCScene* scene_in, RTCScene scene_out)
  {
    size_t numGeometries = scene_in->numGeometries;
    for (size_t i=0; i<numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        rtcUpdate(scene_out,((ISPCSubdivMesh*)geometry)->geomID);
      }
      else if (geometry->type == TRIANGLE_MESH) {
        rtcUpdate(scene_out,((ISPCTriangleMesh*)geometry)->geomID);
      }
      else if (geometry->type == QUAD_MESH) {
        rtcUpdate(scene_out,((ISPCQuadMesh*)geometry)->geomID);
      }
      else if (geometry->type == LINE_SEGMENTS) {
        rtcUpdate(scene_out,((ISPCLineSegments*)geometry)->geomID);
      }
      else if (geometry->type == HAIR_SET) {
        rtcUpdate(scene_out,((ISPCHairSet*)geometry)->geomID);
      }
      else if (geometry->type == CURVES) {
        rtcUpdate(scene_out,((ISPCHairSet*)geometry)->geomID);
      }
      else
        assert(false);
    }
  }

  void Benchmark_DynamicDynamic(ISPCScene* scene_in, size_t benchmark_iterations)
  {
    assert(g_scene == nullptr);
    g_scene = convertScene(scene_in,RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC);
    size_t primitives = getNumPrimitives(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      updateScene(scene_in,g_scene);
      double t0 = getSeconds();
      rtcCommit (g_scene);
      double t1 = getSeconds();
      if (i >= skip_iterations)
      {
        time += t1 - t0;      
        iterations++;
      }
    }
    std::cout << "Update dynamic scene, dynamic geometry " 
              << "(" << primitives << " primitives)  :  "
              << " avg. time  = " <<  time/iterations 
              << " , avg. build perf " << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    rtcDeleteScene (g_scene); 
    g_scene = nullptr;    
  }


  void Benchmark_DynamicStatic(ISPCScene* scene_in, size_t benchmark_iterations)
  {
    assert(g_scene == nullptr);
    g_scene = convertScene(scene_in,RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC);
    size_t primitives = getNumPrimitives(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      updateScene(scene_in,g_scene);
      double t0 = getSeconds();
      rtcCommit (g_scene);
      double t1 = getSeconds();
      if (i >= skip_iterations)
      {
        time += t1 - t0;      
        iterations++;
      }
    }
    std::cout << "Update dynamic scene, static geometry " 
              << "(" << primitives << " primitives)  :  "
              << " avg. time  = " <<  time/iterations 
              << " , avg. build perf " << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    rtcDeleteScene (g_scene); 
    g_scene = nullptr;    
  }

  void Benchmark_StaticStatic(ISPCScene* scene_in, size_t benchmark_iterations)
  {
    assert(g_scene == nullptr);
    size_t primitives = getNumPrimitives(scene_in);
    size_t iterations = 0;
    double time = 0.0;
    for(size_t i=0;i<benchmark_iterations+skip_iterations;i++)
    {
      g_scene = convertScene(scene_in,RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC);
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
    std::cout << "Create static scene, static geometry " 
              << " (" << primitives << " primitives)  :  "
              << " avg. time  = " <<  time/iterations 
              << " , avg. build perf " << 1.0 / (time/iterations) * primitives / 1000000.0 << " Mprims/s" << std::endl;

    g_scene = nullptr;    
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
    error_handler(rtcDeviceGetError(g_device));

    /* set error handler */
    rtcDeviceSetErrorFunction(g_device,error_handler);

    Benchmark_DynamicDynamic(g_ispc_scene,iterations_dynamic_dynamic);
    Benchmark_DynamicStatic(g_ispc_scene,iterations_dynamic_static);
    Benchmark_StaticStatic(g_ispc_scene,iterations_static_static);

    rtcDeleteDevice(g_device); g_device = nullptr;
    //exit(0);
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
