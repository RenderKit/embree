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

#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

#define RAYN_FLAGS RTC_INTERSECT_COHERENT
#define ANIM_FPS 15.0f

  extern "C" ISPCScene* g_ispc_scene;

/* scene data */
  RTCDevice g_device = nullptr;
  RTCScene g_scene   = nullptr;

/* animation data */
  size_t frameID     = 0;
  double animTime        = -1.0f; // global time counter
  unsigned int staticID  = 0;
  unsigned int dynamicID = 0;

  std::vector<double> buildTime;
  std::vector<double> vertexUpdateTime;
  std::vector<double> renderTime;
  bool printStats = false;
  bool timeInitialized = false;

  static const size_t numProfileFrames = 200;

  void dumpBuildAndRenderTimes();

  void device_key_pressed_handler(int key)
  {
    if (key == 100 /*d*/) { 
      std::cout << "dumping build and render times per frame [" << buildTime.size() << " frames]..." << std::flush;
      dumpBuildAndRenderTimes(); 
      std::cout << "done" << std::endl;
    }
    else if (key == 115 /*s*/) { 
      printStats = !printStats; 
    }
    else device_key_pressed_default(key);
  }

  // ==================================================================================================
  // ==================================================================================================
  // ==================================================================================================


  unsigned int convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewTriangleMesh (scene_out, object_flags, mesh->numTriangles, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      //rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
      Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t));
      for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = *(mesh->positions+t*mesh->numVertices+i);        
      rtcUnmapBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
    mesh->geomID = geomID;
    return geomID;
  }


  unsigned int convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewQuadMesh (scene_out, object_flags, mesh->numQuads, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
    mesh->geomID = geomID;
    return geomID;
  }

  unsigned int convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewSubdivisionMesh(scene_out, object_flags, mesh->numFaces, mesh->numEdges, mesh->numVertices,
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

  unsigned int convertLineSegments(ISPCLineSegments* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewLineSegments (scene_out, object_flags, mesh->numSegments, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
    return geomID;
  }

  unsigned int convertHairSet(ISPCHairSet* hair, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewHairGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps);
    for (size_t t=0; t<hair->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),hair->positions+t*hair->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
    return geomID;
  }

  unsigned int convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewCurveGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps);
    for (size_t t=0; t<hair->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),hair->positions+t*hair->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
    return geomID;
  }

  size_t getNumObjects(ISPCScene* scene_in)
  {
    return scene_in->numGeometries;
  }

  RTCScene createScene(ISPCScene* scene_in, bool dynamic = false)
  {
    int scene_flags = RTC_SCENE_INCOHERENT | (dynamic ? RTC_SCENE_DYNAMIC : RTC_SCENE_STATIC);
    int scene_aflags = RTC_INTERSECT1 | RTC_INTERSECT_STREAM | RTC_INTERPOLATE;
    return rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
  }
    

  unsigned int createObject(const size_t i, ISPCScene* scene_in, RTCScene scene_out, bool dynamic = false)
  {
    RTCGeometryFlags object_flags = dynamic ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;

    ISPCGeometry* geometry = scene_in->geometries[i];
    unsigned int geomID = 0;

    if (geometry->type == SUBDIV_MESH) {
      geomID = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out, object_flags);
      ((ISPCSubdivMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      geomID = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out, object_flags);
      ((ISPCTriangleMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == QUAD_MESH) {
      geomID = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out, object_flags);
      ((ISPCQuadMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      geomID = convertLineSegments((ISPCLineSegments*) geometry, scene_out, object_flags);
      ((ISPCLineSegments*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == HAIR_SET) {
      geomID = convertHairSet((ISPCHairSet*) geometry, scene_out, object_flags);
      ((ISPCHairSet*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == CURVES) {
      geomID = convertCurveGeometry((ISPCHairSet*) geometry, scene_out, object_flags);
      ((ISPCHairSet*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else
      assert(false);
    return geomID;
  }

  void updateVertexData(const unsigned int geomID, ISPCScene* scene_in, RTCScene scene_out, size_t keyFrameID, const float t)
  {
    size_t numGeometries = scene_in->numGeometries;
    if (!numGeometries) return;

    ISPCGeometry* geometry0 = scene_in->geometries[max((keyFrameID+0) % numGeometries,(size_t)1)];
    ISPCGeometry* geometry1 = scene_in->geometries[max((keyFrameID+1) % numGeometries,(size_t)1)];

    /* FIXME: only updating triangle meshes works so far */

    if (geometry0->type == SUBDIV_MESH) {
      unsigned int geomID = ((ISPCSubdivMesh*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == TRIANGLE_MESH) {
      ISPCTriangleMesh* mesh0 = (ISPCTriangleMesh*)geometry0;
      ISPCTriangleMesh* mesh1 = (ISPCTriangleMesh*)geometry1;

      assert(mesh0->numTimeSteps == mesh1->numTimeSteps);
      assert(mesh0->numVertices  == mesh1->numVertices);

      for (size_t t=0; t<mesh0->numTimeSteps; t++) {
        //rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
        Vec3fa* __restrict__ vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t));
        const Vec3fa* __restrict__ const input0 = mesh0->positions+t*mesh0->numVertices;
        const Vec3fa* __restrict__ const input1 = mesh1->positions+t*mesh1->numVertices;

        parallel_for(size_t(0),size_t(mesh0->numVertices),[&](const range<size_t>& range) {
            for (size_t i=range.begin(); i<range.end(); i++)
              vertices[i] = lerp(input0[i],input1[i],t);
          }); 

        rtcUnmapBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t));

      }
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == QUAD_MESH) {
      unsigned int geomID = ((ISPCQuadMesh*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == LINE_SEGMENTS) {
      unsigned int geomID = ((ISPCLineSegments*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == HAIR_SET) {
      unsigned int geomID = ((ISPCHairSet*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == CURVES) {
      unsigned int geomID = ((ISPCHairSet*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else
      assert(false);
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
    const unsigned int tileY = taskIndex / numTilesX;
    const unsigned int tileX = taskIndex - tileY * numTilesX;
    const unsigned int x0 = tileX * TILE_SIZE_X;
    const unsigned int x1 = min(x0+TILE_SIZE_X,width);
    const unsigned int y0 = tileY * TILE_SIZE_Y;
    const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

    RTCRay rays[TILE_SIZE_X*TILE_SIZE_Y];

    /* generate stream of primary rays */
    int N = 0;
    for (unsigned int y=y0; y<y1; y++) 
      for (unsigned int x=x0; x<x1; x++)
      {
        /* ISPC workaround for mask == 0 */
    

        RandomSampler sampler;
        RandomSampler_init(sampler, x, y, 0);

        /* initialize ray */
        RTCRay& ray = rays[N++];

        ray.org = Vec3fa(camera.xfm.p);
        ray.dir = Vec3fa(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz));
        bool mask = 1; { // invalidates inactive rays
          ray.tnear = mask ? 0.0f         : (float)(pos_inf);
          ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
        }
        ray.geomID = RTC_INVALID_GEOMETRY_ID;
        ray.primID = RTC_INVALID_GEOMETRY_ID;
        ray.mask = -1;
        ray.time = RandomSampler_get1D(sampler);
      }

    RTCIntersectContext context;
    context.flags = RAYN_FLAGS;

    /* trace stream of rays */
    rtcIntersect1M(g_scene,&context,rays,N,sizeof(RTCRay));

    /* shade stream of rays */
    N = 0;
    for (unsigned int y=y0; y<y1; y++) 
      for (unsigned int x=x0; x<x1; x++)
      {
        /* ISPC workaround for mask == 0 */
    
        RTCRay& ray = rays[N++];

        /* eyelight shading */
        Vec3fa color = Vec3fa(0.0f);
        if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
          color = Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))));

        /* write color to framebuffer */
        unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
        unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
        unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
        pixels[y*width+x] = (b << 16) + (g << 8) + r;
      }
  }

/* task that renders a single screen tile */
  void renderTileTask (int taskIndex, int* pixels,
                       const unsigned int width,
                       const unsigned int height,
                       const float time,
                       const ISPCCamera& camera,
                       const int numTilesX,
                       const int numTilesY)
  {
    renderTile(taskIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }

/* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    /* create new Embree device */
    g_device = rtcNewDevice(cfg);
    error_handler(rtcDeviceGetError(g_device));

    /* set error handler */
    rtcDeviceSetErrorFunction(g_device,error_handler);


    /* create scene */
    g_scene = createScene(g_ispc_scene,true);

    /* instantiate the first two objects (static,dynamic) */
#if 0
    /* num objects */
    size_t numObjects = getNumObjects(g_ispc_scene);
    PRINT(numObjects);

    for (size_t i=0;i<numObjects;i++)
      createObject(i,g_ispc_scene,g_scene,true);
#else
    staticID  = createObject(0,g_ispc_scene,g_scene,false);
    dynamicID = createObject(1,g_ispc_scene,g_scene,true);
    PRINT(staticID);
    PRINT(dynamicID);

#endif

    rtcCommit (g_scene);

    if (!timeInitialized)
    {
      timeInitialized = true;

      buildTime.resize(numProfileFrames);
      renderTime.resize(numProfileFrames);
      vertexUpdateTime.resize(numProfileFrames);

      for (size_t i=0;i<numProfileFrames;i++)
      {
        buildTime[i] = 0.0;
        renderTime[i] = 0.0;
        vertexUpdateTime[i] = 0.0;
      }
      
    }

    /* set render tile function to use */
    renderTile = renderTileStandard;
    key_pressed_handler = device_key_pressed_handler;
  }


  __forceinline void updateTimeLog(std::vector<double> &times, double time)
  {
    if (times[frameID] > 0.0f) 
      times[frameID] = (times[frameID] + time) * 0.5f;
    else
      times[frameID] = time;    
  }

/* called by the C++ code to render */
  extern "C" void device_render (int* pixels,
                                 const unsigned int width,
                                 const unsigned int height,
                                 const float time,
                                 const ISPCCamera& camera)
  {
    /* ============ */
    /* render image */
    /* ============ */

    double renderTime0 = getSeconds();
    const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
    const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
    parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
        for (size_t i=range.begin(); i<range.end(); i++)
          renderTileTask((int)i,pixels,width,height,time,camera,numTilesX,numTilesY);
      }); 
    double renderTime1 = getSeconds();
    assert(frameID < renderTime.size());
    assert(frameID < buildTime.size());

    updateTimeLog(renderTime,renderTime1-renderTime0);

    if (unlikely(printStats)) std::cout << "rendering frame in " << renderTime1-renderTime0 << " ms" << std::endl;

    /* =============== */
    /* update geometry */
    /* =============== */

    double vertexUpdateTime0 = getSeconds();    

    if (animTime < 0.0f) animTime = getSeconds();
    const double atime = (getSeconds() - animTime) * ANIM_FPS;
    double intpart, fracpart;
    fracpart = modf(atime,&intpart);
    const size_t keyFrameID = ((size_t)intpart) % numProfileFrames;    
    updateVertexData(dynamicID, g_ispc_scene, g_scene, keyFrameID, (float)fracpart);

    double vertexUpdateTime1 = getSeconds();    
    updateTimeLog(vertexUpdateTime,vertexUpdateTime1-vertexUpdateTime0);

    /* =========== */
    /* rebuild bvh */
    /* =========== */
    
    double buildTime0 = getSeconds();
    rtcCommit(g_scene);      
    double buildTime1 = getSeconds();

    updateTimeLog(buildTime,buildTime1-buildTime0);

    if (unlikely(printStats)) std::cout << "bvh rebuild in " << buildTime1-buildTime0 << " ms" << std::endl;
    
    frameID = (frameID + 1) % numProfileFrames;
  }

/* plot build and render times */

  void dumpBuildAndRenderTimes()
  {
    FileName name("buildRenderTimes");
    std::fstream plot;
    plot.open(name.addExt(".plot"), std::fstream::out | std::fstream::trunc);

    plot << "set terminal png size 2048,600 enhanced" << std::endl;
    plot << "set output \"" << name.addExt(".png") << "\"" << std::endl;
    plot << "set key inside right top vertical Right noreverse enhanced autotitles box linetype -1 linewidth 1.000" << std::endl;
    plot << "set ylabel \"" << "ms" << "\"" << std::endl;
    plot << "set yrange [0:20]" << std::endl;
    plot << "set ytics 1" << std::endl;
    plot << "factor=1000" << std::endl;
    plot << "plot \"-\" using ($1):(factor*($2)) title \"build time\" with linespoints lw 4,\"-\" using ($1):(factor*($2)) title \"vertex update time\" with linespoints lw 4,\"-\" using ($1):(factor*($2)) title \"render time\" with linespoints lw 4,\"-\" using ($1):(factor*($2)) title \"total time\" with linespoints lw 4" << std::endl;
    for (size_t i=0;i<buildTime.size();i++)
      plot << i << " " << buildTime[i] << std::endl;
    plot << "e" << std::endl;
    for (size_t i=0;i<vertexUpdateTime.size();i++)
      plot << i << " " << vertexUpdateTime[i] << std::endl;
    plot << "e" << std::endl;
    for (size_t i=0;i<renderTime.size();i++)
      plot << i << " " << renderTime[i] << std::endl;
    plot << "e" << std::endl;
    for (size_t i=0;i<renderTime.size();i++)
      plot << i << " " << buildTime[i] + renderTime[i] + vertexUpdateTime[i] << std::endl;
    plot << std::endl;
    plot.close();
  }

/* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
    rtcDeleteScene (g_scene); g_scene = nullptr;
    rtcDeleteDevice(g_device); g_device = nullptr;
    /* dump data at the end of profiling */
    std::cout << "dumping build and render times per frame [" << numProfileFrames << " frames]..." << std::flush;
    dumpBuildAndRenderTimes(); 
    std::cout << "done" << std::endl;
  }

} // namespace embree
