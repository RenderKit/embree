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

#define ANIM_FPS 15.0f
#define ENABLE_ANIM 1
#define VERTEX_NORMALS 0
#define SHADOWS 1
#define DUMP_PROFILE_DATA 0

#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"


namespace embree {

  extern "C" ISPCScene* g_ispc_scene;

  /* scene data */
  RTCDevice g_device   = nullptr;
  RTCScene g_scene     = nullptr;
  Vec3fa *ls_positions = nullptr;

  /* animation data */
  size_t frameID         = 0;
  double animTime        = -1.0f; // global time counter

  /* profile data */
  std::vector<double> buildTime;
  std::vector<double> vertexUpdateTime;
  std::vector<double> renderTime;
  bool printStats = false;
  bool timeInitialized = false;

  static const size_t numProfileFrames = 200;

  /* shadow distance map */

#if DUMP_PROFILE_DATA == 1
  void dumpBuildAndRenderTimes();
#endif

  void device_key_pressed_handler(int key)
  {
    if (key == 111 /*o*/) {
#if DUMP_PROFILE_DATA == 1
      std::cout << "dumping build and render times per frame [" << buildTime.size() << " frames]..." << std::flush;
      dumpBuildAndRenderTimes();
      std::cout << "done" << std::endl;
#endif
    }
    else if (key == 112 /*p*/) {
      printStats = !printStats;
    }
    else device_key_pressed_default(key);
  }

  // ==================================================================================================
  // ==================================================================================================
  // ==================================================================================================


  unsigned int convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    RTCGeometryFlags object_flags = mesh->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    unsigned int geomID = rtcNewTriangleMesh (scene_out, object_flags, mesh->numTriangles, mesh->numVertices, 1);
    /* generate vertex buffer */
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
    mesh->geom.geomID = geomID;
    return geomID;
  }


  unsigned int convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    RTCGeometryFlags object_flags = mesh->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    unsigned int geomID = rtcNewQuadMesh (scene_out, object_flags, mesh->numQuads, mesh->numVertices, mesh->numTimeSteps);
    /* generate vertex buffer */
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
    mesh->geom.geomID = geomID;
    return geomID;
  }

  unsigned int convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    RTCGeometryFlags object_flags = mesh->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    unsigned int geomID = rtcNewSubdivisionMesh(scene_out, object_flags, mesh->numFaces, mesh->numEdges, mesh->numVertices,
                                                mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles, mesh->numTimeSteps);
    mesh->geom.geomID = geomID;
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 4.0f;
    /* generate vertex buffer */
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set all other buffers */
    rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float));
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float));
    rtcSetSubdivisionMode(scene_out, geomID, 0, mesh->position_subdiv_mode);
    return geomID;
  }

  unsigned int convertLineSegments(ISPCLineSegments* mesh, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    RTCGeometryFlags object_flags = mesh->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    unsigned int geomID = rtcNewLineSegments (scene_out, object_flags, mesh->numSegments, mesh->numVertices, mesh->numTimeSteps);
    /* generate vertex buffer */
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
    return geomID;
  }

  unsigned int convertHairSet(ISPCHairSet* hair, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    RTCGeometryFlags object_flags = hair->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    unsigned int geomID = 0;
    switch (hair->basis) {
    case BEZIER_BASIS : geomID = rtcNewBezierHairGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps); break;
    case BSPLINE_BASIS: geomID = rtcNewBSplineHairGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps); break;
    default: assert(false);
    }
    /* generate vertex buffer */
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<hair->numVertices;i++) vertices[i] = hair->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
    rtcSetTessellationRate(scene_out,geomID,(float)hair->tessellation_rate);
    return geomID;
  }

  unsigned int convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    RTCGeometryFlags object_flags = hair->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    unsigned int geomID = 0;
    switch (hair->basis) {
    case BEZIER_BASIS : geomID = rtcNewBezierCurveGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps); break;
    case BSPLINE_BASIS: geomID = rtcNewBSplineCurveGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps); break;
    default: assert(false);
    }
    /* generate vertex buffer */
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<hair->numVertices;i++) vertices[i] = hair->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
    return geomID;
  }

  size_t getNumObjects(ISPCScene* scene_in)
  {
    return scene_in->numGeometries;
  }

  RTCScene createScene(ISPCScene* scene_in)
  {
    int scene_flags = RTC_SCENE_INCOHERENT | RTC_SCENE_DYNAMIC;
    int scene_aflags = RTC_INTERSECT1 | RTC_INTERSECT_STREAM | RTC_INTERPOLATE;
    return rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
  }


  void createObject(const size_t i, ISPCScene* scene_in, RTCScene scene_out)
  {
    ISPCGeometry* geometry = scene_in->geometries[i];
    unsigned int geomID = 0;

    if (geometry->type == SUBDIV_MESH) {
      geomID = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out);
      ((ISPCSubdivMesh*)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      geomID = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out);
      ((ISPCTriangleMesh*)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == QUAD_MESH) {
      geomID = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out);
      ((ISPCQuadMesh*)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      geomID = convertLineSegments((ISPCLineSegments*) geometry, scene_out);
      ((ISPCLineSegments*)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == HAIR_SET) {
      geomID = convertHairSet((ISPCHairSet*) geometry, scene_out);
      ((ISPCHairSet*)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == CURVES) {
      geomID = convertCurveGeometry((ISPCHairSet*) geometry, scene_out);
      ((ISPCHairSet*)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else
      assert(false);
  }

  void interpolateVertices(RTCScene scene_out,
                           const unsigned int geomID,
                           const size_t numVertices,
                           const Vec3fa* __restrict__ const input0,
                           const Vec3fa* __restrict__ const input1,
                           const float tt)
  {
    Vec3fa* __restrict__ vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    parallel_for(size_t(0),numVertices,[&](const range<size_t>& range) {
        for (size_t i=range.begin(); i<range.end(); i++)
          vertices[i] = lerp(input0[i],input1[i],tt);
      });
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    rtcUpdate(scene_out,geomID);
  }

  void updateVertexData(const unsigned int ID,
                        ISPCScene* scene_in,
                        RTCScene scene_out,
                        const size_t keyFrameID,
                        const float tt)
  {
    ISPCGeometry* geometry = scene_in->geometries[ID];

    if (geometry->type == SUBDIV_MESH) {
      unsigned int geomID = ((ISPCSubdivMesh*)geometry)->geom.geomID;
      /* if static do nothing */
      if (((ISPCSubdivMesh*)geometry)->numTimeSteps <= 1) return;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      ISPCTriangleMesh* mesh = (ISPCTriangleMesh*)geometry;
      /* if static do nothing */
      if (mesh->numTimeSteps <= 1) return;
      /* interpolate two vertices from two timesteps */
      const size_t t0 = (keyFrameID+0) % mesh->numTimeSteps;
      const size_t t1 = (keyFrameID+1) % mesh->numTimeSteps;
      const Vec3fa* __restrict__ const input0 = mesh->positions[t0];
      const Vec3fa* __restrict__ const input1 = mesh->positions[t1];
      interpolateVertices(scene_out, mesh->geom.geomID, mesh->numVertices, input0, input1, tt);
    }
    else if (geometry->type == QUAD_MESH) {
      ISPCQuadMesh* mesh = (ISPCQuadMesh*)geometry;
      /* if static do nothing */
      if (mesh->numTimeSteps <= 1) return;
      /* interpolate two vertices from two timesteps */
      const size_t t0 = (keyFrameID+0) % mesh->numTimeSteps;
      const size_t t1 = (keyFrameID+1) % mesh->numTimeSteps;
      const Vec3fa* __restrict__ const input0 = mesh->positions[t0];
      const Vec3fa* __restrict__ const input1 = mesh->positions[t1];
      interpolateVertices(scene_out, mesh->geom.geomID, mesh->numVertices, input0, input1, tt);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      unsigned int geomID = ((ISPCLineSegments*)geometry)->geom.geomID;
      /* if static do nothing */
      if (((ISPCLineSegments*)geometry)->numTimeSteps <= 1) return;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == HAIR_SET) {
      unsigned int geomID = ((ISPCHairSet*)geometry)->geom.geomID;
      /* if static do nothing */
      if (((ISPCHairSet*)geometry)->numTimeSteps <= 1) return;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == CURVES) {
      unsigned int geomID = ((ISPCHairSet*)geometry)->geom.geomID;
      /* if static do nothing */
      if (((ISPCHairSet*)geometry)->numTimeSteps <= 1) return;
      rtcUpdate(scene_out,geomID);
    }
    else
      assert(false);
  }

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
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

    RayStats& stats = g_stats[threadIndex];

    RTCRay rays[TILE_SIZE_X*TILE_SIZE_Y];

    /* generate stream of primary rays */
    int N = 0;
    for (unsigned int y=y0; y<y1; y++)
      for (unsigned int x=x0; x<x1; x++)
      {
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
        ray.time = 0.0f;
        RayStats_addRay(stats);
      }

    RTCIntersectContext context;
    context.flags = g_iflags_coherent;

    /* trace stream of rays */
    rtcIntersect1M(g_scene,&context,rays,N,sizeof(RTCRay));

    /* shade stream of rays */
    Vec3fa colors[TILE_SIZE_X*TILE_SIZE_Y];
    N = 0;
    for (unsigned int y=y0; y<y1; y++)
      for (unsigned int x=x0; x<x1; x++,N++)
      {
        /* ISPC workaround for mask == 0 */
        RTCRay& ray = rays[N];
        Vec3fa Ng = ray.Ng;

        /* shading */
        Vec3fa& color = colors[N];
        color = Vec3fa(1.0f,1.0f,1.0f);
        if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
        {
          /* vertex normals */
          ISPCGeometry* geometry = g_ispc_scene->geometries[ray.geomID];
          if (likely(geometry->type == TRIANGLE_MESH))
          {
#if VERTEX_NORMALS == 1
            ISPCTriangleMesh* mesh = (ISPCTriangleMesh*) geometry;
            if (likely(mesh->normals))
            {
              ISPCTriangle* tri = &mesh->triangles[ray.primID];

              const Vec3fa n0 = mesh->normals[tri->v0];
              const Vec3fa n1 = mesh->normals[tri->v1];
              const Vec3fa n2 = mesh->normals[tri->v2];
              Ng = (1.0f-ray.u-ray.v)*n0 + ray.u*n1 + ray.v*n2;
            }
#endif
          }
          /* final color */
          color = Vec3fa(abs(dot(ray.dir,normalize(Ng))));
        }
      }


#if SHADOWS == 1
    /* do some hard shadows to point lights */
    if (g_ispc_scene->numLights)
    {
      for (unsigned int i=0; i<g_ispc_scene->numLights; i++)
      {
        /* init shadow/occlusion rays */
        for (int n=0;n<N;n++)
        {
          RTCRay& ray = rays[n];
          const bool valid = ray.geomID != RTC_INVALID_GEOMETRY_ID;
          const Vec3fa hitpos = ray.org + ray.tfar*ray.dir;
          ray.org = ls_positions[i];
          ray.dir = hitpos - ray.org;
          ray.tnear = 1E-4f;
          ray.tfar  = valid ? 0.99f : -1.0f;
          ray.geomID = RTC_INVALID_GEOMETRY_ID;
          ray.primID = RTC_INVALID_GEOMETRY_ID;
          ray.mask = 0;
          ray.time = 0.0f;
          RayStats_addShadowRay(stats);
        }
        /* trace shadow rays */
        rtcOccluded1M(g_scene,&context,rays,N,sizeof(RTCRay));

        /* modify pixel color based on occlusion */
        for (int n=0;n<N;n++)
          if (rays[n].geomID != RTC_INVALID_GEOMETRY_ID)
            colors[n] *= 0.1f;

      }
    }
#endif

    /* write colors to framebuffer */
    N = 0;
    for (unsigned int y=y0; y<y1; y++)
      for (unsigned int x=x0; x<x1; x++,N++)
      {
        Vec3fa& color = colors[N];
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
    renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }

/* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    /* create new Embree device */
    g_device = rtcNewDevice(cfg);
    error_handler(nullptr,rtcDeviceGetError(g_device));

    /* set error handler */
    rtcDeviceSetErrorFunction2(g_device,error_handler,nullptr);

    /* create scene */
    g_scene = createScene(g_ispc_scene);

    /* create objects */
    size_t numObjects = getNumObjects(g_ispc_scene);
    PRINT(numObjects);
    for (size_t i=0;i<numObjects;i++)
      createObject(i,g_ispc_scene,g_scene);

    /* commit the scene */
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
    assert(frameID < renderTime.size());
    assert(frameID < vertexUpdateTime.size());
    assert(frameID < buildTime.size());

    /* =================================== */
    /* samples LS positions as pointlights */
    /* =================================== */

    if (g_ispc_scene->numLights)
    {
      if (ls_positions == nullptr) ls_positions = new Vec3fa[g_ispc_scene->numLights];
      DifferentialGeometry dg;
      dg.geomID = 0;
      dg.primID = 0;
      dg.u = 0.0f;
      dg.v = 0.0f;
      dg.P  = Vec3fa(0.0f,0.0f,0.0f);
      dg.Ng = Vec3fa(0.0f,0.0f,0.0f);
      dg.Ns = dg.Ng;
      for (size_t i=0; i<g_ispc_scene->numLights; i++)
      {
        const Light* l = g_ispc_scene->lights[i];
        Light_SampleRes ls = l->sample(l,dg,Vec2f(0.0f,0.0f));
        ls_positions[i] = ls.dir * ls.dist;
      }
    }

    /* ============ */
    /* render image */
    /* ============ */

    const double renderTime0 = getSeconds();
    const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
    const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;

    parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
        const int threadIndex = (int)TaskScheduler::threadIndex();
        for (size_t i=range.begin(); i<range.end(); i++)
          renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
      });

    const double renderTime1 = getSeconds();
    const double renderTimeDelta = renderTime1-renderTime0;

    updateTimeLog(renderTime,renderTimeDelta);

    if (unlikely(printStats)) std::cout << "rendering frame in : " << renderTimeDelta << " ms" << std::endl;


    /* =============== */
    /* update geometry */
    /* =============== */

#if ENABLE_ANIM == 1

    double vertexUpdateTime0 = getSeconds();

    if (animTime < 0.0f) animTime = getSeconds();
    const double atime = (getSeconds() - animTime) * ANIM_FPS;
    const size_t intpart = (size_t)floor(atime);
    const double fracpart = atime - (double)intpart;
    const size_t keyFrameID = intpart;

    size_t numObjects = getNumObjects(g_ispc_scene);
    for (unsigned int i=0;i<numObjects;i++)
      updateVertexData(i, g_ispc_scene, g_scene, keyFrameID, (float)fracpart);

    double vertexUpdateTime1 = getSeconds();
    const double vertexUpdateTimeDelta = vertexUpdateTime1-vertexUpdateTime0;

    updateTimeLog(vertexUpdateTime,vertexUpdateTimeDelta);

    if (unlikely(printStats)) std::cout << "vertex update in :   " << vertexUpdateTimeDelta << " ms" << std::endl;

    /* =========== */
    /* rebuild bvh */
    /* =========== */

    double buildTime0 = getSeconds();
    rtcCommit(g_scene);
    double buildTime1 = getSeconds();
    double buildTimeDelta = buildTime1-buildTime0;

    updateTimeLog(buildTime,buildTimeDelta);

    if (unlikely(printStats)) std::cout << "bvh rebuild in :     " << buildTimeDelta << " ms" << std::endl;
#endif

    frameID = (frameID + 1) % numProfileFrames;
  }

#if DUMP_PROFILE_DATA == 1

/* plot build and render times */
  void dumpBuildAndRenderTimes()
  {
    FileName name("buildRenderTimes");
    std::fstream plot;
    plot.open(name.addExt(".plot"), std::fstream::out | std::fstream::trunc);

    plot << "set terminal png size 1920,1080 enhanced" << std::endl;
    plot << "set output \"" << name.addExt(".png") << "\"" << std::endl;
    plot << "set key inside right top vertical Right noreverse enhanced autotitles box linetype -1 linewidth 1.000" << std::endl;
    plot << "set ylabel \"" << "ms" << "\"" << std::endl;
    plot << "set yrange [0:50]" << std::endl;
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
#endif

/* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
    rtcDeleteScene (g_scene); g_scene = nullptr;
    rtcDeleteDevice(g_device); g_device = nullptr;
#if DUMP_PROFILE_DATA == 1
    /* dump data at the end of profiling */
    std::cout << "dumping build and render times per frame [" << numProfileFrames << " frames]..." << std::flush;
    dumpBuildAndRenderTimes();
    std::cout << "done" << std::endl;
#endif
  }

} // namespace embree
