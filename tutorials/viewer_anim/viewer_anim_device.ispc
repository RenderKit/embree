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

#include "../common/math/random_sampler.isph"
#include "../common/math/sampling.isph"
#include "../common/tutorial/tutorial_device.isph"
#include "../common/tutorial/scene_device.h"

#define ANIM_FPS 15.0f
#define ENABLE_ANIM 1
#define VERTEX_NORMALS 0
#define SHADOWS 1
#define VERTEX_INTERPOLATION_BLOCK_SIZE 1024

extern uniform ISPCScene* uniform g_ispc_scene;

/* scene data */
uniform RTCDevice g_device = NULL;
uniform RTCScene g_scene   = NULL;
varying Vec3f* uniform ls_positions = NULL;

/* animation data */
uniform double animTime        = -1.0f; // global time counter


  // ==================================================================================================
  // ==================================================================================================
  // ==================================================================================================


uniform unsigned int convertTriangleMesh(uniform ISPCTriangleMesh* uniform mesh, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  uniform RTCGeometryFlags object_flags = mesh->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
  /* create object */
  uniform unsigned int geomID = rtcNewTriangleMesh (scene_out, object_flags, mesh->numTriangles, mesh->numVertices, 1);
  /* generate vertex buffer */
  uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
  for (uniform size_t i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
  rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
  /* set index buffer */
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(uniform ISPCTriangle));
  mesh->geom.geomID = geomID;
  return geomID;
}

uniform unsigned int convertQuadMesh(uniform ISPCQuadMesh* uniform mesh, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    uniform RTCGeometryFlags object_flags = mesh->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    uniform unsigned int geomID = rtcNewQuadMesh (scene_out, object_flags, mesh->numQuads, mesh->numVertices, mesh->numTimeSteps);
    /* generate vertex buffer */
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(uniform ISPCQuad));
    mesh->geom.geomID = geomID;
    return geomID;
  }

uniform unsigned int convertSubdivMesh(uniform ISPCSubdivMesh* uniform mesh, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    uniform RTCGeometryFlags object_flags = mesh->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    uniform unsigned int geomID = rtcNewSubdivisionMesh(scene_out, object_flags, mesh->numFaces, mesh->numEdges, mesh->numVertices,
                                                mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles, mesh->numTimeSteps);
    mesh->geom.geomID = geomID;
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 4.0f;
    /* generate vertex buffer */
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set all other buffers */
    rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(uniform float));
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(uniform unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(uniform unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(uniform unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(uniform unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(uniform float));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(uniform unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(uniform float));
    rtcSetSubdivisionMode(scene_out, geomID, 0, mesh->position_subdiv_mode);
    return geomID;
  }

uniform unsigned int convertLineSegments(uniform ISPCLineSegments* uniform mesh, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    uniform RTCGeometryFlags object_flags = mesh->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    uniform unsigned int geomID = rtcNewLineSegments (scene_out, object_flags, mesh->numSegments, mesh->numVertices, mesh->numTimeSteps);
    /* generate vertex buffer */
    uniform Vec3fa* uniform vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(uniform int));
    return geomID;
  }

uniform unsigned int convertHairSet(uniform ISPCHairSet* uniform hair, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    uniform RTCGeometryFlags object_flags = hair->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    uniform unsigned int geomID = 0;
    switch (hair->basis) {
    case BEZIER_BASIS : geomID = rtcNewBezierHairGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps); break;
    case BSPLINE_BASIS: geomID = rtcNewBSplineHairGeometry(scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps); break;
    default: assert(false);
    }
    /* generate vertex buffer */
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<hair->numVertices;i++) vertices[i] = hair->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(uniform ISPCHair));
    rtcSetTessellationRate(scene_out,geomID,(float)hair->tessellation_rate);
    return geomID;
  }

uniform unsigned int convertCurveGeometry(uniform ISPCHairSet* uniform hair, RTCScene scene_out)
  {
    /* if more than a single timestep, mark object as dynamic */
    uniform RTCGeometryFlags object_flags = hair->numTimeSteps > 1 ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
    /* create object */
    uniform unsigned int geomID = 0;
    switch (hair->basis) {
    case BEZIER_BASIS : geomID = rtcNewBezierCurveGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps); break;
    case BSPLINE_BASIS: geomID = rtcNewBSplineCurveGeometry(scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps); break;
    default: assert(false);
    }
    /* generate vertex buffer */
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
    for (size_t i=0;i<hair->numVertices;i++) vertices[i] = hair->positions[0][i];
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    /* set index buffer */
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(uniform ISPCHair));
    return geomID;
  }

  uniform size_t getNumObjects(uniform ISPCScene* uniform scene_in)
  {
    return scene_in->numGeometries;
  }

  uniform RTCScene createScene(uniform ISPCScene* uniform scene_in)
  {
    uniform int scene_flags = RTC_SCENE_INCOHERENT | RTC_SCENE_DYNAMIC;
    uniform int scene_aflags = RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING | RTC_INTERSECT_STREAM | RTC_INTERPOLATE;
    return rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
  }


  void createObject(const uniform size_t i, uniform ISPCScene* uniform scene_in, uniform RTCScene scene_out)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[i];
    uniform unsigned int geomID = 0;

    if (geometry->type == SUBDIV_MESH) {
      geomID = convertSubdivMesh((uniform ISPCSubdivMesh* uniform) geometry, scene_out);
      ((uniform ISPCSubdivMesh* uniform)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      geomID = convertTriangleMesh((uniform ISPCTriangleMesh* uniform) geometry, scene_out);
      ((uniform ISPCTriangleMesh* uniform)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == QUAD_MESH) {
      geomID = convertQuadMesh((uniform ISPCQuadMesh* uniform) geometry, scene_out);
      ((uniform ISPCQuadMesh* uniform)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      geomID = convertLineSegments((uniform ISPCLineSegments* uniform) geometry, scene_out);
      ((uniform ISPCLineSegments* uniform)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == HAIR_SET) {
      geomID = convertHairSet((uniform ISPCHairSet* uniform) geometry, scene_out);
      ((uniform ISPCHairSet* uniform)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == CURVES) {
      geomID = convertCurveGeometry((uniform ISPCHairSet* uniform) geometry, scene_out);
      ((uniform ISPCHairSet* uniform)geometry)->geom.geomID = geomID;
      assert(geomID == i);
    }
    else
      assert(false);
  }

  uniform Vec3fa lerp(const uniform Vec3fa& v0, const uniform Vec3fa& v1, const uniform float t) {
    return v0*(1.0f-t)+v1*t;
  }


 task void interpolateVertexBlock(const uniform size_t numVertices,
                                  uniform Vec3fa* uniform vertices,
                                  const uniform Vec3fa* uniform const input0,
                                  const uniform Vec3fa* uniform const input1,
                                  const uniform float tt)
 {
   const uniform size_t b = taskIndex;
   const uniform size_t startID = b*VERTEX_INTERPOLATION_BLOCK_SIZE;
   const uniform size_t endID = min(startID + VERTEX_INTERPOLATION_BLOCK_SIZE,numVertices);
   for (uniform size_t i=startID; i<endID; i++)
     vertices[i] = lerp(input0[i],input1[i],tt);
 }


  void interpolateVertices(uniform RTCScene scene_out,
                           const uniform unsigned int geomID,
                           const uniform size_t numVertices,
                           const uniform Vec3fa* uniform const input0,
                           const uniform Vec3fa* uniform const input1,
                           const uniform float tt)
  {
    uniform Vec3fa* uniform vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);
#if 1
    const uniform size_t blocks = (numVertices+VERTEX_INTERPOLATION_BLOCK_SIZE-1) / VERTEX_INTERPOLATION_BLOCK_SIZE;
    launch[blocks] interpolateVertexBlock(numVertices,vertices,input0,input1,tt); sync;
#else
    for (uniform size_t i=0; i<numVertices; i++)
      vertices[i] = lerp(input0[i],input1[i],tt);
#endif
    rtcUnmapBuffer(scene_out, geomID, RTC_VERTEX_BUFFER);
    rtcUpdate(scene_out,geomID);
  }

  void updateVertexData(const uniform unsigned int ID,
                        uniform ISPCScene* uniform scene_in,
                        uniform RTCScene uniform scene_out,
                        const uniform size_t keyFrameID,
                        const uniform float tt)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[ID];

    if (geometry->type == SUBDIV_MESH) {
      uniform unsigned int geomID = ((uniform ISPCSubdivMesh* uniform)geometry)->geom.geomID;
      /* if static do nothing */
      if (((uniform ISPCSubdivMesh* uniform)geometry)->numTimeSteps <= 1) return;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      uniform ISPCTriangleMesh* uniform mesh = (uniform ISPCTriangleMesh* uniform)geometry;
      /* if static do nothing */
      if (mesh->numTimeSteps <= 1) return;
      /* interpolate two vertices from two timesteps */
      const uniform size_t t0 = (keyFrameID+0) % mesh->numTimeSteps;
      const uniform size_t t1 = (keyFrameID+1) % mesh->numTimeSteps;
      const uniform Vec3fa* uniform const input0 = mesh->positions[t0];
      const uniform Vec3fa* uniform const input1 = mesh->positions[t1];
      interpolateVertices(scene_out, mesh->geom.geomID, mesh->numVertices, input0, input1, tt);
    }
    else if (geometry->type == QUAD_MESH) {
      uniform ISPCQuadMesh* uniform mesh = (uniform ISPCQuadMesh* uniform)geometry;
      /* if static do nothing */
      if (mesh->numTimeSteps <= 1) return;
      /* interpolate two vertices from two timesteps */
      const uniform size_t t0 = (keyFrameID+0) % mesh->numTimeSteps;
      const uniform size_t t1 = (keyFrameID+1) % mesh->numTimeSteps;
      const uniform Vec3fa* uniform const input0 = mesh->positions[t0];
      const uniform Vec3fa* uniform const input1 = mesh->positions[t1];
      interpolateVertices(scene_out, mesh->geom.geomID, mesh->numVertices, input0, input1, tt);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      uniform unsigned int geomID = ((uniform ISPCLineSegments* uniform)geometry)->geom.geomID;
      /* if static do nothing */
      if (((uniform ISPCLineSegments* uniform)geometry)->numTimeSteps <= 1) return;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == HAIR_SET) {
      uniform unsigned int geomID = ((uniform ISPCHairSet* uniform)geometry)->geom.geomID;
      /* if static do nothing */
      if (((uniform ISPCHairSet* uniform)geometry)->numTimeSteps <= 1) return;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == CURVES) {
      uniform unsigned int geomID = ((uniform ISPCHairSet* uniform)geometry)->geom.geomID;
      /* if static do nothing */
      if (((uniform ISPCHairSet* uniform)geometry)->numTimeSteps <= 1) return;
      rtcUpdate(scene_out,geomID);
    }
    else
      assert(false);
  }


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

  uniform RayStats& stats = g_stats[threadIndex];

  RTCRay rays[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  uniform int N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* initialize ray */
    RTCRay& ray = rays[N++];

    ray.org = make_Vec3f(camera.xfm.p);
    ray.dir = make_Vec3f(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz));
    bool mask = __mask; unmasked { // invalidates inactive rays
      ray.tnear = mask ? 0.0f         : (float)(pos_inf);
      ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.primID = RTC_INVALID_GEOMETRY_ID;
    ray.mask = -1;
    ray.time = 0.0f;
    RayStats_addRay(stats);
  }

  uniform RTCIntersectContext context;
  context.flags = g_iflags_coherent;

  /* trace stream of rays */
  rtcIntersectVM(g_scene,&context,rays,N,sizeof(RTCRay));

  /* shade stream of rays */
  Vec3f colors[TILE_SIZE_X*TILE_SIZE_Y];
  N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;
    RTCRay& ray = rays[N];

    Vec3f Ng = ray.Ng;

    /* shading */
    Vec3f color = make_Vec3f(0.0f,1.0f,0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
    {
      /* vertex normals */
#if VERTEX_NORMALS == 1
      ISPCGeometry* geometry = g_ispc_scene->geometries[ray.geomID];
      if (geometry->type == TRIANGLE_MESH)
      {
        ISPCTriangleMesh* mesh = (ISPCTriangleMesh*) geometry;
        if (mesh->normals)
        {
          ISPCTriangle* tri = &mesh->triangles[ray.primID];

          const Vec3fa n0 = mesh->normals[tri->v0];
          const Vec3fa n1 = mesh->normals[tri->v1];
          const Vec3fa n2 = mesh->normals[tri->v2];
          const Vec3fa n = n0*(1.0f-ray.u-ray.v) + n1*ray.u + n2*ray.v;
          Ng = make_Vec3f(n.x,n.y,n.z);
        }
      }
#endif
      color = make_Vec3f(abs(dot(ray.dir,normalize(Ng))));
    }
    colors[N++] = color;
  }


#if SHADOWS == 1
    /* do some hard shadows to point lights */
    if (g_ispc_scene->numLights)
    {
      for (uniform size_t i=0; i<g_ispc_scene->numLights; i++)
      {
        /* init shadow/occlusion rays */
        for (uniform size_t n=0;n<N;n++)
        {
          RTCRay& ray = rays[n];
          const bool valid = ray.geomID != RTC_INVALID_GEOMETRY_ID;
          const Vec3f hitpos = ray.org + ray.dir * ray.tfar;
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
#if 0
        for (uniform size_t n=0;n<N;n++)
          rtcOccludedEx(g_scene,&context,rays[n]);
#else
        rtcOccludedVM(g_scene,&context,rays,N,sizeof(RTCRay));
#endif
        /* modify pixel color based on occlusion */
        for (uniform size_t n=0;n<N;n++)
          if (rays[n].geomID != RTC_INVALID_GEOMETRY_ID)
            colors[n] = colors[n] * 0.1f;

      }
    }
#endif

  N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;
    Vec3f& color = colors[N++];
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
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

void device_key_pressed_handler(uniform int key)
{
  if (key == 111 /*o*/) {
  }
  else if (key == 112 /*p*/) {
  }
  else device_key_pressed_default(key);
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(NULL,rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction2(g_device,error_handler,NULL);

  /* create scene */
  g_scene = createScene(g_ispc_scene);

  /* create objects */
  uniform size_t numObjects = getNumObjects(g_ispc_scene);
  print("numObjects %\n",numObjects);
  for (uniform size_t i=0;i<numObjects;i++)
    createObject(i,g_ispc_scene,g_scene);

  rtcCommit (g_scene);

  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_handler;
}

#define TICKS_PER_SECOND 2000000000

uniform double getSeconds() { return (double)clock() / TICKS_PER_SECOND; }

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{

  /* =================================== */
  /* samples LS positions as pointlights */
  /* =================================== */

  if (g_ispc_scene->numLights)
  {
    if (ls_positions == NULL) ls_positions = uniform new varying Vec3f[g_ispc_scene->numLights];
    DifferentialGeometry dg;
    dg.geomID = 0;
    dg.primID = 0;
    dg.u = 0.0f;
    dg.v = 0.0f;
    dg.P  = make_Vec3f(0.0f,0.0f,0.0f);
    dg.Ng = make_Vec3f(0.0f,0.0f,0.0f);
    dg.Ns = dg.Ng;
    for (uniform size_t i=0; i<g_ispc_scene->numLights; i++)
    {
      const uniform Light* uniform l = g_ispc_scene->lights[i];
      const Vec2f sample = make_Vec2f(0.0f,0.0f);
      Light_SampleRes ls = l->sample(l,dg,sample);
      ls_positions[i] = ls.dir * ls.dist;
    }
  }

  /* ============ */
  /* render image */
  /* ============ */

  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;

  /* =============== */
  /* update geometry */
  /* =============== */

#if ENABLE_ANIM == 1

  uniform double vertexUpdateTime0 = getSeconds();

  if (animTime < 0.0f) animTime = getSeconds();
  const uniform double atime = (getSeconds() - animTime) * ANIM_FPS;
  const uniform size_t intpart = (size_t)floor(atime);
  const uniform double fracpart = atime - (double)intpart;
  const uniform size_t keyFrameID = intpart;

  uniform size_t numObjects = getNumObjects(g_ispc_scene);
  for (uniform size_t i=0;i<numObjects;i++)
    updateVertexData(i, g_ispc_scene, g_scene, keyFrameID, (float)fracpart);

  /* =========== */
  /* rebuild bvh */
  /* =========== */

  rtcCommit(g_scene);

#endif
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene); g_scene = NULL;
  rtcDeleteDevice(g_device); g_device = NULL;
}
