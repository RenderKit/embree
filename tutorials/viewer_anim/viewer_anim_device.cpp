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

extern "C" ISPCScene* g_ispc_scene;

/* enable obj animation */
extern "C" bool g_anim;

/* scene data */
  RTCDevice g_device = nullptr;
  RTCScene g_scene   = nullptr;

/* animation data */
  size_t animFrameID = 0;
  size_t numAnimFrames  = 1;
  std::vector<double> buildTime;
  std::vector<double> renderTime;

  unsigned int convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
{
  unsigned int geomID = rtcNewTriangleMesh (scene_out, object_flags, mesh->numTriangles, mesh->numVertices, mesh->numTimeSteps);
  for (size_t t=0; t<mesh->numTimeSteps; t++) {
    rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
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

  RTCScene convertScene(ISPCScene* scene_in, bool animSequence = false)
{
  size_t numGeometries = scene_in->numGeometries;
  PRINT(numGeometries);
  int scene_flags = RTC_SCENE_INCOHERENT | (animSequence ? RTC_SCENE_DYNAMIC : RTC_SCENE_STATIC);
  int scene_aflags = RTC_INTERSECT1 | RTC_INTERSECT_STREAM | RTC_INTERPOLATE;
  RTCScene scene_out = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);

  RTCGeometryFlags object_flags = animSequence ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;
  for (size_t i=0; i<numGeometries; i++)
  {
    ISPCGeometry* geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      unsigned int geomID MAYBE_UNUSED = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out, object_flags);
      ((ISPCSubdivMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      unsigned int geomID MAYBE_UNUSED = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out, object_flags);
      ((ISPCTriangleMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == QUAD_MESH) {
      unsigned int geomID MAYBE_UNUSED = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out, object_flags);
      ((ISPCQuadMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      unsigned int geomID MAYBE_UNUSED = convertLineSegments((ISPCLineSegments*) geometry, scene_out, object_flags);
      ((ISPCLineSegments*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == HAIR_SET) {
      unsigned int geomID MAYBE_UNUSED = convertHairSet((ISPCHairSet*) geometry, scene_out, object_flags);
      ((ISPCHairSet*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == CURVES) {
      unsigned int geomID MAYBE_UNUSED = convertCurveGeometry((ISPCHairSet*) geometry, scene_out, object_flags);
      ((ISPCHairSet*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else
      assert(false);
    if (animSequence) 
      break;
  }
  return scene_out;
}

  void updateObjects(ISPCScene* scene_in, RTCScene scene_out, size_t keyFrameID)
  {
    size_t numGeometries = scene_in->numGeometries;
    if (!numGeometries) return;

    ISPCGeometry* geometry = scene_in->geometries[keyFrameID % numGeometries];

    if (geometry->type == SUBDIV_MESH) {
      unsigned int geomID = ((ISPCSubdivMesh*)geometry)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      ISPCTriangleMesh* mesh0 = (ISPCTriangleMesh*)scene_in->geometries[0];
      ISPCTriangleMesh* mesh = (ISPCTriangleMesh*)geometry;
      unsigned int geomID = mesh0->geomID;
      for (size_t t=0; t<mesh->numTimeSteps; t++) {
        rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
      }
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == QUAD_MESH) {
      unsigned int geomID = ((ISPCQuadMesh*)geometry)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      unsigned int geomID = ((ISPCLineSegments*)geometry)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == HAIR_SET) {
      unsigned int geomID = ((ISPCHairSet*)geometry)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry->type == CURVES) {
      unsigned int geomID = ((ISPCHairSet*)geometry)->geomID;
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
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
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
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
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
  g_scene = convertScene(g_ispc_scene,g_anim);
  rtcCommit (g_scene);

  numAnimFrames = getNumObjects(g_ispc_scene);
  PRINT(numAnimFrames);

  buildTime.resize(numAnimFrames);
  renderTime.resize(numAnimFrames);


  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  /* render image */
  double r0 = getSeconds();
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
  double r1 = getSeconds();
  PRINT(r1-r0);
  assert(animFrameID < renderTime.size());
  assert(animFrameID < buildTime.size());

  renderTime[animFrameID] = r1-r0;

  /* update geometry and rebuild */
  if (g_anim)
  {
    updateObjects(g_ispc_scene,g_scene,animFrameID);

    double t0 = getSeconds();
    rtcCommit(g_scene);      
    double t1 = getSeconds();
    renderTime[animFrameID] = t0-t1;

    std::cout << "bvh rebuild in " << t1-t0 << " ms" << std::endl;
    animFrameID = (animFrameID+1) % numAnimFrames;
  }
}

/* plot build and render times */

  void plotBuildAndRenderTimes(FileName &name)
  {
    std::fstream plot;
    plot.open(name.addExt(".plot"), std::fstream::out | std::fstream::trunc);
    plot << "set terminal png size 2048,600 enhanced" << std::endl;
    plot << "set output \"" << name.addExt(".png") << "\"" << std::endl;
    plot << "set key inside right top vertical Right noreverse enhanced autotitles box linetype -1 linewidth 1.000" << std::endl;
    plot << "set samples 50, 50" << std::endl;
    //plot << "set title \"" << name << "\"" << std::endl; 
    //plot << "set xlabel \"" << name << "\""<< std::endl;
    plot << "set xtics axis rotate by 90" << std::endl;
    //plot << "set ylabel \"" << unit << "\"" << std::endl;
    plot << "set yrange [0:]" << std::endl;
    //plot << "plot \"" << FileName(name).addExt(".txt") << "\" using :2:xtic(1) title \"" << name << "\" with lines, \\" << std::endl; 
    //plot << "     \"" << FileName(name).addExt(".txt") << "\" using :3         title \"best\" with lines" << std::endl;
    plot << std::endl;
    plot.close();
  }

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene); g_scene = nullptr;
  rtcDeleteDevice(g_device); g_device = nullptr;
}

} // namespace embree
