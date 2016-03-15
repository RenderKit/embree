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

#include "../common/tutorial/tutorial_device.isph"
#include "../common/tutorial/scene_device.isph"
#include "../common/tutorial/random_sampler.isph"
#include "../pathtracer/shapesampler.isph"

#define AMBIENT_OCCLUSION_SAMPLES 64

extern uniform ISPCScene* uniform g_ispc_scene;

/* scene data */
RTCDevice g_device = NULL;
RTCScene g_scene = NULL;

/* error reporting function */
unmasked void error_handler(const uniform RTCError code, const uniform int8* uniform str = NULL)
{
  if (code == RTC_NO_ERROR) 
    return;

  print("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : print("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : print("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: print("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : print("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : print("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : print("RTC_CANCELLED"); break;
  default                   : print("invalid error code"); break;
  }
  if (str) { 
    print(" ("); 
    while (*str) putchar(*str++); 
    print(")\n"); 
  }
  abort();
}

uniform unsigned int convertTriangleMesh(uniform ISPCTriangleMesh* uniform mesh, RTCScene scene_out)
{
  uniform unsigned int geomID = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices, mesh->positions2 ? 2 : 1);
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(uniform Vec3fa      ));
  if (mesh->positions2) rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER1, mesh->positions2, 0, sizeof(uniform Vec3fa      ));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(uniform ISPCTriangle));
  mesh->geomID = geomID;
  return geomID;
}

uniform unsigned int convertQuadMesh(uniform ISPCQuadMesh* uniform mesh, RTCScene scene_out)
{
  uniform unsigned int geomID = rtcNewQuadMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numQuads, mesh->numVertices, mesh->positions2 ? 2 : 1);
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(uniform Vec3fa      ));
  if (mesh->positions2) rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER1, mesh->positions2, 0, sizeof(uniform Vec3fa      ));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(uniform ISPCQuad));
  mesh->geomID = geomID;
  return geomID;
}

uniform unsigned int convertSubdivMesh(uniform ISPCSubdivMesh* uniform mesh, RTCScene scene_out)
{
  uniform unsigned int geomID = rtcNewSubdivisionMesh(scene_out, RTC_GEOMETRY_STATIC, mesh->numFaces, mesh->numEdges, mesh->numVertices, 
                                                      mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles);
  mesh->geomID = geomID;												
  for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 16.0f;
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(uniform Vec3fa  ));
  rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(uniform float));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(uniform float));
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(uniform float));
  return geomID;
} 

uniform unsigned int convertLineSegments(uniform ISPCLineSegments* uniform mesh, RTCScene scene_out)
{
  uniform unsigned int geomID = rtcNewLineSegments (scene_out, RTC_GEOMETRY_STATIC, mesh->numSegments, mesh->numVertices, mesh->v2 ? 2 : 1);
  rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,mesh->v,0,sizeof(uniform Vertex));
  if (mesh->v2) rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER1,mesh->v2,0,sizeof(uniform Vertex));
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(uniform int));
  return geomID;
}

uniform unsigned int convertHairSet(uniform ISPCHairSet* uniform hair, RTCScene scene_out)
{
  uniform unsigned int geomID = rtcNewHairGeometry (scene_out, RTC_GEOMETRY_STATIC, hair->numHairs, hair->numVertices, hair->v2 ? 2 : 1);
  rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,hair->v,0,sizeof(uniform Vertex));
  if (hair->v2) rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER1,hair->v2,0,sizeof(uniform Vertex));
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(uniform ISPCHair));
  return geomID;
}

uniform unsigned int convertCurveGeometry(uniform ISPCHairSet* uniform hair, RTCScene scene_out)
{
  uniform unsigned int geomID = rtcNewCurveGeometry (scene_out, RTC_GEOMETRY_STATIC, hair->numHairs, hair->numVertices, hair->v2 ? 2 : 1);
  rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,hair->v,0,sizeof(uniform Vertex));
  if (hair->v2) rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER1,hair->v2,0,sizeof(uniform Vertex));
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(uniform ISPCHair));
  return geomID;
}

RTCScene convertScene(uniform ISPCScene* uniform scene_in)
{
  uniform size_t numGeometries = scene_in->numGeometries;
  uniform int scene_flags = RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT;
  uniform int scene_aflags = RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING | RTC_INTERSECTN | RTC_INTERPOLATE;
  RTCScene scene_out = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);

  for (uniform size_t i=0; i<scene_in->numGeometries; i++)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      uniform unsigned int geomID = convertSubdivMesh((uniform ISPCSubdivMesh* uniform) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      uniform unsigned int geomID = convertTriangleMesh((uniform ISPCTriangleMesh* uniform) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == QUAD_MESH) {
      uniform unsigned int geomID = convertQuadMesh((uniform ISPCQuadMesh* uniform) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      uniform unsigned int geomID = convertLineSegments((uniform ISPCLineSegments* uniform) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == HAIR_SET) {
      uniform unsigned int geomID = convertHairSet((uniform ISPCHairSet* uniform) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == CURVES) {
      uniform unsigned int geomID = convertCurveGeometry((uniform ISPCHairSet* uniform) geometry, scene_out);
      assert(geomID == i);
    }
    else
      assert(false);
  }
  return scene_out;
}

/* renders a single pixel casting with ambient occlusion */
Vec3f ambientOcclusionShading(int x, int y, RTCRay& ray)
{
  Vec3f Ng = normalize(ray.Ng);
  Vec3f col = make_Vec3f(min(1.0f,0.3f+0.8f*abs(dot(Ng,normalize(ray.dir)))));

  /* calculate hit point */
  float intensity = 0;
  Vec3f hitPos = ray.org + ray.tfar * ray.dir;

  RTCRay rays[AMBIENT_OCCLUSION_SAMPLES];

  RandomSampler sampler;
  RandomSampler_init(sampler,x,y,0);
  
  for (uniform int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
  {
    /* sample random direction */
    float sx = RandomSampler_get1D(sampler);
    float sy = RandomSampler_get1D(sampler);
    Sample3f dir = cosineSampleHemisphere(sx,sy,Ng);

    /* initialize shadow ray */
    RTCRay& shadow = rays[i];
    shadow.org = hitPos;
    shadow.dir = dir.v;
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;    // FIXME: invalidate inactive rays
  } 
  
  /* trace occlusion rays */
  rtcOccludedN(g_scene,rays,AMBIENT_OCCLUSION_SAMPLES,sizeof(RTCRay),0);

  /* accumulate illumination */
  for (uniform int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++) {
    if (rays[i].geomID == RTC_INVALID_GEOMETRY_ID)
      intensity += 1.0f;   
  }
  
  /* shade pixel */
  return col * (intensity/AMBIENT_OCCLUSION_SAMPLES);
}

/* renders a single screen tile */
void renderTileStandard(uniform int taskIndex, 
                        uniform int* uniform pixels,
                        const uniform int width,
                        const uniform int height, 
                        const uniform float time,
                        const uniform Vec3f& vx, 
                        const uniform Vec3f& vy, 
                        const uniform Vec3f& vz, 
                        const uniform Vec3f& p,
                        const uniform int numTilesX, 
                        const uniform int numTilesY)
{
  const uniform int tileY = taskIndex / numTilesX;
  const uniform int tileX = taskIndex - tileY * numTilesX;
  const uniform int x0 = tileX * TILE_SIZE_X;
  const uniform int x1 = min(x0+TILE_SIZE_X,width);
  const uniform int y0 = tileY * TILE_SIZE_Y;
  const uniform int y1 = min(y0+TILE_SIZE_Y,height);

  RTCRay rays[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  uniform int packets = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler, x, y, 0);
    
    /* initialize ray */
    RTCRay& ray = rays[packets];
    ray.org = p;
    ray.dir = normalize(x*vx + y*vy + vz);
    ray.tnear = 0.0f;
    ray.tfar = inf;
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.primID = RTC_INVALID_GEOMETRY_ID;
    ray.mask = -1;
    ray.time = RandomSampler_get1D(sampler);
    packets++; // FIXME: make invalid rays empty
  }

  /* trace stream of rays */
  rtcIntersectN(g_scene,rays,packets,sizeof(RTCRay),0);

  /* shade stream of rays */
  packets = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    RTCRay& ray = rays[packets++];

    /* eyelight shading */
    Vec3f color = make_Vec3f(0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
      //color = make_Vec3f(abs(dot(ray.dir,normalize(ray.Ng))));
      color = ambientOcclusionShading(x,y,ray);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
task void renderTileTask(uniform int* uniform pixels,
                         const uniform int width,
                         const uniform int height, 
                         const uniform float time,
                         const uniform Vec3f& vx, 
                         const uniform Vec3f& vy, 
                         const uniform Vec3f& vz, 
                         const uniform Vec3f& p,
                         const uniform int numTilesX, 
                         const uniform int numTilesY)
{
  renderTile(taskIndex,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY);
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);

  /* create scene */
  g_scene = convertScene(g_ispc_scene);
  rtcCommit (g_scene);

  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform int width,
                           const uniform int height, 
                           const uniform float time,
                           const uniform Vec3f& vx, 
                           const uniform Vec3f& vy, 
                           const uniform Vec3f& vz, 
                           const uniform Vec3f& p)
{
  /* render image */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
}
