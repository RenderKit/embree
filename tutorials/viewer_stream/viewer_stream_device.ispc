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

#define USE_INTERFACE 0 // 0 = stream, 1 = single rays/packets, 2 = single rays/packets using stream interface
#define AMBIENT_OCCLUSION_SAMPLES 64
//#define rtcOccluded rtcIntersect
//#define rtcOccludedVM rtcIntersectVM

#define SIMPLE_SHADING 1
#define OBJ_MATERIAL 1

extern uniform ISPCScene* uniform g_ispc_scene;
extern uniform int g_instancing_mode;

/* scene data */
RTCScene g_scene = NULL;

#define MAX_EDGE_LEVEL 64.0f
#define MIN_EDGE_LEVEL  4.0f
#define LEVEL_FACTOR   64.0f

inline uniform float updateEdgeLevel( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos, const uniform unsigned int e0, const uniform unsigned int e1)
{
  const uniform Vec3fa v0 = mesh->positions[0][mesh->position_indices[e0]];
  const uniform Vec3fa v1 = mesh->positions[0][mesh->position_indices[e1]];
  const uniform Vec3fa edge = v1-v0;
  const uniform Vec3fa P = 0.5f*(v1+v0);
  const uniform Vec3fa dist = cam_pos - P;
  return max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
}


void updateEdgeLevelBuffer( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos, uniform unsigned int startID, uniform unsigned int endID )
{
  for (uniform unsigned int f=startID; f<endID;f++) {
    uniform unsigned int e = mesh->face_offsets[f];
    uniform unsigned int N = mesh->verticesPerFace[f];
    if (N == 4) /* fast path for quads */
      for (uniform unsigned int i=0; i<4; i++)
        mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%4);
    else if (N == 3) /* fast path for triangles */
      for (uniform unsigned int i=0; i<3; i++)
        mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%3);
    else /* fast path for general polygons */
      for (uniform unsigned int i=0; i<N; i++)
        mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%N);
  }
}

#if defined(ISPC)
task void updateSubMeshEdgeLevelBufferTask( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos )
{
  const uniform unsigned int size = mesh->numFaces;
  const uniform unsigned int startID = ((taskIndex+0)*size)/taskCount;
  const uniform unsigned int endID   = ((taskIndex+1)*size)/taskCount;
  updateEdgeLevelBuffer(mesh,cam_pos,startID,endID);
}
task void updateMeshEdgeLevelBufferTask( uniform ISPCScene* uniform scene_in, const uniform Vec3fa& cam_pos )
{
  uniform ISPCGeometry* uniform geometry = g_ispc_scene->geometries[taskIndex];
  if (geometry->type != SUBDIV_MESH) return;
  uniform ISPCSubdivMesh* uniform mesh = (uniform ISPCSubdivMesh* uniform) geometry;
  uniform unsigned int geomID = mesh->geom.geomID;
  if (mesh->numFaces < 10000) {
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
    rtcUpdateGeometryBuffer(geometry->geometry,RTC_BUFFER_TYPE_LEVEL,0);
    rtcCommitGeometry(geometry->geometry);
  }
}
#endif

void updateEdgeLevels(uniform ISPCScene* uniform scene_in, const uniform Vec3fa& cam_pos)
{
  /* first update small meshes */
#if defined(ISPC)
  launch[ scene_in->numGeometries ] updateMeshEdgeLevelBufferTask(scene_in,cam_pos); sync;
#endif

  /* now update large meshes */
  for (uniform unsigned int g=0; g<scene_in->numGeometries; g++)
  {
    uniform ISPCGeometry* uniform geometry = g_ispc_scene->geometries[g];
    if (geometry->type != SUBDIV_MESH) continue;
    uniform ISPCSubdivMesh* uniform mesh = (uniform ISPCSubdivMesh* uniform) geometry;
#if defined(ISPC)
    if (mesh->numFaces < 10000) continue;
    launch[ (mesh->numFaces+4095)/4096 ] updateSubMeshEdgeLevelBufferTask(mesh,cam_pos); sync;
#else
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
#endif
    rtcUpdateGeometryBuffer(geometry->geometry,RTC_BUFFER_TYPE_LEVEL,0);
    rtcCommitGeometry(geometry->geometry);
  }
}

RTCScene convertScene(uniform ISPCScene* uniform scene_in)
{
  RTCScene scene_out = ConvertScene(g_device, scene_in,RTC_BUILD_QUALITY_MEDIUM);

  /* commit individual objects in case of instancing */
  if (g_instancing_mode != ISPC_INSTANCING_NONE)
  {
    for (uniform unsigned int i=0; i<scene_in->numGeometries; i++) {
      ISPCGeometry* uniform geometry = g_ispc_scene->geometries[i];
      if (geometry->type == GROUP) rtcCommitScene(geometry->scene);
    }
  }

  return scene_out;
}

/* renders a single pixel casting with ambient occlusion */
Vec3f ambientOcclusionShading(int x, int y, Ray& ray, uniform RayStats& stats)
{
  Ray rays[AMBIENT_OCCLUSION_SAMPLES];

  Vec3f Ng = normalize(ray.Ng);
  if (dot(ray.dir,Ng) > 0.0f) Ng = neg(Ng);

  Vec3f col = make_Vec3f(min(1.0f,0.3f+0.8f*abs(dot(Ng,normalize(ray.dir)))));

  /* calculate hit point */
  float intensity = 0;
  Vec3f hitPos = ray.org + ray.tfar * ray.dir;

  RandomSampler sampler;
  RandomSampler_init(sampler,x,y,0);

  /* enable only valid rays */
  for (uniform int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
  {
    /* sample random direction */
    Vec2f s = RandomSampler_get2D(sampler);
    Sample3f dir;
    dir.v = cosineSampleHemisphere(s);
    dir.pdf = cosineSampleHemispherePDF(dir.v);
    dir.v = frame(Ng) * dir.v;

    /* initialize shadow ray */
    Ray& shadow = rays[i];
    bool mask = __mask; unmasked { // invalidate inactive rays
      shadow.tnear = mask ? 0.001f       : (float)(pos_inf);
      shadow.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(shadow, hitPos, dir.v, shadow.tnear, shadow.tfar);

    RayStats_addShadowRay(stats);
  }

  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  context.flags = g_iflags_incoherent;

  /* trace occlusion rays */
#if USE_INTERFACE == 0
  rtcOccludedVM(g_scene,&context,(varying RTCRay* uniform)&rays,AMBIENT_OCCLUSION_SAMPLES,sizeof(Ray));
#elif USE_INTERFACE == 1
  for (uniform unsigned int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
    rtcOccludedV(g_scene,RTCRay_(rays[i]));
#else
  for (uniform unsigned int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
    rtcOccludedVM(g_scene,&context,(varying RTCRay* uniform)&rays[i],1,sizeof(Ray));
#endif

  /* accumulate illumination */
  for (uniform int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++) {
    if (rays[i].tfar >= 0.0f)
      intensity += 1.0f;
  }

  /* shade pixel */
  return col * (intensity/AMBIENT_OCCLUSION_SAMPLES);
}


void postIntersectGeometry(const Ray& ray, DifferentialGeometry& dg, uniform ISPCGeometry* uniform geometry, int& materialID)
{
  if (geometry->type == TRIANGLE_MESH)
  {
    uniform ISPCTriangleMesh* uniform mesh = (uniform ISPCTriangleMesh* uniform) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == QUAD_MESH)
  {
    uniform ISPCQuadMesh* uniform mesh = (uniform ISPCQuadMesh* uniform) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == GRID_MESH)
  {
    uniform ISPCGridMesh* uniform mesh = (uniform ISPCGridMesh* uniform) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == SUBDIV_MESH)
  {
    uniform ISPCSubdivMesh* uniform mesh = (uniform ISPCSubdivMesh* uniform) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == CURVES)
  {
    uniform ISPCHairSet* uniform mesh = (uniform ISPCHairSet* uniform) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == GROUP) {
    foreach_unique (geomID in ray.geomID) {
      postIntersectGeometry(ray,dg,((uniform ISPCGroup*) geometry)->geometries[geomID],materialID);
    }
  }
  else
    assert(false);
}

AffineSpace3f calculate_interpolated_space (uniform ISPCInstance* uniform instance, float gtime)
{
  if (instance->numTimeSteps == 1)
    return make_AffineSpace3f(instance->spaces[0]);

   /* calculate time segment itime and fractional time ftime */
  const int time_segments = instance->numTimeSteps-1;
  const float time = gtime*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(varying int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  return (1.0f-ftime)*make_AffineSpace3f(instance->spaces[itime+0]) + ftime*make_AffineSpace3f(instance->spaces[itime+1]);
}

typedef ISPCInstance* uniform ISPCInstancePtr;

inline int postIntersect(const Ray& ray, DifferentialGeometry& dg)
{
  int materialID = 0;
  foreach_unique (instID in ray.instID) {
    foreach_unique (geomID in ray.geomID) {
      ISPCGeometry* uniform geometry = NULL;
      if (g_instancing_mode != ISPC_INSTANCING_NONE) {
        ISPCInstance* uniform instance = (ISPCInstancePtr) g_ispc_scene->geometries[instID];
        geometry = instance->child;
      } else {
        geometry = g_ispc_scene->geometries[geomID];
      }
      postIntersectGeometry(ray,dg,geometry,materialID);
    }
  }

  if (g_instancing_mode != ISPC_INSTANCING_NONE)
  {
    foreach_unique (instID in ray.instID)
    {
      /* get instance and geometry pointers */
      ISPCInstance* uniform instance = (ISPCInstancePtr) g_ispc_scene->geometries[instID];

      /* convert normals */
      //AffineSpace3f space = (1.0f-ray.time)*make_AffineSpace3f(instance->space0) + ray.time*make_AffineSpace3f(instance->space1);
      AffineSpace3f space = calculate_interpolated_space(instance,ray.time);
      dg.Ng = xfmVector(space,dg.Ng);
      dg.Ns = xfmVector(space,dg.Ns);
    }
  }

  return materialID;
}

inline Vec3f face_forward(const Vec3f& dir, const Vec3f& _Ng) {
  const Vec3f Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* renders a single screen tile */
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

  Ray rays[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  uniform int N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    RandomSampler sampler;
    RandomSampler_init(sampler, x, y, 0);

    /* initialize ray */
    Ray& ray = rays[N++];
    bool mask = __mask; unmasked { // invalidates inactive rays
      ray.tnear = mask ? 0.0f         : (float)(pos_inf);
      ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(ray, make_Vec3f(camera.xfm.p), make_Vec3f(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), ray.tnear, ray.tfar, RandomSampler_get1D(sampler));

    RayStats_addRay(stats);
  }

  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  context.flags = g_iflags_coherent;

  /* trace stream of rays */
#if USE_INTERFACE == 0
  rtcIntersectVM(g_scene,&context,(varying RTCRayHit* uniform)&rays[0],N,sizeof(Ray));
#elif USE_INTERFACE == 1
  for (uniform unsigned int i=0; i<N; i++)
    rtcIntersectV(g_scene,&context,RTCRayHit_(rays[i]));
#else
  for (uniform unsigned int i=0; i<N; i++)
    rtcIntersectVM(g_scene,&context,(varying RTCRayHit* uniform)&rays[i],1,sizeof(Ray));
#endif

  /* shade stream of rays */
  N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;
    Ray& ray = rays[N++];

    /* eyelight shading */
    Vec3f color = make_Vec3f(0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
#if SIMPLE_SHADING == 1
    {
#if OBJ_MATERIAL == 1
      Vec3f Kd = make_Vec3f(0.5f);
      DifferentialGeometry dg;
      dg.geomID = ray.geomID;
      dg.primID = ray.primID;
      dg.u = ray.u;
      dg.v = ray.v;
      dg.P  = ray.org+ray.tfar*ray.dir;
      dg.Ng = ray.Ng;
      dg.Ns = ray.Ng;
      int materialID = postIntersect(ray,dg);
      dg.Ng = face_forward(ray.dir,normalize(dg.Ng));
      dg.Ns = face_forward(ray.dir,normalize(dg.Ns));
      
      /* shade */
      if (g_ispc_scene->materials[materialID]->type == MATERIAL_OBJ) {
        uniform ISPCOBJMaterial* material = (uniform ISPCOBJMaterial*) g_ispc_scene->materials[materialID];
        Kd = make_Vec3f(material->Kd);
      }
      
      color = Kd*dot(neg(ray.dir),dg.Ns);
#else
      color = make_Vec3f(abs(dot(ray.dir,normalize(ray.Ng))));
#endif
    }
#else
      color = ambientOcclusionShading(x,y,ray,g_stats[threadIndex]);
#endif

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

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  /* create scene */
  if (!g_scene) {
    g_scene = convertScene(g_ispc_scene);
    updateEdgeLevels(g_ispc_scene, camera.xfm.p);
    rtcCommitScene (g_scene);
  }
  
  /* render image */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = NULL;
}
