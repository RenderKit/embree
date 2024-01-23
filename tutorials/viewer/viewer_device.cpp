// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "viewer_device.h"

namespace embree {

RTCScene g_scene = nullptr;
extern "C" bool g_changed;
TutorialData data;

#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION) && defined(USE_SPECIALIZATION_CONSTANTS)
const sycl::specialization_id<RTCFeatureFlags> spec_feature_mask;
#endif

extern "C" RTCFeatureFlags g_feature_mask;
extern "C" bool g_use_scene_features;

#define SPP 1

#define FIXED_EDGE_TESSELLATION_VALUE 3

#define MAX_EDGE_LEVEL 64.0f
#define MIN_EDGE_LEVEL  4.0f
#define LEVEL_FACTOR   64.0f

bool monitorProgressFunction(void* ptr, double dn) 
{
  return true;
}

inline float updateEdgeLevel( ISPCSubdivMesh* mesh, const Vec3fa& cam_pos, const unsigned int e0, const unsigned int e1)
{
  const Vec3fa v0 = Vec3fa(mesh->positions[0][mesh->position_indices[e0]]);
  const Vec3fa v1 = Vec3fa(mesh->positions[0][mesh->position_indices[e1]]);
  const Vec3fa edge = v1-v0;
  const Vec3fa P = 0.5f*(v1+v0);
  const Vec3fa dist = cam_pos - P;
  return max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
}


void updateEdgeLevelBuffer( ISPCSubdivMesh* mesh, const Vec3fa& cam_pos, unsigned int startID, unsigned int endID )
{
  for (unsigned int f=startID; f<endID;f++) {
    unsigned int e = mesh->face_offsets[f];
    unsigned int N = mesh->verticesPerFace[f];
    if (N == 4) /* fast path for quads */
      for (unsigned int i=0; i<4; i++)
        mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%4);
    else if (N == 3) /* fast path for triangles */
      for (unsigned int i=0; i<3; i++)
        mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%3);
    else /* fast path for general polygons */
      for (unsigned int i=0; i<N; i++)
        mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%N);
  }
}

#if defined(ISPC)
void updateSubMeshEdgeLevelBufferTask (int taskIndex, int threadIndex,  ISPCSubdivMesh* mesh, const Vec3fa& cam_pos )
{
  const unsigned int size = mesh->numFaces;
  const unsigned int startID = ((taskIndex+0)*size)/taskCount;
  const unsigned int endID   = ((taskIndex+1)*size)/taskCount;
  updateEdgeLevelBuffer(mesh,cam_pos,startID,endID);
}
void updateMeshEdgeLevelBufferTask (int taskIndex, int threadIndex,  ISPCScene* scene_in, const Vec3fa& cam_pos )
{
  ISPCGeometry* geometry = g_ispc_scene->geometries[taskIndex];
  if (geometry->type != SUBDIV_MESH) return;
  ISPCSubdivMesh* mesh = (ISPCSubdivMesh*) geometry;
  if (mesh->numFaces < 10000) {
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
    rtcUpdateGeometryBuffer(geometry->geometry, RTC_BUFFER_TYPE_LEVEL, 0);
  }
  rtcCommitGeometry(geometry->geometry);
}
#endif

void updateEdgeLevels(ISPCScene* scene_in, const Vec3fa& cam_pos)
{
  /* first update small meshes */
#if defined(ISPC)
  parallel_for(size_t(0),size_t( scene_in->numGeometries ),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      updateMeshEdgeLevelBufferTask((int)i,threadIndex,scene_in,cam_pos);
  }); 
#endif

  /* now update large meshes */
  for (unsigned int g=0; g<scene_in->numGeometries; g++)
  {
    ISPCGeometry* geometry = g_ispc_scene->geometries[g];
    if (geometry->type != SUBDIV_MESH) continue;
    ISPCSubdivMesh* mesh = (ISPCSubdivMesh*) geometry;
#if defined(ISPC)
    if (mesh->numFaces < 10000) continue;
    parallel_for(size_t(0),size_t( (mesh->numFaces+4095)/4096 ),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      updateSubMeshEdgeLevelBufferTask((int)i,threadIndex,mesh,cam_pos);
  }); 
#else
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
#endif
    rtcUpdateGeometryBuffer(geometry->geometry, RTC_BUFFER_TYPE_LEVEL, 0);
    rtcCommitGeometry(geometry->geometry);
  }
}

RTCScene convertScene(ISPCScene* scene_in)
{
  for (unsigned int i=0; i<scene_in->numGeometries; i++)
  {
    ISPCGeometry* geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      data.subdiv_mode = true; break;
    }
  }

  RTCFeatureFlags feature_mask = RTC_FEATURE_FLAG_NONE;
  RTCScene scene_out = ConvertScene(g_device, g_ispc_scene, RTC_BUILD_QUALITY_MEDIUM, RTC_SCENE_FLAG_NONE, &feature_mask);
  if (g_use_scene_features) g_feature_mask = feature_mask;
  rtcSetSceneProgressMonitorFunction(scene_out,monitorProgressFunction,nullptr);

  /* commit changes to scene */
  return scene_out;
}

AffineSpace3fa calculate_interpolated_space (ISPCInstance* instance, float gtime)
{
  if (instance->numTimeSteps == 1)
    return AffineSpace3fa(instance->spaces[0]);

   /* calculate time segment itime and fractional time ftime */
  const int time_segments = instance->numTimeSteps-1;
  const float time = gtime*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  return (1.0f-ftime)*AffineSpace3fa(instance->spaces[itime+0]) + ftime*AffineSpace3fa(instance->spaces[itime+1]);
}

AffineSpace3fa calculate_interpolated_space (ISPCInstanceArray* instanceArray, unsigned int primID, float gtime)
{
  if (instanceArray->numTimeSteps == 1)
    return AffineSpace3fa(instanceArray->spaces_array[0][primID]);

   /* calculate time segment itime and fractional time ftime */
  const int time_segments = instanceArray->numTimeSteps-1;
  const float time = gtime*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  return (1.0f-ftime)*AffineSpace3fa(instanceArray->spaces_array[itime+0][primID]) + ftime*AffineSpace3fa(instanceArray->spaces_array[itime+1][primID]);
}

typedef ISPCInstance* ISPCInstancePtr;

unsigned int postIntersect(const TutorialData& data, const Ray& ray, DifferentialGeometry& dg)
{
  AffineSpace3fa local2world = AffineSpace3fa::scale(Vec3fa(1));
  ISPCGeometry** geometries = data.ispc_scene->geometries;
  
  for (int i=0; i<RTC_MAX_INSTANCE_LEVEL_COUNT; i++)
  {
    const unsigned int instID = ray.instID[i];
    if (instID == -1) break;

    if (geometries[instID]->type == INSTANCE) {
      ISPCInstance* instance = (ISPCInstancePtr) geometries[instID];
      local2world = local2world * calculate_interpolated_space(instance,ray.time());
      assert(instance->child->type == GROUP);
      geometries = ((ISPCGroup*)instance->child)->geometries;
    }
#if defined(RTC_GEOMETRY_INSTANCE_ARRAY)
    else if (geometries[instID]->type == INSTANCE_ARRAY) {
      ISPCInstanceArray* instanceArray = (ISPCInstanceArray*) geometries[instID];
      local2world = local2world * calculate_interpolated_space(instanceArray, ray.instPrimID[i],ray.time());
      assert(instanceArray->child->type == GROUP);
      geometries = ((ISPCGroup*)instanceArray->child)->geometries;
    }
#endif

  }

  ISPCGeometry* mesh = geometries[ray.geomID];
  unsigned int materialID = mesh->materialID;
  
  dg.Ng = xfmVector(local2world,dg.Ng);
  dg.Ns = xfmVector(local2world,dg.Ns);
  
  return materialID;
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* task that renders a single screen tile */
void renderPixelStandard(const TutorialData& data,
                         int x, int y, 
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         RayStats& stats,
                         const RTCFeatureFlags feature_mask)
{
  /* initialize sampler */
  RandomSampler sampler;
  RandomSampler_init(sampler, (int)x, (int)y, 0);

  /* initialize ray */
  float ray_time = data.motion_blur ? RandomSampler_get1D(sampler) : time;
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf, ray_time);

  /* intersect ray with scene */
  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.flags = data.iflags_coherent;
#if RTC_MIN_WIDTH
  args.minWidthDistanceFactor = 0.5f*data.min_width/width;
#endif
  args.feature_mask = feature_mask;
  
  rtcIntersect1(data.scene,RTCRayHit_(ray),&args);
  RayStats_addRay(stats);


  /* shade background black */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
    pixels[y*width+x] = 0;
    return;
  }

  /* shade all rays that hit something */
  Vec3fa color = Vec3fa(0.5f);

  /* compute differential geometry */
  DifferentialGeometry dg;
  dg.geomID = ray.geomID;
  dg.primID = ray.primID;
  dg.u = ray.u;
  dg.v = ray.v;
  dg.P  = ray.org+ray.tfar*ray.dir;
  dg.Ng = ray.Ng;
  dg.Ns = ray.Ng;

#if 0
  if (data.use_smooth_normals)
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID) // FIXME: workaround for ISPC bug, location reached with empty execution mask
    {
      Vec3fa dPdu,dPdv;
      auto geomID = ray.geomID; {
        rtcInterpolate1(rtcGetGeometry(data.scene,geomID),ray.primID,ray.u,ray.v,RTC_BUFFER_TYPE_VERTEX,0,nullptr,&dPdu.x,&dPdv.x,3);
      }
      dg.Ns = cross(dPdv,dPdu);
    }
#endif

  int materialID = postIntersect(data,ray,dg);
  dg.Ng = face_forward(ray.dir,normalize(dg.Ng));
  dg.Ns = face_forward(ray.dir,normalize(dg.Ns));

  /* shade */
  if (data.ispc_scene->materials[materialID]->type == MATERIAL_OBJ) {
    ISPCOBJMaterial* material = (ISPCOBJMaterial*) data.ispc_scene->materials[materialID];
    color = Vec3fa(material->Kd);
  }

  color = color*dot(neg(ray.dir),dg.Ns);

  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
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
  const int t = taskIndex;
  const unsigned int tileY = t / numTilesX;
  const unsigned int tileX = t - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex],g_feature_mask);
  }
}

Vec3fa old_p;

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  TutorialData_Constructor(&data);
  old_p = Vec3fa(1E10);
}

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  TutorialData ldata = data;

#if defined(USE_SPECIALIZATION_CONSTANTS)
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh) {
    cgh.set_specialization_constant<spec_feature_mask>(g_feature_mask);
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item, sycl::kernel_handler kh) {
      const unsigned int x = item.get_global_id(1); if (x >= width ) return;
      const unsigned int y = item.get_global_id(0); if (y >= height) return;
      RayStats stats;
      const RTCFeatureFlags feature_mask = kh.get_specialization_constant<spec_feature_mask>();
      renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats,feature_mask);
    });
  });
  global_gpu_queue->wait_and_throw();
#else
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh) {
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
      const unsigned int x = item.get_global_id(1); if (x >= width ) return;
      const unsigned int y = item.get_global_id(0); if (y >= height) return;
      RayStats stats;
      const RTCFeatureFlags feature_mask = RTC_FEATURE_FLAG_ALL;
      renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats,feature_mask);
    });
  });
  global_gpu_queue->wait_and_throw();
#endif

  const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
  const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
  const double dt = (t1-t0)*1E-9;
  ((ISPCCamera*)&camera)->render_time = dt;
 
#else
  /* render image */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
#endif
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  bool camera_changed = g_changed; g_changed = false;

  /* create scene */
  if (data.scene == nullptr) {
    g_scene = data.scene = convertScene(g_ispc_scene);
    if (data.subdiv_mode) updateEdgeLevels(g_ispc_scene, camera.xfm.p);
    rtcCommitScene (data.scene);
    old_p = camera.xfm.p;
  }

  else
  {
    /* check if camera changed */
    if (ne(camera.xfm.p,old_p)) {
      camera_changed = true;
      old_p = camera.xfm.p;
    }

    /* update edge levels if camera changed */
    if (camera_changed && data.subdiv_mode) {
      updateEdgeLevels(g_ispc_scene,camera.xfm.p);
      rtcCommitScene (data.scene);
    }

    if (g_animation_mode)
      UpdateScene(g_ispc_scene, time);
  }
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
