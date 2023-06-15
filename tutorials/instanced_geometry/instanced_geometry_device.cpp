// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "instanced_geometry_device.h"

namespace embree {

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_INSTANCE | \
  RTC_FEATURE_FLAG_32_BIT_RAY_MASK

const int numPhi = 5;
const int numTheta = 2*numPhi;

RTCScene g_scene  = nullptr;
RTCGeometry g_instance0 = nullptr;
RTCGeometry g_instance1 = nullptr;
RTCGeometry g_instance2 = nullptr;
RTCGeometry g_instance3 = nullptr;
TutorialData data;

unsigned int createTriangulatedSphere (RTCScene scene, const Vec3fa& p, float r)
{
  /* create triangle mesh */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* map triangle and vertex buffers */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),numTheta*(numPhi+1));
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2*numTheta*(numPhi-1));

  /* create sphere */
  int tri = 0;
  const float rcpNumTheta = rcp((float)numTheta);
  const float rcpNumPhi   = rcp((float)numPhi);
  for (int phi=0; phi<=numPhi; phi++)
  {
    for (int theta=0; theta<numTheta; theta++)
    {
      const float phif   = phi*float(pi)*rcpNumPhi;
      const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;

      Vertex& v = vertices[phi*numTheta+theta];
      v.x = p.x + r*sin(phif)*sin(thetaf);
      v.y = p.y + r*cos(phif);
      v.z = p.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (int theta=1; theta<=numTheta; theta++)
    {
      int p00 = (phi-1)*numTheta+theta-1;
      int p01 = (phi-1)*numTheta+theta%numTheta;
      int p10 = phi*numTheta+theta-1;
      int p11 = phi*numTheta+theta%numTheta;

      if (phi > 1) {
        triangles[tri].v0 = p10;
        triangles[tri].v1 = p01;
        triangles[tri].v2 = p00;
        tri++;
      }

      if (phi < numPhi) {
        triangles[tri].v0 = p11;
        triangles[tri].v1 = p01;
        triangles[tri].v2 = p10;
        tri++;
      }
    }
  }

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* creates a ground plane */
unsigned int createGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  TutorialData_Constructor(&data);
  
  /* create scene */
  data.g_scene = g_scene = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(data.g_scene,RTC_BUILD_QUALITY_LOW);
  rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC);

  /* create scene with 4 triangulated spheres */
  data.g_scene1 = rtcNewScene(g_device);
  createTriangulatedSphere(data.g_scene1,Vec3fa( 0, 0,+1),0.5f);
  createTriangulatedSphere(data.g_scene1,Vec3fa(+1, 0, 0),0.5f);
  createTriangulatedSphere(data.g_scene1,Vec3fa( 0, 0,-1),0.5f);
  createTriangulatedSphere(data.g_scene1,Vec3fa(-1, 0, 0),0.5f);
  rtcCommitScene (data.g_scene1);

  /* instantiate geometry */
  g_instance0 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryMask(g_instance0, 0x80); // test high instance mask bits
  rtcSetGeometryInstancedScene(g_instance0,data.g_scene1);
  rtcSetGeometryTimeStepCount(g_instance0,1);
  g_instance1 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance1,data.g_scene1);
  rtcSetGeometryTimeStepCount(g_instance1,1);
  g_instance2 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance2,data.g_scene1);
  rtcSetGeometryTimeStepCount(g_instance2,1);
  g_instance3 = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance3,data.g_scene1);
  rtcSetGeometryTimeStepCount(g_instance3,1);
  rtcAttachGeometry(data.g_scene,g_instance0);
  rtcAttachGeometry(data.g_scene,g_instance1);
  rtcAttachGeometry(data.g_scene,g_instance2);
  rtcAttachGeometry(data.g_scene,g_instance3);
  rtcReleaseGeometry(g_instance0);
  rtcReleaseGeometry(g_instance1);
  rtcReleaseGeometry(g_instance2);
  rtcReleaseGeometry(g_instance3);
  createGroundPlane(data.g_scene);

  /* set all colors */
  data.colors[4*0+0] = Vec3fa(0.25f, 0.f, 0.f);
  data.colors[4*0+1] = Vec3fa(0.50f, 0.f, 0.f);
  data.colors[4*0+2] = Vec3fa(0.75f, 0.f, 0.f);
  data.colors[4*0+3] = Vec3fa(1.00f, 0.f, 0.f);

  data.colors[4*1+0] = Vec3fa(0.f, 0.25f, 0.f);
  data.colors[4*1+1] = Vec3fa(0.f, 0.50f, 0.f);
  data.colors[4*1+2] = Vec3fa(0.f, 0.75f, 0.f);
  data.colors[4*1+3] = Vec3fa(0.f, 1.00f, 0.f);

  data.colors[4*2+0] = Vec3fa(0.f, 0.f, 0.25f);
  data.colors[4*2+1] = Vec3fa(0.f, 0.f, 0.50f);
  data.colors[4*2+2] = Vec3fa(0.f, 0.f, 0.75f);
  data.colors[4*2+3] = Vec3fa(0.f, 0.f, 1.00f);

  data.colors[4*3+0] = Vec3fa(0.25f, 0.25f, 0.f);
  data.colors[4*3+1] = Vec3fa(0.50f, 0.50f, 0.f);
  data.colors[4*3+2] = Vec3fa(0.75f, 0.75f, 0.f);
  data.colors[4*3+3] = Vec3fa(1.00f, 1.00f, 0.f);
}

/* task that renders a single screen tile */
Vec3fa renderPixel(const TutorialData& data, float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
  rtcIntersect1(data.g_scene,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* calculate shading normal in world space */
    Vec3fa Ns = ray.Ng;
    if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID)
    {
      AffineSpace3fa xfm;
      rtcGetGeometryTransformFromScene(data.g_scene,ray.instID[0],0.0f,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,&xfm);
      Ns = xfmNormal(xfm,Ns);
      //Ns = xfmVector(data.normal_xfm[ray.instID[0]],Ns);
    }
    Ns = normalize(Ns);

    /* calculate diffuse color of geometries */
    Vec3fa diffuse = Vec3fa(1,1,1);
    if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID)
      diffuse = data.colors[4*ray.instID[0]+ray.geomID];
    color = color + diffuse*0.5;

    /* initialize shadow ray */
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
    
    rtcOccluded1(data.g_scene,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
  }
  return color;
}

void renderPixelStandard(const TutorialData& data,
                         int x, int y, 
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera, RayStats& stats)
{
  /* calculate pixel color */
  Vec3fa color = renderPixel(data, (float)x,(float)y,camera, stats);
  
  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
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

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex]);
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
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
  /* render all pixels */
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  TutorialData ldata = data;
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
      const unsigned int x = item.get_global_id(1); if (x >= width ) return;
      const unsigned int y = item.get_global_id(0); if (y >= height) return;
      RayStats stats;
      renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats);
    });
  });
  global_gpu_queue->wait_and_throw();

  const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
  const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
  const double dt = (t1-t0)*1E-9;
  ((ISPCCamera*)&camera)->render_time = dt;
  
#else
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
  float t0 = 0.7f*time;
  float t1 = 1.5f*time;

  /* rotate instances around themselves */
  LinearSpace3fa xfm;
  xfm.vx = Vec3fa(cos(t1),0,sin(t1));
  xfm.vy = Vec3fa(0,1,0);
  xfm.vz = Vec3fa(-sin(t1),0,cos(t1));

  /* calculate transformations to move instances in circle */
  for (int i=0; i<4; i++) {
    float t = t0+i*2.0f*float(M_PI)/4.0f;
    data.instance_xfm[i] = AffineSpace3fa(xfm,2.2f*Vec3fa(+cos(t),0.0f,+sin(t)));
  }

  /* calculate transformations to properly transform normals */
  for (int i=0; i<4; i++)
    data.normal_xfm[i] = transposed(rcp(data.instance_xfm[i].l));

  /* set instance transformations */
  rtcSetGeometryTransform(g_instance0,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&data.instance_xfm[0]);
  rtcSetGeometryTransform(g_instance1,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&data.instance_xfm[1]);
  rtcSetGeometryTransform(g_instance2,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&data.instance_xfm[2]);
  rtcSetGeometryTransform(g_instance3,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&data.instance_xfm[3]);

  /* update scene */
  rtcCommitGeometry(g_instance0);
  rtcCommitGeometry(g_instance1);
  rtcCommitGeometry(g_instance2);
  rtcCommitGeometry(g_instance3);
  rtcCommitScene (data.g_scene);
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
