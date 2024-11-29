// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "instanced_geometry_device.isph"

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_INSTANCE | \
  RTC_FEATURE_FLAG_32_BIT_RAY_MASK

const uniform int numPhi = 5;
const uniform int numTheta = 2*numPhi;

RTCScene g_scene  = NULL;
RTCGeometry g_instance0 = NULL;
RTCGeometry g_instance1 = NULL;
RTCGeometry g_instance2 = NULL;
RTCGeometry g_instance3 = NULL;
uniform TutorialData data;

uniform unsigned int createTriangulatedSphere (RTCScene scene, const uniform Vec3f& p, uniform float r)
{
  /* create triangle mesh */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* map triangle and vertex buffers */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),numTheta*(numPhi+1));
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2*numTheta*(numPhi-1));

  /* create sphere */
  uniform int tri = 0;
  const uniform float rcpNumTheta = rcp((uniform float)numTheta);
  const uniform float rcpNumPhi   = rcp((uniform float)numPhi);
  for (uniform int phi=0; phi<=numPhi; phi++)
  {
    for (uniform int theta=0; theta<numTheta; theta++)
    {
      const uniform float phif   = phi*pi*rcpNumPhi;
      const uniform float thetaf = theta*2.0f*pi*rcpNumTheta;

      uniform Vertex& v = vertices[phi*numTheta+theta];
      v.x = p.x + r*sin(phif)*sin(thetaf);
      v.y = p.y + r*cos(phif);
      v.z = p.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (uniform int theta=1; theta<=numTheta; theta++)
    {
      uniform int p00 = (phi-1)*numTheta+theta-1;
      uniform int p01 = (phi-1)*numTheta+theta%numTheta;
      uniform int p10 = phi*numTheta+theta-1;
      uniform int p11 = phi*numTheta+theta%numTheta;

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
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* creates a ground plane */
uniform unsigned int createGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  TutorialData_Constructor(&data);
  
  /* create scene */
  data.g_scene = g_scene = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(data.g_scene,RTC_BUILD_QUALITY_LOW);
  rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC);

  /* create scene with 4 triangulated spheres */
  data.g_scene1 = rtcNewScene(g_device);
  createTriangulatedSphere(data.g_scene1,make_Vec3f( 0, 0,+1),0.5f);
  createTriangulatedSphere(data.g_scene1,make_Vec3f(+1, 0, 0),0.5f);
  createTriangulatedSphere(data.g_scene1,make_Vec3f( 0, 0,-1),0.5f);
  createTriangulatedSphere(data.g_scene1,make_Vec3f(-1, 0, 0),0.5f);
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
  data.colors[4*0+0] = make_Vec3f(0.25f, 0.f, 0.f);
  data.colors[4*0+1] = make_Vec3f(0.50f, 0.f, 0.f);
  data.colors[4*0+2] = make_Vec3f(0.75f, 0.f, 0.f);
  data.colors[4*0+3] = make_Vec3f(1.00f, 0.f, 0.f);

  data.colors[4*1+0] = make_Vec3f(0.f, 0.25f, 0.f);
  data.colors[4*1+1] = make_Vec3f(0.f, 0.50f, 0.f);
  data.colors[4*1+2] = make_Vec3f(0.f, 0.75f, 0.f);
  data.colors[4*1+3] = make_Vec3f(0.f, 1.00f, 0.f);

  data.colors[4*2+0] = make_Vec3f(0.f, 0.f, 0.25f);
  data.colors[4*2+1] = make_Vec3f(0.f, 0.f, 0.50f);
  data.colors[4*2+2] = make_Vec3f(0.f, 0.f, 0.75f);
  data.colors[4*2+3] = make_Vec3f(0.f, 0.f, 1.00f);

  data.colors[4*3+0] = make_Vec3f(0.25f, 0.25f, 0.f);
  data.colors[4*3+1] = make_Vec3f(0.50f, 0.50f, 0.f);
  data.colors[4*3+2] = make_Vec3f(0.75f, 0.75f, 0.f);
  data.colors[4*3+3] = make_Vec3f(1.00f, 1.00f, 0.f);
}

/* task that renders a single screen tile */
Vec3f renderPixel(const uniform TutorialData& data, float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  uniform RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
  
  rtcTraversableIntersectV(data.g_traversable,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* calculate shading normal in world space */
    Vec3f Ns = ray.Ng;
    if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID)
    {
      AffineSpace3f xfm;
      rtcGetGeometryTransformFromTraversable(data.g_traversable,ray.instID[0],0.0f,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,&xfm);
      Ns = xfmNormal(xfm,Ns);
      //Ns = xfmVector(data.normal_xfm[ray.instID[0]],Ns);
    }
    Ns = normalize(Ns);

    /* calculate diffuse color of geometries */
    Vec3f diffuse = make_Vec3f(1,1,1);
    if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID)
      diffuse = data.colors[4*ray.instID[0]+ray.geomID];
    color = color + diffuse*0.5;

    /* initialize shadow ray */
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    uniform RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
    
    rtcTraversableOccludedV(data.g_traversable,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
  }
  return color;
}

void renderPixelStandard(const uniform TutorialData& data,
                         int x, int y, 
                         uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const uniform float time,
                         const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  /* calculate pixel color */
  Vec3f color = renderPixel(data, (float)x,(float)y,camera, stats);
  
  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
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

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex]);
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
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
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
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
#endif
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  uniform float t0 = 0.7f*time;
  uniform float t1 = 1.5f*time;

  /* rotate instances around themselves */
  uniform LinearSpace3f xfm;
  xfm.vx = make_Vec3f(cos(t1),0,sin(t1));
  xfm.vy = make_Vec3f(0,1,0);
  xfm.vz = make_Vec3f(-sin(t1),0,cos(t1));

  /* calculate transformations to move instances in circle */
  for (uniform int i=0; i<4; i++) {
    uniform float t = t0+i*2.0f*M_PI/4.0f;
    data.instance_xfm[i] = make_AffineSpace3f(xfm,2.2f*make_Vec3f(+cos(t),0.0f,+sin(t)));
  }

  /* calculate transformations to properly transform normals */
  for (uniform int i=0; i<4; i++)
    data.normal_xfm[i] = transposed(rcp(data.instance_xfm[i].l));

  /* set instance transformations */
  rtcSetGeometryTransform(g_instance0,0,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&data.instance_xfm[0]);
  rtcSetGeometryTransform(g_instance1,0,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&data.instance_xfm[1]);
  rtcSetGeometryTransform(g_instance2,0,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&data.instance_xfm[2]);
  rtcSetGeometryTransform(g_instance3,0,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&data.instance_xfm[3]);

  /* update scene */
  rtcCommitGeometry(g_instance0);
  rtcCommitGeometry(g_instance1);
  rtcCommitGeometry(g_instance2);
  rtcCommitGeometry(g_instance3);
  rtcCommitScene (data.g_scene);
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  TutorialData_Destructor(&data);
}
