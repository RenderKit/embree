// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "curve_geometry_device.isph"
#include "../common/tutorial/optics.isph"

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_CONE_LINEAR_CURVE | \
  RTC_FEATURE_FLAG_ROUND_LINEAR_CURVE | \
  RTC_FEATURE_FLAG_FLAT_BSPLINE_CURVE | \
  RTC_FEATURE_FLAG_ROUND_BSPLINE_CURVE | \
  RTC_FEATURE_FLAG_NORMAL_ORIENTED_BSPLINE_CURVE | \
  RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE | \
  RTC_FEATURE_FLAG_FLAT_CATMULL_ROM_CURVE | \
  RTC_FEATURE_FLAG_ROUND_CATMULL_ROM_CURVE | \
  RTC_FEATURE_FLAG_NORMAL_ORIENTED_CATMULL_ROM_CURVE
  
/* scene data */
RTCScene  g_scene  = NULL;
uniform TutorialData data;

Vec3f interpolate_linear(const uniform TutorialData& data, unsigned int primID, float u)
{
  const Vec3fa c0 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+1];
  const Vec3fa c1 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+2];
  return make_Vec3f(c0*(1.0f-u) + c1*u);
}

Vec3f interpolate_bspline(const uniform TutorialData& data, unsigned int primID, float u)
{
  const Vec3fa c0 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+0];
  const Vec3fa c1 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+1];
  const Vec3fa c2 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+2];
  const Vec3fa c3 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+3];
  const float t  = u;
  const float s  = 1.0f - u;
  const float n0 = s*s*s;
  const float n1 = (4.0f*(s*s*s)+(t*t*t)) + (12.0f*((s*t)*s) + 6.0f*((t*s)*t));
  const float n2 = (4.0f*(t*t*t)+(s*s*s)) + (12.0f*((t*s)*t) + 6.0f*((s*t)*s));
  const float n3 = t*t*t;
  return make_Vec3f((1.0f/6.0f)*(n0*c0 + n1*c1 + n2*c2 + n3*c3));
}

Vec3f interpolate_catmull_rom(const uniform TutorialData& data, unsigned int primID, float u)
{
  const Vec3fa c0 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+0];
  const Vec3fa c1 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+1];
  const Vec3fa c2 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+2];
  const Vec3fa c3 = ((Vec3fa* uniform) data.hair_vertex_colors)[primID+3];
  const float t  = u;
  const float s  = 1.0f - u;
  const float n0 = - t * s * s;
  const float n1 = 2.0f + t * t * (3.0f * t - 5.0f);
  const float n2 = 2.0f + s * s * (3.0f * s - 5.0f);
  const float n3 = - s * t * t;
  return make_Vec3f(0.5f*(n0*c0 + n1*c1 + n2*c2 + n3*c3));
}

/* add hair geometry */
uniform unsigned int addCurve (RTCScene scene, uniform RTCGeometryType gtype, const uniform Vec4f& pos)
{
  RTCGeometry geom = rtcNewGeometry (g_device, gtype);
  rtcSetGeometryVertexAttributeCount(geom,1);
  
  if (gtype == RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE || gtype == RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE || gtype == RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE)
    rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT,   data.hair_indices_linear,0, sizeof(uniform unsigned int), NUM_CURVES);
  else
    rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT,   data.hair_indices,       0, sizeof(uniform unsigned int), NUM_CURVES);
  
  uniform Vec4f* uniform verts = (uniform Vec4f* uniform)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(uniform Vec4f), NUM_VERTICES);
  for (uniform int i = 0; i < NUM_VERTICES; i++) {
    verts[i] = pos + data.hair_vertices[i];
  }
  if (gtype == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE ||
      gtype == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE ||
      gtype == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE) {
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3, data.hair_normals, 0, sizeof(uniform Vec3fa), NUM_VERTICES);
  }
  
  if (gtype == RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE || gtype == RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE) {
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FLAGS, 0, RTC_FORMAT_UCHAR, data.hair_flags_linear, 0, sizeof(uniform int8), NUM_CURVES);
  }

  rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3, data.hair_vertex_colors, 0, sizeof(uniform Vec3fa),       NUM_VERTICES);
  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vertex), 4);
  vertices[0].x = -15; vertices[0].y = -2; vertices[0].z = -15;
  vertices[1].x = -15; vertices[1].y = -2; vertices[1].z = +15;
  vertices[2].x = +15; vertices[2].y = -2; vertices[2].z = -15;
  vertices[3].x = +15; vertices[3].y = -2; vertices[3].z = +15;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(uniform Triangle), 2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create scene */
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);

  /* add ground plane */
  addGroundPlane(data.g_scene);

  /* add curves */
  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE, make_Vec4f(-5.5f, 0.0f, 3.f, 0.0f));
  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE, make_Vec4f(-2.5f, 0.0f, 3.f, 0.0f));
  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE, make_Vec4f(0.5f, 0.0f, 3.f, 0.0f));
  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE, make_Vec4f(3.5f, 0.0f, 3.f, 0.0f));
  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE, make_Vec4f(+6.0f, 0.0f, 3.f, 0.0f));

  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE, make_Vec4f(-4.5f, 0.0f, -2.f, 0.0f));
  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE, make_Vec4f(-1.5f, 0.0f, -2.f, 0.0f));
  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE, make_Vec4f(1.5f, 0.0f, -2.f, 0.0f));
  addCurve(data.g_scene, RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE, make_Vec4f(+4.5f, 0.0f, -2.f, 0.0f));

  /* commit changes to scene */
  rtcCommitScene (data.g_scene);
}

/* task that renders a single screen tile */
void renderPixelStandard(const uniform TutorialData& data,
                         int x, int y, 
                         int* pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const uniform float time,
                         const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  uniform RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
  rtcIntersectV(data.g_scene,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* interpolate diffuse color */
    Vec3f diffuse = make_Vec3f(1.0f,0.0f,0.0f);
    if (ray.geomID > 0)
    {
      switch (ray.geomID) {
      case 1: case 2: case 6: diffuse = interpolate_linear(data,ray.primID,ray.u); break;
      case 3: case 4: case 5: diffuse = interpolate_bspline(data,ray.primID,ray.u); break;
      case 7: case 8: case 9: diffuse = interpolate_catmull_rom(data,ray.primID,ray.u); break;
      }

      diffuse = 0.5f*diffuse;
    }

    /* calculate smooth shading normal */
    Vec3f Ng = normalize(ray.Ng);
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

    /* trace shadow ray */
    uniform RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
    rtcOccludedV(data.g_scene,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f) {
      Vec3f r = normalize(reflect(ray.dir,Ng));
      float s = pow(clamp(dot(r,lightDir),0.0f,1.0f),10.0f);
      float d = clamp(-dot(lightDir,Ng),0.0f,1.0f);
      color = color + diffuse*d + 0.5f*make_Vec3f(s);
    }
  }

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
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseScene (data.g_scene); data.g_scene = NULL;
  TutorialData_Destructor(&data);
}
