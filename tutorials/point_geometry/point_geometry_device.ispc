// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "point_geometry_device.isph"

RTCScene  g_scene  = NULL;
uniform TutorialData data;

/* add point geometry */
void addPoints (RTCScene scene, uniform RTCGeometryType gtype, const uniform Vec3f& pos)
{
  RandomSampler rng;
  RandomSampler_init(rng, 42);

#define COORD extract(RandomSampler_get1D(rng), 0) * 4.f - 2.f
#define RADIUS extract(RandomSampler_get1D(rng), 0) * 0.13f + 0.02f
#define COLOR extract(RandomSampler_get1D(rng), 0)
#define NORMAL extract(RandomSampler_get1D(rng), 0) * 2.f - 1.f

  RTCGeometry geom = rtcNewGeometry (g_device, gtype);
  uniform Vec4f* uniform point_vertices = (uniform Vec4f* uniform)rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(uniform Vec4f), NUM_POINTS);

  for (uniform int i = 0; i < NUM_POINTS; i++)
  {
    const uniform float vx = COORD;
    const uniform float vy = COORD;
    const uniform float vz = COORD;
    const uniform float vr = RADIUS;
    point_vertices[i] = make_Vec4f(pos.x,pos.y,pos.z,0.0f) + make_Vec4f(vx, vy, vz, vr);
    const uniform float cr = COLOR;
    const uniform float cg = COLOR;
    const uniform float cb = COLOR;
    data.point_colors[i] = make_Vec3f(cr,cg,cb);
  }

  if (gtype == RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT) {
    uniform Vec3fa* uniform point_normals = (uniform Vec3fa* uniform)rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vec3fa), NUM_POINTS);
    for (uniform int i = 0; i < NUM_POINTS; i++) {
      const uniform float nx = NORMAL;
      const uniform float ny = NORMAL;
      const uniform float nz = NORMAL;
      point_normals[i] = make_Vec3fa(nx,ny,nz);
      point_normals[i] = normalize(point_normals[i]);
    }
  }

  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vertex), 4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

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
  addGroundPlane(g_scene);

  /* add curve */
  addPoints(g_scene, RTC_GEOMETRY_TYPE_SPHERE_POINT,        make_Vec3f( 0.0f, 0.0f, 0.0f));
  addPoints(g_scene, RTC_GEOMETRY_TYPE_DISC_POINT,          make_Vec3f( 5.0f, 0.0f, 0.0f));
  addPoints(g_scene, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT, make_Vec3f(-5.0f, 0.0f, 0.0f));

  /* commit changes to scene */
  rtcCommitScene (g_scene);
}

/* task that renders a single screen tile */
void renderPixelStandard(const uniform TutorialData& data,
                          int x, int y, 
                          uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const float time,
                          const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);

  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  rtcIntersectV(data.g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* interpolate diffuse color */
    Vec3f diffuse = data.point_colors[ray.geomID ? ray.primID : 0];

    /* calculate smooth shading normal */
    Vec3f Ng = normalize(ray.Ng);
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

    /* trace shadow ray */
    rtcOccludedV(data.g_scene,&context,RTCRay_(shadow));
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

/* task that renders a single screen tile */
task void renderTileTask(uniform int* uniform pixels,
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

export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
{
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
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
  TutorialData_Destructor(&data);
}
