// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/optics.h"

namespace embree {

/* scene data */
RTCScene  g_scene  = nullptr;

#define NUM_VERTICES 9
#define NUM_CURVES 6

#define W 2.0f

float hair_vertices[NUM_VERTICES][4] =
{
  { -1.0f, 0.0f, -   W, 0.2f },

  { +0.0f,-1.0f, +0.0f, 0.2f },
  { +1.0f, 0.0f, +   W, 0.2f },
  { -1.0f, 0.0f, +   W, 0.2f },
  { +0.0f,+1.0f, +0.0f, 0.6f },
  { +1.0f, 0.0f, -   W, 0.2f },
  { -1.0f, 0.0f, -   W, 0.2f },

  { +0.0f,-1.0f, +0.0f, 0.2f },
  { +1.0f, 0.0f, +   W, 0.2f },
};

float hair_normals[NUM_VERTICES][4] =
{
  { -1.0f,  0.0f, 0.0f, 0.0f },

  {  0.0f, +1.0f, 0.0f, 0.0f },
  { +1.0f,  0.0f, 0.0f, 0.0f },
  {  0.0f, -1.0f, 0.0f, 0.0f },
  { -1.0f,  0.0f, 0.0f, 0.0f },
  {  0.0f, +1.0f, 0.0f, 0.0f },
  { +1.0f,  0.0f, 0.0f, 0.0f },

  {  0.0f, -1.0f, 0.0f, 0.0f },
  { -1.0f,  0.0f, 0.0f, 0.0f },
};

float hair_vertex_colors[NUM_VERTICES][4] =
{
  {  1.0f,  1.0f,  0.0f, 0.0f },

  {  1.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  0.0f, 0.0f },
  {  0.0f,  0.0f,  1.0f, 0.0f },
  {  1.0f,  1.0f,  1.0f, 0.0f },
  {  1.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  0.0f, 0.0f },

  {  1.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  0.0f, 0.0f },
};

unsigned int hair_indices[NUM_CURVES] = {
  0, 1, 2, 3, 4, 5
};

unsigned int hair_indices_linear[NUM_CURVES] = {
  1, 2, 3, 4, 5, 6
};

char hair_flags_linear[NUM_CURVES] = {
  0x3, 0x3, 0x3, 0x3, 0x3, 0x3
};

/* add hair geometry */
unsigned int addCurve (RTCScene scene, RTCGeometryType gtype, const Vec4f& pos)
{
  RTCGeometry geom = rtcNewGeometry (g_device, gtype);
  rtcSetGeometryVertexAttributeCount(geom,1);

  if (gtype == RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE || gtype == RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE || gtype == RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE)
    rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT,   hair_indices_linear,0, sizeof(unsigned int), NUM_CURVES);
  else
    rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT,   hair_indices,       0, sizeof(unsigned int), NUM_CURVES);
      
  Vec4f* verts = (Vec4f*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(Vec4f), NUM_VERTICES);
  for (int i = 0; i < NUM_VERTICES; i++) {
    verts[i] = pos + Vec4f(hair_vertices[i][0],hair_vertices[i][1], hair_vertices[i][2], hair_vertices[i][3]);
  }
  if (gtype == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE ||
      gtype == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE ||
      gtype == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE) {
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3, hair_normals, 0, sizeof(Vec3fa), NUM_VERTICES);
  }
  
  if (gtype == RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE || gtype == RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE) {
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FLAGS, 0, RTC_FORMAT_UCHAR, hair_flags_linear, 0, sizeof(char), NUM_CURVES);
  }

  rtcSetSharedGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 0, RTC_FORMAT_FLOAT3, hair_vertex_colors, 0, sizeof(Vec3fa),       NUM_VERTICES);
  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), 4);
  vertices[0].x = -15; vertices[0].y = -2; vertices[0].z = -15;
  vertices[1].x = -15; vertices[1].y = -2; vertices[1].z = +15;
  vertices[2].x = +15; vertices[2].y = -2; vertices[2].z = -15;
  vertices[3].x = +15; vertices[3].y = -2; vertices[3].z = +15;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* add curves */
  addCurve(g_scene, RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE, Vec4f(-5.5f, 0.0f, 3.f, 0.0f));
  addCurve(g_scene, RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE, Vec4f(-2.5f, 0.0f, 3.f, 0.0f));
  addCurve(g_scene, RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE, Vec4f(0.5f, 0.0f, 3.f, 0.0f));
  addCurve(g_scene, RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE, Vec4f(3.5f, 0.0f, 3.f, 0.0f));
  addCurve(g_scene, RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE, Vec4f(+6.0f, 0.0f, 3.f, 0.0f));

  addCurve(g_scene, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE, Vec4f(-4.5f, 0.0f, -2.f, 0.0f));
  addCurve(g_scene, RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE, Vec4f(-1.5f, 0.0f, -2.f, 0.0f));
  addCurve(g_scene, RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE, Vec4f(1.5f, 0.0f, -2.f, 0.0f));
  addCurve(g_scene, RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE, Vec4f(+4.5f, 0.0f, -2.f, 0.0f));

  /* commit changes to scene */
  rtcCommitScene (g_scene);
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  rtcIntersect1(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* interpolate diffuse color */
    Vec3fa diffuse = Vec3fa(1.0f,0.0f,0.0f);
    if (ray.geomID > 0)
    {
      auto geomID = ray.geomID; {
        rtcInterpolate0(rtcGetGeometry(g_scene,geomID),ray.primID,ray.u,ray.v,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,0,&diffuse.x,3);
      }
      diffuse = 0.5f*diffuse;
    }

    /* calculate smooth shading normal */
    Vec3fa Ng = normalize(ray.Ng);
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

    /* trace shadow ray */
    rtcOccluded1(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f) {
      Vec3fa r = normalize(reflect(ray.dir,Ng));
      float s = pow(clamp(dot(r,lightDir),0.0f,1.0f),10.0f);
      float d = clamp(-dot(lightDir,Ng),0.0f,1.0f);
      color = color + diffuse*d + 0.5f*Vec3fa(s);
    }
  }
  return color;
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
    /* calculate pixel color */
    Vec3fa color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
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
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                    const unsigned int width,
                    const unsigned int height,
                    const float time,
                    const ISPCCamera& camera)
{
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = nullptr;
}

} // namespace embree
