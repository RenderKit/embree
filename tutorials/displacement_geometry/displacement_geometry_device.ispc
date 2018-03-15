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

#include "../common/tutorial/tutorial_device.isph"

/* configuration */
#define EDGE_LEVEL 256.0f
#define ENABLE_SMOOTH_NORMALS 0

/* scene data */
RTCScene g_scene = NULL;

/* previous camera position */
uniform Vec3f old_p;

__aligned(16) uniform float cube_vertices[8][4] =
{
  { -1.0f, -1.0f, -1.0f, 0.0f },
  {  1.0f, -1.0f, -1.0f, 0.0f },
  {  1.0f, -1.0f,  1.0f, 0.0f },
  { -1.0f, -1.0f,  1.0f, 0.0f },
  { -1.0f,  1.0f, -1.0f, 0.0f },
  {  1.0f,  1.0f, -1.0f, 0.0f },
  {  1.0f,  1.0f,  1.0f, 0.0f },
  { -1.0f,  1.0f,  1.0f, 0.0f }
};

#if 1

#define NUM_INDICES 24
#define NUM_FACES 6
#define FACE_SIZE 4

uniform unsigned int cube_indices[24] = {
  0, 4, 5, 1,
  1, 5, 6, 2,
  2, 6, 7, 3,
  0, 3, 7, 4,
  4, 7, 6, 5,
  0, 1, 2, 3,
};

uniform unsigned int cube_faces[6] = {
  4, 4, 4, 4, 4, 4
};

#else

#define NUM_INDICES 36
#define NUM_FACES 12
#define FACE_SIZE 3

uniform unsigned int cube_indices[36] = {
  1, 4, 5,  0, 4, 1,
  2, 5, 6,  1, 5, 2,
  3, 6, 7,  2, 6, 3,
  4, 3, 7,  0, 3, 4,
  5, 7, 6,  4, 7, 5,
  3, 1, 2,  0, 1, 3
};

uniform unsigned int cube_faces[12] = {
  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
};

#endif

float displacement(const Vec3f& P)
{
  float dN = 0.0f;
  for (float freq = 1.0f; freq<40.0f; freq*= 2) {
    float n = abs(noise(freq*P));
    dN += 1.4f*n*n/freq;
  }
  return dN;
}

float displacement_du(const Vec3f& P, const Vec3f& dPdu)
{
  const float du = 0.001f;
  return (displacement(P+du*dPdu)-displacement(P))/du;
}

float displacement_dv(const Vec3f& P, const Vec3f& dPdv)
{
  const float dv = 0.001f;
  return (displacement(P+dv*dPdv)-displacement(P))/dv;
}

unmasked void displacementFunction(const struct RTCDisplacementFunctionNArguments* uniform args)
{
  const uniform float* uniform nx = args->Ng_x;
  const uniform float* uniform ny = args->Ng_y;
  const uniform float* uniform nz = args->Ng_z;
  uniform float* uniform px = args->P_x;
  uniform float* uniform py = args->P_y;
  uniform float* uniform pz = args->P_z;
  uniform unsigned int N = args->N;
                                   
  foreach (i=0 ... N) {
    const Vec3f P = make_Vec3f(px[i],py[i],pz[i]);
    const Vec3f Ng = make_Vec3f(nx[i],ny[i],nz[i]);
    const Vec3f dP = displacement(P)*Ng;
    px[i] += dP.x; py[i] += dP.y; pz[i] += dP.z;
  }
}

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene_i)
{
  /* create a triangulated cube with 6 quads and 8 vertices */
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);

  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, cube_vertices, 0, sizeof(uniform Vec3fa),       8);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT,   cube_indices,  0, sizeof(uniform unsigned int), NUM_INDICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,   0, RTC_FORMAT_UINT,   cube_faces,    0, sizeof(uniform unsigned int), NUM_FACES);

  uniform float* uniform level = (uniform float* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, sizeof(uniform float), NUM_INDICES);
  for (uniform uintptr_t i=0; i<NUM_INDICES; i++) level[i] = EDGE_LEVEL;

  rtcSetGeometryDisplacementFunction(geom,displacementFunction);

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
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
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);
  rtcSetSceneFlags(g_scene,RTC_SCENE_FLAG_ROBUST);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* add cube */
  addCube(g_scene);

  /* commit changes to scene */
  rtcCommitScene (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  rtcIntersectV(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3f diffuse = ray.geomID != 0 ? make_Vec3f(0.9f,0.6f,0.5f) : make_Vec3f(0.8f,0.0f,0.0f);
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    Vec3f Ng = normalize(ray.Ng);
#if ENABLE_SMOOTH_NORMALS
    Vec3f P = ray.org + ray.tfar*ray.dir;
    if (ray.geomID > 0) {
      Vec3f dPdu,dPdv;
      foreach_unique (geomID in ray.geomID) {
        rtcInterpolateV1(rtcGetGeometry(g_scene,geomID),ray.primID,ray.u,ray.v,RTC_BUFFER_TYPE_VERTEX,0,NULL,&dPdu.x,&dPdv.x,3);
      }
      Ng = normalize(cross(dPdu,dPdv));
      dPdu = dPdu + Ng*displacement_du(P,dPdu);
      dPdv = dPdv + Ng*displacement_dv(P,dPdv);
      Ng = normalize(cross(dPdu,dPdv));
    }
#endif

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

    /* trace shadow ray */
    rtcOccludedV(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-(dot(lightDir,Ng)),0.0f,1.0f);
  }
  return color;
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
    /* calculate pixel color */
    Vec3f color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

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

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
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
