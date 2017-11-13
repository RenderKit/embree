// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

const uniform int numPhi = 5;
const uniform int numTheta = 2*numPhi;

RTCDevice g_device = NULL;

void renderTileStandardStream(uniform int taskIndex,
                              uniform int threadIndex,
                              uniform int* uniform pixels,
                              const uniform unsigned int width,
                              const uniform unsigned int height,
                              const uniform float time,
                              const uniform ISPCCamera& camera,
                              const uniform int numTilesX,
                              const uniform int numTilesY);

uniform unsigned int createTriangulatedSphere (RTCScene scene, const uniform Vec3f& p, uniform float r)
{
  /* create triangle mesh */
  RTCGeometry geom = rtcNewTriangleMesh (g_device);

  /* map triangle and vertex buffers */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcNewBuffer(geom,RTC_VERTEX_BUFFER,sizeof(uniform Vertex),numTheta*(numPhi+1));
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcNewBuffer(geom,RTC_INDEX_BUFFER,sizeof(uniform Triangle),2*numTheta*(numPhi-1));

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
        triangles[tri].v1 = p00;
        triangles[tri].v2 = p01;
        tri++;
      }

      if (phi < numPhi) {
        triangles[tri].v0 = p11;
        triangles[tri].v1 = p10;
        triangles[tri].v2 = p01;
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
  RTCGeometry geom = rtcNewTriangleMesh (g_device);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcNewBuffer(geom,RTC_VERTEX_BUFFER,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcNewBuffer(geom,RTC_INDEX_BUFFER,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* scene data */
RTCScene g_scene  = NULL;
RTCScene g_scene1 = NULL;

RTCGeometry g_instance0 = NULL;
RTCGeometry g_instance1 = NULL;
RTCGeometry g_instance2 = NULL;
RTCGeometry g_instance3 = NULL;
uniform AffineSpace3f instance_xfm[4];
uniform LinearSpace3f normal_xfm[4];

uniform Vec3f colors[4][4];

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(NULL,rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler,NULL);

  /* create scene */
  g_scene = rtcDeviceNewScene(g_device);
  rtcSetBuildQuality(g_scene,RTC_BUILD_QUALITY_LOW);
  rtcSetBuildHints(g_scene,RTC_BUILD_HINT_DYNAMIC);

  /* create scene with 4 triangulated spheres */
  g_scene1 = rtcDeviceNewScene(g_device);
  createTriangulatedSphere(g_scene1,make_Vec3f( 0, 0,+1),0.5f);
  createTriangulatedSphere(g_scene1,make_Vec3f(+1, 0, 0),0.5f);
  createTriangulatedSphere(g_scene1,make_Vec3f( 0, 0,-1),0.5f);
  createTriangulatedSphere(g_scene1,make_Vec3f(-1, 0, 0),0.5f);
  rtcCommit (g_scene1);

  /* instantiate geometry */
  g_instance0 = rtcNewInstance(g_device,g_scene1,1);
  g_instance1 = rtcNewInstance(g_device,g_scene1,1);
  g_instance2 = rtcNewInstance(g_device,g_scene1,1);
  g_instance3 = rtcNewInstance(g_device,g_scene1,1);
  rtcAttachGeometry(g_scene,g_instance0);
  rtcAttachGeometry(g_scene,g_instance1);
  rtcAttachGeometry(g_scene,g_instance2);
  rtcAttachGeometry(g_scene,g_instance3);
  rtcReleaseGeometry(g_instance0);
  rtcReleaseGeometry(g_instance1);
  rtcReleaseGeometry(g_instance2);
  rtcReleaseGeometry(g_instance3);
  createGroundPlane(g_scene);

  /* set all colors */
  colors[0][0] = make_Vec3f(0.25f, 0.f, 0.f);
  colors[0][1] = make_Vec3f(0.50f, 0.f, 0.f);
  colors[0][2] = make_Vec3f(0.75f, 0.f, 0.f);
  colors[0][3] = make_Vec3f(1.00f, 0.f, 0.f);

  colors[1][0] = make_Vec3f(0.f, 0.25f, 0.f);
  colors[1][1] = make_Vec3f(0.f, 0.50f, 0.f);
  colors[1][2] = make_Vec3f(0.f, 0.75f, 0.f);
  colors[1][3] = make_Vec3f(0.f, 1.00f, 0.f);

  colors[2][0] = make_Vec3f(0.f, 0.f, 0.25f);
  colors[2][1] = make_Vec3f(0.f, 0.f, 0.50f);
  colors[2][2] = make_Vec3f(0.f, 0.f, 0.75f);
  colors[2][3] = make_Vec3f(0.f, 0.f, 1.00f);

  colors[3][0] = make_Vec3f(0.25f, 0.25f, 0.f);
  colors[3][1] = make_Vec3f(0.50f, 0.50f, 0.f);
  colors[3][2] = make_Vec3f(0.75f, 0.75f, 0.f);
  colors[3][3] = make_Vec3f(1.00f, 1.00f, 0.f);

  /* set start render mode */
  if (g_mode == MODE_NORMAL) renderTile = renderTileStandard;
  else                       renderTile = renderTileStandardStream;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform RTCIntersectContext context;
  rtcInitIntersectionContext(&context);
  
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  rtcIntersect(g_scene,&context,RTCRay_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* calculate shading normal in world space */
    Vec3f Ns = ray.Ng;
    if (ray.instID != RTC_INVALID_GEOMETRY_ID)
      Ns = xfmVector(normal_xfm[ray.instID],Ns);
    Ns = normalize(Ns);

    /* calculate diffuse color of geometries */
    Vec3f diffuse = make_Vec3f(1,1,1);
    if (ray.instID != RTC_INVALID_GEOMETRY_ID)
      diffuse = colors[ray.instID][ray.geomID];
    color = color + diffuse*0.5;

    /* initialize shadow ray */
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    rtcOccluded(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.geomID)
      color = color + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
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

/* renders a single screen tile */
void renderTileStandardStream(uniform int taskIndex,
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

  Ray primary_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Ray shadow_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Vec3f color_stream[TILE_SIZE_X*TILE_SIZE_Y];
  bool valid_stream[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  uniform int N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* initialize variables */
    color_stream[N] = make_Vec3f(0.0f);
    bool mask = __mask; unmasked { valid_stream[N] = mask; }

    /* initialize ray */
    Ray& primary = primary_stream[N];
    mask = __mask; unmasked { // invalidates inactive rays
      primary.tnear = mask ? 0.0f         : (float)(pos_inf);
      primary.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(primary, make_Vec3f(camera.xfm.p), make_Vec3f(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), primary.tnear, primary.tfar);

    N++;
    RayStats_addRay(stats);
  }

  Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

  /* trace rays */
  uniform RTCIntersectContext primary_context;
  rtcInitIntersectionContext(&primary_context);
  primary_context.flags = g_iflags_coherent;
  primary_context.userRayExt = &primary_stream;
  rtcIntersectVM(g_scene,&primary_context,(varying RTCRay* uniform)&primary_stream,N,sizeof(Ray));

  /* terminate rays and update color */
  N = -1;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    N++;
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* invalidate shadow rays by default */
    Ray& shadow = shadow_stream[N];
    unmasked {
      shadow.tnear = (float)(pos_inf);
      shadow.tfar  = (float)(neg_inf);
    }

    /* ignore invalid rays */
    if (valid_stream[N] == false) continue;

    /* terminate rays that hit nothing */
    if (primary_stream[N].geomID == RTC_INVALID_GEOMETRY_ID) {
      valid_stream[N] = false;
      continue;
    }

    /* calculate shading normal in world space */
    Ray& primary = primary_stream[N];
    Vec3f Ns = primary.Ng;
    if (primary.instID != RTC_INVALID_GEOMETRY_ID)
      Ns = xfmVector(normal_xfm[primary.instID],Ns);
    Ns = normalize(Ns);

    /* calculate diffuse color of geometries */
    Vec3f diffuse = make_Vec3f(1,1,1);
    if (primary.instID != RTC_INVALID_GEOMETRY_ID)
      diffuse = colors[primary.instID][primary.geomID];
    color_stream[N] = color_stream[N] + diffuse*0.5;

    /* initialize shadow ray tnear/tfar */
    bool mask = __mask; unmasked {
      shadow.tnear = mask ? 0.001f       : (float)(pos_inf);
      shadow.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(shadow, primary.org + primary.tfar*primary.dir, neg(lightDir), shadow.tnear, shadow.tfar);

    RayStats_addShadowRay(stats);
  }
  N++;

  /* trace shadow rays */
  uniform RTCIntersectContext shadow_context;
  rtcInitIntersectionContext(&shadow_context);
  shadow_context.flags = g_iflags_coherent;
  shadow_context.userRayExt = &shadow_stream;
  rtcOccludedVM(g_scene,&shadow_context,(varying RTCRay* uniform)&shadow_stream,N,sizeof(Ray));

  /* add light contribution */
  N = -1;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    N++;
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* ignore invalid rays */
    if (valid_stream[N] == false) continue;

    /* calculate shading normal in world space */
    Ray& primary = primary_stream[N];
    Vec3f Ns = primary.Ng;
    if (primary.instID != RTC_INVALID_GEOMETRY_ID)
      Ns = xfmVector(normal_xfm[primary.instID],Ns);
    Ns = normalize(Ns);

    /* calculate diffuse color of geometries */
    Vec3f diffuse = make_Vec3f(1,1,1);
    if (primary.instID != RTC_INVALID_GEOMETRY_ID)
      diffuse = colors[primary.instID][primary.geomID];

    /* add light contrinution */
    Ray& shadow = shadow_stream[N];
    if (shadow.geomID) {
      color_stream[N] = color_stream[N] + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
    }
  }
  N++;

  /* framebuffer writeback */
  N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color_stream[N].x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color_stream[N].y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color_stream[N].z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
    N++;
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
  uniform float t0 = 0.7f*time;
  uniform float t1 = 1.5f*time;

  /* rotate instances around themselves */
  uniform LinearSpace3f xfm;
  xfm.vx = make_Vec3f(cos(t1),0,sin(t1));
  xfm.vy = make_Vec3f(0,1,0);
  xfm.vz = make_Vec3f(-sin(t1),0,cos(t1));

  /* calculate transformations to move instances in cirle */
  for (uniform int i=0; i<4; i++) {
    uniform float t = t0+i*2.0f*M_PI/4.0f;
    instance_xfm[i] = make_AffineSpace3f(xfm,2.2f*make_Vec3f(+cos(t),0.0f,+sin(t)));
  }

  /* calculate transformations to properly transform normals */
  for (uniform int i=0; i<4; i++)
    normal_xfm[i] = transposed(rcp(instance_xfm[i].l));

  /* set instance transformations */
  rtcSetTransform(g_instance0,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&instance_xfm[0],0);
  rtcSetTransform(g_instance1,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&instance_xfm[1],0);
  rtcSetTransform(g_instance2,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&instance_xfm[2],0);
  rtcSetTransform(g_instance3,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&instance_xfm[3],0);

  /* update scene */
  rtcCommitGeometry(g_instance0);
  rtcCommitGeometry(g_instance1);
  rtcCommitGeometry(g_instance2);
  rtcCommitGeometry(g_instance3);
  rtcCommit (g_scene);

  /* render all pixels */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = NULL;
  rtcReleaseScene (g_scene1); g_scene1 = NULL;
  rtcDeleteDevice(g_device); g_device = NULL;
}
