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

const uniform int numPhi = 20;
const uniform int numTheta = 2*numPhi;
const uniform int numSpheres = 64;

/* state of the lazy geometry */
enum LazyState
{
  LAZY_INVALID = 0,   // the geometry is not yet created
  LAZY_CREATE = 1,    // one thread is creating the geometry
  LAZY_COMMIT = 2,    // possible multiple threads are committing the geometry
  LAZY_VALID = 3      // the geometry is created
};

/* representation for our lazy geometry */
struct LazyGeometry
{
  ALIGNED_STRUCT_(16)
  RTCGeometry geometry;
  uniform LazyState state;
  RTCScene object;
  uniform int userID;
  Vec3f center;
  float radius;
};

uniform LazyGeometry* uniform g_objects[numSpheres];

unmasked void instanceBoundsFunc(const struct RTCBoundsFunctionArguments* uniform args)
{
  const uniform LazyGeometry* uniform instance = (const uniform LazyGeometry* uniform) args->geometryUserPtr;
  uniform RTCBounds* uniform bounds_o = args->bounds_o;
  uniform Vec3f lower = instance->center-make_Vec3f(instance->radius);
  uniform Vec3f upper = instance->center+make_Vec3f(instance->radius);
  bounds_o->lower_x = lower.x;
  bounds_o->lower_y = lower.y;
  bounds_o->lower_z = lower.z;
  bounds_o->upper_x = upper.x;
  bounds_o->upper_y = upper.y;
  bounds_o->upper_z = upper.z;
}

uniform unsigned int createTriangulatedSphere (RTCScene scene, const uniform Vec3f& p, uniform float r)
{
  /* create triangle mesh */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* map triangle and vertex buffers */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vertex), numTheta*(numPhi+1));
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(uniform Triangle), 2*numTheta*(numPhi-1));

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

void lazyCreate(uniform LazyGeometry* uniform instance)
{
  /* one thread will switch the object from the LAZY_INVALID state to the LAZY_CREATE state */
  if (atomic_compare_exchange_global((uniform int32*)&instance->state,LAZY_INVALID,LAZY_CREATE) == 0)
  {
    /* create the geometry */
    //print("creating sphere %i (lazy)\n",instance->userID);
    instance->object = rtcNewScene(g_device);
    createTriangulatedSphere(instance->object,instance->center,instance->radius);

    /* when join mode is not supported we let only a single thread build */
    if (!rtcGetDeviceProperty(g_device,RTC_DEVICE_PROPERTY_JOIN_COMMIT_SUPPORTED))
      rtcCommitScene(instance->object);

    /* now switch to the LAZY_COMMIT state */
    memory_barrier();
    instance->state = LAZY_COMMIT;
  }
  else
  {
    /* wait until the geometry got created */
    while (atomic_compare_exchange_global((uniform int32*)&instance->state,10,11) < LAZY_COMMIT) {
      // instead of actively spinning here, best use a condition to let the thread sleep, or let it help in the creation stage
    }
  }

  /* multiple threads might enter the rtcJoinCommitScene function to jointly
   * build the internal data structures */
  if (rtcGetDeviceProperty(g_device,RTC_DEVICE_PROPERTY_JOIN_COMMIT_SUPPORTED))
    rtcJoinCommitScene(instance->object);

  /* switch to LAZY_VALID state */
  atomic_compare_exchange_global((uniform int32*)&instance->state,LAZY_COMMIT,LAZY_VALID);
}

void eagerCreate(uniform LazyGeometry* uniform instance)
{
  //print("creating sphere %i (eager)\n",instance->userID);
  instance->object = rtcNewScene(g_device);
  createTriangulatedSphere(instance->object,instance->center,instance->radius);
  rtcCommitScene(instance->object);
  instance->state = LAZY_VALID;
}

unmasked void instanceIntersectFuncN(const RTCIntersectFunctionNArguments* uniform args)
{
  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  uniform RTCIntersectContext* uniform context = args->context;
  RTCRayHitN* uniform rays = (RTCRayHitN* uniform)args->rayhit;
  assert(args->N == programCount);
  uniform LazyGeometry* uniform instance = (uniform LazyGeometry* uniform)ptr;

  if (!valid[programIndex])
    return;
  
  varying Ray *uniform ray = (varying Ray *uniform)rays;
  
  /* create the object if it is not yet created */
  if (instance->state != LAZY_VALID)
    lazyCreate(instance);
  
  /* trace ray inside object */
  const unsigned int geomID = ray->geomID;
  ray->geomID = RTC_INVALID_GEOMETRY_ID;
  rtcIntersectV(instance->object,context,RTCRayHit_(*ray));
  if (ray->geomID == RTC_INVALID_GEOMETRY_ID) ray->geomID = geomID;
  else ray->instID = instance->userID;
}

unmasked void instanceOccludedFuncN(const RTCOccludedFunctionNArguments* uniform args)
{
  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
   uniform RTCIntersectContext* uniform context = args->context;
   RTCRayHitN* uniform rays = (RTCRayHitN* uniform)args->ray;
  assert(args->N == programCount);
  uniform LazyGeometry* uniform instance = (uniform LazyGeometry* uniform)ptr;

  if (!valid[programIndex])
    return;
  
  varying Ray *uniform ray = (varying Ray *uniform)rays;
  /* create the object if it is not yet created */
  if (instance->state != LAZY_VALID)
    lazyCreate(instance);
  
  /* trace ray inside object */
  rtcOccludedV(instance->object,context,RTCRay_(*ray));
}

uniform LazyGeometry* uniform createLazyObject (RTCScene scene, uniform int userID, const uniform Vec3f& center, const uniform float radius)
{
  uniform LazyGeometry* uniform instance = uniform new uniform LazyGeometry;
  instance->state = LAZY_INVALID;
  instance->object = NULL;
  instance->userID = userID;
  instance->center = center;
  instance->radius = radius;
  instance->geometry = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  rtcSetGeometryUserPrimitiveCount(instance->geometry,1);
  rtcSetGeometryUserData(instance->geometry,instance);
  rtcSetGeometryBoundsFunction(instance->geometry,instanceBoundsFunc,NULL);
  rtcSetGeometryIntersectFunction(instance->geometry,instanceIntersectFuncN);
  rtcSetGeometryOccludedFunction (instance->geometry,instanceOccludedFuncN);
  rtcCommitGeometry(instance->geometry);
  rtcAttachGeometry(scene,instance->geometry);
  rtcReleaseGeometry(instance->geometry);

  /* if we do not support the join mode then Embree also does not
   * support lazy build */
  if (!rtcGetDeviceProperty(g_device,RTC_DEVICE_PROPERTY_JOIN_COMMIT_SUPPORTED))
    eagerCreate(instance);

  return instance;
}

/* creates a ground plane */
uniform unsigned int createGroundPlane (RTCScene scene)
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
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* scene data */
RTCScene g_scene  = NULL;

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);
  
  /* instantiate geometry */
  createGroundPlane(g_scene);
  for (uniform int i=0; i<numSpheres; i++) {
    uniform float a = 2.0f*M_PI*(uniform float)i/(uniform float)numSpheres;
    g_objects[i] = createLazyObject(g_scene,i,10.0f*make_Vec3f(cosf(a),0,sinf(a)),1);
  }
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
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf, 0.0f, -1, RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID, 4);

  /* intersect ray with scene */
  rtcIntersectV(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3f diffuse = make_Vec3f(1.0f);
    color = color + diffuse*0.5;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    rtcOccludedV(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
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
  /* render all pixels */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  for (uniform int i=0; i<numSpheres; i++) {
    if (g_objects[i]->object) rtcReleaseScene(g_objects[i]->object);
    delete g_objects[i];
  }
  rtcReleaseScene (g_scene); g_scene = NULL;
}
