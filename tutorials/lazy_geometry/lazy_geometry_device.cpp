// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "../common/tutorial/tutorial_device.h"

RTCDevice g_device = nullptr;

const int numPhi = 20;
const int numTheta = 2*numPhi;
const int numSpheres = 64;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const RTCError code, const char* str = nullptr)
{
  if (code == RTC_NO_ERROR) 
    return;
  
  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : printf("RTC_CANCELLED"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  exit(1);
}

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
  ALIGNED_STRUCT
  unsigned int geometry;
  LazyState state;
  RTCScene object;
  int userID;
  Vec3fa center;
  float radius;
};

void instanceBoundsFunc(const LazyGeometry* instance, size_t item, RTCBounds* bounds_o)
{
  Vec3fa lower = instance->center-Vec3fa(instance->radius);
  Vec3fa upper = instance->center+Vec3fa(instance->radius);
  bounds_o->lower_x = lower.x;
  bounds_o->lower_y = lower.y;
  bounds_o->lower_z = lower.z;
  bounds_o->upper_x = upper.x;
  bounds_o->upper_y = upper.y;
  bounds_o->upper_z = upper.z;
}

unsigned int createTriangulatedSphere (RTCScene scene, const Vec3fa& p, float r)
{
  /* create triangle mesh */
  unsigned int mesh = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 2*numTheta*(numPhi-1), numTheta*(numPhi+1));
  
  /* map triangle and vertex buffers */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);

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
  rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
  rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  return mesh;
}

void lazyCreate(LazyGeometry* instance)
{
  /* one thread will switch the object from the LAZY_INVALID state to the LAZY_CREATE state */
  if (atomic_cmpxchg((int32_t*)&instance->state,LAZY_INVALID,LAZY_CREATE) == 0) 
  {
    /* create the geometry */
    printf("creating sphere %i\n",instance->userID);
    instance->object = rtcDeviceNewScene(g_device,RTC_SCENE_STATIC,RTC_INTERSECT1);
    createTriangulatedSphere(instance->object,instance->center,instance->radius);

    /* now switch to the LAZY_COMMIT state */
    __memory_barrier();
    instance->state = LAZY_COMMIT;
  } 
  else 
  {
    /* wait until the geometry got created */
    while (atomic_cmpxchg((int32_t*)&instance->state,10,11) < LAZY_COMMIT) {
      // instead of actively spinning here, best use a condition to let the thread sleep, or let it help in the creation stage
    }
  }

  /* multiple threads might enter the rtcCommit function to jointly
   * build the internal data structures */
  rtcCommit(instance->object);
  
  /* switch to LAZY_VALID state */
  atomic_cmpxchg((int32_t*)&instance->state,LAZY_COMMIT,LAZY_VALID);
}

void instanceIntersectFunc(LazyGeometry* instance, RTCRay& ray, size_t item)
{
  /* create the object if it is not yet created */
  if (instance->state != LAZY_VALID)
    lazyCreate(instance);

  /* trace ray inside object */
  const int geomID = ray.geomID;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  rtcIntersect(instance->object,ray);
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) ray.geomID = geomID;
  else ray.instID = instance->userID;
}

void instanceOccludedFunc(LazyGeometry* instance, RTCRay& ray, size_t item) 
{
  /* create the object if it is not yet created */
  if (instance->state != LAZY_VALID)
    lazyCreate(instance);

  /* trace ray inside object */
  rtcOccluded(instance->object,ray);
}

LazyGeometry* createLazyObject (RTCScene scene, int userID, const Vec3fa& center, const float radius)
{
  LazyGeometry* instance = new LazyGeometry;
  instance->state = LAZY_INVALID;
  instance->object = nullptr;
  instance->userID = userID;
  instance->center = center;
  instance->radius = radius;
  instance->geometry = rtcNewUserGeometry(scene,1);
  rtcSetUserData(scene,instance->geometry,instance);
  rtcSetBoundsFunction(scene,instance->geometry,(RTCBoundsFunc)&instanceBoundsFunc);
  rtcSetIntersectFunction(scene,instance->geometry,(RTCIntersectFunc)&instanceIntersectFunc);
  rtcSetOccludedFunction (scene,instance->geometry,(RTCOccludedFunc )&instanceOccludedFunc);
  return instance;
}

/* creates a ground plane */
unsigned int createGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  unsigned int mesh = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* scene data */
RTCScene g_scene  = nullptr;

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);

  /* create scene */
  g_scene = rtcDeviceNewScene(g_device,RTC_SCENE_STATIC,RTC_INTERSECT1);

  /* instantiate geometry */
  createGroundPlane(g_scene);
  for (size_t i=0; i<numSpheres; i++) {
    float a = 2.0*float(pi)*(float)i/(float)numSpheres;
    createLazyObject(g_scene,i,10.0f*Vec3fa(cosf(a),0,sinf(a)),1);
  }
  rtcCommit (g_scene);

  /* set start render mode */
  renderPixel = renderPixelStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.instID = 4; // set default instance ID
  ray.mask = -1;
  ray.time = 0;
  
  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  
  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
  {
    Vec3fa diffuse = Vec3fa(1.0f);
    color = color + diffuse*0.5;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    
    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = ray.org + ray.tfar*ray.dir;
    shadow.dir = neg(lightDir);
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = 1;
    shadow.primID = 0;
    shadow.mask = -1;
    shadow.time = 0;
    
    /* trace shadow ray */
    rtcOccluded(g_scene,shadow);
    
    /* add light contribution */
    if (shadow.geomID)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }
  return color;
}

/* task that renders a single screen tile */
void renderTile(int taskIndex, int* pixels,
                     const int width,
                     const int height, 
                     const float time,
                     const Vec3fa& vx, 
                     const Vec3fa& vy, 
                     const Vec3fa& vz, 
                     const Vec3fa& p,
                     const int numTilesX, 
                     const int numTilesY)
{
  const int tileY = taskIndex / numTilesX;
  const int tileX = taskIndex - tileY * numTilesX;
  const int x0 = tileX * TILE_SIZE_X;
  const int x1 = min(x0+TILE_SIZE_X,width);
  const int y0 = tileY * TILE_SIZE_Y;
  const int y1 = min(y0+TILE_SIZE_Y,height);

  for (int y = y0; y<y1; y++) for (int x = x0; x<x1; x++)
  {
    /* calculate pixel color */
    Vec3fa color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const int width,
                           const int height, 
                           const float time,
                           const Vec3fa& vx, 
                           const Vec3fa& vy, 
                           const Vec3fa& vz, 
                           const Vec3fa& p)
{
  /* render all pixels */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
}
