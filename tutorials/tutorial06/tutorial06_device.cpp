// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

struct ISPCTriangle 
{
  int v0;                /*< first triangle vertex */
  int v1;                /*< second triangle vertex */
  int v2;                /*< third triangle vertex */
  int materialID;        /*< material of triangle */
};

struct ISPCMaterial
{
  int illum;             /*< illumination model */
  
  float d;               /*< dissolve factor, 1=opaque, 0=transparent */
  float Ns;              /*< specular exponent */
  float Ni;              /*< optical density for the surface (index of refraction) */
  
  Vec3fa Ka;              /*< ambient reflectivity */
  Vec3fa Kd;              /*< diffuse reflectivity */
  Vec3fa Ks;              /*< specular reflectivity */
  Vec3fa Tf;              /*< transmission filter */
};

struct ISPCMesh
{
  Vec3fa* positions;    //!< vertex position array
  Vec3fa* normals;       //!< vertex normal array
  Vec2f* texcoords;     //!< vertex texcoord array
  ISPCTriangle* triangles;  //!< list of triangles
  int numVertices;
  int numTriangles;
};

struct ISPCScene
{
  ISPCMesh** meshes;         //!< list of meshes
  ISPCMaterial* materials;  //!< material list
  int numMeshes;
  int numMaterials;
};

/* scene data */
extern "C" ISPCScene* g_ispc_scene;
RTCScene g_scene = NULL;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const RTCError code, const int8* str)
{
  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  abort();
}

/* accumulation buffer */
Vec3fa* g_accu = NULL;
size_t g_accu_width = 0;
size_t g_accu_height = 0;
size_t g_accu_count = 0;
Vec3fa g_accu_vx;
Vec3fa g_accu_vy;
Vec3fa g_accu_vz;
Vec3fa g_accu_p;
extern "C" bool g_changed;

/* light */
Vec3fa AmbientLight__L;

/* called by the C++ code for initialization */
extern "C" void device_init (int8* cfg)
{
  /* initialize last seen camera */
  g_accu_vx = Vec3fa(0.0f);
  g_accu_vy = Vec3fa(0.0f);
  g_accu_vz = Vec3fa(0.0f);
  g_accu_p  = Vec3fa(0.0f);

  /* initialize ray tracing core */
  rtcInit(cfg);

  /* set error handler */
  rtcSetErrorFunction(error_handler);

  /* set start render mode */
  renderPixel = renderPixelStandard;

  /* set light */
  AmbientLight__L = Vec3fa(0.9f,0.9f,0.9f);
}

RTCScene convertScene(ISPCScene* scene_in)
{
  /* create scene */
  RTCScene scene_out = rtcNewScene(RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT,RTC_INTERSECT1);

  /* add all meshes to the scene */
  for (int i=0; i<scene_in->numMeshes; i++)
  {
    /* get ith mesh */
    ISPCMesh* mesh = scene_in->meshes[i];

    /* create a triangle mesh */
    unsigned int geometry = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices);
    
    /* set vertices */
    Vertex* vertices = (Vertex*) rtcMapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 
    for (int j=0; j<mesh->numVertices; j++) {
      vertices[j].x = mesh->positions[j].x;
      vertices[j].y = mesh->positions[j].y;
      vertices[j].z = mesh->positions[j].z;
    }

    /* set triangles */
    Triangle* triangles = (Triangle*) rtcMapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
    for (int j=0; j<mesh->numTriangles; j++) {
      triangles[j].v0 = mesh->triangles[j].v0;
      triangles[j].v1 = mesh->triangles[j].v1;
      triangles[j].v2 = mesh->triangles[j].v2;
    }
    rtcUnmapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 
    rtcUnmapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
  }

  /* commit changes to scene */
  rtcCommit (scene_out);
  return scene_out;
}

/*! Cosine weighted hemisphere sampling. Up direction is the z direction. */
inline Vec3fa cosineSampleHemisphere(float& pdf, const float u, const float v) 
{
  const float phi = 2.0f * (float)pi * u;
  const float cosTheta = sqrt(v), sinTheta = sqrt(1.0f - v);
  pdf = cosTheta*(1.0f/(float)pi);
  return Vec3fa(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

/*! Cosine weighted hemisphere sampling. Up direction is provided as argument. */
inline Vec3fa cosineSampleHemisphere(float& pdf, const float& u, const float& v, const Vec3fa& N) {
  return frame(N)*cosineSampleHemisphere(pdf,u,v);
}

inline Vec3fa AmbientLight__eval(const Vec3fa& Ns, const Vec3fa& wi) {
  return AmbientLight__L;
}

inline Vec3fa AmbientLight__sample(const Vec3fa& Ns, 
                                  Vec3fa& wi,
                                  float& tMax,
                                  const Vec2f& s) 
{
  float pdf; wi = cosineSampleHemisphere(pdf,s.x,s.y,Ns);
  tMax = 1e20f;
  return AmbientLight__L/pdf;
}

inline Vec3fa Matte__eval(const int& materialID, const Vec3fa& wo, const Vec3fa& Ns, const Vec3fa& wi) 
{
  ISPCMaterial* material = &g_ispc_scene->materials[materialID];
  Vec3fa diffuse = Vec3fa(material->Kd);
  return diffuse * (1.0f/(float)pi) * clamp(dot(wi,Ns),0.0f,1.0f);
}

inline Vec3fa Matte__sample(const int& materialID, const Vec3fa& wo, const Vec3fa& Ns, Vec3fa& wi, const Vec2f& s) 
{
  float pdf; wi = cosineSampleHemisphere(pdf,s.x,s.y,Ns);
  return Matte__eval(materialID, wo, Ns, wi)/pdf;
}

inline float frand(int& seed) {
  seed = 1103515245 * seed + 12345;
  seed = 235543534 * seed + 2341233;
  seed = 43565 * seed + 2332443;
  return (seed & 0xFFFF)/(float)0xFFFF;
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

Vec3fa renderPixelSeed(float x, float y, int& seed, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* radiance accumulator and weight */
  Vec3fa L = Vec3fa(0.0f);
  Vec3fa Lw = Vec3fa(1.0f);

  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;

  /* iterative path tracer loop */
  for (int i=0; i<10; i++)
  {
    /* terminate if contribution too low */
    if (max(Lw.x,max(Lw.y,Lw.z)) < 0.01f)
      break;

    /* intersect ray with scene */ 
    rtcIntersect(g_scene,ray);
    Vec3fa Ns = face_forward(ray.dir,normalize(ray.Ng));
    Vec3fa Ph = ray.org + ray.tfar*ray.dir;

    /* shade background with ambient light */
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
      Vec3fa La = AmbientLight__eval(Ns,neg(ray.dir));
      L = L + Lw*La; // FIXME: +=
      break;
    }
        
    /* shade all rays that hit something */
#if 1 // FIXME: pointer gather not implemented on ISPC for Xeon Phi
    int materialID = g_ispc_scene->meshes[ray.geomID]->triangles[ray.primID].materialID; 
#else
    int materialID = 0;
    foreach_unique (geomID in ray.geomID) {
      if (geomID >= 0 && geomID < g_ispc_scene->numMeshes) { // FIXME: workaround for ISPC bug
        ISPCMesh* mesh = g_ispc_scene->meshes[geomID];
        materialID = mesh->triangles[ray.primID].materialID;
      }
    }
#endif

    /* sample ambient light */
    Vec3fa wi; float tMax;
    Vec2f s = Vec2f(frand(seed),frand(seed));
    Vec3fa Ll = AmbientLight__sample(Ns,wi,tMax,s);
        
    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = Ph;
    shadow.dir = wi;
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;
    
    /* trace shadow ray */
    rtcOccluded(g_scene,shadow);
    
    /* add light contribution */
    if (shadow.geomID == RTC_INVALID_GEOMETRY_ID) {
      Vec3fa Lm = Matte__eval(materialID,neg(ray.dir),Ns,wi);
      L = L + Lw*Ll*Lm; // FIXME: +=
    }

    /* calculate diffuce bounce */
    s = Vec2f(frand(seed),frand(seed));
    Vec3fa c = Matte__sample(materialID,neg(ray.dir), Ns, wi, s);
    Lw = Lw*c; // FIXME: *=

    /* setup secondary ray */
    ray.org = Ph;
    ray.dir = normalize(wi);
    ray.tnear = 0.001f;
    ray.tfar = inf;
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.primID = RTC_INVALID_GEOMETRY_ID;
    ray.mask = -1;
    ray.time = 0;
  }
  return L;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  int seed = 21344*x+121233*y+234532*g_accu_count;
  Vec3fa L = Vec3fa(0.0f,0.0f,0.0f);
  //for (int i=0; i<16; i++) {
  L = L + renderPixelSeed(x,y,seed,vx,vy,vz,p); // FIXME: +=
  //}
  //L = L*(1.0f/16.0f);
  return L;
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
    //    Vec3fa color = Vec3fa(0.0,0.0,0.0); 

    /* write color to framebuffer */
    Vec3fa* dst = &g_accu[y*width+x];
    *dst = *dst + Vec3fa(color.x,color.y,color.z,1.0f); // FIXME: use += operator
    float f = rcp(max(0.001f,dst->w));
    unsigned int r = (unsigned int) (255.0f * clamp(dst->x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(dst->y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(dst->z*f,0.0f,1.0f));
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
  /* create scene */
  if (g_scene == NULL)
    g_scene = convertScene(g_ispc_scene);

  /* create accumulator */
  if (g_accu_width != width || g_accu_height != height) {
    g_accu = new Vec3fa[width*height];
    g_accu_width = width;
    g_accu_height = height;
    memset(g_accu,0,width*height*sizeof(Vec3fa));
  }

  /* reset accumulator */
  bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(g_accu_vx,vx); g_accu_vx = vx; // FIXME: use != operator
  camera_changed |= ne(g_accu_vy,vy); g_accu_vy = vy; // FIXME: use != operator
  camera_changed |= ne(g_accu_vz,vz); g_accu_vz = vz; // FIXME: use != operator
  camera_changed |= ne(g_accu_p,  p); g_accu_p  = p;  // FIXME: use != operator
  g_accu_count++;
  if (camera_changed) {
    g_accu_count=0;
    memset(g_accu,0,width*height*sizeof(Vec3fa));
  }

  /* render image */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
  rtcDebug();
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcExit();
}
