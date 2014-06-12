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

struct DifferentialGeometry
{
  Vec3fa P;
  Vec3fa Ng;
  Vec3fa Ns;
};

inline RTCRay Ray(const Vec3fa& org, const Vec3fa& dir, const float tnear, const float tfar)
{
  RTCRay ray;
  ray.org = org;
  ray.dir = dir;
  ray.tnear = tnear;
  ray.tfar = tfar;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;
  return ray;
}

/*! Cosine weighted hemisphere sampling. Up direction is the z direction. */
inline Sample3f CosineSampleHemisphere(const float u, const float v) 
{
  const float phi = 2.0f * (float)pi * u;
  const float cosTheta = sqrt(v), sinTheta = sqrt(1.0f - v);
  return Sample3f(Vec3fa(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta),cosTheta*(1.0f/(float)pi));
}

/*! Cosine weighted hemisphere sampling. Up direction is provided as argument. */
inline Sample3f CosineSampleHemisphere(const float& u, const float& v, const Vec3fa& N) 
{
  const Sample3f s = CosineSampleHemisphere(u,v);
  return Sample3f(frame(N)*s.v,s.pdf);
}

/*! Uniform sampling of spherical cone. Cone direction is the z
 *  direction. */
inline Sample3f UniformSampleCone(const float u, const float v, const float angle) {
  const float phi = (float)(2.0f * M_PI) * u;
  const float cosTheta = 1.0f - v*(1.0f - cos(angle));
  const float sinTheta = cos2sin(cosTheta);
  return Sample3f(Vec3fa(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta), 1.0f/((float)(4.0f*M_PI)*sqr(sin(0.5f*angle))));
}

/*! Uniform sampling of spherical cone. Cone direction is provided as argument. */
inline Sample3f UniformSampleCone(const float u, const float v, const float angle, const Vec3fa N) { // FIXME: &
  const Sample3f s = UniformSampleCone(u,v,angle);
  return Sample3f(frame(N)*s.v,s.pdf);
}

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

struct ISPCAmbientLight
{
  Vec3fa L;                  //!< radiance of ambient light
};

inline Vec3fa AmbientLight__eval(const ISPCAmbientLight& light, const Vec3fa& wo) {
  return Vec3fa(light.L);
}

inline Vec3fa AmbientLight__sample(const ISPCAmbientLight& light, const DifferentialGeometry& dg, Sample3f& wi, float& tMax, const Vec2f& s) 
{
  wi = CosineSampleHemisphere(s.x,s.y,dg.Ns);
  tMax = 1e20f;
  return Vec3fa(light.L);
}

struct ISPCPointLight
{
  Vec3fa P;                  //!< position of point light
  Vec3fa I;                  //!< radiant intensity of point light
};

Vec3fa PointLight__sample(const ISPCPointLight& light, 
                                 const DifferentialGeometry& dg, 
                                 Sample3f& wi,
                                 float& tMax,
                                 const Vec2f& s) 
{
  Vec3fa d = Vec3fa(light.P) - dg.P;
  float distance = length(d);
  wi = Sample3f(d*rcp(distance), distance*distance);
  tMax = distance;
  return Vec3fa(light.I);
}

struct ISPCDirectionalLight
{
  Vec3fa D;                  //!< Light direction
  Vec3fa E;                  //!< Irradiance (W/m^2)
};

Vec3fa DirectionalLight__sample(const ISPCDirectionalLight& light, 
                                       const DifferentialGeometry& dg, 
                                       Sample3f& wi,
                                       float& tMax,
                                       const Vec2f& s) 
{
  wi = Sample3f(neg(normalize(Vec3fa(light.D))),1.0f); 
  tMax = inf; 
  return Vec3fa(light.E);
}

struct ISPCDistantLight
{
  Vec3fa D;             //!< Light direction
  Vec3fa L;             //!< Radiance (W/(m^2*sr))
  float halfAngle;     //!< Half illumination angle
  float radHalfAngle;  //!< Half illumination angle in radians
  float cosHalfAngle;  //!< Cosine of half illumination angle
};

Vec3fa DistantLight__eval(const ISPCDistantLight& light, const Vec3fa& wo) 
{
  if (dot(wo,Vec3fa(light.D)) >= light.cosHalfAngle) return Vec3fa(light.L);
  return Vec3fa(0.0f);
}

Vec3fa DistantLight__sample(const ISPCDistantLight& light,
                                   const DifferentialGeometry& dg, 
                                   Sample3f& wi,
                                   float& tMax,
                                   const Vec2f& s) 
{
  wi = UniformSampleCone(s.x,s.y,light.radHalfAngle,Vec3fa(light.D));
  tMax = 1e20f;
  return Vec3fa(light.L);
}

struct ISPCScene
{
  ISPCMesh** meshes;   //!< list of meshes
  ISPCMaterial* materials;     //!< material list
  int numMeshes;                       //!< number of meshes
  int numMaterials;                    //!< number of materials

  void** hairsets;
  int numHairSets;
  bool animate;

  ISPCAmbientLight* ambientLights; //!< list of ambient lights
  int numAmbientLights;                    //!< number of ambient lights
  
  ISPCPointLight* pointLights;     //!< list of point lights
  int numPointLights;                      //!< number of point lights
  
  ISPCDirectionalLight* dirLights; //!< list of directional lights
  int numDirectionalLights;                //!< number of directional lights

  ISPCDistantLight* distantLights; //!< list of distant lights
  int numDistantLights;                    //!< number of distant lights
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
}

RTCScene convertScene(ISPCScene* scene_in)
{
  /* create scene */
  RTCScene scene_out = rtcNewScene(RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT, RTC_INTERSECT1);

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

inline Vec3fa Matte__eval(const int& materialID, const Vec3fa& wo, const DifferentialGeometry& dg, const Vec3fa& wi) 
{
  ISPCMaterial* material = &g_ispc_scene->materials[materialID];
  Vec3fa diffuse = Vec3fa(material->Kd);
  return diffuse * (1.0f/(float)pi) * clamp(dot(wi,dg.Ns),0.0f,1.0f);
}

inline Vec3fa Matte__sample(const int& materialID, const Vec3fa& wo, const DifferentialGeometry& dg, Sample3f& wi, const Vec2f& s) 
{
  wi = CosineSampleHemisphere(s.x,s.y,dg.Ns);
  return Matte__eval(materialID, wo, dg, wi.v);
}

/* for details about this random number generator see: P. L'Ecuyer,
   "Maximally Equidistributed Combined Tausworthe Generators",
   Mathematics of Computation, 65, 213 (1996), 203--213:
   http://www.iro.umontreal.ca/~lecuyer/myftp/papers/tausme.ps */

struct rnd_state {
  unsigned int s1, s2, s3, s4;
};

unsigned int irand(rnd_state& state)
{
  #define TAUSWORTHE(s,a,b,c,d) ((s&c)<<d) ^ (((s <<a) ^ s)>>b)
  state.s1 = TAUSWORTHE(state.s1,  6U, 13U, 4294967294U, 18U);
  state.s2 = TAUSWORTHE(state.s2,  2U, 27U, 4294967288U,  2U);
  state.s3 = TAUSWORTHE(state.s3, 13U, 21U, 4294967280U,  7U);
  state.s4 = TAUSWORTHE(state.s4,  3U, 12U, 4294967168U, 13U);
  return (state.s1 ^ state.s2 ^ state.s3 ^ state.s4);
} 

void init_rnd_state(rnd_state& state, int x, int y, int z, int w)
{
  state.s1 = x*13;
  state.s2 = y*276;
  state.s3 = z*78689;
  state.s4 = w*183837438;
  irand(state);
}

inline float frand(rnd_state& state) {
  return (float)(irand(state) & 0xFFFFFFF)/(float)0xFFFFFFFUL;
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

Vec3fa renderPixelFunction(float x, float y, rnd_state& state, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
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
    const Vec3fa wo = neg(ray.dir);
    
    /* invoke environment lights if nothing hit */
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) 
    {
      /* iterate over all ambient lights */
      for (size_t i=0; i<g_ispc_scene->numAmbientLights; i++)
        L = L + Lw*AmbientLight__eval(g_ispc_scene->ambientLights[i],wo); // FIXME: +=

      /* iteratr over all distant lights */
      for (size_t i=0; i<g_ispc_scene->numDistantLights; i++)
        L = L + Lw*DistantLight__eval(g_ispc_scene->distantLights[i],wo); // FIXME: +=

      break;
    }

    /* compute differential geometry */
    DifferentialGeometry dg;
    dg.P  = ray.org+ray.tfar*ray.dir;
    dg.Ng = face_forward(ray.dir,normalize(ray.Ng));
    dg.Ns = face_forward(ray.dir,normalize(ray.Ng)); // FIXME: implement

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

    Sample3f wi; float tMax;
    
    /* iterate over ambient lights */
    for (size_t i=0; i<g_ispc_scene->numAmbientLights; i++)
    {
      Vec3fa Ll = AmbientLight__sample(g_ispc_scene->ambientLights[i],dg,wi,tMax,Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*Matte__eval(materialID,wo,dg,wi.v); // FIXME: +=
    }

    /* iterate over point lights */
    for (size_t i=0; i<g_ispc_scene->numPointLights; i++)
    {
      Vec3fa Ll = PointLight__sample(g_ispc_scene->pointLights[i],dg,wi,tMax,Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*Matte__eval(materialID,wo,dg,wi.v); // FIXME: +=
    }

    /* iterate over directional lights */
    for (size_t i=0; i<g_ispc_scene->numDirectionalLights; i++)
    {
      Vec3fa Ll = DirectionalLight__sample(g_ispc_scene->dirLights[i],dg,wi,tMax,Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*Matte__eval(materialID,wo,dg,wi.v); // FIXME: +=
    }

    /* iterate over distant lights */
    for (size_t i=0; i<g_ispc_scene->numDistantLights; i++)
    {
      Vec3fa Ll = DistantLight__sample(g_ispc_scene->distantLights[i],dg,wi,tMax,Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*Matte__eval(materialID,wo,dg,wi.v); // FIXME: +=
    }

    /* calculate diffuce bounce */
    Vec3fa c = Matte__sample(materialID,wo, dg, wi, Vec2f(frand(state),frand(state)));
    if (wi.pdf <= 0.0f) break;
    Lw = Lw*c/wi.pdf; // FIXME: *=

    /* setup secondary ray */
    ray.org = dg.P;
    ray.dir = normalize(wi.v);
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
  rnd_state state;
  init_rnd_state(state,x,y,x*y,g_accu_count);

  //int state = 21344*x+121233*y+234532*g_accu_count;
  Vec3fa L = Vec3fa(0.0f,0.0f,0.0f);
  //for (int i=0; i<16; i++) {
  L = L + renderPixelFunction(x,y,state,vx,vy,vz,p); // FIXME: +=
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
