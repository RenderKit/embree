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
inline Sample3f cosineSampleHemisphere(const float u, const float v) 
{
  const float phi = 2.0f * (float)pi * u;
  const float cosTheta = sqrt(v), sinTheta = sqrt(1.0f - v);
  return Sample3f(Vec3fa(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta),cosTheta*(1.0f/(float)pi));
}

/*! Cosine weighted hemisphere sampling. Up direction is provided as argument. */
inline Sample3f cosineSampleHemisphere(const float& u, const float& v, const Vec3fa& N) 
{
  const Sample3f s = cosineSampleHemisphere(u,v);
  return Sample3f(frame(N)*s.v,s.pdf);
}

/*! Samples hemisphere with power cosine distribution. Up direction
 *  is the z direction. */
inline Sample3f powerCosineSampleHemisphere(const float u, const float v, const float _exp) 
{
  const float phi = 2.0f * float(pi) * u;
  const float cosTheta = pow(v,1.0f/(_exp+1.0f));
  const float sinTheta = cos2sin(cosTheta);
  return Sample3f(Vec3fa(cos(phi) * sinTheta, 
				   sin(phi) * sinTheta, 
				   cosTheta), 
                       (_exp+1.0f)*pow(cosTheta,_exp)*0.5f/float(pi));
}

/*! Samples hemisphere with power cosine distribution. Up direction
 *  is provided as argument. */
inline Sample3f powerCosineSampleHemisphere(const float u, const float v, const Vec3fa N, const float _exp) {
  const Sample3f s = powerCosineSampleHemisphere(u,v,_exp);
  return Sample3f(frame(N)*s.v,s.pdf);
}

/*! Uniform sampling of spherical cone. Cone direction is the z
 *  direction. */
inline Sample3f UniformSampleCone(const float u, const float v, const float angle) {
  const float phi = (float)(2.0f * float(pi)) * u;
  const float cosTheta = 1.0f - v*(1.0f - cos(angle));
  const float sinTheta = cos2sin(cosTheta);
  return Sample3f(Vec3fa(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta), 1.0f/((float)(4.0f*float(pi))*sqr(sin(0.5f*angle))));
}

/*! Uniform sampling of spherical cone. Cone direction is provided as argument. */
inline Sample3f UniformSampleCone(const float u, const float v, const float angle, const Vec3fa N) { // FIXME: &
  const Sample3f s = UniformSampleCone(u,v,angle);
  return Sample3f(frame(N)*s.v,s.pdf);
}





/*! Reflects a viewing vector V at a normal N. */
inline Sample3f reflect_(const Vec3fa& V, const Vec3fa& N) {
  float cosi = dot(V,N);
  return Sample3f(2.0f*cosi*N-V, 1.0f);
}

/*! Reflects a viewing vector V at a normal N. Cosine between V
 *  and N is given as input. */
inline Sample3f reflect_(const Vec3fa &V, const Vec3fa &N, const float cosi) {
  return Sample3f(2.0f*cosi*N-V, 1.0f);
}

/*! Refracts a viewing vector V at a normal N using the relative
 *  refraction index eta. Eta is refraction index of outside medium
 *  (where N points into) divided by refraction index of the inside
 *  medium. The vectors V and N have to point towards the same side
 *  of the surface. The cosine between V and N is given as input and
 *  the cosine of -N and transmission ray is computed as output. */
inline Sample3f refract(const Vec3fa V, const Vec3fa N, const float eta, 
                        const float cosi, float &cost)
{
  const float k = 1.0f-eta*eta*(1.0f-cosi*cosi);
  if (k < 0.0f) { cost = 0.0f; return Sample3f(Vec3fa(0.f),0.0f); }
  cost = sqrt(k);
  return Sample3f(eta*(cosi*N-V)-cost*N, 1.0f); //sqr(eta));
}

/*! Computes fresnel coefficient for media interface with relative
 *  refraction index eta. Eta is the outside refraction index
 *  divided by the inside refraction index. Both cosines have to be
 *  positive. */
inline float fresnelDielectric(const float cosi, const float cost, const float eta)
{
  const float Rper = (eta*cosi -     cost) * rcp(eta*cosi +     cost);
  const float Rpar = (    cosi - eta*cost) * rcp(    cosi + eta*cost);
  return 0.5f*(Rpar*Rpar + Rper*Rper);
}

/*! Computes fresnel coefficient for media interface with relative
 *  refraction index eta. Eta is the outside refraction index
 *  divided by the inside refraction index. The cosine has to be
 *  positive. */
inline float fresnelDielectric(const float cosi, const float eta)
{
  const float k = 1.0f-eta*eta*(1.0f-cosi*cosi);
  if (k < 0.0f) return 1.0f;
  const float cost = sqrt(k);
  return fresnelDielectric(cosi, cost, eta);
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
  Vec3fa Kt;              /*< transmission filter */
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
  wi = cosineSampleHemisphere(s.x,s.y,dg.Ns);
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
  if (-dot(wo,Vec3fa(light.D)) >= light.cosHalfAngle) return Vec3fa(light.L);
  return Vec3fa(0.0f);
}

Vec3fa DistantLight__sample(const ISPCDistantLight& light,
                                   const DifferentialGeometry& dg, 
                                   Sample3f& wi,
                                   float& tMax,
                                   const Vec2f& s) 
{
  wi = UniformSampleCone(s.x,s.y,light.radHalfAngle,Vec3fa(neg(light.D)));
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

struct BRDF
{
  float Ns;               /*< specular exponent */
  float Ni;               /*< optical density for the surface (index of refraction) */
  Vec3fa Ka;              /*< ambient reflectivity */
  Vec3fa Kd;              /*< diffuse reflectivity */
  Vec3fa Ks;              /*< specular reflectivity */
  Vec3fa Kt;              /*< transmission filter */
};

inline Vec3fa BRDF__eval(const BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Vec3fa& wi) 
{
  Vec3fa R = Vec3fa(0.0f,0.0f,0.0f);
  const float Md = max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z);
  const float Ms = max(max(brdf.Ks.x,brdf.Ks.y),brdf.Ks.z);
  const float Mt = max(max(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z);
  if (Md > 0.0f) {
    R = R + (1.0f/float(pi)) * clamp(dot(wi,dg.Ns)) * brdf.Kd; // FIXME: +=
  }
  if (Ms > 0.0f && brdf.Ns < 1E10) { // FIXME: inf
    const Sample3f refl = reflect_(wo,dg.Ns);
    if (dot(refl.v,wi) > 0.0f) 
      R = R + (brdf.Ns+2) * float(one_over_two_pi) * pow(max(1e-10f,dot(refl.v,wi)),brdf.Ns) * clamp(dot(wi,dg.Ns)) * brdf.Ks; // FIXME: +=
  }
  if (Mt > 0.0f) {
  }
  return R;
}

inline Vec3fa BRDF__sample(const BRDF& brdf, const Vec3fa& Lw, const Vec3fa& wo, const DifferentialGeometry& dg, Sample3f& wi_o, bool& outside, const Vec2f& s)  
{
  Vec3fa cd = Vec3fa(0.0f); 
  Sample3f wid = Sample3f(Vec3fa(0.0f),0.0f);
  if (max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z) > 0.0f) {
    wid = cosineSampleHemisphere(s.x,s.y,dg.Ns);
    cd = float(one_over_pi) * clamp(dot(wid.v,dg.Ns)) * brdf.Kd;
  }

  Vec3fa cs = Vec3fa(0.0f); 
  Sample3f wis = Sample3f(Vec3fa(0.0f),0.0f);
  if (max(max(brdf.Ks.x,brdf.Ks.y),brdf.Ks.z) > 0.0f)
  {
    if (brdf.Ns < 1E10) { // FIXME: inf
      const Sample3f refl = reflect_(wo,dg.Ns);
      wis = powerCosineSampleHemisphere(s.x,s.y,refl.v,brdf.Ns);
      cs = (brdf.Ns+2) * float(one_over_two_pi) * pow(dot(refl.v,wis.v),brdf.Ns) * clamp(dot(wis.v,dg.Ns)) * brdf.Ks;
    }
    else
    {
      float F = 1.0f;
      if (brdf.Ni != 1.0f) {
        float Ni = brdf.Ni;
        if (outside) Ni = 1.0f/brdf.Ni;
        float cosThetaO = clamp(dot(wo,dg.Ns));
        float cosThetaI; Sample3f wt = refract(wo,dg.Ns,Ni,cosThetaO,cosThetaI);
        F = fresnelDielectric(cosThetaO,cosThetaI,Ni);
      }
      wis = reflect_(wo,dg.Ns);
      cs = F * brdf.Ks;
    }
  }

  Vec3fa ct = Vec3fa(0.0f); 
  Sample3f wit = Sample3f(Vec3fa(0.0f),0.0f);
  if (max(max(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z) > 0.0f)
  {
    if (brdf.Ni == 1.0f)
    {
      wit = Sample3f(neg(wo),1.0f);
      ct = brdf.Kt;
      outside = !outside;
    }
    else
    {
      float Ni = brdf.Ni;
      if (outside) Ni = 1.0f/brdf.Ni;
      float cosThetaO = clamp(dot(wo,dg.Ns));
      float cosThetaI; wit = refract(wo,dg.Ns,Ni,cosThetaO,cosThetaI);
      float T = 1.0f-fresnelDielectric(cosThetaO,cosThetaI,Ni);
      ct = brdf.Kt * Vec3fa(T);
      outside = !outside;
    }
  }

  const Vec3fa md = Lw*cd/wid.pdf;
  const Vec3fa ms = Lw*cs/wis.pdf;
  const Vec3fa mt = Lw*ct/wit.pdf;

  const float Cd = wid.pdf == 0.0f ? 0.0f : max(max(md.x,md.y),md.z);
  const float Cs = wis.pdf == 0.0f ? 0.0f : max(max(ms.x,ms.y),ms.z);
  const float Ct = wit.pdf == 0.0f ? 0.0f : max(max(mt.x,mt.y),mt.z);
  const float C  = Cd + Cs + Ct;

  if (C == 0.0f) {
    wi_o = Sample3f(Vec3fa(0,0,0),0);
    return Vec3fa(0,0,0);
  }

  const float CPd = Cd/C;
  const float CPs = Cs/C;
  const float CPt = Ct/C;

  if (s.x < CPd) {
    wi_o = Sample3f(wid.v,wid.pdf*CPd);
    return cd;
  } 
  else if (s.x < CPd + CPs)
  {
    wi_o = Sample3f(wis.v,wis.pdf*CPs);
    return cs;
  }
  else 
  {
    wi_o = Sample3f(wit.v,wit.pdf*CPt);
    return ct;
  }
}

/* for details about this random number generator see: P. L'Ecuyer,
   "Maximally Equidistributed Combined Tausworthe Generators",
   Mathematics of Computation, 65, 213 (1996), 203--213:
   http://www.iro.umontreal.ca/~lecuyer/myftp/papers/tausme.ps */

struct rand_state {
  unsigned int s1, s2, s3;
};

unsigned int irand(rand_state& state)
{
  state.s1 = ((state.s1 & 4294967294U) << 12U) ^ (((state.s1<<13U)^state.s1)>>19U);
  state.s2 = ((state.s2 & 4294967288U) <<  4U) ^ (((state.s2<< 2U)^state.s2)>>25U);
  state.s3 = ((state.s3 & 4294967280U) << 17U) ^ (((state.s3<< 3U)^state.s3)>>11U);
  return state.s1 ^ state.s2 ^ state.s3;
}

void init_rand(rand_state& state, unsigned int x, unsigned int y, unsigned int z)
{
  state.s1 = x >=   2 ? x : x +   2;
  state.s2 = y >=   8 ? y : y +   8;
  state.s3 = z >=  16 ? z : z +  16;
  for (int i=0; i<10; i++) irand(state);
}

inline float frand(rand_state& state) {
  return irand(state)*2.3283064365386963e-10f;
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

Vec3fa renderPixelFunction(float x, float y, rand_state& state, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* radiance accumulator and weight */
  Vec3fa L = Vec3fa(0.0f);
  Vec3fa Lw = Vec3fa(1.0f);
  bool outside = true;

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
        L = L + Lw*AmbientLight__eval(g_ispc_scene->ambientLights[i],ray.dir); // FIXME: +=

      /* iteratr over all distant lights */
      for (size_t i=0; i<g_ispc_scene->numDistantLights; i++)
        L = L + Lw*DistantLight__eval(g_ispc_scene->distantLights[i],ray.dir); // FIXME: +=

      break;
    }

    /* compute differential geometry */
    DifferentialGeometry dg;
    dg.P  = ray.org+ray.tfar*ray.dir;
    dg.Ng = face_forward(ray.dir,normalize(ray.Ng));
    dg.Ns = face_forward(ray.dir,normalize(ray.Ng));

    /* shade all rays that hit something */
#if 1 // FIXME: pointer gather not implemented in ISPC for Xeon Phi
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
    
    /* calculate BRDF */ // FIXME: avoid gathers
    BRDF brdf;
    ISPCMaterial* material = &g_ispc_scene->materials[materialID];
    float d = material->d;
    //if (material->map_d) { d *= material->map_d.get(s,t); }
    brdf.Ka = Vec3fa(material->Ka);
    //if (material->map_Ka) { brdf.Ka *= material->map_Ka->get(dg.st); }
    brdf.Kd = d * Vec3fa(material->Kd);  
    //if (material->map_Kd) brdf.Kd *= material->map_Kd->get(dg.st);  
    brdf.Ks = d * Vec3fa(material->Ks);  
    //if (material->map_Ks) brdf.Ks *= material->map_Ks->get(dg.st); 
    brdf.Ns = material->Ns;  
    //if (material->map_Ns) { brdf.Ns *= material->map_Ns.get(dg.st); }
    brdf.Kt = (1.0f-d)*Vec3fa(material->Kt);
    brdf.Ni = material->Ni;

    /* sample BRDF at hit point */
    Sample3f wi1; 
    Vec3fa c = BRDF__sample(brdf,Lw, wo, dg, wi1, outside, Vec2f(frand(state),frand(state)));

#if 0
    /* iterate over ambient lights */
    for (size_t i=0; i<g_ispc_scene->numAmbientLights; i++)
    {
      Vec3fa L0 = Vec3fa(0.0f);
      Sample3f wi0; float tMax0;
      Vec3fa Ll0 = AmbientLight__sample(g_ispc_scene->ambientLights[i],dg,wi0,tMax0,Vec2f(frand(state),frand(state)));
      if (wi0.pdf > 0.0f) {
        RTCRay shadow = Ray(dg.P,wi0.v,0.001f,tMax0);
        rtcOccluded(g_scene,shadow);
        if (shadow.geomID == RTC_INVALID_GEOMETRY_ID) {
          L0 = Ll0/wi0.pdf*BRDF__eval(brdf,wo,dg,wi0.v);
        }
        L = L + Lw*L0;
      }

#if 0
      Vec3fa L1 = Vec3fa(0.0f);
      Vec3fa Ll1 = AmbientLight__eval(g_ispc_scene->ambientLights[i],wi1.v);
      if (wi1.pdf > 0.0f) {
        RTCRay shadow = Ray(dg.P,wi1.v,0.001f,inf);
        rtcOccluded(g_scene,shadow);
        if (shadow.geomID == RTC_INVALID_GEOMETRY_ID) {
          L1 = Ll1/wi1.pdf*c;
        }
      }

      float s = wi0.pdf*wi0.pdf + wi1.pdf*wi1.pdf;
      if (s > 0) {
        float w0 = 0;
        float w1 = 1;
        //float w0 = wi0.pdf*wi0.pdf/s;
        //float w1 = wi1.pdf*wi1.pdf/s;
        L = L + Lw*(w0*L0+w1*L1);
      }
#endif
    }
    
    /* iterate over point lights */
    Sample3f wi; float tMax;
    for (size_t i=0; i<g_ispc_scene->numPointLights; i++)
    {
      Vec3fa Ll = PointLight__sample(g_ispc_scene->pointLights[i],dg,wi,tMax,Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*BRDF__eval(brdf,wo,dg,wi.v); // FIXME: +=
    }

    /* iterate over directional lights */
    for (size_t i=0; i<g_ispc_scene->numDirectionalLights; i++)
    {
      Vec3fa Ll = DirectionalLight__sample(g_ispc_scene->dirLights[i],dg,wi,tMax,Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*BRDF__eval(brdf,wo,dg,wi.v); // FIXME: +=
    }

    /* iterate over distant lights */
    for (size_t i=0; i<g_ispc_scene->numDistantLights; i++)
    {
      Vec3fa Ll = DistantLight__sample(g_ispc_scene->distantLights[i],dg,wi,tMax,Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*BRDF__eval(brdf,wo,dg,wi.v); // FIXME: +=
    }
#endif

    if (wi1.pdf <= 0.0f) break;
    Lw = Lw*c/wi1.pdf; // FIXME: *=

    /* setup secondary ray */
    ray.org = dg.P;
    ray.dir = normalize(wi1.v);
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
  rand_state state;
  init_rand(state,
            253*x+35*y+152*g_accu_count+54,
            1253*x+345*y+1452*g_accu_count+564,
            10253*x+3435*y+52*g_accu_count+13);

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
    //if (x != 200 || y != 450) continue;

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
