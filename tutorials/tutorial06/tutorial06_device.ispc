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

#include "../common/tutorial/tutorial_device.isph"

struct DifferentialGeometry
{
  Vec3f P;
  Vec3f Ng;
  Vec3f Ns;
};

inline RTCRay make_Ray(const Vec3f& org, const Vec3f& dir, const float tnear, const float tfar)
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
  return make_Sample3f(make_Vec3f(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta),cosTheta*(1.0f/(float)pi));
}

/*! Cosine weighted hemisphere sampling. Up direction is provided as argument. */
inline Sample3f cosineSampleHemisphere(const float& u, const float& v, const Vec3f& N) 
{
  const Sample3f s = cosineSampleHemisphere(u,v);
  return make_Sample3f(frame(N)*s.v,s.pdf);
}

/*! Samples hemisphere with power cosine distribution. Up direction
 *  is the z direction. */
inline Sample3f powerCosineSampleHemisphere(const float u, const float v, const float _exp) 
{
  const float phi = 2.0f * (M_PI) * u;
  const float cosTheta = pow(v,1.0f/(_exp+1.0f));
  const float sinTheta = cos2sin(cosTheta);
  return make_Sample3f(make_Vec3f(cos(phi) * sinTheta, 
				   sin(phi) * sinTheta, 
				   cosTheta), 
                       (_exp+1.0f)*pow(cosTheta,_exp)*0.5f/(M_PI));
}

/*! Samples hemisphere with power cosine distribution. Up direction
 *  is provided as argument. */
inline Sample3f powerCosineSampleHemisphere(const float u, const float v, const Vec3f N, const float _exp) {
  const Sample3f s = powerCosineSampleHemisphere(u,v,_exp);
  return make_Sample3f(frame(N)*s.v,s.pdf);
}

/*! Uniform sampling of spherical cone. Cone direction is the z
 *  direction. */
inline Sample3f UniformSampleCone(const float u, const float v, const float angle) {
  const float phi = (float)(2.0f * M_PI) * u;
  const float cosTheta = 1.0f - v*(1.0f - cos(angle));
  const float sinTheta = cos2sin(cosTheta);
  return make_Sample3f(make_Vec3f(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta), 1.0f/((float)(4.0f*M_PI)*sqr(sin(0.5f*angle))));
}

/*! Uniform sampling of spherical cone. Cone direction is provided as argument. */
inline Sample3f UniformSampleCone(const float u, const float v, const float angle, const Vec3f N) { // FIXME: &
  const Sample3f s = UniformSampleCone(u,v,angle);
  return make_Sample3f(frame(N)*s.v,s.pdf);
}





/*! Reflects a viewing vector V at a normal N. */
inline Sample3f reflect_(const Vec3f& V, const Vec3f& N) {
  float cosi = dot(V,N);
  return make_Sample3f(2.0f*cosi*N-V, 1.0f);
}

/*! Reflects a viewing vector V at a normal N. Cosine between V
 *  and N is given as input. */
inline Sample3f reflect_(const Vec3f &V, const Vec3f &N, const float cosi) {
  return make_Sample3f(2.0f*cosi*N-V, 1.0f);
}

/*! Refracts a viewing vector V at a normal N using the relative
 *  refraction index eta. Eta is refraction index of outside medium
 *  (where N points into) divided by refraction index of the inside
 *  medium. The vectors V and N have to point towards the same side
 *  of the surface. The cosine between V and N is given as input and
 *  the cosine of -N and transmission ray is computed as output. */
inline Sample3f refract(const Vec3f V, const Vec3f N, const float eta, 
                        const float cosi, float &cost)
{
  const float k = 1.0f-eta*eta*(1.0f-cosi*cosi);
  if (k < 0.0f) { cost = 0.0f; return make_Sample3f(make_Vec3f(0.f),0.0f); }
  cost = sqrt(k);
  return make_Sample3f(eta*(cosi*N-V)-cost*N, 1.0f); //sqr(eta));
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
  uniform Vec3fa* positions;    //!< vertex position array
  uniform Vec3fa* normals;       //!< vertex normal array
  uniform Vec2f* texcoords;     //!< vertex texcoord array
  uniform ISPCTriangle* triangles;  //!< list of triangles
  int numVertices;
  int numTriangles;
};

struct ISPCAmbientLight
{
  Vec3fa L;                  //!< radiance of ambient light
};

inline Vec3f AmbientLight__eval(const uniform ISPCAmbientLight& light, const Vec3f& wo) {
  return make_Vec3f(light.L);
}

inline Vec3f AmbientLight__sample(const uniform ISPCAmbientLight& light, const DifferentialGeometry& dg, varying Sample3f& wi, varying float& tMax, varying const Vec2f& s) 
{
  wi = cosineSampleHemisphere(s.x,s.y,dg.Ns);
  tMax = 1e20f;
  return make_Vec3f(light.L);
}

struct ISPCPointLight
{
  Vec3fa P;                  //!< position of point light
  Vec3fa I;                  //!< radiant intensity of point light
};

varying Vec3f PointLight__sample(const uniform ISPCPointLight& light, 
                                 varying const DifferentialGeometry& dg, 
                                 varying Sample3f& wi,
                                 varying float& tMax,
                                 varying const Vec2f& s) 
{
  Vec3f d = make_Vec3f(light.P) - dg.P;
  float distance = length(d);
  wi = make_Sample3f(d*rcp(distance), distance*distance);
  tMax = distance;
  return make_Vec3f(light.I);
}

struct ISPCDirectionalLight
{
  Vec3fa D;                  //!< Light direction
  Vec3fa E;                  //!< Irradiance (W/m^2)
};

varying Vec3f DirectionalLight__sample(const uniform ISPCDirectionalLight& light, 
                                       varying const DifferentialGeometry& dg, 
                                       varying Sample3f& wi,
                                       varying float& tMax,
                                       varying const Vec2f& s) 
{
  wi = make_Sample3f(neg(normalize(make_Vec3f(light.D))),1.0f); 
  tMax = inf; 
  return make_Vec3f(light.E);
}

struct ISPCDistantLight
{
  Vec3fa D;             //!< Light direction
  Vec3fa L;             //!< Radiance (W/(m^2*sr))
  float halfAngle;     //!< Half illumination angle
  float radHalfAngle;  //!< Half illumination angle in radians
  float cosHalfAngle;  //!< Cosine of half illumination angle
};

varying Vec3f DistantLight__eval(const uniform ISPCDistantLight& light, varying const Vec3f& wo) 
{
  if (-dot(wo,make_Vec3f(light.D)) >= light.cosHalfAngle) return make_Vec3f(light.L);
  return make_Vec3f(0.0f);
}

varying Vec3f DistantLight__sample(const uniform ISPCDistantLight& light,
                                   varying const DifferentialGeometry& dg, 
                                   varying Sample3f& wi,
                                   varying float& tMax,
                                   varying const Vec2f& s) 
{
  wi = UniformSampleCone(s.x,s.y,light.radHalfAngle,make_Vec3f(neg(light.D)));
  tMax = 1e20f;
  return make_Vec3f(light.L);
}

struct ISPCScene
{
  uniform ISPCMesh* uniform* meshes;   //!< list of meshes
  uniform ISPCMaterial* materials;     //!< material list
  int numMeshes;                       //!< number of meshes
  int numMaterials;                    //!< number of materials

  void** hairsets;
  int numHairSets;
  bool animate;

  uniform ISPCAmbientLight* ambientLights; //!< list of ambient lights
  int numAmbientLights;                    //!< number of ambient lights
  
  uniform ISPCPointLight* pointLights;     //!< list of point lights
  int numPointLights;                      //!< number of point lights
  
  uniform ISPCDirectionalLight* dirLights; //!< list of directional lights
  int numDirectionalLights;                //!< number of directional lights

  uniform ISPCDistantLight* distantLights; //!< list of distant lights
  int numDistantLights;                    //!< number of distant lights
};

/* scene data */
extern uniform ISPCScene* uniform g_ispc_scene;
RTCScene g_scene = NULL;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const uniform RTCError code, const uniform int8* uniform str)
{
  print("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : print("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : print("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: print("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : print("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : print("RTC_UNSUPPORTED_CPU"); break;
  default                   : print("invalid error code"); break;
  }
  if (str) { 
    print(" ("); 
    while (*str) putchar(*str++); 
    print(")\n"); 
  }
  abort();
}

/* accumulation buffer */
uniform Vec3fa* uniform g_accu = NULL;
uniform size_t g_accu_width = 0;
uniform size_t g_accu_height = 0;
uniform size_t g_accu_count = 0;
uniform Vec3f g_accu_vx;
uniform Vec3f g_accu_vy;
uniform Vec3f g_accu_vz;
uniform Vec3f g_accu_p;
extern uniform bool g_changed;

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize last seen camera */
  g_accu_vx = make_Vec3f(0.0f);
  g_accu_vy = make_Vec3f(0.0f);
  g_accu_vz = make_Vec3f(0.0f);
  g_accu_p  = make_Vec3f(0.0f);

  /* initialize ray tracing core */
  rtcInit(cfg);

  /* set error handler */
  rtcSetErrorFunction(error_handler);

  /* set start render mode */
  renderPixel = renderPixelStandard;
}

RTCScene convertScene(uniform ISPCScene* uniform scene_in)
{
  /* create scene */
  RTCScene scene_out = rtcNewScene(RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT, RTC_INTERSECT_VARYING);

  /* add all meshes to the scene */
  for (uniform int i=0; i<scene_in->numMeshes; i++)
  {
    /* get ith mesh */
    uniform ISPCMesh* uniform mesh = scene_in->meshes[i];

    /* create a triangle mesh */
    uniform unsigned int geometry = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices);
    
    /* set vertices */
    uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 
    for (uniform int j=0; j<mesh->numVertices; j++) {
      vertices[j].x = mesh->positions[j].x;
      vertices[j].y = mesh->positions[j].y;
      vertices[j].z = mesh->positions[j].z;
    }

    /* set triangles */
    uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
    for (uniform int j=0; j<mesh->numTriangles; j++) {
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
  Vec3f Ka;              /*< ambient reflectivity */
  Vec3f Kd;              /*< diffuse reflectivity */
  Vec3f Ks;              /*< specular reflectivity */
  Vec3f Kt;              /*< transmission filter */
};

inline Vec3f BRDF__eval(const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi) 
{
  Vec3f R = make_Vec3f(0.0f,0.0f,0.0f);
  const float Md = max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z);
  const float Ms = max(max(brdf.Ks.x,brdf.Ks.y),brdf.Ks.z);
  const float Mt = max(max(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z);
  if (Md > 0.0f) {
    R = R + (1.0f/(float)(M_PI)) * clamp(dot(wi,dg.Ns)) * brdf.Kd; // FIXME: +=
  }
  if (Ms > 0.0f && brdf.Ns < 1E10) { // FIXME: inf
    const Sample3f refl = reflect_(wo,dg.Ns);
    if (dot(refl.v,wi) > 0.0f) 
      R = R + (brdf.Ns+2) * one_over_two_pi * pow(max(1e-10f,dot(refl.v,wi)),brdf.Ns) * clamp(dot(wi,dg.Ns)) * brdf.Ks; // FIXME: +=
  }
  if (Mt > 0.0f) {
  }
  return R;
}

inline Vec3f BRDF__sample(const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, const Vec2f& s)  
{
  Vec3f cd = make_Vec3f(0.0f); 
  Sample3f wid = make_Sample3f(make_Vec3f(0.0f),0.0f);
  if (max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z) > 0.0f) {
    wid = cosineSampleHemisphere(s.x,s.y,dg.Ns);
    cd = one_over_pi * clamp(dot(wid.v,dg.Ns)) * brdf.Kd;
  }

  Vec3f cs = make_Vec3f(0.0f); 
  Sample3f wis = make_Sample3f(make_Vec3f(0.0f),0.0f);
  if (max(max(brdf.Ks.x,brdf.Ks.y),brdf.Ks.z) > 0.0f)
  {
    if (brdf.Ns < 1E10) { // FIXME: inf
      const Sample3f refl = reflect_(wo,dg.Ns);
      wis = powerCosineSampleHemisphere(s.x,s.y,refl.v,brdf.Ns);
      cs = (brdf.Ns+2) * one_over_two_pi * pow(dot(refl.v,wis.v),brdf.Ns) * clamp(dot(wis.v,dg.Ns)) * brdf.Ks;
    }
    else
    {
      float F = 1.0f;
      if (brdf.Ni != 1.0f) {
        float cosThetaO = clamp(dot(wo,dg.Ns));
        float cosThetaI; Sample3f wt = refract(wo,dg.Ns,brdf.Ni,cosThetaO,cosThetaI);
        F = fresnelDielectric(cosThetaO,cosThetaI,brdf.Ni);
      }
      wis = reflect_(wo,dg.Ns);
      cs = F * brdf.Ks;
    }
  }

  Vec3f ct = make_Vec3f(0.0f); 
  Sample3f wit = make_Sample3f(make_Vec3f(0.0f),0.0f);
  if (max(max(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z) > 0.0f)
  {
    if (brdf.Ni == 1.0f)
    {
      wit = make_Sample3f(neg(wo),1.0f);
      ct = brdf.Kt;
    }
    else
    {
      float cosThetaO = clamp(dot(wo,dg.Ns));
      float cosThetaI; wit = refract(wo,dg.Ns,brdf.Ni,cosThetaO,cosThetaI);
      float T = 1.0f-fresnelDielectric(cosThetaO,cosThetaI,brdf.Ni);
      ct = brdf.Kt * make_Vec3f(T);
    }
  }

  const Vec3f md = Lw*cd/wid.pdf;
  const Vec3f ms = Lw*cs/wis.pdf;
  const Vec3f mt = Lw*ct/wit.pdf;

  const float Cd = wid.pdf == 0.0f ? 0.0f : max(max(md.x,md.y),md.z);
  const float Cs = wis.pdf == 0.0f ? 0.0f : max(max(ms.x,ms.y),ms.z);
  const float Ct = wit.pdf == 0.0f ? 0.0f : max(max(mt.x,mt.y),mt.z);
  const float C  = Cd + Cs + Ct;

  if (C == 0.0f) {
    wi_o = make_Sample3f(make_Vec3f(0,0,0),0);
    return make_Vec3f(0,0,0);
  }

  const float CPd = Cd/C;
  const float CPs = Cs/C;
  const float CPt = Ct/C;

  if (s.x < CPd) {
    wi_o = make_Sample3f(wid.v,wid.pdf*CPd);
    return cd;
  } 
  else if (s.x < CPd + CPs)
  {
    wi_o = make_Sample3f(wis.v,wis.pdf*CPs);
    return cs;
  }
  else 
  {
    wi_o = make_Sample3f(wit.v,wit.pdf*CPt);
    return ct;
  }
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
  return state.s1 ^ state.s2 ^ state.s3 ^ state.s4;
} 

void init_rnd(rnd_state& state, unsigned int x, unsigned int y, unsigned int z, unsigned int w)
{
  x = x * 4294967294U;
  y = y * 4294967288U;
  z = z * 4294967280U;
  w = w * 4294967168U;
  state.s1 = x >=   2 ? x : x +   2;
  state.s2 = y >=   8 ? y : y +   8;
  state.s3 = z >=  16 ? z : z +  16;
  state.s4 = w >= 128 ? w : w + 128;
  for (uniform int i=0; i<10; i++) irand(state);
}

inline float frand(rnd_state& state) {
  return (float)(irand(state) & 0xFFFFFFF)/(float)0xFFFFFFFUL;
}

inline Vec3f face_forward(const Vec3f& dir, const Vec3f& _Ng) {
  const Vec3f Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

Vec3f renderPixelFunction(float x, float y, rnd_state& state, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
{
  /* radiance accumulator and weight */
  Vec3f L = make_Vec3f(0.0f);
  Vec3f Lw = make_Vec3f(1.0f);

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
  for (uniform int i=0; i<1; i++)
  {
    /* terminate if contribution too low */
    if (max(Lw.x,max(Lw.y,Lw.z)) < 0.01f)
      break;

    /* intersect ray with scene */ 
    rtcIntersect(g_scene,ray);
    const Vec3f wo = neg(ray.dir);
    
    /* invoke environment lights if nothing hit */
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) 
    {
      /* iterate over all ambient lights */
      for (uniform size_t i=0; i<g_ispc_scene->numAmbientLights; i++)
        L = L + Lw*AmbientLight__eval(g_ispc_scene->ambientLights[i],ray.dir); // FIXME: +=

      /* iteratr over all distant lights */
      for (uniform size_t i=0; i<g_ispc_scene->numDistantLights; i++)
        L = L + Lw*DistantLight__eval(g_ispc_scene->distantLights[i],ray.dir); // FIXME: +=

      break;
    }

    /* compute differential geometry */
    DifferentialGeometry dg;
    dg.P  = ray.org+ray.tfar*ray.dir;
    dg.Ng = face_forward(ray.dir,normalize(ray.Ng));
    dg.Ns = face_forward(ray.dir,normalize(ray.Ng)); // FIXME: implement

    /* shade all rays that hit something */
#if 0 // FIXME: pointer gather not implemented on ISPC for Xeon Phi
    int materialID = g_ispc_scene->meshes[ray.geomID]->triangles[ray.primID].materialID; 
#else
    int materialID = 0;
    foreach_unique (geomID in ray.geomID) {
      if (geomID >= 0 && geomID < g_ispc_scene->numMeshes) { // FIXME: workaround for ISPC bug
        uniform ISPCMesh* uniform mesh = g_ispc_scene->meshes[geomID];
        materialID = mesh->triangles[ray.primID].materialID;
      }
    }
#endif
    
    /* calculate BRDF */ // FIXME: avoid gathers
    BRDF brdf;
    uniform ISPCMaterial* material = &g_ispc_scene->materials[materialID];
    float d = material->d;
    //if (material->map_d) { d *= material->map_d.get(s,t); }
    brdf.Ka = make_Vec3f(material->Ka);
    //if (material->map_Ka) { brdf.Ka *= material->map_Ka->get(dg.st); }
    brdf.Kd = d * make_Vec3f(material->Kd);  
    //if (material->map_Kd) brdf.Kd *= material->map_Kd->get(dg.st);  
    brdf.Ks = d * make_Vec3f(material->Ks);  
    //if (material->map_Ks) brdf.Ks *= material->map_Ks->get(dg.st); 
    brdf.Ns = material->Ns;  
    //if (material->map_Ns) { brdf.Ns *= material->map_Ns.get(dg.st); }
    brdf.Kt = (1.0f-d)*make_Vec3f(material->Kt);
    brdf.Ni = material->Ni;

    /* calculate diffuce bounce */
    Sample3f wi1;
    Vec3f c = BRDF__sample(brdf,Lw, wo, dg, wi1, make_Vec2f(frand(state),frand(state)));

    /* iterate over ambient lights */
    for (uniform size_t i=0; i<g_ispc_scene->numAmbientLights; i++)
    {
      Vec3f L0 = make_Vec3f(0.0f);
      Sample3f wi0; float tMax0;
      Vec3f Ll0 = AmbientLight__sample(g_ispc_scene->ambientLights[i],dg,wi0,tMax0,make_Vec2f(frand(state),frand(state)));
      if (wi0.pdf > 0.0f) {
        RTCRay shadow = make_Ray(dg.P,wi0.v,0.001f,tMax0);
        rtcOccluded(g_scene,shadow);
        if (shadow.geomID == RTC_INVALID_GEOMETRY_ID) {
          L0 = Ll0/wi0.pdf*BRDF__eval(brdf,wo,dg,wi0.v);
        }
      }

      Vec3f L1 = make_Vec3f(0.0f);
      Vec3f Ll1 = AmbientLight__eval(g_ispc_scene->ambientLights[i],wi1.v);
      if (wi1.pdf > 0.0f) {
        RTCRay shadow = make_Ray(dg.P,wi1.v,0.001f,inf);
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
    }

    /* iterate over point lights */
    Sample3f wi; float tMax;
    for (uniform size_t i=0; i<g_ispc_scene->numPointLights; i++)
    {
      Vec3f Ll = PointLight__sample(g_ispc_scene->pointLights[i],dg,wi,tMax,make_Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = make_Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*BRDF__eval(brdf,wo,dg,wi.v); // FIXME: +=
    }

    /* iterate over directional lights */
    for (uniform size_t i=0; i<g_ispc_scene->numDirectionalLights; i++)
    {
      Vec3f Ll = DirectionalLight__sample(g_ispc_scene->dirLights[i],dg,wi,tMax,make_Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = make_Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*BRDF__eval(brdf,wo,dg,wi.v); // FIXME: +=
    }

    /* iterate over distant lights */
    for (uniform size_t i=0; i<g_ispc_scene->numDistantLights; i++)
    {
      Vec3f Ll = DistantLight__sample(g_ispc_scene->distantLights[i],dg,wi,tMax,make_Vec2f(frand(state),frand(state)));
      if (wi.pdf <= 0.0f) continue;
      RTCRay shadow = make_Ray(dg.P,wi.v,0.001f,tMax);
      rtcOccluded(g_scene,shadow);
      if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      L = L + Lw*Ll/wi.pdf*BRDF__eval(brdf,wo,dg,wi.v); // FIXME: +=
    }


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
Vec3f renderPixelStandard(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
{
  rnd_state state;
  init_rnd(state,x,y,x*y,g_accu_count);

  //int state = 21344*x+121233*y+234532*g_accu_count;
  Vec3f L = make_Vec3f(0.0f,0.0f,0.0f);
  //for (uniform int i=0; i<16; i++) {
  L = L + renderPixelFunction(x,y,state,vx,vy,vz,p); // FIXME: +=
  //}
  //L = L*(1.0f/16.0f);
  return L;
}
  
/* task that renders a single screen tile */
task void renderTile(uniform int* uniform pixels,
                     const uniform int width,
                     const uniform int height, 
                     const uniform float time,
                     const uniform Vec3f& vx, 
                     const uniform Vec3f& vy, 
                     const uniform Vec3f& vz, 
                     const uniform Vec3f& p,
                     const uniform int numTilesX, 
                     const uniform int numTilesY)
{
  const uniform int tileY = taskIndex / numTilesX;
  const uniform int tileX = taskIndex - tileY * numTilesX;
  const uniform int x0 = tileX * TILE_SIZE_X;
  const uniform int x1 = min(x0+TILE_SIZE_X,width);
  const uniform int y0 = tileY * TILE_SIZE_Y;
  const uniform int y1 = min(y0+TILE_SIZE_Y,height);

  foreach (y = y0 ... y1, x = x0 ... x1)
  {
    //if (x != 200 || y != 450) continue;

    /* calculate pixel color */
    Vec3f color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    Vec3fa* dst = &g_accu[y*width+x];
    *dst = *dst + make_Vec3fa(color.x,color.y,color.z,1.0f); // FIXME: use += operator
    float f = rcp(max(0.001f,dst->w));
    unsigned int r = (unsigned int) (255.0f * clamp(dst->x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(dst->y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(dst->z*f,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform int width,
                           const uniform int height, 
                           const uniform float time,
                           const uniform Vec3f& vx, 
                           const uniform Vec3f& vy, 
                           const uniform Vec3f& vz, 
                           const uniform Vec3f& p)
{
  /* create scene */
  if (g_scene == NULL)
    g_scene = convertScene(g_ispc_scene);

  /* create accumulator */
  if (g_accu_width != width || g_accu_height != height) {
    g_accu = uniform new uniform Vec3fa[width*height];
    g_accu_width = width;
    g_accu_height = height;
    memset(g_accu,0,width*height*sizeof(uniform Vec3fa));
  }

  /* reset accumulator */
  uniform bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(g_accu_vx,vx); g_accu_vx = vx; // FIXME: use != operator
  camera_changed |= ne(g_accu_vy,vy); g_accu_vy = vy; // FIXME: use != operator
  camera_changed |= ne(g_accu_vz,vz); g_accu_vz = vz; // FIXME: use != operator
  camera_changed |= ne(g_accu_p,  p); g_accu_p  = p;  // FIXME: use != operator
  g_accu_count++;
  if (camera_changed) {
    g_accu_count=0;
    memset(g_accu,0,width*height*sizeof(uniform Vec3fa));
  }

  /* render image */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
  rtcDebug();
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcExit();
}
