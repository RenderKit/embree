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

#include "../common/math/random_sampler.isph"
#include "../common/math/sampling.isph"
#include "../common/core/differential_geometry.isph"
#include "../common/tutorial/tutorial_device.isph"
#include "../common/tutorial/scene_device.h"
#include "../common/tutorial/optics.isph"

#undef TILE_SIZE_X
#undef TILE_SIZE_Y

#define TILE_SIZE_X 4
#define TILE_SIZE_Y 4

#define FIXED_SAMPLING 0

#define FIXED_EDGE_TESSELLATION_VALUE 4

#define ENABLE_FILTER_FUNCTION 1

#define MAX_EDGE_LEVEL 128.0f
#define MIN_EDGE_LEVEL   4.0f
#define LEVEL_FACTOR    64.0f
#define MAX_PATH_LENGTH  8

uniform bool g_subdiv_mode = false;
uniform unsigned int keyframeID = 0;

struct BRDF
{
  float Ns;               /*< specular exponent */
  float Ni;               /*< optical density for the surface (index of refraction) */
  Vec3f Ka;              /*< ambient reflectivity */
  Vec3f Kd;              /*< diffuse reflectivity */
  Vec3f Ks;              /*< specular reflectivity */
  Vec3f Kt;              /*< transmission filter */
  float dummy[30];
};

struct Medium
{
  Vec3f transmission; //!< Transmissivity of medium.
  float eta;             //!< Refraction index of medium.
};

inline Medium make_Medium(const varying Vec3f& transmission, const float eta)
{
  Medium m;
  m.transmission = transmission;
  m.eta = eta;
  return m;
}

inline Medium make_Medium_Vacuum() {
  return make_Medium(make_Vec3f((varying float)1.0f),1.0f);
}

inline bool eq(const Medium& a, const Medium& b) {
  return (a.eta == b.eta) && eq(a.transmission, b.transmission);
}

inline Vec3f sample_component2(const Vec3f& c0, const Sample3f& wi0, const Medium& medium0,
                               const Vec3f& c1, const Sample3f& wi1, const Medium& medium1,
                               const Vec3f& Lw, Sample3f& wi_o, Medium& medium_o, const float s)
{
  const Vec3f m0 = Lw*c0/wi0.pdf;
  const Vec3f m1 = Lw*c1/wi1.pdf;

  const float C0 = wi0.pdf == 0.0f ? 0.0f : max(max(m0.x,m0.y),m0.z);
  const float C1 = wi1.pdf == 0.0f ? 0.0f : max(max(m1.x,m1.y),m1.z);
  const float C  = C0 + C1;

  if (C == 0.0f) {
    wi_o = make_Sample3f(make_Vec3f(0,0,0),0);
    return make_Vec3f(0,0,0);
  }

  const float CP0 = C0/C;
  const float CP1 = C1/C;
  if (s < CP0) {
    wi_o = make_Sample3f(wi0.v,wi0.pdf*CP0);
    medium_o = medium0; return c0;
  }
  else {
    wi_o = make_Sample3f(wi1.v,wi1.pdf*CP1);
    medium_o = medium1; return c1;
  }
}

/*! Cosine weighted hemisphere sampling. Up direction is provided as argument. */
inline Sample3f cosineSampleHemisphere(const float  u, const float  v, const Vec3f& N)
{
  Vec3f localDir = cosineSampleHemisphere(make_Vec2f(u,v));
  Sample3f s;
  s.v = frame(N) * localDir;
  s.pdf = cosineSampleHemispherePDF(localDir);
  return s;
}

////////////////////////////////////////////////////////////////////////////////
//                          Minneart BRDF                                     //
////////////////////////////////////////////////////////////////////////////////

struct Minneart
{
  /*! The reflectance parameter. The vale 0 means no reflection,
   *  and 1 means full reflection. */
  Vec3f R;

  /*! The amount of backscattering. A value of 0 means lambertian
   *  diffuse, and inf means maximum backscattering. */
  float b;
};

inline Vec3f Minneart__eval(const varying Minneart* uniform This,
                     const Vec3f &wo, const DifferentialGeometry &dg, const Vec3f &wi)
{
  const float cosThetaI = clamp(dot(wi,dg.Ns));
  const float backScatter = powf(clamp(dot(wo,wi)), This->b);
  return (backScatter * cosThetaI * one_over_pi) * This->R;
}

inline Vec3f Minneart__sample(const varying Minneart* uniform This,
                       const Vec3f &wo,
                       const DifferentialGeometry &dg,
                       Sample3f &wi,
                       const Vec2f &s)
{
  wi = cosineSampleHemisphere(s.x,s.y,dg.Ns);
  return Minneart__eval(This, wo, dg, wi.v);
}

inline void Minneart__Constructor(varying Minneart* uniform This, const varying Vec3f& R, const varying float b)
{
  This->R = R;
  This->b = b;
}

inline varying Minneart make_Minneart(const varying Vec3f& R, const varying float f) {
  varying Minneart m; Minneart__Constructor(&m,R,f); return m;
}

////////////////////////////////////////////////////////////////////////////////
//                            Velvet BRDF                                     //
////////////////////////////////////////////////////////////////////////////////

struct Velvety
{
  uniform BRDF base;

  /*! The reflectance parameter. The vale 0 means no reflection,
   *  and 1 means full reflection. */
  Vec3f R;

  /*! The falloff of horizon scattering. 0 no falloff,
   *  and inf means maximum falloff. */
  float f;
};

inline Vec3f Velvety__eval(const varying Velvety* uniform This,
                    const Vec3f &wo, const DifferentialGeometry &dg, const Vec3f &wi)
{
  const float cosThetaO = clamp(dot(wo,dg.Ns));
  const float cosThetaI = clamp(dot(wi,dg.Ns));
  const float sinThetaO = sqrt(1.0f - cosThetaO * cosThetaO);
  const float horizonScatter = powf(sinThetaO, This->f);
  return (horizonScatter * cosThetaI * one_over_pi) * This->R;
}

inline Vec3f Velvety__sample(const varying Velvety* uniform This,
                      const Vec3f &wo,
                      const DifferentialGeometry &dg,
                      Sample3f &wi,
                      const Vec2f &s)
{
  wi = cosineSampleHemisphere(s.x,s.y,dg.Ns);
  return Velvety__eval(This, wo, dg, wi.v);
}

inline void Velvety__Constructor(varying Velvety* uniform This, const varying Vec3f& R, const varying float f)
{
  This->R = R;
  This->f = f;
}

inline varying Velvety make_Velvety(const varying Vec3f& R, const varying float f) {
  varying Velvety m; Velvety__Constructor(&m,R,f); return m;
}

////////////////////////////////////////////////////////////////////////////////
//                  Dielectric Reflection BRDF                                //
////////////////////////////////////////////////////////////////////////////////

struct DielectricReflection
{
  float eta;
};

inline Vec3f DielectricReflection__eval(const varying DielectricReflection* uniform This, const Vec3f &wo, const DifferentialGeometry &dg, const Vec3f &wi) {
  return make_Vec3f(0.f);
}

inline Vec3f DielectricReflection__sample(const varying DielectricReflection* uniform This, const Vec3f &wo, const DifferentialGeometry &dg, Sample3f &wi, const Vec2f &s)
{
  const float cosThetaO = clamp(dot(wo,dg.Ns));
  wi = make_Sample3f(reflect(wo,dg.Ns,cosThetaO),1.0f);
  return make_Vec3f(fresnelDielectric(cosThetaO,This->eta));
}

inline void DielectricReflection__Constructor(varying DielectricReflection* uniform This,
                                              const varying float etai,
                                              const varying float etat)
{
  This->eta = etai*rcp(etat);
}

inline varying DielectricReflection make_DielectricReflection(const varying float etai, const varying float etat) {
  varying DielectricReflection v; DielectricReflection__Constructor(&v,etai,etat); return v;
}

////////////////////////////////////////////////////////////////////////////////
//                                Lambertian BRDF                             //
////////////////////////////////////////////////////////////////////////////////

struct Lambertian
{
  Vec3f R;
};

inline Vec3f Lambertian__eval(const varying Lambertian* uniform This,
                              const Vec3f &wo, const DifferentialGeometry &dg, const Vec3f &wi)
{
  return This->R * (1.0f/(float)(M_PI)) * clamp(dot(wi,dg.Ns));
}

inline Vec3f Lambertian__sample(const varying Lambertian* uniform This,
                                const Vec3f &wo,
                                const DifferentialGeometry &dg,
                                Sample3f &wi,
                                const Vec2f &s)
{
  wi = cosineSampleHemisphere(s.x,s.y,dg.Ns);
  return Lambertian__eval(This, wo, dg, wi.v);
}

inline void Lambertian__Constructor(varying Lambertian* uniform This, const varying Vec3f& R)
{
  This->R = R;
}

inline varying Lambertian make_Lambertian(const varying Vec3f& R) {
  varying Lambertian v; Lambertian__Constructor(&v,R); return v;
}


////////////////////////////////////////////////////////////////////////////////
//              Lambertian BRDF with Dielectric Layer on top                  //
////////////////////////////////////////////////////////////////////////////////

struct DielectricLayerLambertian
{
  Vec3f T;             //!< Transmission coefficient of dielectricum
  float etait;         //!< Relative refraction index etai/etat of both media
  float etati;         //!< relative refraction index etat/etai of both media
  Lambertian ground;   //!< the BRDF of the ground layer
};

inline Vec3f DielectricLayerLambertian__eval(const varying DielectricLayerLambertian* uniform This,
                                             const Vec3f &wo, const DifferentialGeometry &dg, const Vec3f &wi)
{
  const float cosThetaO = dot(wo,dg.Ns);
  const float cosThetaI = dot(wi,dg.Ns);
  if (cosThetaI <= 0.0f || cosThetaO <= 0.0f) return make_Vec3f(0.f);

  float cosThetaO1;
  const Sample3f wo1 = refract(wo,dg.Ns,This->etait,cosThetaO,cosThetaO1);
  float cosThetaI1;
  const Sample3f wi1 = refract(wi,dg.Ns,This->etait,cosThetaI,cosThetaI1);
  const float Fi = 1.0f - fresnelDielectric(cosThetaI,cosThetaI1,This->etait);
  const Vec3f Fg = Lambertian__eval(&This->ground,neg(wo1.v),dg,neg(wi1.v));
  const float Fo = 1.0f - fresnelDielectric(cosThetaO,cosThetaO1,This->etait);
  return Fo * This->T * Fg * This->T * Fi;
}

inline Vec3f DielectricLayerLambertian__sample(const varying DielectricLayerLambertian* uniform This,
                                               const Vec3f &wo,
                                               const DifferentialGeometry &dg,
                                               Sample3f &wi,
                                               const Vec2f &s)
{
  /*! refract ray into medium */
  float cosThetaO = dot(wo,dg.Ns);
  if (cosThetaO <= 0.0f) { wi = make_Sample3f(make_Vec3f(0.0f),0.0f); return make_Vec3f(0.f); }
  float cosThetaO1; Sample3f wo1 = refract(wo,dg.Ns,This->etait,cosThetaO,cosThetaO1);

  /*! sample ground BRDF */
  Sample3f wi1 = make_Sample3f(make_Vec3f(0.f),1.f);
  Vec3f Fg = Lambertian__sample(&This->ground,neg(wo1.v),dg,wi1,s);

  /*! refract ray out of medium */
  float cosThetaI1 = dot(wi1.v,dg.Ns);
  if (cosThetaI1 <= 0.0f) { wi = make_Sample3f(make_Vec3f(0.0f),0.0f); return make_Vec3f(0.f); }

  float cosThetaI;
  Sample3f wi0 = refract(neg(wi1.v),neg(dg.Ns),This->etati,cosThetaI1,cosThetaI);
  if (wi0.pdf == 0.0f) { wi = make_Sample3f(make_Vec3f(0.0f),0.0f); return make_Vec3f(0.f); }

  /*! accumulate contribution of path */
  wi = make_Sample3f(wi0.v,wi1.pdf);
  float Fi = 1.0f - fresnelDielectric(cosThetaI,cosThetaI1,This->etait);
  float Fo = 1.0f - fresnelDielectric(cosThetaO,cosThetaO1,This->etait);
  return Fo * This->T * Fg * This->T * Fi;
}

inline void DielectricLayerLambertian__Constructor(varying DielectricLayerLambertian* uniform This,
                                                   const varying Vec3f& T,
                                                   const varying float etai,
                                                   const varying float etat,
                                                   const varying Lambertian& ground)
{
  This->T = T;
  This->etait = etai*rcp(etat);
  This->etati = etat*rcp(etai);
  This->ground = ground;
}

inline varying DielectricLayerLambertian make_DielectricLayerLambertian(const varying Vec3f& T,
                                                                        const varying float etai,
                                                                        const varying float etat,
                                                                        const varying Lambertian& ground)
{
  varying DielectricLayerLambertian m;
  DielectricLayerLambertian__Constructor(&m,T,etai,etat,ground);
  return m;
}

/*! Anisotropic power cosine microfacet distribution. */
struct AnisotropicBlinn {
  Vec3f dx;       //!< x-direction of the distribution.
  Vec3f dy;       //!< y-direction of the distribution.
  Vec3f dz;       //!< z-direction of the distribution.
  Vec3f Kr,Kt;
  float nx;        //!< Glossiness in x direction with range [0,infinity[ where 0 is a diffuse surface.
  float ny;        //!< Exponent that determines the glossiness in y direction.
  float norm1;     //!< Normalization constant for calculating the pdf for sampling.
  float norm2;     //!< Normalization constant for calculating the distribution.
  float side;
};

  /*! Anisotropic power cosine distribution constructor. */
inline void AnisotropicBlinn__Constructor(varying AnisotropicBlinn* uniform This, const uniform Vec3f& Kr, const uniform Vec3f& Kt,
                                          const Vec3f& dx, float nx, const Vec3f& dy, float ny, const Vec3f& dz)
{
  This->Kr = Kr;
  This->Kt = Kt;
  This->dx = dx;
  This->nx = nx;
  This->dy = dy;
  This->ny = ny;
  This->dz = dz;
  This->norm1 = sqrtf((nx+1)*(ny+1)) * one_over_two_pi;
  This->norm2 = sqrtf((nx+2)*(ny+2)) * one_over_two_pi;
  This->side = reduce_max(Kr)/(reduce_max(Kr)+reduce_max(Kt));
}

/*! Evaluates the power cosine distribution. \param wh is the half
 *  vector */
inline float AnisotropicBlinn__eval(const varying AnisotropicBlinn* uniform This, const Vec3f& wh)
{
  const float cosPhiH   = dot(wh, This->dx);
  const float sinPhiH   = dot(wh, This->dy);
  const float cosThetaH = dot(wh, This->dz);
  const float R = sqr(cosPhiH)+sqr(sinPhiH);
  if (R == 0.0f) return This->norm2;
  const float n = (This->nx*sqr(cosPhiH)+This->ny*sqr(sinPhiH))*rcp(R);
  return This->norm2 * powf(abs(cosThetaH), n);
}

/*! Samples the distribution. \param s is the sample location
 *  provided by the caller. */
inline Vec3fa AnisotropicBlinn__sample(const varying AnisotropicBlinn* uniform This, const float sx, const float sy)
{
  const float phi = two_pi*sx;
  const float sinPhi0 = sqrtf(This->nx+1)*sinf(phi);
  const float cosPhi0 = sqrtf(This->ny+1)*cosf(phi);
  const float norm = rsqrt(sqr(sinPhi0)+sqr(cosPhi0));
  const float sinPhi = sinPhi0*norm;
  const float cosPhi = cosPhi0*norm;
  const float n = This->nx*sqr(cosPhi)+This->ny*sqr(sinPhi);
  const float cosTheta = powf(sy,rcp(n+1));
  const float sinTheta = cos2sin(cosTheta);
  const float pdf = This->norm1*powf(cosTheta,n);
  const Vec3f h = make_Vec3f(cosPhi * sinTheta, sinPhi * sinTheta, cosTheta);
  const Vec3f wh = h.x*This->dx + h.y*This->dy + h.z*This->dz;
  return make_Vec3fa(wh,pdf);
}

inline Vec3f AnisotropicBlinn__eval(const varying AnisotropicBlinn* uniform This, const Vec3f& wo, const Vec3f& wi)
{
  const float cosThetaI = dot(wi,This->dz);

  /* reflection */
  if (cosThetaI > 0.0f) {
    const Vec3f wh = normalize(wi + wo);
    return This->Kr * AnisotropicBlinn__eval(This,wh) * abs(cosThetaI);
  }

  /* transmission */
  else {
    const Vec3f wh = normalize(reflect(wi,This->dz) + wo);
    return This->Kt * AnisotropicBlinn__eval(This,wh) * abs(cosThetaI);
  }
}

inline Vec3f AnisotropicBlinn__sample(const varying AnisotropicBlinn* uniform This, const Vec3f& wo, Sample3f& wi_o, const float sx, const float sy, const float sz)
{
  //wi = Vec3f(reflect(normalize(wo),normalize(dz)),1.0f); return Kr;
  //wi = Vec3f(neg(wo),1.0f); return Kt;
  const Vec3fa wh = AnisotropicBlinn__sample(This,sx,sy);
  //if (dot(wo,wh) < 0.0f) return Vec3f(0.0f,0.0f);

  /* reflection */
  if (sz < This->side) {
    wi_o = make_Sample3f(reflect(wo,make_Vec3f(wh)),wh.w*This->side);
    const float cosThetaI = dot(wi_o.v,This->dz);
    return This->Kr * AnisotropicBlinn__eval(This,make_Vec3f(wh)) * abs(cosThetaI);
  }

  /* transmission */
  else {
    wi_o = make_Sample3f(reflect(reflect(wo,make_Vec3f(wh)),This->dz),wh.w*(1-This->side));
    const float cosThetaI = dot(wi_o.v,This->dz);
    return This->Kt * AnisotropicBlinn__eval(This,make_Vec3f(wh)) * abs(cosThetaI);
  }
}

////////////////////////////////////////////////////////////////////////////////
//                          Matte Material                                    //
////////////////////////////////////////////////////////////////////////////////

void MatteMaterial__preprocess(uniform ISPCMatteMaterial* uniform material, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
}

Vec3f MatteMaterial__eval(ISPCMatteMaterial* uniform This, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi)
{
  Lambertian lambertian = make_Lambertian(make_Vec3f((varying Vec3fa)This->reflectance));
  return Lambertian__eval(&lambertian,wo,dg,wi);
}

Vec3f MatteMaterial__sample(ISPCMatteMaterial* uniform This, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  Lambertian lambertian = make_Lambertian(make_Vec3f((varying Vec3fa)This->reflectance));
  return Lambertian__sample(&lambertian,wo,dg,wi_o,s);
}

////////////////////////////////////////////////////////////////////////////////
//                          Mirror Material                                    //
////////////////////////////////////////////////////////////////////////////////

void MirrorMaterial__preprocess(uniform ISPCMirrorMaterial* uniform material, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
}

Vec3f MirrorMaterial__eval(ISPCMirrorMaterial* uniform This, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi) {
  return make_Vec3f(0.0f);
}

Vec3f MirrorMaterial__sample(ISPCMirrorMaterial* uniform This, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  wi_o = make_Sample3f(reflect(wo,dg.Ns),1.0f);
  return make_Vec3f(This->reflectance);
}

////////////////////////////////////////////////////////////////////////////////
//                          OBJ Material                                      //
////////////////////////////////////////////////////////////////////////////////

void OBJMaterial__preprocess(uniform ISPCOBJMaterial* uniform material, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
    float d = material->d;
    if (material->map_d) d *= getTextureTexel1f(material->map_d,dg.u,dg.v);
    brdf.Ka = make_Vec3f(material->Ka);
    //if (material->map_Ka) { brdf.Ka *= material->map_Ka->get(dg.st); }
    brdf.Kd = d * make_Vec3f(material->Kd);
    if (material->map_Kd) brdf.Kd = brdf.Kd * getTextureTexel3f(material->map_Kd,dg.u,dg.v);
    brdf.Ks = d * make_Vec3f(material->Ks);
    //if (material->map_Ks) brdf.Ks *= material->map_Ks->get(dg.st);
    brdf.Ns = material->Ns;
    //if (material->map_Ns) { brdf.Ns *= material->map_Ns.get(dg.st); }
    brdf.Kt = (1.0f-d)*make_Vec3f(material->Kt);
    brdf.Ni = material->Ni;
}

Vec3f OBJMaterial__eval(ISPCOBJMaterial* uniform material, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi)
{
  Vec3f R = make_Vec3f(0.0f);
  const float Md = max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z);
  const float Ms = max(max(brdf.Ks.x,brdf.Ks.y),brdf.Ks.z);
  const float Mt = max(max(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z);
  if (Md > 0.0f) {
    R = R + (1.0f/M_PI) * clamp(dot(wi,dg.Ns)) * brdf.Kd;
  }
  if (Ms > 0.0f) {
    const Sample3f refl = make_Sample3f(reflect(wo,dg.Ns),1.0f);
    if (dot(refl.v,wi) > 0.0f)
      R = R + (brdf.Ns+2) * one_over_two_pi * powf(max(1e-10f,dot(refl.v,wi)),brdf.Ns) * clamp(dot(wi,dg.Ns)) * brdf.Ks;
  }
  if (Mt > 0.0f) {
  }
  return R;
}

Vec3f OBJMaterial__sample(ISPCOBJMaterial* uniform material, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
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
    const Sample3f refl = make_Sample3f(reflect(wo,dg.Ns),1.0f);
    wis.v = powerCosineSampleHemisphere(brdf.Ns,s);
    wis.pdf = powerCosineSampleHemispherePDF(wis.v,brdf.Ns);
    wis.v = frame(refl.v) * wis.v;
    cs = (brdf.Ns+2) * one_over_two_pi * powf(max(dot(refl.v,wis.v),1e-10f),brdf.Ns) * clamp(dot(wis.v,dg.Ns)) * brdf.Ks;
  }

  Vec3f ct = make_Vec3f(0.0f);
  Sample3f wit = make_Sample3f(make_Vec3f(0.0f),0.0f);
  if (max(max(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z) > 0.0f)
  {
    wit = make_Sample3f(neg(wo),1.0f);
    ct = brdf.Kt;
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

////////////////////////////////////////////////////////////////////////////////
//                        Metal Material                                      //
////////////////////////////////////////////////////////////////////////////////

void MetalMaterial__preprocess(uniform ISPCMetalMaterial* uniform material, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
}

Vec3f MetalMaterial__eval(ISPCMetalMaterial* uniform This, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi)
{
  const FresnelConductor fresnel = make_FresnelConductor(make_Vec3f(This->eta),make_Vec3f(This->k));
  const PowerCosineDistribution distribution = make_PowerCosineDistribution(rcp(This->roughness));

  const float cosThetaO = dot(wo,dg.Ns);
  const float cosThetaI = dot(wi,dg.Ns);
  if (cosThetaI <= 0.0f || cosThetaO <= 0.0f) return make_Vec3f(0.f);
  const Vec3f wh = normalize(wi+wo);
  const float cosThetaH = dot(wh, dg.Ns);
  const float cosTheta = dot(wi, wh); // = dot(wo, wh);
  const Vec3f F = eval(fresnel,cosTheta);
  const float D = eval(distribution,cosThetaH);
  const float G = min(1.0f, min(2.0f * cosThetaH * cosThetaO / cosTheta,
                                2.0f * cosThetaH * cosThetaI / cosTheta));
  return (make_Vec3f(This->reflectance)*F) * D * G * rcp(4.0f*cosThetaO);
}

Vec3f MetalMaterial__sample(ISPCMetalMaterial* uniform This, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  const PowerCosineDistribution distribution = make_PowerCosineDistribution(rcp(This->roughness));

  if (dot(wo,dg.Ns) <= 0.0f) { wi_o = make_Sample3f(make_Vec3f(0.0f),0.0f); return make_Vec3f(0.f); }
  sample(distribution,wo,dg.Ns,wi_o,s);
  if (dot(wi_o.v,dg.Ns) <= 0.0f) { wi_o = make_Sample3f(make_Vec3f(0.0f),0.0f); return make_Vec3f(0.f); }
  return MetalMaterial__eval(This,brdf,wo,dg,wi_o.v);
}

////////////////////////////////////////////////////////////////////////////////
//                        ReflectiveMetal Material                            //
////////////////////////////////////////////////////////////////////////////////

void ReflectiveMetalMaterial__preprocess(uniform ISPCReflectiveMetalMaterial* uniform material, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)  {
}

Vec3f ReflectiveMetalMaterial__eval(ISPCReflectiveMetalMaterial* uniform This, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi) {
  return make_Vec3f(0.0f);
}

Vec3f ReflectiveMetalMaterial__sample(ISPCReflectiveMetalMaterial* uniform This, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  wi_o = make_Sample3f(reflect(wo,dg.Ns),1.0f);
  return make_Vec3f(This->reflectance) * fresnelConductor(dot(wo,dg.Ns),make_Vec3f((varying Vec3fa)This->eta),make_Vec3f((varying Vec3fa)This->k));
}

////////////////////////////////////////////////////////////////////////////////
//                        Velvet Material                                     //
////////////////////////////////////////////////////////////////////////////////

void VelvetMaterial__preprocess(uniform ISPCVelvetMaterial* uniform material, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
}

Vec3f VelvetMaterial__eval(ISPCVelvetMaterial* uniform This, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi)
{
  Minneart minneart; Minneart__Constructor(&minneart,(varying Vec3f)make_Vec3f(This->reflectance),This->backScattering);
  Velvety velvety; Velvety__Constructor (&velvety,make_Vec3f((varying Vec3fa)This->horizonScatteringColor),This->horizonScatteringFallOff);
  return Minneart__eval(&minneart,wo,dg,wi) + Velvety__eval(&velvety,wo,dg,wi);
}

Vec3f VelvetMaterial__sample(ISPCVelvetMaterial* uniform This, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  Minneart minneart; Minneart__Constructor(&minneart,make_Vec3f((varying Vec3fa)This->reflectance),This->backScattering);
  Velvety velvety; Velvety__Constructor (&velvety,make_Vec3f((varying Vec3fa)This->horizonScatteringColor),This->horizonScatteringFallOff);

  Sample3f wi0; Vec3f c0 = Minneart__sample(&minneart,wo,dg,wi0,s);
  Sample3f wi1; Vec3f c1 = Velvety__sample(&velvety,wo,dg,wi1,s);
  return sample_component2(c0,wi0,medium,c1,wi1,medium,Lw,wi_o,medium,s.x);
}

////////////////////////////////////////////////////////////////////////////////
//                          Dielectric Material                               //
////////////////////////////////////////////////////////////////////////////////

void DielectricMaterial__preprocess(uniform ISPCDielectricMaterial* uniform material, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
}

Vec3f DielectricMaterial__eval(ISPCDielectricMaterial* uniform material, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi) {
  return make_Vec3f(0.0f);
}

Vec3f DielectricMaterial__sample(ISPCDielectricMaterial* uniform material, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  float eta = 0.0f;
  Medium mediumOutside = make_Medium(make_Vec3f((varying Vec3fa)material->transmissionOutside),material->etaOutside);
  Medium mediumInside  = make_Medium(make_Vec3f((varying Vec3fa)material->transmissionInside ),material->etaInside );
  Medium mediumFront, mediumBack;
  if (eq(medium,mediumInside)) {
    eta = material->etaInside/material->etaOutside;
    mediumFront = mediumInside;
    mediumBack = mediumOutside;
  }
  else {
    eta = material->etaOutside/material->etaInside;
    mediumFront = mediumOutside;
    mediumBack = mediumInside;
  }

  float cosThetaO = clamp(dot(wo,dg.Ns));
  float cosThetaI; Sample3f wit = refract(wo,dg.Ns,eta,cosThetaO,cosThetaI);
  Sample3f wis = make_Sample3f(reflect(wo,dg.Ns),1.0f);
  float R = fresnelDielectric(cosThetaO,cosThetaI,eta);
  Vec3f cs = make_Vec3f(R);
  Vec3f ct = make_Vec3f(1.0f-R);
  return sample_component2(cs,wis,mediumFront,ct,wit,mediumBack,Lw,wi_o,medium,s.x);
}

////////////////////////////////////////////////////////////////////////////////
//                          ThinDielectric Material                               //
////////////////////////////////////////////////////////////////////////////////

void ThinDielectricMaterial__preprocess(uniform ISPCThinDielectricMaterial* uniform This, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
}

Vec3f ThinDielectricMaterial__eval(ISPCThinDielectricMaterial* uniform This, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi) {
  return make_Vec3f(0.0f);
}

Vec3f ThinDielectricMaterial__sample(ISPCThinDielectricMaterial* uniform This, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  float cosThetaO = clamp(dot(wo,dg.Ns));
  if (cosThetaO <= 0.0f) return make_Vec3f(0.0f);
  float R = fresnelDielectric(cosThetaO,rcp(This->eta));
  Sample3f wit = make_Sample3f(neg(wo),1.0f);
  Sample3f wis = make_Sample3f(reflect(wo,dg.Ns),1.0f);
  Vec3f ct = exp(make_Vec3f(This->transmissionFactor)*rcp(cosThetaO))*make_Vec3f(1.0f-R);
  Vec3f cs = make_Vec3f(R);
  return sample_component2(cs,wis,medium,ct,wit,medium,Lw,wi_o,medium,s.x);
}

////////////////////////////////////////////////////////////////////////////////
//                     MetallicPaint Material                                 //
////////////////////////////////////////////////////////////////////////////////

void MetallicPaintMaterial__preprocess(uniform ISPCMetallicPaintMaterial* uniform material, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
}

Vec3f MetallicPaintMaterial__eval(ISPCMetallicPaintMaterial* uniform This, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi)
{
  DielectricReflection reflection; DielectricReflection__Constructor(&reflection, 1.0f, This->eta);
  DielectricLayerLambertian lambertian; DielectricLayerLambertian__Constructor(&lambertian, make_Vec3f((varying float)1.0f), 1.0f, This->eta, make_Lambertian(make_Vec3f((varying Vec3fa)This->shadeColor)));
  return DielectricReflection__eval(&reflection,wo,dg,wi) + DielectricLayerLambertian__eval(&lambertian,wo,dg,wi);
}

Vec3f MetallicPaintMaterial__sample(ISPCMetallicPaintMaterial* uniform This, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  DielectricReflection reflection; DielectricReflection__Constructor(&reflection, 1.0f, This->eta);
  DielectricLayerLambertian lambertian; DielectricLayerLambertian__Constructor(&lambertian, make_Vec3f((varying float)1.0f), 1.0f, This->eta, make_Lambertian(make_Vec3f((varying Vec3fa)This->shadeColor)));
  Sample3f wi0; Vec3f c0 = DielectricReflection__sample(&reflection,wo,dg,wi0,s);
  Sample3f wi1; Vec3f c1 = DielectricLayerLambertian__sample(&lambertian,wo,dg,wi1,s);
  return sample_component2(c0,wi0,medium,c1,wi1,medium,Lw,wi_o,medium,s.x);
}

////////////////////////////////////////////////////////////////////////////////
//                              Hair Material                                 //
////////////////////////////////////////////////////////////////////////////////

void HairMaterial__preprocess(uniform ISPCHairMaterial* uniform This, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
  AnisotropicBlinn__Constructor((varying AnisotropicBlinn* uniform)&brdf,make_Vec3f(This->Kr),make_Vec3f(This->Kt),dg.Tx,(varying float)This->nx,dg.Ty,(varying float)This->ny,dg.Ng);
}

Vec3f HairMaterial__eval(ISPCHairMaterial* uniform This, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi)
{
  return AnisotropicBlinn__eval((varying AnisotropicBlinn* uniform)&brdf,wo,wi);
}

Vec3f HairMaterial__sample(ISPCHairMaterial* uniform This, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  return AnisotropicBlinn__sample((varying AnisotropicBlinn* uniform)&brdf,wo,wi_o,s.x,s.y,s.x);
}

////////////////////////////////////////////////////////////////////////////////
//                              Material                                      //
////////////////////////////////////////////////////////////////////////////////

inline void Material__preprocess(ISPCMaterial** uniform materials, unsigned int materialID, uniform unsigned int numMaterials, BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Medium& medium)
{
  foreach_unique (id in materialID)
  {
    if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
    {
      ISPCMaterial* uniform material = materials[id];

      switch (material->type) {
      case MATERIAL_OBJ  : OBJMaterial__preprocess  ((uniform ISPCOBJMaterial* uniform)  material,brdf,wo,dg,medium); break;
      case MATERIAL_METAL: MetalMaterial__preprocess((uniform ISPCMetalMaterial* uniform)material,brdf,wo,dg,medium); break;
      case MATERIAL_REFLECTIVE_METAL: ReflectiveMetalMaterial__preprocess((uniform ISPCReflectiveMetalMaterial* uniform)material,brdf,wo,dg,medium); break;
      case MATERIAL_VELVET: VelvetMaterial__preprocess((uniform ISPCVelvetMaterial* uniform)material,brdf,wo,dg,medium); break;
      case MATERIAL_DIELECTRIC: DielectricMaterial__preprocess((uniform ISPCDielectricMaterial* uniform)material,brdf,wo,dg,medium); break;
      case MATERIAL_METALLIC_PAINT: MetallicPaintMaterial__preprocess((uniform ISPCMetallicPaintMaterial* uniform)material,brdf,wo,dg,medium); break;
      case MATERIAL_MATTE: MatteMaterial__preprocess((uniform ISPCMatteMaterial* uniform)material,brdf,wo,dg,medium); break;
      case MATERIAL_MIRROR: MirrorMaterial__preprocess((uniform ISPCMirrorMaterial* uniform)material,brdf,wo,dg,medium); break;
      case MATERIAL_THIN_DIELECTRIC: ThinDielectricMaterial__preprocess((uniform ISPCThinDielectricMaterial* uniform)material,brdf,wo,dg,medium); break;
      case MATERIAL_HAIR: HairMaterial__preprocess((uniform ISPCHairMaterial* uniform)material,brdf,wo,dg,medium); break;
      default: break;
      }
    }
  }
}

inline Vec3f Material__eval(ISPCMaterial** uniform materials, unsigned int materialID, uniform unsigned int numMaterials, const BRDF& brdf, const Vec3f& wo, const DifferentialGeometry& dg, const Vec3f& wi)
{
  Vec3f c = make_Vec3f(0.0f);
  foreach_unique (id in materialID)
  {
    if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
    {
      ISPCMaterial* uniform material = materials[id];
      switch (material->type) {
      case MATERIAL_OBJ  : c = OBJMaterial__eval  ((uniform ISPCOBJMaterial* uniform)  material, brdf, wo, dg, wi); break;
      case MATERIAL_METAL: c = MetalMaterial__eval((uniform ISPCMetalMaterial* uniform)material, brdf, wo, dg, wi); break;
      case MATERIAL_REFLECTIVE_METAL: c = ReflectiveMetalMaterial__eval((uniform ISPCReflectiveMetalMaterial* uniform)material, brdf, wo, dg, wi); break;
      case MATERIAL_VELVET: c = VelvetMaterial__eval((uniform ISPCVelvetMaterial* uniform)material, brdf, wo, dg, wi); break;
      case MATERIAL_DIELECTRIC: c = DielectricMaterial__eval((uniform ISPCDielectricMaterial* uniform)material, brdf, wo, dg, wi); break;
      case MATERIAL_METALLIC_PAINT: c = MetallicPaintMaterial__eval((uniform ISPCMetallicPaintMaterial* uniform)material, brdf, wo, dg, wi); break;
      case MATERIAL_MATTE: c = MatteMaterial__eval((uniform ISPCMatteMaterial* uniform)material, brdf, wo, dg, wi); break;
      case MATERIAL_MIRROR: c = MirrorMaterial__eval((uniform ISPCMirrorMaterial* uniform)material, brdf, wo, dg, wi); break;
      case MATERIAL_THIN_DIELECTRIC: c = ThinDielectricMaterial__eval((uniform ISPCThinDielectricMaterial* uniform)material, brdf, wo, dg, wi); break;
      case MATERIAL_HAIR: c = HairMaterial__eval((uniform ISPCHairMaterial* uniform)material, brdf, wo, dg, wi); break;
      default: c = make_Vec3f(0.0f);
      }
    }
  }
  return c;
}

inline Vec3f Material__sample(ISPCMaterial** uniform uniform materials, unsigned int materialID, uniform unsigned int numMaterials, const BRDF& brdf, const Vec3f& Lw, const Vec3f& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
{
  Vec3f c = make_Vec3f(0.0f);
  foreach_unique (id in materialID)
  {
    if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
    {
      ISPCMaterial* uniform material = materials[id];
      switch (material->type) {
      case MATERIAL_OBJ  : c = OBJMaterial__sample  ((uniform ISPCOBJMaterial* uniform)  material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_METAL: c = MetalMaterial__sample((uniform ISPCMetalMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_REFLECTIVE_METAL: c = ReflectiveMetalMaterial__sample((uniform ISPCReflectiveMetalMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_VELVET: c = VelvetMaterial__sample((uniform ISPCVelvetMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_DIELECTRIC: c = DielectricMaterial__sample((uniform ISPCDielectricMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_METALLIC_PAINT: c = MetallicPaintMaterial__sample((uniform ISPCMetallicPaintMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_MATTE: c = MatteMaterial__sample((uniform ISPCMatteMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_MIRROR: c = MirrorMaterial__sample((uniform ISPCMirrorMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_THIN_DIELECTRIC: c = ThinDielectricMaterial__sample((uniform ISPCThinDielectricMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      case MATERIAL_HAIR: c = HairMaterial__sample((uniform ISPCHairMaterial* uniform)material, brdf, Lw, wo, dg, wi_o, medium, s); break;
      default: wi_o = make_Sample3f(make_Vec3f(0.0f),0.0f); c = make_Vec3f(0.0f); break;
      }
    }
  }
  return c;
}


////////////////////////////////////////////////////////////////////////////////
//                               Scene                                        //
////////////////////////////////////////////////////////////////////////////////

/* scene data */
extern uniform ISPCScene* uniform g_ispc_scene;
RTCDevice g_device = NULL;
RTCScene g_scene = NULL;
extern uniform int g_spp;
extern uniform bool g_accumulate;

/* occlusion filter function */
void intersectionFilterReject(void* uniform ptr, RTCRay& ray);
void intersectionFilterOBJ(void* uniform ptr, RTCRay& ray);
void occlusionFilterOpaque(void* uniform ptr, RTCRay& ray);
void occlusionFilterOBJ(void* uniform ptr, RTCRay& ray);
void occlusionFilterHair(void* uniform ptr, RTCRay& ray);

/* accumulation buffer */
uniform Vec3fa* uniform g_accu = NULL;
uniform unsigned int g_accu_width = 0;
uniform unsigned int g_accu_height = 0;
uniform unsigned int g_accu_count = 0;
uniform Vec3fa g_accu_vx;
uniform Vec3fa g_accu_vy;
uniform Vec3fa g_accu_vz;
uniform Vec3fa g_accu_p;
extern uniform bool g_changed;
extern uniform int g_instancing_mode;


uniform bool g_animation = true;
uniform bool g_use_smooth_normals = false;
void device_key_pressed_handler(uniform int key)
{
  if (key == 32  /* */) g_animation = !g_animation;
  if (key == 110 /*n*/) { g_use_smooth_normals = !g_use_smooth_normals; g_changed = true; }
  else device_key_pressed_default(key);
}

void assignShaders(ISPCGeometry* uniform geometry)
{
  if (geometry->type == SUBDIV_MESH) {
    ISPCSubdivMesh* uniform mesh = (ISPCSubdivMesh* uniform ) geometry;
#if ENABLE_FILTER_FUNCTION == 1
    rtcSetOcclusionFilterFunction(mesh->geom.scene,mesh->geom.geomID,occlusionFilterOpaque);
#endif
  }
  else if (geometry->type == TRIANGLE_MESH) {
    ISPCTriangleMesh* uniform mesh = (ISPCTriangleMesh* uniform ) geometry;
#if ENABLE_FILTER_FUNCTION == 1
    rtcSetOcclusionFilterFunction(mesh->geom.scene,mesh->geom.geomID,occlusionFilterOpaque);

    ISPCMaterial* uniform material = g_ispc_scene->materials[mesh->geom.materialID];
    //if (material->type == MATERIAL_DIELECTRIC || material->type == MATERIAL_THIN_DIELECTRIC)
    //  rtcSetOcclusionFilterFunction(mesh->geom.scene,mesh->geom.geomID,intersectionFilterReject);
    //else
    if (material->type == MATERIAL_OBJ)
    {
      ISPCOBJMaterial* uniform obj = (ISPCOBJMaterial* uniform) material;
      if (obj->d != 1.0f || obj->map_d) {
        rtcSetIntersectionFilterFunction(mesh->geom.scene,mesh->geom.geomID,intersectionFilterOBJ);
        rtcSetOcclusionFilterFunction   (mesh->geom.scene,mesh->geom.geomID,occlusionFilterOBJ);
      }
    }
#endif
  }
  else if (geometry->type == QUAD_MESH) {
    ISPCQuadMesh* uniform mesh = (ISPCQuadMesh* uniform) geometry;
#if ENABLE_FILTER_FUNCTION == 1
    rtcSetOcclusionFilterFunction(mesh->geom.scene,mesh->geom.geomID,occlusionFilterOpaque);

    ISPCMaterial* uniform material = g_ispc_scene->materials[mesh->geom.materialID];
    //if (material->type == MATERIAL_DIELECTRIC || material->type == MATERIAL_THIN_DIELECTRIC)
    //  rtcSetOcclusionFilterFunction(mesh->geom.scene,mesh->geom.geomID,intersectionFilterReject);
    //else
    if (material->type == MATERIAL_OBJ)
    {
      ISPCOBJMaterial* uniform obj = (ISPCOBJMaterial* uniform) material;
      if (obj->d != 1.0f || obj->map_d) {
        rtcSetIntersectionFilterFunction(mesh->geom.scene,mesh->geom.geomID,intersectionFilterOBJ);
        rtcSetOcclusionFilterFunction   (mesh->geom.scene,mesh->geom.geomID,occlusionFilterOBJ);
      }
    }
#endif
  }
  else if (geometry->type == LINE_SEGMENTS) {
    ISPCLineSegments* uniform mesh = (ISPCLineSegments* uniform) geometry;
    rtcSetOcclusionFilterFunction(mesh->geom.scene,mesh->geom.geomID,occlusionFilterHair);
  }
  else if (geometry->type == HAIR_SET) {
    ISPCHairSet* uniform mesh = (ISPCHairSet* uniform) geometry;
    rtcSetOcclusionFilterFunction(mesh->geom.scene,mesh->geom.geomID,occlusionFilterHair);
  }
  else if (geometry->type == CURVES) {
    ISPCHairSet* uniform mesh = (ISPCHairSet* uniform) geometry;
    rtcSetOcclusionFilterFunction(mesh->geom.scene,mesh->geom.geomID,occlusionFilterHair);
  }
  else if (geometry->type == GROUP) {
    ISPCGroup* uniform group = (ISPCGroup* uniform) geometry;
    for (uniform size_t i=0; i<group->numGeometries; i++)
      assignShaders(group->geometries[i]);
  }
}

typedef uniform ISPCInstance* uniform ISPCInstance_ptr;
typedef uniform ISPCGeometry* uniform ISPCGeometry_ptr;

RTCScene convertScene(uniform ISPCScene* uniform scene_in)
{
  for (uniform size_t i=0; i<scene_in->numGeometries; i++)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      g_subdiv_mode = true; break;
    }
  }

  /* create scene */
  uniform int scene_flags = RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT;
  uniform int scene_aflags = RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING;

  if (g_subdiv_mode)
    scene_flags = RTC_SCENE_DYNAMIC | RTC_SCENE_INCOHERENT | RTC_SCENE_ROBUST;

  scene_aflags |= RTC_INTERPOLATE;

  RTCScene scene_out = ConvertScene(g_device, g_ispc_scene,(RTCSceneFlags)scene_flags, (RTCAlgorithmFlags) scene_aflags, RTC_GEOMETRY_STATIC);

  /* assign shaders */
  for (uniform unsigned int i=0; i<scene_in->numGeometries; i++) {
    assignShaders(scene_in->geometries[i]);
  }

  /* commit individual objects in case of instancing */
  if (g_instancing_mode == ISPC_INSTANCING_SCENE_GEOMETRY || g_instancing_mode == ISPC_INSTANCING_SCENE_GROUP)
  {
    for (uniform unsigned int i=0; i<scene_in->numGeometries; i++) {
      if (scene_in->geomID_to_scene[i]) rtcCommit(scene_in->geomID_to_scene[i]);
    }
  }

  /* commit changes to scene */
  //progressStart();
  //rtcSetProgressMonitorFunction(scene_out,(RTCProgressMonitorFunc)&progressMonitor,NULL);
  rtcCommit (scene_out);
  //rtcSetProgressMonitorFunction(scene_out,NULL,NULL);
  //progressEnd();

  return scene_out;
} // convertScene

inline Vec3f face_forward(const Vec3f& dir, const Vec3f& _Ng) {
  const Vec3f Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

inline void evalBezier(const ISPCHairSet* uniform hair, const int primID, const float t, Vec3fa& p, Vec3fa& dp)
{
  const float t0 = 1.0f - t, t1 = t;
  const Vec3fa* vertices = hair->positions[0];
  const ISPCHair* hairs = hair->hairs;

  const int i = hairs[primID].vertex;
  const Vec3fa p00 = vertices[i+0];
  const Vec3fa p01 = vertices[i+1];
  const Vec3fa p02 = vertices[i+2];
  const Vec3fa p03 = vertices[i+3];

  const Vec3fa p10 = p00 * t0 + p01 * t1;
  const Vec3fa p11 = p01 * t0 + p02 * t1;
  const Vec3fa p12 = p02 * t0 + p03 * t1;
  const Vec3fa p20 = p10 * t0 + p11 * t1;
  const Vec3fa p21 = p11 * t0 + p12 * t1;
  const Vec3fa p30 = p20 * t0 + p21 * t1;

  p = p30;
  dp = 3.0f*(p21-p20);
}

void postIntersectGeometry(const RTCRay& ray, DifferentialGeometry& dg, uniform ISPCGeometry* uniform geometry, int& materialID)
{
  if (geometry->type == TRIANGLE_MESH)
  {
    uniform ISPCTriangleMesh* uniform mesh = (uniform ISPCTriangleMesh* uniform) geometry;
    materialID = mesh->geom.materialID;
    if (mesh->texcoords)
    {
      ISPCTriangle* tri = &mesh->triangles[ray.primID];
      const Vec2f st0 = mesh->texcoords[tri->v0];
      const Vec2f st1 = mesh->texcoords[tri->v1];
      const Vec2f st2 = mesh->texcoords[tri->v2];
      const float u = ray.u, v = ray.v, w = 1.0f-ray.u-ray.v;
      const Vec2f st = w*st0 + u*st1 + v*st2;
      dg.u = st.x;
      dg.v = st.y;
    }
    if (mesh->normals)
    {
      if (mesh->numTimeSteps == 1)
      {
        ISPCTriangle* tri = &mesh->triangles[ray.primID];
        const Vec3f n0 = make_Vec3f(mesh->normals[0][tri->v0]);
        const Vec3f n1 = make_Vec3f(mesh->normals[0][tri->v1]);
        const Vec3f n2 = make_Vec3f(mesh->normals[0][tri->v2]);
        const float u = ray.u, v = ray.v, w = 1.0f-ray.u-ray.v;
        dg.Ns = w*n0 + u*n1 + v*n2;
      }
      else
      {
        ISPCTriangle* tri = &mesh->triangles[ray.primID];
        float f = mesh->numTimeSteps*ray.time;
        int itime = clamp((int)floor(f),0,(int)mesh->numTimeSteps-2);
        float t1 = f-itime;
        float t0 = 1.0f-t1;
        const Vec3f a0 = make_Vec3f(mesh->normals[itime+0][tri->v0]);
        const Vec3f a1 = make_Vec3f(mesh->normals[itime+0][tri->v1]);
        const Vec3f a2 = make_Vec3f(mesh->normals[itime+0][tri->v2]);
        const Vec3f b0 = make_Vec3f(mesh->normals[itime+1][tri->v0]);
        const Vec3f b1 = make_Vec3f(mesh->normals[itime+1][tri->v1]);
        const Vec3f b2 = make_Vec3f(mesh->normals[itime+1][tri->v2]);
        const Vec3f n0 = t0*a0 + t1*b0;
        const Vec3f n1 = t0*a1 + t1*b1;
        const Vec3f n2 = t0*a2 + t1*b2;
        const float u = ray.u, v = ray.v, w = 1.0f-ray.u-ray.v;
        dg.Ns = w*n0 + u*n1 + v*n2;
      }
    }
  }
  else if (geometry->type == QUAD_MESH)
  {
    uniform ISPCQuadMesh* uniform mesh = (uniform ISPCQuadMesh* uniform) geometry;
    materialID = mesh->geom.materialID;
    if (mesh->texcoords)
    {
      ISPCQuad* quad = &mesh->quads[ray.primID];
      const Vec2f st0 = mesh->texcoords[quad->v0];
      const Vec2f st1 = mesh->texcoords[quad->v1];
      const Vec2f st2 = mesh->texcoords[quad->v2];
      const Vec2f st3 = mesh->texcoords[quad->v3];
      if (ray.u+ray.v < 1.0f) {
        const float u = ray.u, v = ray.v; const float w = 1.0f-u-v;
        const Vec2f st = w*st0 + u*st1 + v*st3;
        dg.u = st.x;
        dg.v = st.y;
      } else {
        const float u = 1.0f-ray.u, v = 1.0f-ray.v; const float w = 1.0f-u-v;
        const Vec2f st = w*st2 + u*st3 + v*st1;
        dg.u = st.x;
        dg.v = st.y;
      }
    }
    if (mesh->normals)
    {
      if (mesh->numTimeSteps == 1)
      {
        ISPCQuad* quad = &mesh->quads[ray.primID];
        const Vec3f n0 = make_Vec3f(mesh->normals[0][quad->v0]);
        const Vec3f n1 = make_Vec3f(mesh->normals[0][quad->v1]);
        const Vec3f n2 = make_Vec3f(mesh->normals[0][quad->v2]);
        const Vec3f n3 = make_Vec3f(mesh->normals[0][quad->v3]);
        if (ray.u+ray.v < 1.0f) {
          const float u = ray.u, v = ray.v; const float w = 1.0f-u-v;
          dg.Ns = w*n0 + u*n1 + v*n3;
        } else {
          const float u = 1.0f-ray.u, v = 1.0f-ray.v; const float w = 1.0f-u-v;
          dg.Ns = w*n2 + u*n3 + v*n1;
        }
      }
      else
      {
        ISPCQuad* quad = &mesh->quads[ray.primID];
        float f = mesh->numTimeSteps*ray.time;
        int itime = clamp((int)floor(f),0,(int)mesh->numTimeSteps-2);
        float t1 = f-itime;
        float t0 = 1.0f-t1;
        const Vec3f a0 = make_Vec3f(mesh->normals[itime+0][quad->v0]);
        const Vec3f a1 = make_Vec3f(mesh->normals[itime+0][quad->v1]);
        const Vec3f a2 = make_Vec3f(mesh->normals[itime+0][quad->v2]);
        const Vec3f a3 = make_Vec3f(mesh->normals[itime+0][quad->v3]);
        const Vec3f b0 = make_Vec3f(mesh->normals[itime+1][quad->v0]);
        const Vec3f b1 = make_Vec3f(mesh->normals[itime+1][quad->v1]);
        const Vec3f b2 = make_Vec3f(mesh->normals[itime+1][quad->v2]);
        const Vec3f b3 = make_Vec3f(mesh->normals[itime+1][quad->v3]);
        const Vec3f n0 = t0*a0 + t1*b0;
        const Vec3f n1 = t0*a1 + t1*b1;
        const Vec3f n2 = t0*a2 + t1*b2;
        const Vec3f n3 = t0*a3 + t1*b3;
        if (ray.u+ray.v < 1.0f) {
          const float u = ray.u, v = ray.v; const float w = 1.0f-u-v;
          dg.Ns = w*n0 + u*n1 + v*n3;
        } else {
          const float u = 1.0f-ray.u, v = 1.0f-ray.v; const float w = 1.0f-u-v;
          dg.Ns = w*n2 + u*n3 + v*n1;
        }
      }
    }
  }
  else if (geometry->type == SUBDIV_MESH)
  {
    uniform ISPCSubdivMesh* uniform mesh = (uniform ISPCSubdivMesh* uniform) geometry;
    materialID = mesh->geom.materialID;
    const Vec2f st = getTextureCoordinatesSubdivMesh(mesh,ray.primID,ray.u,ray.v);
    dg.u = st.x;
    dg.v = st.y;
  }
  else if (geometry->type == LINE_SEGMENTS)
  {
    uniform ISPCLineSegments* uniform mesh = (uniform ISPCLineSegments* uniform) geometry;
    materialID = mesh->geom.materialID;
    const Vec3f dx = normalize(dg.Ng);
    const Vec3f dy = normalize(cross(neg(ray.dir),dx));
    const Vec3f dz = normalize(cross(dy,dx));
    dg.Tx = dx;
    dg.Ty = dy;
    dg.Ng = dg.Ns = dz;
    int vtx = mesh->indices[ray.primID];
    dg.tnear_eps = 1.1f*mesh->positions[0][vtx].w;
  }
  else if (geometry->type == HAIR_SET)
  {
    uniform ISPCHairSet* uniform mesh = (uniform ISPCHairSet* uniform) geometry;
    materialID = mesh->geom.materialID;
    Vec3fa p,dp; evalBezier(mesh,ray.primID,ray.u,p,dp);
    const Vec3f dx = normalize(dg.Ng);
    const Vec3f dy = normalize(cross(neg(ray.dir),dx));
    const Vec3f dz = normalize(cross(dy,dx));
    dg.Tx = dx;
    dg.Ty = dy;
    dg.Ng = dg.Ns = dz;
    dg.tnear_eps = 1.1f*p.w;
  }
  else if (geometry->type == CURVES)
  {
    uniform ISPCHairSet* uniform mesh = (uniform ISPCHairSet* uniform) geometry;
    materialID = mesh->geom.materialID;
    Vec3fa p,dp; evalBezier(mesh,ray.primID,ray.u,p,dp);
    if (length(dp) < 1E-6f) { // some hair are just points
      dg.Tx = make_Vec3f(1,0,0);
      dg.Ty = make_Vec3f(0,1,0);
      dg.Ng = dg.Ns = make_Vec3f(0,0,1);
    }
    else
    {
      const Vec3f dx = normalize(make_Vec3f(dp));
      const Vec3f dy = normalize(cross(make_Vec3f(dp),dg.Ng));
      const Vec3f dz = normalize(dg.Ng);
      dg.Tx = dx;
      dg.Ty = dy;
      dg.Ng = dg.Ns = dz;
    }
    dg.tnear_eps = 1024.0f*1.19209e-07f*max(max(abs(dg.P.x),abs(dg.P.y)),max(abs(dg.P.z),ray.tfar));
  }
  else if (geometry->type == GROUP) {
    foreach_unique (geomID in ray.geomID) {
      postIntersectGeometry(ray,dg,((uniform ISPCGroup*) geometry)->geometries[geomID],materialID);
    }
  }
  else
    assert(false);

  if (max(max(abs(dg.Ns.x), abs(dg.Ns.y)), abs(dg.Ns.z)) < 1E-4f)
	dg.Ns = make_Vec3f(1, 0, 0);
}

AffineSpace3f calculate_interpolated_space (uniform ISPCInstance* uniform instance, float gtime)
{
  if (instance->numTimeSteps == 1)
    return make_AffineSpace3f(instance->spaces[0]);

  /* calculate time segment itime and fractional time ftime */
  const int time_segments = instance->numTimeSteps-1;
  const float time = gtime*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(varying int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  return (1.0f-ftime)*make_AffineSpace3f(instance->spaces[itime+0]) + ftime*make_AffineSpace3f(instance->spaces[itime+1]);
}

inline int postIntersect(const RTCRay& ray, DifferentialGeometry& dg)
{
  dg.tnear_eps = 32.0f*1.19209e-07f*max(max(abs(dg.P.x),abs(dg.P.y)),max(abs(dg.P.z),ray.tfar));

  int materialID = 0;
  foreach_unique (instID in ray.instID) {
    foreach_unique (geomID in ray.geomID) {
      ISPCGeometry* uniform geometry = NULL;
      if (g_instancing_mode == ISPC_INSTANCING_SCENE_GEOMETRY || g_instancing_mode == ISPC_INSTANCING_SCENE_GROUP) {
        ISPCInstance* uniform instance = g_ispc_scene->geomID_to_inst[instID];
        geometry = g_ispc_scene->geometries[instance->geom.geomID];
      } else {
        geometry = g_ispc_scene->geometries[geomID];
      }
      postIntersectGeometry(ray,dg,geometry,materialID);
    }
  }

  if (g_instancing_mode != ISPC_INSTANCING_NONE)
  {
    foreach_unique (instID in ray.instID)
    {
      /* get instance and geometry pointers */
      ISPCInstance* uniform instance = g_ispc_scene->geomID_to_inst[instID];

      /* convert normals */
      //AffineSpace3f space = (1.0f-ray.time)*make_AffineSpace3f(instance->space0) + ray.time*make_AffineSpace3f(instance->space1);
      AffineSpace3f space = calculate_interpolated_space(instance,ray.time);
      dg.Ng = xfmVector(space,dg.Ng);
      dg.Ns = xfmVector(space,dg.Ns);
    }
  }

  return materialID;
}

void intersectionFilterReject(void* uniform ptr, RTCRay& ray) {
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
}

void intersectionFilterOBJ(void* uniform ptr, RTCRay& ray)
{
  /* compute differential geometry */
  DifferentialGeometry dg;
  dg.geomID = ray.geomID;
  dg.primID = ray.primID;
  dg.u = ray.u;
  dg.v = ray.v;
  dg.P  = ray.org+ray.tfar*ray.dir;
  dg.Ng = ray.Ng;
  dg.Ns = ray.Ng;
  int materialID = postIntersect(ray,dg);
  dg.Ng = face_forward(ray.dir,normalize(dg.Ng));
  if (length(dg.Ns) < 1E-6f) dg.Ns = dg.Ng;
  else dg.Ns = face_forward(ray.dir,normalize(dg.Ns));
  const Vec3f wo = neg(ray.dir);

  /* calculate BRDF */
  BRDF brdf; brdf.Kt = make_Vec3f(0,0,0);
  uniform int numMaterials = g_ispc_scene->numMaterials;
  ISPCMaterial** uniform material_array = &g_ispc_scene->materials[0];
  Medium medium = make_Medium_Vacuum();
  Material__preprocess(material_array,materialID,numMaterials,brdf,wo,dg,medium);
  if (min(min(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z) >= 1.0f)
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
}

void occlusionFilterOpaque(void* uniform ptr, RTCRay& ray) {
  ray.transparency = make_Vec3f(0.0f);
}

void occlusionFilterOBJ(void* uniform ptr, RTCRay& ray)
{
  /* compute differential geometry */
  DifferentialGeometry dg;
  dg.geomID = ray.geomID;
  dg.primID = ray.primID;
  dg.u = ray.u;
  dg.v = ray.v;
  dg.P  = ray.org+ray.tfar*ray.dir;
  dg.Ng = ray.Ng;
  dg.Ns = ray.Ng;
  int materialID = postIntersect(ray,dg);
  dg.Ng = face_forward(ray.dir,normalize(dg.Ng));
  dg.Ns = face_forward(ray.dir,normalize(dg.Ns));
  const Vec3f wo = neg(ray.dir);

  /* calculate BRDF */
  BRDF brdf; brdf.Kt = make_Vec3f(0,0,0);
  uniform int numMaterials = g_ispc_scene->numMaterials;
  ISPCMaterial** uniform material_array = &g_ispc_scene->materials[0];
  Medium medium = make_Medium_Vacuum();
  Material__preprocess(material_array,materialID,numMaterials,brdf,wo,dg,medium);

  ray.transparency = ray.transparency * brdf.Kt;
  if (max(max(ray.transparency.x,ray.transparency.y),ray.transparency.z) > 0.0f)
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
}

/* occlusion filter function */
void occlusionFilterHair(void* uniform ptr, RTCRay& ray)
{
  Vec3f Kt = make_Vec3f(0.0f);
  foreach_unique(geomID in ray.geomID)
  {
    uniform ISPCGeometry* uniform geometry = g_ispc_scene->geometries[geomID];
    if (geometry->type == LINE_SEGMENTS)
    {
      uniform int materialID = ((ISPCLineSegments* uniform)geometry)->geom.materialID;
      ISPCMaterial* uniform material = g_ispc_scene->materials[materialID];
      switch (material->type) {
      case MATERIAL_HAIR: Kt = make_Vec3f(((uniform ISPCHairMaterial* uniform)material)->Kt); break;
      default: break;
      }
    }
    else if (geometry->type == HAIR_SET)
    {
      uniform int materialID = ((ISPCHairSet* uniform)geometry)->geom.materialID;
      ISPCMaterial* uniform material = g_ispc_scene->materials[materialID];
      switch (material->type) {
      case MATERIAL_HAIR: Kt = make_Vec3f(((uniform ISPCHairMaterial* uniform)material)->Kt); break;
      default: break;
      }
    }
    else if (geometry->type == CURVES)
    {
      /*if (dot(ray.dir,ray.Ng) > 0.0f) {
        Kt = make_Vec3f(1.0f);
      }
      else*/
      {
        uniform int materialID = ((ISPCHairSet* uniform)geometry)->geom.materialID;
        ISPCMaterial* uniform material = g_ispc_scene->materials[materialID];
        switch (material->type) {
        case MATERIAL_HAIR: Kt = make_Vec3f(((uniform ISPCHairMaterial* uniform)material)->Kt); break;
        default: break;
        }
      }
    }
  }

  Kt = Kt * ray.transparency;
  ray.transparency = Kt;
  if (max(max(ray.transparency.x,ray.transparency.y),ray.transparency.z) > 0.0f)
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
}

Vec3f renderPixelFunction(float x, float y, RandomSampler& sampler, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  /* radiance accumulator and weight */
  Vec3f L = make_Vec3f(0.0f);
  Vec3f Lw = make_Vec3f(1.0f);
  Medium medium = make_Medium_Vacuum();
  float time = RandomSampler_get1D(sampler);

  /* initialize ray */
  RTCRay ray = make_Ray(make_Vec3f(camera.xfm.p),
                        make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)),0.0f,inf,time);

  DifferentialGeometry dg;

  /* iterative path tracer loop */
  for (uniform int i=0; i<MAX_PATH_LENGTH; i++)
  {
    /* terminate if contribution too low */
    if (max(Lw.x,max(Lw.y,Lw.z)) < 0.01f)
      break;

    /* intersect ray with scene */
    uniform RTCIntersectContext context;
    context.flags = (i == 0) ? g_iflags_coherent : g_iflags_incoherent;
    rtcIntersectEx(g_scene,&context,ray);
    RayStats_addRay(stats);
    const Vec3f wo = neg(ray.dir);

    /* invoke environment lights if nothing hit */
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
    {
      //L = L + Lw*make_Vec3f(1.0f);

      /* iterate over all lights */
      for (uniform size_t i=0; i<g_ispc_scene->numLights; i++)
      {
        const uniform Light* uniform l = g_ispc_scene->lights[i];
        Light_EvalRes le = l->eval(l,dg,ray.dir);
        L = L + Lw*le.value;
      }

      break;
    }
    Vec3f Ns = normalize(ray.Ng);

    if (g_use_smooth_normals)
      if (ray.geomID != RTC_INVALID_GEOMETRY_ID) // FIXME: workaround for ISPC bug, location reached with empty execution mask
    {
      Vec3f dPdu,dPdv;
      foreach_unique (geomID in ray.geomID) {
        rtcInterpolate(g_scene,geomID,ray.primID,ray.u,ray.v,RTC_VERTEX_BUFFER0,NULL,&dPdu.x,&dPdv.x,3);
      }
      Ns = normalize(cross(dPdv,dPdu));
    }

    /* compute differential geometry */
    dg.geomID = ray.geomID;
    dg.primID = ray.primID;
    dg.u = ray.u;
    dg.v = ray.v;
    dg.P  = ray.org+ray.tfar*ray.dir;
    dg.Ng = ray.Ng;
    dg.Ns = Ns;
    int materialID = postIntersect(ray,dg);
    dg.Ng = face_forward(ray.dir,normalize(dg.Ng));
    dg.Ns = face_forward(ray.dir,normalize(dg.Ns));

    /*! Compute  simple volumetric effect. */
    Vec3f c = make_Vec3f(1.0f);
    const Vec3f transmission = medium.transmission;
    if (ne(transmission,make_Vec3f(1.0f)))
      c = c * pow(transmission,ray.tfar);

    /* calculate BRDF */
    BRDF brdf;
    uniform int numMaterials = g_ispc_scene->numMaterials;
    ISPCMaterial** uniform material_array = &g_ispc_scene->materials[0];
    Material__preprocess(material_array,materialID,numMaterials,brdf,wo,dg,medium);

    /* sample BRDF at hit point */
    Sample3f wi1;
    c = c * Material__sample(material_array,materialID,numMaterials,brdf,Lw, wo, dg, wi1, medium, RandomSampler_get2D(sampler));

    /* iterate over lights */
    context.flags = g_iflags_incoherent;
    for (uniform size_t i=0; i<g_ispc_scene->numLights; i++)
    {
      const uniform Light* uniform l = g_ispc_scene->lights[i];
      Light_SampleRes ls = l->sample(l,dg,RandomSampler_get2D(sampler));
      if (ls.pdf <= 0.0f) continue;
      RTCRay shadow = make_Ray(dg.P,ls.dir,dg.tnear_eps,ls.dist,time); shadow.transparency = make_Vec3f(1.0f);
      rtcOccludedEx(g_scene,&context,shadow);
      RayStats_addShadowRay(stats);
      //if (shadow.geomID != RTC_INVALID_GEOMETRY_ID) continue;
      if (max(max(shadow.transparency.x,shadow.transparency.y),shadow.transparency.z) > 0.0f)
        L = L + Lw*ls.weight*shadow.transparency*Material__eval(material_array,materialID,numMaterials,brdf,wo,dg,ls.dir);
    }

    if (wi1.pdf <= 1E-4f /* 0.0f */) break;
    Lw = Lw*c/wi1.pdf;

    /* setup secondary ray */
    float sign = dot(wi1.v,dg.Ng) < 0.0f ? -1.0f : 1.0f;
    dg.P = dg.P + sign*dg.tnear_eps*dg.Ng;
    ray = make_Ray(dg.P,normalize(wi1.v),dg.tnear_eps,inf,time);
  }
  return L;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  RandomSampler sampler;

  Vec3f L = make_Vec3f(0.0f);

  for (uniform int i=0; i<g_spp; i++)
  {
    RandomSampler_init(sampler, (int)x, (int)y, g_accu_count*g_spp+i);

    /* calculate pixel color */
    float fx = x + RandomSampler_get1D(sampler);
    float fy = y + RandomSampler_get1D(sampler);
    L = L + renderPixelFunction(fx,fy,sampler,camera,stats);
  }
  L = L/g_spp;
  return L;
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
    Vec3fa accu_color = g_accu[y*width+x] + make_Vec3fa(color.x,color.y,color.z,1.0f); g_accu[y*width+x] = accu_color;
    float f = rcp(max(0.001f,accu_color.w));
    unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
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


/***************************************************************************************/

inline uniform float updateEdgeLevel( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos, const uniform size_t e0, const uniform size_t e1)
{
  const uniform Vec3fa v0 = mesh->positions[0][mesh->position_indices[e0]];
  const uniform Vec3fa v1 = mesh->positions[0][mesh->position_indices[e1]];
  const uniform Vec3fa edge = v1-v0;
  const uniform Vec3fa P = 0.5f*(v1+v0);
  const uniform Vec3fa dist = cam_pos - P;
  return max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
}

void updateEdgeLevelBuffer( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos, uniform size_t startID, uniform size_t endID )
{
  for (uniform size_t f=startID; f<endID;f++)
  {
    uniform unsigned int e = mesh->face_offsets[f];
    uniform unsigned int N = mesh->verticesPerFace[f];
    if (N == 4) /* fast path for quads */
      for (uniform size_t i=0; i<4; i++)
        mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%4);
       else if (N == 3) /* fast path for triangles */
         for (uniform size_t i=0; i<3; i++)
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%3);
       else /* fast path for general polygons */
         for (uniform size_t i=0; i<N; i++)
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%N);
  }
}

#if defined(ISPC)
task void updateEdgeLevelBufferTask( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos )
{
  const uniform size_t size = mesh->numFaces;
  const uniform size_t startID = ((taskIndex+0)*size)/taskCount;
  const uniform size_t endID   = ((taskIndex+1)*size)/taskCount;
  updateEdgeLevelBuffer(mesh,cam_pos,startID,endID);
}
#endif

void updateEdgeLevels(uniform ISPCScene* uniform scene_in, const uniform Vec3fa& cam_pos)
{
  for (uniform size_t g=0; g<scene_in->numGeometries; g++)
  {
    uniform ISPCGeometry* uniform geometry = g_ispc_scene->geometries[g];
    if (geometry->type != SUBDIV_MESH) continue;
    uniform ISPCSubdivMesh* uniform mesh = (uniform ISPCSubdivMesh* uniform) geometry;
    uniform unsigned int geomID = mesh->geom.geomID;
#if defined(ISPC)
    launch[ (mesh->numFaces+4095)/4096 ] updateEdgeLevelBufferTask(mesh,cam_pos); sync;
#else
      updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
#endif
   rtcUpdateBuffer(g_scene,geomID,RTC_LEVEL_BUFFER);
  }
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize last seen camera */
  g_accu_vx = make_Vec3fa(0.0f);
  g_accu_vy = make_Vec3fa(0.0f);
  g_accu_vz = make_Vec3fa(0.0f);
  g_accu_p  = make_Vec3fa(0.0f);

  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(NULL,rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction2(g_device,error_handler,NULL);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_handler;

#if ENABLE_FILTER_FUNCTION == 0
  print("Warning: filter functions disabled\n");
#endif

} // device_init

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  /* create scene */
  if (g_scene == NULL) {
    g_scene = convertScene(g_ispc_scene);
    if (g_subdiv_mode) updateEdgeLevels(g_ispc_scene,camera.xfm.p);
    rtcCommit (g_scene);
  }

  /* create accumulator */
  if (g_accu_width != width || g_accu_height != height) {
    delete[] g_accu;
    g_accu = uniform new uniform Vec3fa[width*height];
    g_accu_width = width;
    g_accu_height = height;
    for (uniform size_t i=0; i<width*height; i++)
      g_accu[i] = make_Vec3fa(0.0f);
  }

  /* reset accumulator */
  uniform bool camera_changed = g_changed || !g_accumulate; g_changed = false;
  camera_changed |= ne(g_accu_vx,camera.xfm.l.vx); g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(g_accu_vy,camera.xfm.l.vy); g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(g_accu_vz,camera.xfm.l.vz); g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(g_accu_p, camera.xfm.p);    g_accu_p  = camera.xfm.p;

  if (camera_changed)
  {
    g_accu_count=0;
    for (uniform size_t i=0; i<width*height; i++)
      g_accu[i] = make_Vec3fa(0.0f);

    if (g_subdiv_mode) {
      updateEdgeLevels(g_ispc_scene,camera.xfm.p);
      rtcCommit (g_scene);
    }
  }
  else
    g_accu_count++;

  /* render image */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
  //rtcDebug();
} // device_render

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene); g_scene = NULL;
  rtcDeleteDevice(g_device); g_device = NULL;
  delete[] g_accu; g_accu = NULL;
  g_accu_width = 0;
  g_accu_height = 0;
  g_accu_count = 0;
} // device_cleanup
