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

#pragma once

#include "../default.h"
#include "texture.h"

namespace embree
{
  enum MaterialTy 
  {
    MATERIAL_OBJ, 
    MATERIAL_THIN_DIELECTRIC, 
    MATERIAL_METAL, 
    MATERIAL_VELVET, 
    MATERIAL_DIELECTRIC, 
    MATERIAL_METALLIC_PAINT, 
    MATERIAL_MATTE, 
    MATERIAL_MIRROR, 
    MATERIAL_REFLECTIVE_METAL,
    MATERIAL_HAIR
  };
  
  struct MatteMaterial
  {
  public:
    MatteMaterial (const Vec3fa& reflectance)
    : ty(MATERIAL_MATTE), reflectance(reflectance) {}
    
  public:
    int ty;
    int align[3];
    Vec3fa reflectance;
  };
  
  struct MirrorMaterial
  {
  public:
    MirrorMaterial (const Vec3fa& reflectance)
    : ty(MATERIAL_MIRROR), reflectance(reflectance) {}
    
  public:
    int ty;
    int align[3];
    Vec3fa reflectance;
  };
  
  struct ThinDielectricMaterial
  {
  public:
    ThinDielectricMaterial (const Vec3fa& transmission, const float eta, const float thickness)
    : ty(MATERIAL_THIN_DIELECTRIC), transmission(transmission), eta(eta), thickness(thickness), transmissionFactor(log(transmission)*thickness) {}
    
  public:
    int ty;
    int align[3];
    Vec3fa transmission;
    Vec3fa transmissionFactor;
    float eta;
    float thickness;
  };
  
  /*! OBJ material */
  struct OBJMaterial
  {
  public:
    OBJMaterial ()
    : ty(MATERIAL_OBJ), illum(0), d(1.f), Ns(1.f), Ni(1.f), Ka(0.f), Kd(1.f), Ks(0.f), Kt(1.0f), map_d(nullptr), map_Kd(nullptr), map_Displ(nullptr) {}
    
    OBJMaterial (float d, const Vec3fa& Kd, const Vec3fa& Ks, const float Ns)
    : ty(MATERIAL_OBJ), illum(0), d(d), Ns(Ns), Ni(1.f), Ka(0.f), Kd(Kd), Ks(Ks), Kt(1.0f), map_d(nullptr), map_Kd(nullptr), map_Displ(nullptr) {}
    
    OBJMaterial (float d, const Texture* map_d, const Vec3fa& Kd, const Texture* map_Kd, const Vec3fa& Ks, const Texture* map_Ks, const float Ns, const Texture* map_Ns, const Texture* map_Bump)
    : ty(MATERIAL_OBJ), illum(0), d(d), Ns(Ns), Ni(1.f), Ka(0.f), Kd(Kd), Ks(Ks), Kt(1.0f), map_d(map_d), map_Kd(map_Kd), map_Displ(nullptr) {}
    
    ~OBJMaterial() { // FIXME: destructor never called!
    }
    
  public:
    int ty;
    int align[3];
    
    int illum;             /*< illumination model */
    float d;               /*< dissolve factor, 1=opaque, 0=transparent */
    float Ns;              /*< specular exponent */
    float Ni;              /*< optical density for the surface (index of refraction) */
    
    Vec3fa Ka;              /*< ambient reflectivity */
    Vec3fa Kd;              /*< diffuse reflectivity */
    Vec3fa Ks;              /*< specular reflectivity */
    Vec3fa Kt;              /*< transmission filter */

    const Texture* map_d;            /*< d texture */
    const Texture* map_Kd;           /*< Kd texture */
    const Texture* map_Displ;        /*< Displ texture */
  };
  
  struct MetalMaterial
  {
  public:
    MetalMaterial (const Vec3fa& reflectance, const Vec3fa& eta, const Vec3fa& k)
    : ty(MATERIAL_REFLECTIVE_METAL), reflectance(reflectance), eta(eta), k(k), roughness(0.0f) {}
    
    MetalMaterial (const Vec3fa& reflectance, const Vec3fa& eta, const Vec3fa& k, const float roughness)
    : ty(MATERIAL_METAL), reflectance(reflectance), eta(eta), k(k), roughness(roughness) {}
    
  public:
    int ty;
    int align[3];
    
    Vec3fa reflectance;
    Vec3fa eta;
    Vec3fa k;
    float roughness;
  };

  typedef MetalMaterial ReflectiveMetalMaterial;
  
  struct VelvetMaterial
  {
    VelvetMaterial (const Vec3fa& reflectance, const float backScattering, const Vec3fa& horizonScatteringColor, const float horizonScatteringFallOff)
    : ty(MATERIAL_VELVET), reflectance(reflectance), backScattering(backScattering), horizonScatteringColor(horizonScatteringColor), horizonScatteringFallOff(horizonScatteringFallOff) {}
    
  public:
    int ty;
    int align[3];
    
    Vec3fa reflectance;
    Vec3fa horizonScatteringColor;
    float backScattering;
    float horizonScatteringFallOff;
  };
  
  struct DielectricMaterial
  {
    DielectricMaterial (const Vec3fa& transmissionOutside, const Vec3fa& transmissionInside, const float etaOutside, const float etaInside)
    : ty(MATERIAL_DIELECTRIC), transmissionOutside(transmissionOutside), transmissionInside(transmissionInside), etaOutside(etaOutside), etaInside(etaInside) {}
    
  public:
    int ty;
    int align[3];
    Vec3fa transmissionOutside;
    Vec3fa transmissionInside;
    float etaOutside;
    float etaInside;
  };
  
  struct MetallicPaintMaterial
  {
    MetallicPaintMaterial (const Vec3fa& shadeColor, const Vec3fa& glitterColor, float glitterSpread, float eta)
    : ty(MATERIAL_METALLIC_PAINT), shadeColor(shadeColor), glitterColor(glitterColor), glitterSpread(glitterSpread), eta(eta) {}
    
  public:
    int ty;
    int align[3];
    Vec3fa shadeColor;
    Vec3fa glitterColor;
    float glitterSpread;
    float eta;
  };

  struct HairMaterial
  {
    HairMaterial (const Vec3fa& Kr, const Vec3fa& Kt, float nx, float ny)
    : ty(MATERIAL_HAIR), Kr(Kr), Kt(Kt), nx(nx), ny(ny) {}
    
  public:
    int ty;
    int align[3];
    Vec3fa Kr;
    Vec3fa Kt;
    float nx;
    float ny;
  };
  
  /*! Material */
  struct Material
  {
  public:
    Material () { memset(this,0,sizeof(Material)); }
    Material (const OBJMaterial& in) { *((OBJMaterial*)this) = in; }
    OBJMaterial& obj() { return *(OBJMaterial*)this; }
    
  public:
    int ty;
    int align[3];
    Vec3fa v[7];
  };
}
