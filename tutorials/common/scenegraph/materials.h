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

#if defined (CPPTUTORIAL) && !defined(_MATERIALS_H_CPPTUTORIAL) || !defined(_MATERIALS_H_)

#if defined (CPPTUTORIAL)
#define _MATERIALS_H_CPPTUTORIAL
#else
#define _MATERIALS_H_
#endif

//#pragma once

#if !defined(ISPC)
#include "texture.h"
#include "scenegraph.h"
#endif

#if !defined(ISPC)
namespace embree
{
#endif

#if defined(ISPC) || defined(CPPTUTORIAL)
#define MATERIAL_BASE_CLASS
#define PREFIX(x) ISPC##x
#else
#define MATERIAL_BASE_CLASS : public SceneGraph::MaterialNode
#define PREFIX(x) x
#endif

#if !defined(CPPTUTORIAL)
  enum MaterialType
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
#endif

  struct PREFIX(Material)
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    Material (MaterialType type)
     : type(type) {}
#endif
#if !defined(ISPC)
    __aligned(16)
#endif
    int type;
    int align[3];
  };

  struct PREFIX(MatteMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    MatteMaterial (const Vec3fa& reflectance)
      : base(MATERIAL_MATTE), reflectance(reflectance) {}
    virtual Material* material() { return &base; }
#endif

    PREFIX(Material) base;
    Vec3fa reflectance;
  };

  struct PREFIX(MirrorMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    MirrorMaterial (const Vec3fa& reflectance)
      : base(MATERIAL_MIRROR), reflectance(reflectance) {}
    virtual Material* material() { return &base; }
#endif

    PREFIX(Material) base;
    Vec3fa reflectance;
  };

  struct PREFIX(ThinDielectricMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    ThinDielectricMaterial (const Vec3fa& transmission, const float eta, const float thickness)
      : base(MATERIAL_THIN_DIELECTRIC), transmission(transmission), transmissionFactor(log(transmission)*thickness), eta(eta), thickness(thickness) {}
    virtual Material* material() { return &base; }
#endif

    PREFIX(Material) base;
    Vec3fa transmission;
    Vec3fa transmissionFactor;
    float eta;
    float thickness;
  };

  struct PREFIX(OBJMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    OBJMaterial (const std::string name = "")
      : SceneGraph::MaterialNode(name), base(MATERIAL_OBJ), illum(0), d(1.f), Ns(1.f), Ni(1.f), Ka(0.f), Kd(1.f), Ks(0.f), Kt(1.0f), map_d(nullptr), map_Kd(nullptr), map_Displ(nullptr) {}

    OBJMaterial (float d, const Vec3fa& Kd, const Vec3fa& Ks, const float Ns, const std::string name = "")
      : base(MATERIAL_OBJ), illum(0), d(d), Ns(Ns), Ni(1.f), Ka(0.f), Kd(Kd), Ks(Ks), Kt(1.0f), map_d(nullptr), map_Kd(nullptr), map_Displ(nullptr) {}

    OBJMaterial (float d, const std::shared_ptr<Texture> map_d, 
                 const Vec3fa& Kd, const std::shared_ptr<Texture> map_Kd, 
                 const Vec3fa& Ks, const std::shared_ptr<Texture> map_Ks, 
                 const float Ns, const std::shared_ptr<Texture> map_Ns, 
                 const std::shared_ptr<Texture> map_Displ)
      : base(MATERIAL_OBJ), illum(0), d(d), Ns(Ns), Ni(1.f), Ka(0.f), Kd(Kd), Ks(Ks), Kt(1.0f), 
      map_d(nullptr), map_Kd(nullptr), map_Ks(nullptr), map_Ns(nullptr), map_Displ(nullptr),
      _map_d(map_d), _map_Kd(map_Kd), _map_Ks(map_Ks), _map_Ns(map_Ns), _map_Displ(map_Displ) {}

    virtual Material* material() 
    { 
      map_d = _map_d.get();
      map_Kd = _map_Kd.get();
      map_Ks = _map_Ks.get();
      map_Ns = _map_Ns.get();
      map_Displ = _map_Displ.get();
      return &base; 
    }

    ~OBJMaterial() { // FIXME: destructor never called
    }
#endif

    PREFIX(Material) base;
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
    const Texture* map_Ks;           /*< Ks texture */
    const Texture* map_Ns;           /*< Ns texture */
    const Texture* map_Displ;        /*< Displ texture */

#if !defined(ISPC) && !defined(CPPTUTORIAL)
    std::shared_ptr<Texture> _map_d;            /*< d texture */
    std::shared_ptr<Texture> _map_Kd;           /*< Kd texture */
    std::shared_ptr<Texture> _map_Ks;           /*< Ks texture */
    std::shared_ptr<Texture> _map_Ns;           /*< Ns texture */
    std::shared_ptr<Texture> _map_Displ;        /*< Displ texture */
#endif
  };

  struct PREFIX(MetalMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    MetalMaterial (const Vec3fa& reflectance, const Vec3fa& eta, const Vec3fa& k)
      : base(MATERIAL_REFLECTIVE_METAL), reflectance(reflectance), eta(eta), k(k), roughness(0.0f) {}

    MetalMaterial (const Vec3fa& reflectance, const Vec3fa& eta, const Vec3fa& k, const float roughness)
      : base(MATERIAL_METAL), reflectance(reflectance), eta(eta), k(k), roughness(roughness) {}
    
    virtual Material* material() { return &base; }
#endif

    PREFIX(Material) base;
    Vec3fa reflectance;
    Vec3fa eta;
    Vec3fa k;
    float roughness;
  };

  typedef PREFIX(MetalMaterial) PREFIX(ReflectiveMetalMaterial);

  struct PREFIX(VelvetMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    VelvetMaterial (const Vec3fa& reflectance, const float backScattering, const Vec3fa& horizonScatteringColor, const float horizonScatteringFallOff)
      : base(MATERIAL_VELVET), reflectance(reflectance), horizonScatteringColor(horizonScatteringColor), backScattering(backScattering), horizonScatteringFallOff(horizonScatteringFallOff) {}

    virtual Material* material() { return &base; }
#endif

    PREFIX(Material) base;
    Vec3fa reflectance;
    Vec3fa horizonScatteringColor;
    float backScattering;
    float horizonScatteringFallOff;
  };

  struct PREFIX(DielectricMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    DielectricMaterial (const Vec3fa& transmissionOutside, const Vec3fa& transmissionInside, const float etaOutside, const float etaInside)
      : base(MATERIAL_DIELECTRIC), transmissionOutside(transmissionOutside), transmissionInside(transmissionInside), etaOutside(etaOutside), etaInside(etaInside) {}

    virtual Material* material() { return &base; }
#endif

    PREFIX(Material) base;
    Vec3fa transmissionOutside;
    Vec3fa transmissionInside;
    float etaOutside;
    float etaInside;
  };

  struct PREFIX(MetallicPaintMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    MetallicPaintMaterial (const Vec3fa& shadeColor, const Vec3fa& glitterColor, float glitterSpread, float eta)
      : base(MATERIAL_METALLIC_PAINT), shadeColor(shadeColor), glitterColor(glitterColor), glitterSpread(glitterSpread), eta(eta) {}

    virtual Material* material() { return &base; }
#endif

    PREFIX(Material) base;
    Vec3fa shadeColor;
    Vec3fa glitterColor;
    float glitterSpread;
    float eta;
  };

  struct PREFIX(HairMaterial) MATERIAL_BASE_CLASS
  {
#if !defined(ISPC) && !defined(CPPTUTORIAL)
    HairMaterial (const Vec3fa& Kr, const Vec3fa& Kt, float nx, float ny)
      : base(MATERIAL_HAIR), Kr(Kr), Kt(Kt), nx(nx), ny(ny) {}

    virtual Material* material() { return &base; }
#endif

    PREFIX(Material) base;
    Vec3fa Kr;
    Vec3fa Kt;
    float nx;
    float ny;
  };

#undef MATERIAL_BASE_CLASS
#undef PREFIX

#if !defined(ISPC)
}
#endif

#endif
