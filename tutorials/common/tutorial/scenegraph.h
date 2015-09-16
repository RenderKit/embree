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

#include "../../../common/sys/ref.h"
#include "../../../common/sys/vector.h"
#include "../../../common/math/vec2.h"
#include "../../../common/math/vec3.h"
#include "../../../common/math/affinespace.h"

#include <vector>
#include <memory>


namespace embree
{
  enum MaterialTy { MATERIAL_OBJ, MATERIAL_THIN_DIELECTRIC, MATERIAL_METAL, MATERIAL_VELVET, MATERIAL_DIELECTRIC, MATERIAL_METALLIC_PAINT, MATERIAL_MATTE, MATERIAL_MIRROR, MATERIAL_REFLECTIVE_METAL };
  
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
    : ty(MATERIAL_THIN_DIELECTRIC), transmission(log(transmission)*thickness), eta(eta) {}
    
  public:
    int ty;
    int align[3];
    Vec3fa transmission;
    float eta;
  };
  
  /*! OBJ material */
  struct OBJMaterial
  {
  public:
    OBJMaterial ()
    : ty(MATERIAL_OBJ), illum(0), d(1.f), Ns(1.f), Ni(1.f), Ka(0.f), Kd(1.f), Ks(0.f), Kt(0.0f), map_Kd(nullptr), map_Displ(nullptr)
    {};
    
    OBJMaterial (float d, const Vec3fa& Kd, const Vec3fa& Ks, const float Ns)
    : ty(MATERIAL_OBJ), illum(0), d(d), Ns(Ns), Ni(1.f), Ka(0.f), Kd(Kd), Ks(Ks), Kt(0.0f), map_Kd(nullptr), map_Displ(nullptr)
    {};
    
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
    
    void* map_Kd;       /*< dummy */
    void* map_Displ;       /*< dummy */
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
  
  struct AmbientLight
  {
  public:
    AmbientLight () {}
    
    AmbientLight (const Vec3fa& L) 
    : L(L) {}
    
    const AmbientLight transform(const AffineSpace3fa& space) const {
      return AmbientLight(L);
    }
    
  public:
    Vec3fa L;                  //!< radiance of ambient light
  };
  
  struct PointLight
  {
  public:
    PointLight() {}
    
    PointLight (const Vec3fa& P, const Vec3fa& I) 
    : P(P), I(I) {}
    
    const PointLight transform(const AffineSpace3fa& space) const {
      return PointLight(xfmPoint(space,P),I);
    }
    
  public:
    Vec3fa P;                  //!< position of point light
    Vec3fa I;                  //!< radiant intensity of point light
  };
  
  struct DirectionalLight
  {
  public:
    DirectionalLight () {}
    
    DirectionalLight (const Vec3fa& D, const Vec3fa& E) 
    : D(D), E(E) {}
    
    const DirectionalLight transform(const AffineSpace3fa& space) const {
      return DirectionalLight(xfmVector(space,D),E);
    }
    
  public:
    Vec3fa D;                  //!< Light direction
    Vec3fa E;                  //!< Irradiance (W/m^2)
  };
  
  struct SpotLight
  {
    SpotLight () {}
    
    SpotLight (const Vec3fa& P, const Vec3fa& D, const Vec3fa& I, const float angleMin, const float angleMax)
    : P(P), D(D), I(I), angleMin(angleMin), angleMax(angleMax) {}
    
    const SpotLight transform(const AffineSpace3fa& space) const {
      return SpotLight(xfmPoint(space,P),xfmVector(space,D),I,angleMin,angleMax);
    }
    
  public:
    Vec3fa P;                 //!< Position of the spot light
    Vec3fa D;                 //!< Light direction of the spot light
    Vec3fa I;                 //!< Radiant intensity (W/sr)
    float angleMin, angleMax; //!< Linear falloff region
  };
  
  struct DistantLight
  {
  public:
    DistantLight () {}
    
    DistantLight (const Vec3fa& D, const Vec3fa& L, const float halfAngle) 
    : D(D), L(L), halfAngle(halfAngle), radHalfAngle(deg2rad(halfAngle)), cosHalfAngle(cos(deg2rad(halfAngle))) {}
    
    const DistantLight transform(const AffineSpace3fa& space) const {
      return DistantLight(xfmVector(space,D),L,halfAngle);
    }
    
  public:
    Vec3fa D;             //!< Light direction
    Vec3fa L;             //!< Radiance (W/(m^2*sr))
    float halfAngle;     //!< Half illumination angle
    float radHalfAngle;  //!< Half illumination angle in radians
    float cosHalfAngle;  //!< Cosine of half illumination angle
  };
  
  struct TriangleLight
  {
    TriangleLight() {}
    
    TriangleLight (const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& L) 
    : v0(v0), v1(v1), v2(v2), L(L) {}
    
    const TriangleLight transform(const AffineSpace3fa& space) const {
      return TriangleLight(xfmPoint(space,v0),xfmPoint(space,v1),xfmPoint(space,v2),L);
    }
    
  public:
    Vec3fa v0;
    Vec3fa v1;
    Vec3fa v2;
    Vec3fa L;
  };
  
  struct QuadLight
  {
    QuadLight() {}
    
    QuadLight (const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const Vec3fa& L) 
    : v0(v0), v1(v1), v2(v2), v3(v3), L(L) {}
    
    const QuadLight transform(const AffineSpace3fa& space) const {
      return QuadLight(xfmPoint(space,v0),xfmPoint(space,v1),xfmPoint(space,v2),xfmPoint(space,v3),L);
    }
    
  public:
    Vec3fa v0;
    Vec3fa v1;
    Vec3fa v2;
    Vec3fa v3;
    Vec3fa L;
  };
  
  struct SceneGraph
  {
    struct Node : public RefCount
    {
      Node () 
        : name("unnamed") {}
      
      Node (const std::string& name)
        : name(name) {}
      
      std::string name;
    };
    
    struct TransformNode : public Node
    {
      ALIGNED_STRUCT;
      TransformNode (const AffineSpace3fa& xfm, const Ref<Node>& child)
        : xfm(xfm), child(child) {}
      
    public:
      AffineSpace3fa xfm; 
      Ref<Node> child;
    };
    
    struct GroupNode : public Node
    { 
      GroupNode (const size_t N = 0) { 
        children.resize(N); 
      }
      
      void add(const Ref<Node>& node) {
        children.push_back(node);
      }
      
      void set(const size_t i, const Ref<Node>& node) {
        children[i] = node;
      }
      
    public:
      std::vector<Ref<Node> > children;
    };
    
    template<typename Light>
      struct LightNode : public Node
    {
      ALIGNED_STRUCT;

    LightNode(const Light& light)
      : light(light) {}
      
      Light light;
    };
    
    struct MaterialNode : public Node
    {
      ALIGNED_STRUCT;

    MaterialNode(const Material& material)
      : material(material) {}
      
      Material material;
    };
    
    /*! Mesh. */
    struct TriangleMeshNode : public Node
    {
      struct Triangle 
      {
      public:
        Triangle (int v0, int v1, int v2) 
        : v0(v0), v1(v1), v2(v2) {}
      public:
        int v0, v1, v2;
      };
      
    public:
      TriangleMeshNode (Ref<MaterialNode> material) 
        : material(material) {}
      
    public:
      avector<Vec3fa> v;
      avector<Vec3fa> v2;
      avector<Vec3fa> vn;
      std::vector<Vec2f> vt;
      std::vector<Triangle> triangles;
      Ref<MaterialNode> material;
    };
    
    /*! Subdivision Mesh. */
    struct SubdivMeshNode : public Node
    {
      SubdivMeshNode (Ref<MaterialNode> material) 
        : material(material) {}
      
      avector<Vec3fa> positions;            //!< vertex positions
      avector<Vec3fa> normals;              //!< face vertex normals
      std::vector<Vec2f> texcoords;             //!< face texture coordinates
      std::vector<int> position_indices;        //!< position indices for all faces
      std::vector<int> normal_indices;          //!< normal indices for all faces
      std::vector<int> texcoord_indices;        //!< texcoord indices for all faces
      std::vector<int> verticesPerFace;         //!< number of indices of each face
      std::vector<int> holes;                   //!< face ID of holes
      std::vector<Vec2i> edge_creases;          //!< index pairs for edge crease 
      std::vector<float> edge_crease_weights;   //!< weight for each edge crease
      std::vector<int> vertex_creases;          //!< indices of vertex creases
      std::vector<float> vertex_crease_weights; //!< weight for each vertex crease
      Ref<MaterialNode> material;
    };
    
    /*! Hair Set. */
    struct HairSetNode : public Node
    {
      struct Hair
      {
      public:
        Hair () {}
        Hair (int vertex, int id)
        : vertex(vertex), id(id) {}
      public:
        int vertex,id;  //!< index of first control point and hair ID
      };
      
    public:
      HairSetNode (Ref<MaterialNode> material)
      : material(material) {}
      
    public:
      avector<Vec3fa> v;        //!< hair control points (x,y,z,r)
      avector<Vec3fa> v2;       //!< hair control points (x,y,z,r)
      std::vector<Hair> hairs;  //!< list of hairs
      Ref<MaterialNode> material;
    };
  };
}
