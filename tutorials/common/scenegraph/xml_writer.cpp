// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "xml_writer.h"

namespace embree
{
  class XMLWriter
  {
  public:

    XMLWriter(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures);

  public:
    void tab();
    void open(const char* str);
    void open(const char* str, size_t id);
    void close(const char* str);
    
    void store(const char* name, const char* str);
    void store(const char* name, const float& v);
    void store(const char* name, const Vec3fa& v);
    template<typename T> void store(const char* name, const std::vector<T>& vec);
    void store(const char* name, const avector<Vec3fa>& vec);
    void store4f(const char* name, const avector<Vec3fa>& vec);
    void store_parm(const char* name, const float& v);
    void store_parm(const char* name, const Vec3fa& v);
    void store_parm(const char* name, const Texture* tex);
    void store(const char* name, const AffineSpace3fa& space);

    void store(const SceneGraph::PointLight& light, ssize_t id);
    void store(const SceneGraph::SpotLight& light, ssize_t id);
    void store(const SceneGraph::DirectionalLight& light, ssize_t id);
    void store(const SceneGraph::DistantLight& light, ssize_t id);
    void store(const SceneGraph::AmbientLight& light, ssize_t id);
    void store(const SceneGraph::TriangleLight& light, ssize_t id);
    void store(const SceneGraph::QuadLight& light, ssize_t id);
    void store(Ref<SceneGraph::LightNode> light, ssize_t id);
    
    void store(const MatteMaterial& material, ssize_t id);
    void store(const MirrorMaterial& material, ssize_t id);
    void store(const ThinDielectricMaterial& material, ssize_t id);
    void store(const OBJMaterial& material, ssize_t id);
    void store(const MetalMaterial& material, ssize_t id);
    void store(const VelvetMaterial& material, ssize_t id);
    void store(const DielectricMaterial& material, ssize_t id);
    void store(const MetallicPaintMaterial& material, ssize_t id);
    void store(const HairMaterial& material, ssize_t id);
    void store(Ref<SceneGraph::MaterialNode> material);

    void store(Ref<SceneGraph::TriangleMeshNode> mesh, ssize_t id);
    void store(Ref<SceneGraph::QuadMeshNode> mesh, ssize_t id);
    void store(Ref<SceneGraph::SubdivMeshNode> mesh, ssize_t id);
    void store(Ref<SceneGraph::LineSegmentsNode> mesh, ssize_t id);
    void store(Ref<SceneGraph::HairSetNode> hair, ssize_t id);

    void store(Ref<SceneGraph::TransformNode> node, ssize_t id);
    void store(Ref<SceneGraph::GroupNode> group, ssize_t id);
    void store(Ref<SceneGraph::Node> node);

  private:
    std::fstream xml;         //!< .xml file for writing XML data
    std::fstream bin;         //!< .bin file for writing binary data

  private:
    size_t ident;
    size_t currentNodeID;
    std::map<Ref<SceneGraph::Node>, size_t> nodeMap;   
    std::map<const Texture*, size_t> textureMap; // FIXME: use Ref<Texture>
    bool embedTextures;
  };

  //////////////////////////////////////////////////////////////////////////////
  //// Storing of objects to XML file
  //////////////////////////////////////////////////////////////////////////////

  void XMLWriter::tab()
  {
    for (size_t i=0; i<ident; i++) 
      xml << " ";
  }

  void XMLWriter::open(const char* str)
  {
    tab(); xml << "<" << str << ">" << std::endl;
    ident+=2;
  }

  void XMLWriter::open(const char* str, size_t id)
  {
    tab(); xml << "<" << str << " id=\"" << id << "\">" << std::endl;
    ident+=2;
  }

  void XMLWriter::close(const char* str)
  {
    assert(ident>=2);
    ident-=2;
    tab(); xml << "</" << str << ">" << std::endl;
  }

  void XMLWriter::store(const char* name, const char* str) {
    tab(); xml << "<" << name << ">\"" << str << "\"</" << name << ">" << std::endl;
  }

  void XMLWriter::store(const char* name, const float& v) {
    tab(); xml << "<" << name << ">" << v << "</" << name << ">" << std::endl;
  }

  void XMLWriter::store(const char* name, const Vec3fa& v) {
    tab(); xml << "<" << name << ">" << v.x << " " << v.y << " " << v.z << "</" << name << ">" << std::endl;
  }

  template<typename T>
  void XMLWriter::store(const char* name, const std::vector<T>& vec)
  {
    std::streampos offset = bin.tellg();
    tab(); xml << "<" << name << " ofs=\"" << offset << "\" size=\"" << vec.size() << "\"/>" << std::endl;
    if (vec.size()) bin.write((char*)vec.data(),vec.size()*sizeof(T));
  }

  void XMLWriter::store(const char* name, const avector<Vec3fa>& vec)
  {
    std::streampos offset = bin.tellg();
    tab(); xml << "<" << name << " ofs=\"" << offset << "\" size=\"" << vec.size() << "\"/>" << std::endl;
    for (size_t i=0; i<vec.size(); i++) bin.write((char*)&vec[i],sizeof(Vec3f));
  }

  void XMLWriter::store4f(const char* name, const avector<Vec3fa>& vec)
  {
    std::streampos offset = bin.tellg();
    tab(); xml << "<" << name << " ofs=\"" << offset << "\" size=\"" << vec.size() << "\"/>" << std::endl;
    for (size_t i=0; i<vec.size(); i++) bin.write((char*)&vec[i],sizeof(Vec3fa));
  }

  void XMLWriter::store_parm(const char* name, const float& v) {
    tab(); xml << "<float name=\"" << name << "\">" << v << "</float>" << std::endl;
  }

  void XMLWriter::store_parm(const char* name, const Vec3fa& v) {
    tab(); xml << "<float3 name=\"" << name << "\">" << v.x << " " << v.y << " " << v.z << "</float3>" << std::endl;
  }

  void XMLWriter::store_parm(const char* name, const Texture* tex) 
  {
    if (tex == nullptr) return;

    if (textureMap.find(tex) != textureMap.end()) {
      tab(); xml << "<texture3d name=\"" << name << "\" id=\"" << textureMap[tex] << "\"/>" << std::endl;
    } else if (embedTextures) {
      std::streampos offset = bin.tellg();
      bin.write((char*)tex->data,tex->width*tex->height*tex->bytesPerTexel);
      const size_t id = textureMap[tex] = currentNodeID++;
      tab(); xml << "<texture3d name=\"" << name << "\" id=\"" << id << "\" ofs=\"" << offset 
                 << "\" width=\"" << tex->width << "\" height=\"" << tex->height 
                 << "\" format=\"" << Texture::format_to_string(tex->format) << "\"/>" << std::endl;
    }
    else {
      const size_t id = textureMap[tex] = currentNodeID++;
      tab(); xml << "<texture3d name=\"" << name << "\" id=\"" << id << "\" src=\"" << tex->fileName << "\"/>" << std::endl;
    }
  }

  void XMLWriter::store(const char* name, const AffineSpace3fa& space)
  {
    tab(); xml << "<" << name << ">" << std::endl;
    tab(); xml << "  " << space.l.vx.x << " " << space.l.vy.x << " " << space.l.vz.x << " " << space.p.x << std::endl;
    tab(); xml << "  " << space.l.vx.y << " " << space.l.vy.y << " " << space.l.vz.y << " " << space.p.y << std::endl;
    tab(); xml << "  " << space.l.vx.z << " " << space.l.vy.z << " " << space.l.vz.z << " " << space.p.z << std::endl;
    tab(); xml << "</" << name << ">" << std::endl;
  }
                  
  void XMLWriter::store(const SceneGraph::PointLight& light, ssize_t id)
  {
    open("PointLight",id);
    store("AffineSpace",AffineSpace3fa::translate(light.P));
    store("I",light.I);
    close("PointLight");
  }

  void XMLWriter::store(const SceneGraph::SpotLight& light, ssize_t id)
  {
    open("SpotLight",id);
    store("AffineSpace",AffineSpace3fa(frame(light.D),light.P));
    store("I",light.I);
    store("angleMin",light.angleMin);
    store("angleMax",light.angleMax);
    close("SpotLight");
  }

  void XMLWriter::store(const SceneGraph::DirectionalLight& light, ssize_t id)
  {
    open("DirectionalLight",id);
    store("AffineSpace",frame(light.D));
    store("E",light.E);
    close("DirectionalLight");
  }

  void XMLWriter::store(const SceneGraph::DistantLight& light, ssize_t id)
  {
    open("DistantLight",id);
    store("AffineSpace",frame(light.D));
    store("L",light.L);
    store("halfAngle",light.halfAngle);
    close("DistantLight");
  }

  void XMLWriter::store(const SceneGraph::AmbientLight& light, ssize_t id)
  {
    open("AmbientLight");
    store("L",light.L);
    close("AmbientLight");
  }

  void XMLWriter::store(const SceneGraph::TriangleLight& light, ssize_t id)
  {
    open("TriangleLight",id);
    const Vec3fa dx = light.v0-light.v2;
    const Vec3fa dy = light.v1-light.v2;
    const Vec3fa dz = cross(dx,dy);
    const Vec3fa p = light.v2;
    store("AffineSpace",AffineSpace3fa(dx,dy,dz,p));
    store("L",light.L);
    close("TriangleLight");
  }

  void XMLWriter::store(const SceneGraph::QuadLight& light, ssize_t id)
  {
    open("QuadLight",id);
    const Vec3fa dx = light.v3-light.v0;
    const Vec3fa dy = light.v1-light.v0;
    const Vec3fa dz = cross(dx,dy);
    const Vec3fa p = light.v2;
    store("AffineSpace",AffineSpace3fa(dx,dy,dz,p));
    store("L",light.L);
    close("QuadLight");
  }

  void XMLWriter::store(Ref<SceneGraph::LightNode> node, ssize_t id)
  {
    switch (node->light->getType())
    {
    case SceneGraph::LIGHT_AMBIENT     : store(*node->light.dynamicCast<SceneGraph::AmbientLight>(),id); break;
    case SceneGraph::LIGHT_POINT       : store(*node->light.dynamicCast<SceneGraph::PointLight>(),id); break;
    case SceneGraph::LIGHT_DIRECTIONAL : store(*node->light.dynamicCast<SceneGraph::DirectionalLight>(),id); break;
    case SceneGraph::LIGHT_SPOT        : store(*node->light.dynamicCast<SceneGraph::SpotLight>(),id); break;
    case SceneGraph::LIGHT_DISTANT     : store(*node->light.dynamicCast<SceneGraph::DistantLight>(),id); break;
    case SceneGraph::LIGHT_TRIANGLE    : store(*node->light.dynamicCast<SceneGraph::TriangleLight>(),id); break;
    case SceneGraph::LIGHT_QUAD        : store(*node->light.dynamicCast<SceneGraph::QuadLight>(),id); break;

    default: throw std::runtime_error("unsupported light");
    }
  }

  void XMLWriter::store(const MatteMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","Matte");
    open("parameters");
    store_parm("reflectance",material.reflectance);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(const MirrorMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","Mirror");
    open("parameters");
    store_parm("reflectance",material.reflectance);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(const ThinDielectricMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","ThinDielectric");
    open("parameters");
    store_parm("transmission",material.transmission);
    store_parm("eta",material.eta);
    store_parm("thickness",material.thickness);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(const OBJMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","OBJ");
    open("parameters");
    store_parm("d",material.d);
    store_parm("Kd",material.Kd);
    store_parm("Ks",material.Ks);
    store_parm("Ns",material.Ns);
    store_parm("map_d",material.map_d);
    store_parm("map_Kd",material.map_Kd);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(const MetalMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","Metal");
    open("parameters");
    store_parm("reflectance",material.reflectance);
    store_parm("eta",material.eta);
    store_parm("k",material.k);
    store_parm("roughness",material.roughness);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(const VelvetMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","Velvet");
    open("parameters");
    store_parm("reflectance",material.reflectance);
    store_parm("backScattering",material.backScattering);
    store_parm("horizonScatteringColor",material.horizonScatteringColor);
    store_parm("horizonScatteringFallOff",material.horizonScatteringFallOff);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(const DielectricMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","Dielectric");
    open("parameters");
    store_parm("transmissionOutside",material.transmissionOutside);
    store_parm("transmission",material.transmissionInside);
    store_parm("etaOutside",material.etaOutside);
    store_parm("etaInside",material.etaInside);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(const MetallicPaintMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","MetallicPaint");
    open("parameters");
    store_parm("shadeColor",material.shadeColor);
    store_parm("glitterColor",material.glitterColor);
    store_parm("glitterSpread",material.glitterSpread);
    store_parm("eta",material.eta);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(const HairMaterial& material, ssize_t id)
  {
    open("material",id);
    store("code","Hair");
    open("parameters");
    store_parm("Kr",material.Kr);
    store_parm("Kt",material.Kt);
    store_parm("nx",material.nx);
    store_parm("ny",material.ny);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<SceneGraph::MaterialNode> mnode)
  {
    Ref<SceneGraph::Node> node = mnode.dynamicCast<SceneGraph::Node>();
    if (nodeMap.find(node) != nodeMap.end()) {
      tab(); xml << "<material id=\"" << nodeMap[node] << "\"/>" << std::endl;
      return;
    }
    const ssize_t id = nodeMap[node] = currentNodeID++;

    switch (mnode->material.type)
    {
    case MATERIAL_OBJ             : store((OBJMaterial&)mnode->material,id); break;
    case MATERIAL_THIN_DIELECTRIC : store((ThinDielectricMaterial&)mnode->material,id); break;
    case MATERIAL_METAL           : store((MetalMaterial&)mnode->material,id); break;
    case MATERIAL_VELVET          : store((VelvetMaterial&)mnode->material,id); break;
    case MATERIAL_DIELECTRIC      : store((DielectricMaterial&)mnode->material,id); break;
    case MATERIAL_METALLIC_PAINT  : store((MetallicPaintMaterial&)mnode->material,id); break;
    case MATERIAL_MATTE           : store((MatteMaterial&)mnode->material,id); break;
    case MATERIAL_MIRROR          : store((MirrorMaterial&)mnode->material,id); break;
    case MATERIAL_REFLECTIVE_METAL: store((ReflectiveMetalMaterial&)mnode->material,id); break;
    case MATERIAL_HAIR            : store((HairMaterial&)mnode->material,id); break;
    default: throw std::runtime_error("unsupported material");
    }
  }

  void XMLWriter::store(Ref<SceneGraph::TriangleMeshNode> mesh, ssize_t id) 
  {
    open("TriangleMesh",id);
    store(mesh->material);
    if (mesh->numTimeSteps() != 1) open("animated_positions");
    for (const auto& p : mesh->positions) store("positions",p);
    if (mesh->numTimeSteps() != 1) close("animated_positions");
    store("normals",mesh->normals);
    store("texcoords",mesh->texcoords);
    store("triangles",mesh->triangles);
    close("TriangleMesh");
  }

  void XMLWriter::store(Ref<SceneGraph::QuadMeshNode> mesh, ssize_t id) 
  {
    open("QuadMesh",id);
    store(mesh->material);
    if (mesh->numTimeSteps() != 1) open("animated_positions");
    for (const auto& p : mesh->positions) store("positions",p);
    if (mesh->numTimeSteps() != 1) close("animated_positions");
    store("normals",mesh->normals);
    store("texcoords",mesh->texcoords);
    store("indices",mesh->quads);
    close("QuadMesh");
  }

  void XMLWriter::store(Ref<SceneGraph::SubdivMeshNode> mesh, ssize_t id)
  {
    open("SubdivisionMesh",id);
    store(mesh->material);
    if (mesh->numTimeSteps() != 1) open("animated_positions");
    for (const auto& p : mesh->positions) store("positions",p);
    if (mesh->numTimeSteps() != 1) close("animated_positions");
    store("normals",mesh->normals);
    store("texcoords",mesh->texcoords);
    store("position_indices",mesh->position_indices);
    store("normal_indices",mesh->normal_indices);
    store("texcoord_indices",mesh->texcoord_indices);
    store("faces",mesh->verticesPerFace);
    store("holes",mesh->holes);
    store("edge_creases",mesh->edge_creases);
    store("edge_crease_weights",mesh->edge_crease_weights);
    store("vertex_creases",mesh->vertex_creases);
    store("vertex_crease_weights",mesh->vertex_crease_weights);
    close("SubdivisionMesh");
  }

  void XMLWriter::store(Ref<SceneGraph::LineSegmentsNode> mesh, ssize_t id)
  {
    open("LineSegments",id);
    store(mesh->material);
    if (mesh->numTimeSteps() != 1) open("animated_positions");
    for (const auto& p : mesh->positions) store4f("positions",p);
    if (mesh->numTimeSteps() != 1) close("animated_positions");
    store("indices",mesh->indices);
    close("LineSegments");
  }

  void XMLWriter::store(Ref<SceneGraph::HairSetNode> mesh, ssize_t id)
  {
    open("Hair",id);
    store(mesh->material);
    if (mesh->numTimeSteps() != 1) open("animated_positions");
    for (const auto& p : mesh->positions) store4f("positions",p);
    if (mesh->numTimeSteps() != 1) close("animated_positions");
    store("indices",mesh->hairs);
    close("Hair");
  }

  void XMLWriter::store(Ref<SceneGraph::TransformNode> node, ssize_t id)
  {
    if (node->spaces.size() == 1)
    {
      open("Transform",id);
      store("AffineSpace",node->spaces[0]);
      store(node->child);
      close("Transform");
    }
    else 
    {
      open("TransformAnimation",id);
      for (size_t i=0; i<node->spaces.size(); i++)
        store("AffineSpace",node->spaces[i]);
      store(node->child);
      close("TransformAnimation");
    }
  }

  void XMLWriter::store(Ref<SceneGraph::GroupNode> group, ssize_t id)
  {
    open("Group",id);
    for (size_t i=0; i<group->children.size(); i++)
      store(group->children[i]);
    close("Group");
  }

  void XMLWriter::store(Ref<SceneGraph::Node> node)
  {
    if (nodeMap.find(node) != nodeMap.end()) {
      tab(); xml << "<ref id=\"" << nodeMap[node] << "\"/>" << std::endl; return;
    }
    const ssize_t id = nodeMap[node] = currentNodeID++;

    if      (Ref<SceneGraph::LightNode> cnode = node.dynamicCast<SceneGraph::LightNode>()) store(cnode,id);
    //else if (Ref<SceneGraph::MaterialNode> cnode = node.dynamicCast<SceneGraph::MaterialNode>()) store(cnode,id);
    else if (Ref<SceneGraph::TriangleMeshNode> cnode = node.dynamicCast<SceneGraph::TriangleMeshNode>()) store(cnode,id);
    else if (Ref<SceneGraph::QuadMeshNode> cnode = node.dynamicCast<SceneGraph::QuadMeshNode>()) store(cnode,id);
    else if (Ref<SceneGraph::SubdivMeshNode> cnode = node.dynamicCast<SceneGraph::SubdivMeshNode>()) store(cnode,id);
    else if (Ref<SceneGraph::LineSegmentsNode> cnode = node.dynamicCast<SceneGraph::LineSegmentsNode>()) store(cnode,id);
    else if (Ref<SceneGraph::HairSetNode> cnode = node.dynamicCast<SceneGraph::HairSetNode>()) store(cnode,id);
    else if (Ref<SceneGraph::TransformNode> cnode = node.dynamicCast<SceneGraph::TransformNode>()) store(cnode,id);
    else if (Ref<SceneGraph::GroupNode> cnode = node.dynamicCast<SceneGraph::GroupNode>()) store(cnode,id);
    else throw std::runtime_error("unknown node type");
  }
 
  XMLWriter::XMLWriter(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures) 
    : ident(0), currentNodeID(0), embedTextures(embedTextures)
  {
    FileName binFileName = fileName.addExt(".bin");

    xml.exceptions (std::fstream::failbit | std::fstream::badbit);
    xml.open (fileName, std::fstream::out);
    bin.exceptions (std::fstream::failbit | std::fstream::badbit);
    bin.open (binFileName, std::fstream::out | std::fstream::binary);

    xml << "<?xml version=\"1.0\"?>" << std::endl;
    open("scene");
    store(root);
    close("scene");
  }

  void SceneGraph::storeXML(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures) {
    XMLWriter(root,fileName,embedTextures);
  }
}
