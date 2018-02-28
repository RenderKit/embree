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

#include "xml_writer.h"

namespace embree
{
  class XMLWriter
  {
  public:

    XMLWriter(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures, bool referenceMaterials);

  public:
    void tab();
    void open(std::string str);
    void open(std::string str, size_t id);
    void close(std::string str);
    
    void store(const char* name, const char* str);
    void store(const char* name, const float& v);
    void store(const char* name, const Vec3fa& v);
    template<typename T> void store(const char* name, const std::vector<T>& vec);
    void store(const char* name, const avector<Vec3fa>& vec);
    void store4f(const char* name, const avector<Vec3fa>& vec);
    void store_parm(const char* name, const float& v);
    void store_parm(const char* name, const Vec3fa& v);
    void store_parm(const char* name, const std::shared_ptr<Texture> tex);
    void store(const char* name, const AffineSpace3fa& space);

    void store(const SceneGraph::PointLight& light, ssize_t id);
    void store(const SceneGraph::SpotLight& light, ssize_t id);
    void store(const SceneGraph::DirectionalLight& light, ssize_t id);
    void store(const SceneGraph::DistantLight& light, ssize_t id);
    void store(const SceneGraph::AmbientLight& light, ssize_t id);
    void store(const SceneGraph::TriangleLight& light, ssize_t id);
    void store(const SceneGraph::QuadLight& light, ssize_t id);
    void store(Ref<SceneGraph::LightNode> light, ssize_t id);
    
    void store(Ref<MatteMaterial> material, ssize_t id);
    void store(Ref<MirrorMaterial> material, ssize_t id);
    void store(Ref<ThinDielectricMaterial> material, ssize_t id);
    void store(Ref<OBJMaterial> material, ssize_t id);
    void store(Ref<MetalMaterial> material, ssize_t id);
    void store(Ref<VelvetMaterial> material, ssize_t id);
    void store(Ref<DielectricMaterial> material, ssize_t id);
    void store(Ref<MetallicPaintMaterial> material, ssize_t id);
    void store(Ref<HairMaterial> material, ssize_t id);
    void store(Ref<SceneGraph::MaterialNode> material);

    void store(Ref<SceneGraph::TriangleMeshNode> mesh, ssize_t id);
    void store(Ref<SceneGraph::QuadMeshNode> mesh, ssize_t id);
    void store(Ref<SceneGraph::SubdivMeshNode> mesh, ssize_t id);
    void store(Ref<SceneGraph::HairSetNode> hair, ssize_t id);

    void store(Ref<SceneGraph::PerspectiveCameraNode> camera, ssize_t id);
    void store(Ref<SceneGraph::TransformNode> node, ssize_t id);
    void store(std::vector<Ref<SceneGraph::TransformNode>> nodes);
    void store(Ref<SceneGraph::GroupNode> group, ssize_t id);
    void store(Ref<SceneGraph::Node> node);

  private:
    std::fstream xml;         //!< .xml file for writing XML data
    std::fstream bin;         //!< .bin file for writing binary data

  private:
    size_t ident;
    size_t currentNodeID;
    std::map<Ref<SceneGraph::Node>, size_t> nodeMap;   
    std::map<std::shared_ptr<Texture>, size_t> textureMap; // FIXME: use Ref<Texture>
    bool embedTextures;
    bool referenceMaterials;
  };

  //////////////////////////////////////////////////////////////////////////////
  //// Storing of objects to XML file
  //////////////////////////////////////////////////////////////////////////////

  void XMLWriter::tab()
  {
    for (size_t i=0; i<ident; i++) 
      xml << " ";
  }

  void XMLWriter::open(std::string str)
  {
    tab(); xml << "<" << str << ">" << std::endl;
    ident+=2;
  }

  void XMLWriter::open(std::string str, size_t id)
  {
    tab(); xml << "<" << str << " id=\"" << id << "\">" << std::endl;
    ident+=2;
  }

  void XMLWriter::close(std::string str)
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

  void XMLWriter::store_parm(const char* name, const std::shared_ptr<Texture> tex) 
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

  void XMLWriter::store(Ref<MatteMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","Matte");
    open("parameters");
    store_parm("reflectance",material->reflectance);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<MirrorMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","Mirror");
    open("parameters");
    store_parm("reflectance",material->reflectance);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<ThinDielectricMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","ThinDielectric");
    open("parameters");
    store_parm("transmission",material->transmission);
    store_parm("eta",material->eta);
    store_parm("thickness",material->thickness);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<OBJMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","OBJ");
    open("parameters");
    store_parm("d",material->d);
    store_parm("Kd",material->Kd);
    store_parm("Ks",material->Ks);
    store_parm("Ns",material->Ns);
    store_parm("map_d",material->_map_d);
    store_parm("map_Kd",material->_map_Kd);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<MetalMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","Metal");
    open("parameters");
    store_parm("reflectance",material->reflectance);
    store_parm("eta",material->eta);
    store_parm("k",material->k);
    store_parm("roughness",material->roughness);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<VelvetMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","Velvet");
    open("parameters");
    store_parm("reflectance",material->reflectance);
    store_parm("backScattering",material->backScattering);
    store_parm("horizonScatteringColor",material->horizonScatteringColor);
    store_parm("horizonScatteringFallOff",material->horizonScatteringFallOff);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<DielectricMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","Dielectric");
    open("parameters");
    store_parm("transmissionOutside",material->transmissionOutside);
    store_parm("transmission",material->transmissionInside);
    store_parm("etaOutside",material->etaOutside);
    store_parm("etaInside",material->etaInside);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<MetallicPaintMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","MetallicPaint");
    open("parameters");
    store_parm("shadeColor",material->shadeColor);
    store_parm("glitterColor",material->glitterColor);
    store_parm("glitterSpread",material->glitterSpread);
    store_parm("eta",material->eta);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<HairMaterial> material, ssize_t id)
  {
    open("material",id);
    store("code","Hair");
    open("parameters");
    store_parm("Kr",material->Kr);
    store_parm("Kt",material->Kt);
    store_parm("nx",material->nx);
    store_parm("ny",material->ny);
    close("parameters");
    close("material");
  }

  void XMLWriter::store(Ref<SceneGraph::MaterialNode> mnode)
  {
    /* let materials reference by their name, allows separate bindings of materials */
    if (referenceMaterials) {
      tab(); xml << "<material id=\""+mnode->name+"\"/>" << std::endl;
      return;
    }
    
    Ref<SceneGraph::Node> node = mnode.dynamicCast<SceneGraph::Node>();
    if (nodeMap.find(node) != nodeMap.end()) {
      tab(); xml << "<material id=\"" << nodeMap[node] << "\"/>" << std::endl;
      return;
    }
    const ssize_t id = nodeMap[node] = currentNodeID++;

    if      (Ref<OBJMaterial> m = mnode.dynamicCast<OBJMaterial>()) store(m,id);
    else if (Ref<ThinDielectricMaterial> m = mnode.dynamicCast<ThinDielectricMaterial>()) store(m,id);
    else if (Ref<MetalMaterial> m = mnode.dynamicCast<MetalMaterial>()) store(m,id);
    else if (Ref<VelvetMaterial> m = mnode.dynamicCast<VelvetMaterial>()) store(m,id);
    else if (Ref<DielectricMaterial> m = mnode.dynamicCast<DielectricMaterial>()) store(m,id);
    else if (Ref<MetallicPaintMaterial> m = mnode.dynamicCast<MetallicPaintMaterial>()) store(m,id);
    else if (Ref<MatteMaterial> m = mnode.dynamicCast<MatteMaterial>()) store(m,id);
    else if (Ref<MirrorMaterial> m = mnode.dynamicCast<MirrorMaterial>()) store(m,id);
    else if (Ref<ReflectiveMetalMaterial> m = mnode.dynamicCast<ReflectiveMetalMaterial>()) store(m,id);
    else if (Ref<HairMaterial> m = mnode.dynamicCast<HairMaterial>()) store(m,id);
    else throw std::runtime_error("unsupported material");
  }

  void XMLWriter::store(Ref<SceneGraph::TriangleMeshNode> mesh, ssize_t id) 
  {
    open("TriangleMesh",id);
    store(mesh->material);
    
    if (mesh->numTimeSteps() != 1) open("animated_positions");
    for (const auto& p : mesh->positions) store("positions",p);
    if (mesh->numTimeSteps() != 1) close("animated_positions");

    if (mesh->numTimeSteps() != 1) open("animated_normals");
    for (const auto& p : mesh->normals) store("normals",p);
    if (mesh->numTimeSteps() != 1) close("animated_normals");
    
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

    if (mesh->numTimeSteps() != 1) open("animated_normals");
    for (const auto& p : mesh->normals) store("normals",p);
    if (mesh->numTimeSteps() != 1) close("animated_normals");
    
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

    if (mesh->numTimeSteps() != 1) open("animated_normals");
    for (const auto& p : mesh->normals) store("normals",p);
    if (mesh->numTimeSteps() != 1) close("animated_normals");

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

  void XMLWriter::store(Ref<SceneGraph::HairSetNode> mesh, ssize_t id)
  {
    std::string str_type = "";
    std::string str_subtype = "";

    switch (mesh->type) {
    case RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE:
      str_type = "linear";
      str_subtype = "flat";
      break;

    case RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE:
      str_type = "bezier";
      str_subtype = "round";
      break;

    case RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE:
      str_type = "bezier";
      str_subtype = "flat";
      break;

    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE:
      str_type = "bezier";
      str_subtype = "oriented";
      break;

    case RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE:
      str_type = "bspline";
      str_subtype = "round";
      break;

    case RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE:
      str_type = "bspline";
      str_subtype = "flat";
      break;

    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE:
      str_type = "bspline";
      str_subtype = "oriented";
      break;

    default:
      throw std::runtime_error("invalid curve type");
    }

    std::vector<int> indices(mesh->hairs.size());
    std::vector<int> hairid(mesh->hairs.size());
    for (size_t i=0; i<mesh->hairs.size(); i++) {
      indices[i] = mesh->hairs[i].vertex;
      hairid[i] = mesh->hairs[i].id;
    }
    
    open("Curves type=\""+str_subtype+"\" basis=\""+str_type+"\"",id);
    store(mesh->material);
    if (mesh->numTimeSteps() != 1) open("animated_positions");
    for (const auto& p : mesh->positions) store4f("positions",p);
    if (mesh->numTimeSteps() != 1) close("animated_positions");
    if (mesh->normals.size()) {
      if (mesh->numTimeSteps() != 1) open("animated_normals");
      for (const auto& p : mesh->normals) store4f("normals",p);
      if (mesh->numTimeSteps() != 1) close("animated_normals");
    }
    store("indices",indices);
    store("hairid",hairid);
    close("Curves");
  }

  void XMLWriter::store(Ref<SceneGraph::PerspectiveCameraNode> camera, ssize_t id)
  {
    tab(); 
    xml << "<PerspectiveCamera " <<
      "id=\"" << id << "\" " << 
      "name=\"" << camera->name << "\" " <<
      "from=\"" << camera->from.x << " " << camera->from.y << " " << camera->from.z << "\" " <<
      "to=\"" << camera->to.x << " " << camera->to.y << " " << camera->to.z << "\" " <<
      "up=\"" << camera->up.x << " " << camera->up.y << " " << camera->up.z << "\" " <<
      "fov=\"" << camera->fov << "\" " << "/>" << std::endl;
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

  void XMLWriter::store(std::vector<Ref<SceneGraph::TransformNode>> nodes)
  {
    if (nodes.size() == 0)
      return;

    if (nodes.size() == 1) {
      store(nodes[0].dynamicCast<SceneGraph::Node>());
      return;
    }
    
    open("MultiTransform");
    std::streampos offset = bin.tellg();
    tab(); xml << "<AffineSpace3f ofs=\"" << offset << "\" size=\"" << nodes.size() << "\"/>" << std::endl;
    for (size_t i=0; i<nodes.size(); i++) {
      assert(nodes[i]->spaces.size() == 1);
      assert(nodes[i]->child == nodes[0]->child);
      bin.write((char*)&nodes[i]->spaces[0].l.vx,sizeof(Vec3f));
      bin.write((char*)&nodes[i]->spaces[0].l.vy,sizeof(Vec3f));
      bin.write((char*)&nodes[i]->spaces[0].l.vz,sizeof(Vec3f));
      bin.write((char*)&nodes[i]->spaces[0].p,sizeof(Vec3f));
    }
    store(nodes[0]->child);
    close("MultiTransform");
  }

  void XMLWriter::store(Ref<SceneGraph::GroupNode> group, ssize_t id)
  {
    open("Group",id);

    std::map<Ref<SceneGraph::Node>,std::vector<Ref<SceneGraph::TransformNode>>> object_to_transform_map;
    for (size_t i=0; i<group->children.size(); i++)
    {
      /* compress transformation of the same object into MultiTransform nodes if possible */
      if (Ref<SceneGraph::TransformNode> xfmNode = group->children[i].dynamicCast<SceneGraph::TransformNode>())
      {
        /* we can only compress non-animated transform nodes and only ones that are referenced once */
        if (xfmNode->spaces.size() == 1 && xfmNode->indegree == 1) 
        {
          Ref<SceneGraph::Node> child = xfmNode->child;
          if (object_to_transform_map.find(child) == object_to_transform_map.end()) {
            object_to_transform_map[child] = std::vector<Ref<SceneGraph::TransformNode>>();
          }
          object_to_transform_map[child].push_back(xfmNode);
          continue;
        }
      }
      store(group->children[i]);
    }
    
    /* store all compressed transform nodes */
    for (auto i : object_to_transform_map)
      store(i.second);

    close("Group");
  }

  void XMLWriter::store(Ref<SceneGraph::Node> node)
  {
    if (nodeMap.find(node) != nodeMap.end()) {
      tab(); xml << "<ref id=\"" << nodeMap[node] << "\"/>" << std::endl; return;
    }
    
    const ssize_t id = nodeMap[node] = currentNodeID++;
    if (node->fileName != "") {
      tab(); xml << "<extern id=\"" << id << "\" src=\"" << node->fileName << "\"/>" << std::endl; return;
    } 

    if      (Ref<SceneGraph::LightNode> cnode = node.dynamicCast<SceneGraph::LightNode>()) store(cnode,id);
    //else if (Ref<SceneGraph::MaterialNode> cnode = node.dynamicCast<SceneGraph::MaterialNode>()) store(cnode,id);
    else if (Ref<SceneGraph::TriangleMeshNode> cnode = node.dynamicCast<SceneGraph::TriangleMeshNode>()) store(cnode,id);
    else if (Ref<SceneGraph::QuadMeshNode> cnode = node.dynamicCast<SceneGraph::QuadMeshNode>()) store(cnode,id);
    else if (Ref<SceneGraph::SubdivMeshNode> cnode = node.dynamicCast<SceneGraph::SubdivMeshNode>()) store(cnode,id);
    else if (Ref<SceneGraph::HairSetNode> cnode = node.dynamicCast<SceneGraph::HairSetNode>()) store(cnode,id);
    else if (Ref<SceneGraph::PerspectiveCameraNode> cnode = node.dynamicCast<SceneGraph::PerspectiveCameraNode>()) store(cnode,id);
    else if (Ref<SceneGraph::TransformNode> cnode = node.dynamicCast<SceneGraph::TransformNode>()) store(cnode,id);
    else if (Ref<SceneGraph::GroupNode> cnode = node.dynamicCast<SceneGraph::GroupNode>()) store(cnode,id);
    else throw std::runtime_error("unknown node type");
  }
 
  XMLWriter::XMLWriter(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures, bool referenceMaterials) 
    : ident(0), currentNodeID(0), embedTextures(embedTextures), referenceMaterials(referenceMaterials)
  {
    FileName binFileName = fileName.addExt(".bin");

    xml.exceptions (std::fstream::failbit | std::fstream::badbit);
    xml.open (fileName, std::fstream::out);
    bin.exceptions (std::fstream::failbit | std::fstream::badbit);
    bin.open (binFileName, std::fstream::out | std::fstream::binary);

    xml << "<?xml version=\"1.0\"?>" << std::endl;
    root->calculateInDegree();
    open("scene");
    store(root);
    close("scene");
    root->resetInDegree();
  }

  void SceneGraph::storeXML(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures, bool referenceMaterials) {
    XMLWriter(root,fileName,embedTextures,referenceMaterials);
  }
}
