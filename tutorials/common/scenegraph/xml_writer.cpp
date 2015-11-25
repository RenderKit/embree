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

#include "xml_writer.h"

namespace embree
{
  class XMLWriter
  {
  public:

    XMLWriter(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures);
   ~XMLWriter();

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
    void store(Ref<SceneGraph::LightNode<PointLight>> light, ssize_t id);
    void store(Ref<SceneGraph::LightNode<SpotLight>> light, ssize_t id);
    void store(Ref<SceneGraph::LightNode<DirectionalLight>> light, ssize_t id);
    void store(Ref<SceneGraph::LightNode<DistantLight>> light, ssize_t id);
    void store(Ref<SceneGraph::LightNode<AmbientLight>> light, ssize_t id);
    void store(Ref<SceneGraph::LightNode<TriangleLight>> light, ssize_t id);
    void store(Ref<SceneGraph::LightNode<QuadLight>> light, ssize_t id);
    
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
    FILE* xml;         //!< .xml file for writing XML data
    FILE* bin;         //!< .bin file for writing binary data

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
      fprintf(xml," ");
  }

  void XMLWriter::open(const char* str)
  {
    tab(); fprintf(xml,"<%s>\n",str);
    ident+=2;
  }

  void XMLWriter::open(const char* str, size_t id)
  {
    tab(); fprintf(xml,"<%s id=\"%zu\">\n",str,id);
    ident+=2;
  }

  void XMLWriter::close(const char* str)
  {
    assert(ident>=2);
    ident-=2;
    tab(); fprintf(xml,"</%s>\n",str);
  }

  void XMLWriter::store(const char* name, const char* str) {
    tab(); fprintf(xml,"<%s>\"%s\"</%s>\n",name,str,name);
  }

  void XMLWriter::store(const char* name, const float& v) {
    tab(); fprintf(xml,"<%s>%f</%s>\n",name,v,name);
  }

  void XMLWriter::store(const char* name, const Vec3fa& v) {
    tab(); fprintf(xml,"<%s>%f %f %f</%s>\n",name,v.x,v.y,v.z,name);
  }

  template<typename T>
  void XMLWriter::store(const char* name, const std::vector<T>& vec)
  {
    const long int offset = ftell(bin);
    tab(); fprintf(xml, "<%s ofs=\"%li\" size=\"%zu\"/>\n", name, offset, vec.size());
    if (vec.size()) fwrite(vec.data(),vec.size(),sizeof(T),bin);
  }

  void XMLWriter::store(const char* name, const avector<Vec3fa>& vec)
  {
    const long int offset = ftell(bin);
    tab(); fprintf(xml, "<%s ofs=\"%ld\" size=\"%zu\"/>\n", name, offset, vec.size());
    for (size_t i=0; i<vec.size(); i++) fwrite(&vec[i],1,sizeof(Vec3f),bin);
  }

  void XMLWriter::store4f(const char* name, const avector<Vec3fa>& vec)
  {
    const long int offset = ftell(bin);
    tab(); fprintf(xml, "<%s ofs=\"%ld\" size=\"%zu\"/>\n", name, offset, vec.size());
    for (size_t i=0; i<vec.size(); i++) fwrite(&vec[i],1,sizeof(Vec3fa),bin);
  }

  void XMLWriter::store_parm(const char* name, const float& v) {
    tab(); fprintf(xml,"<float name=\"%s\">%f</float>\n",name,v);
  }

  void XMLWriter::store_parm(const char* name, const Vec3fa& v) {
    tab(); fprintf(xml,"<float3 name=\"%s\">%f %f %f</float3>\n",name,v.x,v.y,v.z);
  }

  void XMLWriter::store_parm(const char* name, const Texture* tex) 
  {
    if (tex == nullptr) return;

    if (textureMap.find(tex) != textureMap.end()) {
      tab(); fprintf(xml,"<texture3d name=\"%s\" id=\"%zu\"/>\n",name,textureMap[tex]);
    } else if (embedTextures) {
      const long int offset = ftell(bin);
      fwrite(tex->data,tex->width*tex->height,tex->bytesPerTexel,bin);
      const size_t id = textureMap[tex] = currentNodeID++;
      tab(); fprintf(xml,"<texture3d name=\"%s\" id=\"%zu\" ofs=\"%ld\" width=\"%i\" height=\"%i\" format=\"%s\"/>\n",
                     name,id,offset,tex->width,tex->height,Texture::format_to_string(tex->format));
    }
    else {
      const size_t id = textureMap[tex] = currentNodeID++;
      tab(); fprintf(xml,"<texture3d name=\"%s\" id=\"%zu\" src=\"%s\"/>\n",name,textureMap[tex],tex->fileName.c_str());
    }
  }

  void XMLWriter::store(const char* name, const AffineSpace3fa& space)
  {
    tab(); fprintf(xml,"<%s>\n",name);
    tab(); fprintf(xml,"  %f %f %f %f\n",space.l.vx.x,space.l.vy.x,space.l.vz.x,space.p.x);
    tab(); fprintf(xml,"  %f %f %f %f\n",space.l.vx.y,space.l.vy.y,space.l.vz.y,space.p.y);
    tab(); fprintf(xml,"  %f %f %f %f\n",space.l.vx.z,space.l.vy.z,space.l.vz.z,space.p.z);
    tab(); fprintf(xml,"</%s>\n",name);
  }
                  
  void XMLWriter::store(Ref<SceneGraph::LightNode<PointLight>> node, ssize_t id) 
  {
    open("PointLight",id);
    store("AffineSpace",AffineSpace3fa::translate(node->light.P));
    store("I",node->light.I);
    close("PointLight");
  }

  void XMLWriter::store(Ref<SceneGraph::LightNode<SpotLight>> node, ssize_t id) 
  {
    open("SpotLight",id);
    store("AffineSpace",AffineSpace3fa(frame(node->light.D),node->light.P));
    store("I",node->light.I);
    store("angleMin",node->light.angleMin);
    store("angleMax",node->light.angleMax);
    close("SpotLight");
  }

  void XMLWriter::store(Ref<SceneGraph::LightNode<DirectionalLight>> node, ssize_t id) 
  {
    open("DirectionalLight",id);
    store("AffineSpace",frame(node->light.D));
    store("E",node->light.E);
    close("DirectionalLight");
  }

  void XMLWriter::store(Ref<SceneGraph::LightNode<DistantLight>> node, ssize_t id) 
  {
    open("DistantLight",id);
    store("AffineSpace",frame(node->light.D));
    store("L",node->light.L);
    store("halfAngle",node->light.halfAngle);
    close("DistantLight");
  }

  void XMLWriter::store(Ref<SceneGraph::LightNode<AmbientLight>> node, ssize_t id) 
  {
    open("AmbientLight");
    store("L",node->light.L);
    close("AmbientLight");
  }

  void XMLWriter::store(Ref<SceneGraph::LightNode<TriangleLight>> node, ssize_t id) 
  {
    open("TriangleLight",id);
    const Vec3fa dx = node->light.v0-node->light.v2;
    const Vec3fa dy = node->light.v1-node->light.v2;
    const Vec3fa dz = cross(dx,dy);
    const Vec3fa p = node->light.v2;
    store("AffineSpace",AffineSpace3fa(dx,dy,dy,p));
    store("L",node->light.L);
    close("TriangleLight");
  }

  void XMLWriter::store(Ref<SceneGraph::LightNode<QuadLight>> node, ssize_t id) 
  {
    open("QuadLight",id);
    const Vec3fa dx = node->light.v3-node->light.v0;
    const Vec3fa dy = node->light.v1-node->light.v0;
    const Vec3fa dz = cross(dx,dy);
    const Vec3fa p = node->light.v2;
    store("AffineSpace",AffineSpace3fa(dx,dy,dy,p));
    store("L",node->light.L);
    close("QuadLight");
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
      tab(); fprintf(xml,"<material id=\"%zu\"/>\n",nodeMap[node]);
      return;
    }
    const ssize_t id = nodeMap[node] = currentNodeID++;

    switch (mnode->material.ty)
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
    store("positions",mesh->v);
    if (mesh->v2.size()) store("positions2",mesh->v2);
    store("normals",mesh->vn);
    store("texcoords",mesh->vt);
    store("triangles",mesh->triangles);
    close("TriangleMesh");
  }

  void XMLWriter::store(Ref<SceneGraph::QuadMeshNode> mesh, ssize_t id) 
  {
    open("QuadMesh",id);
    store(mesh->material);
    store("positions",mesh->v);
    if (mesh->v2.size()) store("positions2",mesh->v2);
    store("normals",mesh->vn);
    store("texcoords",mesh->vt);
    store("indices",mesh->quads);
    close("QuadMesh");
  }

  void XMLWriter::store(Ref<SceneGraph::SubdivMeshNode> mesh, ssize_t id)
  {
    open("SubdivisionMesh",id);
    store(mesh->material);
    store("positions",mesh->positions);
    if (mesh->positions2.size()) store("positions2",mesh->positions2);
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
    store4f("positions",mesh->v);
    if (mesh->v2.size()) store4f("positions2",mesh->v2);
    store("indices",mesh->indices);
    close("LineSegments");
  }

  void XMLWriter::store(Ref<SceneGraph::HairSetNode> hair, ssize_t id)
  {
    open("Hair",id);
    store(hair->material);
    store4f("positions",hair->v);
    if (hair->v2.size()) store4f("positions2",hair->v2);
    store("indices",hair->hairs);
    close("Hair");
  }

  void XMLWriter::store(Ref<SceneGraph::TransformNode> node, ssize_t id)
  {
    if (node->xfm0 == node->xfm1)
    {
      open("Transform",id);
      store("AffineSpace",node->xfm0);
      store(node->child);
      close("Transform");
    }
    else
    {
      open("Transform2",id);
      store("AffineSpace",node->xfm0);
      store("AffineSpace",node->xfm1);
      store(node->child);
      close("Transform2");
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
      tab(); fprintf(xml,"<ref id=\"%zu\"/>\n",nodeMap[node]); return;
    }
    const ssize_t id = nodeMap[node] = currentNodeID++;

    if      (Ref<SceneGraph::LightNode<PointLight>> cnode = node.dynamicCast<SceneGraph::LightNode<PointLight>>()) store(cnode,id);
    else if (Ref<SceneGraph::LightNode<SpotLight>> cnode = node.dynamicCast<SceneGraph::LightNode<SpotLight>>()) store(cnode,id);
    else if (Ref<SceneGraph::LightNode<DirectionalLight>> cnode = node.dynamicCast<SceneGraph::LightNode<DirectionalLight>>()) store(cnode,id);
    else if (Ref<SceneGraph::LightNode<DistantLight>> cnode = node.dynamicCast<SceneGraph::LightNode<DistantLight>>()) store(cnode,id);
    else if (Ref<SceneGraph::LightNode<AmbientLight>> cnode = node.dynamicCast<SceneGraph::LightNode<AmbientLight>>()) store(cnode,id);
    else if (Ref<SceneGraph::LightNode<TriangleLight>> cnode = node.dynamicCast<SceneGraph::LightNode<TriangleLight>>()) store(cnode,id);
    else if (Ref<SceneGraph::LightNode<QuadLight>> cnode = node.dynamicCast<SceneGraph::LightNode<QuadLight>>()) store(cnode,id);
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
    : xml(nullptr), bin(nullptr), ident(0), currentNodeID(0), embedTextures(embedTextures)
  {
    FileName binFileName = fileName.addExt(".bin");
    xml = fopen(fileName.c_str(),"w");
    bin = fopen(binFileName.c_str(),"wb");

    fprintf(xml,"<?xml version=\"1.0\"?>\n");
    open("scene");
    store(root);
    close("scene");
  }

  XMLWriter::~XMLWriter() {
    if (xml) fclose(xml);
    if (bin) fclose(bin);
  }

  void storeXML(Ref<SceneGraph::Node> root, const FileName& fileName, bool embedTextures) {
    XMLWriter(root,fileName,embedTextures);
  }
}
