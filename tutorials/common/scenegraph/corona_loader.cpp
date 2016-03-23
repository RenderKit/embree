// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "corona_loader.h"
#include "xml_parser.h"
#include "obj_loader.h"

namespace embree
{
  class CoronaLoader
  {
  public:

    static Ref<SceneGraph::Node> load(const FileName& fileName, const AffineSpace3fa& space);
    CoronaLoader(const FileName& fileName, const AffineSpace3fa& space);

  private:
    template<typename T> T load(const Ref<XML>& xml) { assert(false); return T(zero); }
    Ref<SceneGraph::MaterialNode> loadMaterial(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadObject(const Ref<XML>& xml);
    avector<AffineSpace3fa> loadInstances(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadGroupNode(const Ref<XML>& xml);
    Ref<SceneGraph::Node> loadNode(const Ref<XML>& xml);

  private:
    FileName path; 
    std::map<std::string,Ref<SceneGraph::Node> > sceneMap; 
  public:
    Ref<SceneGraph::Node> root;
  };

  template<> FileName CoronaLoader::load<FileName>(const Ref<XML>& xml) 
  {
    if (xml->body.size() < 1) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong FileName body");
    return xml->body[0].Identifier();
  }

  template<> AffineSpace3fa CoronaLoader::load<AffineSpace3fa>(const Ref<XML>& xml) 
  {
    if (xml->body.size() != 12) THROW_RUNTIME_ERROR(xml->loc.str()+": wrong AffineSpace body");
    return AffineSpace3fa(LinearSpace3fa(xml->body[0].Float(),xml->body[1].Float(),xml->body[ 2].Float(),
                                         xml->body[4].Float(),xml->body[5].Float(),xml->body[ 6].Float(),
                                         xml->body[8].Float(),xml->body[9].Float(),xml->body[10].Float()),
                          Vec3fa(xml->body[3].Float(),xml->body[7].Float(),xml->body[11].Float()));
  }

  Ref<SceneGraph::Node> CoronaLoader::loadObject(const Ref<XML>& xml) 
  {
    if (xml->name != "object") 
      THROW_RUNTIME_ERROR(xml->loc.str()+": invalid object node");
    if (xml->parm("class") != "file")
      THROW_RUNTIME_ERROR(xml->loc.str()+": invalid object class");
    const FileName fileName = load<FileName>(xml);
    return SceneGraph::load(path+fileName);
  }

  avector<AffineSpace3fa> CoronaLoader::loadInstances(const Ref<XML>& xml) 
  {
    if (xml->name != "instance") 
      THROW_RUNTIME_ERROR(xml->loc.str()+": invalid instance node");

    avector<AffineSpace3fa> xfms;
    for (size_t i=0; i<xml->children.size(); i++)
    {
      Ref<XML> child = xml->children[i];
      if      (child->name == "material" ) continue;
      else if (child->name == "transform") xfms.push_back(load<AffineSpace3fa>(child));
      else THROW_RUNTIME_ERROR(child->loc.str()+": unknown node: "+child->name);
    }
    return xfms;
  }

  Ref<SceneGraph::Node> CoronaLoader::loadGroupNode(const Ref<XML>& xml) 
  {
    if (xml->children.size() < 1) 
      THROW_RUNTIME_ERROR(xml->loc.str()+": invalid group node");

    /* load instances */
    avector<AffineSpace3fa> xfms = loadInstances(xml->children[0]);

    /* load meshes */
    Ref<SceneGraph::GroupNode> objects = new SceneGraph::GroupNode;
    for (size_t i=1; i<xml->children.size(); i++)
      objects->add(loadObject(xml->children[i]));

    /* create instances */
    Ref<SceneGraph::GroupNode> instances = new SceneGraph::GroupNode;
    for (size_t i=0; i<xfms.size(); i++) 
      instances->add(new SceneGraph::TransformNode(xfms[i],objects.cast<SceneGraph::Node>()));

    return instances.cast<SceneGraph::Node>();
  }
  
  Ref<SceneGraph::Node> CoronaLoader::loadNode(const Ref<XML>& xml)
  {
    if      (xml->name == "conffile"     ) return nullptr;
    else if (xml->name == "mtllib"       ) return nullptr;
    else if (xml->name == "camera"       ) return nullptr;
    else if (xml->name == "environment"  ) return nullptr;
    else if (xml->name == "geometryGroup") return loadGroupNode(xml);
    else THROW_RUNTIME_ERROR(xml->loc.str()+": unknown tag: "+xml->name);
    return nullptr;
  }

  Ref<SceneGraph::Node> CoronaLoader::load(const FileName& fileName, const AffineSpace3fa& space) {
    CoronaLoader loader(fileName,space); return loader.root;
  }

  CoronaLoader::CoronaLoader(const FileName& fileName, const AffineSpace3fa& space)
  {
    path = fileName.path();
    Ref<XML> xml = parseXML(fileName,"/.",false);
    if (xml->name == "scene") 
    {
      Ref<SceneGraph::GroupNode> group = new SceneGraph::GroupNode;
      for (size_t i=0; i<xml->children.size(); i++) { 
        group->add(loadNode(xml->children[i]));
      }
      root = group.cast<SceneGraph::Node>();
    }
    else 
      THROW_RUNTIME_ERROR(xml->loc.str()+": invalid scene tag");

    if (space == AffineSpace3fa(one)) 
      return;
    
    root = new SceneGraph::TransformNode(space,root);
  }

  /*! read from disk */
  Ref<SceneGraph::Node> loadCorona(const FileName& fileName, const AffineSpace3fa& space) {
    return CoronaLoader::load(fileName,space);
  }
}
