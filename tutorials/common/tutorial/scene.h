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

#pragma once

#include "../default.h"
#include "../scenegraph/scenegraph.h"

namespace embree
{
  enum Shader { 
    SHADER_DEFAULT, 
    SHADER_EYELIGHT,
    SHADER_UV,
    SHADER_TEXCOORDS,
    SHADER_TEXCOORDS_GRID,
    SHADER_NG,
    SHADER_GEOMID,
    SHADER_GEOMID_PRIMID,
    SHADER_AMBIENT_OCCLUSION
  };

  /*! Flattened scene used inside tutorials */
  struct TutorialScene
  {
    void add (Ref<SceneGraph::GroupNode> node);

    unsigned addGeometry(Ref<SceneGraph::Node> node) 
    {
      if (geometry2id.find(node) == geometry2id.end()) {
        geometries.push_back(node);
        geometry2id[node] = unsigned(geometries.size()-1);
      }
      return geometry2id[node];
    }
    
    unsigned materialID(Ref<SceneGraph::MaterialNode> material) 
    {
      if (material2id.find(material) == material2id.end()) {
        materials.push_back(material->material);
        material2id[material] = unsigned(materials.size()-1);
      }
      return material2id[material];
    }

    unsigned geometryID(Ref<SceneGraph::Node> geometry) 
    {
      assert(geometry2id.find(geometry) != geometry2id.end());
      return geometry2id[geometry];
    }
    
  public:
    avector<Material> materials;                      //!< list of materials
    std::vector<Ref<SceneGraph::Node> > geometries;   //!< list of geometries
    std::vector<Ref<SceneGraph::Light>> lights;       //!< list of lights
  public:
    std::map<Ref<SceneGraph::MaterialNode>, unsigned> material2id;
    std::map<Ref<SceneGraph::Node>, unsigned> geometry2id;
  public:
    std::vector<RTCScene> geomID_to_scene;           //!< map a geometry ID to the scene it might have been created for it in case of instancing
    std::vector<void*> geomID_to_inst;       //!< maps instance ID to instance structure
  };
}
