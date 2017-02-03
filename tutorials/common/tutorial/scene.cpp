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

#include "scene.h"

namespace embree
{
  void TutorialScene::add(Ref<SceneGraph::GroupNode> group) 
  {
    std::set<Ref<SceneGraph::Node>> visited;
    for (auto& node : group->children) 
    {
      if (Ref<SceneGraph::LightNode> lightNode = node.dynamicCast<SceneGraph::LightNode>()) {
        lights.push_back(lightNode->light);
      } 
      else if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        if (visited.find(xfmNode->child) == visited.end()) {
          addGeometry(xfmNode->child);
        }
        addGeometry(node);
      } 
      else {
        addGeometry(node);
      }
      geomID_to_scene.resize(geometries.size());
      geomID_to_inst.resize(geometries.size());
    }
  }
};
