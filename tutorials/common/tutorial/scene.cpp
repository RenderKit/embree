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
  struct SceneGraphConverter
  {
    TutorialScene* scene;
    std::map<Ref<SceneGraph::Node>,Ref<SceneGraph::Node>> object_mapping;
    
    SceneGraphConverter (Ref<SceneGraph::Node> in, TutorialScene* scene, SceneGraph::InstancingMode instancing)
      : scene(scene)
    { 
      convertLights(in,one);

      if (instancing != SceneGraph::INSTANCING_NONE) 
      {
        if (instancing == SceneGraph::INSTANCING_SCENE_GROUP) 
        {
          in->reset();
          in->calculateInDegree();
          in->calculateClosed();
        }
        convertInstances(in,one);
      }
      else
      {
        std::vector<Ref<SceneGraph::Node>> geometries;
        convertGeometries(geometries,in,one);
        for (auto& geom : geometries)
          scene->addGeometry(geom);
      }

      scene->geomID_to_scene.resize(scene->geometries.size());
      scene->geomID_to_inst.resize(scene->geometries.size());
    }

    void convertLights(Ref<SceneGraph::Node> node, const SceneGraph::Transformations& spaces)
    {
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertLights(xfmNode->child, spaces*xfmNode->spaces);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (const auto& child : groupNode->children) convertLights(child,spaces);
      }
      else if (Ref<SceneGraph::LightNode> lightNode = node.dynamicCast<SceneGraph::LightNode>()) {
        scene->lights.push_back(lightNode->light->transform(spaces[0]));
      }
    }

    void convertGeometries(std::vector<Ref<SceneGraph::Node>>& group, Ref<SceneGraph::Node> node, const SceneGraph::Transformations& spaces)
    {
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertGeometries(group,xfmNode->child, spaces*xfmNode->spaces);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (const auto& child : groupNode->children) convertGeometries(group,child,spaces);
      }
      else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
      {
        scene->addMaterial(mesh->material);
        group.push_back(new SceneGraph::TriangleMeshNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) 
      {
        scene->addMaterial(mesh->material);
        group.push_back(new SceneGraph::QuadMeshNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>()) 
      {
        scene->addMaterial(mesh->material);
        group.push_back(new SceneGraph::SubdivMeshNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::LineSegmentsNode> mesh = node.dynamicCast<SceneGraph::LineSegmentsNode>()) 
      {
        scene->addMaterial(mesh->material);
        group.push_back(new SceneGraph::LineSegmentsNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
      {
        scene->addMaterial(mesh->material);
        group.push_back(new SceneGraph::HairSetNode(mesh,spaces));
      }
    }

    Ref<SceneGraph::Node> lookupGeometries(Ref<SceneGraph::Node> node)
    {
      if (object_mapping.find(node) == object_mapping.end())
      {
        std::vector<Ref<SceneGraph::Node>> geometries;
        convertGeometries(geometries,node,one);
        
        if (geometries.size() == 1) {
          scene->addGeometry(geometries[0]);
          object_mapping[node] = geometries[0];
        }
        else {
          Ref<SceneGraph::Node> group = new SceneGraph::GroupNode(geometries);
          scene->addGeometry(group);
          object_mapping[node] = group;
        }
      }
      
      return object_mapping[node];
    }

    void convertInstances(Ref<SceneGraph::Node> node, const SceneGraph::Transformations& spaces)
    {
      if (node->isClosed()) {
        scene->addGeometry(new SceneGraph::TransformNode(spaces,lookupGeometries(node)));
      }
      else if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertInstances(xfmNode->child, spaces*xfmNode->spaces);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (const auto& child : groupNode->children) convertInstances(child,spaces);
      }
    }
  };

  void TutorialScene::add(Ref<SceneGraph::Node> node, SceneGraph::InstancingMode instancing) {
    SceneGraphConverter(node,this,instancing);
  }
};
