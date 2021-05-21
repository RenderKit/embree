// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "scene.h"

namespace embree
{
  extern "C" int g_instancing_mode;

  TutorialScene::TutorialScene() {
  }

  TutorialScene::~TutorialScene() {
  }
  
  void TutorialScene::add(Ref<SceneGraph::GroupNode> group) 
  {
    for (auto& node : group->children) 
    {
      if (Ref<SceneGraph::LightNode> lightNode = node.dynamicCast<SceneGraph::LightNode>()) {
        lights.push_back(lightNode);
      } 
      else if (Ref<SceneGraph::PerspectiveCameraNode> cameraNode = node.dynamicCast<SceneGraph::PerspectiveCameraNode>()) {
        cameras.push_back(cameraNode);
      } 
      else {
        geometries.push_back(node);
      }
    }
  }

  unsigned TutorialScene::materialID(Ref<SceneGraph::MaterialNode> material) 
  {
    if (material->id == -1) {
      materials.push_back(material);
      material->id = unsigned(materials.size()-1);
    }
    return material->id;
  }
  
  void TutorialScene::print_camera_names ()
  {
    if (cameras.size() == 0) {
      std::cout << "no cameras inside the scene" << std::endl;
      return;
    }
    
    for (size_t i=0; i<cameras.size(); i++)
      std::cout << "camera " << i << ": " << cameras[i]->name << std::endl;
  }

  Ref<SceneGraph::PerspectiveCameraNode> TutorialScene::getDefaultCamera()
  {
    if (cameras.size()) return cameras[0];
    return nullptr;
  }

  Ref<SceneGraph::PerspectiveCameraNode> TutorialScene::getCamera(const std::string& name)
  {
    for (size_t i=0; i<cameras.size(); i++)
      if (cameras[i]->name == name) return cameras[i];
    
    THROW_RUNTIME_ERROR("camera \"" + name +"\" not found");
  }
};
