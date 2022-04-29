// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../default.h"
#include "../scenegraph/scenegraph.h"

namespace embree
{
  /*! Flattened scene used inside tutorials */
  struct TutorialScene
  {
    TutorialScene();
    ~TutorialScene();
    
    void add (Ref<SceneGraph::GroupNode> node);
    unsigned materialID(Ref<SceneGraph::MaterialNode> material);
    void print_camera_names ();
    Ref<SceneGraph::PerspectiveCameraNode> getDefaultCamera();
    Ref<SceneGraph::PerspectiveCameraNode> getCamera(const std::string& name);
    
  public:
    std::vector<Ref<SceneGraph::PerspectiveCameraNode>> cameras;  //!< list of all cameras
    std::vector<Ref<SceneGraph::MaterialNode>> materials; //!< list of materials
    std::vector<Ref<SceneGraph::Node> > geometries;   //!< list of geometries
    std::vector<Ref<SceneGraph::LightNode>> lights;       //!< list of lights
  };
}
