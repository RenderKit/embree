// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../default.h"
#include "../scenegraph/scenegraph.h"

namespace embree
{
  enum Shader { 
    SHADER_DEFAULT, 
    SHADER_EYELIGHT,
    SHADER_OCCLUSION,
    SHADER_UV,
    SHADER_TEXCOORDS,
    SHADER_TEXCOORDS_GRID,
    SHADER_NG,
    SHADER_CYCLES,
    SHADER_GEOMID,
    SHADER_GEOMID_PRIMID,
    SHADER_AMBIENT_OCCLUSION
  };

  /*! Flattened scene used inside tutorials */
  struct TutorialScene
  {
    TutorialScene();
    ~TutorialScene();
    
    void add (Ref<SceneGraph::GroupNode> node);
    unsigned addGeometry(Ref<SceneGraph::Node> node);
    unsigned materialID(Ref<SceneGraph::MaterialNode> material);
    unsigned geometryID(Ref<SceneGraph::Node> geometry);
    void print_camera_names ();
    Ref<SceneGraph::PerspectiveCameraNode> getDefaultCamera();
    Ref<SceneGraph::PerspectiveCameraNode> getCamera(const std::string& name);
    
  public:
    std::vector<Ref<SceneGraph::PerspectiveCameraNode>> cameras;  //!< list of all cameras
    std::vector<Ref<SceneGraph::MaterialNode>> materials; //!< list of materials
    std::vector<Ref<SceneGraph::Node> > geometries;   //!< list of geometries
    std::vector<Ref<SceneGraph::Light>> lights;       //!< list of lights
  };
}
