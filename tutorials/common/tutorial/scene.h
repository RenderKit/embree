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
