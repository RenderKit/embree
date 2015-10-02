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

#include "scenegraph.h"
#include "xml_loader.h"
#include "xml_writer.h"
#include "obj_loader.h"
#include "hair_loader.h"
#include "cy_hair_loader.h"

namespace embree
{
  Ref<SceneGraph::Node> SceneGraph::load(const FileName& filename)
  {
    if      (strlwr(filename.ext()) == std::string("obj" )) return loadOBJ(filename);
    else if (strlwr(filename.ext()) == std::string("xml" )) return loadXML(filename);
    else if (strlwr(filename.ext()) == std::string("hair")) return loadCYHair(filename);
    else if (strlwr(filename.ext()) == std::string("txt" )) return loadTxtHair(filename);
    else if (strlwr(filename.ext()) == std::string("bin" )) return loadBinHair(filename);
    else throw std::runtime_error("unknown scene format: " + filename.ext());
  }

  void SceneGraph::store(Ref<SceneGraph::Node> root, const FileName& filename)
  {
    if (strlwr(filename.ext()) == std::string("xml")) {
      storeXML(root,filename);
    }
    else
      throw std::runtime_error("unknown scene format: " + filename.ext());
  }
  
  void SceneGraph::set_motion_blur(Ref<SceneGraph::Node> node0, Ref<SceneGraph::Node> node1)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode0 = node0.dynamicCast<SceneGraph::TransformNode>()) 
    {
      if (Ref<SceneGraph::TransformNode> xfmNode1 = node1.dynamicCast<SceneGraph::TransformNode>()) 
      {
        xfmNode0->xfm1 = xfmNode1->xfm0;
        set_motion_blur(xfmNode0->child, xfmNode1->child);
      } 
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
    else if (Ref<SceneGraph::GroupNode> groupNode0 = node0.dynamicCast<SceneGraph::GroupNode>()) 
    {
      if (Ref<SceneGraph::GroupNode> groupNode1 = node1.dynamicCast<SceneGraph::GroupNode>()) 
      {
        if (groupNode0->children.size() != groupNode1->children.size()) 
          THROW_RUNTIME_ERROR("incompatible scene graph");

        for (size_t i=0; i<groupNode0->children.size(); i++) 
          set_motion_blur(groupNode0->children[i],groupNode1->children[i]);
      } 
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh0 = node0.dynamicCast<SceneGraph::TriangleMeshNode>()) 
    {
      if (Ref<SceneGraph::TriangleMeshNode> mesh1 = node1.dynamicCast<SceneGraph::TriangleMeshNode>())
      {
        if (mesh0->v.size() != mesh1->v.size())
          THROW_RUNTIME_ERROR("incompatible scene graph");

        bool different = false;
        for (size_t i=0; i<mesh0->v.size(); i++) 
          different |= mesh0->v[i] != mesh1->v[i];
        
        if (different)
          mesh0->v2 = mesh1->v;
      } 
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
    else if (Ref<SceneGraph::HairSetNode> mesh0 = node0.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      if (Ref<SceneGraph::HairSetNode> mesh1 = node1.dynamicCast<SceneGraph::HairSetNode>()) 
      {
        if (mesh0->v.size() != mesh1->v.size())
          THROW_RUNTIME_ERROR("incompatible scene graph");

        bool different = false;
        for (size_t i=0; i<mesh0->v.size(); i++) 
          different |= mesh0->v[i] != mesh1->v[i];
        
        if (different)
          mesh0->v2 = mesh1->v;
      }
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
  }
}

  
