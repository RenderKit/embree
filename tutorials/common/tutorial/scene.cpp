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

#include "scene.h"

namespace embree
{
  struct SceneGraphConverter
  {
    TutorialScene* scene;
    std::map<Ref<SceneGraph::MaterialNode>, unsigned> material2id;
    std::map<Ref<SceneGraph::Node>, unsigned> geometry2id;
    
    SceneGraphConverter (Ref<SceneGraph::Node> in, TutorialScene* scene, TutorialScene::InstancingMode instancing)
      : scene(scene)
    { 
      convertLights(in,one,one);

      if (instancing != TutorialScene::INSTANCING_NONE) 
      {
        if (instancing == TutorialScene::INSTANCING_SCENE_GROUP) 
        {
          in->reset();
          in->calculateInDegree();
          in->calculateClosed();
        }
        convertInstances(in,one,one);
      }
      else
        convertGeometries(scene->geometries,in,one,one);
    }

    unsigned convert(Ref<SceneGraph::MaterialNode> node) 
    {
      if (material2id.find(node) == material2id.end()) {
        scene->materials.push_back(node->material);
        material2id[node] = unsigned(scene->materials.size()-1);
      }
      return material2id[node];
    }

    Ref<TutorialScene::Geometry> convertTriangleMesh(Ref<SceneGraph::TriangleMeshNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1) {
      return new TutorialScene::TriangleMesh(mesh,space0,space1,convert(mesh->material));
    }
    
    Ref<TutorialScene::Geometry> convertQuadMesh(Ref<SceneGraph::QuadMeshNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1) {
      return new TutorialScene::QuadMesh(mesh,space0,space1,convert(mesh->material));
    }

    Ref<TutorialScene::Geometry> convertSubdivMesh(Ref<SceneGraph::SubdivMeshNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1) {
      return new TutorialScene::SubdivMesh(mesh,space0,space1,convert(mesh->material));
    }

    Ref<TutorialScene::Geometry> convertLineSegments(Ref<SceneGraph::LineSegmentsNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1) {
      return new TutorialScene::LineSegments(mesh,space0,space1,convert(mesh->material));
    }

    Ref<TutorialScene::Geometry> convertHairSet(Ref<SceneGraph::HairSetNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1) {
      return new TutorialScene::HairSet(mesh,space0,space1,convert(mesh->material));
    }

     void convertLights(Ref<SceneGraph::Node> node, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertLights(xfmNode->child, space0*xfmNode->xfm0, space1*xfmNode->xfm1);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (auto child : groupNode->children) convertLights(child,space0,space1);
      }
      else if (Ref<SceneGraph::LightNode> lightNode = node.dynamicCast<SceneGraph::LightNode>()) {
        scene->lights.push_back(lightNode->light->transform(space0));
      }
    }

    void convertGeometries(std::vector<Ref<TutorialScene::Geometry>>& group, Ref<SceneGraph::Node> node, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertGeometries(group,xfmNode->child, space0*xfmNode->xfm0, space1*xfmNode->xfm1);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (auto child : groupNode->children) convertGeometries(group,child,space0,space1);
      }
      else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) {
        group.push_back(convertTriangleMesh(mesh,space0,space1));
      }
      else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) {
        group.push_back(convertQuadMesh(mesh,space0,space1));
      }
      else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>()) {
        group.push_back(convertSubdivMesh(mesh,space0,space1));
      }
      else if (Ref<SceneGraph::LineSegmentsNode> mesh = node.dynamicCast<SceneGraph::LineSegmentsNode>()) {
        group.push_back(convertLineSegments(mesh,space0,space1));
      }
      else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) {
        group.push_back(convertHairSet(mesh,space0,space1));
      }
    }

    unsigned lookupGeometries(Ref<SceneGraph::Node> node)
    {
      if (geometry2id.find(node) == geometry2id.end())
      {
        std::vector<Ref<TutorialScene::Geometry>> geometries;
        convertGeometries(geometries,node,one,one);
        
        if (geometries.size() == 1)
          scene->geometries.push_back(geometries[0]);
        else 
          scene->geometries.push_back(new TutorialScene::Group(geometries));
        
        geometry2id[node] = unsigned(scene->geometries.size()-1);
      }
      return geometry2id[node];
    }

    void convertInstances(Ref<SceneGraph::Node> node, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      if (node->isClosed()) {
        scene->geometries.push_back(new TutorialScene::Instance(space0,space1,lookupGeometries(node)));
      }
      else if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertInstances(xfmNode->child, space0*xfmNode->xfm0, space1*xfmNode->xfm1);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (auto child : groupNode->children) convertInstances(child,space0,space1);
      }
    }
  };

  void TutorialScene::add(Ref<SceneGraph::Node> node, TutorialScene::InstancingMode instancing) {
    SceneGraphConverter(node,this,instancing);
  }
};
