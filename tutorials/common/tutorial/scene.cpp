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

    Ref<TutorialScene::Geometry> convertTriangleMesh(Ref<SceneGraph::TriangleMeshNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      unsigned materialID = convert(mesh->material);
      
      TutorialScene::TriangleMesh* objmesh = new TutorialScene::TriangleMesh();
      const LinearSpace3fa nspace0 = rcp(space0.l).transposed();

      objmesh->v.resize(mesh->positions[0]->size()); 
      for (size_t i=0; i<mesh->positions[0]->size(); i++) 
        objmesh->v [i] = xfmPoint ( space0,mesh->positions[0]->at(i));

      if (mesh->numTimeSteps() > 1) {
        objmesh->v2.resize(mesh->positions[1]->size()); 
        for (size_t i=0; i<mesh->positions[1]->size(); i++) 
          objmesh->v2[i] = xfmPoint ( space1,mesh->positions[1]->at(i));
      }
      objmesh->vn.resize(mesh->normals.size()); 
      for (size_t i=0; i<mesh->normals.size(); i++) 
        objmesh->vn[i] = xfmVector(nspace0,mesh->normals[i]);
      objmesh->vt = mesh->texcoords;
      
      objmesh->triangles.resize(mesh->triangles.size());
      for (size_t i=0; i<mesh->triangles.size(); i++) {
        SceneGraph::TriangleMeshNode::Triangle& tri = mesh->triangles[i];
        objmesh->triangles[i] = TutorialScene::Triangle(tri.v0,tri.v1,tri.v2,materialID);
      }
      objmesh->meshMaterialID = materialID;
      return objmesh;
    }

    Ref<TutorialScene::Geometry> convertQuadMesh(Ref<SceneGraph::QuadMeshNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      unsigned materialID = convert(mesh->material);
      
      TutorialScene::QuadMesh* objmesh = new TutorialScene::QuadMesh();
      const LinearSpace3fa nspace0 = rcp(space0.l).transposed();

      objmesh->v. resize(mesh->positions[0]->size()); 
      for (size_t i=0; i<mesh->positions[0]->size(); i++) 
        objmesh->v [i] = xfmPoint ( space0,mesh->positions[0]->at(i));

      if (mesh->numTimeSteps() > 1) {
        objmesh->v2.resize(mesh->positions[1]->size()); 
        for (size_t i=0; i<mesh->positions[1]->size(); i++) 
          objmesh->v2[i] = xfmPoint ( space1,mesh->positions[1]->at(i));
      }
      objmesh->vn.resize(mesh->normals.size()); 
      for (size_t i=0; i<mesh->normals.size(); i++) 
        objmesh->vn[i] = xfmVector(nspace0,mesh->normals[i]);
      objmesh->vt = mesh->texcoords;
      
      objmesh->quads.resize(mesh->quads.size());
      for (size_t i=0; i<mesh->quads.size(); i++) {
        SceneGraph::QuadMeshNode::Quad& quad = mesh->quads[i];
        objmesh->quads[i] = TutorialScene::Quad(quad.v0,quad.v1,quad.v2,quad.v3);
      }
      objmesh->meshMaterialID = materialID;
      return objmesh;
    }

    Ref<TutorialScene::Geometry> convertSubdivMesh(Ref<SceneGraph::SubdivMeshNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      unsigned materialID = convert(mesh->material);
      
      TutorialScene::SubdivMesh* subdivmesh = new TutorialScene::SubdivMesh();
      const LinearSpace3fa nspace0 = rcp(space0.l).transposed();
      
      subdivmesh->positions.resize(mesh->positions_[0]->size()); 
      for (size_t i=0; i<mesh->positions_[0]->size(); i++) 
        subdivmesh->positions[i] = xfmPoint(space0,mesh->positions_[0]->at(i));

      if (mesh->numTimeSteps() > 1)
      {
        subdivmesh->positions2.resize(mesh->positions_[1]->size()); 
        for (size_t i=0; i<mesh->positions_[1]->size(); i++) 
          subdivmesh->positions2[i] = xfmPoint(space0,mesh->positions_[1]->at(i));
      }
      
      subdivmesh->normals.resize(mesh->normals.size()); 
      for (size_t i=0; i<mesh->normals.size(); i++) 
        subdivmesh->normals[i] = xfmVector(nspace0,mesh->normals[i]);
      
      subdivmesh->texcoords = mesh->texcoords;
      subdivmesh->position_indices = mesh->position_indices;
      subdivmesh->normal_indices = mesh->normal_indices;
      subdivmesh->texcoord_indices = mesh->texcoord_indices;
      subdivmesh->verticesPerFace = mesh->verticesPerFace;
      subdivmesh->holes = mesh->holes;
      subdivmesh->edge_creases = mesh->edge_creases;
      subdivmesh->edge_crease_weights = mesh->edge_crease_weights;
      subdivmesh->vertex_creases = mesh->vertex_creases;
      subdivmesh->vertex_crease_weights = mesh->vertex_crease_weights;
      subdivmesh->materialID = materialID;
      return subdivmesh;
    }

    Ref<TutorialScene::Geometry> convertLineSegments(Ref<SceneGraph::LineSegmentsNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      unsigned materialID = convert(mesh->material);
      
      TutorialScene::LineSegments* out = new TutorialScene::LineSegments;
      
      out->v.resize(mesh->positions[0]->size()); 
      for (size_t i=0; i<mesh->positions[0]->size(); i++) {
        out->v[i] = xfmPoint(space0,mesh->positions[0]->at(i));
        out->v[i].w = mesh->positions[0]->at(i).w;
      }
      
      if (mesh->numTimeSteps() > 1)
      {
        out->v2.resize(mesh->positions[1]->size()); 
        for (size_t i=0; i<mesh->positions[1]->size(); i++) {
          out->v2[i] = xfmPoint(space1,mesh->positions[1]->at(i));
          out->v2[i].w = mesh->positions[1]->at(i).w;
        }
      }
      
      out->indices.resize(mesh->indices.size()); 
      for (size_t i=0; i<mesh->indices.size(); i++)
        out->indices[i] = mesh->indices[i];

      out->materialID = materialID;
      return out;
    }

    Ref<TutorialScene::Geometry> convertHairSet(Ref<SceneGraph::HairSetNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      unsigned materialID = convert(mesh->material);
      
      TutorialScene::HairSet* hairset = new TutorialScene::HairSet(mesh->hair);
      
      hairset->v.resize(mesh->positions[0]->size()); 
      for (size_t i=0; i<mesh->positions[0]->size(); i++) {
        hairset->v[i] = xfmPoint(space0,mesh->positions[0]->at(i));
        hairset->v[i].w = mesh->positions[0]->at(i).w;
      }
      
      if (mesh->numTimeSteps() > 1)
      {
        hairset->v2.resize(mesh->positions[1]->size()); 
        for (size_t i=0; i<mesh->positions[1]->size(); i++) {
          hairset->v2[i] = xfmPoint(space1,mesh->positions[1]->at(i));
          hairset->v2[i].w = mesh->positions[1]->at(i).w;
        }
      }
      
      hairset->hairs.resize(mesh->hairs.size()); 
      for (size_t i=0; i<mesh->hairs.size(); i++)
        hairset->hairs[i] = TutorialScene::Hair(mesh->hairs[i].vertex,mesh->hairs[i].id);

      hairset->materialID = materialID;
      return hairset;
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
