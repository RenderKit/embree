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

#include "scene.h"

namespace embree
{
  struct SceneGraphConverter
  {
    TutorialScene* scene;
    bool instancing;
    std::map<Ref<SceneGraph::MaterialNode>, size_t> material2id;
    std::map<Ref<SceneGraph::Node>, size_t> geometry2id;
    
    SceneGraphConverter (Ref<SceneGraph::Node> in, TutorialScene* scene, bool instancing)
      : scene(scene), instancing(instancing) { convert(in,one,one); }

    int convert(Ref<SceneGraph::MaterialNode> node) 
    {
      if (material2id.find(node) == material2id.end()) {
        scene->materials.push_back(node->material);
        material2id[node] = scene->materials.size()-1;
      }
      return material2id[node];
    }

    int convertTriangleMesh(Ref<SceneGraph::TriangleMeshNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      int materialID = convert(mesh->material);
      
      TutorialScene::Mesh* objmesh = new TutorialScene::Mesh();
      const LinearSpace3fa nspace0 = rcp(space0.l).transposed();
      objmesh->v. resize(mesh->v. size()); for (size_t i=0; i<mesh->v. size(); i++) objmesh->v [i] = xfmPoint ( space0,mesh->v [i]);
      objmesh->v2.resize(mesh->v2.size()); for (size_t i=0; i<mesh->v2.size(); i++) objmesh->v2[i] = xfmPoint ( space1,mesh->v2[i]);
      objmesh->vn.resize(mesh->vn.size()); for (size_t i=0; i<mesh->vn.size(); i++) objmesh->vn[i] = xfmVector(nspace0,mesh->vn[i]);
      objmesh->vt = mesh->vt;
      
      objmesh->triangles.resize(mesh->triangles.size());
      for (size_t i=0; i<mesh->triangles.size(); i++) {
        SceneGraph::TriangleMeshNode::Triangle& tri = mesh->triangles[i];
        objmesh->triangles[i] = TutorialScene::Triangle(tri.v0,tri.v1,tri.v2,materialID);
      }
      objmesh->meshMaterialID = materialID;
      scene->geometries.push_back(objmesh);
      return scene->geometries.size()-1;
    }

    int lookupTriangleMesh(Ref<SceneGraph::TriangleMeshNode> mesh)
    {
      Ref<SceneGraph::Node> node = mesh.dynamicCast<SceneGraph::Node>();
      if (geometry2id.find(node) == geometry2id.end())
        geometry2id[node] = convertTriangleMesh(mesh,one,one);
      return geometry2id[node];
    }

    int convertSubdivMesh(Ref<SceneGraph::SubdivMeshNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      int materialID = convert(mesh->material);
      
      TutorialScene::SubdivMesh* subdivmesh = new TutorialScene::SubdivMesh();
      const LinearSpace3fa nspace0 = rcp(space0.l).transposed();
      
      subdivmesh->positions.resize(mesh->positions.size()); 
      for (size_t i=0; i<mesh->positions.size(); i++) 
        subdivmesh->positions[i] = xfmPoint(space0,mesh->positions[i]);
      
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

      scene->geometries.push_back(subdivmesh);
      return scene->geometries.size()-1;
    }

    int lookupSubdivMesh(Ref<SceneGraph::SubdivMeshNode> mesh)
    {
      Ref<SceneGraph::Node> node = mesh.dynamicCast<SceneGraph::Node>();
      if (geometry2id.find(node) == geometry2id.end())
        geometry2id[node] = convertSubdivMesh(mesh,one,one);
      return geometry2id[node];
    }

    int convertHairSet(Ref<SceneGraph::HairSetNode> mesh, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      int materialID = convert(mesh->material);
      
      TutorialScene::HairSet* hairset = new TutorialScene::HairSet;
      
      hairset->v.resize(mesh->v.size()); 
      for (size_t i=0; i<mesh->v.size(); i++) {
        hairset->v[i] = xfmPoint(space0,mesh->v[i]);
        hairset->v[i].w = mesh->v[i].w;
      }
      
      hairset->v2.resize(mesh->v2.size()); 
      for (size_t i=0; i<mesh->v2.size(); i++) {
        hairset->v2[i] = xfmPoint(space1,mesh->v2[i]);
        hairset->v2[i].w = mesh->v2[i].w;
      }
      
      hairset->hairs.resize(mesh->hairs.size()); 
      for (size_t i=0; i<mesh->hairs.size(); i++)
        hairset->hairs[i] = TutorialScene::Hair(mesh->hairs[i].vertex,mesh->hairs[i].id);

      scene->geometries.push_back(hairset);
      return scene->geometries.size()-1;
    }

    int lookupHairSet(Ref<SceneGraph::HairSetNode> mesh)
    {
      Ref<SceneGraph::Node> node = mesh.dynamicCast<SceneGraph::Node>();
      if (geometry2id.find(node) == geometry2id.end())
        geometry2id[node] = convertHairSet(mesh,one,one);
      return geometry2id[node];
    }

    void convert(Ref<SceneGraph::Node> node, const AffineSpace3fa& space0, const AffineSpace3fa& space1)
    {
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convert(xfmNode->child, space0*xfmNode->xfm0, space1*xfmNode->xfm1);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (auto child : groupNode->children) convert(child,space0,space1);
      }
      else if (Ref<SceneGraph::LightNode<AmbientLight> > ambientLight = node.dynamicCast<SceneGraph::LightNode<AmbientLight> >()) {
        scene->ambientLights.push_back(ambientLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::LightNode<PointLight> > pointLight = node.dynamicCast<SceneGraph::LightNode<PointLight> >()) {
        scene->pointLights.push_back(pointLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::LightNode<DirectionalLight> > directionalLight = node.dynamicCast<SceneGraph::LightNode<DirectionalLight> >()) {
        scene->directionalLights.push_back(directionalLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::LightNode<SpotLight> > spotLight = node.dynamicCast<SceneGraph::LightNode<SpotLight> >()) {
        //scene->spotLights.push_back(spotLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::LightNode<DistantLight> > distantLight = node.dynamicCast<SceneGraph::LightNode<DistantLight> >()) {
        scene->distantLights.push_back(distantLight->light.transform(space0));
      }
      else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
      {
        if (instancing)
          scene->geometries.push_back(new TutorialScene::Instance(space0,lookupTriangleMesh(mesh)));
        else
          convertTriangleMesh(mesh,space0,space1);
      }
      else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>()) 
      {
        //if (instancing)
        //  scene->instances.push_back(TutorialScene::Instance(space0,lookupSubdivMesh(mesh)));
        //else
        convertSubdivMesh(mesh,space0,space1);
      }
      else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
      {
        //if (instancing)
        //  scene->instances.push_back(TutorialScene::Instance(space0,lookupHairSet(mesh)));
        //else
        convertHairSet(mesh,space0,space1);
      }
    }
  };

  void TutorialScene::add(Ref<SceneGraph::Node> node, bool instancing) {
    SceneGraphConverter(node,this,instancing);
  }
};
