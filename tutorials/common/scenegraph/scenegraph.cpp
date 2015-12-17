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
    if      (toLowerCase(filename.ext()) == std::string("obj" )) return loadOBJ(filename);
    else if (toLowerCase(filename.ext()) == std::string("xml" )) return loadXML(filename);
    else if (toLowerCase(filename.ext()) == std::string("hair")) return loadCYHair(filename);
    else if (toLowerCase(filename.ext()) == std::string("txt" )) return loadTxtHair(filename);
    else if (toLowerCase(filename.ext()) == std::string("bin" )) return loadBinHair(filename);
    else throw std::runtime_error("unknown scene format: " + filename.ext());
  }

  void SceneGraph::store(Ref<SceneGraph::Node> root, const FileName& filename, bool embedTextures)
  {
    if (toLowerCase(filename.ext()) == std::string("xml")) {
      storeXML(root,filename,embedTextures);
    }
    else
      throw std::runtime_error("unknown scene format: " + filename.ext());
  }

  void SceneGraph::Node::resetNode(std::set<Ref<Node>>& done)
  {
    indegree = 0;
    closed = true;
  }

  void SceneGraph::TransformNode::resetNode(std::set<Ref<Node>>& done)
  {
    if (done.find(this) != done.end()) return;
    else done.insert(this);
    indegree = 0;
    closed = false;
    child->resetNode(done);
  }

  void SceneGraph::GroupNode::resetNode(std::set<Ref<Node>>& done)
  {
    if (done.find(this) != done.end()) return;
    else done.insert(this);
    indegree = 0;
    closed = false;
    for (auto c : children)
      c->resetNode(done);
  }

  void SceneGraph::Node::calculateInDegree() {
    indegree++;
  }

  void SceneGraph::TransformNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1) {
      child->calculateInDegree();
      if (xfm0 != xfm1) child->calculateInDegree(); // break instanced up when motion blur is used
    }
  }

  void SceneGraph::GroupNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1) {
      for (auto c : children)
        c->calculateInDegree();
    }
  }

  bool SceneGraph::Node::calculateClosed() 
  {
    assert(indegree);
    closed = true;
    return closed && (indegree == 1);
  }

  bool SceneGraph::TransformNode::calculateClosed() 
  {
    assert(indegree);
    closed = child->calculateClosed();
    return closed && (indegree == 1);
  }

  bool SceneGraph::GroupNode::calculateClosed()
  {
    assert(indegree);
    closed = true;
    for (auto c : children)
      closed &= c->calculateClosed();
    return closed && (indegree == 1);
  }
  
  void SceneGraph::TriangleMeshNode::verify() const
  {
    const size_t numVertices = v.size();
    if (v2.size() && v2.size() != numVertices) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    if (vn.size() && vn.size() != numVertices) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    if (vt.size() && vt.size() != numVertices) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (auto tri : triangles) {
      if (size_t(tri.v0) >= numVertices || size_t(tri.v1) >= numVertices || size_t(tri.v2) >= numVertices)
        THROW_RUNTIME_ERROR("invalid triangle");
    }
  }

  void SceneGraph::QuadMeshNode::verify() const
  {
    const size_t numVertices = v.size();
    if (v2.size() && v2.size() != numVertices) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    if (vn.size() && vn.size() != numVertices) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    if (vt.size() && vt.size() != numVertices) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (auto quad : quads) {
      if (size_t(quad.v0) >= numVertices || size_t(quad.v1) >= numVertices || size_t(quad.v2) >= numVertices || size_t(quad.v3) >= numVertices)
        THROW_RUNTIME_ERROR("invalid quad");
    }
  }

  void SceneGraph::SubdivMeshNode::verify() const
  {
    for (auto i : position_indices) 
      if (size_t(i) >= positions.size()) THROW_RUNTIME_ERROR("invalid position index array");
    for (auto i : normal_indices) 
      if (size_t(i) >= normals.size()) THROW_RUNTIME_ERROR("invalid normal index array");
    for (auto i : texcoord_indices) 
      if (size_t(i) >= texcoords.size()) THROW_RUNTIME_ERROR("invalid texcoord index array");
    for (auto i : holes) 
      if (size_t(i) >= verticesPerFace.size()) THROW_RUNTIME_ERROR("invalid hole index array");
    for (auto crease : edge_creases) 
      if (max(size_t(crease.x),size_t(crease.y)) >= positions.size()) THROW_RUNTIME_ERROR("invalid edge crease array");
    if (edge_crease_weights.size() != edge_creases.size())
      THROW_RUNTIME_ERROR("invalid edge crease weight array");
    for (auto crease : vertex_creases) 
      if (size_t(crease) >= positions.size()) THROW_RUNTIME_ERROR("invalid vertex crease array");
    if (vertex_crease_weights.size() != vertex_creases.size())
      THROW_RUNTIME_ERROR("invalid vertex crease weight array");
  }

  void SceneGraph::LineSegmentsNode::verify() const
  {
    const size_t numVertices = v.size();
    if (v2.size() && v2.size() != numVertices) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (auto i : indices) {
      if (size_t(i) >= numVertices)
        THROW_RUNTIME_ERROR("invalid line segment");
    }
  }

  void SceneGraph::HairSetNode::verify() const
  {
    const size_t numVertices = v.size();
    if (v2.size() && v2.size() != numVertices) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (auto hair : hairs) {
      if (size_t(hair.vertex) >= numVertices)
        THROW_RUNTIME_ERROR("invalid hair");
    }
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
    else if (Ref<SceneGraph::QuadMeshNode> mesh0 = node0.dynamicCast<SceneGraph::QuadMeshNode>()) 
    {
      if (Ref<SceneGraph::QuadMeshNode> mesh1 = node1.dynamicCast<SceneGraph::QuadMeshNode>())
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
    else if (Ref<SceneGraph::SubdivMeshNode> mesh0 = node0.dynamicCast<SceneGraph::SubdivMeshNode>()) 
    {
      if (Ref<SceneGraph::SubdivMeshNode> mesh1 = node1.dynamicCast<SceneGraph::SubdivMeshNode>()) 
      {
        if (mesh0->positions.size() != mesh1->positions.size())
          THROW_RUNTIME_ERROR("incompatible scene graph");
        if (mesh0->verticesPerFace.size() != mesh1->verticesPerFace.size())
          THROW_RUNTIME_ERROR("incompatible scene graph");
        for (size_t i=0; i<mesh0->verticesPerFace.size(); i++) 
          if (mesh0->verticesPerFace[i] != mesh1->verticesPerFace[i])
            THROW_RUNTIME_ERROR("incompatible scene graph");

        bool different = false;
        for (size_t i=0; i<mesh0->positions.size(); i++) 
          different |= mesh0->positions[i] != mesh1->positions[i];
        
        if (different)
          mesh0->positions2 = mesh1->positions;
      }
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
  }

  std::pair<int,int> quad_index2(int p, int a0, int a1, int b0, int b1)
  {
    if      (b0 == a0) return std::make_pair(p-1,b1);
    else if (b0 == a1) return std::make_pair(p+0,b1);
    else if (b1 == a0) return std::make_pair(p-1,b0);
    else if (b1 == a1) return std::make_pair(p+0,b0);
    else return std::make_pair(0,-1);
  }
  
  std::pair<int,int> quad_index3(int a0, int a1, int a2, int b0, int b1, int b2)
  {
    if      (b0 == a0) return quad_index2(0,a2,a1,b1,b2);
    else if (b0 == a1) return quad_index2(1,a0,a2,b1,b2);
    else if (b0 == a2) return quad_index2(2,a1,a0,b1,b2);
    else if (b1 == a0) return quad_index2(0,a2,a1,b0,b2);
    else if (b1 == a1) return quad_index2(1,a0,a2,b0,b2);
    else if (b1 == a2) return quad_index2(2,a1,a0,b0,b2);
    else if (b2 == a0) return quad_index2(0,a2,a1,b0,b1);
    else if (b2 == a1) return quad_index2(1,a0,a2,b0,b1);
    else if (b2 == a2) return quad_index2(2,a1,a0,b0,b1);
    else return std::make_pair(0,-1);
  }
  
  Ref<SceneGraph::Node> SceneGraph::convert_triangles_to_quads(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = convert_triangles_to_quads(xfmNode->child);
    } 
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = convert_triangles_to_quads(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> tmesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
    {
      SceneGraph::QuadMeshNode* qmesh = new SceneGraph::QuadMeshNode(tmesh->material);
      qmesh->v = tmesh->v;
      qmesh->v2 = tmesh->v2;
      qmesh->vn = tmesh->vn;
      qmesh->vt = tmesh->vt;
      for (size_t i=0; i<tmesh->triangles.size(); i++)
      {
        const int a0 = tmesh->triangles[i+0].v0;
        const int a1 = tmesh->triangles[i+0].v1;
        const int a2 = tmesh->triangles[i+0].v2;
        if (i+1 == tmesh->triangles.size()) {
          qmesh->quads.push_back(SceneGraph::QuadMeshNode::Quad(a0,a1,a2,a2));
          continue;
        }

        const int b0 = tmesh->triangles[i+1].v0;
        const int b1 = tmesh->triangles[i+1].v1;
        const int b2 = tmesh->triangles[i+1].v2;
        const std::pair<int,int> q = quad_index3(a0,a1,a2,b0,b1,b2);
        const int a3 = q.second;
        if (a3 == -1) {
          qmesh->quads.push_back(SceneGraph::QuadMeshNode::Quad(a0,a1,a2,a2));
          continue;
        }

        if      (q.first == -1) qmesh->quads.push_back(SceneGraph::QuadMeshNode::Quad(a1,a2,a3,a0));
        else if (q.first ==  0) qmesh->quads.push_back(SceneGraph::QuadMeshNode::Quad(a3,a1,a2,a0));
        else if (q.first ==  1) qmesh->quads.push_back(SceneGraph::QuadMeshNode::Quad(a0,a1,a3,a2));
        else if (q.first ==  2) qmesh->quads.push_back(SceneGraph::QuadMeshNode::Quad(a1,a2,a3,a0)); 
        i++;
      }
      return qmesh;
    }
    return node;
  }
    
  Ref<SceneGraph::Node> SceneGraph::convert_bezier_to_lines(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = convert_bezier_to_lines(xfmNode->child);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = convert_bezier_to_lines(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::HairSetNode> hmesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      SceneGraph::LineSegmentsNode* lmesh = new SceneGraph::LineSegmentsNode(hmesh->material);
      lmesh->v  = hmesh->v;
      lmesh->v2 = hmesh->v2;
      for (auto hair : hmesh->hairs) {
        lmesh->indices.push_back(hair.vertex+0);
        lmesh->indices.push_back(hair.vertex+1);
        lmesh->indices.push_back(hair.vertex+2);
      }
      return lmesh;
    }
    return node;
  }
}
