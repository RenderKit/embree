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

#include "scenegraph.h"
#include "xml_loader.h"
#include "xml_writer.h"
#include "obj_loader.h"
#include "ply_loader.h"
#include "corona_loader.h"

namespace embree
{
  extern "C" RTCDevice g_device;
  
  Ref<SceneGraph::Node> SceneGraph::load(const FileName& filename, const bool singleObject)
  {
    if      (toLowerCase(filename.ext()) == std::string("obj" )) return loadOBJ(filename, false, singleObject);
    else if (toLowerCase(filename.ext()) == std::string("ply" )) return loadPLY(filename);
    else if (toLowerCase(filename.ext()) == std::string("xml" )) return loadXML(filename);
    else if (toLowerCase(filename.ext()) == std::string("scn" )) return loadCorona(filename);
    else throw std::runtime_error("unknown scene format: " + filename.ext());
  }

  void SceneGraph::store(Ref<SceneGraph::Node> root, const FileName& filename, bool embedTextures, bool referenceMaterials)
  {
    if (toLowerCase(filename.ext()) == std::string("xml")) {
      storeXML(root,filename,embedTextures,referenceMaterials);
    }
    else
      throw std::runtime_error("unknown scene format: " + filename.ext());
  }

  void SceneGraph::Node::calculateStatistics(Statistics& stat) {
    indegree++;
  }

  void SceneGraph::PerspectiveCameraNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) stat.numCameras++;
  }

  void SceneGraph::LightNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) stat.numLights++;
  }

  void SceneGraph::MaterialNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) stat.numMaterials++;
  }

  void SceneGraph::TriangleMeshNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) {
      stat.numTriangleMeshes++;
      stat.numTriangles += numPrimitives();
      stat.numTriangleBytes += numBytes();
      material->calculateStatistics(stat);
    }
  }

  void SceneGraph::QuadMeshNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) {
      stat.numQuadMeshes++;
      stat.numQuads += numPrimitives();
      stat.numQuadBytes += numBytes();
      material->calculateStatistics(stat);
    }
  }

  void SceneGraph::SubdivMeshNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) {
      stat.numSubdivMeshes++;
      stat.numPatches += numPrimitives();
      stat.numSubdivBytes += numBytes();
      material->calculateStatistics(stat);
    }
  }

  void SceneGraph::HairSetNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) {
      stat.numCurveSets++;
      stat.numCurves += numPrimitives();
      stat.numCurveBytes += numBytes();
      material->calculateStatistics(stat);
    }
  }

  void SceneGraph::PointSetNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) {
      stat.numCurveSets++;
      stat.numCurves += numPrimitives();
      stat.numCurveBytes += numBytes();
      material->calculateStatistics(stat);
    }
  }


  void SceneGraph::GridMeshNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) {
      stat.numGridMeshNodes++;
      stat.numGrids += numPrimitives();
      stat.numGridBytes += numBytes();
      material->calculateStatistics(stat);
    }
  }

  void SceneGraph::TransformNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1)
    {
      child->calculateStatistics(stat);
      
      stat.numTransformNodes++;
      if (child->indegree == 1)
        stat.numTransformedObjects++;
      
      if (spaces.size() > 1) child->calculateStatistics(stat); // break instance up when motion blur is used
    }
  }

  void SceneGraph::GroupNode::calculateStatistics(Statistics& stat)
  {
    indegree++;
    if (indegree == 1) {
      for (auto&  c : children)
        c->calculateStatistics(stat);
    }
  }

  SceneGraph::Statistics SceneGraph::calculateStatistics(Ref<Node> node)
  {
    SceneGraph::Statistics stat;
    node->calculateStatistics(stat);
    node->resetInDegree();
    return stat;
  }

  void SceneGraph::Statistics::print()
  {
    std::cout << "  # transform nodes : " << numTransformNodes << std::endl;
    std::cout << "    # objects       : " << numTransformedObjects << std::endl;
    std::cout << "  # triangle meshes : " << numTriangleMeshes << " ( " << 1E-6*numTriangleBytes << " MB )" << std::endl;
    std::cout << "    # triangles     : " << numTriangles << std::endl;
    std::cout << "  # quad meshes     : " << numQuadMeshes << " ( " << 1E-6*numQuadBytes << " MB )" << std::endl;
    std::cout << "    # quads         : " << numQuads << std::endl;
    std::cout << "  # subdiv meshes   : " << numSubdivMeshes << " ( " << 1E-6*numSubdivBytes << " MB )" << std::endl;
    std::cout << "    # patches       : " << numPatches << std::endl;
    std::cout << "  # curve sets      : " << numCurveSets << " ( " << 1E-6*numCurveBytes << " MB )" << std::endl;
    std::cout << "    # curves        : " << numCurves << std::endl;
    std::cout << "  # grid meshes     : " << numGridMeshNodes << " ( " << 1E-6*numGridBytes << " MB )" << std::endl;
    std::cout << "    # grids         : " << numGrids << std::endl;
    std::cout << "  # point sets      : " << numPointSets << " ( " << 1E-6*numPointBytes << " MB )" << std::endl;
    std::cout << "    # points        : " << numPoints << std::endl;
    std::cout << "  # lights          : " << numLights << std::endl;
    std::cout << "  # cameras         : " << numCameras << std::endl;
    std::cout << "  # materials       : " << numMaterials << std::endl;
  }

  void SceneGraph::Node::calculateInDegree() {
    indegree++;
  }

  void SceneGraph::TriangleMeshNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1)
      material->calculateInDegree();
  }

  void SceneGraph::QuadMeshNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1) 
      material->calculateInDegree();
  }

  void SceneGraph::SubdivMeshNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1) 
      material->calculateInDegree();
  }

  void SceneGraph::HairSetNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1) 
      material->calculateInDegree();
  }

  void SceneGraph::PointSetNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1)
      material->calculateInDegree();
  }

  void SceneGraph::GridMeshNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1) 
      material->calculateInDegree();
  }

  void SceneGraph::TransformNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1) {
      child->calculateInDegree();
      if (spaces.size() > 1) child->calculateInDegree(); // break instance up when motion blur is used
    }
  }

  void SceneGraph::GroupNode::calculateInDegree()
  {
    indegree++;
    if (indegree == 1) {
      for (auto&  c : children)
        c->calculateInDegree();
    }
  }

  bool SceneGraph::Node::calculateClosed(bool group_instancing) 
  {
    assert(indegree);
    closed = true;
    hasLightOrCamera = false;
    return indegree == 1;
  }

  bool SceneGraph::LightNode::calculateClosed(bool group_instancing)
  {
    assert(indegree);
    closed = true;
    hasLightOrCamera = true;
    return indegree == 1;
  }

  bool SceneGraph::PerspectiveCameraNode::calculateClosed(bool group_instancing)
  {
    assert(indegree);
    closed = true;
    hasLightOrCamera = true;
    return indegree == 1;
  }

  bool SceneGraph::TransformNode::calculateClosed(bool group_instancing) 
  {
    assert(indegree);
    if (!closed) {
      closed = group_instancing;
      closed &= child->calculateClosed(group_instancing);
      hasLightOrCamera = child->hasLightOrCamera;
    }
    return closed && (indegree == 1);
  }

  bool SceneGraph::GroupNode::calculateClosed(bool group_instancing)
  {
    assert(indegree);
    if (!closed) {
      closed = group_instancing;
      hasLightOrCamera = false;
      for (auto c : children) {
        closed &= c->calculateClosed(group_instancing);
        hasLightOrCamera |= c->hasLightOrCamera;
      }
    }
    return closed && (indegree == 1);
  }

  void SceneGraph::Node::resetInDegree()
  {
    closed = false;
    indegree--;
  }

  void SceneGraph::TriangleMeshNode::resetInDegree()
  {
    closed = false;
    if (indegree == 1)
      material->resetInDegree();
    indegree--;
  }

  void SceneGraph::QuadMeshNode::resetInDegree()
  {
    closed = false;
    if (indegree == 1)
      material->resetInDegree();
    indegree--;
  }

  void SceneGraph::SubdivMeshNode::resetInDegree()
  {
    closed = false;
    if (indegree == 1)
      material->resetInDegree();
    indegree--;
  }

  void SceneGraph::HairSetNode::resetInDegree()
  {
    closed = false;
    if (indegree == 1)
      material->resetInDegree();
    indegree--;
  }

  void SceneGraph::PointSetNode::resetInDegree()
  {
    closed = false;
    if (indegree == 1)
      material->resetInDegree();
    indegree--;
  }


  void SceneGraph::GridMeshNode::resetInDegree()
  {
    closed = false;
    if (indegree == 1)
      material->resetInDegree();
    indegree--;
  }

  void SceneGraph::TransformNode::resetInDegree()
  {
    closed = false;
    if (indegree == 1) {
      child->resetInDegree();
      if (spaces.size() > 1) child->resetInDegree(); // break instance up when motion blur is used
    }
    indegree--;
  }

  void SceneGraph::GroupNode::resetInDegree()
  {
    closed = false;
    if (indegree == 1) {
      for (auto&  c : children)
        c->resetInDegree();
    }
    indegree--;
  }
  
  void SceneGraph::TriangleMeshNode::verify() const
  {
    const size_t N = numVertices();
    if (normals.size() && normals.size() != positions.size())
       THROW_RUNTIME_ERROR("incompatible number of time steps");
    for (const auto& p : positions) 
      if (p.size() != N) 
        THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (const auto& n : normals) 
      if (n.size() && n.size() != N)
        THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    if (texcoords.size() && texcoords.size() != N) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (auto tri : triangles) {
      if (size_t(tri.v0) >= N || size_t(tri.v1) >= N || size_t(tri.v2) >= N)
        THROW_RUNTIME_ERROR("invalid triangle");
    }
  }

  void SceneGraph::QuadMeshNode::verify() const
  {
    const size_t N = numVertices();
    if (normals.size() && normals.size() != positions.size())
       THROW_RUNTIME_ERROR("incompatible number of time steps");
    for (const auto& p : positions) 
      if (p.size() != N) 
        THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (const auto& n : normals) 
      if (n.size() && n.size() != N)
        THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    if (texcoords.size() && texcoords.size() != N) THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (auto quad : quads) {
      if (size_t(quad.v0) >= N || size_t(quad.v1) >= N || size_t(quad.v2) >= N || size_t(quad.v3) >= N)
        THROW_RUNTIME_ERROR("invalid quad");
    }
  }

  void SceneGraph::GridMeshNode::verify() const
  {
    const size_t N = numVertices();
    for (const auto& p : positions) 
      if (p.size() != N) 
        THROW_RUNTIME_ERROR("incompatible vertex array sizes");
    for (auto grid : grids) {
      if (size_t(grid.startVtx) >= N || size_t(grid.lineStride) >= N || size_t(grid.resX) >= 0x7fff || size_t(grid.resY) >= 0x7fff)
        THROW_RUNTIME_ERROR("invalid grid");
    }
  }

  void SceneGraph::SubdivMeshNode::verify() const
  {
    const size_t N = numPositions();
    if (normals.size() && normals.size() != positions.size())
       THROW_RUNTIME_ERROR("incompatible number of time steps");
    for (const auto& p : positions) 
      if (p.size() != N) 
        THROW_RUNTIME_ERROR("incompatible position array sizes");
    for (auto i : position_indices) 
      if (size_t(i) >= N) THROW_RUNTIME_ERROR("invalid position index array");
    if (normal_indices.size())
      for (auto i : normal_indices) 
        if (size_t(i) >= normals[0].size()) THROW_RUNTIME_ERROR("invalid normal index array");
    for (auto i : texcoord_indices) 
      if (size_t(i) >= texcoords.size()) THROW_RUNTIME_ERROR("invalid texcoord index array");
    for (auto i : holes) 
      if (size_t(i) >= verticesPerFace.size()) THROW_RUNTIME_ERROR("invalid hole index array");
    for (auto crease : edge_creases) 
      if (max(size_t(crease.x),size_t(crease.y)) >= N) THROW_RUNTIME_ERROR("invalid edge crease array");
    if (edge_crease_weights.size() != edge_creases.size())
      THROW_RUNTIME_ERROR("invalid edge crease weight array");
    for (auto crease : vertex_creases) 
      if (size_t(crease) >= N) THROW_RUNTIME_ERROR("invalid vertex crease array");
    if (vertex_crease_weights.size() != vertex_creases.size())
      THROW_RUNTIME_ERROR("invalid vertex crease weight array");
  }

  void SceneGraph::HairSetNode::verify() const
  {
    const size_t N = numVertices();

    for (const auto& p : positions) 
      if (p.size() != N) 
        THROW_RUNTIME_ERROR("incompatible vertex array sizes");

    if (type == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE ||
        type == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE ||
        type == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE)
    {
      if (!normals.size())
        THROW_RUNTIME_ERROR("normal array required for oriented curve");

      for (const auto& n : normals) 
        if (n.size() != N) 
          THROW_RUNTIME_ERROR("incompatible normal array size");
    }
    else
    {
      if (normals.size())
        THROW_RUNTIME_ERROR("normal array not supported for this geometry type");
    }

    if (type == RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE ||
        type == RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE ||
        type == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE)
    {
      for (const auto& t : tangents) 
        if (t.size() != N) 
          THROW_RUNTIME_ERROR("incompatible tangent array size");
    }
    else
    {
      if (tangents.size())
        THROW_RUNTIME_ERROR("tangent array not supported for this geometry type");
    }

    if (type == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE)
    {
      if (!dnormals.size())
        THROW_RUNTIME_ERROR("normal derivative array required for oriented hermite curve");

      for (const auto& n : dnormals) 
        if (n.size() != N) 
          THROW_RUNTIME_ERROR("incompatible normal derivative array size");
    }
    else
    {
      if (dnormals.size())
        THROW_RUNTIME_ERROR("normal derivative array not supported for this geometry type");
    }

    if (type == RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE ||
        //type == RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE ||
        //type == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_LINEAR_CURVE ||
        type == RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE ||
        type == RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE ||
        type == RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE)
    {
      for (auto hair : hairs)
        if (size_t(hair.vertex+1) >= N)
          THROW_RUNTIME_ERROR("invalid hair");
    }
    else 
    {
      for (auto hair : hairs)
        if (size_t(hair.vertex+3) >= N)
          THROW_RUNTIME_ERROR("invalid hair");
    }
    
    if (flags.size() != 0 && flags.size() != hairs.size())
      THROW_RUNTIME_ERROR("size of flags array does not match size of curve array");
  }

  void SceneGraph::PointSetNode::verify() const
  {
    const size_t N = numVertices();

    for (const auto& p : positions)
      if (p.size() != N)
        THROW_RUNTIME_ERROR("incompatible vertex array sizes");

    if (type == RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT)
    {
      if (!normals.size())
        THROW_RUNTIME_ERROR("normal array required for oriented disc");

      for (const auto& n : normals)
        if (n.size() != N)
          THROW_RUNTIME_ERROR("incompatible normal array size");
    }
    else
    {
      if (normals.size())
        THROW_RUNTIME_ERROR("normal array not supported for this geometry type");
    }
  }

  avector<Vec3fa> bspline_to_bezier_helper(const std::vector<SceneGraph::HairSetNode::Hair>& indices, const avector<Vec3fa>& positions)
  {
    avector<Vec3fa> positions_o;
    positions_o.resize(4*indices.size());
    for (size_t i=0; i<indices.size(); i++) 
    {
      const size_t idx = indices[i].vertex;
      const vfloat4 v0 = vfloat4::loadu(&positions[idx+0]);  
      const vfloat4 v1 = vfloat4::loadu(&positions[idx+1]);
      const vfloat4 v2 = vfloat4::loadu(&positions[idx+2]);
      const vfloat4 v3 = vfloat4::loadu(&positions[idx+3]);
      positions_o[4*i+0] = Vec3fa((1.0f/6.0f)*v0 + (2.0f/3.0f)*v1 + (1.0f/6.0f)*v2);
      positions_o[4*i+1] = Vec3fa((2.0f/3.0f)*v1 + (1.0f/3.0f)*v2);
      positions_o[4*i+2] = Vec3fa((1.0f/3.0f)*v1 + (2.0f/3.0f)*v2);
      positions_o[4*i+3] = Vec3fa((1.0f/6.0f)*v1 + (2.0f/3.0f)*v2 + (1.0f/6.0f)*v3);
    }
    return positions_o;
  }

  avector<Vec3fa> bezier_to_bspline_helper(const std::vector<SceneGraph::HairSetNode::Hair>& indices, const avector<Vec3fa>& positions)
  {
    avector<Vec3fa> positions_o;
    positions_o.resize(4*indices.size());
    for (size_t i=0; i<indices.size(); i++) 
    {
      const size_t idx = indices[i].vertex;
      vfloat4 v0 = vfloat4::loadu(&positions[idx+0]);  
      vfloat4 v1 = vfloat4::loadu(&positions[idx+1]);
      vfloat4 v2 = vfloat4::loadu(&positions[idx+2]);
      vfloat4 v3 = vfloat4::loadu(&positions[idx+3]);
      positions_o[4*i+0] = Vec3fa( 6.0f*v0 - 7.0f*v1 + 2.0f*v2);
      positions_o[4*i+1] = Vec3fa( 2.0f*v1 - 1.0f*v2);
      positions_o[4*i+2] = Vec3fa(-1.0f*v1 + 2.0f*v2);
      positions_o[4*i+3] = Vec3fa( 2.0f*v1 - 7.0f*v2 + 6.0f*v3);
    }
    return positions_o;
  }

  std::pair<avector<Vec3fa>,avector<Vec3fa>> bezier_to_hermite_helper(const std::vector<SceneGraph::HairSetNode::Hair>& indices, const avector<Vec3fa>& positions)
  {
    avector<Vec3fa> positions_o; positions_o.resize(2*indices.size());
    avector<Vec3fa> tangents_o;  tangents_o.resize(2*indices.size());
    
    for (size_t i=0; i<indices.size(); i++) 
    {
      const size_t idx = indices[i].vertex;
      vfloat4 v0 = vfloat4::loadu(&positions[idx+0]);  
      vfloat4 v1 = vfloat4::loadu(&positions[idx+1]);
      vfloat4 v2 = vfloat4::loadu(&positions[idx+2]);
      vfloat4 v3 = vfloat4::loadu(&positions[idx+3]);
      positions_o[2*i+0] = Vec3fa(v0);
      positions_o[2*i+1] = Vec3fa(v3);
      tangents_o[2*i+0] = Vec3fa(3.0f*(v1-v0));
      tangents_o[2*i+1] = Vec3fa(3.0f*(v3-v2));
    }
    return std::make_pair(positions_o,tangents_o);
  }

  void SceneGraph::HairSetNode::convert_bezier_to_bspline()
  {
    if (type != RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE &&
        type != RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE)
      return;

    for (size_t i=0; i<positions.size(); i++) {
      positions[i] = bezier_to_bspline_helper(hairs,positions[i]);
    }
    for (size_t i=0; i<hairs.size(); i++) {
      hairs[i] = SceneGraph::HairSetNode::Hair(unsigned(4*i),0);
    }

    if (type == RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE)
      type = RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE;
    else
      type = RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE;
  }

  void SceneGraph::HairSetNode::convert_bezier_to_hermite()
  {
    if (type != RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE &&
        type != RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE)
      return;

    tangents.resize(numTimeSteps());
    for (size_t i=0; i<positions.size(); i++) {
      auto pt = bezier_to_hermite_helper(hairs,positions[i]);
      positions[i] = pt.first;
      tangents[i] = pt.second;
    }
    for (size_t i=0; i<hairs.size(); i++) {
      hairs[i] = SceneGraph::HairSetNode::Hair(unsigned(2*i),0);
    }

    if (type == RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE)
      type = RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE;
    else
      type = RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE;
  }

  void SceneGraph::HairSetNode::convert_bspline_to_bezier()
  {
    if (type != RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE &&
        type != RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE)
      return;

    for (size_t i=0; i<positions.size(); i++) {
      positions[i] = bspline_to_bezier_helper(hairs,positions[i]);
    }
    for (size_t i=0; i<hairs.size(); i++) {
      hairs[i] = SceneGraph::HairSetNode::Hair(unsigned(4*i),0);
    }

    if (type == RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE)
      type = RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE;
    else
      type = RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE;
  }

  bool test_location(const std::vector<avector<Vec3fa>>& in, ssize_t ipos, std::vector<avector<Vec3fa>>& out, ssize_t opos)
  {
    if (opos < 0) 
      return false;

    for (ssize_t i=ipos, j=opos; i<ipos+4 && j<(ssize_t)out[0].size(); i++, j++) {
      for (size_t k=0; k<in.size(); k++) {
        if (any(abs((vfloat4)in[k][i]-(vfloat4)out[k][j]) > 0.01f*(vfloat4)max(abs(in[k][i]),abs(out[k][j]))))
          return false;
      }
    }
    return true;
  }
  
  void SceneGraph::HairSetNode::compact_vertices()
  {
    std::vector<avector<Vec3fa>> positions_o(positions.size());
    for (size_t i=0; i<positions.size(); i++)
      positions_o.reserve(positions[i].size());
    
    for (size_t i=0; i<hairs.size(); i++) 
    {
      unsigned idx = hairs[i].vertex;
      if (test_location(positions,idx,positions_o,positions_o[0].size()-1)) 
      {
        hairs[i].vertex = (unsigned) positions_o[0].size()-1;
        for (size_t k=0; k<positions.size(); k++) {
          positions_o[k].push_back(positions[k][idx+1]);
          positions_o[k].push_back(positions[k][idx+2]);
          positions_o[k].push_back(positions[k][idx+3]);
        }
      }

      else if (test_location(positions,idx,positions_o,positions_o[0].size()-3)) 
      {
        hairs[i].vertex = (unsigned) positions_o[0].size()-3;
        for (size_t k=0; k<positions.size(); k++)
          positions_o[k].push_back(positions[k][idx+3]);
      }

      else
      {
        hairs[i].vertex = (unsigned) positions_o[0].size();
        for (size_t k=0; k<positions.size(); k++) {
          positions_o[k].push_back(positions[k][idx+0]);
          positions_o[k].push_back(positions[k][idx+1]);
          positions_o[k].push_back(positions[k][idx+2]);
          positions_o[k].push_back(positions[k][idx+3]);
        }
      }
    }

    for (auto& v : positions_o)
      v.shrink_to_fit();
   
    positions = std::move(positions_o);
  }

  void SceneGraph::extend_animation(Ref<SceneGraph::Node> node0, Ref<SceneGraph::Node> node1)
  {
    if (node0 == node1) return;
      
    if (Ref<SceneGraph::TransformNode> xfmNode0 = node0.dynamicCast<SceneGraph::TransformNode>()) 
    {
      if (Ref<SceneGraph::TransformNode> xfmNode1 = node1.dynamicCast<SceneGraph::TransformNode>()) 
      {
        xfmNode0->spaces.add(xfmNode1->spaces);
        extend_animation(xfmNode0->child, xfmNode1->child);
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
          extend_animation(groupNode0->children[i],groupNode1->children[i]);
      } 
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh0 = node0.dynamicCast<SceneGraph::TriangleMeshNode>()) 
    {
      if (Ref<SceneGraph::TriangleMeshNode> mesh1 = node1.dynamicCast<SceneGraph::TriangleMeshNode>())
      {
        if (mesh0->numVertices() != mesh1->numVertices())
          THROW_RUNTIME_ERROR("incompatible scene graph");

        for (auto& p : mesh1->positions)
          mesh0->positions.push_back(std::move(p));
      } 
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh0 = node0.dynamicCast<SceneGraph::QuadMeshNode>()) 
    {
      if (Ref<SceneGraph::QuadMeshNode> mesh1 = node1.dynamicCast<SceneGraph::QuadMeshNode>())
      {
        if (mesh0->numVertices() != mesh1->numVertices())
          THROW_RUNTIME_ERROR("incompatible scene graph");

        for (auto& p : mesh1->positions)
          mesh0->positions.push_back(std::move(p));
      } 
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
    else if (Ref<SceneGraph::HairSetNode> mesh0 = node0.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      if (Ref<SceneGraph::HairSetNode> mesh1 = node1.dynamicCast<SceneGraph::HairSetNode>()) 
      {
        if (mesh0->numVertices() != mesh1->numVertices())
          THROW_RUNTIME_ERROR("incompatible scene graph");

        for (auto& p : mesh1->positions)
          mesh0->positions.push_back(std::move(p));
      }
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
    else if (Ref<SceneGraph::PointSetNode> mesh0 = node0.dynamicCast<SceneGraph::PointSetNode>())
    {
      if (Ref<SceneGraph::PointSetNode> mesh1 = node1.dynamicCast<SceneGraph::PointSetNode>())
      {
        if (mesh0->numVertices() != mesh1->numVertices())
          THROW_RUNTIME_ERROR("incompatible scene graph");

        for (auto& p : mesh1->positions)
          mesh0->positions.push_back(std::move(p));
      }
      else THROW_RUNTIME_ERROR("incompatible scene graph");
    }
    else if (Ref<SceneGraph::SubdivMeshNode> mesh0 = node0.dynamicCast<SceneGraph::SubdivMeshNode>())
    {
      if (Ref<SceneGraph::SubdivMeshNode> mesh1 = node1.dynamicCast<SceneGraph::SubdivMeshNode>())
      {
        if (mesh0->numPositions() != mesh1->numPositions())
          THROW_RUNTIME_ERROR("incompatible scene graph");
        if (mesh0->verticesPerFace != mesh1->verticesPerFace)
          THROW_RUNTIME_ERROR("incompatible scene graph");

        for (auto& p : mesh1->positions)
          mesh0->positions.push_back(std::move(p));
      }
      else THROW_RUNTIME_ERROR("incompatible scene graph"); 
    }
  }

  void SceneGraph::optimize_animation(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) 
      optimize_animation(xfmNode->child);

    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (const auto& child : groupNode->children) 
        optimize_animation(child);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
    {
      bool equal = true;
      for (size_t i=1; i<mesh->numTimeSteps(); i++)
        equal &= mesh->positions[0] == mesh->positions[i];

      if (equal)
        mesh->positions.resize(1);
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) 
    {
      bool equal = true;
      for (size_t i=1; i<mesh->numTimeSteps(); i++)
        equal &= mesh->positions[0] == mesh->positions[i];

      if (equal)
        mesh->positions.resize(1);
    }
    else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      bool equal = true;
      for (size_t i=1; i<mesh->numTimeSteps(); i++)
        equal &= mesh->positions[0] == mesh->positions[i];

      if (equal)
        mesh->positions.resize(1);
    }
    else if (Ref<SceneGraph::PointSetNode> mesh = node.dynamicCast<SceneGraph::PointSetNode>())
    {
      bool equal = true;
      for (size_t i=1; i<mesh->numTimeSteps(); i++)
        equal &= mesh->positions[0] == mesh->positions[i];

      if (equal)
        mesh->positions.resize(1);
    }
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>())
    {
      bool equal = true;
      for (size_t i=1; i<mesh->numTimeSteps(); i++)
        equal &= mesh->positions[0] == mesh->positions[i];

      if (equal)
        mesh->positions.resize(1);
    }
  }

  void SceneGraph::set_motion_vector(Ref<SceneGraph::Node> node, const Vec3fa& dP)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      set_motion_vector(xfmNode->child, dP);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        set_motion_vector(groupNode->children[i],dP);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
    {
      avector<Vec3fa> positions1;
      for (auto P : mesh->positions.back()) 
        positions1.push_back(P+dP);
      mesh->positions.push_back(std::move(positions1));
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) 
    {
      avector<Vec3fa> positions1;
      for (auto P : mesh->positions.back()) 
        positions1.push_back(P+dP);
      mesh->positions.push_back(std::move(positions1));
    }
    else if (Ref<SceneGraph::GridMeshNode> mesh = node.dynamicCast<SceneGraph::GridMeshNode>()) 
    {
      avector<Vec3fa> positions1;
      for (auto P : mesh->positions.back()) 
        positions1.push_back(P+dP);
      mesh->positions.push_back(std::move(positions1));
    }
    else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      avector<Vec3fa> positions1;
      for (auto P : mesh->positions.back()) 
        positions1.push_back(P+dP);
      mesh->positions.push_back(std::move(positions1));
    }
    else if (Ref<SceneGraph::PointSetNode> mesh = node.dynamicCast<SceneGraph::PointSetNode>())
    {
      avector<Vec3fa> positions1;
      for (auto P : mesh->positions.back())
        positions1.push_back(P+dP);
      mesh->positions.push_back(std::move(positions1));

      if (mesh->normals.size())
        mesh->normals.push_back(mesh->normals[0]);
    }
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>())
    {
      avector<Vec3fa> positions1;
      for (auto P : mesh->positions.back())
        positions1.push_back(P+dP);
      mesh->positions.push_back(std::move(positions1));
    }
  }

  void SceneGraph::set_motion_vector(Ref<SceneGraph::Node> node, const avector<Vec3fa>& motion_vector)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      set_motion_vector(xfmNode->child,motion_vector);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        set_motion_vector(groupNode->children[i],motion_vector);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
    {
      avector<Vec3fa> positions = std::move(mesh->positions[0]);
      mesh->positions.clear();
      for (size_t t=0; t<motion_vector.size(); t++) {
        avector<Vec3fa> tpositions(positions.size());
        for (size_t i=0; i<positions.size(); i++) tpositions[i] = positions[i] + motion_vector[t];
        mesh->positions.push_back(std::move(tpositions));
      }
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) 
    {
      avector<Vec3fa> positions = std::move(mesh->positions[0]);
      mesh->positions.clear();
      for (size_t t=0; t<motion_vector.size(); t++) {
        avector<Vec3fa> tpositions(positions.size());
        for (size_t i=0; i<positions.size(); i++) tpositions[i] = positions[i] + motion_vector[t];
        mesh->positions.push_back(std::move(tpositions));
      }
    }
    else if (Ref<SceneGraph::GridMeshNode> mesh = node.dynamicCast<SceneGraph::GridMeshNode>()) 
    {
      avector<Vec3fa> positions = std::move(mesh->positions[0]);
      mesh->positions.clear();
      for (size_t t=0; t<motion_vector.size(); t++) {
        avector<Vec3fa> tpositions(positions.size());
        for (size_t i=0; i<positions.size(); i++) tpositions[i] = positions[i] + motion_vector[t];
        mesh->positions.push_back(std::move(tpositions));
      }
    }
    else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      avector<Vec3fa> positions = std::move(mesh->positions[0]);
      mesh->positions.clear();
      for (size_t t=0; t<motion_vector.size(); t++) {
        avector<Vec3fa> tpositions(positions.size());
        for (size_t i=0; i<positions.size(); i++) tpositions[i] = positions[i] + motion_vector[t];
        mesh->positions.push_back(std::move(tpositions));
      }
    }
    else if (Ref<SceneGraph::PointSetNode> mesh = node.dynamicCast<SceneGraph::PointSetNode>())
    {
      avector<Vec3fa> positions = std::move(mesh->positions[0]);
      mesh->positions.clear();
      for (size_t t=0; t<motion_vector.size(); t++) {
        avector<Vec3fa> tpositions(positions.size());
        for (size_t i=0; i<positions.size(); i++) tpositions[i] = positions[i] + motion_vector[t];
        mesh->positions.push_back(std::move(tpositions));
      }
      if (mesh->normals.size()) {
        for (size_t t=1; t<motion_vector.size(); t++)
          mesh->normals.push_back(mesh->normals[0]);
      }
    }
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>())
    {
      avector<Vec3fa> positions = std::move(mesh->positions[0]);
      mesh->positions.clear();
      for (size_t t=0; t<motion_vector.size(); t++) {
        avector<Vec3fa> tpositions(positions.size());
        for (size_t i=0; i<positions.size(); i++) tpositions[i] = positions[i] + motion_vector[t];
        mesh->positions.push_back(std::move(tpositions));
      }
    }
  }

  void SceneGraph::resize_randomly(RandomSampler& sampler, Ref<Node> node, const size_t N)
  {
     if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
       resize_randomly(sampler,xfmNode->child, N);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        resize_randomly(sampler,groupNode->children[i],N);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
    {
      if (!mesh->triangles.size()) return;
      for (size_t i=0; i<N; i++) {
        size_t j = RandomSampler_getInt(sampler)%(min(mesh->triangles.size(),N));
        if (i < mesh->triangles.size()) std::swap(mesh->triangles[i],mesh->triangles[j]);
        else                            mesh->triangles.push_back(mesh->triangles[j]);
      }
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) 
    {
      if (!mesh->quads.size()) return;
      for (size_t i=0; i<N; i++) {
        size_t j = RandomSampler_getInt(sampler)%(min(mesh->quads.size(),N));
        if (i < mesh->quads.size()) std::swap(mesh->quads[i],mesh->quads[j]);
        else                        mesh->quads.push_back(mesh->quads[j]);
      }
    }
    else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      if (!mesh->hairs.size()) return;
      for (size_t i=0; i<N; i++) {
        size_t j = RandomSampler_getInt(sampler)%(min(mesh->hairs.size(),N));
        if (i < mesh->hairs.size()) std::swap(mesh->hairs[i],mesh->hairs[j]);
        else                        mesh->hairs.push_back(mesh->hairs[j]);
      }
    }
    else if (Ref<SceneGraph::PointSetNode> mesh = node.dynamicCast<SceneGraph::PointSetNode>())
    {
      if (mesh->positions.size() <= N) return;
      mesh->positions.resize(N);
    }
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>())
    {
      if (mesh->verticesPerFace.size() <= N) return;
      mesh->verticesPerFace.resize(N);
    }
  }

  void SceneGraph::set_time_range(Ref<SceneGraph::Node> node, const BBox1f& time_range)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->spaces.time_range = time_range;
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        set_time_range(groupNode->children[i],time_range);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) {
      mesh->time_range = time_range;
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) {
      mesh->time_range = time_range;
    }
    else if (Ref<SceneGraph::GridMeshNode> mesh = node.dynamicCast<SceneGraph::GridMeshNode>()) {
      mesh->time_range = time_range;
    }
    else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) {
      mesh->time_range = time_range;
    }
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>()) {
      mesh->time_range = time_range;
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
  
  Ref<SceneGraph::Node> SceneGraph::convert_triangles_to_quads ( Ref<SceneGraph::TriangleMeshNode> tmesh )
  {
    Ref<SceneGraph::QuadMeshNode> qmesh = new SceneGraph::QuadMeshNode(tmesh->material,tmesh->time_range,0);

    for (auto& p : tmesh->positions)
      qmesh->positions.push_back(p);
    
    qmesh->normals = tmesh->normals;
    qmesh->texcoords = tmesh->texcoords;
    
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
    return qmesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::convert_triangles_to_quads(Ref<SceneGraph::Node> node, float prop)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = convert_triangles_to_quads(xfmNode->child,prop);
    } 
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = convert_triangles_to_quads(groupNode->children[i],prop);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> tmesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) {
      if (random<float>() <= prop) return convert_triangles_to_quads(tmesh);
      else                         return node;
    }
    return node;
  }

  Ref<SceneGraph::Node> SceneGraph::convert_quads_to_grids ( Ref<SceneGraph::QuadMeshNode> qmesh , const unsigned int resX, const unsigned int resY )
  {
    const size_t timeSteps = qmesh->positions.size();
    Ref<SceneGraph::GridMeshNode> gmesh = new SceneGraph::GridMeshNode(qmesh->material,qmesh->time_range,timeSteps);
    std::vector<SceneGraph::QuadMeshNode::Quad>& quads = qmesh->quads;

    for (size_t i=0;i<quads.size();i++)
    {
      const unsigned int startVtx = (unsigned int) gmesh->positions[0].size();
      const unsigned int lineStride = resX;
      for (size_t t=0;t<timeSteps;t++)
      {
        const SceneGraph::GridMeshNode::Vertex v00 = qmesh->positions[t][quads[i].v0];
        const SceneGraph::GridMeshNode::Vertex v01 = qmesh->positions[t][quads[i].v1];
        const SceneGraph::GridMeshNode::Vertex v10 = qmesh->positions[t][quads[i].v3];
        const SceneGraph::GridMeshNode::Vertex v11 = qmesh->positions[t][quads[i].v2];
        for (unsigned int y=0; y<resY; y++)
        {
          for (unsigned int x=0; x<resX; x++)
          {
            const float u = (float)x / (resX-1);
            const float v = (float)y / (resY-1);
            const SceneGraph::GridMeshNode::Vertex vtx = v00 * (1.0f-u) * (1.0f-v) + v01 * u * (1.0f-v) + v10 * (1.0f-u) * v + v11 * u * v;
            gmesh->positions[t].push_back( vtx );
          }
        }
      }
      assert(startVtx + resX * resY == gmesh->positions[0].size());
      gmesh->grids.push_back(SceneGraph::GridMeshNode::Grid(startVtx,lineStride,resX,resY));
    }
    return gmesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::convert_quads_to_grids(Ref<SceneGraph::Node> node, const unsigned int resX, const unsigned int resY )
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = convert_quads_to_grids(xfmNode->child, resX, resY);
    } 
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = convert_quads_to_grids(groupNode->children[i], resX, resY);
    }
    else if (Ref<SceneGraph::QuadMeshNode> qmesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) {
      return convert_quads_to_grids(qmesh, resX, resY);
    }
    return node;
  }

  Ref<SceneGraph::Node> SceneGraph::convert_grids_to_quads ( Ref<SceneGraph::GridMeshNode> gmesh )
  {
    Ref<SceneGraph::QuadMeshNode> qmesh = new SceneGraph::QuadMeshNode(gmesh->material,gmesh->time_range,0);

    for (size_t i=0; i<gmesh->numPrimitives(); i++)
    {
      const unsigned int startVtx = gmesh->grids[i].startVtx;
      const unsigned int lineStride = gmesh->grids[i].lineStride;
      const unsigned int resX = gmesh->grids[i].resX;
      const unsigned int resY = gmesh->grids[i].resY;

      for (unsigned int y=0; y<resY-1; y++)
      {
        for (unsigned int x=0; x<resX-1; x++)
        {
          const unsigned int a0 = startVtx + y * lineStride + x;
          const unsigned int a1 = a0 + 1;
          const unsigned int a3 = a0 + lineStride;
          const unsigned int a2 = a0 + lineStride + 1;
          qmesh->quads.push_back(SceneGraph::QuadMeshNode::Quad(a0,a1,a2,a3));
        }
      }
    }
    const size_t timeSteps = gmesh->positions.size();
    for (size_t t=0;t<timeSteps;t++)
      qmesh->positions.push_back(gmesh->positions[t]);
    return qmesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::convert_grids_to_quads(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = convert_grids_to_quads(xfmNode->child);
    } 
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = convert_grids_to_quads(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::GridMeshNode> gmesh = node.dynamicCast<SceneGraph::GridMeshNode>()) {
      return convert_grids_to_quads(gmesh);
    }
    return node;
  }
  
  bool extend_grid(RTCGeometry geom, std::vector<bool>& visited, std::deque<unsigned int>& left, std::deque<unsigned int>& top, std::deque<unsigned int>& right)
  {
    //        top
    // | <----------
    // |            /\ .
    // | left        | right 
    // |             |
    // \/            |

    
    /* test if all neighboring faces of top exist and are properly connected */
    unsigned int prev_opposite_edge = -1;
    for (size_t i=0; i<top.size(); i++)
    {
      const unsigned int edge = top[i];
      const unsigned int opposite_edge = rtcGetGeometryOppositeHalfEdge(geom,0,edge);
      if (opposite_edge == edge) return false;
      const unsigned int opposite_face = rtcGetGeometryFace(geom,opposite_edge);
      if (visited[opposite_face]) return false;

      /* test if we share an edge with the last quad */
      if (i > 0) {
        const unsigned int border0 = rtcGetGeometryOppositeHalfEdge(geom,0,rtcGetGeometryPreviousHalfEdge(geom,prev_opposite_edge));
        const unsigned int border1 = rtcGetGeometryNextHalfEdge(geom,opposite_edge);
        if (border0 != border1) return false;
      }
      prev_opposite_edge = opposite_edge;
    }

    /* extend border edges */
    for (size_t i=0; i<top.size(); i++)
    {
      const unsigned int edge = top[i];
      const unsigned int opposite_edge = rtcGetGeometryOppositeHalfEdge(geom,0,edge);
      assert(opposite_edge != edge);
      const unsigned int opposite_face = rtcGetGeometryFace(geom,opposite_edge);
      assert(!visited[opposite_face]);
      visited[opposite_face] = true;
      unsigned int next_edge = opposite_edge;
      next_edge = rtcGetGeometryNextHalfEdge(geom,next_edge);
      if (i == 0) right.push_back(next_edge);
      next_edge = rtcGetGeometryNextHalfEdge(geom,next_edge);
      top[i] = next_edge;
      next_edge = rtcGetGeometryNextHalfEdge(geom,next_edge);
      if (i == top.size()-1) left.push_front(next_edge);
    }

    return true;
  }

  void gather_grid(RTCGeometry geom, avector<Vec3fa>& positions, size_t width, size_t height, unsigned int* indices, avector<Vec3fa>& vertices, unsigned int edgey)
  {
    //        
    // | <----------    <----------
    // |            /\ |           /\ .
    // | edgey       | |           |
    // |             | |           |
    // \/            | \/          |
    
    /* gather all rows */
    size_t y=0;
    for (; y<height; y++)
    {
      /* here edgey points from top/left vertex downwards */
      unsigned int edgex = edgey;

      /* gather all columns */
      size_t x=0;
      for (; x<width; x++)
      {
        /* here edgex points from left vertex of row downwards */
        positions[y*(width+1)+x] = vertices[indices[edgex]];

        /* prev -> prev -> opposite moves to the next column (unless we reach the right end) */
        edgex = rtcGetGeometryPreviousHalfEdge(geom,edgex);
        if (x+1 < width) {
          edgex = rtcGetGeometryPreviousHalfEdge(geom,edgex);
          edgex = rtcGetGeometryOppositeHalfEdge(geom,0,edgex);
        }
      }
      /* load rightmost vertex */
      positions[y*(width+1)+x] = vertices[indices[edgex]];

      /* next -> opposite -> next moves to next row (unless we reach the bottom) */
      edgey = rtcGetGeometryNextHalfEdge(geom,edgey);
      if (y+1 < height) {
        edgey = rtcGetGeometryOppositeHalfEdge(geom,0,edgey);
        edgey = rtcGetGeometryNextHalfEdge(geom,edgey);
      }
    }

    /* special treatment for last row, edgy points from the bottom/left vertex to the right */
    unsigned int edgex = edgey;
    for (size_t x=0; x<width; x++)
    {
      positions[y*(width+1)+x] = vertices[indices[edgex]];

      /* next -> opposite -> next moves to the next column (unless we reach the right end) */
      edgex = rtcGetGeometryNextHalfEdge(geom,edgex);
      if (x+1 < width) {
        edgex = rtcGetGeometryOppositeHalfEdge(geom,0,edgex);
        edgex = rtcGetGeometryNextHalfEdge(geom,edgex);
      }
    }
    /* load rightmost vertex */
    positions[height*(width+1)+width] = vertices[indices[edgex]];
  }

  Ref<SceneGraph::Node> SceneGraph::my_merge_quads_to_grids(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = my_merge_quads_to_grids(xfmNode->child);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = my_merge_quads_to_grids(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::QuadMeshNode> qmesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) 
    {
      Ref<SceneGraph::GridMeshNode> gmesh = new SceneGraph::GridMeshNode(qmesh->material,qmesh->time_range,qmesh->numTimeSteps());
      
      std::vector<bool> visited;
      visited.resize(qmesh->numPrimitives());
      for (size_t i=0; i<visited.size(); i++) visited[i] = false;
      std::vector<unsigned int> faces(qmesh->numPrimitives());
      for (size_t i=0; i<faces.size(); i++) faces[i] = 4;

      /* create temporary subdiv mesh to get access to mesh topology */
      RTCGeometry geom = rtcNewGeometry(g_device,RTC_GEOMETRY_TYPE_SUBDIVISION);
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,   0, RTC_FORMAT_UINT,   faces.data(), 0, sizeof(unsigned int), qmesh->numPrimitives());
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT,   qmesh->quads.data(), 0, sizeof(unsigned int), 4*qmesh->numPrimitives());
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, qmesh->positions[0].data(), 0, sizeof(Vec3fa), qmesh->numVertices());
      rtcCommitGeometry(geom);

      /* iterate over mesh and collect all grids */
      for (unsigned int i=0; i<qmesh->numPrimitives(); i++)
      {
        /* skip face if already added to some grid */
        if (visited[i]) continue;
        visited[i] = true;

        /* initialize grid with start quad */
        unsigned int edge = rtcGetGeometryFirstHalfEdge(geom,i);
        std::deque<unsigned int> left, right, top, bottom;
        left.push_back(edge);   edge = rtcGetGeometryNextHalfEdge(geom,edge);
        bottom.push_back(edge); edge = rtcGetGeometryNextHalfEdge(geom,edge);
        right.push_back(edge);  edge = rtcGetGeometryNextHalfEdge(geom,edge);
        top.push_back(edge);    edge = rtcGetGeometryNextHalfEdge(geom,edge);
        assert(edge == rtcGetGeometryFirstHalfEdge(geom,i));
        
        /* extend grid unless no longer possible */
        unsigned int width = 1;
        unsigned int height = 1;
        while (true)
        {
          const bool extended_top    = extend_grid(geom,visited,left,top,right);
          const bool extended_right  = extend_grid(geom,visited,top,right,bottom);
          const bool extended_bottom = extend_grid(geom,visited,right,bottom,left);
          const bool extended_left   = extend_grid(geom,visited,bottom,left,top);
          width  += extended_left + extended_right;
          height += extended_top  + extended_bottom;
          if (!extended_top && !extended_right && !extended_bottom && !extended_left) break;
          if (width+2  > SceneGraph::GridMeshNode::GRID_RES_MAX) break;
          if (height+2 > SceneGraph::GridMeshNode::GRID_RES_MAX) break;
        }
        
        /* add new grid to grid mesh */
        unsigned int startVertex = (unsigned int) gmesh->positions[0].size();
        gmesh->grids.push_back(SceneGraph::GridMeshNode::Grid(startVertex,width+1,width+1,height+1));

        /* gather all vertices of grid */
        for (size_t t=0; t<qmesh->numTimeSteps(); t++)
        {
          avector<Vec3fa> positions;
          positions.resize((width+1)*(height+1));
          gather_grid(geom,positions,width,height,(unsigned int*)qmesh->quads.data(), qmesh->positions[t], left.front());
          for (size_t i=0; i<positions.size(); i++)
            gmesh->positions[t].push_back(positions[i]);
        }
      }

      rtcReleaseGeometry(geom);

      return gmesh.dynamicCast<SceneGraph::Node>();
    }
    return node;
  }
      
  Ref<SceneGraph::Node> SceneGraph::convert_quads_to_subdivs(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = convert_quads_to_subdivs(xfmNode->child);
    } 
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = convert_quads_to_subdivs(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::QuadMeshNode> tmesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) 
    {
      Ref<SceneGraph::SubdivMeshNode> smesh = new SceneGraph::SubdivMeshNode(tmesh->material,tmesh->time_range,0);

      for (auto& p : tmesh->positions)
        smesh->positions.push_back(p);

      for (size_t i=0; i<tmesh->quads.size(); i++) {
        smesh->position_indices.push_back(tmesh->quads[i].v0);
        smesh->position_indices.push_back(tmesh->quads[i].v1);
        smesh->position_indices.push_back(tmesh->quads[i].v2);
        if (tmesh->quads[i].v2 != tmesh->quads[i].v3)
          smesh->position_indices.push_back(tmesh->quads[i].v3);
      }
      
      smesh->normals = tmesh->normals;
      if (smesh->normals.size())
        smesh->normal_indices = smesh->position_indices;
      
      smesh->texcoords = tmesh->texcoords;
      if (smesh->texcoords.size())
        smesh->texcoord_indices = smesh->position_indices;
      
      for (size_t i=0; i<tmesh->quads.size(); i++) 
        smesh->verticesPerFace.push_back(3 + (int)(tmesh->quads[i].v2 != tmesh->quads[i].v3));

      return smesh.dynamicCast<SceneGraph::Node>();
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
      Ref<SceneGraph::HairSetNode> lmesh = new SceneGraph::HairSetNode(RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE, hmesh->material, hmesh->time_range, 0);

      for (auto& p : hmesh->positions)
        lmesh->positions.push_back(p);

      for (auto hair : hmesh->hairs) {
        lmesh->hairs.push_back(SceneGraph::HairSetNode::Hair(hair.vertex+0,hair.id));
        lmesh->hairs.push_back(SceneGraph::HairSetNode::Hair(hair.vertex+1,hair.id));
        lmesh->hairs.push_back(SceneGraph::HairSetNode::Hair(hair.vertex+2,hair.id));
      }
      return lmesh.dynamicCast<SceneGraph::Node>();
    }
    return node;
  }

  Ref<SceneGraph::Node> SceneGraph::convert_flat_to_round_curves(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = convert_flat_to_round_curves(xfmNode->child);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = convert_flat_to_round_curves(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::HairSetNode> hmesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      //if (hmesh->type == RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE) // FIXME: not supported yet
      //  hmesh->type = RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE;
      //else
      if (hmesh->type == RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE)
        hmesh->type = RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE;
      else if (hmesh->type == RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE)
        hmesh->type = RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE;

      return hmesh.dynamicCast<SceneGraph::Node>();
    }
    return node;
  }

   Ref<SceneGraph::Node> SceneGraph::convert_round_to_flat_curves(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      xfmNode->child = convert_round_to_flat_curves(xfmNode->child);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        groupNode->children[i] = convert_round_to_flat_curves(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::HairSetNode> hmesh = node.dynamicCast<SceneGraph::HairSetNode>()) 
    {
      //if (hmesh->type == RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE) // FIXME: not supported yet
      //  hmesh->type = RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE;
      //else
      if (hmesh->type == RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE)
        hmesh->type = RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE;
      else if (hmesh->type == RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE)
        hmesh->type = RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE;

      return hmesh.dynamicCast<SceneGraph::Node>();
    }
    return node;
  }

  Ref<SceneGraph::Node> SceneGraph::convert_bezier_to_bspline(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      convert_bezier_to_bspline(xfmNode->child);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        convert_bezier_to_bspline(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::HairSetNode> hmesh = node.dynamicCast<SceneGraph::HairSetNode>()) {
      hmesh->convert_bezier_to_bspline();
      //hmesh->compact_vertices();
    }
    return node;
  }

  Ref<SceneGraph::Node> SceneGraph::convert_bspline_to_bezier(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      convert_bspline_to_bezier(xfmNode->child);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        convert_bspline_to_bezier(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::HairSetNode> hmesh = node.dynamicCast<SceneGraph::HairSetNode>()) {
      hmesh->convert_bspline_to_bezier();
    }
    return node;
  }

  Ref<SceneGraph::Node> SceneGraph::convert_bezier_to_hermite(Ref<SceneGraph::Node> node)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      convert_bezier_to_hermite(xfmNode->child);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        convert_bezier_to_hermite(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::HairSetNode> hmesh = node.dynamicCast<SceneGraph::HairSetNode>()) {
      hmesh->convert_bezier_to_hermite();
      //hmesh->compact_vertices();
    }
    return node;
  }

  Ref<SceneGraph::Node> SceneGraph::remove_mblur(Ref<SceneGraph::Node> node, bool mblur)
  {
    if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
      if (mblur) {
        if (xfmNode->spaces.size() > 1)
          return nullptr;
      } else {
        if (xfmNode->spaces.size() > 1)
          return node;
      }
      xfmNode->child = remove_mblur(xfmNode->child, mblur);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>())
    {
      for (size_t i=0; i<groupNode->children.size(); i++)
        groupNode->children[i] = remove_mblur(groupNode->children[i], mblur);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>())
    {
      if ((mesh->numTimeSteps() > 1) == mblur)
        return nullptr;
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>())
    {
      if ((mesh->numTimeSteps() > 1) == mblur)
        return nullptr;
    }
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>())
    {
      if ((mesh->numTimeSteps() > 1) == mblur)
        return nullptr;
    }
    else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>())
    {
      if ((mesh->numTimeSteps() > 1) == mblur)
        return nullptr;
    }
    else if (Ref<SceneGraph::PointSetNode> mesh = node.dynamicCast<SceneGraph::PointSetNode>())
    {
      if ((mesh->numTimeSteps() > 1) == mblur)
        return nullptr;
    }
    return node;
  }

  void SceneGraph::convert_mblur_to_nonmblur(Ref<Node> node)
  {
     if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
       xfmNode->spaces.spaces.resize(1);
       convert_mblur_to_nonmblur(xfmNode->child);
    }
    else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) 
    {
      for (size_t i=0; i<groupNode->children.size(); i++) 
        convert_mblur_to_nonmblur(groupNode->children[i]);
    }
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) {
      if (mesh->positions.size()) mesh->positions.resize(1);
      if (mesh->normals.size())   mesh->normals.resize(1);
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) {
      if (mesh->positions.size()) mesh->positions.resize(1);
      if (mesh->normals.size())   mesh->normals.resize(1);
    }
    else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) {
      if (mesh->positions.size()) mesh->positions.resize(1);
      if (mesh->normals.size())   mesh->normals.resize(1);
      if (mesh->tangents.size())  mesh->tangents.resize(1);
      if (mesh->dnormals.size())  mesh->dnormals.resize(1);
    }
    else if (Ref<SceneGraph::PointSetNode> mesh = node.dynamicCast<SceneGraph::PointSetNode>()) {
      if (mesh->positions.size()) mesh->positions.resize(1);
      if (mesh->normals.size())   mesh->normals.resize(1);
    }
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>()) {
      if (mesh->positions.size()) mesh->positions.resize(1);
      if (mesh->normals .size()) mesh->normals .resize(1);
    }
    else if (Ref<SceneGraph::GridMeshNode> mesh = node.dynamicCast<SceneGraph::GridMeshNode>()) {
      mesh->positions.resize(1);
    }
  }

  struct SceneGraphFlattener
  {
    Ref<SceneGraph::Node> node;
    std::map<Ref<SceneGraph::Node>,Ref<SceneGraph::Node>> object_mapping;
    std::map<std::string,int> unique_id;
    
    SceneGraphFlattener (Ref<SceneGraph::Node> in, SceneGraph::InstancingMode instancing)
    {
       in->calculateInDegree();
       in->calculateClosed(instancing == SceneGraph::INSTANCING_GROUP);
               
      std::vector<Ref<SceneGraph::Node>> geometries;      
      if (instancing != SceneGraph::INSTANCING_NONE) 
      {
        if (instancing == SceneGraph::INSTANCING_FLATTENED) convertFlattenedInstances(geometries,in);
        else                                                convertInstances(geometries,in,one);
        convertLightsAndCameras(geometries,in,one);
      }
      else
      {
        convertGeometries(geometries,in,one);
        convertLightsAndCameras(geometries,in,one);
      }
      in->resetInDegree();
      
      node = new SceneGraph::GroupNode(geometries);
    }

    std::string makeUniqueID(std::string id) 
    {
      if (id == "") id = "camera";
      std::map<std::string,int>::iterator i = unique_id.find(id);
      if (i == unique_id.end()) {
        unique_id[id] = 0;
        return id;
      }
      else {
        int n = ++unique_id[id];
        return id + "_" + toString(n);
      }
    }

    void convertLightsAndCameras(std::vector<Ref<SceneGraph::Node>>& group, const Ref<SceneGraph::Node>& node, const SceneGraph::Transformations& spaces)
    {
      if (!node->hasLightOrCamera) return;
      
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertLightsAndCameras(group,xfmNode->child, spaces*xfmNode->spaces);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (const auto& child : groupNode->children) convertLightsAndCameras(group,child,spaces);
      }
      else if (Ref<SceneGraph::LightNode> lightNode = node.dynamicCast<SceneGraph::LightNode>()) {
        group.push_back(new SceneGraph::LightNode(lightNode->light->transform(spaces[0])));
      }
      else if (Ref<SceneGraph::PerspectiveCameraNode> cameraNode = node.dynamicCast<SceneGraph::PerspectiveCameraNode>()) {
        group.push_back(new SceneGraph::PerspectiveCameraNode(cameraNode,spaces[0],makeUniqueID(cameraNode->name)));
      }
    }

    void convertGeometries(std::vector<Ref<SceneGraph::Node>>& group, const Ref<SceneGraph::Node>& node, const SceneGraph::Transformations& spaces)
    {
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertGeometries(group,xfmNode->child, spaces*xfmNode->spaces);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (const auto& child : groupNode->children) convertGeometries(group,child,spaces);
      }
      else if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) {
        group.push_back(new SceneGraph::TriangleMeshNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>()) {
        group.push_back(new SceneGraph::QuadMeshNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::GridMeshNode> mesh = node.dynamicCast<SceneGraph::GridMeshNode>()) {
        group.push_back(new SceneGraph::GridMeshNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>()) {
        group.push_back(new SceneGraph::SubdivMeshNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>()) {
        group.push_back(new SceneGraph::HairSetNode(mesh,spaces));
      }
      else if (Ref<SceneGraph::PointSetNode> mesh = node.dynamicCast<SceneGraph::PointSetNode>()) {
        group.push_back(new SceneGraph::PointSetNode(mesh,spaces));
      }
    }

    Ref<SceneGraph::Node> lookupGeometries(Ref<SceneGraph::Node> node)
    {
      if (object_mapping.find(node) == object_mapping.end())
      {
        std::vector<Ref<SceneGraph::Node>> geometries;
        convertGeometries(geometries,node,one);
        object_mapping[node] = new SceneGraph::GroupNode(geometries);
      }
      
      return object_mapping[node];
    }

    void convertInstances(std::vector<Ref<SceneGraph::Node>>& group, const Ref<SceneGraph::Node>& node, const SceneGraph::Transformations& spaces)
    {
      if (node->isClosed()) {
        //if (group.size() % 10000 == 0) std::cout << "." << std::flush;
        group.push_back(new SceneGraph::TransformNode(spaces,lookupGeometries(node)));
      }
      else if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        convertInstances(group,xfmNode->child, spaces*xfmNode->spaces);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (const auto& child : groupNode->children) convertInstances(group,child,spaces);
      }
    }

    void convertFlattenedInstances(std::vector<Ref<SceneGraph::Node>>& group, const Ref<SceneGraph::Node>& node)
    {
      if (Ref<SceneGraph::TransformNode> xfmNode = node.dynamicCast<SceneGraph::TransformNode>()) {
        group.push_back(node);
      } 
      else if (Ref<SceneGraph::GroupNode> groupNode = node.dynamicCast<SceneGraph::GroupNode>()) {
        for (const auto& child : groupNode->children) convertFlattenedInstances(group,child);
      }
    }
  };

  Ref<SceneGraph::Node> SceneGraph::flatten(Ref<Node> node, InstancingMode mode) {
    return SceneGraphFlattener(node,mode).node;
  }

  Ref<SceneGraph::GroupNode> SceneGraph::flatten(Ref<SceneGraph::GroupNode> node, SceneGraph::InstancingMode mode) {
    return flatten(node.dynamicCast<SceneGraph::Node>(),mode).dynamicCast<SceneGraph::GroupNode>();
  }
}
