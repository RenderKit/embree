// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
    SHADER_UV,
    SHADER_TEXCOORDS,
    SHADER_TEXCOORDS_GRID,
    SHADER_NG,
    SHADER_GEOMID,
    SHADER_GEOMID_PRIMID,
    SHADER_AMBIENT_OCCLUSION
  };

  /*! Scene representing the OBJ file */
  struct TutorialScene
  {
    TutorialScene () {}

    enum InstancingMode { INSTANCING_NONE, INSTANCING_GEOMETRY, INSTANCING_SCENE_GEOMETRY, INSTANCING_SCENE_GROUP };
    void add (Ref<SceneGraph::Node> node, InstancingMode instancing);

    /*! OBJ Triangle */
    struct Triangle 
    {
    public:
      Triangle () {}
      
      Triangle (const Triangle& other) 
      : v0(other.v0), v1(other.v1), v2(other.v2), materialID(other.materialID) {}

      Triangle (unsigned v0, unsigned v1, unsigned v2, unsigned materialID) 
      : v0(v0), v1(v1), v2(v2), materialID(materialID) {}

    public:
      unsigned v0, v1, v2, materialID;
    };

    /*! OBJ Quad */
    struct Quad 
    {
    public:
      Quad () {}

      Quad (unsigned v0, unsigned v1, unsigned v2, unsigned v3) 
      : v0(v0), v1(v1), v2(v2), v3(v3) {}

    public:
      unsigned v0, v1, v2, v3;
    };

    struct Hair 
    {
    public:
      Hair () {}
      Hair (unsigned vertex, unsigned id)
      : vertex(vertex), id(id) {}
    public:
      unsigned vertex,id;  //!< index of first control point and hair ID
    };

    struct Geometry : public RefCount
    {
      enum Type { TRIANGLE_MESH, SUBDIV_MESH, HAIR_SET, INSTANCE, GROUP, QUAD_MESH, LINE_SEGMENTS, CURVES };
      Type type;

      Geometry (Type type) : type(type) {}
    };

    /*! Triangle Mesh. */
    struct TriangleMesh : public Geometry
    {
      TriangleMesh (avector<Vec3fa>& positions, avector<Vec3fa>& normals, std::vector<Vec2f>& texcoords, std::vector<Triangle>& triangles, unsigned materialID)
        : Geometry(TRIANGLE_MESH), positions(positions), normals(normals), texcoords(texcoords), triangles(triangles), numTimeSteps(1), numVertices((unsigned int)positions.size()), materialID(materialID) {}

      TriangleMesh (Ref<SceneGraph::TriangleMeshNode> mesh, const SceneGraph::Transformations& spaces, unsigned materialID)
        : Geometry(TRIANGLE_MESH), numTimeSteps((unsigned int)mesh->numTimeSteps()), numVertices((unsigned int)mesh->numVertices()), materialID(materialID)
      {
        positions.resize(numTimeSteps*numVertices); 
        for (size_t t=0; t<numTimeSteps; t++) {
          float time = numTimeSteps > 1 ? float(t)/float(numTimeSteps-1) : 0.0f;
          const AffineSpace3fa space = spaces.interpolate(time);
          for (size_t i=0; i<numVertices; i++) 
            positions[t*numVertices+i] = xfmPoint (space,mesh->positions[t][i]);
        }

        const LinearSpace3fa nspace0 = rcp(spaces[0].l).transposed();
        normals.resize(mesh->normals.size()); 
        for (size_t i=0; i<mesh->normals.size(); i++) 
          normals[i] = xfmVector(nspace0,mesh->normals[i]);

        texcoords = mesh->texcoords;
        
        triangles.resize(mesh->triangles.size());
        for (size_t i=0; i<mesh->triangles.size(); i++) {
          SceneGraph::TriangleMeshNode::Triangle& tri = mesh->triangles[i];
          triangles[i] = TutorialScene::Triangle(tri.v0,tri.v1,tri.v2,materialID);
        }
      }

    public:
      avector<Vec3fa> positions;        //!< vertex positions for all time steps
      avector<Vec3fa> normals;          //!< vertex normals
      std::vector<Vec2f> texcoords;     //!< texture coordinates
      std::vector<Triangle> triangles;  //!< triangle indices

      unsigned numTimeSteps;
      unsigned numVertices;
      unsigned materialID;
    };

    /*! Quad Mesh. */
    struct QuadMesh : public Geometry
    {
      QuadMesh (Ref<SceneGraph::QuadMeshNode> mesh, const SceneGraph::Transformations& spaces, unsigned materialID)
		  : Geometry(QUAD_MESH), numTimeSteps((unsigned int)mesh->numTimeSteps()), numVertices((unsigned int)mesh->numVertices()), materialID(materialID)
      {
        positions.resize(numTimeSteps*numVertices); 
        for (size_t t=0; t<numTimeSteps; t++) {
          float time = numTimeSteps > 1 ? float(t)/float(numTimeSteps-1) : 0.0f;
          const AffineSpace3fa space = spaces.interpolate(time);
          for (size_t i=0; i<numVertices; i++) 
            positions[t*numVertices+i] = xfmPoint (space,mesh->positions[t][i]);
        }

        const LinearSpace3fa nspace0 = rcp(spaces[0].l).transposed();
        normals.resize(mesh->normals.size()); 
        for (size_t i=0; i<mesh->normals.size(); i++) 
          normals[i] = xfmVector(nspace0,mesh->normals[i]);

        texcoords = mesh->texcoords;
        
        quads.resize(mesh->quads.size());
        for (size_t i=0; i<mesh->quads.size(); i++) {
          SceneGraph::QuadMeshNode::Quad& quad = mesh->quads[i];
          quads[i] = TutorialScene::Quad(quad.v0,quad.v1,quad.v2,quad.v3);
        }
      }

    public:
      avector<Vec3fa> positions;       //!< vertex positions for all time steps
      avector<Vec3fa> normals;         //!< vertex normals
      std::vector<Vec2f> texcoords;    //!< texture coordinates
      std::vector<Quad> quads;         //!< quad indices

      unsigned numTimeSteps;
      unsigned numVertices;
      unsigned materialID;
    };

    /*! Subdivision Mesh. */
    struct SubdivMesh : public Geometry
    {
      SubdivMesh (Ref<SceneGraph::SubdivMeshNode> mesh, const SceneGraph::Transformations& spaces, unsigned materialID)
        : Geometry(SUBDIV_MESH), 
           position_subdiv_mode(mesh->position_subdiv_mode),
           normal_subdiv_mode(mesh->normal_subdiv_mode),
           texcoord_subdiv_mode(mesh->texcoord_subdiv_mode),
           numTimeSteps((unsigned int)mesh->numTimeSteps()), numPositions((unsigned int)mesh->numPositions()), materialID(materialID)
      {
        positions.resize(numTimeSteps*numPositions); 
        for (size_t t=0; t<numTimeSteps; t++) {
          float time = numTimeSteps > 1 ? float(t)/float(numTimeSteps-1) : 0.0f;
          const AffineSpace3fa space = spaces.interpolate(time);
          for (size_t i=0; i<numPositions; i++) 
            positions[t*numPositions+i] = xfmPoint (space,mesh->positions[t][i]);
        }

        const LinearSpace3fa nspace0 = rcp(spaces[0].l).transposed();
        normals.resize(mesh->normals.size()); 
        for (size_t i=0; i<mesh->normals.size(); i++) 
          normals[i] = xfmVector(nspace0,mesh->normals[i]);
        
        texcoords = mesh->texcoords;
        if (texcoords.size()) { // zero pad to 16 bytes
          texcoords.reserve(texcoords.size()+1);
          texcoords.data()[texcoords.size()] = zero;
        }
        position_indices = mesh->position_indices;
        normal_indices = mesh->normal_indices;
        texcoord_indices = mesh->texcoord_indices;
        verticesPerFace = mesh->verticesPerFace;
        holes = mesh->holes;
        edge_creases = mesh->edge_creases;
        edge_crease_weights = mesh->edge_crease_weights;
        vertex_creases = mesh->vertex_creases;
        vertex_crease_weights = mesh->vertex_crease_weights;
      }

    public:
      avector<Vec3fa> positions;                //!< vertex positions for all timesteps
      avector<Vec3fa> normals;                  //!< face vertex normals
      std::vector<Vec2f> texcoords;             //!< face texture coordinates
      std::vector<unsigned> position_indices;   //!< position indices for all faces
      std::vector<unsigned> normal_indices;     //!< normal indices for all faces
      std::vector<unsigned> texcoord_indices;   //!< texcoord indices for all faces
      RTCSubdivisionMode position_subdiv_mode;  
      RTCSubdivisionMode normal_subdiv_mode;
      RTCSubdivisionMode texcoord_subdiv_mode;
      std::vector<unsigned> verticesPerFace;    //!< number of indices of each face
      std::vector<unsigned> holes;              //!< face ID of holes
      std::vector<Vec2i> edge_creases;          //!< index pairs for edge crease 
      std::vector<float> edge_crease_weights;   //!< weight for each edge crease
      std::vector<unsigned> vertex_creases;     //!< indices of vertex creases
      std::vector<float> vertex_crease_weights; //!< weight for each vertex crease

      unsigned numTimeSteps;
      unsigned numPositions;
      unsigned materialID;
    };

    /*! Line segments. */
    struct LineSegments : public Geometry
    {
      LineSegments (Ref<SceneGraph::LineSegmentsNode> mesh, const SceneGraph::Transformations& spaces, unsigned materialID)
		  : Geometry(LINE_SEGMENTS), numTimeSteps((unsigned int)mesh->numTimeSteps()), numVertices((unsigned int)mesh->numVertices()), materialID(materialID)
      {
        positions.resize(numTimeSteps*numVertices); 
        for (size_t t=0; t<numTimeSteps; t++) {
          float time = numTimeSteps > 1 ? float(t)/float(numTimeSteps-1) : 0.0f;
          const AffineSpace3fa space = spaces.interpolate(time);
          for (size_t i=0; i<numVertices; i++) {
            positions[t*numVertices+i] = xfmPoint (space,mesh->positions[t][i]);
            positions[t*numVertices+i].w = mesh->positions[t][i].w;
          }
        }

        indices.resize(mesh->indices.size()); 
        for (size_t i=0; i<mesh->indices.size(); i++)
          indices[i] = mesh->indices[i];
      }

    public:
      avector<Vec3fa> positions;        //!< control points (x,y,z,r) for all timesteps
      std::vector<unsigned> indices;    //!< index buffer

      unsigned numTimeSteps;
      unsigned numVertices;
      unsigned materialID;
    };

    /*! Hair Set. */
    struct HairSet : public Geometry
    {
      HairSet (avector<Vec3fa>& positions, std::vector<Hair>& hairs, unsigned materialID, bool hair)
        : Geometry(hair ? HAIR_SET : CURVES), positions(positions), hairs(hairs), numTimeSteps(1), numVertices((unsigned int)positions.size()), materialID(materialID), tessellation_rate(4) {}
      
      HairSet (Ref<SceneGraph::HairSetNode> mesh, const SceneGraph::Transformations& spaces, unsigned materialID)
        : Geometry(mesh->hair ? HAIR_SET : CURVES), numTimeSteps((unsigned int)mesh->numTimeSteps()), numVertices((unsigned int)mesh->numVertices()), materialID(materialID), tessellation_rate(mesh->tessellation_rate)
      {
        positions.resize(numTimeSteps*numVertices); 
        for (size_t t=0; t<numTimeSteps; t++) {
          float time = numTimeSteps > 1 ? float(t)/float(numTimeSteps-1) : 0.0f;
          const AffineSpace3fa space = spaces.interpolate(time);
          for (size_t i=0; i<numVertices; i++) {
            positions[t*numVertices+i] = xfmPoint (space,mesh->positions[t][i]);
            positions[t*numVertices+i].w = mesh->positions[t][i].w;
          }
        }
        
        hairs.resize(mesh->hairs.size()); 
        for (size_t i=0; i<mesh->hairs.size(); i++)
          hairs[i] = TutorialScene::Hair(mesh->hairs[i].vertex,mesh->hairs[i].id);
      }

    public:
      avector<Vec3fa> positions;  //!< hair control points (x,y,z,r)
      std::vector<Hair> hairs;    //!< list of hairs
      
      unsigned numTimeSteps;
      unsigned numVertices;
      unsigned materialID;
      unsigned tessellation_rate;
    };
    
    struct Instance : public Geometry
    {
      ALIGNED_STRUCT;

      Instance (const SceneGraph::Transformations& spaces, unsigned geomID)
        : Geometry(INSTANCE), spaces(spaces), geomID(geomID) {}
      
    public:
      SceneGraph::Transformations spaces;
      unsigned geomID;
    };

    struct Group : public Geometry
    {
      Group (std::vector<Ref<Geometry>> children)
        : Geometry(GROUP), children(children) {}

      size_t size() const { return children.size(); }
      Ref<Geometry> at(size_t i) { return children[i]; }

    public:
      std::vector<Ref<Geometry>> children;
    };

    bool empty() const {
      return geometries.size() == 0;
    }

  public:
    avector<Material> materials;                 //!< material list
    std::vector<Ref<Geometry> > geometries;      //!< list of geometries
    std::vector<Ref<SceneGraph::Light>> lights;  //!< list of lights
  };
}
