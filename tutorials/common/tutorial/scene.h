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

#pragma once

#include "../default.h"
#include "../scenegraph/scenegraph.h"

namespace embree
{
  enum Shader { 
    SHADER_DEFAULT = 0, 
    SHADER_EYELIGHT = 1,
    SHADER_UV = 2,
    SHADER_NG = 3,
    SHADER_GEOMID = 4,
    SHADER_GEOMID_PRIMID = 5,
    SHADER_AMBIENT_OCCLUSION = 6
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

    struct Geometry : public RefCount
    {
      enum Type { TRIANGLE_MESH, SUBDIV_MESH, HAIR_SET, INSTANCE, GROUP, QUAD_MESH, LINE_SEGMENTS, CURVES };
      Type type;

      Geometry (Type type) : type(type) {}
    };

    /*! Triangle Mesh. */
    struct TriangleMesh : public Geometry
    {
      TriangleMesh () : Geometry(TRIANGLE_MESH) {}
      avector<Vec3fa> v;
      avector<Vec3fa> v2;
      avector<Vec3fa> vn;
      std::vector<Vec2f> vt;
      std::vector<Triangle> triangles;
      unsigned materialID;
    };

    /*! Quad Mesh. */
    struct QuadMesh : public Geometry
    {
      QuadMesh () : Geometry(QUAD_MESH) {}
      avector<Vec3fa> v;
      avector<Vec3fa> v2;
      avector<Vec3fa> vn;
      std::vector<Vec2f> vt;
      std::vector<Quad> quads;
      unsigned materialID;
    };

    /*! Subdivision Mesh. */
    struct SubdivMesh : public Geometry
    {
      SubdivMesh () : Geometry(SUBDIV_MESH) {}
      avector<Vec3fa> positions;            //!< vertex positions
      avector<Vec3fa> positions2;            //!< vertex positions
      avector<Vec3fa> normals;              //!< face vertex normals
      std::vector<Vec2f> texcoords;             //!< face texture coordinates
      std::vector<unsigned> position_indices;        //!< position indices for all faces
      std::vector<unsigned> normal_indices;          //!< normal indices for all faces
      std::vector<unsigned> texcoord_indices;        //!< texcoord indices for all faces
      std::vector<unsigned> verticesPerFace;         //!< number of indices of each face
      std::vector<unsigned> holes;                   //!< face ID of holes
      std::vector<Vec2i> edge_creases;          //!< index pairs for edge crease 
      std::vector<float> edge_crease_weights;   //!< weight for each edge crease
      std::vector<unsigned> vertex_creases;          //!< indices of vertex creases
      std::vector<float> vertex_crease_weights; //!< weight for each vertex crease
      unsigned materialID;
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

    /*! Line segments. */
    struct LineSegments : public Geometry
    {
      LineSegments () : Geometry(LINE_SEGMENTS) {}
      avector<Vec3fa> v;        //!< control points (x,y,z,r)
      avector<Vec3fa> v2;       //!< control points (x,y,z,r)
      std::vector<unsigned> indices; //!< index buffer
      unsigned materialID;
    };

    /*! Hair Set. */
    struct HairSet : public Geometry
    {
      HairSet (bool hair) : Geometry(hair ? HAIR_SET : CURVES) {}
      avector<Vec3fa> v;       //!< hair control points (x,y,z,r)
      avector<Vec3fa> v2;      //!< hair control points (x,y,z,r)
      std::vector<Hair> hairs; //!< list of hairs
      unsigned materialID;
    };

    struct Instance : public Geometry
    {
      ALIGNED_STRUCT;

    Instance(const AffineSpace3fa& space0, const AffineSpace3fa& space1, unsigned geomID)
      : Geometry(INSTANCE), space0(space0), space1(space1), geomID(geomID) {}

    public:
      AffineSpace3fa space0;
      AffineSpace3fa space1;
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
