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

#pragma once

#include "../default.h"
#include "../scenegraph/scenegraph.h"

namespace embree
{
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

      Triangle (int v0, int v1, int v2, int materialID) 
      : v0(v0), v1(v1), v2(v2), materialID(materialID) {}

    public:
      int v0, v1, v2, materialID;
    };

    /*! OBJ Quad */
    struct Quad 
    {
    public:
      Quad () {}

      Quad (int v0, int v1, int v2, int v3) 
      : v0(v0), v1(v1), v2(v2), v3(v3) {}

    public:
      int v0, v1, v2, v3;
    };

    struct Geometry : public RefCount
    {
      enum Type { TRIANGLE_MESH, SUBDIV_MESH, HAIR_SET, INSTANCE, GROUP, QUAD_MESH, LINE_SEGMENTS };
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
      std::vector<Quad> quads; // FIXME: remove
      int meshMaterialID;
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
      int meshMaterialID;
    };

    /*! Subdivision Mesh. */
    struct SubdivMesh : public Geometry
    {
      SubdivMesh () : Geometry(SUBDIV_MESH) {}
      avector<Vec3fa> positions;            //!< vertex positions
      avector<Vec3fa> normals;              //!< face vertex normals
      std::vector<Vec2f> texcoords;             //!< face texture coordinates
      std::vector<int> position_indices;        //!< position indices for all faces
      std::vector<int> normal_indices;          //!< normal indices for all faces
      std::vector<int> texcoord_indices;        //!< texcoord indices for all faces
      std::vector<int> verticesPerFace;         //!< number of indices of each face
      std::vector<int> holes;                   //!< face ID of holes
      std::vector<Vec2i> edge_creases;          //!< index pairs for edge crease 
      std::vector<float> edge_crease_weights;   //!< weight for each edge crease
      std::vector<int> vertex_creases;          //!< indices of vertex creases
      std::vector<float> vertex_crease_weights; //!< weight for each vertex crease
      int materialID;
    };

    struct Hair 
    {
    public:
      Hair () {}
      Hair (int vertex, int id)
      : vertex(vertex), id(id) {}
    public:
      int vertex,id;  //!< index of first control point and hair ID
    };

    /*! Line segments. */
    struct LineSegments : public Geometry
    {
      LineSegments () : Geometry(LINE_SEGMENTS) {}
      avector<Vec3fa> v;        //!< control points (x,y,z,r)
      avector<Vec3fa> v2;       //!< control points (x,y,z,r)
      std::vector<int> indices; //!< index buffer
      int materialID;
    };

    /*! Hair Set. */
    struct HairSet : public Geometry
    {
      HairSet () : Geometry(HAIR_SET) {}
      avector<Vec3fa> v;       //!< hair control points (x,y,z,r)
      avector<Vec3fa> v2;       //!< hair control points (x,y,z,r)
      std::vector<Hair> hairs;  //!< list of hairs
      int materialID;
    };

    struct Instance : public Geometry
    {
      ALIGNED_STRUCT;

      Instance(const AffineSpace3fa& space, int geomID)
        : Geometry(INSTANCE), space(space), geomID(geomID) {}

    public:
      AffineSpace3fa space;
      int geomID;
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
    avector<Material> materials;                   //!< material list
    std::vector<Ref<Geometry> > geometries;        //!< list of geometries
    avector<AmbientLight> ambientLights;           //!< list of ambient lights
    avector<PointLight> pointLights;               //!< list of point lights
    avector<DirectionalLight> directionalLights;   //!< list of directional lights
    avector<DistantLight> distantLights;           //!< list of distant lights
  };
}
