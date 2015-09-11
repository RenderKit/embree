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

#include "../../../common/sys/ref.h"
#include "../../../common/sys/vector.h"
#include "../../../common/math/vec2.h"
#include "../../../common/math/vec3.h"
#include "../../../common/math/affinespace.h"
#include "scenegraph.h"

#include <vector>
#include <memory>
#include <map>

namespace embree
{
  /*! Scene representing the OBJ file */
  struct OBJScene  // FIXME: name Scene
  {
    OBJScene () {}

    void add (Ref<SceneGraph::Node> node);

    ~OBJScene () 
    {
      for (size_t i=0; i<meshes.size(); i++)
        delete meshes[i];

      for (size_t i=0; i<hairsets.size(); i++)
        delete hairsets[i];
    }

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
      Quad (int v0, int v1, int v2, int v3) 
      : v0(v0), v1(v1), v2(v2), v3(v3) {}

    public:
      int v0, v1, v2, v3;
    };

    /*! Mesh. */
    struct Mesh 
    {
      void set_motion_blur(const Mesh* other);

      avector<Vec3fa> v;
      avector<Vec3fa> v2;
      avector<Vec3fa> vn;

      std::vector<Vec2f> vt;
      std::vector<Triangle> triangles;
      std::vector<Quad> quads;
      int meshMaterialID;
    };

    /*! Subdivision Mesh. */
    struct SubdivMesh 
    {
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

    /*! Hair Set. */
    struct HairSet
    {
      void set_motion_blur(const HairSet* other);

      avector<Vec3fa> v;       //!< hair control points (x,y,z,r)
      avector<Vec3fa> v2;       //!< hair control points (x,y,z,r)
      std::vector<Hair> hairs;  //!< list of hairs
    };

    bool empty() const {
      return meshes.size() == 0 && hairsets.size() == 0;
    }

    void set_motion_blur(OBJScene& other);

    void convert_to_subdiv()
    {
      for (size_t i=0; i<meshes.size(); i++)
      {
	Mesh* mesh = meshes[i];
	SubdivMesh* smesh = new SubdivMesh;
	smesh->positions = mesh->v;
	smesh->normals = mesh->vn;
	smesh->texcoords = mesh->vt;
	const size_t numTriangles = mesh->triangles.size();
	smesh->verticesPerFace.resize(numTriangles);
	smesh->position_indices.resize(3*numTriangles);
	int materialID = 0;
	for (size_t j=0; j<numTriangles; j++) {
	  smesh->verticesPerFace[j] = 3;
	  smesh->position_indices[3*j+0] = mesh->triangles[j].v0;
	  smesh->position_indices[3*j+1] = mesh->triangles[j].v1;
	  smesh->position_indices[3*j+2] = mesh->triangles[j].v2;
	  materialID                     = mesh->triangles[j].materialID;
	}
	smesh->materialID = materialID;

	delete mesh;
	subdiv.push_back(smesh);
      }
      meshes.clear();
    }

  public:
    avector<Material> materials;                      //!< material list
    std::vector<Mesh*> meshes;                         //!< list of meshes
    std::vector<HairSet*> hairsets;                    //!< list of hair sets
    std::vector<SubdivMesh*> subdiv;                  //!< list of subdivision meshes
    avector<AmbientLight> ambientLights;           //!< list of ambient lights
    avector<PointLight> pointLights;               //!< list of point lights
    avector<DirectionalLight> directionalLights;   //!< list of directional lights
    avector<DistantLight> distantLights;           //!< list of distant lights
  };
}
