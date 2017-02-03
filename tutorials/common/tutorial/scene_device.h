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

#include "../math/sampling.h"
#include "../lights/light.h"
#include "../lights/ambient_light.h"
#include "../lights/directional_light.h"
#include "../lights/point_light.h"
#include "../lights/quad_light.h"
#include "../lights/spot_light.h"
#include "scene.h"

namespace embree
{
  struct ISPCTriangle
  {
    unsigned v0;                /*< first triangle vertex */
    unsigned v1;                /*< second triangle vertex */
    unsigned v2;                /*< third triangle vertex */
  };

  struct ISPCQuad
  {
    unsigned v0;                /*< first triangle vertex */
    unsigned v1;                /*< second triangle vertex */
    unsigned v2;                /*< third triangle vertex */
    unsigned v3;                /*< fourth triangle vertex */
  };

  struct ISPCHair
  {
    unsigned vertex;
    unsigned id;
  };

  enum ISPCType { TRIANGLE_MESH, SUBDIV_MESH, HAIR_SET, INSTANCE, GROUP, QUAD_MESH, LINE_SEGMENTS, CURVES };

  struct ISPCMaterial
  {
    int ty;
    int align0,align1,align2;
    Vec3fa v[7];
  };

  enum TEXTURE_FORMAT {
    Texture_RGBA8        = 1,
    Texture_RGB8         = 2,
    Texture_FLOAT32      = 3,
  };

  struct ISPCGeometry
  {
    ISPCGeometry (ISPCType type) : type(type) {}
    
    ISPCType type;
  };

  struct ISPCTriangleMesh
  {
    ISPCTriangleMesh (TutorialScene* scene_in, Ref<SceneGraph::TriangleMeshNode> in) 
    : geom(TRIANGLE_MESH)
    {
      positions = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        positions[i] = in->positions[i].data();
      normals = in->normals.data();
      texcoords = in->texcoords.data();
      triangles = (ISPCTriangle*) in->triangles.data();
      numTimeSteps = in->numTimeSteps();
      numVertices = in->numVertices();
      numTriangles = in->numPrimitives();
      scene = nullptr;
      geomID = -1;
      materialID = scene_in->materialID(in->material);
    }
    
  public:
    ISPCGeometry geom;
    Vec3fa** positions;     //!< vertex position array with all timesteps
    Vec3fa* normals;       //!< vertex normal array
    Vec2f* texcoords;      //!< vertex texcoord array
    ISPCTriangle* triangles;  //!< list of triangles
    
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numTriangles;
    RTCScene scene;
    unsigned int geomID;
    unsigned int materialID;
  };
  
  struct ISPCQuadMesh
  {
    ISPCQuadMesh (TutorialScene* scene_in, Ref<SceneGraph::QuadMeshNode> in) 
    : geom(QUAD_MESH)
    {
      positions = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        positions[i] = in->positions[i].data();
      normals = in->normals.data();
      texcoords = in->texcoords.data();
      quads = (ISPCQuad*) in->quads.data();
      numTimeSteps = in->numTimeSteps();
      numVertices = in->numVertices();
      numQuads = in->numPrimitives();
      scene = nullptr;
      geomID = -1;
      materialID = scene_in->materialID(in->material);
    }
    
  public:
    ISPCGeometry geom;
    Vec3fa** positions;    //!< vertex position array
    Vec3fa* normals;       //!< vertex normal array
    Vec2f* texcoords;     //!< vertex texcoord array
    ISPCQuad* quads;      //!< list of quads
    
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numQuads;
    RTCScene scene;
    unsigned int geomID;
    unsigned int materialID;
  };
  
  struct ISPCSubdivMesh
  {
    ISPCSubdivMesh (TutorialScene* scene_in, Ref<SceneGraph::SubdivMeshNode> in) 
    : geom(SUBDIV_MESH)
    {
      positions = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        positions[i] = in->positions[i].data();
      normals = in->normals.data();
      texcoords = in->texcoords.data();
      position_indices = in->position_indices.data();
      normal_indices = in->normal_indices.data();
      texcoord_indices = in->texcoord_indices.data();
      position_subdiv_mode =  in->position_subdiv_mode;
      normal_subdiv_mode = in->normal_subdiv_mode;
      texcoord_subdiv_mode = in->texcoord_subdiv_mode;
      verticesPerFace = in->verticesPerFace.data();
      holes = in->holes.data();
      edge_creases = in->edge_creases.data();
      edge_crease_weights = in->edge_crease_weights.data();
      vertex_creases = in->vertex_creases.data();
      vertex_crease_weights = in->vertex_crease_weights.data();
      numTimeSteps = in->numTimeSteps();
      numVertices = in->numPositions();
      numFaces = in->numPrimitives();
      numEdges = unsigned(in->position_indices.size());
      numEdgeCreases = unsigned(in->edge_creases.size());
      numVertexCreases = unsigned(in->vertex_creases.size());
      numHoles = unsigned(in->holes.size());
      numNormals = unsigned(in->normals.size());
      numTexCoords = unsigned(in->texcoords.size());
      materialID = scene_in->materialID(in->material);
      scene = nullptr;
      geomID = -1;
      
      size_t numEdges = in->position_indices.size();
      size_t numFaces = in->verticesPerFace.size();
      subdivlevel = new float[numEdges];
      face_offsets = new unsigned[numFaces];
      for (size_t i=0; i<numEdges; i++) subdivlevel[i] = 1.0f;
      int offset = 0;
      for (size_t i=0; i<numFaces; i++)
      {
        face_offsets[i] = offset;
        offset+=verticesPerFace[i];
      }
    }
    
    ~ISPCSubdivMesh ()
    {
      delete[] subdivlevel;
      delete[] face_offsets;
    }
    
  private:
    ISPCSubdivMesh (const ISPCSubdivMesh& other) DELETED; // do not implement
    ISPCSubdivMesh& operator= (const ISPCSubdivMesh& other) DELETED; // do not implement
    
  public:
    ISPCGeometry geom;
    Vec3fa** positions;       //!< vertex positions
    Vec3fa* normals;         //!< face vertex normals
    Vec2f* texcoords;        //!< face texture coordinates
    unsigned* position_indices;   //!< position indices for all faces
    unsigned* normal_indices;     //!< normal indices for all faces
    unsigned* texcoord_indices;   //!< texcoord indices for all faces
    RTCSubdivisionMode position_subdiv_mode;  
    RTCSubdivisionMode normal_subdiv_mode;
    RTCSubdivisionMode texcoord_subdiv_mode;
    unsigned* verticesPerFace;    //!< number of indices of each face
    unsigned* holes;              //!< face ID of holes
    float* subdivlevel;      //!< subdivision level
    Vec2i* edge_creases;          //!< crease index pairs
    float* edge_crease_weights;   //!< weight for each crease
    unsigned* vertex_creases;          //!< indices of vertex creases
    float* vertex_crease_weights; //!< weight for each vertex crease
    unsigned* face_offsets;
    
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numFaces;
    unsigned int numEdges;
    unsigned int numEdgeCreases;
    unsigned int numVertexCreases;
    unsigned int numHoles;
    unsigned int numNormals;
    unsigned int numTexCoords;
    unsigned int materialID;
    RTCScene scene;
    unsigned int geomID;
  };
  
  struct ISPCLineSegments
  {
    ISPCLineSegments (TutorialScene* scene_in, Ref<SceneGraph::LineSegmentsNode> in) 
    : geom(LINE_SEGMENTS)
    {
      positions = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        positions[i] = in->positions[i].data();
      indices = in->indices.data();
      numTimeSteps = in->numTimeSteps();
      numVertices = in->numVertices();
      numSegments = in->numPrimitives();
      materialID = scene_in->materialID(in->material);
      scene = nullptr;
      geomID = -1;
    }
    
  public:
    ISPCGeometry geom;
    Vec3fa** positions;        //!< control points (x,y,z,r)
    unsigned* indices;        //!< for each segment, index to first control point
    
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numSegments;
    unsigned int materialID;
    RTCScene scene;
    unsigned int geomID;
  };
  
  struct ISPCHairSet
  {
    ISPCHairSet (TutorialScene* scene_in, bool hair, Ref<SceneGraph::HairSetNode> in) 
    : geom(hair ? HAIR_SET : CURVES)
    {
      positions = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        positions[i] = in->positions[i].data();
      hairs = (ISPCHair*) in->hairs.data();
      numTimeSteps = in->numTimeSteps();
      numVertices = in->numVertices();
      numHairs = in->numPrimitives();
      materialID = scene_in->materialID(in->material);
      scene = nullptr;
      geomID = -1;
      tessellation_rate = in->tessellation_rate;
    }
    
  public:
    ISPCGeometry geom;
    Vec3fa** positions;       //!< hair control points (x,y,z,r)
    ISPCHair* hairs;         //!< for each hair, index to first control point
    
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numHairs;
    unsigned int materialID;
    RTCScene scene;
    unsigned int geomID;
    unsigned int tessellation_rate;
  };
  
  struct ISPCInstance
  {
    ALIGNED_STRUCT;
    
    static ISPCInstance* create (TutorialScene* scene, Ref<SceneGraph::TransformNode> in) {
      return ::new (alignedMalloc(sizeof(ISPCInstance)+(in->spaces.size()-1)*sizeof(AffineSpace3fa))) ISPCInstance(scene,in);
    }
    
  private:
    ISPCInstance (TutorialScene* scene, Ref<SceneGraph::TransformNode> in)
    : geom(INSTANCE), geomID(scene->geometryID(in->child)), numTimeSteps(unsigned(in->spaces.size())) 
    {
      for (size_t i=0; i<numTimeSteps; i++)
        spaces[i] = in->spaces[i];
    }
    
  public:
    ISPCGeometry geom;
    unsigned int geomID;
    unsigned int numTimeSteps;
    AffineSpace3fa spaces[1];
  };
  
  struct ISPCScene
  {
    struct ISPCGroup
    {
      ISPCGroup (TutorialScene* scene, Ref<SceneGraph::GroupNode> in)
      : geom(GROUP)
      {
        numGeometries = in->size();
        geometries = new ISPCGeometry*[numGeometries];
        for (size_t i=0; i<numGeometries; i++)
          geometries[i] = ISPCScene::convertGeometry(scene,in->child(i));
      }
      
      ~ISPCGroup()
      {
        for (size_t i=0; i<numGeometries; i++)
          delete geometries[i];
        delete[] geometries;
      }
      
    private:
      ISPCGroup (const ISPCGroup& other) DELETED; // do not implement
      ISPCGroup& operator= (const ISPCGroup& other) DELETED; // do not implement
      
    public:
      ISPCGeometry geom;
      ISPCGeometry** geometries;
      size_t numGeometries;
    };
    
    ISPCScene(TutorialScene* in)
    {
      geometries = new ISPCGeometry*[in->geometries.size()];
      for (size_t i=0; i<in->geometries.size(); i++)
        geometries[i] = convertGeometry(in,in->geometries[i]);
      numGeometries = unsigned(in->geometries.size());
      
      materials = (ISPCMaterial*) in->materials.data();
      numMaterials = unsigned(in->materials.size());
      
      lights = new Light*[in->lights.size()];
      numLights = 0;
      for (size_t i=0; i<in->lights.size(); i++)
      {
        Light* light = convertLight(in->lights[i]);
        if (light) lights[numLights++] = light;
      }
      
      geomID_to_scene = in->geomID_to_scene.data();
      geomID_to_inst = (ISPCInstance**) in->geomID_to_inst.data();
    }
    
    ~ISPCScene()
    {
      /* delete all geometries */
      for (size_t i=0; i<numGeometries; i++) 
      {
        switch (geometries[i]->type) {
        case TRIANGLE_MESH: delete (ISPCTriangleMesh*) geometries[i]; break;
        case SUBDIV_MESH  : delete (ISPCSubdivMesh*) geometries[i]; break;
        case HAIR_SET: delete (ISPCHairSet*) geometries[i]; break;
        case INSTANCE: delete (ISPCInstance*) geometries[i]; break;
        case GROUP: delete (ISPCGroup*) geometries[i]; break;
        case QUAD_MESH: delete (ISPCQuadMesh*) geometries[i]; break;
        case LINE_SEGMENTS: delete (ISPCLineSegments*) geometries[i]; break;
        case CURVES: delete (ISPCHairSet*) geometries[i]; break;
        default: assert(false); break;
        }
      }        
      delete[] geometries;
      
      /* delete all lights */
      //for (size_t i=0; i<numLights; i++)
      //{
      // FIXME: currently lights cannot get deleted
      //}
      delete[] lights;
    }
    
    static ISPCGeometry* convertGeometry (TutorialScene* scene, Ref<SceneGraph::Node> in)
    {
      if (Ref<SceneGraph::TriangleMeshNode> mesh = in.dynamicCast<SceneGraph::TriangleMeshNode>())
        return (ISPCGeometry*) new ISPCTriangleMesh(scene,mesh);
      else if (Ref<SceneGraph::QuadMeshNode> mesh = in.dynamicCast<SceneGraph::QuadMeshNode>())
        return (ISPCGeometry*) new ISPCQuadMesh(scene,mesh);
      else if (Ref<SceneGraph::SubdivMeshNode> mesh = in.dynamicCast<SceneGraph::SubdivMeshNode>())
        return (ISPCGeometry*) new ISPCSubdivMesh(scene,mesh);
      else if (Ref<SceneGraph::LineSegmentsNode> mesh = in.dynamicCast<SceneGraph::LineSegmentsNode>())
        return (ISPCGeometry*) new ISPCLineSegments(scene,mesh);
      else if (Ref<SceneGraph::HairSetNode> mesh = in.dynamicCast<SceneGraph::HairSetNode>())
        return (ISPCGeometry*) new ISPCHairSet(scene,mesh->hair,mesh);
      else if (Ref<SceneGraph::TransformNode> mesh = in.dynamicCast<SceneGraph::TransformNode>())
        return (ISPCGeometry*) ISPCInstance::create(scene,mesh);
      else if (Ref<SceneGraph::GroupNode> mesh = in.dynamicCast<SceneGraph::GroupNode>())
        return (ISPCGeometry*) new ISPCGroup(scene,mesh);
      else
        THROW_RUNTIME_ERROR("unknown geometry type");
    }
    
    static Light* convertLight(Ref<SceneGraph::Light> in)
    {
      void* out = 0;
      
      switch (in->getType())
      {
      case SceneGraph::LIGHT_AMBIENT:
      {
        Ref<SceneGraph::AmbientLight> inAmbient = in.dynamicCast<SceneGraph::AmbientLight>();
        out = AmbientLight_create();
        AmbientLight_set(out, inAmbient->L);
        break;
      }
      case SceneGraph::LIGHT_DIRECTIONAL:
      {
        Ref<SceneGraph::DirectionalLight> inDirectional = in.dynamicCast<SceneGraph::DirectionalLight>();
        out = DirectionalLight_create();
        DirectionalLight_set(out, -normalize(inDirectional->D), inDirectional->E, 1.0f);
        break;
      }
      case SceneGraph::LIGHT_DISTANT:
      {
        Ref<SceneGraph::DistantLight> inDistant = in.dynamicCast<SceneGraph::DistantLight>();
        out = DirectionalLight_create();
        DirectionalLight_set(out,
                             -normalize(inDistant->D),
                             inDistant->L * rcp(uniformSampleConePDF(inDistant->cosHalfAngle)),
                             inDistant->cosHalfAngle);
        break;
      }
      case SceneGraph::LIGHT_POINT:
      {
        Ref<SceneGraph::PointLight> inPoint = in.dynamicCast<SceneGraph::PointLight>();
        out = PointLight_create();
        PointLight_set(out, inPoint->P, inPoint->I, 0.f);
        break;
      }
      case SceneGraph::LIGHT_SPOT:
      case SceneGraph::LIGHT_TRIANGLE:
      case SceneGraph::LIGHT_QUAD:
      {
        // FIXME: not implemented yet
        break;
      }
      
      default:
        THROW_RUNTIME_ERROR("unknown light type");
      }
      
      return (Light*)out;
    }
    
  private:
    ISPCScene (const ISPCScene& other) DELETED; // do not implement
    ISPCScene& operator= (const ISPCScene& other) DELETED; // do not implement
    
  public:
    ISPCGeometry** geometries;   //!< list of geometries
    ISPCMaterial* materials;     //!< material list
    unsigned int numGeometries;           //!< number of geometries
    unsigned int numMaterials;            //!< number of materials
    
    Light** lights;              //!< list of lights
    unsigned int numLights;               //!< number of lights
  public:
    RTCScene* geomID_to_scene;
    ISPCInstance** geomID_to_inst;
  };
  
  typedef ISPCScene::ISPCGroup ISPCGroup;
  
  extern "C" RTCScene ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCSceneFlags sflags, RTCAlgorithmFlags aflags, RTCGeometryFlags gflags);
}
