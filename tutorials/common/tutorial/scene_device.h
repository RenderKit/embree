// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if !defined(ISPC)
#include "../math/sampling.h"
#include "../lights/light.h"
#include "../lights/ambient_light.h"
#include "../lights/directional_light.h"
#include "../lights/point_light.h"
#include "../lights/quad_light.h"
#include "../lights/spot_light.h"
#include "scene.h"
#else
#include "../lights/light.isph"
#endif

#include "../scenegraph/texture.h"
#include "../scenegraph/materials.h"

#if !defined(ISPC)

#define CPPTUTORIAL
#include "../scenegraph/materials.h"
#undef CPPTUTORIAL

namespace embree
{
#endif

  struct ISPCTriangle
  {
    unsigned int v0;                /*< first triangle vertex */
    unsigned int v1;                /*< second triangle vertex */
    unsigned int v2;                /*< third triangle vertex */
  };

  struct ISPCQuad
  {
    unsigned int v0;                /*< first triangle vertex */
    unsigned int v1;                /*< second triangle vertex */
    unsigned int v2;                /*< third triangle vertex */
    unsigned int v3;                /*< fourth triangle vertex */
  };

  struct ISPCHair
  {
    unsigned int vertex;
    unsigned int id;
  };

  struct ISPCGrid
  {
    unsigned int startVtxID;
    unsigned int lineOffset;
#if !defined(ISPC)
    unsigned short resX,resY; // max is a 32k x 32k grid
#else
    int16 resX,resY;
#endif
  };

  enum ISPCType { TRIANGLE_MESH, SUBDIV_MESH, CURVES, INSTANCE, GROUP, QUAD_MESH, GRID_MESH, POINTS };

  struct ISPCGeometry
  {
#if !defined(ISPC)
    ISPCGeometry (ISPCType type) : type(type), geometry(nullptr), materialID(-1), visited(false) {}
    ~ISPCGeometry () { if (geometry) rtcReleaseGeometry(geometry); }
#endif
    ISPCType type;
    RTCGeometry geometry;
    unsigned int materialID;
    bool visited;
  };

#if !defined(ISPC)
  enum TEXTURE_FORMAT {
    Texture_RGBA8        = 1,
    Texture_RGB8         = 2,
    Texture_FLOAT32      = 3,
  };
#endif

  struct ISPCTriangleMesh
  {
#if !defined(ISPC)
    ISPCTriangleMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::TriangleMeshNode> in);
    ~ISPCTriangleMesh ();

    void commit();

  private:
    ISPCTriangleMesh (const ISPCTriangleMesh& other) DELETED; // do not implement
    ISPCTriangleMesh& operator= (const ISPCTriangleMesh& other) DELETED; // do not implement

  public:
#endif

    ISPCGeometry geom;
    Vec3fa** positions;     //!< vertex position array
    Vec3fa** normals;       //!< vertex normal array
    Vec2f* texcoords;      //!< vertex texcoord array
    ISPCTriangle* triangles;  //!< list of triangles

    float startTime;
    float endTime;
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numTriangles; 
  };
  
  struct ISPCQuadMesh
  {
#if !defined(ISPC)
    ISPCQuadMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::QuadMeshNode> in);
    ~ISPCQuadMesh ();

    void commit();

  private:
    ISPCQuadMesh (const ISPCQuadMesh& other) DELETED; // do not implement
    ISPCQuadMesh& operator= (const ISPCQuadMesh& other) DELETED; // do not implement

  public:
#endif

    ISPCGeometry geom;
    Vec3fa** positions;    //!< vertex position array
    Vec3fa** normals;       //!< vertex normal array
    Vec2f* texcoords;     //!< vertex texcoord array
    ISPCQuad* quads;      //!< list of quads

    float startTime;
    float endTime;
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numQuads;
  };
  
  struct ISPCSubdivMesh
  {
#if !defined(ISPC)
    ISPCSubdivMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::SubdivMeshNode> in);
    ~ISPCSubdivMesh ();

    void commit();
    
  private:
    ISPCSubdivMesh (const ISPCSubdivMesh& other) DELETED; // do not implement
    ISPCSubdivMesh& operator= (const ISPCSubdivMesh& other) DELETED; // do not implement
    
  public:
#endif

    ISPCGeometry geom;
    Vec3fa** positions;       //!< vertex positions
    Vec3fa** normals;         //!< face vertex normals
    Vec2f* texcoords;        //!< face texture coordinates
    unsigned int* position_indices;   //!< position indices for all faces
    unsigned int* normal_indices;     //!< normal indices for all faces
    unsigned int* texcoord_indices;   //!< texcoord indices for all faces
    RTCSubdivisionMode position_subdiv_mode;  
    RTCSubdivisionMode normal_subdiv_mode;
    RTCSubdivisionMode texcoord_subdiv_mode;
    unsigned int* verticesPerFace;    //!< number of indices of each face
    unsigned int* holes;              //!< face ID of holes
    float* subdivlevel;      //!< subdivision level
    Vec2i* edge_creases;          //!< crease index pairs
    float* edge_crease_weights;   //!< weight for each crease
    unsigned int* vertex_creases;          //!< indices of vertex creases
    float* vertex_crease_weights; //!< weight for each vertex crease
    unsigned int* face_offsets;

    float startTime;
    float endTime;
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numFaces;
    unsigned int numEdges;
    unsigned int numEdgeCreases;
    unsigned int numVertexCreases;
    unsigned int numHoles;
    unsigned int numNormals;
    unsigned int numTexCoords;
  };
  
  struct ISPCHairSet
  {
#if !defined(ISPC)
    ISPCHairSet (RTCDevice device, TutorialScene* scene_in, RTCGeometryType type, Ref<SceneGraph::HairSetNode> in);
    ~ISPCHairSet();

    void commit();

  private:
    ISPCHairSet (const ISPCHairSet& other) DELETED; // do not implement
    ISPCHairSet& operator= (const ISPCHairSet& other) DELETED; // do not implement

  public:
#endif

    ISPCGeometry geom;
    Vec3fa** positions;       //!< hair control points (x,y,z,r)
    Vec3fa** normals;         //!< normal control points (x,y,z,r)
    Vec3fa** tangents;        //!< tangent control points (x,y,z,r)
    Vec3fa** dnormals;         //!< normal derivative control points (x,y,z,r)
    ISPCHair* hairs;          //!< for each hair, index to first control point
#if !defined(ISPC)
    unsigned char* flags;     //!< end cap flags per segment
#else
    uint8* flags;             //!< end cap flags per segment
#endif
    RTCGeometryType type;
    float startTime;
    float endTime;
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numHairs;
    unsigned int numHairCurves;
    unsigned int tessellation_rate;
  };

  struct ISPCPointSet
  {
#if !defined(ISPC)
    ISPCPointSet (RTCDevice device, TutorialScene* scene_in, RTCGeometryType type, Ref<SceneGraph::PointSetNode> in);
    ~ISPCPointSet();

    void commit();

  private:
    ISPCPointSet (const ISPCPointSet& other) DELETED; // do not implement
    ISPCPointSet& operator= (const ISPCPointSet& other) DELETED; // do not implement

  public:
#endif

    ISPCGeometry geom;
    Vec3fa** positions;       //!< hair control points (x,y,z,r)
    Vec3fa** normals;         //!< normal control points (x,y,z,r)

    RTCGeometryType type;
    float startTime;
    float endTime;
    unsigned int numTimeSteps;
    unsigned int numVertices;
  };

  struct ISPCGridMesh
  {
#if !defined(ISPC)
    ISPCGridMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::GridMeshNode> in);
    ~ISPCGridMesh ();

    void commit();

  private:
    ISPCGridMesh (const ISPCGridMesh& other) DELETED; // do not implement
    ISPCGridMesh& operator= (const ISPCGridMesh& other) DELETED; // do not implement

  public:
#endif

    ISPCGeometry geom;
    Vec3fa** positions;    //!< vertex position array
    ISPCGrid* grids;      //!< list of quads

    float startTime;
    float endTime;
    unsigned int numTimeSteps;
    unsigned int numVertices;
    unsigned int numGrids;
  };

  
  struct ISPCInstance
  {
#if !defined(ISPC)
    ISPCInstance (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::TransformNode> in);
    ~ISPCInstance();

    void commit();

  private:
    ISPCInstance (const ISPCInstance& other) DELETED; // do not implement
    ISPCInstance& operator= (const ISPCInstance& other) DELETED; // do not implement

  public:
#endif

    ISPCGeometry geom;
    ISPCGeometry* child;
    float startTime;
    float endTime;
    unsigned int numTimeSteps;
    bool quaternion;
    AffineSpace3fa* spaces;
  };

  struct ISPCGroup
  {
#if !defined(ISPC)
    ISPCGroup (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::GroupNode> in);
    ~ISPCGroup();

    void commit();
    
  private:
    ISPCGroup (const ISPCGroup& other) DELETED; // do not implement
    ISPCGroup& operator= (const ISPCGroup& other) DELETED; // do not implement
    
  public:
#endif
    ISPCGeometry geom;
    RTCScene scene;
    ISPCGeometry** geometries;
    unsigned int numGeometries;
    unsigned int requiredInstancingDepth; // instancing depth required for this group
  };
  
  struct ISPCScene
  {
#if !defined(ISPC)
    ISPCScene(RTCDevice device, TutorialScene* in);
    ~ISPCScene();

    void commit();
    
    static ISPCGeometry* convertGeometry (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::Node> in);   
    static Light* convertLight(Ref<SceneGraph::LightNode> in);
    static Light* createLight(Ref<SceneGraph::LightNode> in);

    template<typename LightNode> static void updateLight(const LightNode& in, Light* out);
    static void updateLight(const Ref<SceneGraph::LightNode>& in, Light* out);
    
  private:
    ISPCScene (const ISPCScene& other) DELETED; // do not implement
    ISPCScene& operator= (const ISPCScene& other) DELETED; // do not implement
    
  public:
#endif
    RTCScene scene;
    ISPCGeometry** geometries;   //!< list of geometries
    ISPCMaterial** materials;     //!< material list
    unsigned int numGeometries;           //!< number of geometries
    unsigned int numMaterials;            //!< number of materials
    
    Light** lights;              //!< list of lights
    unsigned int numLights;               //!< number of lights
    void* tutorialScene;
  };

#if !defined(ISPC)
  typedef void (*AssignShaderTy)(ISPCGeometry* geometry);
  extern "C" { extern AssignShaderTy assignShadersFunc; };
#else
  typedef unmasked void (*AssignShaderTy)(uniform ISPCGeometry* uniform geometry);
  extern uniform AssignShaderTy assignShadersFunc;
#endif
  
#if !defined(ISPC)
  extern "C" void UpdateScene(ISPCScene* scene_in, float time);
  extern "C" RTCScene ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCBuildQuality quality, RTCSceneFlags flags = RTC_SCENE_FLAG_NONE);
#else
  unmasked extern "C" void UpdateScene(ISPCScene* uniform scene_in, uniform float time);
  unmasked extern "C" RTCScene ConvertScene (RTCDevice g_device, ISPCScene* uniform scene_in, uniform RTCBuildQuality quality, uniform RTCSceneFlags flags = RTC_SCENE_FLAG_NONE);
#endif

#if !defined(ISPC)                    
}
#endif

