// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "scene_device.h"
#include "application.h"

#define FIXED_EDGE_TESSELLATION_VALUE 4

namespace embree
{
  extern "C" {
    int g_instancing_mode = SceneGraph::INSTANCING_NONE;
    float g_min_width_max_radius_scale = 1.0f;
    AssignShaderTy assignShadersFunc = nullptr;
  }

  extern "C" int g_animation_mode;
  
  void deleteGeometry(ISPCGeometry* geom)
  {
    if (geom == nullptr)
      return;
    
    switch (geom->type) {
    case TRIANGLE_MESH: delete (ISPCTriangleMesh*) geom; break;
    case SUBDIV_MESH  : delete (ISPCSubdivMesh*) geom; break;
    case INSTANCE: delete (ISPCInstance*) geom; break;
    case GROUP: delete (ISPCGroup*) geom; break;
    case QUAD_MESH: delete (ISPCQuadMesh*) geom; break;
    case CURVES: delete (ISPCHairSet*) geom; break;
    case GRID_MESH: delete (ISPCGridMesh*) geom; break;
    case POINTS: delete (ISPCPointSet*) geom; break;
    default: assert(false); break;
    }
  }
  
  ISPCScene::ISPCScene(TutorialScene* in)
    : scene(nullptr), tutorialScene(in)
  {
    SceneGraph::opaque_geometry_destruction = (void(*)(void*)) deleteGeometry;
    
    geometries = new ISPCGeometry*[in->geometries.size()];
    for (size_t i=0; i<in->geometries.size(); i++)
      geometries[i] = convertGeometry(in,in->geometries[i]);
    numGeometries = unsigned(in->geometries.size());
    
    materials = new ISPCMaterial*[in->materials.size()];
    for (size_t i=0; i<in->materials.size(); i++)
      materials[i] = (ISPCMaterial*) in->materials[i]->material();
    numMaterials = unsigned(in->materials.size());
    
    lights = new Light*[in->lights.size()];
    numLights = 0;
    for (size_t i=0; i<in->lights.size(); i++)
    {
      Light* light = convertLight(in->lights[i]->get(0.0f));
      if (light) lights[numLights++] = light;
    }
  }

  ISPCScene::~ISPCScene()
  {
    delete[] geometries;
    delete[] materials;
    for (size_t i=0; i<numLights; i++)
      Light_destroy(lights[i]);
    delete[] lights;
  }
  
  Light* ISPCScene::convertLight(Ref<SceneGraph::LightNode> in)
  {
    Light* l = createLight(in);
    updateLight(in->get(0.0f),l);
    return l;
  }

  template<> void ISPCScene::updateLight(const SceneGraph::AmbientLight& in, Light* out) {
    AmbientLight_set(out, in.L);
  }

  template<> void ISPCScene::updateLight(const SceneGraph::DirectionalLight& in, Light* out) {
    DirectionalLight_set(out, -normalize(in.D), in.E, 1.0f);
  }

  template<> void ISPCScene::updateLight(const SceneGraph::DistantLight& in, Light* out)
  {
    DirectionalLight_set(out,
                         -normalize(in.D),
                         in.L * rcp(uniformSampleConePDF(in.cosHalfAngle)),
                         in.cosHalfAngle);
  }

  template<> void ISPCScene::updateLight(const SceneGraph::PointLight& in, Light* out) {
    PointLight_set(out, in.P, in.I, 0.f);
  }

  Light* ISPCScene::createLight(Ref<SceneGraph::LightNode> in)
  {
    switch (in->getType()) {
    case SceneGraph::LIGHT_AMBIENT    : return (Light*) AmbientLight_create();
    case SceneGraph::LIGHT_DIRECTIONAL: return (Light*) DirectionalLight_create();
    case SceneGraph::LIGHT_DISTANT    : return (Light*) DirectionalLight_create();
    case SceneGraph::LIGHT_POINT      : return (Light*) PointLight_create();
    case SceneGraph::LIGHT_SPOT       : return nullptr;
    case SceneGraph::LIGHT_TRIANGLE   : return nullptr;
    case SceneGraph::LIGHT_QUAD       : return nullptr;
    default                           : THROW_RUNTIME_ERROR("unknown light type");
    }
    return nullptr;
  }

  void ISPCScene::updateLight(const Ref<SceneGraph::LightNode>& in, Light* out)
  {
    if (auto light = in.dynamicCast<SceneGraph::LightNodeImpl<SceneGraph::AmbientLight>>())
      updateLight(light->light, out);
    else if (auto light = in.dynamicCast<SceneGraph::LightNodeImpl<SceneGraph::DirectionalLight>>())
      updateLight(light->light, out);
    else if (auto light = in.dynamicCast<SceneGraph::LightNodeImpl<SceneGraph::DistantLight>>())
      updateLight(light->light, out);
    else if (auto light = in.dynamicCast<SceneGraph::LightNodeImpl<SceneGraph::PointLight>>())
      updateLight(light->light, out);
  }
  
  ISPCTriangleMesh::ISPCTriangleMesh (TutorialScene* scene_in, Ref<SceneGraph::TriangleMeshNode> in) 
    : geom(TRIANGLE_MESH), positions(nullptr), normals(nullptr)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();
    
    if (in->normals.size()) {
      normals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = in->normals[i].data();
    }
    
    texcoords = in->texcoords.data();
    triangles = (ISPCTriangle*) in->triangles.data();
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numTriangles = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCTriangleMesh::~ISPCTriangleMesh () {
    if (positions) delete[] positions;
    if (normals) delete[] normals;
  }
  
  ISPCQuadMesh::ISPCQuadMesh (TutorialScene* scene_in, Ref<SceneGraph::QuadMeshNode> in) 
    : geom(QUAD_MESH), positions(nullptr), normals(nullptr)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();

    if (in->normals.size()) {
      normals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = in->normals[i].data();
    }
    
    texcoords = in->texcoords.data();
    quads = (ISPCQuad*) in->quads.data();
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numQuads = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCQuadMesh::~ISPCQuadMesh () {
    if (positions) delete[] positions;
    if (normals) delete[] normals;
  }


  ISPCGridMesh::ISPCGridMesh (TutorialScene* scene_in, Ref<SceneGraph::GridMeshNode> in) 
    : geom(GRID_MESH), positions(nullptr)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();

    grids = (ISPCGrid*) in->grids.data();
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numGrids = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCGridMesh::~ISPCGridMesh () {
    if (positions) delete[] positions;
  }


  ISPCSubdivMesh::ISPCSubdivMesh (TutorialScene* scene_in, Ref<SceneGraph::SubdivMeshNode> in) 
    : geom(SUBDIV_MESH), positions(nullptr), normals(nullptr)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();

    if (in->normals.size()) {
      normals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = in->normals[i].data();
    }
    
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
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = unsigned(in->numTimeSteps());
    numVertices = unsigned(in->numPositions());
    numFaces = unsigned(in->numPrimitives());
    numEdges = unsigned(in->position_indices.size());
    numEdgeCreases = unsigned(in->edge_creases.size());
    numVertexCreases = unsigned(in->vertex_creases.size());
    numHoles = unsigned(in->holes.size());
    numNormals = unsigned(in->numNormals());
    numTexCoords = unsigned(in->texcoords.size());
    geom.materialID = scene_in->materialID(in->material);
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
  
  ISPCSubdivMesh::~ISPCSubdivMesh ()
  {
    if (positions) delete[] positions;
    if (normals) delete[] normals;
    if (subdivlevel) delete[] subdivlevel;
    if (face_offsets) delete[] face_offsets;
  }
  
  ISPCHairSet::ISPCHairSet (TutorialScene* scene_in, RTCGeometryType type, Ref<SceneGraph::HairSetNode> in)
    : geom(CURVES), normals(nullptr), tangents(nullptr), dnormals(nullptr), hairs(nullptr), flags(nullptr), type(type)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = (Vec3fa*) in->positions[i].data();

    if (in->normals.size()) {
      normals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = in->normals[i].data();
    }

    if (in->tangents.size()) {
      tangents = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        tangents[i] = (Vec3fa*) in->tangents[i].data();
    }

    if (in->dnormals.size()) {
      dnormals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        dnormals[i] = in->dnormals[i].data();
    }
    
    hairs = (ISPCHair*) in->hairs.data();

    if (in->flags.size())
      flags = (unsigned char*)in->flags.data();
    
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numHairs = (unsigned)in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
    tessellation_rate = in->tessellation_rate;
  }

  ISPCHairSet::~ISPCHairSet() {
    delete[] positions;
    delete[] normals;
    delete[] tangents;
    delete[] dnormals;
  }

  ISPCPointSet::ISPCPointSet (TutorialScene* scene_in, RTCGeometryType type, Ref<SceneGraph::PointSetNode> in)
    : geom(POINTS), positions(nullptr), normals(nullptr), type(type)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = (Vec3fa*) in->positions[i].data();

    if (in->normals.size()) {
      normals = new Vec3fa*[in->numTimeSteps()];
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = in->normals[i].data();
    }

    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCPointSet::~ISPCPointSet () {
    if (positions) delete[] positions;
    if (normals) delete[] normals;
  }


  ISPCInstance::ISPCInstance (TutorialScene* scene, Ref<SceneGraph::TransformNode> in)
    : geom(INSTANCE), numTimeSteps(unsigned(in->spaces.size()))
  {
    spaces = (AffineSpace3fa*) alignedMalloc(in->spaces.size()*sizeof(AffineSpace3fa),16);
    child = ISPCScene::convertGeometry(scene,in->child);
    startTime  = in->spaces.time_range.lower;
    endTime    = in->spaces.time_range.upper;
    quaternion = in->spaces.quaternion;
    for (size_t i=0; i<numTimeSteps; i++)
      spaces[i] = in->spaces[i];
  }

  ISPCInstance::~ISPCInstance() {
    alignedFree(spaces);
  }

  ISPCGroup::ISPCGroup (TutorialScene* scene, Ref<SceneGraph::GroupNode> in)
    : geom(GROUP), scene(nullptr), requiredInstancingDepth(0)
  {
    numGeometries = (unsigned int) in->size();
    geometries = new ISPCGeometry*[numGeometries];
    for (size_t i=0; i<numGeometries; i++)
      geometries[i] = ISPCScene::convertGeometry(scene,in->child(i));
  }

  ISPCGroup::~ISPCGroup()
  {
    delete[] geometries;
    rtcReleaseScene(scene);
  }

  ISPCGeometry* ISPCScene::convertGeometry (TutorialScene* scene, Ref<SceneGraph::Node> in)
  {
    ISPCGeometry* geom = nullptr;
    if (in->geometry)
      return (ISPCGeometry*) in->geometry;
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = in.dynamicCast<SceneGraph::TriangleMeshNode>())
      geom = (ISPCGeometry*) new ISPCTriangleMesh(scene,mesh);
    else if (Ref<SceneGraph::QuadMeshNode> mesh = in.dynamicCast<SceneGraph::QuadMeshNode>())
      geom = (ISPCGeometry*) new ISPCQuadMesh(scene,mesh);
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = in.dynamicCast<SceneGraph::SubdivMeshNode>())
      geom = (ISPCGeometry*) new ISPCSubdivMesh(scene,mesh);
    else if (Ref<SceneGraph::HairSetNode> mesh = in.dynamicCast<SceneGraph::HairSetNode>())
      geom = (ISPCGeometry*) new ISPCHairSet(scene,mesh->type,mesh);
    else if (Ref<SceneGraph::GridMeshNode> mesh = in.dynamicCast<SceneGraph::GridMeshNode>())
      geom = (ISPCGeometry*) new ISPCGridMesh(scene,mesh); 
    else if (Ref<SceneGraph::TransformNode> mesh = in.dynamicCast<SceneGraph::TransformNode>())
      geom = (ISPCGeometry*) new ISPCInstance(scene,mesh);
    else if (Ref<SceneGraph::GroupNode> mesh = in.dynamicCast<SceneGraph::GroupNode>())
      geom = (ISPCGeometry*) new ISPCGroup(scene,mesh);
    else if (Ref<SceneGraph::PointSetNode> mesh = in.dynamicCast<SceneGraph::PointSetNode>())
      geom = (ISPCGeometry*) new ISPCPointSet(scene, mesh->type, mesh);
    else
      THROW_RUNTIME_ERROR("unknown geometry type");

    in->geometry = geom;
    return geom;
  }

  void ConvertTriangleMesh(RTCDevice device, ISPCTriangleMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    
    RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_TRIANGLE);
    mesh->geom.geometry = geom;
    
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryTimeRange(geom,mesh->startTime,mesh->endTime);
    rtcSetGeometryBuildQuality(geom, quality);
    for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, mesh->triangles, 0, sizeof(ISPCTriangle), mesh->numTriangles);
    rtcSetGeometryUserData(geom, mesh);
    
    if (assignShadersFunc) assignShadersFunc(&mesh->geom);
    rtcCommitGeometry(geom);
  }
  
  void ConvertQuadMesh(RTCDevice device, ISPCQuadMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    
    RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_QUAD);
    mesh->geom.geometry = geom;
    
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryTimeRange(geom,mesh->startTime,mesh->endTime);
    rtcSetGeometryBuildQuality(geom, quality);
    for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, mesh->quads, 0, sizeof(ISPCQuad), mesh->numQuads);
    rtcSetGeometryUserData(geom, mesh);
    
    if (assignShadersFunc) assignShadersFunc(&mesh->geom);
    rtcCommitGeometry(geom);
  }

  void ConvertGridMesh(RTCDevice device, ISPCGridMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    
    RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_GRID);
    mesh->geom.geometry = geom;
    
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryTimeRange(geom,mesh->startTime,mesh->endTime);
    rtcSetGeometryBuildQuality(geom, quality);
    for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }    
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_GRID, 0, RTC_FORMAT_GRID, mesh->grids, 0, sizeof(ISPCGrid), mesh->numGrids);
    rtcSetGeometryUserData(geom, mesh);
       
    if (assignShadersFunc) assignShadersFunc(&mesh->geom);
     rtcCommitGeometry(geom);
  }
  
  void ConvertSubdivMesh(RTCDevice device, ISPCSubdivMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SUBDIVISION);
    mesh->geom.geometry = geom;
    
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryTimeRange(geom,mesh->startTime,mesh->endTime);
    rtcSetGeometryBuildQuality(geom, quality);
    for (unsigned int i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = FIXED_EDGE_TESSELLATION_VALUE;
    for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, mesh->subdivlevel, 0, sizeof(float), mesh->numEdges);

    /* create geometry topology */
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, mesh->position_indices, 0, sizeof(unsigned int), mesh->numEdges);
    rtcSetGeometrySubdivisionMode(geom, 0, mesh->position_subdiv_mode);

    /* set normal buffers and optionally normal topology */
    if (mesh->normals) {
      rtcSetGeometryVertexAttributeCount(geom,2);
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT3, mesh->normals[0], 0, sizeof(Vec3fa), mesh->numNormals);
      if (mesh->normal_indices) {
        rtcSetGeometryTopologyCount(geom,2);
        rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 1, RTC_FORMAT_UINT, mesh->normal_indices, 0, sizeof(unsigned int), mesh->numEdges);
        rtcSetGeometryVertexAttributeTopology(geom, 1, 1);
        rtcSetGeometrySubdivisionMode(geom, 1, mesh->normal_subdiv_mode);
      }
    }

    /* set texcoord buffer and optionally texcoord topology */
    if (mesh->texcoords) {
      rtcSetGeometryVertexAttributeCount(geom,3);
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 2, RTC_FORMAT_FLOAT2, mesh->texcoords, 0, sizeof(Vec2f), mesh->numTexCoords);
      if (mesh->texcoord_indices) {
        rtcSetGeometryTopologyCount(geom,3);
        rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 2, RTC_FORMAT_UINT, mesh->texcoord_indices, 0, sizeof(unsigned int), mesh->numEdges);
        rtcSetGeometryVertexAttributeTopology(geom, 2, 2);
        rtcSetGeometrySubdivisionMode(geom, 2, mesh->texcoord_subdiv_mode);
      }
    }

    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,                 0, RTC_FORMAT_UINT,   mesh->verticesPerFace,       0, sizeof(unsigned int),   mesh->numFaces);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_HOLE,                 0, RTC_FORMAT_UINT,   mesh->holes,                 0, sizeof(unsigned int),   mesh->numHoles);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_INDEX,    0, RTC_FORMAT_UINT2,  mesh->edge_creases,          0, 2*sizeof(unsigned int), mesh->numEdgeCreases);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT,   0, RTC_FORMAT_FLOAT,  mesh->edge_crease_weights,   0, sizeof(float),          mesh->numEdgeCreases);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX,  0, RTC_FORMAT_UINT,   mesh->vertex_creases,        0, sizeof(unsigned int),   mesh->numVertexCreases);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT,  mesh->vertex_crease_weights, 0, sizeof(float),          mesh->numVertexCreases);
    rtcSetGeometryUserData(geom, mesh);
    
    if (assignShadersFunc) assignShadersFunc(&mesh->geom);
    rtcCommitGeometry(geom);
  }
  
  void ConvertCurveGeometry(RTCDevice device, ISPCHairSet* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;

    RTCGeometry geom = rtcNewGeometry(device, mesh->type);
    mesh->geom.geometry = geom;
    
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryTimeRange(geom,mesh->startTime,mesh->endTime);
    rtcSetGeometryBuildQuality(geom, quality);

    for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT4, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    
    if (mesh->normals) {
      for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
        rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_NORMAL, t, RTC_FORMAT_FLOAT3, mesh->normals[t], 0, sizeof(Vec3fa), mesh->numVertices);
      }
    }
    
    if (mesh->tangents) {
      for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
        rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_TANGENT, t, RTC_FORMAT_FLOAT4, mesh->tangents[t], 0, sizeof(Vec3fa), mesh->numVertices);
      }
    }

    if (mesh->dnormals) {
      for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
        rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_NORMAL_DERIVATIVE, t, RTC_FORMAT_FLOAT3, mesh->dnormals[t], 0, sizeof(Vec3fa), mesh->numVertices);
      }
    }
    
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, mesh->hairs, 0, sizeof(ISPCHair), mesh->numHairs);
    if (mesh->type != RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE && mesh->type != RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE && mesh->type != RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE) {
      rtcSetGeometryTessellationRate(geom,(float)mesh->tessellation_rate);
    }
    
#if RTC_MIN_WIDTH
    if (g_min_width_max_radius_scale >= 1.0f)
      rtcSetGeometryMaxRadiusScale(geom,g_min_width_max_radius_scale);
#endif

    if (mesh->flags) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FLAGS, 0, RTC_FORMAT_UCHAR, mesh->flags, 0, sizeof(unsigned char), mesh->numHairs);
    }
    rtcSetGeometryUserData(geom, mesh);
   
    if (assignShadersFunc) assignShadersFunc(&mesh->geom);
    rtcCommitGeometry(geom);
  }

  void ConvertPoints(RTCDevice device, ISPCPointSet* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    
    RTCGeometry geom = rtcNewGeometry(device, mesh->type);
    mesh->geom.geometry = geom;
    
    rtcSetGeometryTimeStepCount(geom,mesh->numTimeSteps);
    rtcSetGeometryTimeRange(geom,mesh->startTime,mesh->endTime);
    rtcSetGeometryBuildQuality(geom, quality);

    for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT4, mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    if (mesh->normals) {
      for (unsigned int t=0; t<mesh->numTimeSteps; t++) {
        rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_NORMAL, t, RTC_FORMAT_FLOAT3, mesh->normals[t], 0, sizeof(Vec3fa), mesh->numVertices);
      }
    }
#if RTC_MIN_WIDTH
    if (g_min_width_max_radius_scale >= 1.0f)
      rtcSetGeometryMaxRadiusScale(geom,g_min_width_max_radius_scale);
#endif
      
    rtcSetGeometryUserData(geom, mesh);
    
    if (assignShadersFunc) assignShadersFunc(&mesh->geom);
    rtcCommitGeometry(geom);
  }

  unsigned int ConvertInstance(RTCDevice device, ISPCInstance* instance, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth);

  unsigned int ConvertGroup(RTCDevice device, ISPCGroup* group, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth)
  {
    if (group->geom.visited) return group->requiredInstancingDepth;
    group->geom.visited = true;
    
    RTCScene scene = rtcNewScene(device);
    rtcSetSceneFlags(scene, flags);

    unsigned int requiredInstancingDepth = 0;
    for (unsigned int geomID=0; geomID<group->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = group->geometries[geomID];
      if (geometry->type == SUBDIV_MESH)
        ConvertSubdivMesh(device,(ISPCSubdivMesh*) geometry, quality, flags);
      else if (geometry->type == TRIANGLE_MESH)
        ConvertTriangleMesh(device,(ISPCTriangleMesh*) geometry, quality, flags);
      else if (geometry->type == QUAD_MESH)
        ConvertQuadMesh(device,(ISPCQuadMesh*) geometry, quality, flags);
      else if (geometry->type == CURVES)
        ConvertCurveGeometry(device,(ISPCHairSet*) geometry, quality, flags);
      else if (geometry->type == GRID_MESH)
        ConvertGridMesh(device,(ISPCGridMesh*) geometry, quality, flags);
      else if (geometry->type == POINTS)
        ConvertPoints(device,(ISPCPointSet*) geometry, quality, flags);
      else if (geometry->type == INSTANCE) {
        unsigned int reqDepth = ConvertInstance(device,(ISPCInstance*) geometry, quality, flags, depth);
        requiredInstancingDepth = max(requiredInstancingDepth, reqDepth);
      }
      else
        assert(false);

      rtcAttachGeometryByID(scene,geometry->geometry,geomID);
    }
    rtcCommitScene(scene);
    
    group->scene = scene;
    group->requiredInstancingDepth = requiredInstancingDepth;

    return requiredInstancingDepth;
  }

  unsigned int ConvertInstance(RTCDevice device, ISPCInstance* instance, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth)
  {
    if (instance->child->type != GROUP)
      THROW_RUNTIME_ERROR("invalid scene structure");

    ISPCGroup* group = (ISPCGroup*) instance->child;
    unsigned int requiredInstancingDepth = 1+ConvertGroup(device, group, quality, flags, depth+1);
    RTCScene scene_inst = group->scene;

    if (depth + requiredInstancingDepth > RTC_MAX_INSTANCE_LEVEL_COUNT)
      THROW_RUNTIME_ERROR("scene instancing depth is too large");

    if (instance->geom.visited) return requiredInstancingDepth;
    instance->geom.visited = true;
      
    if (instance->numTimeSteps == 1 || g_animation_mode)
    {
      RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE);
      instance->geom.geometry = geom;
      
      rtcSetGeometryInstancedScene(geom,scene_inst);
      rtcSetGeometryTimeStepCount(geom,1);
      if (instance->quaternion) {
        QuaternionDecomposition qd = quaternionDecomposition(instance->spaces[0]);
        rtcSetGeometryTransformQuaternion(geom,0,(RTCQuaternionDecomposition*)&qd);
      } else {
        rtcSetGeometryTransform(geom,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,&instance->spaces[0].l.vx.x);
      }
      rtcSetGeometryUserData(geom, instance);
      rtcCommitGeometry(geom);
      
      return requiredInstancingDepth;
    }
    else
    {
      RTCGeometry geom = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE);
      instance->geom.geometry = geom;
      
      rtcSetGeometryInstancedScene(geom,scene_inst);
      rtcSetGeometryTimeStepCount(geom,instance->numTimeSteps);
      rtcSetGeometryTimeRange(geom,instance->startTime,instance->endTime);
      for (size_t t=0; t<instance->numTimeSteps; t++) {
        if (instance->quaternion) {
          QuaternionDecomposition qd = quaternionDecomposition(instance->spaces[t]);
          rtcSetGeometryTransformQuaternion(geom,t,(RTCQuaternionDecomposition*)&qd);
        } else {
          rtcSetGeometryTransform(geom,(unsigned int)t,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,&instance->spaces[t].l.vx.x);
        }
      }
      rtcSetGeometryUserData(geom, instance);
      rtcCommitGeometry(geom);
      
      return requiredInstancingDepth;
    }
  }
 
  void UpdateInstance(ISPCInstance* instance, float time)
  {
    if (instance->child->type != GROUP)
      THROW_RUNTIME_ERROR("invalid scene structure");
    
    if (instance->numTimeSteps <= 1)
      return;

    int numTimeSteps = instance->numTimeSteps;
    BBox1f time_range(instance->startTime, instance->endTime);
    time = frac((time-time_range.lower)/time_range.size());
    time = (numTimeSteps-1)*time;
    int   itime = (int)floor(time);
    itime = min(max(itime,0),(int)numTimeSteps-2);
    float ftime = time - (float)itime;
    
    const AffineSpace3fa xfm0 = instance->spaces[itime+0];
    const AffineSpace3fa xfm1 = instance->spaces[itime+1];
    const AffineSpace3fa xfm  = lerp(xfm0,xfm1,ftime);

    RTCGeometry geom = instance->geom.geometry;
    if (instance->quaternion) {
      const QuaternionDecomposition qd = quaternionDecomposition(xfm);
      rtcSetGeometryTransformQuaternion(geom,0,(RTCQuaternionDecomposition*)&qd);
    } else {
      rtcSetGeometryTransform(geom,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,&xfm.l.vx.x);
    }
    rtcCommitGeometry(geom);
  }
  
  extern "C" RTCScene ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    RTCScene scene = rtcNewScene(g_device);
    rtcSetSceneFlags(scene, flags);
    scene_in->scene = scene;
    
    for (unsigned int geomID=0; geomID<scene_in->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = scene_in->geometries[geomID];
      if (geometry->type == SUBDIV_MESH)
        ConvertSubdivMesh(g_device,(ISPCSubdivMesh*) geometry, quality, flags);
      else if (geometry->type == TRIANGLE_MESH)
        ConvertTriangleMesh(g_device,(ISPCTriangleMesh*) geometry, quality, flags);
      else if (geometry->type == QUAD_MESH)
        ConvertQuadMesh(g_device,(ISPCQuadMesh*) geometry, quality, flags);
      else if (geometry->type == CURVES)
        ConvertCurveGeometry(g_device,(ISPCHairSet*) geometry, quality, flags);
      else if (geometry->type == GRID_MESH)
        ConvertGridMesh(g_device,(ISPCGridMesh*) geometry, quality, flags);
      else if (geometry->type == POINTS)
        ConvertPoints(g_device,(ISPCPointSet*) geometry, quality, flags);
      else if (geometry->type == INSTANCE)
        ConvertInstance(g_device, (ISPCInstance*) geometry, quality, flags, 0);
      else
        assert(false);

      rtcAttachGeometryByID(scene,geometry->geometry,geomID);
    }

    Application::instance->log(1,"creating Embree objects done");
    return scene;
  }

  extern "C" void UpdateScene(ISPCScene* scene_in, float time)
  {
    for (unsigned int geomID=0; geomID<scene_in->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = scene_in->geometries[geomID];
      if (geometry->type != INSTANCE) continue;
      UpdateInstance((ISPCInstance*) geometry, time);
    }

    rtcCommitScene(scene_in->scene);

    TutorialScene* tutorial_scene = (TutorialScene*) scene_in->tutorialScene;
    for (unsigned int i=0; i<scene_in->numLights; i++)
      ISPCScene::updateLight(tutorial_scene->lights[i]->get(time), scene_in->lights[i]);
  }
}
