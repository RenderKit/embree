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
  
  ISPCScene::ISPCScene(RTCDevice device, TutorialScene* in)
    : scene(rtcNewScene(device)), tutorialScene(in)
  {
    SceneGraph::opaque_geometry_destruction = (void(*)(void*)) deleteGeometry;
    
    geometries = new ISPCGeometry*[in->geometries.size()];
    for (size_t i=0; i<in->geometries.size(); i++)
      geometries[i] = convertGeometry(device,in,in->geometries[i]);
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

  void ISPCScene::commit()
  {
    for (unsigned int geomID=0; geomID<numGeometries; geomID++)
    {
      ISPCGeometry* geometry = geometries[geomID];
      rtcAttachGeometryByID(scene,geometry->geometry,geomID);
    }

    rtcCommitScene(scene);
  }
  
  ISPCTriangleMesh::ISPCTriangleMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::TriangleMeshNode> in) 
    : geom(TRIANGLE_MESH), positions(nullptr), normals(nullptr)
  {
    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_TRIANGLE);
    
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

  void ISPCTriangleMesh::commit()
  {
    RTCGeometry g = geom.geometry;
    rtcSetGeometryTimeStepCount(g,numTimeSteps);
    rtcSetGeometryTimeRange(g,startTime,endTime);

    for (unsigned int t=0; t<numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, positions[t], 0, sizeof(Vec3fa), numVertices);
    }
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, triangles, 0, sizeof(ISPCTriangle), numTriangles);
    rtcSetGeometryUserData(g, this);
    
    if (assignShadersFunc) assignShadersFunc(&geom);
    rtcCommitGeometry(g);
  }
  
  ISPCQuadMesh::ISPCQuadMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::QuadMeshNode> in) 
    : geom(QUAD_MESH), positions(nullptr), normals(nullptr)
  {
    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_QUAD);
    
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

  void ISPCQuadMesh::commit()
  {
    RTCGeometry g = geom.geometry;
    rtcSetGeometryTimeStepCount(g,numTimeSteps);
    rtcSetGeometryTimeRange(g,startTime,endTime);
  
    for (unsigned int t=0; t<numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, positions[t], 0, sizeof(Vec3fa), numVertices);
    }
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, quads, 0, sizeof(ISPCQuad), numQuads);
    rtcSetGeometryUserData(g, this);
    
    if (assignShadersFunc) assignShadersFunc(&geom);
    rtcCommitGeometry(g);
  }
  
  ISPCGridMesh::ISPCGridMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::GridMeshNode> in) 
    : geom(GRID_MESH), positions(nullptr)
  {
    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_GRID);
    
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

  void ISPCGridMesh::commit()
  {
    RTCGeometry g = geom.geometry;
    rtcSetGeometryTimeStepCount(g,numTimeSteps);
    rtcSetGeometryTimeRange(g,startTime,endTime);
    
    for (unsigned int t=0; t<numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, positions[t], 0, sizeof(Vec3fa), numVertices);
    }    
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_GRID, 0, RTC_FORMAT_GRID, grids, 0, sizeof(ISPCGrid), numGrids);
    rtcSetGeometryUserData(g, this);
       
    if (assignShadersFunc) assignShadersFunc(&geom);
    rtcCommitGeometry(g);
  }
  
  ISPCSubdivMesh::ISPCSubdivMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::SubdivMeshNode> in) 
    : geom(SUBDIV_MESH), positions(nullptr), normals(nullptr)
  {
    geom.geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SUBDIVISION);
    
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

  void ISPCSubdivMesh::commit()
  {
    RTCGeometry g = geom.geometry;
    rtcSetGeometryTimeStepCount(g,numTimeSteps);
    rtcSetGeometryTimeRange(g,startTime,endTime);

    for (unsigned int i=0; i<numEdges; i++) subdivlevel[i] = FIXED_EDGE_TESSELLATION_VALUE;
    for (unsigned int t=0; t<numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT3, positions[t], 0, sizeof(Vec3fa), numVertices);
    }
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, subdivlevel, 0, sizeof(float), numEdges);

    /* create geometry topology */
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, position_indices, 0, sizeof(unsigned int), numEdges);
    rtcSetGeometrySubdivisionMode(g, 0, position_subdiv_mode);

    /* set normal buffers and optionally normal topology */
    if (normals) {
      rtcSetGeometryVertexAttributeCount(g,2);
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 1, RTC_FORMAT_FLOAT3, normals[0], 0, sizeof(Vec3fa), numNormals);
      if (normal_indices) {
        rtcSetGeometryTopologyCount(g,2);
        rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 1, RTC_FORMAT_UINT, normal_indices, 0, sizeof(unsigned int), numEdges);
        rtcSetGeometryVertexAttributeTopology(g, 1, 1);
        rtcSetGeometrySubdivisionMode(g, 1, normal_subdiv_mode);
      }
    }

    /* set texcoord buffer and optionally texcoord topology */
    if (texcoords) {
      rtcSetGeometryVertexAttributeCount(g,3);
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE, 2, RTC_FORMAT_FLOAT2, texcoords, 0, sizeof(Vec2f), numTexCoords);
      if (texcoord_indices) {
        rtcSetGeometryTopologyCount(g,3);
        rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 2, RTC_FORMAT_UINT, texcoord_indices, 0, sizeof(unsigned int), numEdges);
        rtcSetGeometryVertexAttributeTopology(g, 2, 2);
        rtcSetGeometrySubdivisionMode(g, 2, texcoord_subdiv_mode);
      }
    }

    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_FACE,                 0, RTC_FORMAT_UINT,   verticesPerFace,       0, sizeof(unsigned int),   numFaces);
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_HOLE,                 0, RTC_FORMAT_UINT,   holes,                 0, sizeof(unsigned int),   numHoles);
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_EDGE_CREASE_INDEX,    0, RTC_FORMAT_UINT2,  edge_creases,          0, 2*sizeof(unsigned int), numEdgeCreases);
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT,   0, RTC_FORMAT_FLOAT,  edge_crease_weights,   0, sizeof(float),          numEdgeCreases);
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX,  0, RTC_FORMAT_UINT,   vertex_creases,        0, sizeof(unsigned int),   numVertexCreases);
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT,  vertex_crease_weights, 0, sizeof(float),          numVertexCreases);
    rtcSetGeometryUserData(g, this);
    
    if (assignShadersFunc) assignShadersFunc(&geom);
    rtcCommitGeometry(g);
  }
  
  ISPCHairSet::ISPCHairSet (RTCDevice device, TutorialScene* scene_in, RTCGeometryType type, Ref<SceneGraph::HairSetNode> in)
    : geom(CURVES), normals(nullptr), tangents(nullptr), dnormals(nullptr), hairs(nullptr), flags(nullptr), type(type)
  {
    geom.geometry = rtcNewGeometry(device, type);
    
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

  void ISPCHairSet::commit()
  {
    RTCGeometry g = geom.geometry;
    rtcSetGeometryTimeStepCount(g,numTimeSteps);
    rtcSetGeometryTimeRange(g,startTime,endTime);

    for (unsigned int t=0; t<numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT4, positions[t], 0, sizeof(Vec3fa), numVertices);
    }
    
    if (normals) {
      for (unsigned int t=0; t<numTimeSteps; t++) {
        rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_NORMAL, t, RTC_FORMAT_FLOAT3, normals[t], 0, sizeof(Vec3fa), numVertices);
      }
    }
    
    if (tangents) {
      for (unsigned int t=0; t<numTimeSteps; t++) {
        rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_TANGENT, t, RTC_FORMAT_FLOAT4, tangents[t], 0, sizeof(Vec3fa), numVertices);
      }
    }

    if (dnormals) {
      for (unsigned int t=0; t<numTimeSteps; t++) {
        rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_NORMAL_DERIVATIVE, t, RTC_FORMAT_FLOAT3, dnormals[t], 0, sizeof(Vec3fa), numVertices);
      }
    }
    
    rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, hairs, 0, sizeof(ISPCHair), numHairs);
    if (type != RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE && type != RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE && type != RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE) {
      rtcSetGeometryTessellationRate(g,(float)tessellation_rate);
    }
    
#if RTC_MIN_WIDTH
    if (g_min_width_max_radius_scale >= 1.0f)
      rtcSetGeometryMaxRadiusScale(g,g_min_width_max_radius_scale);
#endif

    if (flags) {
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_FLAGS, 0, RTC_FORMAT_UCHAR, flags, 0, sizeof(unsigned char), numHairs);
    }
    rtcSetGeometryUserData(g, this);
   
    if (assignShadersFunc) assignShadersFunc(&geom);
    rtcCommitGeometry(g);
  }

  ISPCPointSet::ISPCPointSet (RTCDevice device, TutorialScene* scene_in, RTCGeometryType type, Ref<SceneGraph::PointSetNode> in)
    : geom(POINTS), positions(nullptr), normals(nullptr), type(type)
  {
    geom.geometry = rtcNewGeometry(device, type);
    
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

  void ISPCPointSet::commit()
  {
    RTCGeometry g = geom.geometry;
    rtcSetGeometryTimeStepCount(g,numTimeSteps);
    rtcSetGeometryTimeRange(g,startTime,endTime);
    
    for (unsigned int t=0; t<numTimeSteps; t++) {
      rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, t, RTC_FORMAT_FLOAT4, positions[t], 0, sizeof(Vec3fa), numVertices);
    }
    if (normals) {
      for (unsigned int t=0; t<numTimeSteps; t++) {
        rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_NORMAL, t, RTC_FORMAT_FLOAT3, normals[t], 0, sizeof(Vec3fa), numVertices);
      }
    }
#if RTC_MIN_WIDTH
    if (g_min_width_max_radius_scale >= 1.0f)
      rtcSetGeometryMaxRadiusScale(g,g_min_width_max_radius_scale);
#endif
      
    rtcSetGeometryUserData(g, this);
    
    if (assignShadersFunc) assignShadersFunc(&geom);
    rtcCommitGeometry(g);
  }

  ISPCInstance::ISPCInstance (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::TransformNode> in)
    : geom(INSTANCE), child(nullptr), startTime(0.0f), endTime(1.0f), numTimeSteps(1), quaternion(false), spaces(nullptr)
  {
    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE);
    
    if (g_animation_mode)
    {
      spaces = (AffineSpace3fa*) alignedMalloc(sizeof(AffineSpace3fa),16);
      child = ISPCScene::convertGeometry(device,scene,in->child);
      spaces[0] = in->get(0.0f);
    }
    else
    {
      spaces = (AffineSpace3fa*) alignedMalloc(in->spaces.size()*sizeof(AffineSpace3fa),16);
      child = ISPCScene::convertGeometry(device,scene,in->child);
      startTime  = in->spaces.time_range.lower;
      endTime    = in->spaces.time_range.upper;
      numTimeSteps = unsigned(in->spaces.size());
      quaternion = in->spaces.quaternion;
      for (size_t i=0; i<numTimeSteps; i++)
        spaces[i] = in->spaces[i];
    }
  }

  ISPCInstance::~ISPCInstance() {
    alignedFree(spaces);
  }

  void ISPCInstance::commit()
  {
    if (child->type != GROUP)
      THROW_RUNTIME_ERROR("invalid scene structure");

    ISPCGroup* group = (ISPCGroup*) child;
    RTCScene scene_inst = group->scene;
    
    if (numTimeSteps == 1 || g_animation_mode)
    {
      RTCGeometry g = geom.geometry;
      rtcSetGeometryInstancedScene(g,scene_inst);
      rtcSetGeometryTimeStepCount(g,1);
      if (quaternion) {
        QuaternionDecomposition qd = quaternionDecomposition(spaces[0]);
        rtcSetGeometryTransformQuaternion(g,0,(RTCQuaternionDecomposition*)&qd);
      } else {
        rtcSetGeometryTransform(g,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,&spaces[0].l.vx.x);
      }
      rtcSetGeometryUserData(g, this);
      rtcCommitGeometry(g);
    }
    else
    {
      RTCGeometry g = geom.geometry;
      rtcSetGeometryInstancedScene(g,scene_inst);
      rtcSetGeometryTimeStepCount(g,numTimeSteps);
      rtcSetGeometryTimeRange(g,startTime,endTime);
      for (size_t t=0; t<numTimeSteps; t++) {
        if (quaternion) {
          QuaternionDecomposition qd = quaternionDecomposition(spaces[t]);
          rtcSetGeometryTransformQuaternion(g,t,(RTCQuaternionDecomposition*)&qd);
        } else {
          rtcSetGeometryTransform(g,(unsigned int)t,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,&spaces[t].l.vx.x);
        }
      }
      rtcSetGeometryUserData(g, this);
      rtcCommitGeometry(g);
    }
  }

  ISPCGroup::ISPCGroup (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::GroupNode> in)
    : geom(GROUP), scene(rtcNewScene(device)), requiredInstancingDepth(0)
  {
    numGeometries = (unsigned int) in->size();
    geometries = new ISPCGeometry*[numGeometries];
    for (size_t i=0; i<numGeometries; i++)
      geometries[i] = ISPCScene::convertGeometry(device,scene,in->child(i));
  }

  ISPCGroup::~ISPCGroup()
  {
    delete[] geometries;
    rtcReleaseScene(scene);
  }

  void ISPCGroup::commit()
  {
    for (unsigned int geomID=0; geomID<numGeometries; geomID++)
    {
      ISPCGeometry* geometry = geometries[geomID];
      rtcAttachGeometryByID(scene,geometry->geometry,geomID);
    }
    rtcCommitScene(scene);
  }

  ISPCGeometry* ISPCScene::convertGeometry (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::Node> in)
  {
    ISPCGeometry* geom = nullptr;
    if (in->geometry)
      return (ISPCGeometry*) in->geometry;
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = in.dynamicCast<SceneGraph::TriangleMeshNode>())
      geom = (ISPCGeometry*) new ISPCTriangleMesh(device,scene,mesh);
    else if (Ref<SceneGraph::QuadMeshNode> mesh = in.dynamicCast<SceneGraph::QuadMeshNode>())
      geom = (ISPCGeometry*) new ISPCQuadMesh(device,scene,mesh);
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = in.dynamicCast<SceneGraph::SubdivMeshNode>())
      geom = (ISPCGeometry*) new ISPCSubdivMesh(device,scene,mesh);
    else if (Ref<SceneGraph::HairSetNode> mesh = in.dynamicCast<SceneGraph::HairSetNode>())
      geom = (ISPCGeometry*) new ISPCHairSet(device,scene,mesh->type,mesh);
    else if (Ref<SceneGraph::GridMeshNode> mesh = in.dynamicCast<SceneGraph::GridMeshNode>())
      geom = (ISPCGeometry*) new ISPCGridMesh(device,scene,mesh); 
    else if (Ref<SceneGraph::TransformNode> mesh = in.dynamicCast<SceneGraph::TransformNode>())
      geom = (ISPCGeometry*) new ISPCInstance(device,scene,mesh);
    else if (Ref<SceneGraph::GroupNode> mesh = in.dynamicCast<SceneGraph::GroupNode>())
      geom = (ISPCGeometry*) new ISPCGroup(device,scene,mesh);
    else if (Ref<SceneGraph::PointSetNode> mesh = in.dynamicCast<SceneGraph::PointSetNode>())
      geom = (ISPCGeometry*) new ISPCPointSet(device,scene, mesh->type, mesh);
    else
      THROW_RUNTIME_ERROR("unknown geometry type");

    in->geometry = geom;
    return geom;
  }

  void ConvertTriangleMesh(RTCDevice device, ISPCTriangleMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }
  
  void ConvertQuadMesh(RTCDevice device, ISPCQuadMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }

  void ConvertGridMesh(RTCDevice device, ISPCGridMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }
  
  void ConvertSubdivMesh(RTCDevice device, ISPCSubdivMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }
  
  void ConvertCurveGeometry(RTCDevice device, ISPCHairSet* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }

  void ConvertPoints(RTCDevice device, ISPCPointSet* mesh, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    if (mesh->geom.visited) return;
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }

  unsigned int ConvertInstance(RTCDevice device, ISPCInstance* instance, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth);

  unsigned int ConvertGroup(RTCDevice device, ISPCGroup* group, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth)
  {
    if (group->geom.visited) return group->requiredInstancingDepth;
    group->geom.visited = true;
    
    RTCScene scene = group->scene;
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
    }
    
    group->commit();
    
    group->requiredInstancingDepth = requiredInstancingDepth;
    return requiredInstancingDepth;
  }

  unsigned int ConvertInstance(RTCDevice device, ISPCInstance* instance, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth)
  {
    if (instance->child->type != GROUP)
      THROW_RUNTIME_ERROR("invalid scene structure");

    ISPCGroup* group = (ISPCGroup*) instance->child;
    unsigned int requiredInstancingDepth = 1+ConvertGroup(device, group, quality, flags, depth+1);

    if (depth + requiredInstancingDepth > RTC_MAX_INSTANCE_LEVEL_COUNT)
      THROW_RUNTIME_ERROR("scene instancing depth is too large");

    if (instance->geom.visited) return requiredInstancingDepth;
    instance->geom.visited = true;
    instance->commit();
    
    return requiredInstancingDepth;
  }

  extern "C" RTCScene ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCBuildQuality quality, RTCSceneFlags flags)
  {
    RTCScene scene = scene_in->scene;
    rtcSetSceneFlags(scene, flags);
    
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
    TutorialScene* tutorial_scene = (TutorialScene*) scene_in->tutorialScene;
    if (!tutorial_scene) return;
    if (!g_animation_mode) return;
    
    for (unsigned int geomID=0; geomID<scene_in->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = scene_in->geometries[geomID];
      if (geometry->type != INSTANCE) continue;
      ISPCInstance* inst = (ISPCInstance*) geometry;

      Ref<SceneGraph::TransformNode> node = tutorial_scene->geometries[geomID].dynamicCast<SceneGraph::TransformNode>();
      assert(node);
      inst->spaces[0] = node->get(time);
      inst->commit();
    }

    rtcCommitScene(scene_in->scene);

    for (unsigned int i=0; i<scene_in->numLights; i++)
      ISPCScene::updateLight(tutorial_scene->lights[i]->get(time), scene_in->lights[i]);
  }
}
