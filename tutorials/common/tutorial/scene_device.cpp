// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "scene_device.h"
#include "application.h"

#define FIXED_EDGE_TESSELLATION_VALUE 4

namespace embree
{
  RTCFeatureFlags operator |= (RTCFeatureFlags& a, RTCFeatureFlags b) {
    return a = (RTCFeatureFlags) (a | b);
  }

  std::string to_string(RTCFeatureFlags features)
  {
    std::string out = "";
    if (features & RTC_FEATURE_FLAG_MOTION_BLUR) out += "MOTION_BLUR";
    if (features & RTC_FEATURE_FLAG_TRIANGLE) out += "TRIANGLE ";
    if (features & RTC_FEATURE_FLAG_QUAD) out += "QUAD ";
    if (features & RTC_FEATURE_FLAG_GRID) out += "GRID ";
    if (features & RTC_FEATURE_FLAG_SUBDIVISION) out += "SUBDIVISION ";
    if (features & RTC_FEATURE_FLAG_CONE_LINEAR_CURVE) out += "CONE_LINEAR_CURVE ";
    if (features & RTC_FEATURE_FLAG_ROUND_LINEAR_CURVE ) out += "ROUND_LINEAR_CURVE ";
    if (features & RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE) out += "FLAT_LINEAR_CURVE ";
    if (features & RTC_FEATURE_FLAG_ROUND_BEZIER_CURVE) out += "ROUND_BEZIER_CURVE ";
    if (features & RTC_FEATURE_FLAG_FLAT_BEZIER_CURVE) out += "FLAT_BEZIER_CURVE ";
    if (features & RTC_FEATURE_FLAG_NORMAL_ORIENTED_BEZIER_CURVE) out += "NORMAL_ORIENTED_BEZIER_CURVE ";
    if (features & RTC_FEATURE_FLAG_ROUND_BSPLINE_CURVE) out += "ROUND_BSPLINE_CURVE ";
    if (features & RTC_FEATURE_FLAG_FLAT_BSPLINE_CURVE) out += "FLAT_BSPLINE_CURVE ";
    if (features & RTC_FEATURE_FLAG_NORMAL_ORIENTED_BSPLINE_CURVE) out += "NORMAL_ORIENTED_BSPLINE_CURVE ";
    if (features & RTC_FEATURE_FLAG_ROUND_HERMITE_CURVE) out += "ROUND_HERMITE_CURVE ";
    if (features & RTC_FEATURE_FLAG_FLAT_HERMITE_CURVE) out += "FLAT_HERMITE_CURVE ";
    if (features & RTC_FEATURE_FLAG_NORMAL_ORIENTED_HERMITE_CURVE) out += "NORMAL_ORIENTED_HERMITE_CURVE ";
    if (features & RTC_FEATURE_FLAG_ROUND_CATMULL_ROM_CURVE) out += "ROUND_CATMULL_ROM_CURVE ";
    if (features & RTC_FEATURE_FLAG_FLAT_CATMULL_ROM_CURVE) out += "FLAT_CATMULL_ROM_CURVE ";
    if (features & RTC_FEATURE_FLAG_NORMAL_ORIENTED_CATMULL_ROM_CURVE) out += "NORMAL_ORIENTED_CATMULL_ROM_CURVE ";
    if (features & RTC_FEATURE_FLAG_SPHERE_POINT) out += "SPHERE_POINT ";
    if (features & RTC_FEATURE_FLAG_DISC_POINT) out += "DISC_POINT ";
    if (features & RTC_FEATURE_FLAG_ORIENTED_DISC_POINT) out += "ORIENTED_DISC_POINT ";
    if (features & RTC_FEATURE_FLAG_INSTANCE) out += "INSTANCE ";
    if (features & RTC_FEATURE_FLAG_INSTANCE_ARRAY) out += "INSTANCE_ARRAY ";
    if (features & RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS) out += "FILTER_FUNCTION_IN_ARGUMENTS ";
    if (features & RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY) out += "FILTER_FUNCTION_IN_GEOMETRY ";
    if (features & RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS) out += "USER_GEOMETRY_CALLBACK_IN_ARGUMENTS ";
    if (features & RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY) out += "USER_GEOMETRY_CALLBACK_IN_GEOMETRY ";
    if (out == "") return "NONE";
    return out;
  }
  
  extern "C" {
    int g_instancing_mode = SceneGraph::INSTANCING_NONE;
    float g_min_width_max_radius_scale = 1.0f;
    AssignShaderTy assignShadersFunc = nullptr;
  }

  template<typename Ty>
  Ty* copyArrayToUSM(const avector<Ty>& in) {
    Ty* out = (Ty*)alignedUSMMalloc(in.size()*sizeof(Ty));
    memcpy((void*)out,in.data(),in.size()*sizeof(Ty));
    return out;
  }

  template<typename Ty>
  Ty* copyArrayToUSM(const std::vector<Ty>& in) {
    Ty* out = (Ty*)alignedUSMMalloc(in.size()*sizeof(Ty));
    memcpy((void*)out,in.data(),in.size()*sizeof(Ty));
    return out;
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
    case INSTANCE_ARRAY: delete (ISPCInstanceArray*) geom; break;
    case GROUP: delete (ISPCGroup*) geom; break;
    case QUAD_MESH: delete (ISPCQuadMesh*) geom; break;
    case CURVES: delete (ISPCHairSet*) geom; break;
    case GRID_MESH: delete (ISPCGridMesh*) geom; break;
    case POINTS: delete (ISPCPointSet*) geom; break;
    default: assert(false); break;
    }
  }

  ISPCScene::ISPCScene(RTCDevice device, unsigned int numGeometries, unsigned int numMaterials, unsigned int numLights)
    : scene(rtcNewScene(device)), geometries(nullptr), materials(nullptr), numGeometries(numGeometries), numMaterials(numMaterials), lights(0), numLights(numLights), tutorialScene(nullptr)
  {
    geometries = (ISPCGeometry**) alignedUSMMalloc(numGeometries*sizeof(ISPCGeometry*));
    for (size_t i=0; i<numGeometries; i++) geometries[i] = nullptr;
    
    materials = (ISPCMaterial**) alignedUSMMalloc(numMaterials*sizeof(ISPCMaterial*));
    for (size_t i=0; i<numMaterials; i++) materials[i] = nullptr;
    
    lights = (Light**) alignedUSMMalloc(numLights*sizeof(Light*));
    for (size_t i=0; i<numLights; i++) lights[i] = nullptr;
  }
  
  ISPCScene::ISPCScene(RTCDevice device, TutorialScene* in)
    : scene(rtcNewScene(device)), tutorialScene(in)
  {
    SceneGraph::opaque_geometry_destruction = (void(*)(void*)) deleteGeometry;

    geometries = (ISPCGeometry**) alignedUSMMalloc(sizeof(ISPCGeometry*)*in->geometries.size());

    for (size_t i=0; i<in->geometries.size(); i++)
      geometries[i] = convertGeometry(device,in,in->geometries[i]);
    numGeometries = unsigned(in->geometries.size());
    
    materials = (ISPCMaterial**) alignedUSMMalloc(sizeof(ISPCMaterial*)*in->materials.size());
    for (size_t i=0; i<in->materials.size(); i++)
      materials[i] = (ISPCMaterial*) in->materials[i]->material();
    numMaterials = unsigned(in->materials.size());
    
    lights = (Light**) alignedUSMMalloc(sizeof(Light*)*in->lights.size());
    numLights = 0;
    for (size_t i=0; i<in->lights.size(); i++)
    {
      Light* light = convertLight(in->lights[i]->get(0.0f));
      if (light) lights[numLights++] = light;
    }
  }

  ISPCScene::~ISPCScene()
  {
    alignedUSMFree(geometries);
    alignedUSMFree(materials);

    for (size_t i=0; i<numLights; i++)
      Light_destroy(lights[i]);
    alignedUSMFree(lights);
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

  ISPCTriangleMesh::ISPCTriangleMesh (RTCDevice device, unsigned int numTriangles, unsigned int numVertices, bool hasNormals, bool hasTexcoords, unsigned int numTimeSteps)
    : geom(TRIANGLE_MESH),
      positions(nullptr), normals(nullptr), texcoords(nullptr), triangles(nullptr), startTime(0.0f), endTime(1.0f),
      numTimeSteps(numTimeSteps), numVertices(numVertices), numTriangles(numTriangles)
  {
    assert(numTimeSteps);

    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_TRIANGLE);
    
    positions = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*numTimeSteps);
    for (size_t i=0; i<numTimeSteps; i++)
      positions[i] = (Vec3fa*) alignedUSMMalloc(sizeof(Vec3fa)*numVertices);

    if (hasNormals) {
      normals = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*numTimeSteps);
      for (size_t i=0; i<numTimeSteps; i++)
        normals[i] = (Vec3fa*) alignedUSMMalloc(sizeof(Vec3fa)*numVertices);
    }

    if (hasTexcoords)
      texcoords = (Vec2f*) alignedUSMMalloc(sizeof(Vec2f)*numVertices);

    triangles = (ISPCTriangle*) alignedUSMMalloc(sizeof(ISPCTriangle)*numTriangles);
  }
  
  ISPCTriangleMesh::ISPCTriangleMesh (RTCDevice device, TutorialScene* scene_in, Ref<SceneGraph::TriangleMeshNode> in) 
    : geom(TRIANGLE_MESH), positions(nullptr), normals(nullptr)
  {
    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_TRIANGLE);
   
    positions = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = copyArrayToUSM(in->positions[i]);
    
    if (in->normals.size()) {
      normals = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = copyArrayToUSM(in->normals[i]);
    }
    
    texcoords = copyArrayToUSM(in->texcoords);

    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numTriangles = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
    triangles = (ISPCTriangle*) copyArrayToUSM(in->triangles);
  }

  ISPCTriangleMesh::~ISPCTriangleMesh ()
  {
    if (positions) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(positions[i]);
      alignedUSMFree(positions);
    }
    
    if (normals) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(normals[i]);
      alignedUSMFree(normals);
    }

    alignedUSMFree(texcoords);
    alignedUSMFree(triangles);
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
    
    positions = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = copyArrayToUSM(in->positions[i]);

    if (in->normals.size()) {
      normals = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = copyArrayToUSM(in->normals[i]);
    }
    
    texcoords = copyArrayToUSM(in->texcoords);
    quads = (ISPCQuad*) copyArrayToUSM(in->quads);
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numQuads = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCQuadMesh::~ISPCQuadMesh ()
  {
    if (positions) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(positions[i]);
      alignedUSMFree(positions);
    }
    
    if (normals) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(normals[i]);
      alignedUSMFree(normals);
    }

    alignedUSMFree(texcoords);
    alignedUSMFree(quads);
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
    
    positions = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = copyArrayToUSM(in->positions[i]);

    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numGrids = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
    grids = (ISPCGrid*) copyArrayToUSM(in->grids);
  }

  ISPCGridMesh::~ISPCGridMesh ()
  {
    if (positions) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(positions[i]);
      alignedUSMFree(positions);
    }

    alignedUSMFree(grids);
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
    
    positions = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = (Vec3fa*) copyArrayToUSM(in->positions[i]);

    if (in->normals.size()) {
      normals = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = copyArrayToUSM(in->normals[i]);
    }

    if (in->tangents.size()) {
      tangents = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
      for (size_t i=0; i<in->numTimeSteps(); i++)
        tangents[i] = (Vec3fa*) copyArrayToUSM(in->tangents[i]);
    }

    if (in->dnormals.size()) {
      dnormals = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
      for (size_t i=0; i<in->numTimeSteps(); i++)
        dnormals[i] = copyArrayToUSM(in->dnormals[i]);
    }
    
    hairs = (ISPCHair*) copyArrayToUSM(in->hairs);

    if (in->flags.size())
      flags = (unsigned char*) copyArrayToUSM(in->flags);
    
    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numHairs = (unsigned)in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
    tessellation_rate = in->tessellation_rate;
  }

  ISPCHairSet::~ISPCHairSet()
  {
    if (positions) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(positions[i]);
      alignedUSMFree(positions);
    }
    
    if (normals) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(normals[i]);
      alignedUSMFree(normals);
    }

    if (tangents) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(tangents[i]);
      alignedUSMFree(tangents);
    }
    
    if (dnormals) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(dnormals[i]);
      alignedUSMFree(dnormals);
    }

    alignedUSMFree(hairs);
    alignedUSMFree(flags);
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
    
    positions = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = (Vec3fa*) copyArrayToUSM(in->positions[i]);

    if (in->normals.size()) {
      normals = (Vec3fa**) alignedUSMMalloc(sizeof(Vec3fa*)*in->numTimeSteps());
      for (size_t i=0; i<in->numTimeSteps(); i++)
        normals[i] = copyArrayToUSM(in->normals[i]);
    }

    startTime = in->time_range.lower;
    endTime   = in->time_range.upper;
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCPointSet::~ISPCPointSet ()
  {
    if (positions) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(positions[i]);
      alignedUSMFree(positions);
    }
    
    if (normals) {
      for (size_t i=0; i<numTimeSteps; i++) alignedUSMFree(normals[i]);
      alignedUSMFree(normals);
    }
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


  ISPCInstance::ISPCInstance (RTCDevice device, unsigned int numTimeSteps)
    : geom(INSTANCE), child(nullptr), startTime(0.0f), endTime(1.0f), numTimeSteps(numTimeSteps), quaternion(false), spaces(nullptr)
  {
    assert(numTimeSteps);
    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE);
    spaces = (AffineSpace3fa*) alignedUSMMalloc(numTimeSteps*sizeof(AffineSpace3fa),16);
    for (size_t i=0; i<numTimeSteps; i++)
      spaces[i] = AffineSpace3fa(one);
  }
  
  ISPCInstance::ISPCInstance (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::TransformNode> in)
    : geom(INSTANCE), child(nullptr), startTime(0.0f), endTime(1.0f), numTimeSteps(1), quaternion(false), spaces(nullptr)
  {
    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE);
    
    if (g_animation_mode)
    {
      spaces = (AffineSpace3fa*) alignedUSMMalloc(sizeof(AffineSpace3fa),16);
      child = ISPCScene::convertGeometry(device,scene,in->child);
      spaces[0] = in->get(0.0f);
    }
    else
    {
      spaces = (AffineSpace3fa*) alignedUSMMalloc(in->spaces.size()*sizeof(AffineSpace3fa),16);
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
    alignedUSMFree(spaces);
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

  ISPCInstanceArray::ISPCInstanceArray (RTCDevice device, unsigned int numTimeSteps)
    : geom(INSTANCE_ARRAY), child(nullptr), startTime(0.0f), endTime(1.0f), numTimeSteps(numTimeSteps), numInstances(0), quaternion(false), spaces_array(nullptr)
  {
    assert(numTimeSteps);
    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE_ARRAY);
  }

  ISPCInstanceArray::ISPCInstanceArray (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::MultiTransformNode> in)
    : geom(INSTANCE_ARRAY), child(nullptr), startTime(0.0f), endTime(1.0f), numTimeSteps(1), quaternion(false), spaces_array(nullptr)
  {
    if (in->spaces.size() == 0) {
      THROW_RUNTIME_ERROR("MultiTransformNode invalid. has no transforms.");
    }
    numInstances = in->spaces.size();

    geom.geometry = rtcNewGeometry (device, RTC_GEOMETRY_TYPE_INSTANCE_ARRAY);

    if (g_animation_mode)
    {
      spaces_array = (AffineSpace3fa**) alignedUSMMalloc(sizeof(AffineSpace3fa*),16);
      child = ISPCScene::convertGeometry(device,scene,in->child);

      spaces_array[0] = (AffineSpace3fa*) alignedUSMMalloc(numInstances*sizeof(AffineSpace3fa),16);
      for (size_t i = 0; i < numInstances; ++i) {
        spaces_array[0][i] = in->get(i, 0.0f);
      }
    }
    else
    {
      numTimeSteps = (unsigned)in->spaces[0].size();
      if (numTimeSteps == 0) {
        THROW_RUNTIME_ERROR("MultiTransformNode invalid. numTimeSteps can not be 0.");
      }
      child = ISPCScene::convertGeometry(device,scene,in->child);
      startTime  = in->spaces[0].time_range.lower;
      endTime    = in->spaces[0].time_range.upper;
      quaternion = in->spaces[0].quaternion;
      spaces_array = (AffineSpace3fa**) alignedUSMMalloc(numTimeSteps*sizeof(AffineSpace3fa*),16);
      for (size_t i=0; i<numTimeSteps; i++) {
        spaces_array[i] = (AffineSpace3fa*) alignedUSMMalloc(numInstances*sizeof(AffineSpace3fa),16);
        for (size_t j = 0; j < numInstances; ++j)
          spaces_array[i][j] = in->spaces[j][i];
      }
    }
  }

  ISPCInstanceArray::~ISPCInstanceArray() {
    for (size_t i = 0; i < numTimeSteps; ++i)
      alignedUSMFree(spaces_array[i]);
    alignedUSMFree(spaces_array);
  }

  void ISPCInstanceArray::commit()
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
        rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_TRANSFORM, 0, RTC_FORMAT_QUATERNION_DECOMPOSITION, (void*)&spaces_array[0][0].l.vx.x, 0, sizeof(AffineSpace3fa), numInstances);
      } else {
        rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_TRANSFORM, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, (void*)&spaces_array[0][0].l.vx.x, 0, sizeof(AffineSpace3fa), numInstances);
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
      for (unsigned t=0; t<numTimeSteps; t++) {
        if (quaternion) {
          rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_TRANSFORM, t, RTC_FORMAT_QUATERNION_DECOMPOSITION, (void*)&spaces_array[t][0].l.vx.x, 0, sizeof(AffineSpace3fa), numInstances);
        } else {
          rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_TRANSFORM, t, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, (void*)&spaces_array[t][0].l.vx.x, 0, sizeof(AffineSpace3fa), numInstances);
        }
      }
      rtcSetGeometryUserData(g, this);
      rtcCommitGeometry(g);
    }
  }

  ISPCGroup::ISPCGroup( RTCDevice device, unsigned int numGeometries )
    : geom(GROUP), scene(rtcNewScene(device)), geometries(nullptr), numGeometries(numGeometries), requiredInstancingDepth(0)
  {
    geometries = (ISPCGeometry**) alignedUSMMalloc(sizeof(ISPCGeometry*)*numGeometries);
  }
  
  ISPCGroup::ISPCGroup (RTCDevice device, TutorialScene* scene, Ref<SceneGraph::GroupNode> in)
    : geom(GROUP), scene(rtcNewScene(device)), requiredInstancingDepth(0)
  {
    numGeometries = (unsigned int) in->size();
    geometries = (ISPCGeometry**) alignedUSMMalloc(sizeof(ISPCGeometry*)*numGeometries);
    for (size_t i=0; i<numGeometries; i++)
      geometries[i] = ISPCScene::convertGeometry(device,scene,in->child(i));
  }

  ISPCGroup::~ISPCGroup()
  {
    alignedUSMFree(geometries);
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
    else if (Ref<SceneGraph::MultiTransformNode> mesh = in.dynamicCast<SceneGraph::MultiTransformNode>()) {
      geom = (ISPCGeometry*) new ISPCInstanceArray(device,scene,mesh);
    }
    else if (Ref<SceneGraph::GroupNode> mesh = in.dynamicCast<SceneGraph::GroupNode>())
      geom = (ISPCGeometry*) new ISPCGroup(device,scene,mesh);
    else if (Ref<SceneGraph::PointSetNode> mesh = in.dynamicCast<SceneGraph::PointSetNode>())
      geom = (ISPCGeometry*) new ISPCPointSet(device,scene, mesh->type, mesh);
    else
      THROW_RUNTIME_ERROR("unknown geometry type");

    in->geometry = geom;
    return geom;
  }

  void ConvertTriangleMesh(RTCDevice device, ISPCTriangleMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags, RTCFeatureFlags& used_features)
  {
    if (mesh->geom.visited) return;
    
    used_features |= RTC_FEATURE_FLAG_TRIANGLE;
    if (mesh->numTimeSteps > 1) used_features |= RTC_FEATURE_FLAG_MOTION_BLUR;
    
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }
  
  void ConvertQuadMesh(RTCDevice device, ISPCQuadMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags, RTCFeatureFlags& used_features)
  {
    if (mesh->geom.visited) return;
    
    used_features |= RTC_FEATURE_FLAG_QUAD;
    if (mesh->numTimeSteps > 1) used_features |= RTC_FEATURE_FLAG_MOTION_BLUR;
    
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }

  void ConvertGridMesh(RTCDevice device, ISPCGridMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags, RTCFeatureFlags& used_features)
  {
    if (mesh->geom.visited) return;
    
    used_features |= RTC_FEATURE_FLAG_GRID;
    if (mesh->numTimeSteps > 1) used_features |= RTC_FEATURE_FLAG_MOTION_BLUR;
    
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }
  
  void ConvertSubdivMesh(RTCDevice device, ISPCSubdivMesh* mesh, RTCBuildQuality quality, RTCSceneFlags flags, RTCFeatureFlags& used_features)
  {
    if (mesh->geom.visited) return;
    
    used_features |= RTC_FEATURE_FLAG_SUBDIVISION;
    if (mesh->numTimeSteps > 1) used_features |= RTC_FEATURE_FLAG_MOTION_BLUR;
    
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }
  
  void ConvertCurveGeometry(RTCDevice device, ISPCHairSet* mesh, RTCBuildQuality quality, RTCSceneFlags flags, RTCFeatureFlags& used_features)
  {
    if (mesh->geom.visited) return;
    
    switch (mesh->type) {
    case RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE:                 used_features |= RTC_FEATURE_FLAG_CONE_LINEAR_CURVE; break;
    case RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE:                used_features |= RTC_FEATURE_FLAG_ROUND_LINEAR_CURVE ; break;
    case RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE:                 used_features |= RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE; break;
    case RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE:                used_features |= RTC_FEATURE_FLAG_ROUND_BEZIER_CURVE; break;
    case RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE:                 used_features |= RTC_FEATURE_FLAG_FLAT_BEZIER_CURVE; break;
    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE:      used_features |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_BEZIER_CURVE; break;
    case RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE:               used_features |= RTC_FEATURE_FLAG_ROUND_BSPLINE_CURVE; break;
    case RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE:                used_features |= RTC_FEATURE_FLAG_FLAT_BSPLINE_CURVE; break;
    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE:     used_features |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_BSPLINE_CURVE; break;
    case RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE:               used_features |= RTC_FEATURE_FLAG_ROUND_HERMITE_CURVE; break;
    case RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE:                used_features |= RTC_FEATURE_FLAG_FLAT_HERMITE_CURVE; break;
    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE:     used_features |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_HERMITE_CURVE; break;
    case RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE:           used_features |= RTC_FEATURE_FLAG_ROUND_CATMULL_ROM_CURVE; break;
    case RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE:            used_features |= RTC_FEATURE_FLAG_FLAT_CATMULL_ROM_CURVE; break;
    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE: used_features |= RTC_FEATURE_FLAG_NORMAL_ORIENTED_CATMULL_ROM_CURVE; break;
    default: assert(false); break;
    }
    if (mesh->numTimeSteps > 1) used_features |= RTC_FEATURE_FLAG_MOTION_BLUR;
    
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }

  void ConvertPoints(RTCDevice device, ISPCPointSet* mesh, RTCBuildQuality quality, RTCSceneFlags flags, RTCFeatureFlags& used_features)
  {
    if (mesh->geom.visited) return;

    switch (mesh->type) {
    case RTC_GEOMETRY_TYPE_SPHERE_POINT:        used_features |= RTC_FEATURE_FLAG_SPHERE_POINT; break;
    case RTC_GEOMETRY_TYPE_DISC_POINT:          used_features |= RTC_FEATURE_FLAG_DISC_POINT; break;
    case RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT: used_features |= RTC_FEATURE_FLAG_ORIENTED_DISC_POINT; break;
    default: assert(false); break;
    }
    if (mesh->numTimeSteps > 1) used_features |= RTC_FEATURE_FLAG_MOTION_BLUR;
    
    mesh->geom.visited = true;
    rtcSetGeometryBuildQuality(mesh->geom.geometry, quality);
    mesh->commit();
  }

  unsigned int ConvertInstance(RTCDevice device, ISPCInstance* instance, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth, RTCFeatureFlags& used_features);
  unsigned int ConvertInstanceArray(RTCDevice device, ISPCInstanceArray* instance, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth, RTCFeatureFlags& used_features);

  unsigned int ConvertGroup(RTCDevice device, ISPCGroup* group, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth, RTCFeatureFlags& used_features)
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
        ConvertSubdivMesh(device,(ISPCSubdivMesh*) geometry, quality, flags, used_features);
      else if (geometry->type == TRIANGLE_MESH)
        ConvertTriangleMesh(device,(ISPCTriangleMesh*) geometry, quality, flags, used_features);
      else if (geometry->type == QUAD_MESH)
        ConvertQuadMesh(device,(ISPCQuadMesh*) geometry, quality, flags, used_features);
      else if (geometry->type == CURVES)
        ConvertCurveGeometry(device,(ISPCHairSet*) geometry, quality, flags, used_features);
      else if (geometry->type == GRID_MESH)
        ConvertGridMesh(device,(ISPCGridMesh*) geometry, quality, flags, used_features);
      else if (geometry->type == POINTS)
        ConvertPoints(device,(ISPCPointSet*) geometry, quality, flags, used_features);
      else if (geometry->type == INSTANCE) {
        unsigned int reqDepth = ConvertInstance(device,(ISPCInstance*) geometry, quality, flags, depth, used_features);
        requiredInstancingDepth = max(requiredInstancingDepth, reqDepth);
      }
      else if (geometry->type == INSTANCE_ARRAY) {
        unsigned int reqDepth = ConvertInstanceArray(device,(ISPCInstanceArray*) geometry, quality, flags, depth, used_features);
        requiredInstancingDepth = max(requiredInstancingDepth, reqDepth);
      }
      else
        assert(false);
    }
    
    group->commit();
    
    group->requiredInstancingDepth = requiredInstancingDepth;
    return requiredInstancingDepth;
  }

  unsigned int ConvertInstance(RTCDevice device, ISPCInstance* instance, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth, RTCFeatureFlags& used_features)
  {
    used_features |= RTC_FEATURE_FLAG_INSTANCE;
    if (instance->numTimeSteps > 1) used_features |= RTC_FEATURE_FLAG_MOTION_BLUR;
    
    if (instance->child->type != GROUP)
      THROW_RUNTIME_ERROR("invalid scene structure");

    ISPCGroup* group = (ISPCGroup*) instance->child;
    unsigned int requiredInstancingDepth = 1+ConvertGroup(device, group, quality, flags, depth+1, used_features);

    if (depth + requiredInstancingDepth > RTC_MAX_INSTANCE_LEVEL_COUNT)
      THROW_RUNTIME_ERROR("scene instancing depth is too large");

    if (instance->geom.visited) return requiredInstancingDepth;
    instance->geom.visited = true;
    instance->commit();

    return requiredInstancingDepth;
  }

  unsigned int ConvertInstanceArray(RTCDevice device, ISPCInstanceArray* instance, RTCBuildQuality quality, RTCSceneFlags flags, unsigned int depth, RTCFeatureFlags& used_features)
  {
    used_features |= RTC_FEATURE_FLAG_INSTANCE_ARRAY;
    if (instance->numTimeSteps > 1) used_features |= RTC_FEATURE_FLAG_MOTION_BLUR;

    if (instance->child->type != GROUP)
      THROW_RUNTIME_ERROR("invalid scene structure");

    ISPCGroup* group = (ISPCGroup*) instance->child;
    unsigned int requiredInstancingDepth = 1+ConvertGroup(device, group, quality, flags, depth+1, used_features);

    if (depth + requiredInstancingDepth > RTC_MAX_INSTANCE_LEVEL_COUNT)
      THROW_RUNTIME_ERROR("scene instancing depth is too large");

    if (instance->geom.visited) return requiredInstancingDepth;
    instance->geom.visited = true;
    instance->commit();

    return requiredInstancingDepth;
  }

  extern "C" RTCScene ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCBuildQuality quality, RTCSceneFlags flags, RTCFeatureFlags *used_features_out)
  {
    TutorialScene* tutorial_scene = (TutorialScene*) scene_in->tutorialScene;
    if (!tutorial_scene) return scene_in->scene;
    
    RTCFeatureFlags used_features = RTC_FEATURE_FLAG_NONE;

    RTCScene scene = scene_in->scene;
    rtcSetSceneFlags(scene, flags);
    
    for (unsigned int geomID=0; geomID<scene_in->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = scene_in->geometries[geomID];
      if (geometry->type == SUBDIV_MESH)
        ConvertSubdivMesh(g_device,(ISPCSubdivMesh*) geometry, quality, flags, used_features);
      else if (geometry->type == TRIANGLE_MESH)
        ConvertTriangleMesh(g_device,(ISPCTriangleMesh*) geometry, quality, flags, used_features);
      else if (geometry->type == QUAD_MESH)
        ConvertQuadMesh(g_device,(ISPCQuadMesh*) geometry, quality, flags, used_features);
      else if (geometry->type == CURVES)
        ConvertCurveGeometry(g_device,(ISPCHairSet*) geometry, quality, flags, used_features);
      else if (geometry->type == GRID_MESH)
        ConvertGridMesh(g_device,(ISPCGridMesh*) geometry, quality, flags, used_features);
      else if (geometry->type == POINTS)
        ConvertPoints(g_device,(ISPCPointSet*) geometry, quality, flags, used_features);
      else if (geometry->type == INSTANCE)
        ConvertInstance(g_device, (ISPCInstance*) geometry, quality, flags, 0, used_features);
      else if (geometry->type == INSTANCE_ARRAY)
        ConvertInstanceArray(g_device, (ISPCInstanceArray*) geometry, quality, flags, 0, used_features);
      else
        assert(false);

      rtcAttachGeometryByID(scene,geometry->geometry,geomID);
    }

    if (Application::instance) {
      Application::instance->log(1,"scene features = " + to_string(used_features));
      Application::instance->log(1,"creating Embree objects done");
    }
    
    if (used_features_out)
      *used_features_out = used_features;

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

      if (geometry->type == INSTANCE) {
        ISPCInstance* inst = (ISPCInstance*) geometry;
        Ref<SceneGraph::TransformNode> node = tutorial_scene->geometries[geomID].dynamicCast<SceneGraph::TransformNode>();
        assert(node);
        inst->spaces[0] = node->get(time);
        inst->commit();
      }
      else if (geometry->type == INSTANCE_ARRAY) {
        ISPCInstanceArray* inst = (ISPCInstanceArray*) geometry;
        Ref<SceneGraph::MultiTransformNode> node = tutorial_scene->geometries[geomID].dynamicCast<SceneGraph::MultiTransformNode>();
        assert(node);
        for (size_t i = 0; i < inst->numInstances; ++i) {
          inst->spaces_array[0][i] = node->get(i, time);
        }
        inst->commit();
      }
    }

    rtcCommitScene(scene_in->scene);

    for (unsigned int i=0; i<scene_in->numLights; i++)
      ISPCScene::updateLight(tutorial_scene->lights[i]->get(time), scene_in->lights[i]);
  }
}
