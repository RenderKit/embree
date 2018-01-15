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

#include "scene_device.h"

#define FIXED_EDGE_TESSELLATION_VALUE 4

namespace embree
{
  extern "C" {
    int g_instancing_mode = SceneGraph::INSTANCING_NONE;
  }

  std::map<Ref<SceneGraph::Node>,ISPCGeometry*> node2geom;

  ISPCScene::ISPCScene(TutorialScene* in)
  {
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
      Light* light = convertLight(in->lights[i]);
      if (light) lights[numLights++] = light;
    }
    
    geomID_to_scene = in->geomID_to_scene.data();
    geomID_to_inst = (ISPCInstance**) in->geomID_to_inst.data();
  }
  
  ISPCScene::~ISPCScene()
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
    delete[] materials;
    for (size_t i=0; i<numLights; i++)
      Light_destroy(lights[i]);
    delete[] lights;
  }
  
  Light* ISPCScene::convertLight(Ref<SceneGraph::Light> in)
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
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numQuads = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCQuadMesh::~ISPCQuadMesh () {
    if (positions) delete[] positions;
    if (normals) delete[] normals;  }

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
  
  ISPCLineSegments::ISPCLineSegments (TutorialScene* scene_in, Ref<SceneGraph::LineSegmentsNode> in) 
    : geom(LINE_SEGMENTS)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();
    indices = in->indices.data();
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numSegments = (unsigned) in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
  }

  ISPCLineSegments::~ISPCLineSegments () {
    delete[] positions;
  }
  
  ISPCHairSet::ISPCHairSet (TutorialScene* scene_in, SceneGraph::HairSetNode::Type type, SceneGraph::HairSetNode::Basis basis, Ref<SceneGraph::HairSetNode> in) 
    : geom(type == SceneGraph::HairSetNode::HAIR ? HAIR_SET : CURVES), basis(basis == SceneGraph::HairSetNode::BEZIER ? BEZIER_BASIS : BSPLINE_BASIS)
  {
    positions = new Vec3fa*[in->numTimeSteps()];
    for (size_t i=0; i<in->numTimeSteps(); i++)
      positions[i] = in->positions[i].data();
    hairs = (ISPCHair*) in->hairs.data();
    numTimeSteps = (unsigned) in->numTimeSteps();
    numVertices = (unsigned) in->numVertices();
    numHairs = (unsigned)in->numPrimitives();
    geom.materialID = scene_in->materialID(in->material);
    tessellation_rate = in->tessellation_rate;
  }

  ISPCHairSet::~ISPCHairSet() {
    delete[] positions;
  }

  ISPCInstance::ISPCInstance (TutorialScene* scene, Ref<SceneGraph::TransformNode> in)
    : geom(INSTANCE), numTimeSteps(unsigned(in->spaces.size())) 
  {
    spaces = (AffineSpace3fa*) alignedMalloc(in->spaces.size()*sizeof(AffineSpace3fa));
    geom.geomID = scene->geometryID(in->child);
    for (size_t i=0; i<numTimeSteps; i++)
      spaces[i] = in->spaces[i];
  }

  ISPCInstance::~ISPCInstance() {
    alignedFree(spaces);
  }

  ISPCGroup::ISPCGroup (TutorialScene* scene, Ref<SceneGraph::GroupNode> in)
    : geom(GROUP)
  {
    numGeometries = in->size();
    geometries = new ISPCGeometry*[numGeometries];
    for (size_t i=0; i<numGeometries; i++)
      geometries[i] = ISPCScene::convertGeometry(scene,in->child(i));
  }
  
  ISPCGroup::~ISPCGroup()
  {
    for (size_t i=0; i<numGeometries; i++)
      delete geometries[i];
    delete[] geometries;
  }

  ISPCGeometry* ISPCScene::convertGeometry (TutorialScene* scene, Ref<SceneGraph::Node> in)
  {
    ISPCGeometry* geom = nullptr;
    if (node2geom.find(in) != node2geom.end())
      return node2geom[in];
    else if (Ref<SceneGraph::TriangleMeshNode> mesh = in.dynamicCast<SceneGraph::TriangleMeshNode>())
      geom = (ISPCGeometry*) new ISPCTriangleMesh(scene,mesh);
    else if (Ref<SceneGraph::QuadMeshNode> mesh = in.dynamicCast<SceneGraph::QuadMeshNode>())
      geom = (ISPCGeometry*) new ISPCQuadMesh(scene,mesh);
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = in.dynamicCast<SceneGraph::SubdivMeshNode>())
      geom = (ISPCGeometry*) new ISPCSubdivMesh(scene,mesh);
    else if (Ref<SceneGraph::LineSegmentsNode> mesh = in.dynamicCast<SceneGraph::LineSegmentsNode>())
      geom = (ISPCGeometry*) new ISPCLineSegments(scene,mesh);
    else if (Ref<SceneGraph::HairSetNode> mesh = in.dynamicCast<SceneGraph::HairSetNode>())
      geom = (ISPCGeometry*) new ISPCHairSet(scene,mesh->type,mesh->basis,mesh);
    else if (Ref<SceneGraph::TransformNode> mesh = in.dynamicCast<SceneGraph::TransformNode>())
      geom = (ISPCGeometry*) new ISPCInstance(scene,mesh);
    else if (Ref<SceneGraph::GroupNode> mesh = in.dynamicCast<SceneGraph::GroupNode>())
      geom = (ISPCGeometry*) new ISPCGroup(scene,mesh);
    else
      THROW_RUNTIME_ERROR("unknown geometry type");

    node2geom[in] = geom;
    return geom;
  }

  unsigned int ConvertTriangleMesh(ISPCTriangleMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewTriangleMesh (scene_out, gflags, mesh->numTriangles, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions[t], 0, sizeof(Vec3fa      ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertQuadMesh(ISPCQuadMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewQuadMesh (scene_out, gflags, mesh->numQuads, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions[t], 0, sizeof(Vec3fa      ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertSubdivMesh(ISPCSubdivMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewSubdivisionMesh(scene_out, gflags, mesh->numFaces, mesh->numEdges, mesh->numVertices,
                                                mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles, mesh->numTimeSteps);
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = FIXED_EDGE_TESSELLATION_VALUE;
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions[t], 0, sizeof(Vec3fa  ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float));

    /* create geometry topology */
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int));
    rtcSetSubdivisionMode(scene_out, geomID, 0, mesh->position_subdiv_mode);

    /* set normal buffers and optionally normal topology */
    if (mesh->normals) {
      rtcSetBuffer2(scene_out, geomID, (RTCBufferType)(RTC_USER_VERTEX_BUFFER+1), mesh->normals[0], 0, sizeof(Vec3fa  ), mesh->numNormals);
      if (mesh->normal_indices) {
        rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_INDEX_BUFFER+1),  mesh->normal_indices  , 0, sizeof(unsigned int));
        rtcSetIndexBuffer(scene_out, geomID, (RTCBufferType)(RTC_USER_VERTEX_BUFFER+1), (RTCBufferType)(RTC_INDEX_BUFFER+1));
        rtcSetSubdivisionMode(scene_out, geomID, 1, mesh->normal_subdiv_mode);
      }
    }

    /* set texcoord buffer and optionally texcoord topology */
    if (mesh->texcoords) {
      rtcSetBuffer2(scene_out, geomID, (RTCBufferType)(RTC_USER_VERTEX_BUFFER+2), mesh->texcoords, 0, sizeof(Vec2f), mesh->numTexCoords);
      if (mesh->texcoord_indices) {
        rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_INDEX_BUFFER+2),  mesh->texcoord_indices  , 0, sizeof(unsigned int));
        rtcSetIndexBuffer(scene_out, geomID, (RTCBufferType)(RTC_USER_VERTEX_BUFFER+2), (RTCBufferType)(RTC_INDEX_BUFFER+2));
        rtcSetSubdivisionMode(scene_out, geomID, 2, mesh->texcoord_subdiv_mode);
      }
    }

    rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float));
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertLineSegments(ISPCLineSegments* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewLineSegments (scene_out, gflags, mesh->numSegments, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions[t],0,sizeof(Vec3fa));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertHairSet(ISPCHairSet* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID;
    if (mesh->basis == BEZIER_BASIS)
      geomID = rtcNewBezierHairGeometry  (scene_out, gflags, mesh->numHairs, mesh->numVertices, mesh->numTimeSteps);
    else
      geomID = rtcNewBSplineHairGeometry (scene_out, gflags, mesh->numHairs, mesh->numVertices, mesh->numTimeSteps);

    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions[t],0,sizeof(Vec3fa));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->hairs,0,sizeof(ISPCHair));
    rtcSetTessellationRate(scene_out,geomID,(float)mesh->tessellation_rate);
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertCurveGeometry(ISPCHairSet* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID;
    if (mesh->basis == BEZIER_BASIS)
      geomID = rtcNewBezierCurveGeometry  (scene_out, gflags, mesh->numHairs, mesh->numVertices, mesh->numTimeSteps);
    else
      geomID = rtcNewBSplineCurveGeometry (scene_out, gflags, mesh->numHairs, mesh->numVertices, mesh->numTimeSteps);

    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions[t],0,sizeof(Vec3fa));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->hairs,0,sizeof(ISPCHair));
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    return geomID;
  }
  
  void ConvertGroup(ISPCGroup* group, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    for (size_t i=0; i<group->numGeometries; i++)
    {
      ISPCGeometry* geometry = group->geometries[i];
      if (geometry->type == SUBDIV_MESH)
        ConvertSubdivMesh((ISPCSubdivMesh*) geometry, gflags, scene_out);
      else if (geometry->type == TRIANGLE_MESH)
        ConvertTriangleMesh((ISPCTriangleMesh*) geometry, gflags, scene_out);
      else if (geometry->type == QUAD_MESH)
        ConvertQuadMesh((ISPCQuadMesh*) geometry, gflags, scene_out);
      else if (geometry->type == LINE_SEGMENTS)
        ConvertLineSegments((ISPCLineSegments*) geometry, gflags, scene_out);
      else if (geometry->type == HAIR_SET)
        ConvertHairSet((ISPCHairSet*) geometry, gflags, scene_out);
      else if (geometry->type == CURVES)
        ConvertCurveGeometry((ISPCHairSet*) geometry, gflags, scene_out);
      else
        assert(false);
    }
  }

  unsigned int ConvertGroupGeometry(ISPCGroup* group, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    std::vector<unsigned> geometries(group->numGeometries);
    for (size_t i=0; i<group->numGeometries; i++) {
      geometries[i] = group->geometries[i]->geomID;
      assert(geometries[i] != -1);
    }
    DISABLE_DEPRECATED_WARNING;
    unsigned int geomID = rtcNewGeometryGroup (scene_out, gflags, geometries.data(), geometries.size());
    ENABLE_DEPRECATED_WARNING;
    group->geom.scene = scene_out;
    group->geom.geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertInstance(ISPCScene* scene_in, ISPCInstance* instance, int meshID, RTCScene scene_out)
  {
    if (g_instancing_mode == SceneGraph::INSTANCING_GEOMETRY || g_instancing_mode == SceneGraph::INSTANCING_GEOMETRY_GROUP) {
      if (instance->numTimeSteps == 1) {
        unsigned int geom_inst = instance->geom.geomID;
        DISABLE_DEPRECATED_WARNING;
        unsigned int geomID = rtcNewGeometryInstance(scene_out, geom_inst);
        ENABLE_DEPRECATED_WARNING;
        rtcSetTransform2(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->spaces[0].l.vx.x,0);
        return geomID;
      } 
      else
        throw std::runtime_error("motion blur not yet supported for geometry instances");
    } 
    else
    {
      RTCScene scene_inst = scene_in->geomID_to_scene[instance->geom.geomID];
      if (instance->numTimeSteps == 1) {
        unsigned int geomID = rtcNewInstance2(scene_out, scene_inst, 1);
        rtcSetTransform2(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->spaces[0].l.vx.x,0);
        return geomID;
      }
      else {
        unsigned int geomID = rtcNewInstance2(scene_out, scene_inst, instance->numTimeSteps);
        for (size_t t=0; t<instance->numTimeSteps; t++)
          rtcSetTransform2(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->spaces[t].l.vx.x,t);
        return geomID;
      }
    }
  }
  
  typedef ISPCInstance* ISPCInstance_ptr;
  typedef ISPCGeometry* ISPCGeometry_ptr;
  
  extern "C" RTCScene ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCSceneFlags sflags, RTCAlgorithmFlags aflags, RTCGeometryFlags gflags)
  {
    node2geom.clear();
    RTCScene scene_out = rtcDeviceNewScene(g_device,sflags,aflags);
    
    /* use geometry instancing feature */
    if (g_instancing_mode == SceneGraph::INSTANCING_GEOMETRY || g_instancing_mode == SceneGraph::INSTANCING_GEOMETRY_GROUP)
    {
      for (unsigned int i=0; i<scene_in->numGeometries; i++)
      {
        ISPCGeometry* geometry = scene_in->geometries[i];
        if (geometry->type == SUBDIV_MESH) {
          unsigned int geomID = ConvertSubdivMesh((ISPCSubdivMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(scene_out,geomID);
        }
        else if (geometry->type == TRIANGLE_MESH) {
          unsigned int geomID = ConvertTriangleMesh((ISPCTriangleMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(scene_out,geomID);
        }
        else if (geometry->type == QUAD_MESH) {
          unsigned int geomID = ConvertQuadMesh((ISPCQuadMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(scene_out,geomID);
        }
        else if (geometry->type == LINE_SEGMENTS) {
          unsigned int geomID = ConvertLineSegments((ISPCLineSegments*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(scene_out,geomID);
        }
        else if (geometry->type == HAIR_SET) {
          unsigned int geomID = ConvertHairSet((ISPCHairSet*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(scene_out,geomID);
        }
        else if (geometry->type == CURVES) {
          unsigned int geomID = ConvertCurveGeometry((ISPCHairSet*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(scene_out,geomID);
        }
        else if (geometry->type == GROUP) {
          unsigned int geomID = ConvertGroupGeometry((ISPCGroup*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(scene_out,geomID);
        }
        else if (geometry->type == INSTANCE) {
          unsigned int geomID = ConvertInstance(scene_in, (ISPCInstance*) geometry, i, scene_out);
          assert(geomID == i); scene_in->geomID_to_inst[geomID] = (ISPCInstance*) geometry;
        }
        else
          assert(false);
      }
    }
    
    /* use scene instancing feature */
    else if (g_instancing_mode == SceneGraph::INSTANCING_SCENE_GEOMETRY || g_instancing_mode == SceneGraph::INSTANCING_SCENE_GROUP)
    {
      for (unsigned int i=0; i<scene_in->numGeometries; i++)
      {
        ISPCGeometry* geometry = scene_in->geometries[i];
        if (geometry->type == SUBDIV_MESH) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertSubdivMesh((ISPCSubdivMesh*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == TRIANGLE_MESH) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertTriangleMesh((ISPCTriangleMesh*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == QUAD_MESH) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertQuadMesh((ISPCQuadMesh*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == LINE_SEGMENTS) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertLineSegments((ISPCLineSegments*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == HAIR_SET) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertHairSet((ISPCHairSet*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == CURVES) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertCurveGeometry((ISPCHairSet*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == GROUP) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertGroup((ISPCGroup*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == INSTANCE) {
          unsigned int geomID = ConvertInstance(scene_in, (ISPCInstance*) geometry, i, scene_out);
          scene_in->geomID_to_scene[i] = nullptr; scene_in->geomID_to_inst[geomID] = (ISPCInstance*) geometry;
        }
        else
          assert(false);
      }
    }
    
    /* no instancing */
    else
    {
      for (unsigned int i=0; i<scene_in->numGeometries; i++)
      {
        ISPCGeometry* geometry = scene_in->geometries[i];
        if (geometry->type == SUBDIV_MESH) {
          unsigned int geomID MAYBE_UNUSED = ConvertSubdivMesh((ISPCSubdivMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else if (geometry->type == TRIANGLE_MESH) {
          unsigned int geomID MAYBE_UNUSED = ConvertTriangleMesh((ISPCTriangleMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else if (geometry->type == QUAD_MESH) {
          unsigned int geomID MAYBE_UNUSED = ConvertQuadMesh((ISPCQuadMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else if (geometry->type == LINE_SEGMENTS) {
          unsigned int geomID MAYBE_UNUSED = ConvertLineSegments((ISPCLineSegments*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else if (geometry->type == HAIR_SET) {
          unsigned int geomID MAYBE_UNUSED = ConvertHairSet((ISPCHairSet*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else if (geometry->type == CURVES) {
          unsigned int geomID MAYBE_UNUSED = ConvertCurveGeometry((ISPCHairSet*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else
          assert(false);
      }
    }
    return scene_out;
  }
}
