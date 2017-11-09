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
      case INSTANCE: delete (ISPCInstance*) geometries[i]; break;
      case GROUP: delete (ISPCGroup*) geometries[i]; break;
      case QUAD_MESH: delete (ISPCQuadMesh*) geometries[i]; break;
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
  
  ISPCHairSet::ISPCHairSet (TutorialScene* scene_in, RTCCurveType type, RTCCurveBasis basis, Ref<SceneGraph::HairSetNode> in) 
    : geom(CURVES), type(type), basis(basis)
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

  unsigned int ConvertTriangleMesh(RTCDevice device, ISPCTriangleMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    RTCGeometry geom = rtcNewTriangleMesh (device, gflags, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(geom, RTC_VERTEX_BUFFER_(t), mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetBuffer(geom, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle), mesh->numTriangles);
    rtcCommitGeometry(geom);
    unsigned int geomID = rtcAttachGeometry(scene_out,geom);
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    rtcReleaseGeometry(geom);
    return geomID;
  }
  
  unsigned int ConvertQuadMesh(RTCDevice device, ISPCQuadMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    RTCGeometry geom = rtcNewQuadMesh (device, gflags, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(geom, RTC_VERTEX_BUFFER_(t), mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetBuffer(geom, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad), mesh->numQuads);
    rtcCommitGeometry(geom);
    unsigned int geomID = rtcAttachGeometry(scene_out,geom);
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    rtcReleaseGeometry(geom);
    return geomID;
  }
  
  unsigned int ConvertSubdivMesh(RTCDevice device, ISPCSubdivMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    RTCGeometry geom = rtcNewSubdivisionMesh(device, gflags, mesh->numTimeSteps);
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = FIXED_EDGE_TESSELLATION_VALUE;
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(geom, RTC_VERTEX_BUFFER_(t), mesh->positions[t], 0, sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetBuffer(geom, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float), mesh->numEdges);

    /* create geometry topology */
    rtcSetBuffer(geom, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int), mesh->numEdges);
    rtcSetSubdivisionMode(geom, 0, mesh->position_subdiv_mode);

    /* set normal buffers and optionally normal topology */
    if (mesh->normals) {
      rtcSetBuffer(geom, RTC_USER_VERTEX_BUFFER_(1), mesh->normals[0], 0, sizeof(Vec3fa  ), mesh->numNormals);
      if (mesh->normal_indices) {
        rtcSetBuffer(geom, RTC_INDEX_BUFFER_(1),  mesh->normal_indices  , 0, sizeof(unsigned int), mesh->numEdges);
        rtcSetIndexBuffer(geom, RTC_USER_VERTEX_BUFFER_(1), RTC_INDEX_BUFFER_(1));
        rtcSetSubdivisionMode(geom, 1, mesh->normal_subdiv_mode);
      }
    }

    /* set texcoord buffer and optionally texcoord topology */
    if (mesh->texcoords) {
      rtcSetBuffer(geom, RTC_USER_VERTEX_BUFFER_(2), mesh->texcoords, 0, sizeof(Vec2f), mesh->numTexCoords);
      if (mesh->texcoord_indices) {
        rtcSetBuffer(geom, RTC_INDEX_BUFFER_(2),  mesh->texcoord_indices  , 0, sizeof(unsigned int), mesh->numEdges);
        rtcSetIndexBuffer(geom, RTC_USER_VERTEX_BUFFER_(2), RTC_INDEX_BUFFER_(2));
        rtcSetSubdivisionMode(geom, 2, mesh->texcoord_subdiv_mode);
      }
    }

    rtcSetBuffer(geom, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int), mesh->numFaces);
    rtcSetBuffer(geom, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int), mesh->numHoles);
    rtcSetBuffer(geom, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(unsigned int), mesh->numEdgeCreases);
    rtcSetBuffer(geom, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float), mesh->numEdgeCreases);
    rtcSetBuffer(geom, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(unsigned int), mesh->numVertexCreases);
    rtcSetBuffer(geom, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float), mesh->numVertexCreases);
    rtcCommitGeometry(geom);

    unsigned int geomID = rtcAttachGeometry(scene_out,geom);
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    rtcReleaseGeometry(geom);
    return geomID;
  }
  
  unsigned int ConvertCurveGeometry(RTCDevice device, ISPCHairSet* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    RTCGeometry geom = rtcNewCurveGeometry  (device, gflags, mesh->type, mesh->basis, mesh->numTimeSteps);

    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(geom,RTC_VERTEX_BUFFER_(t), mesh->positions[t],0,sizeof(Vec3fa), mesh->numVertices);
    }
    rtcSetBuffer(geom,RTC_INDEX_BUFFER,mesh->hairs,0,sizeof(ISPCHair),mesh->numHairs);
    if (mesh->basis != RTC_BASIS_LINEAR)
      rtcSetTessellationRate(geom,(float)mesh->tessellation_rate);
    rtcCommitGeometry(geom);

    unsigned int geomID = rtcAttachGeometry(scene_out,geom);
    mesh->geom.scene = scene_out;
    mesh->geom.geomID = geomID;
    rtcReleaseGeometry(geom);
    return geomID;
  }
  
  void ConvertGroup(RTCDevice device, ISPCGroup* group, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    for (size_t i=0; i<group->numGeometries; i++)
    {
      ISPCGeometry* geometry = group->geometries[i];
      if (geometry->type == SUBDIV_MESH)
        ConvertSubdivMesh(device,(ISPCSubdivMesh*) geometry, gflags, scene_out);
      else if (geometry->type == TRIANGLE_MESH)
        ConvertTriangleMesh(device,(ISPCTriangleMesh*) geometry, gflags, scene_out);
      else if (geometry->type == QUAD_MESH)
        ConvertQuadMesh(device,(ISPCQuadMesh*) geometry, gflags, scene_out);
      else if (geometry->type == CURVES)
        ConvertCurveGeometry(device,(ISPCHairSet*) geometry, gflags, scene_out);
      else
        assert(false);
    }
  }

  unsigned int ConvertGroupGeometry(RTCDevice device, ISPCGroup* group, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    std::vector<unsigned> geometries(group->numGeometries);
    for (size_t i=0; i<group->numGeometries; i++) {
      geometries[i] = group->geometries[i]->geomID;
      assert(geometries[i] != -1);
    }
    DISABLE_DEPRECATED_WARNING;
    RTCGeometry geom = rtcNewGeometryGroup (device, scene_out, gflags, geometries.data(), geometries.size());
    ENABLE_DEPRECATED_WARNING;
    rtcCommitGeometry(geom);

    unsigned int geomID = rtcAttachGeometry(scene_out,geom);
    group->geom.scene = scene_out;
    group->geom.geomID = geomID;
    rtcReleaseGeometry(geom);
    return geomID;
  }
  
  unsigned int ConvertInstance(RTCDevice device, ISPCScene* scene_in, ISPCInstance* instance, int meshID, RTCScene scene_out)
  {
    if (g_instancing_mode == SceneGraph::INSTANCING_GEOMETRY || g_instancing_mode == SceneGraph::INSTANCING_GEOMETRY_GROUP)
    {
      if (instance->numTimeSteps == 1) {
        unsigned int geom_inst = instance->geom.geomID;
        DISABLE_DEPRECATED_WARNING;
        RTCGeometry geom = rtcNewGeometryInstance(device, scene_out, geom_inst);
        ENABLE_DEPRECATED_WARNING;
        rtcSetTransform(geom,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->spaces[0].l.vx.x,0);
        rtcCommitGeometry(geom);
        unsigned int geomID = rtcAttachGeometry(scene_out,geom);
        rtcReleaseGeometry(geom);
        return geomID;
      } 
      else
        throw std::runtime_error("motion blur not yet supported for geometry instances");
    } 
    else
    {
      RTCScene scene_inst = scene_in->geomID_to_scene[instance->geom.geomID];
      if (instance->numTimeSteps == 1) {
        RTCGeometry geom = rtcNewInstance(device, scene_inst, 1);
        rtcSetTransform(geom,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->spaces[0].l.vx.x,0);
        rtcCommitGeometry(geom);
        unsigned int geomID = rtcAttachGeometry(scene_out,geom);
        rtcReleaseGeometry(geom);
        return geomID;
      }
      else {
        RTCGeometry geom = rtcNewInstance(device, scene_inst, instance->numTimeSteps);
        for (size_t t=0; t<instance->numTimeSteps; t++)
          rtcSetTransform(geom,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->spaces[t].l.vx.x,t);
        rtcCommitGeometry(geom);
        unsigned int geomID = rtcAttachGeometry(scene_out,geom);
        rtcReleaseGeometry(geom);
        return geomID;
      }
    }
  }
  
  typedef ISPCInstance* ISPCInstance_ptr;
  typedef ISPCGeometry* ISPCGeometry_ptr;
  
  extern "C" RTCScene ConvertScene(RTCDevice g_device, ISPCScene* scene_in, RTCGeometryFlags gflags)
  {
    node2geom.clear();
    RTCScene scene_out = rtcDeviceNewScene(g_device);
    
    /* use geometry instancing feature */
    if (g_instancing_mode == SceneGraph::INSTANCING_GEOMETRY || g_instancing_mode == SceneGraph::INSTANCING_GEOMETRY_GROUP)
    {
      for (unsigned int i=0; i<scene_in->numGeometries; i++)
      {
        ISPCGeometry* geometry = scene_in->geometries[i];
        if (geometry->type == SUBDIV_MESH) {
          unsigned int geomID = ConvertSubdivMesh(g_device,(ISPCSubdivMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(rtcGetGeometry(scene_out,geomID));
        }
        else if (geometry->type == TRIANGLE_MESH) {
          unsigned int geomID = ConvertTriangleMesh(g_device,(ISPCTriangleMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(rtcGetGeometry(scene_out,geomID));
        }
        else if (geometry->type == QUAD_MESH) {
          unsigned int geomID = ConvertQuadMesh(g_device,(ISPCQuadMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(rtcGetGeometry(scene_out,geomID));
        }
        else if (geometry->type == CURVES) {
          unsigned int geomID = ConvertCurveGeometry(g_device,(ISPCHairSet*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(rtcGetGeometry(scene_out,geomID));
        }
        else if (geometry->type == GROUP) {
          unsigned int geomID = ConvertGroupGeometry(g_device,(ISPCGroup*) geometry, gflags, scene_out);
          assert(geomID == i);
          rtcDisable(rtcGetGeometry(scene_out,geomID));
        }
        else if (geometry->type == INSTANCE) {
          unsigned int geomID = ConvertInstance(g_device, scene_in, (ISPCInstance*) geometry, i, scene_out);
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
          RTCScene objscene = rtcDeviceNewScene(g_device);
          ConvertSubdivMesh(g_device,(ISPCSubdivMesh*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == TRIANGLE_MESH) {
          RTCScene objscene = rtcDeviceNewScene(g_device);
          ConvertTriangleMesh(g_device,(ISPCTriangleMesh*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == QUAD_MESH) {
          RTCScene objscene = rtcDeviceNewScene(g_device);
          ConvertQuadMesh(g_device,(ISPCQuadMesh*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == CURVES) {
          RTCScene objscene = rtcDeviceNewScene(g_device);
          ConvertCurveGeometry(g_device,(ISPCHairSet*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == GROUP) {
          RTCScene objscene = rtcDeviceNewScene(g_device);
          ConvertGroup(g_device,(ISPCGroup*) geometry,gflags,objscene);
          scene_in->geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == INSTANCE) {
          unsigned int geomID = ConvertInstance(g_device,scene_in, (ISPCInstance*) geometry, i, scene_out);
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
          unsigned int geomID MAYBE_UNUSED = ConvertSubdivMesh(g_device,(ISPCSubdivMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else if (geometry->type == TRIANGLE_MESH) {
          unsigned int geomID MAYBE_UNUSED = ConvertTriangleMesh(g_device,(ISPCTriangleMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else if (geometry->type == QUAD_MESH) {
          unsigned int geomID MAYBE_UNUSED = ConvertQuadMesh(g_device,(ISPCQuadMesh*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else if (geometry->type == CURVES) {
          unsigned int geomID MAYBE_UNUSED = ConvertCurveGeometry(g_device,(ISPCHairSet*) geometry, gflags, scene_out);
          assert(geomID == i);
        }
        else
          assert(false);
      }
    }
    return scene_out;
  }
}
