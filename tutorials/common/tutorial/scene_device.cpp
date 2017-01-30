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
  extern "C" RTCScene* geomID_to_scene = nullptr;
  extern "C" ISPCInstance** geomID_to_inst = nullptr;
  extern "C" int g_instancing_mode;

  unsigned int ConvertTriangleMesh(ISPCTriangleMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewTriangleMesh (scene_out, gflags, mesh->numTriangles, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
    mesh->scene = scene_out;
    mesh->geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertQuadMesh(ISPCQuadMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewQuadMesh (scene_out, gflags, mesh->numQuads, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
    mesh->scene = scene_out;
    mesh->geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertSubdivMesh(ISPCSubdivMesh* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewSubdivisionMesh(scene_out, gflags, mesh->numFaces, mesh->numEdges, mesh->numVertices,
                                                mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles, mesh->numTimeSteps);
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = FIXED_EDGE_TESSELLATION_VALUE;
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa  ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float));

    /* create geometry topology */
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int));
    rtcSetSubdivisionMode(scene_out, geomID, 0, mesh->position_subdiv_mode);

    /* set normal buffers and optionally normal topology */
    if (mesh->normals) {
      rtcSetBuffer2(scene_out, geomID, (RTCBufferType)(RTC_USER_VERTEX_BUFFER+1), mesh->normals, 0, sizeof(Vec3fa  ), mesh->numNormals);
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
    mesh->scene = scene_out;
    mesh->geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertLineSegments(ISPCLineSegments* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewLineSegments (scene_out, gflags, mesh->numSegments, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions+t*mesh->numVertices,0,sizeof(Vec3fa));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
    mesh->scene = scene_out;
    mesh->geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertHairSet(ISPCHairSet* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewHairGeometry (scene_out, gflags, mesh->numHairs, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions+t*mesh->numVertices,0,sizeof(Vec3fa));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->hairs,0,sizeof(ISPCHair));
    rtcSetTessellationRate(scene_out,geomID,(float)mesh->tessellation_rate);
    mesh->scene = scene_out;
    mesh->geomID = geomID;
    return geomID;
  }
  
  unsigned int ConvertCurveGeometry(ISPCHairSet* mesh, RTCGeometryFlags gflags, RTCScene scene_out)
  {
    unsigned int geomID = rtcNewCurveGeometry (scene_out, gflags, mesh->numHairs, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t), mesh->positions+t*mesh->numVertices,0,sizeof(Vec3fa));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->hairs,0,sizeof(ISPCHair));
    mesh->scene = scene_out;
    mesh->geomID = geomID;
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
  
  unsigned int ConvertInstance(ISPCInstance* instance, int meshID, RTCScene scene_out)
  {
    /*if (g_instancing_mode == 1) {
      unsigned int geom_inst = instance->geomID;
      unsigned int geomID = rtcNewGeometryInstance(scene_out, geom_inst);
      rtcSetTransform(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->space.l.vx.x);
      return geomID;
      } else */
    {
      RTCScene scene_inst = geomID_to_scene[instance->geomID];
      if (instance->numTimeSteps == 1) {
        unsigned int geomID = rtcNewInstance(scene_out, scene_inst);
        rtcSetTransform(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->spaces[0].l.vx.x);
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
    size_t numGeometries = scene_in->numGeometries;
    geomID_to_scene = (RTCScene*) alignedMalloc(numGeometries*sizeof(RTCScene));
    geomID_to_inst  = (ISPCInstance_ptr*) alignedMalloc(numGeometries*sizeof(ISPCInstance_ptr));
    RTCScene scene_out = rtcDeviceNewScene(g_device,sflags,aflags);
    
    /* use geometry instancing feature */
    if (g_instancing_mode == 1)
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
        else if (geometry->type == INSTANCE) {
          unsigned int geomID = ConvertInstance((ISPCInstance*) geometry, i, scene_out);
          assert(geomID == i); geomID_to_inst[geomID] = (ISPCInstance*) geometry;
        }
        else
          assert(false);
      }
    }
    
    /* use scene instancing feature */
    else if (g_instancing_mode == 2 || g_instancing_mode == 3)
    {
      for (unsigned int i=0; i<scene_in->numGeometries; i++)
      {
        ISPCGeometry* geometry = scene_in->geometries[i];
        if (geometry->type == SUBDIV_MESH) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertSubdivMesh((ISPCSubdivMesh*) geometry,gflags,objscene);
          geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == TRIANGLE_MESH) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertTriangleMesh((ISPCTriangleMesh*) geometry,gflags,objscene);
          geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == QUAD_MESH) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertQuadMesh((ISPCQuadMesh*) geometry,gflags,objscene);
          geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == LINE_SEGMENTS) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertLineSegments((ISPCLineSegments*) geometry,gflags,objscene);
          geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == HAIR_SET) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertHairSet((ISPCHairSet*) geometry,gflags,objscene);
          geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == CURVES) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertCurveGeometry((ISPCHairSet*) geometry,gflags,objscene);
          geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == GROUP) {
          RTCScene objscene = rtcDeviceNewScene(g_device,sflags,aflags);
          ConvertGroup((ISPCGroup*) geometry,gflags,objscene);
          geomID_to_scene[i] = objscene;
          //rtcCommit(objscene);
        }
        else if (geometry->type == INSTANCE) {
          unsigned int geomID = ConvertInstance((ISPCInstance*) geometry, i, scene_out);
          geomID_to_scene[i] = nullptr; geomID_to_inst[geomID] = (ISPCInstance*) geometry;
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
