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

#include "transport_host.h"
#include "transport_device.h"
#include "../../common/tutorial/scene.h"
#include "../../common/tutorial/tutorial_device.h"
#include "../../common/tutorial/scene_device.h"

extern "C" int64_t get_tsc() {
  return read_tsc();
}

namespace embree
{
  /* framebuffer */
  int* g_pixels = nullptr;
  int g_width = -1;
  int g_height = -1;

  /* scene */
  extern "C" ISPCScene* g_ispc_scene = nullptr;

  extern "C" ISPCScene** g_ispc_scene_keyframes = nullptr;
  extern "C" size_t g_numframes = 0;

  ISPCHairSet* convertHairSet (Ref<TutorialScene::HairSet> in)
  {
    ISPCHairSet* out = new ISPCHairSet;
    out->v = in->v.size() ? &in->v[0] : nullptr;
    out->v2 = in->v2.size() ? &in->v2[0] : nullptr;
    out->hairs = (ISPCHair*) (in->hairs.size() ? &in->hairs[0] : nullptr);
    out->numVertices = in->v.size();
    out->numHairs = in->hairs.size();
    return out;
  }
  
  ISPCMesh* convertTriangleMesh (Ref<TutorialScene::Mesh> in)
  {
    ISPCMesh* out = new ISPCMesh;
    out->positions = in->v.size() ? &in->v[0] : nullptr;
    out->positions2 = in->v2.size() ? &in->v2[0] : nullptr;
    out->normals = in->vn.size() ? &in->vn[0] : nullptr;
    out->texcoords = in->vt.size() ? &in->vt[0] : nullptr;
    out->triangles = (ISPCTriangle*) (in->triangles.size() ? &in->triangles[0] : nullptr);
    out->quads = (ISPCQuad*) (in->quads.size() ? &in->quads[0] : nullptr);
    out->numVertices = in->v.size();
    out->numTriangles = in->triangles.size();
    out->numQuads = in->quads.size();   
    out->geomID = -1;
    out->meshMaterialID = in->meshMaterialID;
    return out;
  }

  ISPCSubdivMesh* convertSubdivMesh (Ref<TutorialScene::SubdivMesh> in)
  {
    ISPCSubdivMesh* out = new ISPCSubdivMesh;
    out->positions = in->positions.size() ? &in->positions[0] : nullptr;
    out->normals = in->normals.size() ? &in->normals[0] : nullptr;
    out->texcoords = in->texcoords.size() ? &in->texcoords[0] : nullptr;
    out->position_indices = in->position_indices.size()   ? &in->position_indices[0] : nullptr;
    out->normal_indices = in->normal_indices.size()   ? &in->normal_indices[0] : nullptr;
    out->texcoord_indices = in->texcoord_indices.size()   ? &in->texcoord_indices[0] : nullptr;
    out->verticesPerFace = in->verticesPerFace.size() ? &in->verticesPerFace[0] : nullptr;
    out->holes = in->holes.size() ? &in->holes[0] : nullptr;
    out->edge_creases = in->edge_creases.size() ? &in->edge_creases[0] : nullptr;
    out->edge_crease_weights = in->edge_crease_weights.size() ? &in->edge_crease_weights[0] : nullptr;
    out->vertex_creases = in->vertex_creases.size() ? &in->vertex_creases[0] : nullptr;
    out->vertex_crease_weights = in->vertex_crease_weights.size() ? &in->vertex_crease_weights[0] : nullptr;
    out->numVertices = in->positions.size();
    out->numFaces = in->verticesPerFace.size();
    out->numEdges = in->position_indices.size();   
    out->numEdgeCreases = in->edge_creases.size();
    out->numVertexCreases = in->vertex_creases.size();
    out->numHoles = in->holes.size();
    out->materialID = in->materialID;
    out->geomID = -1;
    out->colors = (Vec3fa*) alignedMalloc(in->positions.size()*sizeof(Vec3fa));
    for (size_t i=0; i<in->positions.size(); i++)
      out->colors[i] = Vec3fa(drand48(),drand48(),drand48());

    size_t numEdges = in->position_indices.size();
    size_t numFaces = in->verticesPerFace.size();
    out->subdivlevel = new float[numEdges];
    out->face_offsets = new int[numFaces];
    for (size_t i=0; i<numEdges; i++) out->subdivlevel[i] = 1.0f;
    int offset = 0;
    for (size_t i=0; i<numFaces; i++)
      {
        out->face_offsets[i] = offset;
        offset+=out->verticesPerFace[i];       
      }
    return out;
  }

  ISPCInstance* convertInstance (Ref<TutorialScene::Instance> in) {
    return new ISPCInstance(in->space,in->geomID);
  }

  ISPCGeometry* convertGeometry (Ref<TutorialScene::Geometry> in)
  {
    if (in->type == TutorialScene::Geometry::TRIANGLE_MESH)
      return (ISPCGeometry*) convertTriangleMesh(in.dynamicCast<TutorialScene::Mesh>());
    else if (in->type == TutorialScene::Geometry::SUBDIV_MESH)
      return (ISPCGeometry*) convertSubdivMesh(in.dynamicCast<TutorialScene::SubdivMesh>());
    else if (in->type == TutorialScene::Geometry::HAIR_SET)
      return (ISPCGeometry*) convertHairSet(in.dynamicCast<TutorialScene::HairSet>());
    else if (in->type == TutorialScene::Geometry::INSTANCE)
      return (ISPCGeometry*) convertInstance(in.dynamicCast<TutorialScene::Instance>());
    else 
      THROW_RUNTIME_ERROR("unknown geometry type");
  }

  void init(const char* cfg) {
    device_init(cfg);
  }

  void key_pressed (int key) 
  {
    call_key_pressed_handler(key);
  }

  void resize(int width, int height)
  {
    if (width == g_width && height == g_height)
      return;

    if (g_pixels) alignedFree(g_pixels);
    g_width = width;
    g_height = height;
    g_pixels = (int*) alignedMalloc(g_width*g_height*sizeof(int),64);
  }

  void set_scene (TutorialScene* in) 
  {
    ISPCScene* out = new ISPCScene;

    out->geometries = new ISPCGeometry*[in->geometries.size()];
    for (size_t i=0; i<in->geometries.size(); i++) out->geometries[i] = convertGeometry(in->geometries[i]);
    out->numGeometries = in->geometries.size();

    out->materials = (ISPCMaterial*) (in->materials.size() ? &in->materials[0] : nullptr);
    out->numMaterials = in->materials.size();

    out->ambientLights = (ISPCAmbientLight*) (in->ambientLights.size() ? in->ambientLights.begin() : nullptr);
    out->numAmbientLights = in->ambientLights.size();

    out->pointLights = (ISPCPointLight*) (in->pointLights.size() ? in->pointLights.begin() : nullptr);
    out->numPointLights = in->pointLights.size();

    out->dirLights = (ISPCDirectionalLight*) (in->directionalLights.size() ? in->directionalLights.begin() : nullptr);
    out->numDirectionalLights = in->directionalLights.size();

    out->distantLights = (ISPCDistantLight*) (in->distantLights.size() ? in->distantLights.begin() : nullptr);
    out->numDistantLights = in->distantLights.size();

    out->subdivMeshKeyFrames = nullptr;
    out->numSubdivMeshKeyFrames = 0;

    g_ispc_scene = out;
  }

  void set_scene_keyframes(TutorialScene** in, size_t numKeyFrames)
  {
    if (g_ispc_scene)
      {
	g_ispc_scene->subdivMeshKeyFrames    = new ISPCSubdivMeshKeyFrame*[numKeyFrames];
	g_ispc_scene->numSubdivMeshKeyFrames = numKeyFrames;
	for (size_t k=0; k<numKeyFrames; k++)
	  {
	    ISPCSubdivMeshKeyFrame *kf = new ISPCSubdivMeshKeyFrame;

	    kf->subdiv = new ISPCSubdivMesh*[in[k]->geometries.size()];

	    for (size_t i=0; i<in[k]->geometries.size(); i++)
	      kf->subdiv[i] = convertSubdivMesh(in[k]->geometries[i].dynamicCast<TutorialScene::SubdivMesh>());

	    kf->numSubdivMeshes = in[k]->geometries.size();

	    g_ispc_scene->subdivMeshKeyFrames[k] = kf;
	    
	  }
      }
  }

  bool pick(const float x, const float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p, Vec3fa& hitPos) {
    return device_pick(x,y,vx,vy,vz,p,hitPos);
  }

  void render(const float time, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p) {
    device_render(g_pixels,g_width,g_height,time,vx,vy,vz,p);
  }

  int* map () {
    return g_pixels;
  }
  
  void unmap () {
  }

  void cleanup()
  {
    device_cleanup();
    alignedFree(g_pixels); 
    g_pixels = nullptr;
  }
}
