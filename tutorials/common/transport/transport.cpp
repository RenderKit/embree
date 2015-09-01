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
#include "../../common/tutorial/obj_loader.h"
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

  ISPCHairSet* convertHair (OBJScene::HairSet* in)
  {
    ISPCHairSet* out = new ISPCHairSet;
    out->v = in->v.size() ? &in->v[0] : nullptr;
    out->v2 = in->v2.size() ? &in->v2[0] : nullptr;
    out->hairs = (ISPCHair*) (in->hairs.size() ? &in->hairs[0] : nullptr);
    out->numVertices = in->v.size();
    out->numHairs = in->hairs.size();
    return out;
  }
  
  ISPCMesh* convertMesh (OBJScene::Mesh* in)
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

  ISPCSubdivMesh* convertSubdivMesh (OBJScene::SubdivMesh* in)
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

  void set_scene (OBJScene* in) 
  {
    ISPCScene* out = new ISPCScene;

    size_t total_triangles = 0;
    size_t total_quads     = 0;
    
    out->meshes = new ISPCMesh*[in->meshes.size()];
    for (size_t i=0; i<in->meshes.size(); i++)
      {
        out->meshes[i] = convertMesh(in->meshes[i]);
        total_triangles += out->meshes[i]->numTriangles;
        total_quads     += out->meshes[i]->numQuads;        
      }
    
    out->numMeshes = in->meshes.size();

    out->materials = (ISPCMaterial*) (in->materials.size() ? &in->materials[0] : nullptr);
    out->numMaterials = in->materials.size();

    out->hairs = new ISPCHairSet*[in->hairsets.size()];
    for (size_t i=0; i<in->hairsets.size(); i++) out->hairs[i] = convertHair(in->hairsets[i]);
    out->numHairSets = in->hairsets.size();

    out->ambientLights = (ISPCAmbientLight*) (in->ambientLights.size() ? &*in->ambientLights.begin() : nullptr);
    out->numAmbientLights = in->ambientLights.size();

    out->pointLights = (ISPCPointLight*) (in->pointLights.size() ? &*in->pointLights.begin() : nullptr);
    out->numPointLights = in->pointLights.size();

    out->dirLights = (ISPCDirectionalLight*) (in->directionalLights.size() ? &*in->directionalLights.begin() : nullptr);
    out->numDirectionalLights = in->directionalLights.size();

    out->distantLights = (ISPCDistantLight*) (in->distantLights.size() ? &*in->distantLights.begin() : nullptr);
    out->numDistantLights = in->distantLights.size();

    out->subdiv = new ISPCSubdivMesh*[in->subdiv.size()];

    size_t coarse_primitives = 0;
    for (size_t i=0; i<in->subdiv.size(); i++)
      {
	out->subdiv[i] = convertSubdivMesh(in->subdiv[i]);
	coarse_primitives += out->subdiv[i]->numFaces;
      }
    out->numSubdivMeshes = in->subdiv.size();

    out->subdivMeshKeyFrames = nullptr;
    out->numSubdivMeshKeyFrames = 0;

    g_ispc_scene = out;
  }

  void set_scene_keyframes(OBJScene** in, size_t numKeyFrames)
  {
    if (g_ispc_scene)
      {
	g_ispc_scene->subdivMeshKeyFrames    = new ISPCSubdivMeshKeyFrame*[numKeyFrames];
	g_ispc_scene->numSubdivMeshKeyFrames = numKeyFrames;
	for (size_t k=0;k<numKeyFrames;k++)
	  {
	    ISPCSubdivMeshKeyFrame *kf = new ISPCSubdivMeshKeyFrame;

	    kf->subdiv = new ISPCSubdivMesh*[in[k]->subdiv.size()];

	    for (size_t i=0; i<in[k]->subdiv.size(); i++)
	      kf->subdiv[i] = convertSubdivMesh(in[k]->subdiv[i]);

	    kf->numSubdivMeshes = in[k]->subdiv.size();

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
