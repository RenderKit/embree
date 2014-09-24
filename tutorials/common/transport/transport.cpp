// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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
#include "tutorial/obj_loader.h"

extern "C" int64 get_tsc() {
  return __rdtsc();
}

namespace embree
{
  /* framebuffer */
  int* g_pixels = NULL;
  int g_width = -1;
  int g_height = -1;

  /*! ISPC compatible hair set */
  struct ISPCHairSet
  {
    Vec3fa* v;     //!< hair control points (x,y,z,r)
    Vec3fa* v2;     //!< hair control points (x,y,z,r)
    OBJScene::Hair* hairs;   
    int numVertices;
    int numHairs;
  };

  /* ISPC compatible mesh */
  struct ISPCMesh
  {
    Vec3fa* positions;    //!< vertex position array
    Vec3fa* positions2;    //!< vertex position array
    Vec3fa* normals;       //!< vertex normal array
    Vec2f* texcoords;     //!< vertex texcoord array
    OBJScene::Triangle* triangles;  //!< list of triangles
    OBJScene::Quad* quads;
    int numVertices;
    int numTriangles;
    int numQuads;
    Vec3f dir;
    float offset;
  };

  /* ISPC compatible scene */
  struct ISPCScene
  {
    ISPCMesh** meshes;                       //!< list of meshes
    OBJScene::Material* materials;           //!< list of materials
    int numMeshes;                           //!< number of meshes
    int numMaterials;                        //!< number of materials

    ISPCHairSet** hairsets;                  //!< list of hair sets
    int numHairSets;                         //!< number of hair sets
    
    OBJScene::AmbientLight* ambientLights;   //!< list of ambient lights
    int numAmbientLights;                    //!< number of ambient lights
  
    OBJScene::PointLight* pointLights;       //!< list of point lights
    int numPointLights;                      //!< number of point lights
  
    OBJScene::DirectionalLight* directionalLights; //!< list of directional lights
    int numDirectionalLights;                //!< number of directional lights

    OBJScene::DistantLight* distantLights;   //!< list of distant lights
    int numDistantLights;                    //!< number of distant lights
  };

  /* scene */
  extern "C" ISPCScene* g_ispc_scene = NULL;

  ISPCHairSet* convertHair (OBJScene::HairSet* in)
  {
    ISPCHairSet* out = new ISPCHairSet;
    out->v = in->v.size() ? &in->v[0] : NULL;
    out->v2 = in->v2.size() ? &in->v2[0] : NULL;
    out->hairs = in->hairs.size() ? &in->hairs[0] : NULL;
    out->numVertices = in->v.size();
    out->numHairs = in->hairs.size();
    return out;
  }
  
  ISPCMesh* convertMesh (OBJScene::Mesh* in)
  {
    ISPCMesh* out = new ISPCMesh;
    out->positions = in->v.size() ? &in->v[0] : NULL;
    out->positions2 = in->v2.size() ? &in->v2[0] : NULL;
    out->normals = in->vn.size() ? &in->vn[0] : NULL;
    out->texcoords = in->vt.size() ? &in->vt[0] : NULL;
    out->triangles = in->triangles.size() ? &in->triangles[0] : NULL;
    out->quads = in->quads.size() ? &in->quads[0] : NULL;
    out->numVertices = in->v.size();
    out->numTriangles = in->triangles.size();
    out->numQuads = in->quads.size();   
    out->dir = normalize(Vec3f(drand48(),drand48(),drand48())-Vec3f(0.5f));
    out->offset = 5.0f*drand48();
    return out;
  }

  void init(const char* cfg) {
    device_init(cfg);
  }

  void key_pressed (int32 key) {
    device_key_pressed(key);
  }

  void resize(int32 width, int32 height)
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

    out->meshes = new ISPCMesh*[in->meshes.size()];
    for (size_t i=0; i<in->meshes.size(); i++) out->meshes[i] = convertMesh(in->meshes[i]);
    out->numMeshes = in->meshes.size();

    out->materials = in->materials.size() ? &in->materials[0] : NULL;
    out->numMaterials = in->materials.size();

    out->hairsets = new ISPCHairSet*[in->hairsets.size()];
    for (size_t i=0; i<in->hairsets.size(); i++) out->hairsets[i] = convertHair(in->hairsets[i]);
    out->numHairSets = in->hairsets.size();

    out->ambientLights = in->ambientLights.size() ? &in->ambientLights[0] : NULL;
    out->numAmbientLights = in->ambientLights.size();

    out->pointLights = in->pointLights.size() ? &in->pointLights[0] : NULL;
    out->numPointLights = in->pointLights.size();

    out->directionalLights = in->directionalLights.size() ? &in->directionalLights[0] : NULL;
    out->numDirectionalLights = in->directionalLights.size();

    out->distantLights = in->distantLights.size() ? &in->distantLights[0] : NULL;
    out->numDistantLights = in->distantLights.size();

    g_ispc_scene = out;
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
    g_pixels = NULL;
  }
}
