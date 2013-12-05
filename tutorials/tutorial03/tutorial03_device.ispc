// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "../common/tutorial/tutorial_device.isph"

struct ISPCTriangle 
{
  int v0;                /*< first triangle vertex */
  int v1;                /*< second triangle vertex */
  int v2;                /*< third triangle vertex */
  int materialID;        /*< material of triangle */
};

struct ISPCMaterial
{
  int illum;             /*< illumination model */
  
  float d;               /*< dissolve factor, 1=opaque, 0=transparent */
  float Ns;              /*< specular exponent */
  float Ni;              /*< optical density for the surface (index of refraction) */
  
  vec3f Ka;              /*< ambient reflectivity */
  vec3f Kd;              /*< diffuse reflectivity */
  vec3f Ks;              /*< specular reflectivity */
  vec3f Tf;              /*< transmission filter */
};

struct ISPCMesh
{
  uniform vec3fa* positions;    //!< vertex position array
  uniform vec3fa* normals;       //!< vertex normal array
  uniform vec2f* texcoords;     //!< vertex texcoord array
  uniform ISPCTriangle* triangles;  //!< list of triangles
  int numVertices;
  int numTriangles;
  vec3f dir;
  float offset;
};

struct ISPCScene
{
  uniform ISPCMesh* uniform* meshes;         //!< list of meshes
  uniform ISPCMaterial* materials;  //!< material list
  int numMeshes;
  int numMaterials;
  int8 animate;
};

/* scene data */
extern uniform ISPCScene* uniform g_ispc_scene;
RTCScene scene = NULL;

/* render function to use */
vec3f renderPixelNormal(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p);
vec3f (* uniform renderPixel)(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p);

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize ray tracing core */
  rtcInit(cfg);

  /* set start render mode */
  renderPixel = renderPixelNormal;
}

RTCScene convertScene(uniform ISPCScene* uniform scene_in)
{
  /* create scene */
  RTCScene scene_out = rtcNewScene(RTC_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* add all meshes to the scene */
  for (uniform int i=0; i<scene_in->numMeshes; i++)
  {
    /* get ith mesh */
    uniform ISPCMesh* uniform mesh = scene_in->meshes[i];

    /* create a triangle mesh */
    uniform unsigned int geometry = rtcNewTriangleMesh (scene_out, RTC_STATIC, mesh->numTriangles, mesh->numVertices);
    
    /* set vertices */
    uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 
    for (uniform int j=0; j<mesh->numVertices; j++) {
      vertices[j].x = mesh->positions[j].x;
      vertices[j].y = mesh->positions[j].y;
      vertices[j].z = mesh->positions[j].z;
    }

    /* set triangles */
    uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
    for (uniform int j=0; j<mesh->numTriangles; j++) {
      triangles[j].v0 = mesh->triangles[j].v0;
      triangles[j].v1 = mesh->triangles[j].v1;
      triangles[j].v2 = mesh->triangles[j].v2;
    }
    rtcUnmapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 
    rtcUnmapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
  }

  /* commit changes to scene */
  rtcCommit (scene_out);
  return scene_out;
}

#if 0 // FIXME: remove
/* task that animates objects in the scene */
task void animateTask(const uniform float time)
{
  uniform int i = taskIndex;
  if (i % 2) return;

  /* get ith mesh */
  uniform unsigned int geometry = i;
  uniform ISPCMesh* uniform mesh = g_ispc_scene->meshes[i];
  uniform vec3f dir = mesh->dir;
  uniform float offset = 10.0f*sin(time+mesh->offset);
  
  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene,geometry,RTC_VERTEX_BUFFER); 
  for (uniform int j=0; j<mesh->numVertices; j++) {
    vertices[j].x = offset*dir.x + mesh->positions[j].x;
    vertices[j].y = offset*dir.y + mesh->positions[j].y;
    vertices[j].z = offset*dir.z + mesh->positions[j].z;
  }
  rtcUnmapBuffer(scene,geometry,RTC_VERTEX_BUFFER);

  /* update geometry */
  rtcUpdate (scene,geometry);
}
#endif

/* task that renders a single screen tile */
vec3f renderPixelNormal(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;
  
  /* intersect ray with scene */
  rtcIntersect(scene,ray);
  
  /* shade background black */
  if (ray.geomID == -1) return make_vec3f(0.0f);
  
  /* shade all rays that hit something */
  vec3f color = make_vec3f(0.0f);
#if 0 // FIXME: pointer gather not implemented on ISPC for Xeon Phi
  int materialID = g_ispc_scene->meshes[ray.geomID]->triangles[ray.primID].materialID; 
#else
  int materialID = 0;
  foreach_unique (geomID in ray.geomID) {
    uniform ISPCMesh* uniform mesh = g_ispc_scene->meshes[geomID];
    materialID = mesh->triangles[ray.primID].materialID;
  }
#endif
  uniform ISPCMaterial* material = &g_ispc_scene->materials[materialID];
  color = material->Kd;
  
  /* apply ambient light */
  vec3f Ng = normalize(ray.Ng);
  vec3f Nf = dot(ray.dir,Ng) < 0.0f ? Ng : neg(Ng);
  color = mul(color,abs(dot(ray.dir,Ng)));    
  return color;
}

/* renders a single pixel with UV shading */
vec3f renderPixelUV(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(scene,ray);

  /* shade pixel */
  if (ray.geomID == -1) return make_vec3f(0.0f);
  else return make_vec3f(ray.u,ray.v,1.0f-ray.u-ray.v);
}

/* renders a single pixel with geomID shading */
vec3f renderPixelGeomID(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.instID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(scene,ray);

  /* shade pixel */
  float r = (3453*ray.geomID % 124) / 124.0f;
  float g = (3453*ray.geomID % 224) / 224.0f;
  float b = (3453*ray.geomID % 54) / 54.0f;
  return make_vec3f(r,g,b);
}

/* vizualizes the traversal cost of a pixel */
uniform float scale = 0.0001f;

vec3f renderPixelCycles(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  uniform int64 c0 = get_tsc();
  rtcIntersect(scene,ray);
  uniform int64 c1 = get_tsc();
  
  /* shade pixel */
  return make_vec3f((uniform float)(c1-c0)*scale,0.0f,0.0f);
}

/* called when a key is pressed */
export void device_key_pressed(uniform int key)
{
  /*if      (key == GLUT_KEY_F1) renderPixel = renderPixelNormal;
    else*/ if (key == GLUT_KEY_F2) renderPixel = renderPixelUV;
  else if (key == GLUT_KEY_F3) {
    if (renderPixel == renderPixelCycles) scale *= 1.1f;
    renderPixel = renderPixelCycles;
  }
  else if (key == GLUT_KEY_F4) {
    if (renderPixel == renderPixelCycles) scale *= 0.9f;
    renderPixel = renderPixelCycles;
  }
  else if (key == GLUT_KEY_F5) renderPixel = renderPixelGeomID;
}

/* returns the point seen through specified pixel */
export uniform bool device_pick(const uniform float x,
                                const uniform float y, 
                                const uniform vec3f& vx, 
                                const uniform vec3f& vy, 
                                const uniform vec3f& vz, 
                                const uniform vec3f& p,
                                uniform vec3f& hitPos)
{
  /* initialize ray */
  uniform RTCRay1 ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect1(scene,ray);

  /* shade pixel */
  if (ray.geomID == -1) {
    hitPos = make_vec3f(0.0f,0.0f,0.0f);
    return false;
  }
  else {
    hitPos = add(ray.org,mul(ray.tfar,ray.dir));
    return true;
  }
}

/* task that renders a single screen tile */
task void renderTile(uniform int* uniform pixels,
                     const uniform int width,
                     const uniform int height, 
                     const uniform float time,
                     const uniform vec3f& vx, 
                     const uniform vec3f& vy, 
                     const uniform vec3f& vz, 
                     const uniform vec3f& p,
                     const uniform int numTilesX, 
                     const uniform int numTilesY)
{
  const uniform int tileY = taskIndex / numTilesX;
  const uniform int tileX = taskIndex - tileY * numTilesX;
  const uniform int x0 = tileX * TILE_SIZE_X;
  const uniform int x1 = min(x0+TILE_SIZE_X,width);
  const uniform int y0 = tileY * TILE_SIZE_Y;
  const uniform int y1 = min(y0+TILE_SIZE_Y,height);

  foreach (y = y0 ... y1, x = x0 ... x1)
  {
    /* calculate pixel color */
    vec3f color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0,1));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0,1));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0,1));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform int width,
                           const uniform int height, 
                           const uniform float time,
                           const uniform vec3f& vx, 
                           const uniform vec3f& vy, 
                           const uniform vec3f& vz, 
                           const uniform vec3f& p)
{
  /* create scene */
  if (scene == NULL)
    scene = convertScene(g_ispc_scene);

#if 0 // FIXME: remove
  /* animate some meshes if enabled */
  if (g_ispc_scene->animate)
  {
    /* animate meshes */
    uniform int numMeshes = g_ispc_scene->numMeshes;
    launch[numMeshes] animateTask(time); sync;
    
    /* commit changes to scene */
    rtcCommit (scene);
  }
#endif

  /* render image */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
  rtcDebug();
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (scene);
  rtcExit();
}
