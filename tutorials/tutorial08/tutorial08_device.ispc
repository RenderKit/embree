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

#include "../common/tutorial/tutorial_device.isph"

//#define PARALLEL_COMMIT

struct ISPCTriangle 
{
  int v0;                /*< first triangle vertex */
  int v1;                /*< second triangle vertex */
  int v2;                /*< third triangle vertex */
  int materialID;        /*< material of triangle */
};

struct ISPCQuad
{
  int v0;                /*< first triangle vertex */
  int v1;                /*< second triangle vertex */
  int v2;                /*< third triangle vertex */
  int v4;                /*< fourth triangle vertex */
};

struct ISPCMaterial
{
  int type;
  int align[3];

  int illum;             /*< illumination model */
  float d;               /*< dissolve factor, 1=opaque, 0=transparent */
  float Ns;              /*< specular exponent */
  float Ni;              /*< optical density for the surface (index of refraction) */
  
  Vec3fa Ka;              /*< ambient reflectivity */
  Vec3fa Kd;              /*< diffuse reflectivity */
  Vec3fa Ks;              /*< specular reflectivity */
  Vec3fa Tf;              /*< transmission filter */
  Vec3fa v[2];
};

struct ISPCMesh
{
  uniform Vec3fa* positions;    //!< vertex position array
  uniform Vec3fa* normals;       //!< vertex normal array
  uniform Vec2f* texcoords;     //!< vertex texcoord array
  uniform ISPCTriangle* triangles;  //!< list of triangles
  uniform ISPCQuad* quads;  //!< list of triangles
  int numVertices;
  int numTriangles;
  int numQuads;
  Vec3f dir;
  float offset;
};

struct ISPCHair
{
 int vertex,id;  //!< index of first control point and hair ID
};

struct ISPCHairSet
{
 uniform Vec3fa *positions;   //!< hair control points (x,y,z,r)
 uniform ISPCHair *hairs;    //!< list of hairs
 int numVertices;
 int numHairs;
};


struct ISPCScene
{
  uniform ISPCMesh* uniform* meshes;         //!< list of meshes
  uniform ISPCMaterial* materials;  //!< material list
  int numMeshes;
  int numMaterials;
  uniform ISPCHairSet** hairsets;
  int numHairSets;
  bool animate;
};

/* scene data */
extern uniform ISPCScene* uniform g_ispc_scene;
RTCScene g_scene = NULL;
uniform unsigned int g_threads = 0;


/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const uniform RTCError code, const uniform int8* uniform str)
{
  print("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : print("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : print("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: print("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : print("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : print("RTC_UNSUPPORTED_CPU"); break;
  default                   : print("invalid error code"); break;
  }
  if (str) { 
    print(" ("); 
    while (*str) putchar(*str++); 
    print(")\n"); 
  }
  abort();
}

Vec3fa renderPixelEyeLight(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p);


/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize ray tracing core */
  rtcInit(cfg);

  /* get number of HW threads */
  g_threads = getNumHWThreads();
  print("% HW threads found \n",g_threads);

  /* set error handler */
  rtcSetErrorFunction(error_handler);

  /* set start render mode */
  renderPixel = renderPixelStandard;
  //renderPixel = renderPixelEyeLight;	
}


#define VERTICES 8
#define EDGES    24
#define FACES    (EDGES/4)

uniform float test_vertices[8][4] = {
  {-1.0f, -1.0f, -1.0f, 0.0f},
  { 1.0f, -1.0f, -1.0f, 0.0f},
  { 1.0f, -1.0f,  1.0f, 0.0f},
  {-1.0f, -1.0f,  1.0f, 0.0f},
  {-1.0f,  1.0f, -1.0f, 0.0f},
  { 1.0f,  1.0f, -1.0f, 0.0f},
  { 1.0f,  1.0f,  1.0f, 0.0f},
  {-1.0f,  1.0f,  1.0f, 0.0f}
};

uniform unsigned int test_indices[EDGES] = {0, 1, 5, 4,  1, 2, 6, 5,  2, 3, 7, 6,  0, 4, 7, 3,  4, 5, 6, 7,  0, 3, 2, 1};

uniform unsigned int test_offsets[FACES] = {0, 4, 8, 12, 16, 20};


RTCScene convertTestSubdivScene()
{
  /* create scene */
  RTCScene scene_out = rtcNewScene(RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);
  uniform unsigned int subdivMeshID = rtcNewSubdivisionMesh(scene_out, RTC_GEOMETRY_STATIC, FACES, EDGES, VERTICES);

  void *vtx_data_ptr = rtcMapBuffer(scene_out, subdivMeshID, RTC_VERTEX_BUFFER);
  assert( vtx_data_ptr );
  memcpy( vtx_data_ptr , test_vertices, sizeof(uniform Vec3fa) * VERTICES);
  rtcUnmapBuffer(scene_out, subdivMeshID, RTC_VERTEX_BUFFER);

  void *vtx_index_ptr = rtcMapBuffer(scene_out, subdivMeshID, RTC_INDEX_BUFFER);
  assert( vtx_index_ptr );
  memcpy( vtx_index_ptr , test_indices, sizeof(uniform unsigned int) * EDGES);
  rtcUnmapBuffer(scene_out, subdivMeshID, RTC_INDEX_BUFFER);

  void *vtx_offset_ptr = rtcMapBuffer(scene_out, subdivMeshID, RTC_OFFSET_BUFFER);
  assert( vtx_offset_ptr );
  memcpy( vtx_offset_ptr , test_offsets, sizeof(uniform unsigned int) * FACES);
  rtcUnmapBuffer(scene_out, subdivMeshID, RTC_OFFSET_BUFFER);
  return scene_out;
}

RTCScene convertScene(uniform ISPCScene* uniform scene_in)
{
  /* create scene */
  RTCScene scene_out = rtcNewScene(RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);


  /* add all meshes to the scene */
  for (uniform int i=0; i<scene_in->numMeshes; i++)
  {
    /* get ith mesh */
    uniform ISPCMesh* uniform mesh = scene_in->meshes[i];

    /* create a triangle mesh */
    uniform unsigned int geometry = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices);

#if !defined(__XEON_PHI__)
    /* share vertex buffer */
    rtcSetBuffer(scene_out, geometry, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(uniform Vec3fa      ));
    rtcSetBuffer(scene_out, geometry, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(uniform ISPCTriangle));
#else

    /* set vertices */
    uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 
    for (uniform int j=0; j<mesh->numVertices; j++) {
      vertices[j].x = mesh->positions[j].x;
      vertices[j].y = mesh->positions[j].y;
      vertices[j].z = mesh->positions[j].z;
    }
    rtcUnmapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 

    /* set triangles */
    uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
    for (uniform int j=0; j<mesh->numTriangles; j++) {
      triangles[j].v0 = mesh->triangles[j].v0;
      triangles[j].v1 = mesh->triangles[j].v1;
      triangles[j].v2 = mesh->triangles[j].v2;
    }
    rtcUnmapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
#endif
  }

  /* commit changes to scene */
  return scene_out;
}

Vec3f rndColor(const int ID) 
{
  int r = ((ID+13)*17*23) & 255;
  int g = ((ID+15)*11*13) & 255;
  int b = ((ID+17)* 7*19) & 255;
  const float oneOver255f = 1.f/255.f;
  return make_Vec3f(r*oneOver255f,g*oneOver255f,b*oneOver255f);
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;
  
  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  
  /* shade background black */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
  
  /* shade all rays that hit something */
  Vec3f color = make_Vec3f(1.0f);
#if 0
    color = rndColor(ray.geomID);
#else
  /* apply ambient light */
  Vec3f Ng = normalize(ray.Ng);
  color = color*abs(dot(ray.dir,Ng));   
#endif
  return color;
}

/* task that renders a single screen tile */
task void renderTile(uniform int* uniform pixels,
                     const uniform int width,
                     const uniform int height, 
                     const uniform float time,
                     const uniform Vec3f& vx, 
                     const uniform Vec3f& vy, 
                     const uniform Vec3f& vz, 
                     const uniform Vec3f& p,
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
    Vec3f color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}



task void parallelCommit(RTCScene scene)
{
  rtcCommitThread (scene,threadIndex,threadCount); 
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform int width,
                           const uniform int height, 
                           const uniform float time,
                           const uniform Vec3f& vx, 
                           const uniform Vec3f& vy, 
                           const uniform Vec3f& vz, 
                           const uniform Vec3f& p)
{
  /* create scene */
  if (g_scene == NULL)
  { 
   //g_scene = convertScene(g_ispc_scene);
   g_scene = convertTestSubdivScene();

#if !defined(PARALLEL_COMMIT)
  rtcCommit (g_scene);
#else
  launch[  g_threads ] parallelCommit(g_scene); sync;
#endif
  }

  /* render image */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
  rtcDebug();
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcExit();
}
