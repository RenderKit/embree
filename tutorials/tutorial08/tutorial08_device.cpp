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

#include "tutorial/tutorial_device.h"
// #include "opensubdiv/hbr/mesh.h"
// #include "extensions/triangulatemesh.h"
// #include "extensions/subdivisionmesh.h"

/*! Function used to render a pixel. */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const RTCError code, const char* str)
{
  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  exit(code);
}

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
  Vec3fa* positions;    //!< vertex position array
  Vec3fa* positions2;    //!< vertex position array
  Vec3fa* normals;       //!< vertex normal array
  Vec2f* texcoords;     //!< vertex texcoord array
  ISPCTriangle* triangles;  //!< list of triangles
  ISPCQuad* quads;  //!< list of triangles
  int numVertices;
  int numTriangles;
  int numQuads;

  Vec3fa dir;
  float offset;
};

struct ISPCHair
{
  int vertex,id;  //!< index of first control point and hair ID
};

struct ISPCHairSet
{
  Vec3fa* positions;   //!< hair control points (x,y,z,r)
  Vec3fa* positions2;   //!< hair control points (x,y,z,r)
  ISPCHair* hairs;    //!< list of hairs
  int numVertices;
  int numHairs;
};

struct ISPCScene
{
  ISPCMesh** meshes;         //!< list of meshes
  ISPCMaterial* materials;  //!< material list
  int numMeshes;
  int numMaterials;
  ISPCHairSet** hairsets;
  int numHairSets;
  bool animate;
};

/* scene data */
extern "C" ISPCScene* g_ispc_scene;


/*! Embree state identifier for the scene. */
RTCScene g_scene = NULL;

/* scene data */

/*! Requested subdivision level set in tutorial08.cpp. */
extern int subdivisionLevel;

__forceinline RTCRay constructRay(const Vec3fa &origin, const Vec3fa &direction, float near, float far, int originGeomID, int originPrimID) {

  RTCRay ray;
  ray.org = origin;
  ray.dir = direction;
  ray.tnear = near;
  ray.tfar = far;
  ray.geomID = originGeomID;
  ray.primID = originPrimID;
  ray.mask = -1;
  ray.time = 0;
  return(ray);

}

unsigned int packPixel(const Vec3f &color) {

  unsigned int r = (unsigned int) (255.0f * clamp(color.x, 0.0f, 1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y, 0.0f, 1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z, 0.0f, 1.0f));
  return((b << 16) + (g << 8) + r);

}

// void constructCubeFaces(OpenSubdiv::HbrMesh<Vec3f> *mesh) {

//     const int32_t cubeIndices[24] = {0, 1, 5, 4,  1, 2, 6, 5,  2, 3, 7, 6,  0, 4, 7, 3,  4, 5, 6, 7,  0, 3, 2, 1};

//     mesh->NewFace(4, &cubeIndices[ 0], 0);
//     mesh->NewFace(4, &cubeIndices[ 4], 0);
//     mesh->NewFace(4, &cubeIndices[ 8], 0);
//     mesh->NewFace(4, &cubeIndices[12], 0);
//     mesh->NewFace(4, &cubeIndices[16], 0);
//     mesh->NewFace(4, &cubeIndices[20], 0);

// }

// void constructCubeVertices(OpenSubdiv::HbrMesh<Vec3f> *mesh) {

//     mesh->NewVertex(Vec3f(-1.0f, -1.0f, -1.0f));
//     mesh->NewVertex(Vec3f( 1.0f, -1.0f, -1.0f));
//     mesh->NewVertex(Vec3f( 1.0f, -1.0f,  1.0f));
//     mesh->NewVertex(Vec3f(-1.0f, -1.0f,  1.0f));
//     mesh->NewVertex(Vec3f(-1.0f,  1.0f, -1.0f));
//     mesh->NewVertex(Vec3f( 1.0f,  1.0f, -1.0f));
//     mesh->NewVertex(Vec3f( 1.0f,  1.0f,  1.0f));
//     mesh->NewVertex(Vec3f(-1.0f,  1.0f,  1.0f));

// }

// void triangulateMesh(RTCScene sceneID, unsigned int meshID, SubdivisionMesh &mesh) {

//     /*! Map the triangle mesh vertex buffer from Embree space into user space. */
//     Vertex *vertices = (Vertex *) rtcMapBuffer(sceneID, meshID, RTC_VERTEX_BUFFER);

//     /*! Copy vertex data from the subdivision mesh into the triangle mesh buffer. */
//     for (size_t i=0 ; i < mesh.vertexCount() ; i++) { Vec3f p = mesh.getCoordinates(i);  vertices[i].x = p.x;  vertices[i].y = p.y;  vertices[i].z = p.z; }

//     /*! Unmap the triangle mesh buffer. */
//     rtcUnmapBuffer(sceneID, meshID, RTC_VERTEX_BUFFER);

//     /*! Map the triangle mesh index buffer from Embree space into user space. */
//     Triangle *triangles = (Triangle *) rtcMapBuffer(sceneID, meshID, RTC_INDEX_BUFFER);

//     /* Copy vertex indices into the triangle mesh buffer. */
//     for (size_t i=0, j=0 ; i < mesh.faceCount() ; j += mesh.getFace(i).vertexCount() - 2, i++) triangulateFace(mesh.getFace(i), &triangles[j]);

//     /*! Unmap the triangle mesh buffer. */
//     rtcUnmapBuffer(sceneID, meshID, RTC_INDEX_BUFFER);

// }

// void constructCubeMesh() {

//     /*! The subdivision algorithm is specified by the HbrSubdivision object subtype (currently ignored). */
//     OpenSubdiv::HbrCatmarkSubdivision<Vec3f> *method = new OpenSubdiv::HbrCatmarkSubdivision<Vec3f>();

//     /*! Create a subdivision mesh object in OpenSubdiv. */
//     OpenSubdiv::HbrMesh<Vec3f> *mesh = new OpenSubdiv::HbrMesh<Vec3f>(method);

//     /*! Add vertices and faces to the mesh. */
//     constructCubeVertices(mesh);  constructCubeFaces(mesh);

//     /*! Construct the corresponding subdivision mesh in Embree. */
//     mesh->Finish(g_scene);

//     /*! The OpenSubdiv mesh is no longer needed. */
//     delete mesh;  delete method;

// }

// void constructGroundPlane() {

//     /*! Plane vertices. */
//     Vertex planeVertices[4] = {{-100, -2, -100, 0}, {-100, -2, 100, 0}, {100, -2, -100, 0}, {100, -2, 100, 0}};

//     /*! Construct a triangle mesh object. */
//     unsigned int meshID = rtcNewTriangleMesh(g_scene, RTC_GEOMETRY_STATIC, 2, 4);

//     /*! Map the triangle mesh vertex buffer from Embree space into user space. */
//     Vertex *vertices = (Vertex *) rtcMapBuffer(g_scene, meshID, RTC_VERTEX_BUFFER);

//     /*! Copy vertex data into the triangle mesh buffer. */
//     memcpy(vertices, planeVertices, 4 * sizeof(Vertex));

//     /*! Unmap the triangle mesh buffer. */
//     rtcUnmapBuffer(g_scene, meshID, RTC_VERTEX_BUFFER);

//     /*! Map the triangle mesh index buffer from Embree space into user space. */
//     Triangle *triangles = (Triangle *) rtcMapBuffer(g_scene, meshID, RTC_INDEX_BUFFER);

//     /*! Copy vertex indices into the triangle mesh buffer. */
//     triangles[0].v0 = 0;  triangles[0].v1 = 2;  triangles[0].v2 = 1;  triangles[1].v0 = 1;  triangles[1].v1 = 2;  triangles[1].v2 = 3;

//     /*! Unmap the triangle mesh buffer. */
//     rtcUnmapBuffer(g_scene, meshID, RTC_INDEX_BUFFER);

// }

#define VERTICES 8
#define EDGES    24
#define FACES    (EDGES/4)

Vec3fa test_vertices[] = {
  Vec3fa(-1.0f, -1.0f, -1.0f),
  Vec3fa( 1.0f, -1.0f, -1.0f),
  Vec3fa( 1.0f, -1.0f,  1.0f),
  Vec3fa(-1.0f, -1.0f,  1.0f),
  Vec3fa(-1.0f,  1.0f, -1.0f),
  Vec3fa( 1.0f,  1.0f, -1.0f),
  Vec3fa( 1.0f,  1.0f,  1.0f),
  Vec3fa(-1.0f,  1.0f,  1.0f)
};

unsigned int test_indices[EDGES] = {0, 1, 5, 4,  1, 2, 6, 5,  2, 3, 7, 6,  0, 4, 7, 3,  4, 5, 6, 7,  0, 3, 2, 1};

unsigned int test_offsets[FACES] = {0, 4, 8, 12, 16, 20};

void constructScene() {
  /*! Create an Embree object to hold scene state. */
  g_scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);

  unsigned int totalNumQuads = 0;
  if (g_ispc_scene)
    {
      DBG_PRINT(g_ispc_scene->numMeshes);

      for (int i=0; i<g_ispc_scene->numMeshes; i++)
	{
	  /* get ith mesh */
	  ISPCMesh* mesh = g_ispc_scene->meshes[i];
	  DBG_PRINT(mesh->quads);

	  if (mesh->numQuads)
	    {
	      totalNumQuads += mesh->numQuads;
	      unsigned int *offset_buffer = new unsigned int[mesh->numQuads];
	      for (size_t i=0;i<mesh->numQuads;i++) offset_buffer[i] = i*4;

	      unsigned int subdivMeshID = rtcNewSubdivisionMesh(g_scene, 
								RTC_GEOMETRY_STATIC, 
								mesh->numQuads, 
								mesh->numQuads*4, 
								mesh->numVertices);

	      rtcSetBuffer(g_scene, subdivMeshID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa  ));
	      rtcSetBuffer(g_scene, subdivMeshID, RTC_INDEX_BUFFER,  mesh->quads    , 0, sizeof(unsigned int));
	      rtcSetBuffer(g_scene, subdivMeshID, RTC_OFFSET_BUFFER, offset_buffer  , 0, sizeof(unsigned int));
	      //delete offset_buffer;
	    }
	}       
    }
    

  if (totalNumQuads == 0)
    {
      std::cout << "Loading dummy cube..." << std::endl;
      unsigned int subdivMeshID = rtcNewSubdivisionMesh(g_scene, RTC_GEOMETRY_STATIC, FACES, EDGES, VERTICES);
 
      rtcSetBuffer(g_scene, subdivMeshID, RTC_VERTEX_BUFFER, test_vertices, 0, sizeof(Vec3fa  ));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_INDEX_BUFFER,  test_indices , 0, sizeof(unsigned int));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_OFFSET_BUFFER, test_offsets , 0, sizeof(unsigned int));
    }
    
  rtcCommit(g_scene);

  //g_scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);

  /*! Construct a cube shaped subdivision mesh. */
  //constructCubeMesh();

  /*! Construct a triangle mesh object for the ground plane. */
  //constructGroundPlane();

  /*! Commit the changes to the scene state. */
  //rtcxCommit(g_scene);

}

Vec3fa renderPixelStandard(float x, float y, const Vec3fa &vx, const Vec3fa &vy, const Vec3fa &vz, const Vec3fa &p) {

  /*! Colors of the subdivision mesh and ground plane. */
  Vec3f colors[2];  colors[0] = Vec3f(1.0f, 0.0f, 0.0f);  colors[1] = Vec3f(0.5f, 0.5f, 0.5f);

  /*! Initialize a ray and intersect with the scene. */
  RTCRay ray = constructRay(p, normalize(x * vx + y * vy + vz), 0.0f, inf, RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);  rtcIntersect(g_scene, ray);
  
  /*! The ray may not have hit anything. */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return(Vec3f(0.0f));

  /*! Compute a vector parallel to a directional light. */
  Vec3f lightVector = normalize(Vec3f(-1.0f, -1.0f, -1.0f));

  /*! Initialize a shadow ray and intersect with the scene. */
  RTCRay shadow = constructRay(ray.org + ray.tfar * ray.dir, neg(lightVector), 0.001f, inf, 1, 0);  rtcOccluded(g_scene, shadow);

  /*! Compute a color at the ray hit point. */
  Vec3f color = Vec3f(0.0f), diffuse = colors[ray.geomID % 2];  color = color + diffuse * 0.5f;

  /*! Add contribution from the light. */
  if (shadow.geomID) color = color + diffuse * clamp(-dot(lightVector, (Vec3f)normalize(ray.Ng)), 0.0f, 1.0f);  return(color);

}

void renderTile(int taskIndex, int *pixels, int width, int height, float time, const Vec3fa &vx, const Vec3fa &vy, const Vec3fa &vz, const Vec3fa &p, int tileCountX, int tileCountY) {

  /*! 2D indices of the tile in the window. */
  const Vec2i tileIndex(taskIndex % tileCountX, taskIndex / tileCountX);

  /*! 2D indices of the pixel in the lower left of the tile corner. */
  const int x0 = tileIndex.x * TILE_SIZE_X, y0 = tileIndex.y * TILE_SIZE_Y;

  /*! 2D indices of the pixel in the upper right of the tile corner. */
  const int x1 = min(x0 + TILE_SIZE_X, width), y1 = min(y0 + TILE_SIZE_Y, height);

  /*! Compute the color of each pixel in the tile. */
  for (int y=y0 ; y < y1 ; y++) for (int x=x0 ; x < x1 ; x++) pixels[y * width + x] = packPixel(renderPixel(x, y, vx, vy, vz, p));

}

extern "C" void device_cleanup() {

  rtcDeleteScene(g_scene);
  rtcExit();

}

Vec3fa renderPixelEyeLightTest(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
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

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0,0,1.0f);
  else return Vec3fa(embree::abs(dot(ray.dir,normalize(ray.Ng))));
}

extern "C" void device_init(int8 *configuration) {
  /*! Initialize Embree ray tracing state. */
  rtcInit(configuration);

  /* set error handler */
  rtcSetErrorFunction(error_handler);

  /*! Set the render mode to use on entry into the run loop. */
  //renderPixel = renderPixelStandard;
  renderPixel = renderPixelEyeLightTest;


}

extern "C" void setSubdivisionLevel(unsigned int); // for now hidden fct in the core 
extern unsigned int g_subdivision_levels;

extern "C" void device_render(int *pixels, int width, int height, float time, const Vec3fa &vx, const Vec3fa &vy, const Vec3fa &vz, const Vec3fa &p) {
  if (g_scene == NULL)
    {
      constructScene();
    }

  /*! Refine the subdivision mesh as needed. */
  setSubdivisionLevel( g_subdivision_levels );

  /*! Number of tiles spanning the window in width and height. */
  const Vec2i tileCount((width + TILE_SIZE_X - 1) / TILE_SIZE_X, (height + TILE_SIZE_Y - 1) / TILE_SIZE_Y);

  /*! Render a tile at a time. */
  launch_renderTile(tileCount.x * tileCount.y, pixels, width, height, time, vx, vy, vz, p, tileCount.x, tileCount.y); 

  /*! Debugging information. */
  rtcDebug();

}

