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
#include "../common/tutorial/scene_device.h"
// #include "opensubdiv/hbr/mesh.h"
// #include "extensions/triangulatemesh.h"
// #include "extensions/subdivisionmesh.h"

/*! Function used to render a pixel. */
renderPixelFunc renderPixel;

const int numPhi = 10; 
const int numTheta = 2*numPhi;

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

void DisplacementFunc(void* ptr, unsigned geomID, unsigned primID, float* u, float* v, float* x, float* y, float* z, size_t N)
{
#if 1
  for (size_t i=0; i<N; i++) {
    const Vec3fa dP = 0.02f*Vec3fa(sin(100.0f*x[i]+0.5f),sin(100.0f*z[i]+1.5f),cos(100.0f*y[i]));
    x[i] += dP.x; y[i] += dP.y; z[i] += dP.z;
  }
#else
  for (size_t i=0; i<N; i++) {
    const Vec3fa P(x[i],y[i],z[i]);
    float dN = 0.0f;
    for (float freq = 10.0f; freq<400.0f; freq*= 2) {
      float n = fabs(noise(freq*P));
      dN += 1.4f*n*n/freq;
    }
    const Vec3fa dP = dN*P;
    x[i] += dP.x; y[i] += dP.y; z[i] += dP.z;
  }
#endif
}

unsigned int g_sphere = -1;

/* adds a sphere to the scene */
unsigned int createSphere (RTCGeometryFlags flags, const Vec3fa& pos, const float r)
{
  /* create a triangulated sphere */
  unsigned int mesh = rtcNewSubdivisionMesh(g_scene, flags, numTheta*numPhi, 4*numTheta*numPhi, numTheta*(numPhi+1), 0, 0);
  g_sphere = mesh;

  BBox3fa bounds(Vec3fa(-0.1f,-0.1f,-0.1f),Vec3fa(0.1f,0.1f,0.1f));
  rtcSetDisplacementFunction(g_scene, mesh, (RTCDisplacementFunc)DisplacementFunc,(RTCBounds&)bounds);
  
  /* map buffers */
  Vertex* vertices = (Vertex*  ) rtcMapBuffer(g_scene,mesh,RTC_VERTEX_BUFFER); 
  int*    indices  = (int     *) rtcMapBuffer(g_scene,mesh,RTC_INDEX_BUFFER);
  int*    offsets  = (int     *) rtcMapBuffer(g_scene,mesh,RTC_FACE_BUFFER);
  
  /* create sphere geometry */
  int tri = 0;
  const float rcpNumTheta = rcp((float)numTheta);
  const float rcpNumPhi   = rcp((float)numPhi);
  for (int phi=0; phi<=numPhi; phi++)
  {
    for (int theta=0; theta<numTheta; theta++)
    {
      const float phif   = phi*float(pi)*rcpNumPhi;
      const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
      Vertex& v = vertices[phi*numTheta+theta];
      Vec3fa P(pos.x + r*sin(phif)*sin(thetaf),
               pos.y + r*cos(phif),
               pos.z + r*sin(phif)*cos(thetaf));
      v.x = P.x;
      v.y = P.y;
      v.z = P.z;
      v.r = 3.0f;
    }
    if (phi == 0) continue;

    for (int theta=1; theta<=numTheta; theta++) 
    {
      int p00 = (phi-1)*numTheta+theta-1;
      int p01 = (phi-1)*numTheta+theta%numTheta;
      int p10 = phi*numTheta+theta-1;
      int p11 = phi*numTheta+theta%numTheta;

      indices[4*tri+0] = p10; 
      indices[4*tri+1] = p00; 
      indices[4*tri+2] = p01; 
      indices[4*tri+3] = p11; 
      offsets[tri] = 4;//*tri;
      tri++;
    }
  }
  rtcUnmapBuffer(g_scene,mesh,RTC_VERTEX_BUFFER); 
  rtcUnmapBuffer(g_scene,mesh,RTC_INDEX_BUFFER);
  rtcUnmapBuffer(g_scene,mesh,RTC_FACE_BUFFER);

  return mesh;
}

void updateSphere(const Vec3fa& cam_pos)
{
  if (g_sphere == -1) return;
  Vertex* vertices = (Vertex*  ) rtcMapBuffer(g_scene,g_sphere,RTC_VERTEX_BUFFER); 
  for (size_t i=0; i<numTheta*(numPhi+1); i++) {
    Vec3fa P(vertices[i].x,vertices[i].y,vertices[i].z);
    vertices[i].r = floor(log(100.0f/length(cam_pos-P))/log(2.0f));
  }
  rtcUnmapBuffer(g_scene,g_sphere,RTC_VERTEX_BUFFER); 
  rtcUpdate(g_scene,g_sphere);
  rtcCommit(g_scene);
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

#if 0

#define VERTICES 8
#define EDGES    24
#define FACES    (EDGES/4)

Vec3fa test_vertices[] = {
  Vec3fa(-1.0f, -1.0f, -1.0f, 4.0f),
  Vec3fa( 1.0f, -1.0f, -1.0f, 4.0f),
  Vec3fa( 1.0f, -1.0f,  1.0f, 4.0f),
  Vec3fa(-1.0f, -1.0f,  1.0f, 4.0f),
  Vec3fa(-1.0f,  1.0f, -1.0f, 4.0f),
  Vec3fa( 1.0f,  1.0f, -1.0f, 4.0f),
  Vec3fa( 1.0f,  1.0f,  1.0f, 4.0f),
  Vec3fa(-1.0f,  1.0f,  1.0f, 4.0f)
};

float s = 3.3f;
float        test_creases[EDGES] = { s, 0, 0, 0,  s, 0, 0, 0,  s, 0, 0, 0,  0, 0, 0, s,  0, 0, 0, 0,  s, s, s, s };
unsigned int test_indices[EDGES] = { 0, 1, 5, 4,  1, 2, 6, 5,  2, 3, 7, 6,  0, 4, 7, 3,  4, 5, 6, 7,  0, 3, 2, 1 };

unsigned int test_offsets[FACES] = { 4, 4, 4, 4, 4, 4 };

#else

#define VERTICES 9
#define EDGES    16
#define FACES    (EDGES/4)

Vec3fa test_vertices[] = {
  Vec3fa(-1.0f, -1.0f, 0.0f), Vec3fa( 0.0f, -1.0f, 0.0f), Vec3fa(+1.0f, -1.0f, 0.0f),
  Vec3fa(-1.0f,  0.0f, 1.0f), Vec3fa( 0.0f,  0.0f, 1.0f), Vec3fa(+1.0f,  0.0f, 1.0f),
  Vec3fa(-1.0f, +1.0f, 0.0f), Vec3fa( 0.0f, +1.0f, 0.0f), Vec3fa(+1.0f, +1.0f, 0.0f),
};

float a = 1.0f;
float b = 4.0f;
//float b = 16.0f;
int          test_creases[4] = { 3, 4, 4, 5 };
float        test_crease_weights[2] = { a, b };

int          test_corners[4] = { 0, 2, 6, 8 };
float        test_corner_weights[4] = { 0, 0, 0, 0 };

//float        test_creases[EDGES] = { 0, 0, a, 0,  0, 0, b, 0,  a, 0, 0, 0,  b, 0, 0, 0 };
unsigned int test_indices[EDGES] = { 0, 1, 4, 3,  1, 2, 5, 4,  3, 4, 7, 6,  4, 5, 8, 7 };

unsigned int test_faces[FACES] = { 4, 4, 4, 4 };

#endif

void constructScene(const Vec3fa& cam_pos) 
{
  if (g_ispc_scene)
  {
    /*! Create an Embree object to hold scene state. */
    g_scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);
    
    for (size_t i=0; i<g_ispc_scene->numMeshes; i++)
    {
      ISPCMesh* mesh = g_ispc_scene->meshes[i];
      if (mesh->numQuads == 0) continue;
      
      unsigned int subdivMeshID = rtcNewSubdivisionMesh(g_scene, RTC_GEOMETRY_STATIC, mesh->numQuads, mesh->numQuads*4, mesh->numVertices, 0,0);
      rtcSetBuffer(g_scene, subdivMeshID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa  ));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_INDEX_BUFFER,  mesh->quads    , 0, sizeof(unsigned int));
      
      unsigned int* face_buffer = new unsigned int[mesh->numQuads];
      for (size_t i=0;i<mesh->numQuads;i++) face_buffer[i] = 4;
      rtcSetBuffer(g_scene, subdivMeshID, RTC_FACE_BUFFER, face_buffer    , 0, sizeof(unsigned int));
      //delete face_buffer; // FIXME: never deleted
    }       

    for (size_t i=0; i<g_ispc_scene->numSubdivMeshes; i++)
    {
      ISPCSubdivMesh* mesh = g_ispc_scene->subdiv[i];
      unsigned int subdivMeshID = rtcNewSubdivisionMesh(g_scene, RTC_GEOMETRY_STATIC, mesh->numFaces, mesh->numEdges, mesh->numVertices, mesh->numCreases, mesh->numCorners);
      rtcSetBuffer(g_scene, subdivMeshID, RTC_VERTEX_BUFFER, mesh->vertices , 0, sizeof(Vec3fa  ));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_INDEX_BUFFER,  mesh->indices  , 0, sizeof(unsigned int));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_CREASE_BUFFER, mesh->creases, 0, 2*sizeof(unsigned int));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_CREASE_WEIGHT_BUFFER, mesh->creaseWeights, 0, sizeof(float));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_CORNER_BUFFER, mesh->corners, 0, sizeof(unsigned int));
      rtcSetBuffer(g_scene, subdivMeshID, RTC_CORNER_WEIGHT_BUFFER, mesh->cornerWeights, 0, sizeof(float));
    }       
  }
    
#if 0
  if (totalNumQuads == 0)
  {
    /*! Create an Embree object to hold scene state. */
    g_scene = rtcNewScene(RTC_SCENE_DYNAMIC, RTC_INTERSECT1);

#if 1
    std::cout << "Loading dummy cube..." << std::endl;
    //unsigned int subdivMeshID = rtcNewSubdivisionMesh(g_scene, RTC_GEOMETRY_STATIC, 1, EDGES, VERTICES);
    unsigned int subdivMeshID = rtcNewSubdivisionMesh(g_scene, RTC_GEOMETRY_STATIC, FACES, EDGES, VERTICES, 2, 4);
 
    rtcSetBuffer(g_scene, subdivMeshID, RTC_VERTEX_BUFFER, test_vertices, 0, sizeof(Vec3fa  ));
    rtcSetBuffer(g_scene, subdivMeshID, RTC_INDEX_BUFFER,  test_indices , 0, sizeof(unsigned int));
    rtcSetBuffer(g_scene, subdivMeshID, RTC_FACE_BUFFER, test_faces , 0, sizeof(unsigned int));
    rtcSetBuffer(g_scene, subdivMeshID, RTC_CREASE_BUFFER, test_creases , 0, 2*sizeof(unsigned int));
    rtcSetBuffer(g_scene, subdivMeshID, RTC_CREASE_WEIGHT_BUFFER, test_crease_weights , 0, sizeof(float));
    rtcSetBuffer(g_scene, subdivMeshID, RTC_CORNER_BUFFER, test_corners , 0, sizeof(unsigned int));
    rtcSetBuffer(g_scene, subdivMeshID, RTC_CORNER_WEIGHT_BUFFER, test_corner_weights , 0, sizeof(float));
    
    //BBox3fa bounds(Vec3fa(-0.1f,-0.1f,-0.1f),Vec3fa(0.1f,0.1f,0.1f));
    //rtcSetDisplacementFunction(g_scene, subdivMeshID, (RTCDisplacementFunc)DisplacementFunc,(RTCBounds&)bounds);
#else
    createSphere (RTC_GEOMETRY_STATIC, Vec3fa(0.0f,0.0f,0.0f), 1.0f);
    updateSphere (cam_pos);
#endif
  }
#endif
    
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
  else {
    //DBG_PRINT( embree::abs(dot(ray.dir,normalize(ray.Ng))) );
    return Vec3fa(embree::abs(dot(ray.dir,normalize(ray.Ng))));
  }
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

extern "C" void device_render(int *pixels, int width, int height, float time, const Vec3fa &vx, const Vec3fa &vy, const Vec3fa &vz, const Vec3fa &p) 
{
  if (g_scene == NULL)
      constructScene(p);
  else {
    static Vec3fa oldP = zero;
    if (oldP != p) updateSphere (p);
    oldP = p;
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

