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

#include "../common/tutorial/tutorial_device.h"

//#define FORCE_FIXED_EDGE_TESSELLATION
#define FIXED_EDGE_TESSELLATION_VALUE 16

#define MAX_EDGE_LEVEL 64.0f
#define MIN_EDGE_LEVEL  4.0f
#define LEVEL_FACTOR  128.0f


/* scene data */
RTCDevice g_device = nullptr;
RTCScene g_scene = nullptr;
Vec3fa* vertex_colors = nullptr;
unsigned int triCubeID, quadCubeID, quadCubeID2;

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const RTCError code, const char* str = nullptr)
{
  if (code == RTC_NO_ERROR) 
    return;

  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : printf("RTC_CANCELLED"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  exit(1);
}

#define NUM_VERTICES 8

__aligned(16) float cube_vertices[8][4] = 
{
  { -1.0f, -1.0f, -1.0f, 0.0f },
  {  1.0f, -1.0f, -1.0f, 0.0f },
  {  1.0f, -1.0f,  1.0f, 0.0f },
  { -1.0f, -1.0f,  1.0f, 0.0f },
  { -1.0f,  1.0f, -1.0f, 0.0f },
  {  1.0f,  1.0f, -1.0f, 0.0f },
  {  1.0f,  1.0f,  1.0f, 0.0f },
  { -1.0f,  1.0f,  1.0f, 0.0f }
};

__aligned(16) float cube_vertex_colors[8][4] = 
{
  {  0.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  0.0f,  1.0f, 0.0f },
  {  0.0f,  0.0f,  1.0f, 0.0f },
  {  0.0f,  1.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  1.0f, 0.0f },
  {  0.0f,  1.0f,  1.0f, 0.0f }
};

__aligned(16) float cube_vertex_crease_weights[8] = {
  inf, inf,inf, inf, inf, inf, inf, inf
};

__aligned(16) unsigned int cube_vertex_crease_indices[8] = {
  0,1,2,3,4,5,6,7
};

__aligned(16) float cube_edge_crease_weights[12] = {
  inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf
};

__aligned(16) unsigned int cube_edge_crease_indices[24] = 
{
  0,1, 1,2, 2,3, 3,0,
  4,5, 5,6, 6,7, 7,4,
  0,4, 1,5, 2,6, 3,7,
};

#define NUM_QUAD_INDICES 24
#define NUM_QUAD_FACES 6

unsigned int cube_quad_indices[24] = { 
  0, 1, 5, 4, 
  1, 2, 6, 5, 
  2, 3, 7, 6, 
  0, 4, 7, 3, 
  4, 5, 6, 7, 
  0, 3, 2, 1, 
};

unsigned int cube_quad_faces[6] = { 
  4, 4, 4, 4, 4, 4 
};

#define NUM_TRI_INDICES 36
#define NUM_TRI_FACES 12

unsigned int cube_tri_indices[36] = { 
  1, 5, 4,  0, 1, 4,   
  2, 6, 5,  1, 2, 5,
  3, 7, 6,  2, 3, 6,  
  4, 7, 3,  0, 4, 3,
  5, 6, 7,  4, 5, 7,    
  3, 2, 1,  0, 3, 1 
};

unsigned int cube_tri_faces[12] = { 
  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
};

#define NUM_HAIR_VERTICES 4

__aligned(16) float hair_vertices[4][4] = 
{
  { 0.0f, 0.0f, 0.0f, 0.1f },
  { 0.5f, 1.0f, 0.0f, 0.1f },
  { 0.0f, 2.0f, -0.5f, 0.1f },
  { 0.0f, 3.0f, 0.0f, 0.1f }
};

__aligned(16) float hair_vertex_colors[4][4] = 
{
  {  1.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  0.0f, 0.0f },
  {  0.0f,  0.0f,  1.0f, 0.0f },
  {  1.0f,  1.0f,  1.0f, 0.0f },
};

unsigned int hair_indices[1] = {
  0
};

inline float updateEdgeLevel(const Vec3fa& cam_pos, Vec3fa* vtx, unsigned int* indices, const size_t e0, const size_t e1)
{
  const Vec3fa v0 = vtx[indices[e0]];
  const Vec3fa v1 = vtx[indices[e1]];
  const Vec3fa edge = v1-v0;
  const Vec3fa P = 0.5f*(v1+v0);
  const Vec3fa dist = cam_pos - P;
  const float level = max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
  return level;
}
                                      
/* adds a subdiv cube to the scene */
unsigned int addTriangleSubdivCube (RTCScene scene_i, const Vec3fa& pos)
{
  unsigned int geomID = rtcNewSubdivisionMesh(scene_i, RTC_GEOMETRY_DYNAMIC, NUM_TRI_FACES, NUM_TRI_INDICES, NUM_VERTICES, 0, 0, 0);
  
  //rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER, cube_vertices, 0, sizeof(Vec3fa  ));
  Vec3fa* vtx = (Vec3fa*) rtcMapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  for (size_t i=0; i<NUM_VERTICES; i++) vtx[i] = Vec3fa(cube_vertices[i][0]+pos.x,cube_vertices[i][1]+pos.y,cube_vertices[i][2]+pos.z);
  rtcUnmapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);

  rtcSetBuffer(scene_i, geomID, RTC_INDEX_BUFFER,  cube_tri_indices , 0, sizeof(unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_FACE_BUFFER,   cube_tri_faces,    0, sizeof(unsigned int));

  rtcSetBuffer(scene_i, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,   cube_edge_crease_indices,  0, 2*sizeof(unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,  cube_edge_crease_weights,  0, sizeof(float));

  rtcSetBuffer(scene_i, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER, cube_vertex_crease_indices,0, sizeof(unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER,cube_vertex_crease_weights,0, sizeof(float));

  rtcSetBuffer(scene_i, geomID, RTC_USER_VERTEX_BUFFER0, cube_vertex_colors, 0, sizeof(Vec3fa));

  float* level = (float*) rtcMapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
  for (size_t i=0; i<NUM_TRI_INDICES; i++) level[i] = FIXED_EDGE_TESSELLATION_VALUE;
  rtcUnmapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);

  return geomID;
}

void setTriangleSubdivCubeLevels (RTCScene scene_i, unsigned int geomID, const Vec3fa& cam_pos)
{
  Vec3fa* vtx = (Vec3fa*) rtcMapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  float* level = (float*) rtcMapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);

  for (size_t i=0; i<NUM_TRI_INDICES; i+=3)
  {
    level[i+0] = updateEdgeLevel(cam_pos, vtx, cube_tri_indices, i+0, i+1);
    level[i+1] = updateEdgeLevel(cam_pos, vtx, cube_tri_indices, i+1, i+2);
    level[i+2] = updateEdgeLevel(cam_pos, vtx, cube_tri_indices, i+2, i+0);
  }

  rtcUnmapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
  rtcUnmapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  rtcUpdateBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
}

/* adds a subdiv cube to the scene */
unsigned int addQuadSubdivCube (RTCScene scene_i, const Vec3fa& pos)
{
  unsigned int geomID = rtcNewSubdivisionMesh(scene_i, RTC_GEOMETRY_DYNAMIC, NUM_QUAD_FACES, NUM_QUAD_INDICES, NUM_VERTICES, 0, 0, 0);

  //rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER, cube_vertices, 0, sizeof(Vec3fa  ));
  Vec3fa* vtx = (Vec3fa*) rtcMapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  for (size_t i=0; i<NUM_VERTICES; i++) vtx[i] = Vec3fa(cube_vertices[i][0]+pos.x,cube_vertices[i][1]+pos.y,cube_vertices[i][2]+pos.z);
  rtcUnmapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);

  rtcSetBuffer(scene_i, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, sizeof(unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_FACE_BUFFER,   cube_quad_faces,    0, sizeof(unsigned int));

  rtcSetBuffer(scene_i, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,   cube_edge_crease_indices,  0, 2*sizeof(unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,  cube_edge_crease_weights,  0, sizeof(float));

  rtcSetBuffer(scene_i, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER, cube_vertex_crease_indices,0, sizeof(unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER,cube_vertex_crease_weights,0, sizeof(float));

  rtcSetBuffer(scene_i, geomID, RTC_USER_VERTEX_BUFFER0, cube_vertex_colors, 0, sizeof(Vec3fa));

  float* level = (float*) rtcMapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
  for (size_t i=0; i<NUM_QUAD_INDICES; i++) level[i] = FIXED_EDGE_TESSELLATION_VALUE;
  rtcUnmapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);

  return geomID;
}

void setQuadSubdivCubeLevels (RTCScene scene_i, unsigned int geomID, const Vec3fa& cam_pos)
{
  Vec3fa* vtx = (Vec3fa*) rtcMapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  float* level = (float*) rtcMapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);

  for (size_t i=0; i<NUM_QUAD_INDICES; i+=4)
  {
    level[i+0] = updateEdgeLevel(cam_pos, vtx, cube_quad_indices, i+0, i+1);
    level[i+1] = updateEdgeLevel(cam_pos, vtx, cube_quad_indices, i+1, i+2);
    level[i+2] = updateEdgeLevel(cam_pos, vtx, cube_quad_indices, i+2, i+3);
    level[i+3] = updateEdgeLevel(cam_pos, vtx, cube_quad_indices, i+3, i+0);
  }

  rtcUnmapBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
  rtcUnmapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  rtcUpdateBuffer(scene_i, geomID, RTC_LEVEL_BUFFER);
}

/* adds a cube to the scene */
unsigned int addTriangleCube (RTCScene scene_i, const Vec3fa& pos)
{
  unsigned int geomID = rtcNewTriangleMesh(scene_i, RTC_GEOMETRY_DYNAMIC, NUM_TRI_INDICES/3, NUM_VERTICES);

  //rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER, cube_vertices, 0, sizeof(Vec3fa  ));
  Vec3fa* vtx = (Vec3fa*) rtcMapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);
  for (size_t i=0; i<NUM_VERTICES; i++) vtx[i] = Vec3fa(cube_vertices[i][0]+pos.x,cube_vertices[i][1]+pos.y,cube_vertices[i][2]+pos.z);
  rtcUnmapBuffer(scene_i, geomID, RTC_VERTEX_BUFFER);

  rtcSetBuffer(scene_i, geomID, RTC_INDEX_BUFFER,  cube_tri_indices , 0, 3*sizeof(unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_USER_VERTEX_BUFFER0, cube_vertex_colors, 0, sizeof(Vec3fa));
  return geomID;
}

/* add hair geometry */
unsigned int addHair (RTCScene scene, const Vec3fa& pos)
{
  unsigned int geomID = rtcNewHairGeometry (scene, RTC_GEOMETRY_DYNAMIC, 1, 4);

  //rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER, hair_vertices, 0, sizeof(Vec3fa));
   Vec3fa* vtx = (Vec3fa*) rtcMapBuffer(scene, geomID, RTC_VERTEX_BUFFER);
   for (size_t i=0; i<NUM_HAIR_VERTICES; i++) {
     vtx[i].x = hair_vertices[i][0]+pos.x;
     vtx[i].y = hair_vertices[i][1]+pos.y;
     vtx[i].z = hair_vertices[i][2]+pos.z;
     vtx[i].w = hair_vertices[i][3];
   }
  rtcUnmapBuffer(scene, geomID, RTC_VERTEX_BUFFER);

  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  hair_indices , 0, sizeof(unsigned int));
  rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, hair_vertex_colors, 0, sizeof(Vec3fa));
  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  unsigned int mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);
 
  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_DYNAMIC,RTC_INTERSECT1 | RTC_INTERPOLATE);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* add cube */
  addHair(g_scene,Vec3fa(0.0f,-1.0f,-4.5f));
  addTriangleCube(g_scene,Vec3fa(0.0f,0.0f,5.0f));

  triCubeID  = addTriangleSubdivCube(g_scene,Vec3fa(0.0f,0.0f,1.5f));
  quadCubeID = addQuadSubdivCube(g_scene,Vec3fa(0.0f,0.0f,-1.5f));
  quadCubeID2 = addQuadSubdivCube(g_scene,Vec3fa(0.0f,0.0f,-1.5f));
  rtcDisable(g_scene,quadCubeID2);
  
  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  renderPixel = renderPixelStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
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
  
  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
  {
    /* interpolate diffuse color */

    Vec3fa diffuse = Vec3fa(1.0f,0.0f,0.0f);
    if (ray.geomID > 0) 
    {
      int geomID = ray.geomID; {
        int geom = geomID == quadCubeID ? quadCubeID2 : geomID; // use special interpolation mesh
        rtcInterpolate(g_scene,geom,ray.primID,ray.u,ray.v,RTC_USER_VERTEX_BUFFER0,&diffuse.x,nullptr,nullptr,3); 
      }
      //return diffuse;
      diffuse = 0.5f*diffuse;
    }

    /* calculate smooth shading normal */
    Vec3fa Ng = ray.Ng;
    if (ray.geomID >= 3) {
      Vec3fa dPdu,dPdv;
      int geomID = ray.geomID; {
        rtcInterpolate(g_scene,geomID,ray.primID,ray.u,ray.v,RTC_VERTEX_BUFFER0,nullptr,&dPdu.x,&dPdv.x,3);
      }
      //return dPdu;
      Ng = cross(dPdv,dPdu);
    }
    Ng = normalize(Ng);
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    
    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = ray.org + ray.tfar*ray.dir;
    shadow.dir = neg(lightDir);
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = 1;
    shadow.primID = 0;
    shadow.mask = -1;
    shadow.time = 0;
    
    /* trace shadow ray */
    rtcOccluded(g_scene,shadow);
    
    /* add light contribution */
    if (shadow.geomID) {
      Vec3fa r = normalize(reflect(ray.dir,Ng));
      float s = pow(clamp(dot(r,lightDir),0.0f,1.0f),10.0f);
      float d = clamp(-dot(lightDir,Ng),0.0f,1.0f);
      color = color + diffuse*d + 0.5f*Vec3fa(s);
    }
  }
  return color;
}

/* task that renders a single screen tile */
void renderTile(int taskIndex, int* pixels,
                     const int width,
                     const int height, 
                     const float time,
                     const Vec3fa& vx, 
                     const Vec3fa& vy, 
                     const Vec3fa& vz, 
                     const Vec3fa& p,
                     const int numTilesX, 
                     const int numTilesY)
{
  const int tileY = taskIndex / numTilesX;
  const int tileX = taskIndex - tileY * numTilesX;
  const int x0 = tileX * TILE_SIZE_X;
  const int x1 = min(x0+TILE_SIZE_X,width);
  const int y0 = tileY * TILE_SIZE_Y;
  const int y1 = min(y0+TILE_SIZE_Y,height);

  for (int y = y0; y<y1; y++) for (int x = x0; x<x1; x++)
  {
    /* calculate pixel color */
    Vec3fa color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;  
  }
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                    const int width,
                    const int height,
                    const float time,
                    const Vec3fa& vx, 
                    const Vec3fa& vy, 
                    const Vec3fa& vz, 
                    const Vec3fa& p)
{

  Vec3fa cam_org = Vec3fa(p.x,p.y,p.z);

#if !defined(FORCE_FIXED_EDGE_TESSELLATION)
  setQuadSubdivCubeLevels (g_scene, quadCubeID, cam_org);
  setTriangleSubdivCubeLevels (g_scene, triCubeID, cam_org);
#endif

  rtcCommit(g_scene);

  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
}

