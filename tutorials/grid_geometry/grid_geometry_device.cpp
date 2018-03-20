// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

namespace embree {

/* configuration */
#define EDGE_LEVEL 257
#define GRID_RESOLUTION_X EDGE_LEVEL
#define GRID_RESOLUTION_Y EDGE_LEVEL

/* scene data */
RTCScene g_scene = nullptr;

#if 1

#define NUM_INDICES 80
#define NUM_FACES 22
#define NUM_VERTICES (5+10+5)

/* this geometry is a sphere with a pentagon at the top, 5 quads connected 
to the edges, and triangles between the quads. This is mirrored to make a 
sphere topology. */
unsigned int sphere_indices[NUM_INDICES] =
{
  0, 1, 2, 3, 4,
  0, 14, 5,
  0, 5, 6, 1,
  1, 6, 7,
  1, 7, 8, 2,
  2, 8, 9,
  2, 9, 10, 3,
  3, 10, 11,
  3, 11, 12, 4,
  4, 12, 13,
  4, 13, 14, 0,

  15, 19, 18, 17, 16,
  15, 5, 14,
  15, 16, 6, 5,
  16, 7, 6,
  16, 17, 8, 7,
  17, 9, 8,
  17, 18, 10, 9,
  18, 11, 10,
  18, 19, 12, 11,
  19, 13, 12,
  19, 15, 14, 13,
};

unsigned int sphere_faces[NUM_FACES] = {
  5, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4,
  5, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4,
};

__aligned(16) Vec3fa sphere_vertices[NUM_VERTICES];

#else

#define NUM_INDICES 24
#define NUM_FACES 6
#define NUM_VERTICES 8

unsigned int sphere_indices[NUM_INDICES] = {
  0, 4, 5, 1,
  1, 5, 6, 2,
  2, 6, 7, 3,
  0, 3, 7, 4,
  4, 7, 6, 5,
  0, 1, 2, 3,
};

unsigned int sphere_faces[NUM_FACES] = {
  4, 4, 4, 4, 4, 4
};

__aligned(16) float sphere_vertices[NUM_VERTICES][4] =
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

#endif

struct Grid
{
  unsigned int startVertexID;
  int strideX, strideY;
  unsigned int width, height;
};

struct GridMesh
{
  RTCGeometry geom;
  RTCGeometry geomNormals;
  RTCGrid* egrids;
  Vec3fa* vertices;
  Vec3fa* normals;
};

GridMesh gmesh;

float displacement(const Vec3fa& P)
{
  float dN = 0.0f;
  for (float freq = 1.0f; freq<40.0f; freq*= 2) {
    float n = abs(noise(freq*P));
    dN += 1.4f*n*n/freq;
  }
  return dN;
}

float displacement_du(const Vec3fa& P, const Vec3fa& dPdu)
{
  const float du = 0.001f;
  return (displacement(P+du*dPdu)-displacement(P))/du;
}

float displacement_dv(const Vec3fa& P, const Vec3fa& dPdv)
{
  const float dv = 0.001f;
  return (displacement(P+dv*dPdv)-displacement(P))/dv;
}

void displacementFunction(const struct RTCDisplacementFunctionNArguments* args)
{
  const float* nx = args->Ng_x;
  const float* ny = args->Ng_y;
  const float* nz = args->Ng_z;
  float* px = args->P_x;
  float* py = args->P_y;
  float* pz = args->P_z;
  unsigned int N = args->N;
                                   
  for (unsigned int i=0; i<N; i++) {
    const Vec3fa P = Vec3fa(px[i],py[i],pz[i]);
    const Vec3fa Ng = Vec3fa(nx[i],ny[i],nz[i]);
    const Vec3fa dP = displacement(P)*Ng;
    px[i] += dP.x; py[i] += dP.y; pz[i] += dP.z;
  }
}


Vec3fa getVertex(GridMesh& gmesh, Grid& grid, int x, int y)
{
  int startVertexID = grid.startVertexID;
  int strideX = grid.strideX;
  int strideY = grid.strideY;
  int width   = (int)grid.width;
  int height  = (int)grid.height;
  assert(x >= 0 && x < width);
  assert(y >= 0 && y < height);
  return Vec3fa(gmesh.vertices[startVertexID + y*strideY + x*strideX]);
}

#if defined (ISPC)
Vec3fa getVertex(GridMesh& gmesh, Grid& grid, int x, int y)
{
  int startVertexID = grid.startVertexID;
  int strideX = grid.strideX;
  int strideY = grid.strideY;
  int width   = (int)grid.width;
  int height  = (int)grid.height;
  assert(x >= 0 && x < (int)width);
  assert(y >= 0 && y < (int)height);
  return Vec3fa(gmesh.vertices[startVertexID + y*strideY + x*strideX]);
}
#endif

Vec3fa getVertex(GridMesh& gmesh, RTCGeometry subdiv, Grid *hgrids, int firstHalfEdge, int f, int i, int x, int y, const Vec3fa& opt)
{
  int width  = (int)hgrids[firstHalfEdge].width;
  int height = (int)hgrids[firstHalfEdge].height;
  if (x < 0) {
    assert(x == -1 && y >= 0 && y < height);
    int edge = rtcGetGeometryPreviousHalfEdge(subdiv,firstHalfEdge);
    int oedge = rtcGetGeometryOppositeHalfEdge(subdiv,0,edge);
    if (oedge == edge) return opt; // return alternative vertex when requested vertex does not exist
    return getVertex(gmesh,hgrids[oedge],y,1);
  }
  else if (y < 0) {
    assert(y == -1 && x >= 0 && x < width);
    int oedge = rtcGetGeometryOppositeHalfEdge(subdiv,0,firstHalfEdge);
    if (oedge == firstHalfEdge) return opt; // return alternative vertex when requested vertex does not exist
    int noedge = rtcGetGeometryNextHalfEdge(subdiv,oedge);
    return getVertex(gmesh,hgrids[noedge],1,x);
  }
  else if (x >= width) {
    assert(x == width && y >= 0 && y < height);
    int nedge = rtcGetGeometryNextHalfEdge(subdiv,firstHalfEdge);
    return getVertex(gmesh,hgrids[nedge],y,hgrids[nedge].height-2);
  }
  else if (y >= height) {
    assert(y == height && x >= 0 && x < width);
    int pedge = rtcGetGeometryPreviousHalfEdge(subdiv,firstHalfEdge);
    return getVertex(gmesh,hgrids[pedge],hgrids[pedge].height-2,x);
  }
  else {
    return getVertex(gmesh,hgrids[firstHalfEdge],x,y);
  }
}

/* adds a displaced sphere to the scene */
void createGridGeometry (GridMesh& gmesh)
{
#if 1
  /* calculates top vertex ring */
  for (int i=0; i<5; i++) {
    const float theta = 45.0f*float(pi)/180.0f;
    const float phi = 72.0f*i*float(pi)/180.0f;
    sphere_vertices[i] = Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }

  /* calculates center vertex ring */
  for (int i=0; i<10; i++) {
    const float theta = 90.0f*float(pi)/180.0f;
    const float phi = (18.0f+36.0f*i)*float(pi)/180.0f;
    sphere_vertices[5+i] = Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }

  /* calculates bottom vertex ring */
  for (int i=0; i<5; i++) {
    const float theta = 135.0f*float(pi)/180.0f;
    const float phi = 72.0f*i*float(pi)/180.0f;
    sphere_vertices[5+10+i] = Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }
#endif
  
  /* temporary subdivision geometry to evaluate base surface */
  RTCGeometry geomSubdiv = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
  rtcSetSharedGeometryBuffer(geomSubdiv, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sphere_vertices, 0, sizeof(Vec3fa),       NUM_VERTICES);
  rtcSetSharedGeometryBuffer(geomSubdiv, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT,   sphere_indices,  0, sizeof(unsigned int), NUM_INDICES);
  rtcSetSharedGeometryBuffer(geomSubdiv, RTC_BUFFER_TYPE_FACE,   0, RTC_FORMAT_UINT,   sphere_faces,    0, sizeof(unsigned int), NUM_FACES);
  rtcCommitGeometry(geomSubdiv);

  /* grid resolution has to be uneven as non-quads are split into multiple quads */
  assert((GRID_RESOLUTION_X%2) == 1);
  assert((GRID_RESOLUTION_Y%2) == 1);

  /* subgrid resolution for non-quads */
  unsigned int SUB_GRID_RESOLUTION_X = GRID_RESOLUTION_X/2+1;
  unsigned int SUB_GRID_RESOLUTION_Y = GRID_RESOLUTION_Y/2+1;

  /* grid resolution for quads */
  unsigned int QUAD_GRID_RESOLUTION_X = GRID_RESOLUTION_X;
  unsigned int QUAD_GRID_RESOLUTION_Y = GRID_RESOLUTION_Y;
    
  /* each quad becomes one grid, other faces become multiple grids */
  int numGrids = 0;
  int numVertices = 0;
  for (int f=0; f<NUM_FACES; f++)
  {
    if (sphere_faces[f] == 4)
    {
      numGrids++;
      numVertices += QUAD_GRID_RESOLUTION_X*QUAD_GRID_RESOLUTION_Y;
    }
    else
    {
      numGrids += sphere_faces[f];
      numVertices += sphere_faces[f]*SUB_GRID_RESOLUTION_X*SUB_GRID_RESOLUTION_Y;
    }
  }

  /* create grid geometry */
  gmesh.geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_GRID);
  gmesh.vertices = (Vec3fa *) rtcSetNewGeometryBuffer(gmesh.geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vec3fa),numVertices);
  gmesh.egrids = (RTCGrid *) rtcSetNewGeometryBuffer(gmesh.geom,RTC_BUFFER_TYPE_GRID,0,RTC_FORMAT_GRID,sizeof(RTCGrid),numGrids);
  gmesh.normals = (Vec3fa*) alignedMalloc(numVertices*sizeof(Vec3fa),16);

  Grid* hgrids = (Grid*) alignedMalloc(NUM_INDICES*sizeof(Grid),16);

  unsigned int g=0; // grid index for embree grids
  unsigned int h=0; // grid index for helper grids
  unsigned int startVertexIndex = 0;
  for (unsigned int f=0; f<NUM_FACES; f++) 
  {
    if (sphere_faces[f] == 4)
    {
      gmesh.egrids[g].startVertexID = startVertexIndex;
      gmesh.egrids[g].stride        = QUAD_GRID_RESOLUTION_X;
      gmesh.egrids[g].width         = QUAD_GRID_RESOLUTION_X;
      gmesh.egrids[g].height        = QUAD_GRID_RESOLUTION_Y;

      assert(h < NUM_INDICES);
      hgrids[h].startVertexID = startVertexIndex;
      hgrids[h].strideX       = 1;
      hgrids[h].strideY       = QUAD_GRID_RESOLUTION_X;
      hgrids[h].width         = QUAD_GRID_RESOLUTION_X/2+1;
      hgrids[h].height        = QUAD_GRID_RESOLUTION_Y/2+1;
      h++;

      assert(h < NUM_INDICES);
      hgrids[h].startVertexID = startVertexIndex+QUAD_GRID_RESOLUTION_X-1;
      hgrids[h].strideX       = QUAD_GRID_RESOLUTION_X;
      hgrids[h].strideY       = -1;
      hgrids[h].width         = QUAD_GRID_RESOLUTION_X/2+1;
      hgrids[h].height        = QUAD_GRID_RESOLUTION_Y/2+1;
      h++;

      assert(h < NUM_INDICES);
      hgrids[h].startVertexID = startVertexIndex+QUAD_GRID_RESOLUTION_X*QUAD_GRID_RESOLUTION_Y-1;
      hgrids[h].strideX       = -1;
      hgrids[h].strideY       = -(int)QUAD_GRID_RESOLUTION_X;
      hgrids[h].width         = QUAD_GRID_RESOLUTION_X/2+1;
      hgrids[h].height        = QUAD_GRID_RESOLUTION_Y/2+1;
      h++;

      assert(h < NUM_INDICES);
      hgrids[h].startVertexID = startVertexIndex+(QUAD_GRID_RESOLUTION_X-1)*QUAD_GRID_RESOLUTION_Y;
      hgrids[h].strideX       = -(int)QUAD_GRID_RESOLUTION_X;
      hgrids[h].strideY       = 1;
      hgrids[h].width         = QUAD_GRID_RESOLUTION_X/2+1;
      hgrids[h].height        = QUAD_GRID_RESOLUTION_Y/2+1;
      h++;

      /* calculate displaced vertices for quad-face */
      for (unsigned int y=0; y<QUAD_GRID_RESOLUTION_Y; y++)
      {
        for (unsigned int x=0; x<QUAD_GRID_RESOLUTION_X; x++)
        {
          float u = (float)x / (QUAD_GRID_RESOLUTION_X-1);
          float v = (float)y / (QUAD_GRID_RESOLUTION_Y-1);

          /* evaluate subdiv surface and displace points */
          Vec3fa P,dPdu,dPdv;
          rtcInterpolate1(geomSubdiv,f,u,v,RTC_BUFFER_TYPE_VERTEX,0,&P.x,&dPdu.x,&dPdv.x,3);
          P = P + displacement(P)*normalize(cross(dPdu,dPdv));

          /* write result to vertex buffer */
          gmesh.vertices[startVertexIndex + y * QUAD_GRID_RESOLUTION_X + x] = Vec3fa(P);
          gmesh.normals [startVertexIndex + y * QUAD_GRID_RESOLUTION_X + x] = Vec3fa(0.0f); // calculated later
        }
      }
      startVertexIndex += QUAD_GRID_RESOLUTION_X * QUAD_GRID_RESOLUTION_Y;
      g++;
    }
    else
    {
      /* iterate over all sub-faces */
      for (unsigned int i=0; i<sphere_faces[f]; i++)
      {
        gmesh.egrids[g].startVertexID = startVertexIndex;
        gmesh.egrids[g].stride        = SUB_GRID_RESOLUTION_X;
        gmesh.egrids[g].width         = SUB_GRID_RESOLUTION_X;
        gmesh.egrids[g].height        = SUB_GRID_RESOLUTION_Y;

        hgrids[h].startVertexID = startVertexIndex;
        hgrids[h].strideX       = 1;
        hgrids[h].strideY       = SUB_GRID_RESOLUTION_X;
        hgrids[h].width         = SUB_GRID_RESOLUTION_X;
        hgrids[h].height        = SUB_GRID_RESOLUTION_Y;
        h++;

        /* calculate displaced vertices for sub-face */
        for (unsigned int y=0; y<SUB_GRID_RESOLUTION_Y; y++)
        {
          for (unsigned int x=0; x<SUB_GRID_RESOLUTION_X; x++)
          {
            float u = (float)x / (SUB_GRID_RESOLUTION_X-1);
            float v = (float)y / (SUB_GRID_RESOLUTION_Y-1);

            /* encode UVs */
            const int h = (i >> 2) & 3, l = i & 3;
            const float U = 2.0f*l + 0.5f + u;
            const float V = 2.0f*h + 0.5f + v;

            /* evaluate subdiv surface and displace points */
            Vec3fa P,dPdu,dPdv;
            rtcInterpolate1(geomSubdiv,f,U,V,RTC_BUFFER_TYPE_VERTEX,0,&P.x,&dPdu.x,&dPdv.x,3);
            P = P + displacement(P)*normalize(cross(dPdu,dPdv));

            /* write result to vertex buffer */
            gmesh.vertices[startVertexIndex + y * SUB_GRID_RESOLUTION_X + x] = Vec3fa(P);
            gmesh.normals [startVertexIndex + y * SUB_GRID_RESOLUTION_X + x] = Vec3fa(0.0f); // calculated later
          }
        }
        startVertexIndex += SUB_GRID_RESOLUTION_X * SUB_GRID_RESOLUTION_Y;
        g++;
      }
    }
  }
  
  /* calculate normals by averaging normals of neighboring faces */
  h = 0;
  for (unsigned int f=0; f<NUM_FACES; f++) 
  {
    for (unsigned int i=0; i<sphere_faces[f]; i++)
    {
      for (int y=0; y<(int)SUB_GRID_RESOLUTION_Y; y++)
      {
        for (unsigned int x=0; x<SUB_GRID_RESOLUTION_X; x++)
        {
          Vec3fa p  = getVertex(gmesh,hgrids[h+i],x,y);
          Vec3fa pr = getVertex(gmesh,geomSubdiv,hgrids,h+i,f,i,x+1,y,p);
          Vec3fa pl = getVertex(gmesh,geomSubdiv,hgrids,h+i,f,i,x-1,y,p);
          Vec3fa pt = getVertex(gmesh,geomSubdiv,hgrids,h+i,f,i,x,y+1,p);
          Vec3fa pb = getVertex(gmesh,geomSubdiv,hgrids,h+i,f,i,x,y-1,p);
          Vec3fa Ng = Vec3fa(0.0f);
          Ng = Ng + cross(p-pr,p-pt);
          Ng = Ng + cross(p-pt,p-pl);
          Ng = Ng + cross(p-pl,p-pb);
          Ng = Ng + cross(p-pb,p-pr);
          Ng = normalize(Ng);
          Grid grid = hgrids[h+i];
          int index = grid.startVertexID + y*grid.strideY + x*grid.strideX;
          gmesh.normals[index] = Ng;
        }
      }
    }

    /* First special corner at (0,0). A different number than 4 faces may be 
       connected to this vertex. We need to walk all neighboring faces to 
       calculate a consistent normal. */
    for (unsigned int i=0; i<sphere_faces[f]; i++)
    {
      /* find start of ring */
      bool first = true;
      int startEdge = h+i;
      while (first || startEdge != h+i)
      {
        first = false;
        int oedge = rtcGetGeometryOppositeHalfEdge(geomSubdiv,0,startEdge);
        if (oedge == startEdge) break;
        startEdge = rtcGetGeometryNextHalfEdge(geomSubdiv,oedge);
      }
      
      /* walk ring beginning at start */
      first = true;
      int edge = startEdge;
      Vec3fa Ng = Vec3fa(0.0f);
      Vec3fa p = getVertex(gmesh,hgrids[edge],0,0);
      while (first || edge != startEdge)
      {
        first = false;
        int nedge = rtcGetGeometryNextHalfEdge(geomSubdiv,edge);
        int pedge = rtcGetGeometryPreviousHalfEdge(geomSubdiv,edge);
        Vec3fa p0 = getVertex(gmesh,hgrids[nedge],0,0);
        Vec3fa p1 = getVertex(gmesh,hgrids[pedge],0,0);
        Ng = Ng + cross(p-p0,p-p1);

        int oedge = rtcGetGeometryOppositeHalfEdge(geomSubdiv,0,pedge);
        if (oedge == pedge) break;
        edge = oedge;
      }
      
      Ng = normalize(Ng);
      gmesh.normals[hgrids[h+i].startVertexID] = Ng;
    }

    /* Last special corner at (width-1,height-1). This fixes the center corner 
       for non-quad faces. We need to walk all sub-faces to calculate a 
       consistent normal. */
    
    Vec3fa Ng = Vec3fa(0.0f);
    for (unsigned int i=0; i<sphere_faces[f]; i++)
    {
      Grid& grid = hgrids[h+i];
      Vec3fa p  = getVertex(gmesh,grid,grid.width-1,grid.height-1);
      Vec3fa pl = getVertex(gmesh,grid,grid.width-2,grid.height-1);
      Vec3fa pr = getVertex(gmesh,grid,grid.width-1,grid.height-2);
      Ng = Ng + cross(p-pl,p-pr);
    }
    Ng = normalize(Ng);

    for (unsigned int i=0; i<sphere_faces[f]; i++)
    {
      Grid& grid = hgrids[h+i];
      gmesh.normals[grid.startVertexID + (grid.height-1)*grid.strideY + (grid.width-1)*grid.strideX] = Ng;
    }
    
    h+=sphere_faces[f];
  }

  /* create normal debug geometry */
  gmesh.geomNormals = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE);
  Vec3fa *nvertices = (Vec3fa *) rtcSetNewGeometryBuffer(gmesh.geomNormals,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT4,sizeof(Vec3fa),4*numVertices);
  int*    curves    = (int    *) rtcSetNewGeometryBuffer(gmesh.geomNormals,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT   ,sizeof(int)   ,numVertices);

  h = 0;
  for (unsigned int f=0; f<NUM_FACES; f++) 
  {
    for (unsigned int i=0; i<sphere_faces[f]; i++)
    {
      for (int y=0; y<(int)SUB_GRID_RESOLUTION_Y; y++)
      {
        for (unsigned int x=0; x<SUB_GRID_RESOLUTION_X; x++)
        {
          Grid grid = hgrids[h+i];
          int index = grid.startVertexID + y*grid.strideY + x*grid.strideX;
          Vec3fa Ng = gmesh.normals[index];

          nvertices[4*index+0] = gmesh.vertices[index];
          nvertices[4*index+1] = gmesh.vertices[index];
          nvertices[4*index+2] = gmesh.vertices[index]+0.01f*Vec3fa(Ng);
          nvertices[4*index+3] = gmesh.vertices[index]+0.01f*Vec3fa(Ng);
          nvertices[4*index+0].w = 0.0001f;
          nvertices[4*index+1].w = 0.0001f;
          nvertices[4*index+2].w = 0.0001f;
          nvertices[4*index+3].w = 0.0001f;
          curves[index] = 4*index;
        }
      }
    }
    h+=sphere_faces[f];
  }

  /* we do not need this temporary data anymore */
  rtcReleaseGeometry(geomSubdiv);
  alignedFree(hgrids);
  
  rtcCommitGeometry(gmesh.geom);
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);
  rtcSetSceneFlags(g_scene,RTC_SCENE_FLAG_ROBUST);

  addGroundPlane(g_scene);

  createGridGeometry(gmesh);
  rtcAttachGeometry(g_scene,gmesh.geom);
  //rtcAttachGeometry(g_scene,gmesh.geomNormals);
   
  /* commit changes to scene */
  rtcCommitScene (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

Vec3fa mylerp(float f, const Vec3fa& a, const Vec3fa& b) { // FIXME: use lerpr, need to make ISPC lerpr and C++ lerpr compatible first
  return (1.0f-f)*a + f*b;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  rtcIntersect1(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3fa diffuse = ray.geomID != 0 ? Vec3fa(0.9f,0.6f,0.5f) : Vec3fa(0.8f,0.0f,0.0f);
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    Vec3fa Ng = normalize(ray.Ng);

    if (ray.geomID == 1)
    {
      unsigned int startVertexID = gmesh.egrids[ray.primID].startVertexID;
      int width = gmesh.egrids[ray.primID].width;
      int height = gmesh.egrids[ray.primID].height;
      unsigned int stride = gmesh.egrids[ray.primID].stride;
      float U = ray.u*(width-1);
      float V = ray.v*(height-1);
      int x = min((int)floor(U),width -2);
      int y = min((int)floor(V),height-2);
      float u = U-x;
      float v = V-y;
      Vec3fa N00 = gmesh.normals[startVertexID+(y+0)*stride+(x+0)];
      Vec3fa N01 = gmesh.normals[startVertexID+(y+0)*stride+(x+1)];
      Vec3fa N10 = gmesh.normals[startVertexID+(y+1)*stride+(x+0)];
      Vec3fa N11 = gmesh.normals[startVertexID+(y+1)*stride+(x+1)];
      Vec3fa N0 = mylerp(u,N00,N01);
      Vec3fa N1 = mylerp(u,N10,N11);
      Ng = normalize(mylerp(v,N0,N1));
      //return Ng;
    }

    /* initialize shadow ray */
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

    /* trace shadow ray */
    rtcOccluded1(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-(dot(lightDir,Ng)),0.0f,1.0f);
  }
  return color;
}

/* renders a single screen tile */
void renderTileStandard(int taskIndex,
                        int threadIndex,
                        int* pixels,
                        const unsigned int width,
                        const unsigned int height,
                        const float time,
                        const ISPCCamera& camera,
                        const int numTilesX,
                        const int numTilesY)
{
  const unsigned int tileY = taskIndex / numTilesX;
  const unsigned int tileX = taskIndex - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* calculate pixel color */
    Vec3fa color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int threadIndex, int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         const int numTilesX,
                         const int numTilesY)
{
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  /* render image */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseGeometry(gmesh.geom);
  rtcReleaseScene (g_scene); g_scene = nullptr;
}

} // namespace embree
