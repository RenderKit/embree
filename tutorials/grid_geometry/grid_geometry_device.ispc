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

#include "../common/tutorial/tutorial_device.isph"

/* configuration */
#define EDGE_LEVEL 257
#define GRID_RESOLUTION_X EDGE_LEVEL
#define GRID_RESOLUTION_Y EDGE_LEVEL

/* scene data */
RTCScene g_scene = NULL;

#if 1

#define NUM_INDICES 80
#define NUM_FACES 22
#define NUM_VERTICES (5+10+5)

/* this geometry is a sphere with a pentagon at the top, 5 quads connected 
to the edges, and triangles between the quads. This is mirrored to make a 
sphere topology. */
uniform unsigned int sphere_indices[NUM_INDICES] =
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

uniform unsigned int sphere_faces[NUM_FACES] = {
  5, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4,
  5, 3, 4, 3, 4, 3, 4, 3, 4, 3, 4,
};

__aligned(16) uniform Vec3fa sphere_vertices[NUM_VERTICES];

#else

#define NUM_INDICES 24
#define NUM_FACES 6
#define NUM_VERTICES 8

uniform unsigned int sphere_indices[NUM_INDICES] = {
  0, 4, 5, 1,
  1, 5, 6, 2,
  2, 6, 7, 3,
  0, 3, 7, 4,
  4, 7, 6, 5,
  0, 1, 2, 3,
};

uniform unsigned int sphere_faces[NUM_FACES] = {
  4, 4, 4, 4, 4, 4
};

__aligned(16) uniform float sphere_vertices[NUM_VERTICES][4] =
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
  uniform RTCGrid* uniform egrids;
  uniform Vec3fa* uniform vertices;
  uniform Vec3f* uniform normals;
};

uniform GridMesh gmesh;

float displacement(const Vec3f& P)
{
  float dN = 0.0f;
  for (float freq = 1.0f; freq<40.0f; freq*= 2) {
    float n = abs(noise(freq*P));
    dN += 1.4f*n*n/freq;
  }
  return dN;
}

float displacement_du(const Vec3f& P, const Vec3f& dPdu)
{
  const float du = 0.001f;
  return (displacement(P+du*dPdu)-displacement(P))/du;
}

float displacement_dv(const Vec3f& P, const Vec3f& dPdv)
{
  const float dv = 0.001f;
  return (displacement(P+dv*dPdv)-displacement(P))/dv;
}

unmasked void displacementFunction(const struct RTCDisplacementFunctionNArguments* uniform args)
{
  const uniform float* uniform nx = args->Ng_x;
  const uniform float* uniform ny = args->Ng_y;
  const uniform float* uniform nz = args->Ng_z;
  uniform float* uniform px = args->P_x;
  uniform float* uniform py = args->P_y;
  uniform float* uniform pz = args->P_z;
  uniform unsigned int N = args->N;
                                   
  foreach (i=0 ... N) {
    const Vec3f P = make_Vec3f(px[i],py[i],pz[i]);
    const Vec3f Ng = make_Vec3f(nx[i],ny[i],nz[i]);
    const Vec3f dP = displacement(P)*Ng;
    px[i] += dP.x; py[i] += dP.y; pz[i] += dP.z;
  }
}


uniform Vec3f getVertex(uniform GridMesh& gmesh, uniform Grid& grid, uniform int x, uniform int y)
{
  uniform int startVertexID = grid.startVertexID;
  uniform int strideX = grid.strideX;
  uniform int strideY = grid.strideY;
  uniform int width   = (int)grid.width;
  uniform int height  = (int)grid.height;
  assert(x >= 0 && x < width);
  assert(y >= 0 && y < height);
  return make_Vec3f(gmesh.vertices[startVertexID + y*strideY + x*strideX]);
}

#if defined (ISPC)
varying Vec3f getVertex(uniform GridMesh& gmesh, uniform Grid& grid, varying int x, varying int y)
{
  uniform int startVertexID = grid.startVertexID;
  uniform int strideX = grid.strideX;
  uniform int strideY = grid.strideY;
  uniform int width   = (int)grid.width;
  uniform int height  = (int)grid.height;
  assert(x >= 0 && x < (int)width);
  assert(y >= 0 && y < (int)height);
  return make_Vec3f(gmesh.vertices[startVertexID + y*strideY + x*strideX]);
}
#endif

Vec3f getVertex(uniform GridMesh& gmesh, RTCGeometry subdiv, uniform Grid *uniform hgrids, uniform int firstHalfEdge, uniform int f, uniform int i, int x, int y, const Vec3f& opt)
{
  int width  = (int)hgrids[firstHalfEdge].width;
  int height = (int)hgrids[firstHalfEdge].height;
  if (x < 0) {
    assert(x == -1 && y >= 0 && y < height);
    uniform int edge = rtcGetGeometryPreviousHalfEdge(subdiv,firstHalfEdge);
    uniform int oedge = rtcGetGeometryOppositeHalfEdge(subdiv,0,edge);
    if (oedge == edge) return opt; // return alternative vertex when requested vertex does not exist
    return getVertex(gmesh,hgrids[oedge],y,1);
  }
  else if (y < 0) {
    assert(y == -1 && x >= 0 && x < width);
    uniform int oedge = rtcGetGeometryOppositeHalfEdge(subdiv,0,firstHalfEdge);
    if (oedge == firstHalfEdge) return opt; // return alternative vertex when requested vertex does not exist
    uniform int noedge = rtcGetGeometryNextHalfEdge(subdiv,oedge);
    return getVertex(gmesh,hgrids[noedge],1,x);
  }
  else if (x >= width) {
    assert(x == width && y >= 0 && y < height);
    uniform int nedge = rtcGetGeometryNextHalfEdge(subdiv,firstHalfEdge);
    return getVertex(gmesh,hgrids[nedge],y,hgrids[nedge].height-2);
  }
  else if (y >= height) {
    assert(y == height && x >= 0 && x < width);
    uniform int pedge = rtcGetGeometryPreviousHalfEdge(subdiv,firstHalfEdge);
    return getVertex(gmesh,hgrids[pedge],hgrids[pedge].height-2,x);
  }
  else {
    return getVertex(gmesh,hgrids[firstHalfEdge],x,y);
  }
}

/* adds a displaced sphere to the scene */
void createGridGeometry (uniform GridMesh& gmesh)
{
#if 1
  /* calculates top vertex ring */
  for (uniform int i=0; i<5; i++) {
    const uniform float theta = 45.0f*pi/180.0f;
    const uniform float phi = 72.0f*i*pi/180.0f;
    sphere_vertices[i] = make_Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }

  /* calculates center vertex ring */
  for (uniform int i=0; i<10; i++) {
    const uniform float theta = 90.0f*pi/180.0f;
    const uniform float phi = (18.0f+36.0f*i)*pi/180.0f;
    sphere_vertices[5+i] = make_Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }

  /* calculates bottom vertex ring */
  for (uniform int i=0; i<5; i++) {
    const uniform float theta = 135.0f*pi/180.0f;
    const uniform float phi = 72.0f*i*pi/180.0f;
    sphere_vertices[5+10+i] = make_Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }
#endif
  
  /* temporary subdivision geometry to evaluate base surface */
  RTCGeometry geomSubdiv = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
  rtcSetSharedGeometryBuffer(geomSubdiv, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sphere_vertices, 0, sizeof(uniform Vec3fa),       NUM_VERTICES);
  rtcSetSharedGeometryBuffer(geomSubdiv, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT,   sphere_indices,  0, sizeof(uniform unsigned int), NUM_INDICES);
  rtcSetSharedGeometryBuffer(geomSubdiv, RTC_BUFFER_TYPE_FACE,   0, RTC_FORMAT_UINT,   sphere_faces,    0, sizeof(uniform unsigned int), NUM_FACES);
  rtcCommitGeometry(geomSubdiv);

  /* grid resolution has to be uneven as non-quads are split into multiple quads */
  assert((GRID_RESOLUTION_X%2) == 1);
  assert((GRID_RESOLUTION_Y%2) == 1);

  /* subgrid resolution for non-quads */
  uniform unsigned int SUB_GRID_RESOLUTION_X = GRID_RESOLUTION_X/2+1;
  uniform unsigned int SUB_GRID_RESOLUTION_Y = GRID_RESOLUTION_Y/2+1;

  /* grid resolution for quads */
  uniform unsigned int QUAD_GRID_RESOLUTION_X = GRID_RESOLUTION_X;
  uniform unsigned int QUAD_GRID_RESOLUTION_Y = GRID_RESOLUTION_Y;
    
  /* each quad becomes one grid, other faces become multiple grids */
  uniform int numGrids = 0;
  uniform int numVertices = 0;
  for (uniform int f=0; f<NUM_FACES; f++)
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
  gmesh.vertices = (uniform Vec3fa *uniform) rtcSetNewGeometryBuffer(gmesh.geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vec3fa),numVertices);
  gmesh.egrids = (uniform RTCGrid *uniform) rtcSetNewGeometryBuffer(gmesh.geom,RTC_BUFFER_TYPE_GRID,0,RTC_FORMAT_GRID,sizeof(uniform RTCGrid),numGrids);
  gmesh.normals = uniform new uniform Vec3f[numVertices];

  uniform Grid* uniform hgrids = uniform new uniform Grid[NUM_INDICES];

  uniform unsigned int g=0; // grid index for embree grids
  uniform unsigned int h=0; // grid index for helper grids
  uniform unsigned int startVertexIndex = 0;
  for (uniform unsigned int f=0; f<NUM_FACES; f++) 
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
      for (uniform unsigned int y=0; y<QUAD_GRID_RESOLUTION_Y; y++)
      {
        foreach (x=0 ... QUAD_GRID_RESOLUTION_X)
        {
          float u = (float)x / (QUAD_GRID_RESOLUTION_X-1);
          float v = (float)y / (QUAD_GRID_RESOLUTION_Y-1);

          /* evaluate subdiv surface and displace points */
          Vec3f P,dPdu,dPdv;
          rtcInterpolateV1(geomSubdiv,f,u,v,RTC_BUFFER_TYPE_VERTEX,0,&P.x,&dPdu.x,&dPdv.x,3);
          P = P + displacement(P)*normalize(cross(dPdu,dPdv));

          /* write result to vertex buffer */
          gmesh.vertices[startVertexIndex + y * QUAD_GRID_RESOLUTION_X + x] = make_Vec3fa(P);
          gmesh.normals [startVertexIndex + y * QUAD_GRID_RESOLUTION_X + x] = make_Vec3f(0.0f); // calculated later
        }
      }
      startVertexIndex += QUAD_GRID_RESOLUTION_X * QUAD_GRID_RESOLUTION_Y;
      g++;
    }
    else
    {
      /* iterate over all sub-faces */
      for (uniform unsigned int i=0; i<sphere_faces[f]; i++)
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
        for (uniform unsigned int y=0; y<SUB_GRID_RESOLUTION_Y; y++)
        {
          foreach (x=0 ... SUB_GRID_RESOLUTION_X)
          {
            float u = (float)x / (SUB_GRID_RESOLUTION_X-1);
            float v = (float)y / (SUB_GRID_RESOLUTION_Y-1);

            /* encode UVs */
            const uniform int h = (i >> 2) & 3, l = i & 3;
            const float U = 2.0f*l + 0.5f + u;
            const float V = 2.0f*h + 0.5f + v;

            /* evaluate subdiv surface and displace points */
            Vec3f P,dPdu,dPdv;
            rtcInterpolateV1(geomSubdiv,f,U,V,RTC_BUFFER_TYPE_VERTEX,0,&P.x,&dPdu.x,&dPdv.x,3);
            P = P + displacement(P)*normalize(cross(dPdu,dPdv));

            /* write result to vertex buffer */
            gmesh.vertices[startVertexIndex + y * SUB_GRID_RESOLUTION_X + x] = make_Vec3fa(P);
            gmesh.normals [startVertexIndex + y * SUB_GRID_RESOLUTION_X + x] = make_Vec3f(0.0f); // calculated later
          }
        }
        startVertexIndex += SUB_GRID_RESOLUTION_X * SUB_GRID_RESOLUTION_Y;
        g++;
      }
    }
  }
  
  /* calculate normals by averaging normals of neighboring faces */
  h = 0;
  for (uniform unsigned int f=0; f<NUM_FACES; f++) 
  {
    for (uniform unsigned int i=0; i<sphere_faces[f]; i++)
    {
      for (uniform int y=0; y<(int)SUB_GRID_RESOLUTION_Y; y++)
      {
        foreach (x=0 ... SUB_GRID_RESOLUTION_X)
        {
          Vec3f p  = getVertex(gmesh,hgrids[h+i],x,y);
          Vec3f pr = getVertex(gmesh,geomSubdiv,hgrids,h+i,f,i,x+1,y,p);
          Vec3f pl = getVertex(gmesh,geomSubdiv,hgrids,h+i,f,i,x-1,y,p);
          Vec3f pt = getVertex(gmesh,geomSubdiv,hgrids,h+i,f,i,x,y+1,p);
          Vec3f pb = getVertex(gmesh,geomSubdiv,hgrids,h+i,f,i,x,y-1,p);
          Vec3f Ng = make_Vec3f(0.0f);
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
    for (uniform unsigned int i=0; i<sphere_faces[f]; i++)
    {
      /* find start of ring */
      uniform bool first = true;
      uniform int startEdge = h+i;
      while (first || startEdge != h+i)
      {
        first = false;
        uniform int oedge = rtcGetGeometryOppositeHalfEdge(geomSubdiv,0,startEdge);
        if (oedge == startEdge) break;
        startEdge = rtcGetGeometryNextHalfEdge(geomSubdiv,oedge);
      }
      
      /* walk ring beginning at start */
      first = true;
      uniform int edge = startEdge;
      uniform Vec3f Ng = make_Vec3f(0.0f);
      uniform Vec3f p = getVertex(gmesh,hgrids[edge],0,0);
      while (first || edge != startEdge)
      {
        first = false;
        uniform int nedge = rtcGetGeometryNextHalfEdge(geomSubdiv,edge);
        uniform int pedge = rtcGetGeometryPreviousHalfEdge(geomSubdiv,edge);
        uniform Vec3f p0 = getVertex(gmesh,hgrids[nedge],0,0);
        uniform Vec3f p1 = getVertex(gmesh,hgrids[pedge],0,0);
        Ng = Ng + cross(p-p0,p-p1);

        uniform int oedge = rtcGetGeometryOppositeHalfEdge(geomSubdiv,0,pedge);
        if (oedge == pedge) break;
        edge = oedge;
      }
      
      Ng = normalize(Ng);
      gmesh.normals[hgrids[h+i].startVertexID] = Ng;
    }

    /* Last special corner at (width-1,height-1). This fixes the center corner 
       for non-quad faces. We need to walk all sub-faces to calculate a 
       consistent normal. */
    
    uniform Vec3f Ng = make_Vec3f(0.0f);
    for (uniform unsigned int i=0; i<sphere_faces[f]; i++)
    {
      uniform Grid& grid = hgrids[h+i];
      uniform Vec3f p  = getVertex(gmesh,grid,grid.width-1,grid.height-1);
      uniform Vec3f pl = getVertex(gmesh,grid,grid.width-2,grid.height-1);
      uniform Vec3f pr = getVertex(gmesh,grid,grid.width-1,grid.height-2);
      Ng = Ng + cross(p-pl,p-pr);
    }
    Ng = normalize(Ng);

    for (uniform unsigned int i=0; i<sphere_faces[f]; i++)
    {
      uniform Grid& grid = hgrids[h+i];
      gmesh.normals[grid.startVertexID + (grid.height-1)*grid.strideY + (grid.width-1)*grid.strideX] = Ng;
    }
    
    h+=sphere_faces[f];
  }

  /* create normal debug geometry */
  gmesh.geomNormals = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE);
  uniform Vec3fa *uniform nvertices = (uniform Vec3fa *uniform) rtcSetNewGeometryBuffer(gmesh.geomNormals,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT4,sizeof(uniform Vec3fa),4*numVertices);
  uniform int* uniform    curves    = (uniform int    *uniform) rtcSetNewGeometryBuffer(gmesh.geomNormals,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT   ,sizeof(uniform int)   ,numVertices);

  h = 0;
  for (uniform unsigned int f=0; f<NUM_FACES; f++) 
  {
    for (uniform unsigned int i=0; i<sphere_faces[f]; i++)
    {
      for (uniform int y=0; y<(int)SUB_GRID_RESOLUTION_Y; y++)
      {
        foreach (x=0 ... SUB_GRID_RESOLUTION_X)
        {
          Grid grid = hgrids[h+i];
          int index = grid.startVertexID + y*grid.strideY + x*grid.strideX;
          Vec3f Ng = gmesh.normals[index];

          nvertices[4*index+0] = gmesh.vertices[index];
          nvertices[4*index+1] = gmesh.vertices[index];
          nvertices[4*index+2] = gmesh.vertices[index]+0.01f*make_Vec3fa(Ng);
          nvertices[4*index+3] = gmesh.vertices[index]+0.01f*make_Vec3fa(Ng);
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
  delete[] hgrids;
  
  rtcCommitGeometry(gmesh.geom);
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
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

Vec3f mylerp(float f, const Vec3f& a, const Vec3f& b) { // FIXME: use lerp, need to make ISPC lerp and C++ lerp compatible first
  return (1.0f-f)*a + f*b;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  rtcIntersectV(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3f diffuse = ray.geomID != 0 ? make_Vec3f(0.9f,0.6f,0.5f) : make_Vec3f(0.8f,0.0f,0.0f);
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));
    Vec3f Ng = normalize(ray.Ng);

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
      Vec3f N00 = gmesh.normals[startVertexID+(y+0)*stride+(x+0)];
      Vec3f N01 = gmesh.normals[startVertexID+(y+0)*stride+(x+1)];
      Vec3f N10 = gmesh.normals[startVertexID+(y+1)*stride+(x+0)];
      Vec3f N11 = gmesh.normals[startVertexID+(y+1)*stride+(x+1)];
      Vec3f N0 = mylerp(u,N00,N01);
      Vec3f N1 = mylerp(u,N10,N11);
      Ng = normalize(mylerp(v,N0,N1));
      //return Ng;
    }

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

    /* trace shadow ray */
    rtcOccludedV(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-(dot(lightDir,Ng)),0.0f,1.0f);
  }
  return color;
}

/* renders a single screen tile */
void renderTileStandard(uniform int taskIndex,
                        uniform int threadIndex,
                        uniform int* uniform pixels,
                        const uniform unsigned int width,
                        const uniform unsigned int height,
                        const uniform float time,
                        const uniform ISPCCamera& camera,
                        const uniform int numTilesX,
                        const uniform int numTilesY)
{
  const uniform unsigned int tileY = taskIndex / numTilesX;
  const uniform unsigned int tileX = taskIndex - tileY * numTilesX;
  const uniform unsigned int x0 = tileX * TILE_SIZE_X;
  const uniform unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const uniform unsigned int y0 = tileY * TILE_SIZE_Y;
  const uniform unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* calculate pixel color */
    Vec3f color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
task void renderTileTask(uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const uniform float time,
                         const uniform ISPCCamera& camera,
                         const uniform int numTilesX,
                         const uniform int numTilesY)
{
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  /* render image */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseGeometry(gmesh.geom);
  rtcReleaseScene (g_scene); g_scene = NULL;
}
