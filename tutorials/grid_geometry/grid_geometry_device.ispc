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
#define EDGE_LEVEL 256
#define GRID_RESOLUTION_X EDGE_LEVEL
#define GRID_RESOLUTION_Y EDGE_LEVEL

/* scene data */
RTCScene g_scene = NULL;

#if 1

#define NUM_INDICES 80
#define NUM_FACES 22
#define NUM_VERTICES (5+10+5)

__aligned(16) uniform Vec3fa sphere_vertices[NUM_VERTICES];

uniform RTCGrid* uniform grids = NULL; 
uniform Vec3f* uniform normals = NULL;

uniform unsigned int sphere_indices[NUM_INDICES] = {
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

#else

#define NUM_INDICES 24
#define NUM_FACES 6
//#define FACE_SIZE 4

__aligned(16) uniform float sphere_vertices[8][4] =
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

uniform unsigned int sphere_indices[24] = {
  0, 4, 5, 1,
  1, 5, 6, 2,
  2, 6, 7, 3,
  0, 3, 7, 4,
  4, 7, 6, 5,
  0, 1, 2, 3,
};

uniform unsigned int sphere_faces[6] = {
  4, 4, 4, 4, 4, 4
};

#endif

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

#if 0

Vec3f getVertex(uniform RTCGrid *uniform grid, int x, int y)
{
  uniform int startVertexID = grid->startVertexID;
  uniform int stride = grid->stride;
  unsigned int width = grid->width;
  unsigned int height = grid->height;
  assert(x >= 0 && x < width);
  assert(y >= 0 && y < height);
  return vertices[startVertexID + y*stride + x];
}

Vec3f getVertex(uniform RTCGrid *uniform grid, uniform int firstHalfEdge, uniform int f, uniform int i, int x, int y)
{
  if (x < 0) {
    assert(x == -1 && y >= 0 && y < height);
    uniform int edge = rtcGetGeometryPrevHalfEdge(geom,firstHalfEdge+i);
    edge = rtcGetGeometryOppositeHalfEdge(geom,edge);
    RTCGrid& ogrid = edge_to_grid[edge]
    return getVertex(ogrid,y,1);
  }
  else if (x >= width) {
    assert(x == width && y >= 0 && y < height);
    uniform int edge = rtcGetGeometryNextHalfEdge(geom,firstHalfEdge+i);
    RTCGrid& ogrid = edge_to_grid[edge]
    return getVertex(ogrid,y,ogrid.height-2);
  }
  else if (y < 0) {
    assert(y == -1 && x >= 0 && x < width);
    uniform int edge = rtcGetGeometryOppositeHalfEdge(geom,firstHalfEdge+i);
    edge = rtcGetGeometryNextHalfEdge(geom,edge);
    RTCGrid& ogrid = edge_to_grid[edge]
    return getVertex(ogrid,y,1);
  }
  else if (y >= height) {
    assert(y == height && x >= 0 && x < width);
    uniform int edge = rtcGetGeometryPrevHalfEdge(geom,firstHalfEdge+i);
    RTCGrid& ogrid = edge_to_grid[edge]
    return getVertex(ogrid,y,ogrid.height-2);
  }
  else {
    return getVertex(grid,x,y);
  }
}

#endif

/* adds a cube to the scene */
uniform unsigned int addGridGeometry (RTCScene scene_i)
{
  for (uniform int i=0; i<5; i++) {
    const uniform float theta = 45.0f*pi/180.0f;
    const uniform float phi = 72.0f*i*pi/180.0f;
    sphere_vertices[i] = make_Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }

  for (uniform int i=0; i<10; i++) {
    const uniform float theta = 90.0f*pi/180.0f;
    const uniform float phi = (18.0f+36.0f*i)*pi/180.0f;
    sphere_vertices[5+i] = make_Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }

  for (uniform int i=0; i<5; i++) {
    const uniform float theta = 135.0f*pi/180.0f;
    const uniform float phi = 72.0f*i*pi/180.0f;
    sphere_vertices[5+10+i] = make_Vec3fa(sin(theta)*sin(phi),cos(theta),sin(theta)*cos(phi));
  }
  
  /* create a triangulated cube with 6 quads and 8 vertices */
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);

  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sphere_vertices, 0, sizeof(uniform Vec3fa),       NUM_VERTICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT,   sphere_indices,  0, sizeof(uniform unsigned int), NUM_INDICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,   0, RTC_FORMAT_UINT,   sphere_faces,    0, sizeof(uniform unsigned int), NUM_FACES);

  uniform float* uniform level = (uniform float* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, sizeof(uniform float), NUM_INDICES);
  for (uniform uintptr_t i=0; i<NUM_INDICES; i++) level[i] = (float)EDGE_LEVEL;

  //rtcSetGeometryDisplacementFunction(geom,displacementFunction);
  rtcCommitGeometry(geom);
  
  /* sample subdiv surface to generate grid vertices */
  uniform int numGrids = 0;
  for (uniform int f=0; f<NUM_FACES; f++) {
    //if (sphere_faces[f] == 4) numGrids++;
    //else
    numGrids += sphere_faces[f];
  }

  RTCGeometry geomGrid = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_GRID);
  uniform unsigned int numVertices = GRID_RESOLUTION_X * GRID_RESOLUTION_Y * numGrids;
  
  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex *uniform) rtcSetNewGeometryBuffer(geomGrid,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),numVertices);

  grids = (uniform RTCGrid *uniform) rtcSetNewGeometryBuffer(geomGrid,RTC_BUFFER_TYPE_GRID,0,RTC_FORMAT_GRID,sizeof(uniform RTCGrid),numGrids);
  normals = uniform new uniform Vec3f[numVertices];

  uniform unsigned int g=0;
  uniform unsigned int startVertexIndex = 0;
  for (uniform unsigned int f=0; f<NUM_FACES; f++) 
  {
    /*if (sphere_faces[f] == 4)
    {
      grids[g].startVertexID = startVertexIndex;
      grids[g].stride        = GRID_RESOLUTION_X;
      grids[g].width         = GRID_RESOLUTION_X;
      grids[g].height        = GRID_RESOLUTION_Y;
      
      for (uniform unsigned int y=0; y<GRID_RESOLUTION_Y; y++)
      {
        for (uniform unsigned int x=0; x<GRID_RESOLUTION_X; x+=programCount)
        {
          Vec3f dP,dPdu,dPdv;
          float u = (float)(x+programIndex) / (GRID_RESOLUTION_X-1);
          float v = (float)y / (GRID_RESOLUTION_Y-1);
          rtcInterpolateV1(geom,f,u,v,RTC_BUFFER_TYPE_VERTEX,0,&dP.x,&dPdu.x,&dPdv.x,3);
          Vec3f Ng = normalize(cross(dPdu,dPdv));
          dP = dP + displacement(dP)*Ng;
          Vertex vertex;
          vertex.x = dP.x;
          vertex.y = dP.y;
          vertex.z = dP.z;
          if (x + programIndex < GRID_RESOLUTION_X) {
            vertices[startVertexIndex + y * GRID_RESOLUTION_X + x + programIndex] = vertex;
            normals[startVertexIndex + y * GRID_RESOLUTION_X + x + programIndex] = Ng;
          }
        }
      }
      startVertexIndex += GRID_RESOLUTION_X * GRID_RESOLUTION_Y;
      g++;
    }
    else*/
    {
      for (uniform unsigned int i=0; i<sphere_faces[f]; i++)
      {
        grids[g].startVertexID = startVertexIndex;
        grids[g].stride        = GRID_RESOLUTION_X;
        grids[g].width         = GRID_RESOLUTION_X;
        grids[g].height        = GRID_RESOLUTION_Y;
        
        for (uniform unsigned int y=0; y<GRID_RESOLUTION_Y; y++)
        {
          for (uniform unsigned int x=0; x<GRID_RESOLUTION_X; x+=programCount)
          {
            Vec3f dP,dPdu,dPdv;
            float u = (float)(x+programIndex) / (GRID_RESOLUTION_X-1);
            float v = (float)y / (GRID_RESOLUTION_Y-1);
            if (sphere_faces[f] == 4) {
              Vec2f uv0, du, dv;
              if      (i == 0) { uv0 = make_Vec2f(0.0f,0.0f); du = make_Vec2f(0.5f,0.0f); dv = make_Vec2f(0.0f,0.5f); }
              else if (i == 1) { uv0 = make_Vec2f(1.0f,0.0f); du = make_Vec2f(0.0f,0.5f); dv = make_Vec2f(-0.5f,0.0f); }
              else if (i == 2) { uv0 = make_Vec2f(1.0f,1.0f); du = make_Vec2f(-0.5f,0.0f); dv = make_Vec2f(0.0f,-0.5f); }
              else if (i == 3) { uv0 = make_Vec2f(0.0f,1.0f); du = make_Vec2f(0.0f,-0.5f); dv = make_Vec2f(0.5f,0.0f); }
              Vec2f uv = uv0 + u*du + v*dv;
              rtcInterpolateV1(geom,f,uv.x,uv.y,RTC_BUFFER_TYPE_VERTEX,0,&dP.x,&dPdu.x,&dPdv.x,3);
            }
            else {
              const uniform int h = (i >> 2) & 3, l = i & 3;
              const float U = 2.0f*l + 0.5f + u;
              const float V = 2.0f*h + 0.5f + v;
              rtcInterpolateV1(geom,f,U,V,RTC_BUFFER_TYPE_VERTEX,0,&dP.x,&dPdu.x,&dPdv.x,3);
            }
            Vec3f Ng = normalize(cross(dPdu,dPdv));
            dP = dP + displacement(dP)*Ng;
            Vertex vertex;
            vertex.x = dP.x;
            vertex.y = dP.y;
            vertex.z = dP.z;
            if (x + programIndex < GRID_RESOLUTION_X) {
              vertices[startVertexIndex + y * GRID_RESOLUTION_X + x + programIndex] = vertex;
              normals[startVertexIndex + y * GRID_RESOLUTION_X + x + programIndex] = Ng;
            }
          }
        }
        startVertexIndex += GRID_RESOLUTION_X * GRID_RESOLUTION_Y;
        g++;
      }
    }
  }
#if 0
  /* calculate normals */
  for (uniform unsigned int f=0; f<NUM_FACES; f++) 
  {
    if (sphere_faces[f] == 4)
    {
    }
    else
    {
      assert((GRID_RESOLUTION_X%2) == 0);
      assert((GRID_RESOLUTION_Y%2) == 0);

      for (uniform unsigned int i=0; i<sphere_faces[f]; i++)
      {
        for (uniform int y=0; y<GRID_RESOLUTION_Y; y++)
        {
          for (uniform int x=0; x<GRID_RESOLUTION_X; x+=programCount)
          {
            Vec3fa p  = getVertex(grid,f,i,x,y);
            Vec3fa pr = getVertex(grid,f,i,x+1,y);
            Vec3fa pl = getVertex(grid,f,i,x-1,y);
            Vec3fa pt = getVertex(grid,f,i,x,y+1);
            Vec3fa pb = getVertex(grid,f,i,x,y-1);
            Vec3f Ng = make_Vec3f(0.0f);
            Ng += normalize(cross(p-pr,p-pt));
            Ng += normalize(cross(p-pt,p-pl));
            Ng += normalize(cross(p-pl,p-pb));
            Ng += normalize(cross(p-pb,p-pr));
            if (x + programIndex < GRID_RESOLUTION_X)
              normals[startVertexIndex + y * GRID_RESOLUTION_X + x + programIndex] = normalize(Ng);
          }
        }
      }
    }
  }
#endif

  rtcCommitGeometry(geomGrid);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geomGrid);
  rtcReleaseGeometry(geomGrid);
  rtcReleaseGeometry(geom);

  return geomID;
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

  addGridGeometry(g_scene);
 
  /* commit changes to scene */
  rtcCommitScene (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
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

    /*if (ray.geomID == 1)
    {
      unsigned int startVertexID = grids[ray.primID].startVertexID;
      int width = grids[ray.primID].width;
      int height = grids[ray.primID].height;
      unsigned int stride = grids[ray.primID].stride;
      float U = ray.u*(width-1);
      float V = ray.v*(height-1);
      int x = min((int)floor(U),width -2);
      int y = min((int)floor(V),height-2);
      float u = U-x;
      float v = V-y;
      Vec3f N00 = normals[startVertexID+(y+0)*stride+(x+0)];
      Vec3f N01 = normals[startVertexID+(y+0)*stride+(x+1)];
      Vec3f N10 = normals[startVertexID+(y+1)*stride+(x+0)];
      Vec3f N11 = normals[startVertexID+(y+1)*stride+(x+1)];
      Ng = normalize(lerp(v,lerp(u,N00,N01),lerp(u,N10,N11)));
      return Ng;
      }*/

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
  rtcReleaseScene (g_scene); g_scene = NULL;
}
