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
#include "../common/tutorial/scene_device.isph"

#define PARALLEL_COMMIT
//#define ENABLE_DISPLACEMENTS

//#define FORCE_FIXED_EDGE_TESSELLATION
#define FIXED_EDGE_TESSELLATION_VALUE 4

#define MAX_EDGE_LEVEL 64.0f
#define MIN_EDGE_LEVEL 2.0f
#define ENABLE_DISPLACEMENTS 0
#if ENABLE_DISPLACEMENTS
#  define LEVEL_FACTOR 256.0f
#else
#  define LEVEL_FACTOR 64.0f
#endif

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
  //  renderPixel = renderPixelUV;	
}

task void updateEdgeLevelBuffer(uniform ISPCMesh* uniform mesh, const uniform Vec3fa& cam_pos )
{
  const uniform size_t size = mesh->numQuads;
  const uniform size_t startID = ((threadIndex+0)*size)/threadCount;
  const uniform size_t endID   = ((threadIndex+1)*size)/threadCount;

  for (uniform size_t f=startID;f<endID; f++) 
  {
      const uniform int N = 4;
      for (uniform size_t i=0; i<N; i++) {
      	const uniform int * uniform index = (uniform int * uniform)&mesh->quads[f];	
        const uniform Vec3fa v0 = mesh->positions[index[i]];
        const uniform Vec3fa v1 = mesh->positions[index[(i+1)%N]];
        const uniform Vec3fa edge = v1-v0;
        const uniform Vec3fa P = 0.5f*(v1+v0);
	const uniform Vec3fa dist = cam_pos - P;
        mesh->edge_level[f*4+i] = max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL); // FIXME: map mesh subdivlevel buffer
	assert( mesh->edge_level[f*4+i] >= 1.0f );
      }
  }
}

void updateEdgeLevels(uniform ISPCScene* uniform scene_in, const uniform Vec3fa& cam_pos)
{
  for (uniform int i=0; i<scene_in->numMeshes; i++)
  {
    uniform ISPCMesh* uniform mesh = scene_in->meshes[i];
    if (mesh->edge_level) {
      uniform unsigned int geomID = mesh->geomID;
      launch[  g_threads ] updateEdgeLevelBuffer(mesh,cam_pos); sync;	           
      rtcUpdateBuffer(g_scene,geomID,RTC_LEVEL_BUFFER);
    }
  }

  for (uniform size_t g=0; g<scene_in->numSubdivMeshes; g++)
  {
    uniform ISPCSubdivMesh* uniform mesh = g_ispc_scene->subdiv[g];
    uniform unsigned int geomID = mesh->geomID;
    for (uniform size_t f=0, e=0; f<mesh->numFaces; e+=mesh->verticesPerFace[f++]) {
      uniform int N = mesh->verticesPerFace[f];
      for (uniform size_t i=0; i<N; i++) {
        const uniform Vec3fa v0 = mesh->positions[mesh->position_indices[e+(i+0)]];
        const uniform Vec3fa v1 = mesh->positions[mesh->position_indices[e+(i+1)%N]];
        const uniform Vec3fa edge = v1-v0;
        const uniform Vec3fa P = 0.5f*(v1+v0);
	const uniform Vec3fa dist = cam_pos - P;
        mesh->subdivlevel[e+i] = max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
      }
    }
  }
}

void displacementFunction(void* uniform ptr,               /*!< pointer to user data of geometry */
                          uniform unsigned int geomID,     /*!< ID of geometry to displace */	
                          uniform unsigned int primID,     /*!< ID of primitive of geometry to displace */
                          uniform const float* uniform u,          /*!< u coordinates (source) */
                          uniform const float* uniform v,          /*!< v coordinates (source) */
                          uniform const float* uniform nx,         /*!< x coordinates of normal at point to displace (source) */
                          uniform const float* uniform ny,         /*!< y coordinates of normal at point to displace (source) */
                          uniform const float* uniform nz,         /*!< z coordinates of normal at point to displace (source) */
                          uniform float* uniform px,               /*!< x coordinates of points to displace (source and target) */
                          uniform float* uniform py,               /*!< y coordinates of points to displace (source and target) */
                          uniform float* uniform pz,               /*!< z coordinates of points to displace (source and target) */
                          uniform size_t N                 /*!< number of points to displace */ )
{
 foreach (i = 0 ... 16) {

   const float dP_x = 0.02f*(sin(100.0f*px[i]+0.5f));
   const float dP_y = 0.02f*(sin(100.0f*py[i]+1.5f));
   const float dP_z = 0.02f*(cos(100.0f*(pz[i])));

   px[i] += nx[i] * dP_x; 
   py[i] += ny[i] * dP_y; 
   pz[i] += nz[i] * dP_z;

}
}


void convertScene(uniform ISPCScene* uniform scene_in, const uniform Vec3fa& p)
{
  /* add all meshes to the scene */
  for (uniform int i=0; i<scene_in->numMeshes; i++)
  {
    /* get ith mesh */
    uniform ISPCMesh* uniform mesh = scene_in->meshes[i];
    print("Input: mesh->numQuads %\n",mesh->numQuads);
    if (mesh->numQuads)
      {   
	uniform int   * uniform offset_buffer = uniform new uniform int[mesh->numQuads];
	mesh->edge_level  = uniform new uniform float[mesh->numQuads*4];

	for (size_t i=0;i<mesh->numQuads;i++) offset_buffer[i] = 4;
	for (size_t i=0;i<mesh->numQuads*4;i++) mesh->edge_level[i] = FIXED_EDGE_TESSELLATION_VALUE;

       /* create a triangle mesh */
        uniform unsigned int geomID = rtcNewSubdivisionMesh (g_scene, RTC_GEOMETRY_STATIC, mesh->numQuads, mesh->numQuads*4, mesh->numVertices, 0, 0, 0);
	mesh->geomID = geomID;

	rtcSetBuffer(g_scene, geomID, RTC_VERTEX_BUFFER, mesh->positions , 0, sizeof(uniform Vec3fa  ));
	rtcSetBuffer(g_scene, geomID, RTC_INDEX_BUFFER,  mesh->quads     , 0, sizeof(uniform unsigned int));
	rtcSetBuffer(g_scene, geomID, RTC_FACE_BUFFER,   offset_buffer   , 0, sizeof(uniform unsigned int));
	rtcSetBuffer(g_scene, geomID, RTC_LEVEL_BUFFER,  mesh->edge_level, 0, sizeof(uniform float));
	
#if ENABLE_DISPLACEMENTS == 1
        rtcSetDisplacementFunction(g_scene,geomID,(RTCDisplacementFunc)&displacementFunction,NULL);
#endif
       }
  }

  /* add all subdiv meshes to the scene */
  for (uniform size_t i=0; i<scene_in->numSubdivMeshes; i++)
  {
    uniform ISPCSubdivMesh* uniform mesh = scene_in->subdiv[i];
    uniform unsigned int geomID = rtcNewSubdivisionMesh(g_scene, RTC_GEOMETRY_STATIC, mesh->numFaces, mesh->numEdges, mesh->numVertices, 
							mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles);
    mesh->geomID = geomID;

    rtcSetBuffer(g_scene, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(uniform Vec3fa  ));
    rtcSetBuffer(g_scene, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(uniform float));
    rtcSetBuffer(g_scene, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(uniform unsigned int));
    rtcSetBuffer(g_scene, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(uniform unsigned int));
    rtcSetBuffer(g_scene, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(uniform unsigned int));
    rtcSetBuffer(g_scene, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(uniform unsigned int));
    rtcSetBuffer(g_scene, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(uniform float));
    rtcSetBuffer(g_scene, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(uniform unsigned int));
    rtcSetBuffer(g_scene, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(uniform float));

#if ENABLE_DISPLACEMENTS == 1
    //BBox3fa bounds(Vec3fa(-0.1f,-0.1f,-0.1f),Vec3fa(0.1f,0.1f,0.1f));
    rtcSetDisplacementFunction(g_scene, geomID, (RTCDisplacementFunc)DisplacementFunc,NULL);
#endif
  }
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
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f,0.0f,1.0f);
  
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

export void toggleOpenSubdiv(uniform int key, uniform int x, uniform int y)
{
}



uniform Vec3f old_p; 
 
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
   uniform Vec3fa cam_org = make_Vec3fa(p.x,p.y,p.z);

#if 0
   print("vx % % %\n", vx.x,vx.y,vz.z);
   print("vy % % %\n", vy.x,vy.y,vy.z);
   print("vz % % %\n", vz.x,vz.y,vz.z);
   print("p % % %\n", p.x,p.y,p.z);
#endif

  /* create scene */
  if (g_scene == NULL)
  { 
   //
   g_scene = rtcNewScene(RTC_SCENE_DYNAMIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);
   convertScene(g_ispc_scene,cam_org);

#if !defined(FORCE_FIXED_EDGE_TESSELLATION)
    updateEdgeLevels(g_ispc_scene, cam_org);
#endif

    old_p = p;

#if !defined(PARALLEL_COMMIT)
  rtcCommit (g_scene);
#else
  launch[  g_threads ] parallelCommit(g_scene); sync;
#endif
  }

#if !defined(FORCE_FIXED_EDGE_TESSELLATION)
  {
    if ((p.x != old_p.x | p.y != old_p.y | p.z != old_p.z))
    {
     old_p = p;

   updateEdgeLevels(g_ispc_scene, cam_org);

#if !defined(PARALLEL_COMMIT)
     rtcCommit (g_scene);
#else
     launch[  g_threads ] parallelCommit(g_scene); sync;
#endif
    }
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
  rtcDeleteScene (g_scene);
  rtcExit();
}
