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

#include "../common/tutorial/tutorial_device.isph"
#include "../common/tutorial/scene_device.isph"

#define INSTANCING 0  // 0=no instancing, 1=geometry instancing, 2=scene instancing

extern uniform ISPCScene* uniform g_ispc_scene;
extern uniform bool g_changed;

/* scene data */
RTCDevice g_device = NULL;
RTCScene g_scene = NULL;
uniform ISPCGeometry* uniform* uniform geomID_to_mesh = NULL;
int* uniform meshID_to_geomID = NULL;
RTCScene* uniform meshID_to_scene = NULL;
//uniform int* uniform geomID_to_type = NULL;
uniform bool g_subdiv_mode = false;

#define SPP 1

//#define FORCE_FIXED_EDGE_TESSELLATION
#define FIXED_EDGE_TESSELLATION_VALUE 3

#define MAX_EDGE_LEVEL 64.0f
#define MIN_EDGE_LEVEL  4.0f
#define LEVEL_FACTOR   64.0f

inline uniform float updateEdgeLevel( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos, const uniform size_t e0, const uniform size_t e1)
{
  const uniform Vec3fa v0 = mesh->positions[mesh->position_indices[e0]];
  const uniform Vec3fa v1 = mesh->positions[mesh->position_indices[e1]];
  const uniform Vec3fa edge = v1-v0;
  const uniform Vec3fa P = 0.5f*(v1+v0);
  const uniform Vec3fa dist = cam_pos - P;
  return max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
}


void updateEdgeLevelBuffer( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos, uniform size_t startID, uniform size_t endID )
{
  for (uniform size_t f=startID; f<endID;f++) {
       uniform int e = mesh->face_offsets[f];
       uniform int N = mesh->verticesPerFace[f];
       if (N == 4) /* fast path for quads */
         for (uniform size_t i=0; i<4; i++) 
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%4);
       else if (N == 3) /* fast path for triangles */
         for (uniform size_t i=0; i<3; i++) 
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%3);
       else /* fast path for general polygons */
        for (uniform size_t i=0; i<N; i++) 
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%N);              
 }
}

#if defined(ISPC)
task void updateSubMeshEdgeLevelBufferTask( uniform ISPCSubdivMesh* uniform mesh, const uniform Vec3fa& cam_pos )
{
  const uniform size_t size = mesh->numFaces;
  const uniform size_t startID = ((taskIndex+0)*size)/taskCount;
  const uniform size_t endID   = ((taskIndex+1)*size)/taskCount;
  updateEdgeLevelBuffer(mesh,cam_pos,startID,endID);
}
task void updateMeshEdgeLevelBufferTask( uniform ISPCScene* uniform scene_in, const uniform Vec3fa& cam_pos )
{
  uniform ISPCGeometry* uniform geometry = g_ispc_scene->geometries[taskIndex];
  if (geometry->type != SUBDIV_MESH) return;
  uniform ISPCSubdivMesh* uniform mesh = (uniform ISPCSubdivMesh* uniform) geometry;
  uniform unsigned int geomID = mesh->geomID;
  if (mesh->numFaces < 10000) {
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
    rtcUpdateBuffer(g_scene,mesh->geomID,RTC_LEVEL_BUFFER);   
  }
}
#endif

void updateEdgeLevels(uniform ISPCScene* uniform scene_in, const uniform Vec3fa& cam_pos)
{
  /* first update small meshes */
#if defined(ISPC)
  launch[ scene_in->numGeometries ] updateMeshEdgeLevelBufferTask(scene_in,cam_pos); sync;
#endif

  /* now update large meshes */
  for (uniform size_t g=0; g<scene_in->numGeometries; g++)
  {
    uniform ISPCGeometry* uniform geometry = g_ispc_scene->geometries[g];
    if (geometry->type != SUBDIV_MESH) continue;
    uniform ISPCSubdivMesh* uniform mesh = (uniform ISPCSubdivMesh* uniform) geometry;
    if (mesh->numFaces < 10000) continue;
#if defined(ISPC)
    launch[ getNumHWThreads() ] updateSubMeshEdgeLevelBufferTask(mesh,cam_pos); sync;	           
#else
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
#endif
    rtcUpdateBuffer(g_scene,mesh->geomID,RTC_LEVEL_BUFFER);    
  }
}

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
  case RTC_CANCELLED        : print("RTC_CANCELLED"); break;
  default                   : print("invalid error code"); break;
  }
  if (str) { 
    print(" ("); 
    while (*str) putchar(*str++); 
    print(")\n"); 
  }
  abort();
}

uniform bool g_use_smooth_normals = false;
void device_key_pressed(uniform int key)
{
  //print("key = %\n",key);
  if (key == 115 /*c*/) g_use_smooth_normals = !g_use_smooth_normals;
  else device_key_pressed_default(key);
}

Vec3fa renderPixelEyeLight(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p);


/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);

  /* set start render mode */
  renderPixel = renderPixelStandard;
  //renderPixel = renderPixelEyeLight;	
  key_pressed_handler = device_key_pressed;
}

uniform unsigned int convertTriangleMesh(uniform ISPCTriangleMesh* uniform mesh, uniform int meshID, RTCScene scene_out)
{
  /* create a triangle mesh */
  uniform unsigned int geomID = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices, mesh->positions2 ? 2 : 1);
  
  /* share vertex buffer */
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(uniform Vec3fa      ));
  if (mesh->positions2) rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER1, mesh->positions2, 0, sizeof(uniform Vec3fa      ));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(uniform ISPCTriangle));
  return geomID;
}

uniform unsigned int convertSubdivMesh(uniform ISPCSubdivMesh* uniform mesh, uniform int meshID, RTCScene scene_out)
{
  uniform unsigned int geomID = rtcNewSubdivisionMesh(scene_out, 
                                                      RTC_GEOMETRY_STATIC, 
                                                      mesh->numFaces, mesh->numEdges, mesh->numVertices, 
                                                      mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles);
  mesh->geomID = geomID;												
  geomID_to_mesh[geomID] = (uniform ISPCGeometry* uniform) mesh;
  //geomID_to_type[geomID] = 1; //2
  meshID_to_geomID[meshID] = geomID;
  
  for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = FIXED_EDGE_TESSELLATION_VALUE;
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(uniform Vec3fa  ));
  rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(uniform float));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(uniform float));
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(uniform float));
  return geomID;
}

uniform unsigned int convertInstance(uniform ISPCInstance* uniform instance, uniform int meshID, RTCScene scene_out)
{
#if INSTANCING == 1
  uniform unsigned int geom_inst = meshID_to_geomID[instance->geomID];
  uniform unsigned int geomID = rtcNewGeometryInstance(scene_out, geom_inst);
  rtcSetTransform(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->space.l.vx.x);
  geomID_to_mesh[geomID] = (uniform ISPCGeometry* uniform) instance;
  meshID_to_geomID[meshID] = geomID;
  return geomID;
#elif INSTANCING == 2
  RTCScene scene_inst = meshID_to_scene[instance->geomID];
  uniform unsigned int geomID = rtcNewInstance(scene_out, scene_inst);
  rtcSetTransform(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->space.l.vx.x);
  geomID_to_mesh[geomID] = (uniform ISPCGeometry* uniform) instance;
  meshID_to_geomID[meshID] = geomID;
  return geomID;
#else
  return 0;
#endif
}

RTCScene convertScene(uniform ISPCScene* uniform scene_in)
{
  for (uniform size_t i=0; i<scene_in->numGeometries; i++)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      g_subdiv_mode = true; break;
    }
  } 

  uniform size_t numGeometries = scene_in->numGeometries;
  typedef uniform ISPCGeometry* uniform ISPCGeometry_ptr;
  geomID_to_mesh = uniform new ISPCGeometry_ptr[numGeometries];
  meshID_to_geomID = uniform new uniform int[numGeometries];
  meshID_to_scene = uniform new RTCScene[numGeometries];

  uniform int scene_flags = RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT;
  uniform int scene_aflags = RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING | RTC_INTERPOLATE;
  if (g_subdiv_mode) 
    scene_flags = RTC_SCENE_DYNAMIC | RTC_SCENE_INCOHERENT | RTC_SCENE_ROBUST;

  RTCScene scene_out = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);

#if INSTANCING == 1
  for (uniform size_t i=0; i<scene_in->numGeometries; i++)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      uniform unsigned int geomID = convertSubdivMesh((uniform ISPCSubdivMesh* uniform) geometry, i, scene_out);
      geomID_to_mesh[geomID] = geometry;
      meshID_to_geomID[i] = geomID;
      rtcDisable(scene_out,geomID);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      uniform unsigned int geomID = convertTriangleMesh((uniform ISPCTriangleMesh* uniform) geometry, i, scene_out);
      geomID_to_mesh[geomID] = geometry;
      meshID_to_geomID[i] = geomID;
      rtcDisable(scene_out,geomID);
    }
    else if (geometry->type == INSTANCE)
      convertInstance((uniform ISPCInstance* uniform) geometry, i, scene_out);
  }
#elif INSTANCING == 2
  for (uniform size_t i=0; i<scene_in->numGeometries; i++)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      RTCScene objscene = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
      convertSubdivMesh((uniform ISPCSubdivMesh* uniform) geometry, i, scene_out);
      meshID_to_scene[i] = objscene;
      rtcCommit(objscene);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      RTCScene objscene = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
      convertTriangleMesh((uniform ISPCTriangleMesh* uniform) geometry, i, objscene);
      meshID_to_scene[i] = objscene;
      rtcCommit(objscene);
    }
    else if (geometry->type == INSTANCE) {
      convertInstance((uniform ISPCInstance* uniform) geometry, i, scene_out);
      meshID_to_scene[i] = NULL;
    }
  }
#else
  for (uniform size_t i=0; i<scene_in->numGeometries; i++)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      uniform unsigned int geomID = convertSubdivMesh((uniform ISPCSubdivMesh* uniform) geometry, i, scene_out);
      geomID_to_mesh[geomID] = geometry;
    }
    else if (geometry->type == TRIANGLE_MESH) {
      uniform unsigned int geomID = convertTriangleMesh((uniform ISPCTriangleMesh* uniform) geometry, i, scene_out);
      geomID_to_mesh[geomID] = geometry;
    }
  }
#endif

  /* commit changes to scene */
  return scene_out;
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
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
    return make_Vec3f(0.0f);
  }

#if INSTANCING
  Vec3fa Ng = ray.Ng;
#if INSTANCING == 1
  unsigned instID = ray.geomID;
#else
  unsigned instID = ray.instID;
#endif
  uniform ISPCInstance* instance = (uniform ISPCInstance*) geomID_to_mesh[instID];
  Ng = xfmVector(instance->space,Ng);
  return Vec3fa(abs(dot(normalize(ray.dir),normalize(Ng))));
#endif
  
  /* shade all rays that hit something */
  Vec3f color = make_Vec3f(0.0f);
  Vec3f Ns = ray.Ng;

  if (g_use_smooth_normals)
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID) // FIXME: workaround for ISPC bug, location reached with empty execution mask
  {
    Vec3f dPdu,dPdv;
    foreach_unique (geomID in ray.geomID) {
      rtcInterpolate(g_scene,geomID,ray.primID,ray.u,ray.v,RTC_VERTEX_BUFFER0,NULL,&dPdu.x,&dPdv.x,3);
    }
    Ns = cross(dPdv,dPdu);
  }

  int materialID = 0;
#if 0 // FIXME: pointer gather not implemented on ISPC for Xeon Phi

  uniform ISPCGeometry* varying geometry = geomID_to_mesh[ray.geomID];
  if (geometry->type == TRIANGLE_MESH)
  {
    uniform ISPCTriangleMesh* varying mesh = (uniform ISPCTriangleMesh* varying) geometry;
    uniform ISPCTriangle* varying tri = &mesh->triangles[ray.primID];

    /* load material ID */
    materialID = tri->materialID;

    /* interpolate shading normal */
    if (mesh->normals) {
      Vec3f n0 = make_Vec3f(mesh->normals[tri->v0]);
      Vec3f n1 = make_Vec3f(mesh->normals[tri->v1]);
      Vec3f n2 = make_Vec3f(mesh->normals[tri->v2]);
      float u = ray.u, v = ray.v, w = 1.0f-ray.u-ray.v;
      Ns = normalize(w*n0 + u*n1 + v*n2);
    } else {
      Ns = normalize(ray.Ng);
    }
  } 
  else if (geometry->type == SUBDIV_MESH) {
    uniform ISPCSubdivMesh* varying mesh = (uniform ISPCSubdivMesh* varying) geometry;
    materialID = mesh->materialID;    
  }

#else

  foreach_unique (geomID in ray.geomID) 
  {
    uniform ISPCGeometry* uniform geometry = geomID_to_mesh[geomID];
    if (geometry->type == TRIANGLE_MESH)
    {
      uniform ISPCTriangleMesh* uniform mesh = (uniform ISPCTriangleMesh* uniform) geometry;

      foreach_unique (primID in ray.primID) 
      {
        uniform ISPCTriangle* uniform tri = &mesh->triangles[primID];
      
        /* load material ID */
        materialID = tri->materialID;

        /* interpolate shading normal */
        if (mesh->normals) {
          Vec3f n0 = make_Vec3f(mesh->normals[tri->v0]);
          Vec3f n1 = make_Vec3f(mesh->normals[tri->v1]);
          Vec3f n2 = make_Vec3f(mesh->normals[tri->v2]);
          float u = ray.u, v = ray.v, w = 1.0f-ray.u-ray.v;
          Ns = w*n0 + u*n1 + v*n2;
         } else {
          Ns = normalize(ray.Ng);
         }
      }
    }
    else if (geometry->type == SUBDIV_MESH)
    {
      uniform ISPCSubdivMesh* uniform mesh = (uniform ISPCSubdivMesh* uniform) geometry;
      materialID = mesh->materialID; 
    }
  }
#endif

  Ns = normalize(Ns);
  uniform OBJMaterial* material = (uniform OBJMaterial*) &g_ispc_scene->materials[materialID];
  color = make_Vec3f(material->Kd);

  /* apply ambient light */
  Vec3f Nf = faceforward(Ns,neg(ray.dir),Ns);
  //Vec3f Ng = normalize(ray.Ng);
  //Vec3f Nf = dot(ray.dir,Ng) < 0.0f ? Ng : neg(Ng);
  color = color*dot(ray.dir,Nf);   // FIXME: *=
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

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    Vec3f color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
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

  /* create scene */
  if (g_scene == NULL) { 
    g_scene = convertScene(g_ispc_scene);

#if !defined(FORCE_FIXED_EDGE_TESSELLATION)
    if (g_subdiv_mode)
      updateEdgeLevels(g_ispc_scene, cam_org);
#endif
    old_p = p;
    rtcCommit (g_scene);
  }

  uniform bool camera_changed = g_changed; g_changed = false;
  if ((p.x != old_p.x || p.y != old_p.y || p.z != old_p.z))
    {
     camera_changed = true;
     old_p = p;
    } 

  if (camera_changed) {
#if !defined(FORCE_FIXED_EDGE_TESSELLATION)
    if (g_subdiv_mode)
      {
       updateEdgeLevels(g_ispc_scene, cam_org);
       rtcCommit (g_scene);
      }
#endif
  }


  /* render image */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
  //rtcDebug();
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
}
