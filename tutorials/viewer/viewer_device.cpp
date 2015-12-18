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
#include "../common/tutorial/scene_device.h"
#include "../common/tutorial/random_sampler.h"

extern "C" ISPCScene* g_ispc_scene;
extern "C" bool g_changed;
extern "C" int g_instancing_mode;

/* scene data */
RTCDevice g_device = nullptr;
RTCScene g_scene = nullptr;
RTCScene* geomID_to_scene = nullptr;
ISPCInstance** geomID_to_inst = nullptr;
bool g_subdiv_mode = false;

#define SPP 1

//#define FORCE_FIXED_EDGE_TESSELLATION
#define FIXED_EDGE_TESSELLATION_VALUE 3

#define MAX_EDGE_LEVEL 64.0f
#define MIN_EDGE_LEVEL  4.0f
#define LEVEL_FACTOR   64.0f

inline float updateEdgeLevel( ISPCSubdivMesh* mesh, const Vec3fa& cam_pos, const size_t e0, const size_t e1)
{
  const Vec3fa v0 = mesh->positions[mesh->position_indices[e0]];
  const Vec3fa v1 = mesh->positions[mesh->position_indices[e1]];
  const Vec3fa edge = v1-v0;
  const Vec3fa P = 0.5f*(v1+v0);
  const Vec3fa dist = cam_pos - P;
  return max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
}


void updateEdgeLevelBuffer( ISPCSubdivMesh* mesh, const Vec3fa& cam_pos, size_t startID, size_t endID )
{
  for (size_t f=startID; f<endID;f++) {
       int e = mesh->face_offsets[f];
       int N = mesh->verticesPerFace[f];
       if (N == 4) /* fast path for quads */
         for (size_t i=0; i<4; i++) 
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%4);
       else if (N == 3) /* fast path for triangles */
         for (size_t i=0; i<3; i++) 
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%3);
       else /* fast path for general polygons */
        for (size_t i=0; i<N; i++) 
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%N);              
 }
}

#if defined(ISPC)
task void updateSubMeshEdgeLevelBufferTask( ISPCSubdivMesh* mesh, const Vec3fa& cam_pos )
{
  const size_t size = mesh->numFaces;
  const size_t startID = ((taskIndex+0)*size)/taskCount;
  const size_t endID   = ((taskIndex+1)*size)/taskCount;
  updateEdgeLevelBuffer(mesh,cam_pos,startID,endID);
}
task void updateMeshEdgeLevelBufferTask( ISPCScene* scene_in, const Vec3fa& cam_pos )
{
  ISPCGeometry* geometry = g_ispc_scene->geometries[taskIndex];
  if (geometry->type != SUBDIV_MESH) return;
  ISPCSubdivMesh* mesh = (ISPCSubdivMesh*) geometry;
  unsigned int geomID = mesh->geomID;
  if (mesh->numFaces < 10000) {
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
    rtcUpdateBuffer(g_scene,mesh->geomID,RTC_LEVEL_BUFFER);   
  }
}
#endif

void updateEdgeLevels(ISPCScene* scene_in, const Vec3fa& cam_pos)
{
  /* first update small meshes */
#if defined(ISPC)
  launch[ scene_in->numGeometries ] updateMeshEdgeLevelBufferTask(scene_in,cam_pos); 
#endif

  /* now update large meshes */
  for (size_t g=0; g<scene_in->numGeometries; g++)
  {
    ISPCGeometry* geometry = g_ispc_scene->geometries[g];
    if (geometry->type != SUBDIV_MESH) continue;
    ISPCSubdivMesh* mesh = (ISPCSubdivMesh*) geometry;
#if defined(ISPC)
    if (mesh->numFaces < 10000) continue;
    launch[ getNumHWThreads() ] updateSubMeshEdgeLevelBufferTask(mesh,cam_pos); 	           
#else
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
#endif
    rtcUpdateBuffer(g_scene,mesh->geomID,RTC_LEVEL_BUFFER);    
  }
}

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

bool g_use_smooth_normals = false;
void device_key_pressed(int key)
{
  //printf("key = %\n",key);
  if (key == 115 /*c*/) g_use_smooth_normals = !g_use_smooth_normals;
  else device_key_pressed_default(key);
}

Vec3fa renderPixelEyeLight(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p);


/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);

  /* set start render mode */
  renderPixel = renderPixelStandard;
  //renderPixel = renderPixelEyeLight;	
  key_pressed_handler = device_key_pressed;
}

unsigned int convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices, mesh->positions2 ? 2 : 1);
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa      ));
  if (mesh->positions2) rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER1, mesh->positions2, 0, sizeof(Vec3fa      ));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
  mesh->geomID = geomID;
  return geomID;
}

unsigned int convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewQuadMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numQuads, mesh->numVertices, mesh->positions2 ? 2 : 1);
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa      ));
  if (mesh->positions2) rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER1, mesh->positions2, 0, sizeof(Vec3fa      ));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
  mesh->geomID = geomID;
  return geomID;
}

unsigned int convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewSubdivisionMesh(scene_out, RTC_GEOMETRY_STATIC, mesh->numFaces, mesh->numEdges, mesh->numVertices, 
                                                      mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles);
  mesh->geomID = geomID;												
  for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = FIXED_EDGE_TESSELLATION_VALUE;
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa  ));
  rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float));
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float));
  return geomID;
} 

unsigned int convertLineSegments(ISPCLineSegments* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewLineSegments (scene_out, RTC_GEOMETRY_STATIC, mesh->numSegments, mesh->numVertices, mesh->v2 ? 2 : 1);
  rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,mesh->v,0,sizeof(Vertex));
  if (mesh->v2) rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER1,mesh->v2,0,sizeof(Vertex));
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
  return geomID;
}

unsigned int convertHairSet(ISPCHairSet* hair, RTCScene scene_out)
{
  unsigned int geomID = rtcNewHairGeometry (scene_out, RTC_GEOMETRY_STATIC, hair->numHairs, hair->numVertices, hair->v2 ? 2 : 1);
  rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,hair->v,0,sizeof(Vertex));
  if (hair->v2) rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER1,hair->v2,0,sizeof(Vertex));
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
  return geomID;
}

void convertGroup(ISPCGroup* group, RTCScene scene_out)
{
  for (size_t i=0; i<group->numGeometries; i++)
  {
    ISPCGeometry* geometry = group->geometries[i];
    if (geometry->type == SUBDIV_MESH)
      convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out);
    else if (geometry->type == TRIANGLE_MESH)
      convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out);
    else if (geometry->type == QUAD_MESH)
      convertQuadMesh((ISPCQuadMesh*) geometry, scene_out);
    else if (geometry->type == LINE_SEGMENTS)
      convertLineSegments((ISPCLineSegments*) geometry, scene_out);
    else if (geometry->type == HAIR_SET)
      convertHairSet((ISPCHairSet*) geometry, scene_out);
    else
      assert(false);
  }
}

unsigned int convertInstance(ISPCInstance* instance, int meshID, RTCScene scene_out)
{
  /*if (g_instancing_mode == 1) {
    unsigned int geom_inst = instance->geomID;
    unsigned int geomID = rtcNewGeometryInstance(scene_out, geom_inst);
    rtcSetTransform(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->space.l.vx.x);
    return geomID;
    } else */
  {
    RTCScene scene_inst = geomID_to_scene[instance->geomID];
    unsigned int geomID = rtcNewInstance(scene_out, scene_inst);
    rtcSetTransform(scene_out,geomID,RTC_MATRIX_COLUMN_MAJOR_ALIGNED16,&instance->space.l.vx.x);
    return geomID;
  } 
}     

typedef ISPCInstance* ISPCInstance_ptr;
typedef ISPCGeometry* ISPCGeometry_ptr;

RTCScene convertScene(ISPCScene* scene_in)
{
  for (size_t i=0; i<scene_in->numGeometries; i++)
  {
    ISPCGeometry* geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      g_subdiv_mode = true; break;
    }
  } 

  size_t numGeometries = scene_in->numGeometries;
  geomID_to_scene = new RTCScene[numGeometries];
  geomID_to_inst  = new ISPCInstance_ptr[numGeometries];

  int scene_flags = RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT;
  int scene_aflags = RTC_INTERSECT1 | RTC_INTERPOLATE;
  if (g_subdiv_mode) 
    scene_flags = RTC_SCENE_DYNAMIC | RTC_SCENE_INCOHERENT | RTC_SCENE_ROBUST;

  RTCScene scene_out = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);

   /* use geometry instancing feature */
  if (g_instancing_mode == 1)
  {
    for (size_t i=0; i<scene_in->numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        unsigned int geomID = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out);
        assert(geomID == i); 
        rtcDisable(scene_out,geomID);
      }
      else if (geometry->type == TRIANGLE_MESH) {
        unsigned int geomID = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out);
        assert(geomID == i); 
        rtcDisable(scene_out,geomID);
      }
      else if (geometry->type == QUAD_MESH) {
        unsigned int geomID = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out);
        assert(geomID == i); 
        rtcDisable(scene_out,geomID);
      }
      else if (geometry->type == LINE_SEGMENTS) {
        unsigned int geomID = convertLineSegments((ISPCLineSegments*) geometry, scene_out);
        assert(geomID == i); 
        rtcDisable(scene_out,geomID);
      }
      else if (geometry->type == HAIR_SET) {
        unsigned int geomID = convertHairSet((ISPCHairSet*) geometry, scene_out);
        assert(geomID == i); 
        rtcDisable(scene_out,geomID);
      }
      else if (geometry->type == INSTANCE) {
        unsigned int geomID = convertInstance((ISPCInstance*) geometry, i, scene_out);
        assert(geomID == i); geomID_to_inst[geomID] = (ISPCInstance*) geometry;
      }
      else
        assert(false);
    }
  }

  /* use scene instancing feature */
  else if (g_instancing_mode == 2 || g_instancing_mode == 3)
  {
    for (size_t i=0; i<scene_in->numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        RTCScene objscene = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
        convertSubdivMesh((ISPCSubdivMesh*) geometry, objscene);
        geomID_to_scene[i] = objscene;
        rtcCommit(objscene);
      }
      else if (geometry->type == TRIANGLE_MESH) {
        RTCScene objscene = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
        convertTriangleMesh((ISPCTriangleMesh*) geometry, objscene);
        geomID_to_scene[i] = objscene;
        rtcCommit(objscene);
      }
      else if (geometry->type == QUAD_MESH) {
        RTCScene objscene = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
        convertQuadMesh((ISPCQuadMesh*) geometry, objscene);
        geomID_to_scene[i] = objscene;
        rtcCommit(objscene);
      }
      else if (geometry->type == LINE_SEGMENTS) {
        RTCScene objscene = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
        convertLineSegments((ISPCLineSegments*) geometry, objscene);
        geomID_to_scene[i] = objscene;
        rtcCommit(objscene);
      }
      else if (geometry->type == HAIR_SET) {
        RTCScene objscene = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
        convertHairSet((ISPCHairSet*) geometry, objscene);
        geomID_to_scene[i] = objscene;
        rtcCommit(objscene);
      }
      else if (geometry->type == GROUP) {
        RTCScene objscene = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
        convertGroup((ISPCGroup*) geometry, objscene);
        geomID_to_scene[i] = objscene;
        rtcCommit(objscene);
      }
      else if (geometry->type == INSTANCE) {
        unsigned int geomID = convertInstance((ISPCInstance*) geometry, i, scene_out);
        geomID_to_scene[i] = nullptr; geomID_to_inst[geomID] = (ISPCInstance*) geometry;
      }
      else
        assert(false);
    }
  } 

  /* no instancing */
  else
  {
    for (size_t i=0; i<scene_in->numGeometries; i++)
    {
      ISPCGeometry* geometry = scene_in->geometries[i];
      if (geometry->type == SUBDIV_MESH) {
        unsigned int geomID = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out);
        assert(geomID == i);
      }
      else if (geometry->type == TRIANGLE_MESH) {
        unsigned int geomID = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out);
        assert(geomID == i);
      }
      else if (geometry->type == QUAD_MESH) {
        unsigned int geomID = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out);
        assert(geomID == i);
      }
      else if (geometry->type == LINE_SEGMENTS) {
        unsigned int geomID = convertLineSegments((ISPCLineSegments*) geometry, scene_out);
        assert(geomID == i);
      }
      else if (geometry->type == HAIR_SET) {
        unsigned int geomID = convertHairSet((ISPCHairSet*) geometry, scene_out);
        assert(geomID == i);
      }
      else
        assert(false);
    }
  }

  /* commit changes to scene */
  return scene_out;
}

struct DifferentialGeometry
{
  int geomID;
  int primID;
  float u,v;
  Vec3fa P;
  Vec3fa Ng;
  Vec3fa Ns;
  Vec3fa Tx; //direction along hair
  Vec3fa Ty;
  float tnear_eps;
};

void postIntersectGeometry(const RTCRay& ray, DifferentialGeometry& dg, ISPCGeometry* geometry, int& materialID)
{
  if (geometry->type == TRIANGLE_MESH) 
  {
    ISPCTriangleMesh* mesh = (ISPCTriangleMesh*) geometry;
    materialID = mesh->triangles[ray.primID].materialID;
  }
  else if (geometry->type == QUAD_MESH) 
  {
    ISPCQuadMesh* mesh = (ISPCQuadMesh*) geometry;
    materialID = mesh->meshMaterialID;
  }
  else if (geometry->type == SUBDIV_MESH) 
  {
    ISPCSubdivMesh* mesh = (ISPCSubdivMesh*) geometry;
    materialID = mesh->materialID; 
  }
  else if (geometry->type == LINE_SEGMENTS) 
  {
    ISPCLineSegments* mesh = (ISPCLineSegments*) geometry;
    materialID = mesh->materialID;
  }
  else if (geometry->type == HAIR_SET) 
  {
    ISPCHairSet* mesh = (ISPCHairSet*) geometry;
    materialID = mesh->materialID;
  }
  else if (geometry->type == GROUP) {
    int geomID = ray.geomID; {
      postIntersectGeometry(ray,dg,((ISPCGroup*) geometry)->geometries[geomID],materialID);
    }
  }
  else
    assert(false);
}

inline int postIntersect(const RTCRay& ray, DifferentialGeometry& dg)
{
  int materialID = 0;
  unsigned ray_geomID = g_instancing_mode >= 2 ? ray.instID : ray.geomID;
  int geomID = ray_geomID; 
  {
    /* get instance and geometry pointers */
    ISPCInstance* instance;
    ISPCGeometry* geometry;
    if (g_instancing_mode) {
      instance = geomID_to_inst[geomID];
      geometry = g_ispc_scene->geometries[instance->geomID];
    } else {
      instance = nullptr;
      geometry = g_ispc_scene->geometries[geomID];
    }

    postIntersectGeometry(ray,dg,geometry,materialID);

    /* convert normals */
    if (instance) {
      dg.Ng = dg.Ng.x * Vec3fa(instance->space.l.vx) + dg.Ng.y * Vec3fa(instance->space.l.vy) + dg.Ng.z * Vec3fa(instance->space.l.vz);
      dg.Ns = dg.Ns.x * Vec3fa(instance->space.l.vx) + dg.Ns.y * Vec3fa(instance->space.l.vy) + dg.Ns.z * Vec3fa(instance->space.l.vz);
    }
  }

  return materialID;
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  /* initialize sampler */
  RandomSampler sampler;
  RandomSampler_init(sampler, x, y, 0);

  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = RandomSampler_get1D(sampler);
  
  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  
  /* shade background black */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
    return Vec3fa(0.0f);
  }

  /* shade all rays that hit something */
  Vec3fa color = Vec3fa(0.5f);
  
  /* compute differential geometry */
  DifferentialGeometry dg;
  dg.geomID = ray.geomID;
  dg.primID = ray.primID;
  dg.u = ray.u;
  dg.v = ray.v;
  dg.P  = ray.org+ray.tfar*ray.dir;
  dg.Ng = ray.Ng;
  dg.Ns = ray.Ng;

  if (g_use_smooth_normals)
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID) // FIXME: workaround for ISPC bug, location reached with empty execution mask
  {
    Vec3fa dPdu,dPdv;
    int geomID = ray.geomID; {
      rtcInterpolate(g_scene,geomID,ray.primID,ray.u,ray.v,RTC_VERTEX_BUFFER0,nullptr,&dPdu.x,&dPdv.x,3);
    }
    dg.Ns = cross(dPdv,dPdu);
  }

  int materialID = postIntersect(ray,dg);
  dg.Ng = face_forward(ray.dir,normalize(dg.Ng));
  dg.Ns = face_forward(ray.dir,normalize(dg.Ns));
  
  /* shade */
  if (g_ispc_scene->materials[materialID].ty == MATERIAL_OBJ) {
    OBJMaterial* material = (OBJMaterial*) &g_ispc_scene->materials[materialID];
    color = Vec3fa(material->Kd);
  }

  return color*dot(neg(ray.dir),dg.Ns);
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
  const int t = taskIndex;
  const int tileY = t / numTilesX;
  const int tileX = t - tileY * numTilesX;
  const int x0 = tileX * TILE_SIZE_X;
  const int x1 = min(x0+TILE_SIZE_X,width);
  const int y0 = tileY * TILE_SIZE_Y;
  const int y1 = min(y0+TILE_SIZE_Y,height);


  for (int y = y0; y<y1; y++) for (int x = x0; x<x1; x++)
  {
    Vec3fa color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

Vec3fa old_p; 

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

  /* create scene */
  if (g_scene == nullptr) { 
    g_scene = convertScene(g_ispc_scene);

#if !defined(FORCE_FIXED_EDGE_TESSELLATION)
    if (g_subdiv_mode)
      updateEdgeLevels(g_ispc_scene, cam_org);
#endif
    old_p = p;
    rtcCommit (g_scene);
  }

  bool camera_changed = g_changed; g_changed = false;
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
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
  //rtcDebug();
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
}
