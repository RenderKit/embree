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

/*! Function used to render a pixel. */
renderPixelFunc renderPixel;

const int numPhi = 10; 
const int numTheta = 2*numPhi;

//extern unsigned int g_subdivision_levels;

#define MAX_EDGE_LEVEL 16.0f
#define MIN_EDGE_LEVEL 2.0f

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
RTCScene g_embree_scene = NULL;
RTCScene g_osd_scene = NULL;

/* scene data */

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

void DisplacementFunc(void* ptr, unsigned int geomID, int unsigned primID, 
                      const float* u,      /*!< u coordinates (source) */
                      const float* v,      /*!< v coordinates (source) */
                      const float* nx,     /*!< x coordinates of normal at point to displace (source) */
                      const float* ny,     /*!< y coordinates of normal at point to displace (source) */
                      const float* nz,     /*!< z coordinates of normal at point to displace (source) */
                      float* px,           /*!< x coordinates of points to displace (source and target) */
                      float* py,           /*!< y coordinates of points to displace (source and target) */
                      float* pz,           /*!< z coordinates of points to displace (source and target) */
                      size_t N)
{
 #if 0
  for (size_t i=0; i<N; i++) {
    const Vec3fa dP = 0.02f*Vec3fa(sin(100.0f*px[i]+0.5f),sin(100.0f*pz[i]+1.5f),cos(100.0f*py[i]));
    px[i] += dP.x; py[i] += dP.y; pz[i] += dP.z;
  }
#else
  for (size_t i=0; i<N; i++) {
    const Vec3fa P(px[i],py[i],pz[i]);
    const Vec3fa nor = Vec3fa(nx[i],ny[i],nz[i]);
    float dN = 0.0f;
    for (float freq = 1.0f; freq<40.0f; freq*= 2) {
      float n = fabsf(noise(freq*P));
      dN += 1.4f*n*n/freq;
    }
    const Vec3fa dP = dN*nor;
    px[i] += dP.x; py[i] += dP.y; pz[i] += dP.z;
  }
#endif
}

unsigned int g_sphere = -1;

/* adds a sphere to the scene */
unsigned int createSphere (RTCGeometryFlags flags, const Vec3fa& pos, const float r)
{
  /* create a triangulated sphere */
  unsigned int mesh = rtcNewSubdivisionMesh(g_scene, flags, numTheta*numPhi, 4*numTheta*numPhi, numTheta*(numPhi+1), 0, 0, 0);
  g_sphere = mesh;

  BBox3fa bounds(Vec3fa(-0.1f,-0.1f,-0.1f),Vec3fa(0.1f,0.1f,0.1f));
  rtcSetDisplacementFunction(g_scene, mesh, (RTCDisplacementFunc)DisplacementFunc,(RTCBounds*)&bounds);
  
  /* map buffers */
  Vec3fa* vertices = (Vec3fa*  ) rtcMapBuffer(g_scene,mesh,RTC_VERTEX_BUFFER); 
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
      Vec3fa& v = vertices[phi*numTheta+theta];
      Vec3fa P(pos.x + r*sin(phif)*sin(thetaf),
               pos.y + r*cos(phif),
               pos.z + r*sin(phif)*cos(thetaf));
      v.x = P.x;
      v.y = P.y;
      v.z = P.z;
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

extern float g_debug;


void updateScene(RTCScene scene, const Vec3fa& cam_pos)
{
  if (!g_ispc_scene) return;
  if (g_debug != 0.0f) return;

  for (size_t j=0; j<g_ispc_scene->numMeshes; j++)
  {
    ISPCMesh* mesh = g_ispc_scene->meshes[j];
    if (mesh->numQuads == 0) continue;

    unsigned int geomID = mesh->geomID;
    float* level = (float*) rtcMapBuffer(scene, geomID, RTC_LEVEL_BUFFER);
    for (size_t i=0; i<mesh->numQuads; i++) {
      unsigned int *quad_vtx = (unsigned int*)&mesh->quads[i];
 
      for (size_t k=0; k<4; k++) {
        const Vec3fa v0 = mesh->positions[quad_vtx[(k+0)%4]];
        const Vec3fa v1 = mesh->positions[quad_vtx[(k+1)%4]];
        const Vec3fa edge = v1-v0;
        const Vec3fa P = 0.5f*(v1+v0);
	const Vec3fa dist = cam_pos - P;

        level[i*4+k] = max(min(64.0f*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
        //level[i*4+k] = 4; // MAX_EDGE_LEVEL;

      } 
    }
    rtcUnmapBuffer(scene,geomID, RTC_LEVEL_BUFFER);
    rtcUpdate(scene,geomID);
  }

  for (size_t g=0; g<g_ispc_scene->numSubdivMeshes; g++)
  {
    ISPCSubdivMesh* mesh = g_ispc_scene->subdiv[g];
    unsigned int geomID = mesh->geomID;
    for (size_t f=0, e=0; f<mesh->numFaces; e+=mesh->verticesPerFace[f++]) {
      const int N = mesh->verticesPerFace[f];
      for (size_t i=0; i<N; i++) {
        const Vec3fa v0 = mesh->positions[mesh->position_indices[e+(i+0)]];
        const Vec3fa v1 = mesh->positions[mesh->position_indices[e+(i+1)%N]];
        const Vec3fa edge = v1-v0;
        const Vec3fa P = 0.5f*(v1+v0);
	const Vec3fa dist = cam_pos - P;

        //mesh->subdivlevel[e+i] = float(g_subdivision_levels)/16.0f;
        //mesh->subdivlevel[e+i] = 200.0f*atan(0.5f*length(edge)/length(cam_pos-P));
	//mesh->subdivlevel[e+i] = 60.0f*atan(0.5f*length(edge)/length(Vec3fa(2.86598f, -0.784929f, -0.0090338f)-P));
        //mesh->subdivlevel[e+i] = max(min(64.0f*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
	mesh->subdivlevel[e+i] = 17;

        //srand48(length(edge)/length(cam_pos-P)*12343.0f); mesh->subdivlevel[e+i] = 10.0f*drand48();
      }
    }
    /*for (size_t i=0; i<8; i++) mesh->subdivlevel[i] = 16.2;
    //for (size_t i=0; i<8; i++) mesh->subdivlevel[i] = 17.2;
    float level = float(g_subdivision_levels)/16.0f;
    mesh->subdivlevel[0] = level;
    mesh->subdivlevel[1] = level;
    mesh->subdivlevel[2] = level;
    mesh->subdivlevel[3] = level;
    mesh->subdivlevel[7] = level;*/

    rtcUpdate(scene,geomID);
  }
  rtcCommit(scene);
}

RTCScene constructScene(const Vec3fa& cam_pos) 
{
  if (!g_ispc_scene) return NULL;

  /*! Create an Embree object to hold scene state. */
  //RTCScene scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);
  RTCScene scene = rtcNewScene(RTC_SCENE_DYNAMIC, RTC_INTERSECT1);
  
  for (size_t i=0; i<g_ispc_scene->numMeshes; i++)
  {
    ISPCMesh* mesh = g_ispc_scene->meshes[i];
    if (mesh->numQuads == 0) continue;
    
    unsigned int subdivMeshID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, mesh->numQuads, mesh->numQuads*4, mesh->numVertices, 0,0,0);
    rtcSetBuffer(scene, subdivMeshID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa  ));
    rtcSetBuffer(scene, subdivMeshID, RTC_INDEX_BUFFER,  mesh->quads    , 0, sizeof(unsigned int));
    
    unsigned int* face_buffer = new unsigned int[mesh->numQuads];
    for (size_t i=0;i<mesh->numQuads;i++) face_buffer[i] = 4;
    rtcSetBuffer(scene, subdivMeshID, RTC_FACE_BUFFER, face_buffer    , 0, sizeof(unsigned int));
    //delete face_buffer; // FIXME: never deleted

    float* level = (float*) rtcMapBuffer(scene, subdivMeshID, RTC_LEVEL_BUFFER);
    for (size_t i=0; i<4*mesh->numQuads; i++) level[i] = 4; // 16
    //for (size_t i=0; i<4*mesh->numQuads; i++) level[i] = 32;
    rtcUnmapBuffer(scene,subdivMeshID, RTC_LEVEL_BUFFER);

    //BBox3fa bounds(Vec3fa(-0.1f,-0.1f,-0.1f),Vec3fa(0.1f,0.1f,0.1f));
    //rtcSetDisplacementFunction(scene, subdivMeshID, (RTCDisplacementFunc)DisplacementFunc,(RTCBounds*)&bounds);
    //rtcSetDisplacementFunction(scene, subdivMeshID, (RTCDisplacementFunc)DisplacementFunc,NULL);
    mesh->geomID = subdivMeshID;
  }       
  
  for (size_t i=0; i<g_ispc_scene->numSubdivMeshes; i++)
  {
    ISPCSubdivMesh* mesh = g_ispc_scene->subdiv[i];
    unsigned int geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, mesh->numFaces, mesh->numEdges, mesh->numVertices, 
                                                      mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles);

    //BBox3fa bounds(Vec3fa(-0.1f,-0.1f,-0.1f),Vec3fa(0.1f,0.1f,0.1f));
    //rtcSetDisplacementFunction(scene, geomID, (RTCDisplacementFunc)DisplacementFunc,(RTCBounds*)&bounds);

    rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa  ));
    rtcSetBuffer(scene, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float));
    rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int));
    rtcSetBuffer(scene, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int));
    rtcSetBuffer(scene, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int));
    rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_BUFFER,          mesh->edge_creases,          0, 2*sizeof(unsigned int));
    rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float));
    rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_BUFFER,        mesh->vertex_creases,        0, sizeof(unsigned int));
    rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float));
    mesh->geomID = geomID;
  }       
  
  updateScene(scene,cam_pos);
  return scene;
}

#if defined(__USE_OPENSUBDIV__)

#include <opensubdiv/far/topologyRefinerFactory.h>
using namespace OpenSubdiv;

struct OSDVertex {

    // Minimal required interface ----------------------
    OSDVertex() { }

    OSDVertex(OSDVertex const & src) {
        _position[0] = src._position[0];
        _position[1] = src._position[1];
        _position[1] = src._position[1];
    }

    void Clear( void * =0 ) {
        _position[0]=_position[1]=_position[2]=0.0f;
    }

    void AddWithWeight(OSDVertex const & src, float weight) {
        _position[0]+=weight*src._position[0];
        _position[1]+=weight*src._position[1];
        _position[2]+=weight*src._position[2];
    }

    void AddVaryingWithWeight(OSDVertex const &, float) { }

    // Public interface ------------------------------------
    void SetPosition(float x, float y, float z) {
        _position[0]=x;
        _position[1]=y;
        _position[2]=z;
    }

    const float * GetPosition() const {
        return _position;
    }

private:
    float _position[3];
};

RTCScene constructSceneOpenSubdiv() 
{
  if (!g_ispc_scene) return NULL;

  typedef Far::TopologyRefinerFactoryBase::TopologyDescriptor Descriptor;

  Sdc::Options options;
  options.SetVVarBoundaryInterpolation(Sdc::Options::VVAR_BOUNDARY_EDGE_ONLY);
  options.SetCreasingMethod(Sdc::Options::CREASE_CHAIKIN);

  RTCScene scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);
  
  for (size_t i=0; i<g_ispc_scene->numSubdivMeshes; i++)
  {
    ISPCSubdivMesh* mesh = g_ispc_scene->subdiv[i];
    
    Descriptor desc;
    desc.numVertices  = mesh->numVertices;
    desc.numFaces     = mesh->numFaces;
    desc.vertsPerFace = mesh->verticesPerFace;
    desc.vertIndices  = mesh->position_indices;
    desc.numCreases   = mesh->numEdgeCreases;
    desc.creaseVertexIndexPairs = (int*) mesh->edge_creases;
    desc.creaseWeights = mesh->edge_crease_weights;
    desc.numCorners    = mesh->numVertexCreases;
    desc.cornerVertexIndices = mesh->vertex_creases;
    desc.cornerWeights = mesh->vertex_crease_weights;
    
    size_t maxlevel = 5;
    Far::TopologyRefiner* refiner = Far::TopologyRefinerFactory<Descriptor>::Create(OpenSubdiv::Sdc::TYPE_CATMARK, options, desc);
    refiner->RefineUniform(maxlevel);
    
    std::vector<OSDVertex> vbuffer(refiner->GetNumVerticesTotal());
    OSDVertex* verts = &vbuffer[0];
    
    for (int i=0; i<mesh->numVertices; ++i)
      verts[i].SetPosition(mesh->positions[i].x,mesh->positions[i].y,mesh->positions[i].z);
    
    refiner->Interpolate(verts, verts + mesh->numVertices);
    
    for (int level=0; level<maxlevel; ++level)
        verts += refiner->GetNumVertices(level);
    
    const size_t numVertices = refiner->GetNumVertices(maxlevel);
    const size_t numFaces    = refiner->GetNumFaces(maxlevel);
    
    unsigned int meshID = rtcNewTriangleMesh(scene, RTC_GEOMETRY_STATIC, 2*numFaces, numVertices);
    rtcSetBuffer(scene, meshID, RTC_VERTEX_BUFFER, verts, 0, sizeof(Vec3f));
    
    Vec3i* tris = (Vec3i*) rtcMapBuffer(scene, meshID, RTC_INDEX_BUFFER);
    for (size_t i=0; i<numFaces; i++) {
      Far::IndexArray fverts = refiner->GetFaceVertices(maxlevel, i);
      assert(fverts.size() == 4);
      tris[2*i+0] = Vec3i(fverts[0],fverts[1],fverts[2]);
      tris[2*i+1] = Vec3i(fverts[2],fverts[3],fverts[0]);
    }
    rtcUnmapBuffer(scene,meshID,RTC_INDEX_BUFFER);
  }
  rtcCommit(scene);
  return scene;
}

#endif

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

extern "C" void toggleOpenSubdiv(unsigned char key, int x, int y)
{
#if defined(__USE_OPENSUBDIV__)
  if (g_osd_scene == NULL) {
    g_osd_scene = constructSceneOpenSubdiv();
    g_embree_scene = g_scene;
  }
  if (g_scene == g_embree_scene) g_scene = g_osd_scene;
  else                           g_scene = g_embree_scene;
#endif
}

extern "C" void device_render(int *pixels, int width, int height, float time, const Vec3fa &vx, const Vec3fa &vy, const Vec3fa &vz, const Vec3fa &p) 
{
  if (g_scene == NULL) {
    g_scene = constructScene(p);
  } else {
    static Vec3fa oldP = zero;
    if (oldP != p) 
      updateScene (g_scene,p);
    oldP = p;
  }
  
  /*! Number of tiles spanning the window in width and height. */
  const Vec2i tileCount((width + TILE_SIZE_X - 1) / TILE_SIZE_X, (height + TILE_SIZE_Y - 1) / TILE_SIZE_Y);

  /*! Render a tile at a time. */
  launch_renderTile(tileCount.x * tileCount.y, pixels, width, height, time, vx, vy, vz, p, tileCount.x, tileCount.y); 

  /*! Debugging information. */
  rtcDebug();

}

