// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "verify.h"
#include <regex>

#define DEFAULT_STACK_SIZE 4*1024*1024

#if defined(__WIN32__)
#  define GREEN(x) x
#  define RED(x) x
#else
#  define GREEN(x) "\033[32m" x "\033[0m"
#  define RED(x) "\033[31m" x "\033[0m"
#endif

#if defined(__INTEL_COMPILER)
#pragma warning (disable: 1478) // warning: function was declared deprecated
#elif defined(_MSC_VER)
#pragma warning (disable: 4996) // warning: function was declared deprecated
#elif defined(__clang__)
#pragma clang diagnostic ignored "-Wdeprecated-declarations" // warning: xxx is deprecated
#elif defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations" // warning: xxx is deprecated
#endif

namespace embree
{
  atomic_t errorCounter = 0;
  std::vector<thread_t> g_threads;

  bool hasISA(const int isa) 
  {
    int cpu_features = getCPUFeatures();
    return (cpu_features & isa) == isa;
  }

  /* vertex and triangle layout */
  struct Vertex  {
    Vertex() {}
    Vertex(float x, float y, float z, float a = 0.0f) 
      : x(x), y(y), z(z), a(a) {}
    float x,y,z,a; 
  };
  typedef Vec3f  Vertex3f;
  typedef Vec3fa Vertex3fa;
  
  struct Triangle {
    Triangle () {}
    Triangle(int v0, int v1, int v2) : v0(v0), v1(v1), v2(v2) {}
    int v0, v1, v2; 
  };

  std::vector<void*> buffers;
  MutexSys g_mutex2;
  void* allocBuffer(size_t size) { 
    g_mutex2.lock();
    void* ptr = alignedMalloc(size);
    buffers.push_back(ptr); 
    g_mutex2.unlock();
    return ptr; 
  }
  void clearBuffers() {
    for (size_t i=0; i<buffers.size(); i++) {
      alignedFree(buffers[i]);
    }
    buffers.clear();
  }
  struct ClearBuffers {
    ~ClearBuffers() { clearBuffers(); }
  };

  const size_t numSceneFlags = 64;

  RTCSceneFlags getSceneFlag(size_t i) 
  {
    int flag = 0;                               
    if (i & 1) flag |= RTC_SCENE_DYNAMIC;
    if (i & 2) flag |= RTC_SCENE_COMPACT;
    if (i & 4) flag |= RTC_SCENE_COHERENT;
    if (i & 8) flag |= RTC_SCENE_INCOHERENT;
    if (i & 16) flag |= RTC_SCENE_HIGH_QUALITY;
    if (i & 32) flag |= RTC_SCENE_ROBUST;
    return (RTCSceneFlags) flag;
  }

  const size_t numSceneGeomFlags = 32;

  void getSceneGeomFlag(size_t i, RTCSceneFlags& sflags, RTCGeometryFlags& gflags) 
  {
    int sflag = 0, gflag = 0;
    if (i & 4) {
      sflag |= RTC_SCENE_DYNAMIC;
      gflag = min(i&3,size_t(2));
    }
    if (i & 8) sflag |= RTC_SCENE_HIGH_QUALITY;
    if (i & 16) sflag |= RTC_SCENE_ROBUST;
    sflags = (RTCSceneFlags) sflag;
    gflags = (RTCGeometryFlags) gflag;
  }

  void rtcIntersectN(const RTCSceneRef& scene, RTCRay& ray, int N) 
  {
    switch (N) {
    case 1: {
      rtcIntersect(scene,ray); 
      break;
    }
#if defined(RTCORE_RAY_PACKETS)
    case 4: {
      RTCRay4 ray4; memset(&ray4,0,sizeof(ray4));
      for (size_t i=0; i<4; i++) setRay(ray4,i,ray);
      __aligned(16) int valid[4] = { -1,-1,-1,-1 };
      rtcIntersect4(valid,scene,ray4);
      ray = getRay(ray4,0);
      break;
    }
    case 8: {
      RTCRay8 ray8; memset(&ray8,0,sizeof(ray8));
      for (size_t i=0; i<8; i++) setRay(ray8,i,ray);
      __aligned(32) int valid[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
      rtcIntersect8(valid,scene,ray8);
      ray = getRay(ray8,0);
      break;
    }
    case 16: {
      RTCRay16 ray16; memset(&ray16,0,sizeof(ray16));
      for (size_t i=0; i<16; i++) setRay(ray16,i,ray);
      __aligned(64) int valid[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
      rtcIntersect16(valid,scene,ray16);
      ray = getRay(ray16,0);
      break;
    }
#endif
    default: break;
    }
  }

  void rtcOccludedN(const RTCSceneRef& scene, RTCRay& ray, int N) 
  {
    switch (N) {
    case 1: {
      rtcOccluded(scene,ray); 
      break;
    }
#if defined(RTCORE_RAY_PACKETS)
    case 4: {
      RTCRay4 ray4; memset(&ray4,0,sizeof(ray4));
      for (size_t i=0; i<4; i++) setRay(ray4,i,ray);
      __aligned(16) int valid[4] = { -1,-1,-1,-1 };
      rtcOccluded4(valid,scene,ray4);
      ray.geomID = ray4.geomID[0];
      break;
    }
    case 8: {
      RTCRay8 ray8; memset(&ray8,0,sizeof(ray8));
      for (size_t i=0; i<8; i++) setRay(ray8,i,ray);
      __aligned(32) int valid[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
      rtcOccluded8(valid,scene,ray8);
      ray.geomID = ray8.geomID[0];
      break;
    }
    case 16: {
      RTCRay16 ray16; memset(&ray16,0,sizeof(ray16));
      for (size_t i=0; i<16; i++) setRay(ray16,i,ray);
      __aligned(64) int valid[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
      rtcOccluded16(valid,scene,ray16);
      ray.geomID = ray16.geomID[0];
      break;
    }
#endif
    default: break;
    }
  }

  #define CountErrors(device) \
    if (rtcDeviceGetError(device) != RTC_NO_ERROR) atomic_add(&errorCounter,1);

  void AssertNoError(RTCDevice device) 
  {
    RTCError error = rtcDeviceGetError(device);
    if (error != RTC_NO_ERROR) 
      throw std::runtime_error("Error occured: "+string_of(error));
  }

  void AssertAnyError(RTCDevice device)
  {
    RTCError error = rtcDeviceGetError(device);
    if (error == RTC_NO_ERROR) 
      throw std::runtime_error("Any error expected");
  }

  void AssertError(RTCDevice device, RTCError expectedError)
  {
    RTCError error = rtcDeviceGetError(device);
    if (error != expectedError) 
      throw std::runtime_error("Error "+string_of(expectedError)+" expected");
  }

  RTCAlgorithmFlags aflags = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8 | RTC_INTERSECT16);

  bool g_enable_build_cancel = false;

  unsigned addPlane (RTCDevice g_device, const RTCSceneRef& scene, RTCGeometryFlags flag, size_t num, const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy)
  {
    unsigned mesh = rtcNewTriangleMesh (scene, flag, 2*num*num, (num+1)*(num+1));
    Vertex3fa*   vertices  = (Vertex3fa*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
    Triangle* triangles = (Triangle*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
    for (size_t y=0; y<=num; y++) {
      for (size_t x=0; x<=num; x++) {
        Vec3fa p = p0+float(x)/float(num)*dx+float(y)/float(num)*dy;
        size_t i = y*(num+1)+x;
        vertices[i].x = p.x;
        vertices[i].y = p.y;
        vertices[i].z = p.z;
      }
    }
    for (size_t y=0; y<num; y++) {
      for (size_t x=0; x<num; x++) {
        size_t i = 2*y*num+2*x;
        size_t p00 = (y+0)*(num+1)+(x+0);
        size_t p01 = (y+0)*(num+1)+(x+1);
        size_t p10 = (y+1)*(num+1)+(x+0);
        size_t p11 = (y+1)*(num+1)+(x+1);
        triangles[i+0].v0 = p01; triangles[i+0].v1 = p00; triangles[i+0].v2 = p11;
        triangles[i+1].v0 = p10; triangles[i+1].v1 = p11; triangles[i+1].v2 = p00;
      }
    }
    rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
    rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);
    return mesh;
  }

  unsigned addSubdivPlane (RTCDevice g_device, const RTCSceneRef& scene, RTCGeometryFlags flag, size_t num, const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy)
  {
    unsigned mesh = rtcNewSubdivisionMesh (scene, flag, num*num, 4*num*num, (num+1)*(num+1), 0,0,0);
    Vertex3fa*   vertices  = (Vertex3fa*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
    int* indices = (int*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
    int* faces = (int*) rtcMapBuffer(scene,mesh,RTC_FACE_BUFFER);
    for (size_t y=0; y<=num; y++) {
      for (size_t x=0; x<=num; x++) {
        Vec3fa p = p0+float(x)/float(num)*dx+float(y)/float(num)*dy;
        size_t i = y*(num+1)+x;
        vertices[i].x = p.x;
        vertices[i].y = p.y;
        vertices[i].z = p.z;
      }
    }
    for (size_t y=0; y<num; y++) {
      for (size_t x=0; x<num; x++) {
        size_t i = y*num+x;
        size_t p00 = (y+0)*(num+1)+(x+0);
        size_t p01 = (y+0)*(num+1)+(x+1);
        size_t p10 = (y+1)*(num+1)+(x+0);
        size_t p11 = (y+1)*(num+1)+(x+1);
        indices[4*i+0] = p00; 
        indices[4*i+1] = p01; 
        indices[4*i+2] = p11; 
        indices[4*i+3] = p10; 
        faces[i] = 4;
      }
    }
    rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
    rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);
    rtcUnmapBuffer(scene,mesh,RTC_FACE_BUFFER);
    rtcSetBoundaryMode(scene,mesh,RTC_BOUNDARY_EDGE_AND_CORNER);
    return mesh;
  }

  unsigned addSphere (RTCDevice g_device, const RTCSceneRef& scene, RTCGeometryFlags flag, const Vec3fa& pos, const float r, size_t numPhi, size_t maxTriangles = -1, float motion = 0.0f, BBox3fa* bounds_o = nullptr)
  {
    /* create a triangulated sphere */
    size_t numTheta = 2*numPhi;
    size_t numTriangles = min(maxTriangles,2*numTheta*(numPhi-1));
    size_t numTimeSteps = motion == 0.0f ? 1 : 2;
    size_t numVertices = numTheta*(numPhi+1);
    
    unsigned mesh = rtcNewTriangleMesh (scene, flag, numTriangles, numVertices,numTimeSteps);
    
    /* map triangle and vertex buffer */
    Vertex3f* vertices0 = nullptr;
    Vertex3f* vertices1 = nullptr;
    if (numTimeSteps >= 1) rtcSetBuffer(scene,mesh,RTC_VERTEX_BUFFER0,vertices0 = (Vertex3f*) allocBuffer(numVertices*sizeof(Vertex3f)), 0, sizeof(Vertex3f)); 
    if (numTimeSteps >= 2) rtcSetBuffer(scene,mesh,RTC_VERTEX_BUFFER1,vertices1 = (Vertex3f*) allocBuffer(numVertices*sizeof(Vertex3f)), 0, sizeof(Vertex3f)); 
    Triangle* triangles = (Triangle*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
    if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }

    /* create sphere geometry */
    BBox3fa bounds = empty;
    size_t tri = 0;
    const float rcpNumTheta = 1.0f/float(numTheta);
    const float rcpNumPhi   = 1.0f/float(numPhi);
    for (size_t phi=0; phi<=numPhi; phi++)
    {
      for (size_t theta=0; theta<numTheta; theta++)
      {
        const float phif   = phi*float(pi)*rcpNumPhi;
        const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
        Vertex3f* v = &vertices0[phi*numTheta+theta];
        const float cosThetaf = cos(thetaf);
        v->x = pos.x + r*sin(phif)*sin(thetaf);
        v->y = pos.y + r*cos(phif);
        v->z = pos.z + r*sin(phif)*cosThetaf;
        bounds.extend(Vec3fa(v->x,v->y,v->z));

        if (vertices1) {
          Vertex3f* v1 = &vertices1[phi*numTheta+theta];
          const float cosThetaf = cos(thetaf);
          v1->x = motion + pos.x + r*sin(phif)*sin(thetaf);
          v1->y = motion + pos.y + r*cos(phif);
          v1->z = motion + pos.z + r*sin(phif)*cosThetaf;
          bounds.extend(Vec3fa(v1->x,v1->y,v1->z));
        }
      }
      if (phi == 0) continue;

      for (size_t theta=1; theta<=numTheta; theta++) 
      {
        int p00 = (phi-1)*numTheta+theta-1;
        int p01 = (phi-1)*numTheta+theta%numTheta;
        int p10 = phi*numTheta+theta-1;
        int p11 = phi*numTheta+theta%numTheta;
        
        if (phi > 1) {
          if (tri < numTriangles) {
            triangles[tri].v0 = p10; 
            triangles[tri].v1 = p00; 
            triangles[tri].v2 = p01; 
            tri++;
          }
        }
        
        if (phi < numPhi) {
          if (tri < numTriangles) {
            triangles[tri].v0 = p11; 
            triangles[tri].v1 = p10;
            triangles[tri].v2 = p01; 
            tri++;
          }
        }
      }
    }

    //if (numTimeSteps >= 1) rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER0); 
    //if (numTimeSteps >= 2) rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER1); 
    rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

    if (bounds_o) *bounds_o = bounds;
    return mesh;
  }

  /* adds a subdiv sphere to the scene */
  unsigned int addSubdivSphere (RTCDevice g_device, const RTCSceneRef& scene, RTCGeometryFlags flags, const Vec3fa& pos, const float r, size_t numPhi, float level, size_t maxFaces = -1, float motion = 0.0f)
  {
    size_t numTheta = 2*numPhi;
    avector<Vec3fa> vertices(numTheta*(numPhi+1));
    std::vector<int> indices;
    std::vector<int> faces;
    std::vector<int> offsets;
    
    /* create sphere geometry */
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
      
      if (phi == 1)
      {
	for (int theta=1; theta<=numTheta; theta++) 
	{
	  int p00 = numTheta-1;
	  int p10 = phi*numTheta+theta-1;
	  int p11 = phi*numTheta+theta%numTheta;
	  offsets.push_back(indices.size());
	  indices.push_back(p10); 
	  indices.push_back(p00);
	  indices.push_back(p11);
	  faces.push_back(3);
	}
      }
      else if (phi == numPhi)
      {
	for (int theta=1; theta<=numTheta; theta++) 
	{
	  int p00 = (phi-1)*numTheta+theta-1;
	  int p01 = (phi-1)*numTheta+theta%numTheta;
	  int p10 = numPhi*numTheta;
	  offsets.push_back(indices.size());
	  indices.push_back(p10);
	  indices.push_back(p00);
	  indices.push_back(p01);
	  faces.push_back(3);
	}
      }
      else
      {
	for (int theta=1; theta<=numTheta; theta++) 
	{
	  int p00 = (phi-1)*numTheta+theta-1;
	  int p01 = (phi-1)*numTheta+theta%numTheta;
	  int p10 = phi*numTheta+theta-1;
	  int p11 = phi*numTheta+theta%numTheta;
	  offsets.push_back(indices.size());
	  indices.push_back(p10);
	  indices.push_back(p00);
	  indices.push_back(p01);
	  indices.push_back(p11);
	  faces.push_back(4);
	}
      }
    }
    
    /* create subdiv geometry */
    size_t numFaces = min(faces.size(),maxFaces);
    size_t numEdges = indices.size();
    size_t numVertices = vertices.size();
    size_t numEdgeCreases = 10;
    size_t numVertexCreases = 10;
    size_t numHoles = 0; // do not test holes as this causes some tests that assume a closed sphere to fail
    unsigned int mesh = rtcNewSubdivisionMesh(scene, flags, numFaces, numEdges, numVertices, numEdgeCreases, numVertexCreases, numHoles);
    Vec3fa* vertexBuffer = (Vec3fa*  ) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER);  if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }
    int*    indexBuffer  = (int     *) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);   if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }
    int*    facesBuffer = (int     *) rtcMapBuffer(scene,mesh,RTC_FACE_BUFFER);     if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }
    float*  levelBuffer  = (float   *) rtcMapBuffer(scene,mesh,RTC_LEVEL_BUFFER);   if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }

    memcpy(vertexBuffer,vertices.data(),numVertices*sizeof(Vec3fa));
    memcpy(indexBuffer ,indices.data() ,numEdges*sizeof(int));
    memcpy(facesBuffer,faces.data() ,numFaces*sizeof(int));
    for (size_t i=0; i<indices.size(); i++) levelBuffer[i] = level;
    rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
    rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);
    rtcUnmapBuffer(scene,mesh,RTC_FACE_BUFFER);
    rtcUnmapBuffer(scene,mesh,RTC_LEVEL_BUFFER);
    
    int* edgeCreaseIndices  = (int*) rtcMapBuffer(scene,mesh,RTC_EDGE_CREASE_INDEX_BUFFER);
    if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }
    float* edgeCreaseWeights = (float*) rtcMapBuffer(scene,mesh,RTC_EDGE_CREASE_WEIGHT_BUFFER);
    if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }

    for (size_t i=0; i<numEdgeCreases; i++) 
    {
      if (faces.size()) {
	int f = random<int>() % faces.size();
	int n = faces[f];
	int e = random<int>() % n;
	edgeCreaseIndices[2*i+0] = indices[offsets[f]+(e+0)%n];
	edgeCreaseIndices[2*i+1] = indices[offsets[f]+(e+1)%n];
      } else {
	edgeCreaseIndices[2*i+0] = 0;
	edgeCreaseIndices[2*i+1] = 0;
      }
      edgeCreaseWeights[i] = 10.0f*drand48();
    }
    rtcUnmapBuffer(scene,mesh,RTC_EDGE_CREASE_INDEX_BUFFER); 
    rtcUnmapBuffer(scene,mesh,RTC_EDGE_CREASE_WEIGHT_BUFFER); 
    
    int* vertexCreaseIndices  = (int*) rtcMapBuffer(scene,mesh,RTC_VERTEX_CREASE_INDEX_BUFFER);
    if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }
    float* vertexCreaseWeights = (float*) rtcMapBuffer(scene,mesh,RTC_VERTEX_CREASE_WEIGHT_BUFFER);
    if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,mesh); return -1; }

    for (size_t i=0; i<numVertexCreases; i++) 
    {
      int v = numTheta-1 + random<int>() % (vertices.size()+2-2*numTheta);
      vertexCreaseIndices[i] = v;
      vertexCreaseWeights[i] = 10.0f*drand48();
    }
    rtcUnmapBuffer(scene,mesh,RTC_VERTEX_CREASE_INDEX_BUFFER); 
    rtcUnmapBuffer(scene,mesh,RTC_VERTEX_CREASE_WEIGHT_BUFFER); 
    
    int* holeBuffer  = (int*) rtcMapBuffer(scene,mesh,RTC_HOLE_BUFFER);
    for (size_t i=0; i<numHoles; i++) {
      holeBuffer[i] = random<int>() % faces.size();
    }
    rtcUnmapBuffer(scene,mesh,RTC_HOLE_BUFFER); 
    
    return mesh;
  }

  unsigned addHair (RTCDevice g_device, const RTCSceneRef& scene, RTCGeometryFlags flag, const Vec3fa& pos, const float scale, const float r, size_t numHairs = 1, float motion = 0.0f)
  {
    size_t numTimeSteps = motion == 0.0f ? 1 : 2;
    unsigned geomID = rtcNewHairGeometry (scene, flag, numHairs, numHairs*4, numTimeSteps);
    
    /* map triangle and vertex buffer */
    Vec3fa* vertices0 = nullptr;
    Vec3fa* vertices1 = nullptr;
    if (numTimeSteps >= 1) {
      vertices0 = (Vec3fa*) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER0); 
      if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,geomID); return -1; }
    }
    if (numTimeSteps >= 2) {
      vertices1 = (Vec3fa*) rtcMapBuffer(scene,geomID,RTC_VERTEX_BUFFER1); 
      if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,geomID); return -1; }
    }
    int* indices = (int*) rtcMapBuffer(scene,geomID,RTC_INDEX_BUFFER);
    if (rtcDeviceGetError(g_device) != RTC_NO_ERROR) { rtcDeleteGeometry(scene,geomID); return -1; }

    for (size_t i=0; i<numHairs; i++) 
    {
      indices[i] = 4*i;
      const Vec3fa p0 = pos + scale*Vec3fa(i%7,i%13,i%31);
      const Vec3fa p1 = p0 + scale*Vec3fa(1,0,0);
      const Vec3fa p2 = p0 + scale*Vec3fa(0,1,1);
      const Vec3fa p3 = p0 + scale*Vec3fa(0,1,0);
      
      if (vertices0) {
        vertices0[4*i+0] = Vec3fa(p0,r);
        vertices0[4*i+1] = Vec3fa(p1,r);
        vertices0[4*i+2] = Vec3fa(p2,r);
        vertices0[4*i+3] = Vec3fa(p3,r);
      }
      if (vertices1) {
        vertices1[4*i+0] = Vec3fa(p0+Vec3fa(motion),r);
        vertices1[4*i+1] = Vec3fa(p1+Vec3fa(motion),r);
        vertices1[4*i+2] = Vec3fa(p2+Vec3fa(motion),r);
        vertices1[4*i+3] = Vec3fa(p3+Vec3fa(motion),r);
      }
    }

    if (numTimeSteps >= 1) rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER0); 
    if (numTimeSteps >= 2) rtcUnmapBuffer(scene,geomID,RTC_VERTEX_BUFFER1); 
    rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);
    return geomID;
  }

  unsigned addGarbageTriangles (RTCDevice g_device, const RTCSceneRef& scene, RTCGeometryFlags flag, size_t numTriangles, bool motion)
  {
    /* create a triangulated sphere */
    size_t numTimeSteps = motion ? 2 : 1;
    unsigned mesh = rtcNewTriangleMesh (scene, flag, numTriangles, 3*numTriangles,numTimeSteps);
    
    /* map triangle and vertex buffer */
    if (numTimeSteps >= 1) {
      int* v = (int*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER0); 
      for (size_t i=0; i<4*3*numTriangles; i++) v[i] = random<uint32_t>();
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER0); 
    }
    if (numTimeSteps >= 2) {
      int* v = (int*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER1); 
      for (size_t i=0; i<4*3*numTriangles; i++) v[i] = random<uint32_t>();
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER1); 
    }
    
    Triangle* triangles = (Triangle*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
    for (size_t i=0; i<numTriangles; i++) {
      triangles[i].v0 = (random<int>() % 32 == 0) ? random<uint32_t>() : 3*i+0;
      triangles[i].v1 = (random<int>() % 32 == 0) ? random<uint32_t>() : 3*i+1;
      triangles[i].v2 = (random<int>() % 32 == 0) ? random<uint32_t>() : 3*i+2;
    }
    rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

    return mesh;
  }

  unsigned addGarbageHair (RTCDevice g_device, const RTCSceneRef& scene, RTCGeometryFlags flag, size_t numCurves, bool motion)
  {
    /* create a triangulated sphere */
    size_t numTimeSteps = motion ? 2 : 1;
    unsigned mesh = rtcNewHairGeometry (scene, flag, numCurves, 4*numCurves,numTimeSteps);
    
    /* map triangle and vertex buffer */
    if (numTimeSteps >= 1) {
      int* v = (int*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER0); 
      for (size_t i=0; i<4*4*numCurves; i++) v[i] = random<uint32_t>();
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER0); 
    }
    if (numTimeSteps >= 2) {
      int* v = (int*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER1); 
      for (size_t i=0; i<4*4*numCurves; i++) v[i] = random<uint32_t>();
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER1); 
    }
    
    int* curves = (int*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
    for (size_t i=0; i<numCurves; i++) 
      curves[i] = (random<int>() % 32 == 0) ? random<uint32_t>() : 4*i;
    rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

    return mesh;
  }

  struct Sphere
  {
    ALIGNED_CLASS;
  public:
    Sphere () : pos(zero), r(zero) {}
    Sphere (const Vec3fa& pos, float r) : pos(pos), r(r) {}
    __forceinline BBox3fa bounds() const { return BBox3fa(pos-Vec3fa(r),pos+Vec3fa(r)); }
  public:
    Vec3fa pos;
    float r;
  };

  void BoundsFunc(Sphere* sphere, size_t index, BBox3fa* bounds_o)
  {
    bounds_o->lower.x = sphere->pos.x-sphere->r;
    bounds_o->lower.y = sphere->pos.y-sphere->r;
    bounds_o->lower.z = sphere->pos.z-sphere->r;
    bounds_o->upper.x = sphere->pos.x+sphere->r;
    bounds_o->upper.y = sphere->pos.y+sphere->r;
    bounds_o->upper.z = sphere->pos.z+sphere->r;
  }

  void IntersectFunc(void* ptr, RTCRay& ray, size_t item) {
  }

  void IntersectFunc4(const void* valid, void* ptr, RTCRay4& ray, size_t item) {
  }

  void IntersectFunc8(const void* valid, void* ptr, RTCRay8& ray, size_t item) {
  }

  void IntersectFunc16(const void* valid, void* ptr, RTCRay16& ray, size_t item) {
  }

  void OccludedFunc (void* ptr, RTCRay& ray, size_t item) {
  }

  void OccludedFunc4 (const void* valid, void* ptr, RTCRay4& ray, size_t item) {
  }

  void OccludedFunc8 (const void* valid, void* ptr, RTCRay8& ray, size_t item) {
  }

  void OccludedFunc16 (const void* valid, void* ptr, RTCRay16& ray, size_t item) {
  }

  unsigned addUserGeometryEmpty (RTCDevice g_device, IntersectMode imode, const RTCSceneRef& scene, Sphere* sphere)
  {
    BBox3fa bounds = sphere->bounds(); 
    unsigned geom = rtcNewUserGeometry (scene,1);
    rtcSetBoundsFunction(scene,geom,(RTCBoundsFunc)BoundsFunc);
    rtcSetUserData(scene,geom,sphere);
    if (imode != MODE_INTERSECT_NONE)
    {
      rtcSetIntersectFunction(scene,geom,IntersectFunc);
#if defined(RTCORE_RAY_PACKETS)
      rtcSetIntersectFunction4(scene,geom,IntersectFunc4);
      rtcSetIntersectFunction8(scene,geom,IntersectFunc8);
      rtcSetIntersectFunction16(scene,geom,&IntersectFunc16);
#endif
      rtcSetOccludedFunction(scene,geom,OccludedFunc);
#if defined(RTCORE_RAY_PACKETS)
      rtcSetOccludedFunction4(scene,geom,OccludedFunc4);
      rtcSetOccludedFunction8(scene,geom,OccludedFunc8);
      rtcSetOccludedFunction16(scene,geom,&OccludedFunc16);
#endif
    }
    return geom;
  }

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////


  struct MultipleDevicesTest : public VerifyApplication::Test
  {
    MultipleDevicesTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}

    bool run(VerifyApplication* state)
    {
      /* test creation of multiple devices */
      RTCDevice device1 = rtcNewDevice("threads=4");
      AssertNoError(device1);
      RTCDevice device2 = rtcNewDevice("threads=8");
      AssertNoError(device2);
      RTCDevice device3 = rtcNewDevice("threads=12");
      AssertNoError(device3);
      rtcDeleteDevice(device1);
      rtcDeleteDevice(device3);
      rtcDeleteDevice(device2);
      return true;
    }
  };

  struct EmptySceneTest : public VerifyApplication::Test
  {
    EmptySceneTest (std::string name, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,VerifyApplication::PASS), sflags(sflags) {}

    bool run(VerifyApplication* state)
    {
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,aflags);
      AssertNoError(state->device);
      rtcCommit (scene);
      AssertNoError(state->device);
      return true;
    }

  public:
    RTCSceneFlags sflags;
  };

  struct BaryDistanceTest : public VerifyApplication::Test
  {
    BaryDistanceTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}

    bool run(VerifyApplication* state)
    {
      std::vector<Vertex> m_vertices;
      std::vector<Triangle> m_triangles;
      
      const float length = 1000.0f;
      const float width = 1000.0f;
      
      m_vertices.resize(4);
      m_vertices[0] = Vertex(-length / 2.0f, -width / 2.0f, 0);
      m_vertices[1] = Vertex( length / 2.0f, -width / 2.0f, 0);
      m_vertices[2] = Vertex( length / 2.0f,  width / 2.0f, 0);
      m_vertices[3] = Vertex(-length / 2.0f,  width / 2.0f, 0);
      
      m_triangles.resize(2);
      m_triangles[0] = Triangle(0, 1, 2);
      m_triangles[1] = Triangle(2, 3, 0);
      
      //const RTCSceneFlags flags = RTCSceneFlags(0); 
      const RTCSceneFlags flags = RTC_SCENE_ROBUST;
      const RTCSceneRef mainSceneId = rtcDeviceNewScene(state->device,RTC_SCENE_STATIC | flags , RTC_INTERSECT1);
      
      const unsigned int id = rtcNewTriangleMesh(mainSceneId, RTC_GEOMETRY_STATIC, m_triangles.size(), m_vertices.size());
      
      rtcSetBuffer(mainSceneId, id, RTC_VERTEX_BUFFER, m_vertices.data(), 0, sizeof(Vertex));
      rtcSetBuffer(mainSceneId, id, RTC_INDEX_BUFFER, m_triangles.data(), 0, sizeof(Triangle));
      
      rtcCommit(mainSceneId);
      
      RTCRay ray;
      ray.org[0] = 0.1f;
      ray.org[1] = 1.09482f;
      ray.org[2] = 29.8984f;
      ray.dir[0] = 0.f;
      ray.dir[1] = 0.99482f;
      ray.dir[2] = -0.101655f;
      ray.tnear = 0.05f;
      ray.tfar  = inf;
      ray.mask  = -1;
      
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      ray.primID = RTC_INVALID_GEOMETRY_ID;
      ray.instID = RTC_INVALID_GEOMETRY_ID;
      
      rtcIntersect(mainSceneId, ray);
      
      if (ray.geomID == RTC_INVALID_GEOMETRY_ID) 
        throw std::runtime_error("no triangle hit");
      
      const Triangle &triangle = m_triangles[ray.primID];
      
      const Vertex &v0_ = m_vertices[triangle.v0];
      const Vertex &v1_ = m_vertices[triangle.v1];
      const Vertex &v2_ = m_vertices[triangle.v2];
      
      const Vec3fa v0(v0_.x, v0_.y, v0_.z);
      const Vec3fa v1(v1_.x, v1_.y, v1_.z);
      const Vec3fa v2(v2_.x, v2_.y, v2_.z);
      
      const Vec3fa hit_tri = v0 + ray.u * (v1 - v0) + ray.v * (v2 - v0);
      
      const Vec3fa ray_org = Vec3fa(ray.org[0], ray.org[1], ray.org[2]);
      const Vec3fa ray_dir = Vec3fa(ray.dir[0], ray.dir[1], ray.dir[2]);
      
      const Vec3fa hit_tfar = ray_org + ray.tfar * ray_dir;
      const Vec3fa delta = hit_tri - hit_tfar;
      const float distance = embree::length(delta);
      
      return distance < 0.0002f;
    }
  };

  struct FlagsTest : public VerifyApplication::Test
  {
    FlagsTest (std::string name, VerifyApplication::TestType type, RTCSceneFlags sceneFlags, RTCGeometryFlags geomFlags)
      : VerifyApplication::Test(name,type), sceneFlags(sceneFlags), geomFlags(geomFlags) {}

    bool run(VerifyApplication* state)
    {
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sceneFlags,aflags);
      AssertNoError(state->device);
      rtcNewTriangleMesh (scene, geomFlags, 0, 0);
      AssertNoError(state->device);
      rtcNewHairGeometry (scene, geomFlags, 0, 0);
      AssertNoError(state->device);
      rtcCommit (scene);
      AssertNoError(state->device);
      scene = nullptr;
      return true;
    }

  public:
    RTCSceneFlags sceneFlags;
    RTCGeometryFlags geomFlags;
  };
  
  struct StaticSceneTest : public VerifyApplication::Test
  {
    StaticSceneTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}

    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_STATIC,aflags);
      AssertNoError(state->device);
      unsigned geom0 = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,zero,1.0f,50);
      AssertNoError(state->device);
      unsigned geom1 = addSubdivSphere(state->device,scene,RTC_GEOMETRY_STATIC,zero,1.0f,10,16);
      AssertNoError(state->device);
      unsigned geom2 = addHair(state->device,scene,RTC_GEOMETRY_STATIC,Vec3fa(0,0,0),1.0f,0.5f,100);
      AssertNoError(state->device);
      rtcCommit (scene);
      AssertNoError(state->device);
      rtcDisable(scene,geom0); // static scene cannot get modified anymore after commit
      AssertAnyError(state->device);
      scene = nullptr;
      return true;
    }
  };

  struct UnmappedBeforeCommitTest : public VerifyApplication::Test
  {
    UnmappedBeforeCommitTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}

    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_STATIC,aflags);
      AssertNoError(state->device);
      unsigned geom0 = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,zero,1.0f,50);
      unsigned geom1 = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,zero,1.0f,50);
      AssertNoError(state->device);
      rtcMapBuffer(scene,geom0,RTC_INDEX_BUFFER);
      rtcMapBuffer(scene,geom0,RTC_VERTEX_BUFFER);
      AssertNoError(state->device);
      rtcCommit (scene);
      AssertError(state->device,RTC_INVALID_OPERATION); // error, buffers still mapped
      scene = nullptr;
      return true;
    }
  };

  struct GetBoundsTest : public VerifyApplication::Test
  {
    GetBoundsTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}

    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(state->device);
      BBox3fa bounds0;
      unsigned geom0 = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,zero,1.0f,50,-1,0,&bounds0);
      AssertNoError(state->device);
      rtcCommit (scene);
      AssertNoError(state->device);
      BBox3fa bounds1;
      rtcGetBounds(scene,(RTCBounds&)bounds1);
      scene = nullptr;
      return bounds0 == bounds1;
    }
  };

  struct BufferStrideTest : public VerifyApplication::Test
  {
    BufferStrideTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}
    
    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_STATIC,aflags);
      AssertNoError(state->device);
      unsigned geom = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 16, 16);
      AssertNoError(state->device);
      avector<char> indexBuffer(8+16*6*sizeof(int));
      avector<char> vertexBuffer(12+16*9*sizeof(float)+4);
      
      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),1,3*sizeof(int));
      AssertError(state->device,RTC_INVALID_OPERATION);
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),1,3*sizeof(float));
      AssertError(state->device,RTC_INVALID_OPERATION);

      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int)+3);
      AssertError(state->device,RTC_INVALID_OPERATION);
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,3*sizeof(float)+3);
      AssertError(state->device,RTC_INVALID_OPERATION);
      
      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int));
      AssertNoError(state->device);
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,3*sizeof(float));
      AssertNoError(state->device);
      
      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),8,6*sizeof(int));
      AssertNoError(state->device);
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),12,9*sizeof(float));
      AssertNoError(state->device);
      
      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int));
      AssertNoError(state->device);
      
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,4*sizeof(float));
      AssertNoError(state->device);
      
      scene = nullptr;
      return true;
    }
  };
  
  struct DynamicEnableDisableTest : public VerifyApplication::Test
  {
    DynamicEnableDisableTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}
    
    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_DYNAMIC,aflags);
      AssertNoError(state->device);
      unsigned geom0 = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,Vec3fa(-1,0,-1),1.0f,50);
      //unsigned geom1 = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,Vec3fa(-1,0,+1),1.0f,50);
      unsigned geom1 = addHair  (state->device,scene,RTC_GEOMETRY_STATIC,Vec3fa(-1,0,+1),1.0f,1.0f,1);
      unsigned geom2 = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,Vec3fa(+1,0,-1),1.0f,50);
      //unsigned geom3 = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,Vec3fa(+1,0,+1),1.0f,50);
      unsigned geom3 = addHair  (state->device,scene,RTC_GEOMETRY_STATIC,Vec3fa(+1,0,+1),1.0f,1.0f,1);
      AssertNoError(state->device);
      
      for (size_t i=0; i<16; i++) 
      {
        bool enabled0 = i & 1, enabled1 = i & 2, enabled2 = i & 4, enabled3 = i & 8;
        if (enabled0) rtcEnable(scene,geom0); else rtcDisable(scene,geom0); AssertNoError(state->device);
        if (enabled1) rtcEnable(scene,geom1); else rtcDisable(scene,geom1); AssertNoError(state->device);
        if (enabled2) rtcEnable(scene,geom2); else rtcDisable(scene,geom2); AssertNoError(state->device);
        if (enabled3) rtcEnable(scene,geom3); else rtcDisable(scene,geom3); AssertNoError(state->device);
        rtcCommit (scene);
        AssertNoError(state->device);
        {
          RTCRay ray0 = makeRay(Vec3fa(-1,10,-1),Vec3fa(0,-1,0));
          RTCRay ray1 = makeRay(Vec3fa(-1,10,+1),Vec3fa(0,-1,0)); 
          RTCRay ray2 = makeRay(Vec3fa(+1,10,-1),Vec3fa(0,-1,0)); 
          RTCRay ray3 = makeRay(Vec3fa(+1,10,+1),Vec3fa(0,-1,0)); 
          rtcIntersect(scene,ray0);
          rtcIntersect(scene,ray1);
          rtcIntersect(scene,ray2);
          rtcIntersect(scene,ray3);
          bool ok0 = enabled0 ? ray0.geomID == 0 : ray0.geomID == -1;
          bool ok1 = enabled1 ? ray1.geomID == 1 : ray1.geomID == -1;
          bool ok2 = enabled2 ? ray2.geomID == 2 : ray2.geomID == -1;
          bool ok3 = enabled3 ? ray3.geomID == 3 : ray3.geomID == -1;
          if (!ok0 || !ok1 || !ok2 || !ok3) return false;
        }
      }
      scene = nullptr;
      return true;
    }
  };

  struct GetUserDataTest : public VerifyApplication::Test
  {
    GetUserDataTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}
    
    bool run(VerifyApplication* state)
    {
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(state->device);
      unsigned geom0 = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      AssertNoError(state->device);
      rtcSetUserData(scene,geom0,(void*)1);
      
      unsigned geom1 = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, 0, 0, 0, 0, 0, 0, 1);
      AssertNoError(state->device);
      rtcSetUserData(scene,geom1,(void*)2);
      
      unsigned geom2 = rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      AssertNoError(state->device);
      rtcSetUserData(scene,geom2,(void*)3);
      
      unsigned geom3 = rtcNewUserGeometry (scene,0);
      AssertNoError(state->device);
      rtcSetUserData(scene,geom3,(void*)4);
      
      rtcCommit (scene);
      AssertNoError(state->device);
      
      if ((size_t)rtcGetUserData(scene,geom0) != 1) return false;
      if ((size_t)rtcGetUserData(scene,geom1) != 2) return false;
      if ((size_t)rtcGetUserData(scene,geom2) != 3) return false;
      if ((size_t)rtcGetUserData(scene,geom3) != 4) return false;
      
      scene = nullptr;
      AssertNoError(state->device);
      return true;
    }
  };

  void move_mesh_vec3f(const RTCSceneRef& scene, unsigned mesh, size_t numVertices, Vec3fa& pos) 
  {
    Vertex3f* vertices = (Vertex3f*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
    for (size_t i=0; i<numVertices; i++) vertices[i] += Vertex3f(pos);
    rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
    rtcUpdate(scene,mesh);
  }

  void move_mesh_vec3fa(const RTCSceneRef& scene, unsigned mesh, size_t numVertices, Vec3fa& pos) 
  {
    Vertex3fa* vertices = (Vertex3fa*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
    for (size_t i=0; i<numVertices; i++) vertices[i] += Vertex3fa(pos);
    rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
    rtcUpdate(scene,mesh);
  }
  
  struct UpdateTest : public VerifyApplication::Test
  {
    UpdateTest (std::string name, RTCGeometryFlags flags)
      : VerifyApplication::Test(name,VerifyApplication::PASS), flags(flags) {}
    
    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_DYNAMIC,aflags);
      AssertNoError(state->device);
      size_t numPhi = 50;
      size_t numVertices = 2*numPhi*(numPhi+1);
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      unsigned geom0 = addSphere(state->device,scene,flags,pos0,1.0f,numPhi);
      unsigned geom1 = addHair  (state->device,scene,flags,pos1,1.0f,1.0f,1);
      unsigned geom2 = addSphere(state->device,scene,flags,pos2,1.0f,numPhi);
      unsigned geom3 = addHair  (state->device,scene,flags,pos3,1.0f,1.0f,1);
      AssertNoError(state->device);
      
      for (size_t i=0; i<16; i++) 
      {
        bool move0 = i & 1, move1 = i & 2, move2 = i & 4, move3 = i & 8;
        Vec3fa ds(2,0.1f,2);
        if (move0) { move_mesh_vec3f (scene,geom0,numVertices,ds); pos0 += ds; }
        if (move1) { move_mesh_vec3fa(scene,geom1,4,ds); pos1 += ds; }
        if (move2) { move_mesh_vec3f (scene,geom2,numVertices,ds); pos2 += ds; }
        if (move3) { move_mesh_vec3fa(scene,geom3,4,ds); pos3 += ds; }
        rtcCommit (scene);
        AssertNoError(state->device);
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); 
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); 
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); 
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); 
          rtcIntersect(scene,ray0);
          rtcIntersect(scene,ray1);
          rtcIntersect(scene,ray2);
          rtcIntersect(scene,ray3);
          if (ray0.geomID != 0 || 
              ray1.geomID != 1 || 
              ray2.geomID != 2 || 
              ray3.geomID != 3) return false;

#if defined(RTCORE_RAY_PACKETS)
          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT4))
          {
            RTCRay4 ray4; memset(&ray4,0,sizeof(ray4));
            setRay(ray4,0,ray0);
            setRay(ray4,1,ray1);
            setRay(ray4,2,ray2);
            setRay(ray4,3,ray3);
            __aligned(16) int valid4[4] = { -1,-1,-1,-1 };
            rtcIntersect4(valid4,scene,ray4);
            if (ray4.geomID[0] != 0 || 
                ray4.geomID[1] != 1 || 
                ray4.geomID[2] != 2 || 
                ray4.geomID[3] != 3) return false;
          }
          
          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT8))
          {
            RTCRay8 ray8; memset(&ray8,0,sizeof(ray8));
            setRay(ray8,0,ray0);
            setRay(ray8,1,ray1);
            setRay(ray8,2,ray2);
            setRay(ray8,3,ray3);
            __aligned(32) int valid8[8] = { -1,-1,-1,-1, 0, 0, 0, 0 };
            rtcIntersect8(valid8,scene,ray8);
            if (ray8.geomID[0] != 0 || 
                ray8.geomID[1] != 1 || 
                ray8.geomID[2] != 2 || 
                ray8.geomID[3] != 3) return false;
          }
          
          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT16))
          {
            RTCRay16 ray16; memset(&ray16,0,sizeof(ray16));
            setRay(ray16,0,ray0);
            setRay(ray16,1,ray1);
            setRay(ray16,2,ray2);
            setRay(ray16,3,ray3);
            __aligned(64) int valid16[16] = { -1,-1,-1,-1,+0,+0,+0,+0, 
                                              +0,+0,+0,+0,+0,+0,+0,+0 };
            rtcIntersect16(valid16,scene,ray16);
            if (ray16.geomID[0] != 0 || 
                ray16.geomID[1] != 1 || 
                ray16.geomID[2] != 2 || 
                ray16.geomID[3] != 3) return false;
          }
#endif
        }
      }
      scene = nullptr;
      return true;
    }
    
  public:
    RTCGeometryFlags flags;
  };
  
  struct RayMasksTest : public VerifyApplication::Test
  {
    RayMasksTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}
    
    bool rtcore_ray_masks_intersect(VerifyApplication* state, RTCSceneFlags sflags, RTCGeometryFlags gflags)
    {
      ClearBuffers clear_before_return;
      bool passed = true;
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,aflags);
      unsigned geom0 = addSphere(state->device,scene,gflags,pos0,1.0f,50);
      //unsigned geom1 = addSphere(state->device,scene,gflags,pos1,1.0f,50);
      unsigned geom1 = addHair  (state->device,scene,gflags,pos1,1.0f,1.0f,1);
      unsigned geom2 = addSphere(state->device,scene,gflags,pos2,1.0f,50);
      //unsigned geom3 = addSphere(state->device,scene,gflags,pos3,1.0f,50);
      unsigned geom3 = addHair  (state->device,scene,gflags,pos3,1.0f,1.0f,1);
      rtcSetMask(scene,geom0,1);
      rtcSetMask(scene,geom1,2);
      rtcSetMask(scene,geom2,4);
      rtcSetMask(scene,geom3,8);
      rtcCommit (scene);
      
      for (size_t i=0; i<16; i++) 
      {
        int mask0 = i;
        int mask1 = i+1;
        int mask2 = i+2;
        int mask3 = i+3;
        
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
          rtcIntersect(scene,ray0);
          rtcIntersect(scene,ray1);
          rtcIntersect(scene,ray2);
          rtcIntersect(scene,ray3);
          bool ok0 = mask0 & 1 ? ray0.geomID == 0 : ray0.geomID == -1;
          bool ok1 = mask1 & 2 ? ray1.geomID == 1 : ray1.geomID == -1;
          bool ok2 = mask2 & 4 ? ray2.geomID == 2 : ray2.geomID == -1;
          bool ok3 = mask3 & 8 ? ray3.geomID == 3 : ray3.geomID == -1;
          if (!ok0 || !ok1 || !ok2 || !ok3) passed = false;
        }
        
#if defined(RTCORE_RAY_PACKETS)
        if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT4))
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
          
          RTCRay4 ray4; memset(&ray4,0,sizeof(ray4));
          setRay(ray4,0,ray0);
          setRay(ray4,1,ray1);
          setRay(ray4,2,ray2);
          setRay(ray4,3,ray3);
          __aligned(16) int valid4[4] = { -1,-1,-1,-1 };
          rtcIntersect4(valid4,scene,ray4);
          bool ok4a = mask0 & 1 ? ray4.geomID[0] == 0 : ray4.geomID[0] == -1;
          bool ok4b = mask1 & 2 ? ray4.geomID[1] == 1 : ray4.geomID[1] == -1;
          bool ok4c = mask2 & 4 ? ray4.geomID[2] == 2 : ray4.geomID[2] == -1;
          bool ok4d = mask3 & 8 ? ray4.geomID[3] == 3 : ray4.geomID[3] == -1;
          if (!ok4a || !ok4b || !ok4c || !ok4d) passed = false; 
        }
        
        if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT8))
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
          
          RTCRay8 ray8; memset(&ray8,0,sizeof(ray8));
          setRay(ray8,0,ray0);
          setRay(ray8,1,ray1);
          setRay(ray8,2,ray2);
          setRay(ray8,3,ray3);
          __aligned(32) int valid8[8] = { -1,-1,-1,-1,0,0,0,0 };
          rtcIntersect8(valid8,scene,ray8);
          bool ok8a = mask0 & 1 ? ray8.geomID[0] == 0 : ray8.geomID[0] == -1;
          bool ok8b = mask1 & 2 ? ray8.geomID[1] == 1 : ray8.geomID[1] == -1;
          bool ok8c = mask2 & 4 ? ray8.geomID[2] == 2 : ray8.geomID[2] == -1;
          bool ok8d = mask3 & 8 ? ray8.geomID[3] == 3 : ray8.geomID[3] == -1;
          if (!ok8a || !ok8b || !ok8c || !ok8d) passed = false; 
        }
        
        if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT16))
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
          
          RTCRay16 ray16; memset(&ray16,0,sizeof(ray16));
          setRay(ray16,0,ray0);
          setRay(ray16,1,ray1);
          setRay(ray16,2,ray2);
          setRay(ray16,3,ray3);
          __aligned(64) int valid16[16] = { -1,-1,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0 };
          rtcIntersect16(valid16,scene,ray16);
          bool ok16a = mask0 & 1 ? ray16.geomID[0] == 0 : ray16.geomID[0] == -1;
          bool ok16b = mask1 & 2 ? ray16.geomID[1] == 1 : ray16.geomID[1] == -1;
          bool ok16c = mask2 & 4 ? ray16.geomID[2] == 2 : ray16.geomID[2] == -1;
          bool ok16d = mask3 & 8 ? ray16.geomID[3] == 3 : ray16.geomID[3] == -1;
          if (!ok16a || !ok16b || !ok16c || !ok16d) passed = false;
        }
#endif
      }
      scene = nullptr;
      return passed;
    }
    
    bool rtcore_ray_masks_occluded(VerifyApplication* state, RTCSceneFlags sflags, RTCGeometryFlags gflags)
    {
      ClearBuffers clear_before_return;
      bool passed = true;
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,aflags);
      unsigned geom0 = addSphere(state->device,scene,gflags,pos0,1.0f,50);
      unsigned geom1 = addSphere(state->device,scene,gflags,pos1,1.0f,50);
      unsigned geom2 = addSphere(state->device,scene,gflags,pos2,1.0f,50);
      unsigned geom3 = addSphere(state->device,scene,gflags,pos3,1.0f,50);
      rtcSetMask(scene,geom0,1);
      rtcSetMask(scene,geom1,2);
      rtcSetMask(scene,geom2,4);
      rtcSetMask(scene,geom3,8);
      rtcCommit (scene);
      
      for (size_t i=0; i<16; i++) 
      {
        int mask0 = i;
        int mask1 = i+1;
        int mask2 = i+2;
        int mask3 = i+3;
        
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
          rtcOccluded(scene,ray0);
          rtcOccluded(scene,ray1);
          rtcOccluded(scene,ray2);
          rtcOccluded(scene,ray3);
          bool ok0 = mask0 & 1 ? ray0.geomID == 0 : ray0.geomID == -1;
          bool ok1 = mask1 & 2 ? ray1.geomID == 0 : ray1.geomID == -1;
          bool ok2 = mask2 & 4 ? ray2.geomID == 0 : ray2.geomID == -1;
          bool ok3 = mask3 & 8 ? ray3.geomID == 0 : ray3.geomID == -1;
          
          if (!ok0 || !ok1 || !ok2 || !ok3) passed = false;
        }
        
#if defined(RTCORE_RAY_PACKETS)
        if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT4))
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
          
          RTCRay4 ray4; memset(&ray4,0,sizeof(ray4));
          setRay(ray4,0,ray0);
          setRay(ray4,1,ray1);
          setRay(ray4,2,ray2);
          setRay(ray4,3,ray3);
          __aligned(16) int valid4[4] = { -1,-1,-1,-1 };
          rtcOccluded4(valid4,scene,ray4);
          bool ok4a = mask0 & 1 ? ray4.geomID[0] == 0 : ray4.geomID[0] == -1;
          bool ok4b = mask1 & 2 ? ray4.geomID[1] == 0 : ray4.geomID[1] == -1;
          bool ok4c = mask2 & 4 ? ray4.geomID[2] == 0 : ray4.geomID[2] == -1;
          bool ok4d = mask3 & 8 ? ray4.geomID[3] == 0 : ray4.geomID[3] == -1;
          if (!ok4a || !ok4b || !ok4c || !ok4d) passed = false;
        }
        
        if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT8))
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
          
          RTCRay8 ray8; memset(&ray8,0,sizeof(ray8));
          setRay(ray8,0,ray0);
          setRay(ray8,1,ray1);
          setRay(ray8,2,ray2);
          setRay(ray8,3,ray3);
          __aligned(32) int valid8[8] = { -1,-1,-1,-1,0,0,0,0 };
          rtcOccluded8(valid8,scene,ray8);
          bool ok8a = mask0 & 1 ? ray8.geomID[0] == 0 : ray8.geomID[0] == -1;
          bool ok8b = mask1 & 2 ? ray8.geomID[1] == 0 : ray8.geomID[1] == -1;
          bool ok8c = mask2 & 4 ? ray8.geomID[2] == 0 : ray8.geomID[2] == -1;
          bool ok8d = mask3 & 8 ? ray8.geomID[3] == 0 : ray8.geomID[3] == -1;
          if (!ok8a || !ok8b || !ok8c || !ok8d) passed = false;
        }
        
        if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT16))
        {
          RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
          RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
          RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
          RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
          
          RTCRay16 ray16; memset(&ray16,0,sizeof(ray16));
          setRay(ray16,0,ray0);
          setRay(ray16,1,ray1);
          setRay(ray16,2,ray2);
          setRay(ray16,3,ray3);
          __aligned(64) int valid16[16] = { -1,-1,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0 };
          
          rtcOccluded16(valid16,scene,ray16);
          
          bool ok16a = mask0 & 1 ? ray16.geomID[0] == 0 : ray16.geomID[0] == -1;
          bool ok16b = mask1 & 2 ? ray16.geomID[1] == 0 : ray16.geomID[1] == -1;
          bool ok16c = mask2 & 4 ? ray16.geomID[2] == 0 : ray16.geomID[2] == -1;
          bool ok16d = mask3 & 8 ? ray16.geomID[3] == 0 : ray16.geomID[3] == -1;
          if (!ok16a || !ok16b || !ok16c || !ok16d) passed = false;
        }
#endif
      }
      scene = nullptr;
      return passed;
    }
    
    bool run(VerifyApplication* state)
    {
      bool passed = true;
      for (int i=0; i<numSceneFlags; i++) 
      {
        RTCSceneFlags flag = getSceneFlag(i);
        bool ok0 = rtcore_ray_masks_intersect(state,flag,RTC_GEOMETRY_STATIC);
        if (ok0) printf(GREEN("+")); else printf(RED("-"));
        passed &= ok0;
        bool ok1 = rtcore_ray_masks_occluded(state,flag,RTC_GEOMETRY_STATIC);
        if (ok1) printf(GREEN("+")); else printf(RED("-"));
        passed &= ok1;
        fflush(stdout);
      }
      return passed;
    }
  };

  struct BuildTest : public VerifyApplication::Test
  {
    BuildTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}
    
    bool rtcore_build(VerifyApplication* state, RTCSceneFlags sflags, RTCGeometryFlags gflags)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,aflags);
      addSphere(state->device,scene,gflags,zero,1E-24f,50);
      addHair(state->device,scene,gflags,zero,1E-24f,1E-26f,100,1E-26f);
      addSphere(state->device,scene,gflags,zero,1E-24f,50);
      addHair(state->device,scene,gflags,zero,1E-24f,1E-26f,100,1E-26f);
      rtcCommit (scene);
      scene = nullptr;
      return true;
    }
  
    bool run(VerifyApplication* state)
    {
      bool passed = true;
      for (int i=0; i<numSceneGeomFlags; i++) 
      {
        RTCSceneFlags sflags; RTCGeometryFlags gflags;
        getSceneGeomFlag(i,sflags,gflags);
        bool ok = rtcore_build(state,sflags,gflags);
        if (ok) printf(GREEN("+")); else printf(RED("-"));
        passed &= ok;
        fflush(stdout);
      }
      return passed;
    }
  };

  struct IntersectionFilterTest : public VerifyApplication::Test
  {
    bool subdiv;

    IntersectionFilterTest (std::string name, bool subdiv)
      : VerifyApplication::Test(name,VerifyApplication::PASS), subdiv(subdiv) {}
    
    static void intersectionFilter1(void* ptr, RTCRay& ray) 
    {
      if ((size_t)ptr != 123) 
        return;
      
      if (ray.primID & 2)
        ray.geomID = -1;
    }
    
    static void intersectionFilter4(const void* valid_i, void* ptr, RTCRay4& ray) 
    {
      if ((size_t)ptr != 123) 
        return;
      
      int* valid = (int*)valid_i;
      for (size_t i=0; i<4; i++)
        if (valid[i] == -1)
          if (ray.primID[i] & 2) 
            ray.geomID[i] = -1;
    }
    
    static void intersectionFilter8(const void* valid_i, void* ptr, RTCRay8& ray) 
    {
      if ((size_t)ptr != 123) 
        return;
      
      int* valid = (int*)valid_i;
      for (size_t i=0; i<8; i++)
        if (valid[i] == -1)
          if (ray.primID[i] & 2) 
            ray.geomID[i] = -1;
    }
    
    static void intersectionFilter16(const void* valid_i, void* ptr, RTCRay16& ray) 
    {
      if ((size_t)ptr != 123) 
        return;
      
      unsigned int valid = *(unsigned int*)valid_i;
      for (size_t i=0; i<16; i++)
	if (valid & ((unsigned int)1 << i))
	  if (ray.primID[i] & 2) 
	    ray.geomID[i] = -1;
    }
    
    bool rtcore_filter_intersect(VerifyApplication* state, RTCSceneFlags sflags, RTCGeometryFlags gflags, bool subdiv)
    {
      ClearBuffers clear_before_return;
      bool passed = true;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,aflags);
      Vec3fa p0(-0.75f,-0.25f,-10.0f), dx(4,0,0), dy(0,4,0);
      int geom0 = 0;
      if (subdiv) geom0 = addSubdivPlane (state->device,scene, gflags, 4, p0, dx, dy);
      else        geom0 = addPlane (state->device,scene, gflags, 4, p0, dx, dy);
      rtcSetUserData(scene,geom0,(void*)123);
      rtcSetIntersectionFilterFunction(scene,geom0,intersectionFilter1);
#if defined(RTCORE_RAY_PACKETS)
      rtcSetIntersectionFilterFunction4(scene,geom0,intersectionFilter4);
      rtcSetIntersectionFilterFunction8(scene,geom0,intersectionFilter8);
      rtcSetIntersectionFilterFunction16(scene,geom0,intersectionFilter16);
#endif
      rtcCommit (scene);
      
      for (size_t iy=0; iy<4; iy++) 
      {
        for (size_t ix=0; ix<4; ix++) 
        {
          int primID = iy*4+ix;
          if (!subdiv) primID *= 2;
          {
            RTCRay ray0 = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
            rtcIntersect(scene,ray0);
            bool ok0 = (primID & 2) ? (ray0.geomID == -1) : (ray0.geomID == 0);
            if (!ok0) passed = false;
          }
          
#if defined(RTCORE_RAY_PACKETS)
          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT4))
          {
            RTCRay ray0 = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
            
            RTCRay4 ray4; memset(&ray4,0,sizeof(ray4));
            setRay(ray4,0,ray0);
            __aligned(16) int valid4[4] = { -1,0,0,0 };
            rtcIntersect4(valid4,scene,ray4);
            bool ok0 = (primID & 2) ? (ray4.geomID[0] == -1) : (ray4.geomID[0] == 0);
            if (!ok0) passed = false;
          }
          
          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT8))
          {
            RTCRay ray0 = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
            
            RTCRay8 ray8; memset(&ray8,0,sizeof(ray8));
            setRay(ray8,0,ray0);
            __aligned(32) int valid8[8] = { -1,0,0,0,0,0,0,0 };
            rtcIntersect8(valid8,scene,ray8);
            bool ok0 = (primID & 2) ? (ray8.geomID[0] == -1) : (ray8.geomID[0] == 0);
            if (!ok0) passed = false;
          }
          
          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT16))
          {
            RTCRay ray0 = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
            
            RTCRay16 ray16; memset(&ray16,0,sizeof(ray16));
            setRay(ray16,0,ray0);
            __aligned(64) int valid16[16] = { -1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
            rtcIntersect16(valid16,scene,ray16);
            bool ok0 = (primID & 2) ? (ray16.geomID[0] == -1) : (ray16.geomID[0] == 0);
            if (!ok0) passed = false;
          }
#endif
        }
      }
      scene = nullptr;
      return passed;
    }
    
    bool rtcore_filter_occluded(VerifyApplication* state, RTCSceneFlags sflags, RTCGeometryFlags gflags, bool subdiv)
    {
      ClearBuffers clear_before_return;
      bool passed = true;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,aflags);
      Vec3fa p0(-0.75f,-0.25f,-10.0f), dx(4,0,0), dy(0,4,0);
      int geom0 = 0;
      if (subdiv) geom0 = addSubdivPlane (state->device, scene, gflags, 4, p0, dx, dy);
      else        geom0 = addPlane (state->device, scene, gflags, 4, p0, dx, dy);
      rtcSetUserData(scene,geom0,(void*)123);
      rtcSetOcclusionFilterFunction(scene,geom0,intersectionFilter1);
#if defined(RTCORE_RAY_PACKETS)
      rtcSetOcclusionFilterFunction4(scene,geom0,intersectionFilter4);
      rtcSetOcclusionFilterFunction8(scene,geom0,intersectionFilter8);
      rtcSetOcclusionFilterFunction16(scene,geom0,intersectionFilter16);
#endif
      rtcCommit (scene);
      
      for (size_t iy=0; iy<4; iy++) 
      {
        for (size_t ix=0; ix<4; ix++) 
        {
          int primID = iy*4+ix;
          if (!subdiv) primID *= 2;
          
          {
            RTCRay ray0 = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
            rtcOccluded(scene,ray0);
            bool ok0 = (primID & 2) ? (ray0.geomID == -1) : (ray0.geomID == 0);
            if (!ok0) passed = false;
          }
          
          if (subdiv) continue; // FIXME: subdiv filter callbacks only working for single ray queries
          
#if defined(RTCORE_RAY_PACKETS)
          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT4))
          {
            RTCRay ray0 = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
            
            RTCRay4 ray4; memset(&ray4,0,sizeof(ray4));
            setRay(ray4,0,ray0);
            __aligned(16) int valid4[4] = { -1,0,0,0 };
            rtcOccluded4(valid4,scene,ray4);
            bool ok0 = (primID & 2) ? (ray4.geomID[0] == -1) : (ray4.geomID[0] == 0);
            if (!ok0) passed = false;
          }
          
          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT8))
          {
            RTCRay ray0 = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
            
            RTCRay8 ray8; memset(&ray8,0,sizeof(ray8));
            setRay(ray8,0,ray0);
            __aligned(32) int valid8[8] = { -1,0,0,0,0,0,0,0 };
            rtcOccluded8(valid8,scene,ray8);
            bool ok0 = (primID & 2) ? (ray8.geomID[0] == -1) : (ray8.geomID[0] == 0);
            if (!ok0) passed = false;
          }

          if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT16))
          {
            RTCRay ray0 = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
            
            RTCRay16 ray16; memset(&ray16,0,sizeof(ray16));
            setRay(ray16,0,ray0);
            __aligned(64) int valid16[16] = { -1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
            rtcOccluded16(valid16,scene,ray16);
            bool ok0 = (primID & 2) ? (ray16.geomID[0] == -1) : (ray16.geomID[0] == 0);
            if (!ok0) passed = false;
          }
#endif
        }
      }
      scene = nullptr;
      return passed;
    }
    
    bool run(VerifyApplication* state)
    {
      bool passed = true;
      for (int i=0; i<numSceneFlags; i++) 
      {
        RTCSceneFlags flag = getSceneFlag(i);
        bool ok0 = rtcore_filter_intersect(state,flag,RTC_GEOMETRY_STATIC, subdiv);
        if (ok0) printf(GREEN("+")); else printf(RED("-"));
        passed &= ok0;
        bool ok1 = rtcore_filter_occluded(state,flag,RTC_GEOMETRY_STATIC, subdiv);
        if (ok1) printf(GREEN("+")); else printf(RED("-"));
        passed &= ok1;
        fflush(stdout);
      }
      return passed;
    }
  };

  struct InactiveRaysTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    IntersectMode imode;
    static const size_t N = 10;
    static const size_t maxStreamSize = 100;
    
    InactiveRaysTest (std::string name, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode)
      : VerifyApplication::Test(name,VerifyApplication::PASS), sflags(sflags), gflags(gflags), imode(imode) {}
   
     bool run(VerifyApplication* state)
    {
      Vec3fa pos = zero;
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,to_aflags(imode));
      addSphere(state->device,scene,RTC_GEOMETRY_STATIC,pos,2.0f,50); // FIXME: use different geometries too
      rtcCommit (scene);
      AssertNoError(state->device);

      RTCRay invalid_ray;
      memset(&invalid_ray,-1,sizeof(RTCRay));
      invalid_ray.tnear = pos_inf;
      invalid_ray.tfar  = neg_inf;
      invalid_ray = invalid_ray;
      
      size_t numFailures = 0;
      for (size_t i=0; i<N; i++) 
      {
        IntersectVariant ivariant = (IntersectVariant) (i%2);
        for (size_t M=1; M<maxStreamSize; M++)
        {
          bool valid[maxStreamSize];
          __aligned(16) RTCRay rays[maxStreamSize];
          for (size_t j=0; j<M; j++) 
          {
            if (rand()%2) {
              valid[j] = true;
              Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
              Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
              rays[j] = makeRay(pos+org,dir); 
            } else {
              valid[j] = false;
              rays[j] = invalid_ray;
            }
          }
          IntersectWithMode(imode,ivariant,scene,rays,M);
          for (size_t j=0; j<M; j++) {
            if (valid[j]) continue;
            numFailures += neq_ray_special(rays[j],invalid_ray);
          }
        }
      }
      AssertNoError(state->device);
      scene = nullptr;
      AssertNoError(state->device);
      fflush(stdout);
      return numFailures == 0;
    }
  };

  struct WatertightTest : public VerifyApplication::Test
  {
    ALIGNED_STRUCT;
    RTCSceneFlags sflags;
    IntersectMode imode;
    std::string model;
    Vec3fa pos;
    static const size_t N = 10;
    static const size_t maxStreamSize = 100;
    
    WatertightTest (std::string name, RTCSceneFlags sflags, IntersectMode imode, std::string model, const Vec3fa& pos)
      : VerifyApplication::Test(name,VerifyApplication::PASS), sflags(sflags), imode(imode), model(model), pos(pos) {}
    
    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,to_aflags(imode));
      if      (model == "sphere") addSphere(state->device,scene,RTC_GEOMETRY_STATIC,pos,2.0f,500);
      else if (model == "plane" ) addPlane(state->device,scene,RTC_GEOMETRY_STATIC,500,Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,12.0f,0.0f),Vec3fa(0.0f,0.0f,12.0f));
      bool plane = model == "plane";
      rtcCommit (scene);
      AssertNoError(state->device);
      
      size_t numTests = 0;
      size_t numFailures = 0;
      for (size_t i=0; i<N; i++) 
      {
        IntersectVariant ivariant = (IntersectVariant) (i%2);
        for (size_t M=1; M<maxStreamSize; M++)
        {
          __aligned(16) RTCRay rays[maxStreamSize];
          for (size_t j=0; j<M; j++) 
          {
            if (plane) {
              Vec3fa org(drand48()-0.5f,drand48()-0.5f,drand48()-0.5f);
              Vec3fa dir(1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
              rays[j] = makeRay(Vec3fa(pos.x-3.0f,0.0f,0.0f),dir); 
            } else {
              Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
              Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
              rays[j] = makeRay(pos+org,dir); 
            }
          }
          IntersectWithMode(imode,ivariant,scene,rays,M);
          for (size_t j=0; j<M; j++) {
            numTests++;
            numFailures += rays[j].geomID == -1;
          }
        }
      }
      AssertNoError(state->device);
      scene = nullptr;
      AssertNoError(state->device);
      double failRate = double(numFailures) / double(numTests);
      bool failed = failRate > 0.00002;
      
      printf(" (%f%%)", 100.0f*failRate);
      fflush(stdout);
      return !failed;
    }
  };

  struct NaNTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    IntersectMode imode;
    
    NaNTest (std::string name, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode)
      : VerifyApplication::Test(name,VerifyApplication::PASS), sflags(sflags), gflags(gflags), imode(imode) {}
    
    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      const size_t numRays = 1000;
      RTCRay rays[numRays];
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,to_aflags(imode));
      addSphere(state->device,scene,gflags,zero,2.0f,100);
      addHair  (state->device,scene,gflags,zero,1.0f,1.0f,100);
      rtcCommit (scene);
      size_t numFailures = 0;

      double c0 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);
    
      double c1 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org+Vec3fa(nan),dir); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);

      double c2 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org+Vec3fa(nan),dir+Vec3fa(nan)); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);

      double c3 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir,nan,nan); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);

      double c4 = getSeconds();
      double d1 = c1-c0;
      double d2 = c2-c1;
      double d3 = c3-c2;
      double d4 = c4-c3;
      scene = nullptr;
      
      bool ok = (d2 < 2.5*d1) && (d3 < 2.5*d1) && (d4 < 2.5*d1);
      float f = max(d2/d1,d3/d1,d4/d1);
      printf(" (%3.2fx)",f);
      fflush(stdout);
      return ok;
    }
  };
    
  struct InfTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    IntersectMode imode;
    
    InfTest (std::string name, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode)
      : VerifyApplication::Test(name,VerifyApplication::PASS), sflags(sflags), gflags(gflags), imode(imode) {}
   
    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      size_t numRays = 1000;
      RTCRay rays[numRays];
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,to_aflags(imode));
      addSphere(state->device,scene,gflags,zero,2.0f,100);
      addHair  (state->device,scene,gflags,zero,1.0f,1.0f,100);
      rtcCommit (scene);
      AssertNoError(state->device);

      size_t numFailures = 0;
      double c0 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);

      double c1 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org+Vec3fa(inf),dir); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);

      double c2 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir+Vec3fa(inf)); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);

      double c3 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org+Vec3fa(inf),dir+Vec3fa(inf)); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);

      double c4 = getSeconds();
      /*for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir,-0.0f,inf); 
      }
      IntersectWithMode(imode, VARIANT_OCCLUDED ,scene,rays,numRays);
      IntersectWithMode(imode, VARIANT_INTERSECT,scene,rays,numRays);*/ // FIXME: enable this test again

      double c5 = getSeconds();      
      double d1 = c1-c0;
      double d2 = c2-c1;
      double d3 = c3-c2;
      double d4 = c4-c3;
      //double d5 = c5-c4;
      scene = nullptr;
      AssertNoError(state->device);
      
      bool ok = (d2 < 2.5*d1) && (d3 < 2.5*d1) && (d4 < 2.5*d1);// && (d5 < 2.5*d1);
      float f = max(d2/d1,d3/d1,d4/d1);//,d5/d1);
      printf(" (%3.2fx)",f);
      fflush(stdout);
      return ok;
    }
  };

  struct OverlappingTrianglesTest : public VerifyApplication::Test
  {
    int N;
    
    OverlappingTrianglesTest (std::string name, int N)
      : VerifyApplication::Test(name,VerifyApplication::PASS), N(N) {}
    
    bool run(VerifyApplication* state)
    {
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_STATIC,aflags);
      AssertNoError(state->device);
      rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, N, 3);
      AssertNoError(state->device);
      
      Vertex3fa* vertices = (Vertex3fa*) rtcMapBuffer(scene,0,RTC_VERTEX_BUFFER);
      vertices[0].x = 0.0f; vertices[0].y = 0.0f; vertices[0].z = 0.0f;
      vertices[1].x = 1.0f; vertices[1].y = 0.0f; vertices[1].z = 0.0f;
      vertices[2].x = 0.0f; vertices[2].y = 1.0f; vertices[2].z = 0.0f;
      rtcUnmapBuffer(scene,0,RTC_VERTEX_BUFFER);
      AssertNoError(state->device);
      
      Triangle* triangles = (Triangle*) rtcMapBuffer(scene,0,RTC_INDEX_BUFFER);
      for (size_t i=0; i<N; i++) {
        triangles[i].v0 = 0;
        triangles[i].v1 = 1;
        triangles[i].v2 = 2;
      }
      rtcUnmapBuffer(scene,0,RTC_INDEX_BUFFER);
      AssertNoError(state->device);
      
      rtcCommit (scene);
      AssertNoError(state->device);
      
      return true;
    }
  };
    
  struct OverlappingHairTest : public VerifyApplication::Test
  {
    int N;
    
    OverlappingHairTest (std::string name, int N)
      : VerifyApplication::Test(name,VerifyApplication::PASS), N(N) {}
    
    bool run(VerifyApplication* state)
    {
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_STATIC,aflags);
      AssertNoError(state->device);
      rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, N, 4);
      AssertNoError(state->device);
      
      Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene,0,RTC_VERTEX_BUFFER);
      vertices[0].x = 0.0f; vertices[0].y = 0.0f; vertices[0].z = 0.0f; vertices[0].w = 0.1f;
      vertices[1].x = 0.0f; vertices[1].y = 0.0f; vertices[1].z = 1.0f; vertices[1].w = 0.1f;
      vertices[2].x = 0.0f; vertices[2].y = 1.0f; vertices[2].z = 1.0f; vertices[2].w = 0.1f;
      vertices[3].x = 0.0f; vertices[3].y = 1.0f; vertices[3].z = 0.0f; vertices[3].w = 0.1f;
      rtcUnmapBuffer(scene,0,RTC_VERTEX_BUFFER);
      AssertNoError(state->device);
      
      int* indices = (int*) rtcMapBuffer(scene,0,RTC_INDEX_BUFFER);
      for (size_t i=0; i<N; i++) {
        indices[i] = 0;
      }
      rtcUnmapBuffer(scene,0,RTC_INDEX_BUFFER);
      AssertNoError(state->device);
      
      rtcCommit (scene);
      AssertNoError(state->device);
      
      return true;
    }
  };

  struct BackfaceCullingTest : public VerifyApplication::Test
  {
    BackfaceCullingTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}
    
    bool rtcore_backface_culling (VerifyApplication* state, RTCSceneFlags sflags, RTCGeometryFlags gflags)
    {
      /* create triangle that is front facing for a right handed 
         coordinate system if looking along the z direction */
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,aflags);
      unsigned mesh = rtcNewTriangleMesh (scene, gflags, 1, 3);
      Vertex3fa*   vertices  = (Vertex3fa*  ) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
      Triangle* triangles = (Triangle*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
      vertices[0].x = 0; vertices[0].y = 0; vertices[0].z = 0;
      vertices[1].x = 0; vertices[1].y = 1; vertices[1].z = 0;
      vertices[2].x = 1; vertices[2].y = 0; vertices[2].z = 0;
      triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
      rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);
      rtcCommit (scene);
      
      bool passed = true;
      RTCRay ray;
      RTCRay backfacing = makeRay(Vec3fa(0.25f,0.25f,1),Vec3fa(0,0,-1)); 
      RTCRay frontfacing = makeRay(Vec3fa(0.25f,0.25f,-1),Vec3fa(0,0,1)); 
      
      ray = frontfacing; rtcOccludedN(scene,ray,1);  if (ray.geomID != 0) passed = false;
      ray = frontfacing; rtcIntersectN(scene,ray,1); if (ray.geomID != 0) passed = false;
      ray = backfacing;  rtcOccludedN(scene,ray,1);  if (ray.geomID != -1) passed = false;
      ray = backfacing;  rtcIntersectN(scene,ray,1); if (ray.geomID != -1) passed = false;

#if defined(RTCORE_RAY_PACKETS)
      if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT4))
      {
        ray = frontfacing; rtcOccludedN(scene,ray,4);  if (ray.geomID != 0) passed = false;
        ray = frontfacing; rtcIntersectN(scene,ray,4); if (ray.geomID != 0) passed = false;
        ray = backfacing;  rtcOccludedN(scene,ray,4);  if (ray.geomID != -1) passed = false;
        ray = backfacing;  rtcIntersectN(scene,ray,4); if (ray.geomID != -1) passed = false;
      }
      if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT8))
      {
        ray = frontfacing; rtcOccludedN(scene,ray,8);  if (ray.geomID != 0) passed = false;
        ray = frontfacing; rtcIntersectN(scene,ray,8); if (ray.geomID != 0) passed = false;
        ray = backfacing;  rtcOccludedN(scene,ray,8);  if (ray.geomID != -1) passed = false;
        ray = backfacing;  rtcIntersectN(scene,ray,8); if (ray.geomID != -1) passed = false;
      }
      if (rtcDeviceGetParameter1i(state->device,RTC_CONFIG_INTERSECT16))
      {
        ray = frontfacing; rtcOccludedN(scene,ray,16); if (ray.geomID != 0) passed = false;
        ray = frontfacing; rtcIntersectN(scene,ray,16);if (ray.geomID != 0) passed = false;
        ray = backfacing;  rtcOccludedN(scene,ray,16); if (ray.geomID != -1) passed = false;
        ray = backfacing;  rtcIntersectN(scene,ray,16);if (ray.geomID != -1) passed = false;
      }
#endif
      return passed;
    }
    
    bool run(VerifyApplication* state)
    {
      bool passed = true;
      for (int i=0; i<numSceneFlags; i++) 
      {
        RTCSceneFlags flag = getSceneFlag(i);
        bool ok0 = rtcore_backface_culling(state,flag,RTC_GEOMETRY_STATIC);
        if (ok0) printf(GREEN("+")); else printf(RED("-"));
        passed &= ok0;
        fflush(stdout);
      }
      return passed;
    }
  };

  struct NewDeleteGeometryTest : public VerifyApplication::Test
  {
    NewDeleteGeometryTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}
    
    bool run(VerifyApplication* state)
    {
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_DYNAMIC,aflags);
      AssertNoError(state->device);
      int geom[128];
      for (size_t i=0; i<128; i++) geom[i] = -1;
      Sphere spheres[128];
      memset(spheres,0,sizeof(spheres));
      
      for (size_t i=0; i<50; i++) {
        for (size_t j=0; j<10; j++) {
          int index = random<int>()%128;
          Vec3fa pos = 100.0f*Vec3fa(drand48(),drand48(),drand48());
          if (geom[index] == -1) {
            switch (random<int>()%4) {
            case 0: geom[index] = addSphere(state->device,scene,RTC_GEOMETRY_STATIC,pos,2.0f,10); break;
            case 1: geom[index] = addHair  (state->device,scene,RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,10); break;
            case 2: geom[index] = addSubdivSphere(state->device,scene,RTC_GEOMETRY_STATIC,pos,2.0f,4,4); break;
            case 3: 
              spheres[index] = Sphere(pos,2.0f);
              geom[index] = addUserGeometryEmpty(state->device,MODE_INTERSECT_NONE,scene,&spheres[index]); break;
            }
            AssertNoError(state->device);
          }
          else { 
            rtcDeleteGeometry(scene,geom[index]);     
            AssertNoError(state->device);
            geom[index] = -1; 
          }
        }
        rtcCommit(scene);
        AssertNoError(state->device);
        rtcCommit(scene);
        AssertNoError(state->device);
        if (i%2 == 0) std::cout << "." << std::flush;
      }
      rtcCommit (scene);
      AssertNoError(state->device);
      scene = nullptr;
      return true;
    }
  };

   /*********************************************************************************/
  /*                        rtcInterpolate                                         */
  /*********************************************************************************/

  const size_t num_interpolation_vertices = 16;
  const size_t num_interpolation_quad_faces = 9;
  const size_t num_interpolation_triangle_faces = 18;

  __aligned(16) float interpolation_vertices[num_interpolation_vertices*3] = {
    -1.0f, -1.0f, 0.0f,
    0.0f, -1.0f, 0.0f, 
    +1.0f, -1.0f, 0.0f,
    +2.0f, -1.0f, 0.0f,
    
    -1.0f,  0.0f, 0.0f,
    0.0f,  0.0f, 0.0f,
    +1.0f,  0.0f, 0.0f,
    +2.0f,  0.0f, 0.0f,

    -1.0f, +1.0f, 0.0f,
    0.0f, +1.0f, 0.0f,
    +1.0f, +1.0f, 0.0f,
    +2.0f, +1.0f, 0.0f,

    -1.0f, +2.0f, 0.0f,
    0.0f, +2.0f, 0.0f,
    +1.0f, +2.0f, 0.0f,
    +2.0f, +2.0f, 0.0f,
  };

  __aligned(16) int interpolation_quad_indices[num_interpolation_quad_faces*4] = {
    0, 1, 5, 4,
    1, 2, 6, 5,
    2, 3, 7, 6,
    4, 5, 9, 8, 
    5, 6, 10, 9,
    6, 7, 11, 10,
    8, 9, 13, 12,
    9, 10, 14, 13,
    10, 11, 15, 14
  };

  __aligned(16) int interpolation_triangle_indices[num_interpolation_triangle_faces*3] = {
    0, 1, 5,  0, 5, 4,
    1, 2, 6,  1, 6, 5,
    2, 3, 7,  2, 7, 6,
    4, 5, 9,  4, 9, 8, 
    5, 6, 10,  5, 10, 9,
    6, 7, 11,  6, 11, 10,
    8, 9, 13,   8, 13, 12,
    9, 10, 14,   9, 14, 13,
    10, 11, 15,  10, 15, 14
  };

  __aligned(16) int interpolation_quad_faces[num_interpolation_quad_faces] = {
    4, 4, 4, 4, 4, 4, 4, 4, 4
  };

  __aligned(16) float interpolation_vertex_crease_weights[2] = {
    inf, inf
  };

  __aligned(16) unsigned int interpolation_vertex_crease_indices[2] = {
    12, 15
  };

  __aligned(16) float interpolation_edge_crease_weights[3] = {
    inf, inf, inf
  };

  __aligned(16) unsigned int interpolation_edge_crease_indices[6] = 
  {
    8,9, 9,10, 10,11
  };

  struct InterpolateSubdivTest : public VerifyApplication::Test
  {
    size_t N;
    
    InterpolateSubdivTest (std::string name, size_t N)
      : VerifyApplication::Test(name,VerifyApplication::PASS), N(N) {}

    bool checkInterpolation2D(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        float p0 = (1.0f/6.0f)*(1.0f*data[(v0-4-1)*N_total+i] + 4.0f*data[(v0-4+0)*N_total+i] + 1.0f*data[(v0-4+1)*N_total+i]);
        float p1 = (1.0f/6.0f)*(1.0f*data[(v0+0-1)*N_total+i] + 4.0f*data[(v0+0+0)*N_total+i] + 1.0f*data[(v0+0+1)*N_total+i]);
        float p2 = (1.0f/6.0f)*(1.0f*data[(v0+4-1)*N_total+i] + 4.0f*data[(v0+4+0)*N_total+i] + 1.0f*data[(v0+4+1)*N_total+i]);
        float p = (1.0f/6.0f)*(1.0f*p0+4.0f*p1+1.0f*p2);
        passed &= fabsf(p-P[i]) < 1E-4f;
      }
      return passed;
    }
    
    bool checkInterpolation1D(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, int v1, int v2, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        float v = (1.0f/6.0f)*(1.0f*data[v0*N_total+i] + 4.0f*data[v1*N_total+i] + 1.0f*data[v2*N_total+i]);
        passed &= fabsf(v-P[i]) < 0.001f;
      }
      return passed;
    }
    
    bool checkInterpolationSharpVertex(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        float v = data[v0*N_total+i];
        passed &= fabs(v-P[i]) < 1E-3f;
      }
      return passed;
    }
    
    bool checkSubdivInterpolation(VerifyApplication* state, const RTCSceneRef& scene, int geomID, RTCBufferType buffer, float* vertices0, size_t N, size_t N_total)
    {
      rtcSetBoundaryMode(scene,geomID,RTC_BOUNDARY_EDGE_ONLY);
      AssertNoError(state->device);
      rtcDisable(scene,geomID);
      AssertNoError(state->device);
      rtcCommit(scene);
      AssertNoError(state->device);
      bool passed = true;
      passed &= checkInterpolation1D(scene,geomID,0,0.0f,0.0f,4,0,1,buffer,vertices0,N,N_total);
      passed &= checkInterpolation1D(scene,geomID,2,1.0f,0.0f,2,3,7,buffer,vertices0,N,N_total);
      
      passed &= checkInterpolation2D(scene,geomID,3,1.0f,0.0f,5,buffer,vertices0,N,N_total);
      passed &= checkInterpolation2D(scene,geomID,1,1.0f,1.0f,6,buffer,vertices0,N,N_total);
      
      //passed &= checkInterpolation1D(scene,geomID,3,1.0f,1.0f,8,9,10,buffer,vertices0,N,N_total);
      //passed &= checkInterpolation1D(scene,geomID,7,1.0f,0.0f,9,10,11,buffer,vertices0,N,N_total);
      
      passed &= checkInterpolationSharpVertex(scene,geomID,6,0.0f,1.0f,12,buffer,vertices0,N,N_total);
      passed &= checkInterpolationSharpVertex(scene,geomID,8,1.0f,1.0f,15,buffer,vertices0,N,N_total);
      
      rtcSetBoundaryMode(scene,geomID,RTC_BOUNDARY_EDGE_AND_CORNER);
      AssertNoError(state->device);
      rtcCommit(scene);
      AssertNoError(state->device);
      
      passed &= checkInterpolationSharpVertex(scene,geomID,0,0.0f,0.0f,0,buffer,vertices0,N,N_total);
      passed &= checkInterpolationSharpVertex(scene,geomID,2,1.0f,0.0f,3,buffer,vertices0,N,N_total);
      return passed;
    }
    
    bool run(VerifyApplication* state)
    {
      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(state->device);
      unsigned int geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, num_interpolation_quad_faces, num_interpolation_quad_faces*4, num_interpolation_vertices, 3, 2, 0, 1);
      AssertNoError(state->device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_quad_indices , 0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_FACE_BUFFER,   interpolation_quad_faces,    0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,   interpolation_edge_crease_indices,  0, 2*sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,  interpolation_edge_crease_weights,  0, sizeof(float));
      rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER, interpolation_vertex_crease_indices,0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER,interpolation_vertex_crease_weights,0, sizeof(float));
      AssertNoError(state->device);
      
      float* vertices0 = new float[M];
      for (size_t i=0; i<M; i++) vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      /*float* vertices1 = new float[M];
        for (size_t i=0; i<M; i++) vertices1[i] = drand48();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1, 0, N*sizeof(float));
        AssertNoError(state->device);*/
      
      float* user_vertices0 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      float* user_vertices1 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices1[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      bool passed = true;
      passed &= checkSubdivInterpolation(state,scene,geomID,RTC_VERTEX_BUFFER0,vertices0,N,N);
      //passed &= checkSubdivInterpolation(state,scene,geomID,RTC_VERTEX_BUFFER1,vertices1,N,N);
      passed &= checkSubdivInterpolation(state,scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0,N,N);
      passed &= checkSubdivInterpolation(state,scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1,N,N);
      
      passed &= checkSubdivInterpolation(state,scene,geomID,RTC_VERTEX_BUFFER0,vertices0,1,N);
      //passed &= checkSubdivInterpolation(state,scene,geomID,RTC_VERTEX_BUFFER1,vertices1,1,N);
      passed &= checkSubdivInterpolation(state,scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0,1,N);
      passed &= checkSubdivInterpolation(state,scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1,1,N);
      
      delete[] vertices0;
      //delete[] vertices1;
      delete[] user_vertices0;
      delete[] user_vertices1;
      
      return passed;
    }
  };

  struct InterpolateTrianglesTest : public VerifyApplication::Test
  {
    size_t N;
    
    InterpolateTrianglesTest (std::string name, size_t N)
      : VerifyApplication::Test(name,VerifyApplication::PASS), N(N) {}
    
    bool checkTriangleInterpolation(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, int v1, int v2, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        float p0 = data[v0*N_total+i];
        float p1 = data[v1*N_total+i];
        float p2 = data[v2*N_total+i];
        float p = (1.0f-u-v)*p0 + u*p1 + v*p2;
        passed &= fabs(p-P[i]) < 1E-4f;
      }
      return passed;
    }
    
    bool checkTriangleInterpolation(const RTCSceneRef& scene, int geomID, RTCBufferType buffer, float* vertices0, size_t N, size_t N_total)
    {
      bool passed = true;
      passed &= checkTriangleInterpolation(scene,geomID,0,0.0f,0.0f,0,1,5,buffer,vertices0,N,N_total);
      passed &= checkTriangleInterpolation(scene,geomID,0,0.5f,0.5f,0,1,5,buffer,vertices0,N,N_total);
      passed &= checkTriangleInterpolation(scene,geomID,17,0.0f,0.0f,10,15,14,buffer,vertices0,N,N_total);
      passed &= checkTriangleInterpolation(scene,geomID,17,0.5f,0.5f,10,15,14,buffer,vertices0,N,N_total);
      return passed;
    }

    bool run(VerifyApplication* state)
    {
      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(state->device);
      unsigned int geomID = rtcNewTriangleMesh(scene, RTC_GEOMETRY_STATIC, num_interpolation_triangle_faces, num_interpolation_vertices, 1);
      AssertNoError(state->device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_triangle_indices , 0, 3*sizeof(unsigned int));
      AssertNoError(state->device);
      
      float* vertices0 = new float[M];
      for (size_t i=0; i<M; i++) vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      /*float* vertices1 = new float[M];
        for (size_t i=0; i<M; i++) vertices1[i] = drand48();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1, 0, N*sizeof(float));
        AssertNoError(state->device);*/
      
      float* user_vertices0 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      float* user_vertices1 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices1[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      rtcDisable(scene,geomID);
      AssertNoError(state->device);
      rtcCommit(scene);
      AssertNoError(state->device);
      
      bool passed = true;
      passed &= checkTriangleInterpolation(scene,geomID,RTC_VERTEX_BUFFER0,vertices0,N,N);
      //passed &= checkTriangleInterpolation(scene,geomID,RTC_VERTEX_BUFFER1,vertices1,N,N);
      passed &= checkTriangleInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0,N,N);
      passed &= checkTriangleInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1,N,N);
      
      passed &= checkTriangleInterpolation(scene,geomID,RTC_VERTEX_BUFFER0,vertices0,1,N);
      //passed &= checkTriangleInterpolation(scene,geomID,RTC_VERTEX_BUFFER1,vertices1,1,N);
      passed &= checkTriangleInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0,1,N);
      passed &= checkTriangleInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1,1,N);
      
      delete[] vertices0;
      //delete[] vertices1;
      delete[] user_vertices0;
      delete[] user_vertices1;
      
      return passed;
    }
  };
  
  const size_t num_interpolation_hair_vertices = 13;
  const size_t num_interpolation_hairs = 4;

  __aligned(16) int interpolation_hair_indices[num_interpolation_hairs] = {
    0, 3, 6, 9
  };

  struct InterpolateHairTest : public VerifyApplication::Test
  {
    size_t N;
    
    InterpolateHairTest (std::string name, size_t N)
      : VerifyApplication::Test(name,VerifyApplication::PASS), N(N) {}
    
    bool checkHairInterpolation(const RTCSceneRef& scene, int geomID, int primID, float u, float v, int v0, RTCBufferType buffer, float* data, size_t N, size_t N_total)
    {
      bool passed = true;
      float P[256], dPdu[256], dPdv[256];
      rtcInterpolate(scene,geomID,primID,u,v,buffer,P,dPdu,dPdv,N);
      
      for (size_t i=0; i<N; i++) {
        const float p00 = data[(v0+0)*N_total+i];
        const float p01 = data[(v0+1)*N_total+i];
        const float p02 = data[(v0+2)*N_total+i];
        const float p03 = data[(v0+3)*N_total+i];
        const float t0 = 1.0f - u, t1 = u;
        const float p10 = p00 * t0 + p01 * t1;
        const float p11 = p01 * t0 + p02 * t1;
        const float p12 = p02 * t0 + p03 * t1;
        const float p20 = p10 * t0 + p11 * t1;
        const float p21 = p11 * t0 + p12 * t1;
        const float p30 = p20 * t0 + p21 * t1;
        passed &= fabs(p30-P[i]) < 1E-4f;
      }
      return passed;
    }
    
    bool checkHairInterpolation(const RTCSceneRef& scene, int geomID, RTCBufferType buffer, float* vertices0, size_t N, size_t N_total)
    {
      bool passed = true;
      passed &= checkHairInterpolation(scene,geomID,0,0.0f,0.0f,0,buffer,vertices0,N,N_total);
      passed &= checkHairInterpolation(scene,geomID,1,0.5f,0.0f,3,buffer,vertices0,N,N_total);
      passed &= checkHairInterpolation(scene,geomID,2,0.0f,0.0f,6,buffer,vertices0,N,N_total);
      passed &= checkHairInterpolation(scene,geomID,3,0.2f,0.0f,9,buffer,vertices0,N,N_total);
      return passed;
    }
    
    bool run(VerifyApplication* state)
    {
      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(state->device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(state->device);
      unsigned int geomID = rtcNewHairGeometry(scene, RTC_GEOMETRY_STATIC, num_interpolation_hairs, num_interpolation_hair_vertices, 1);
      AssertNoError(state->device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_hair_indices , 0, sizeof(unsigned int));
      AssertNoError(state->device);
      
      float* vertices0 = new float[M];
      for (size_t i=0; i<M; i++) vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      /*float* vertices1 = new float[M];
        for (size_t i=0; i<M; i++) vertices1[i] = drand48();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1, 0, N*sizeof(float));
        AssertNoError(state->device);*/
      
      float* user_vertices0 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      float* user_vertices1 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices1[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1, 0, N*sizeof(float));
      AssertNoError(state->device);
      
      rtcDisable(scene,geomID);
      AssertNoError(state->device);
      rtcCommit(scene);
      AssertNoError(state->device);
      
      bool passed = true;
      passed &= checkHairInterpolation(scene,geomID,RTC_VERTEX_BUFFER0,vertices0,N,N);
      //passed &= checkHairInterpolation(scene,geomID,RTC_VERTEX_BUFFER1,vertices1,N,N);
      passed &= checkHairInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0,N,N);
      passed &= checkHairInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1,N,N);
      
      passed &= checkHairInterpolation(scene,geomID,RTC_VERTEX_BUFFER0,vertices0,1,N);
      //passed &= checkHairInterpolation(scene,geomID,RTC_VERTEX_BUFFER1,vertices1,1,N);
      passed &= checkHairInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0,1,N);
      passed &= checkHairInterpolation(scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1,1,N);
      
      delete[] vertices0;
      //delete[] vertices1;
      delete[] user_vertices0;
      delete[] user_vertices1;
      
      return passed;
    }
  };

  void shootRays (RTCDevice device, const RTCSceneRef& scene)
  {
    {
      Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
      Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
      RTCRay ray = makeRay(org,dir); 
      rtcOccluded(scene,ray);
      rtcIntersect(scene,ray);
    }

#if defined(RTCORE_RAY_PACKETS)
    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT4))
    {
      RTCRay4 ray4; memset(&ray4,0,sizeof(ray4)); 
      for (size_t j=0; j<4; j++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        RTCRay ray = makeRay(org,dir); 
        setRay(ray4,j,ray);
      }
      __aligned(16) int valid4[4] = { -1,-1,-1,-1 };
      rtcOccluded4(valid4,scene,ray4);
      rtcIntersect4(valid4,scene,ray4);
    }

    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT8))
    {
      RTCRay8 ray8; memset(&ray8,0,sizeof(ray8));
      for (size_t j=0; j<8; j++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        RTCRay ray = makeRay(org,dir); 
        setRay(ray8,j,ray);
      }
      __aligned(32) int valid8[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
      rtcOccluded8(valid8,scene,ray8);
      rtcIntersect8(valid8,scene,ray8);
    }

    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT16))
    {
      RTCRay16 ray16; memset(&ray16,0,sizeof(ray16));
      for (size_t j=0; j<16; j++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        RTCRay ray = makeRay(org,dir); 
        setRay(ray16,j,ray);
      }
      __aligned(16) int valid16[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
      rtcOccluded16(valid16,scene,ray16);
      rtcIntersect16(valid16,scene,ray16);
    }
#endif
  }

  static bool build_join_test = false;

  struct RegressionTask
  {
    RegressionTask (size_t sceneIndex, size_t sceneCount, size_t threadCount)
      : sceneIndex(sceneIndex), sceneCount(sceneCount), scene(nullptr), numActiveThreads(0) { barrier.init(threadCount); }

    size_t sceneIndex;
    size_t sceneCount;
    RTCSceneRef scene;
    BarrierSys barrier;
    volatile size_t numActiveThreads;
  };

  struct ThreadRegressionTask
  {
    ThreadRegressionTask (size_t threadIndex, size_t threadCount, RTCDevice device, RegressionTask* task)
      : threadIndex(threadIndex), threadCount(threadCount), device(device), task(task) {}

    size_t threadIndex;
    size_t threadCount;
    RTCDevice device;
    RegressionTask* task;
  };

  ssize_t monitorProgressBreak = -1;
  atomic_t monitorProgressInvokations = 0;
  bool monitorProgressFunction(void* ptr, double dn) 
  {
    size_t n = atomic_add(&monitorProgressInvokations,1);
    if (n == monitorProgressBreak) return false;
    return true;
  }

  void rtcore_regression_static_thread(void* ptr)
  {
    ThreadRegressionTask* thread = (ThreadRegressionTask*) ptr;
    RegressionTask* task = thread->task;
    if (thread->threadIndex > 0) 
    {
      for (size_t i=0; i<task->sceneCount; i++) 
      {
	task->barrier.wait();
	if (thread->threadIndex < task->numActiveThreads) 
	{
          if (build_join_test) rtcCommit(task->scene);
          else                 {
            rtcCommitThread(task->scene,thread->threadIndex,task->numActiveThreads);
            rtcCommitThread(task->scene,thread->threadIndex,task->numActiveThreads);
          }
	  //CountErrors(thread->device);
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            atomic_add(&errorCounter,1);
          }
          else {
            for (size_t i=0; i<100; i++)
              shootRays(thread->device,task->scene);
          }
	}
        task->barrier.wait();
      }
      delete thread; thread = nullptr;
      return;
    }

    CountErrors(thread->device);
    int geom[1024];
    int types[1024];
    Sphere spheres[1024];
    size_t numVertices[1024];
    for (size_t i=0; i<1024; i++)  {
      geom[i] = -1;
      types[i] = 0;
      numVertices[i] = 0;
    }
    bool hasError = false;

    for (size_t i=0; i<task->sceneCount; i++) 
    {
      srand(task->sceneIndex*13565+i*3242);
      if (i%20 == 0) std::cout << "." << std::flush;

      RTCSceneFlags sflag = getSceneFlag(i); 
      task->scene = rtcDeviceNewScene(thread->device,sflag,aflags);
      CountErrors(thread->device);
      if (g_enable_build_cancel) rtcSetProgressMonitorFunction(task->scene,monitorProgressFunction,nullptr);
      avector<Sphere*> spheres;
      
      for (size_t j=0; j<10; j++) 
      {
        Vec3fa pos = 100.0f*Vec3fa(drand48(),drand48(),drand48());
	int type = random<int>()%6;
#if !defined(__MIC__) 
        switch (random<int>()%16) {
        case 0: pos = Vec3fa(nan); break;
        case 1: pos = Vec3fa(inf); break;
        case 2: pos = Vec3fa(1E30f); break;
        default: break;
        };
#endif
	size_t numPhi = random<int>()%100;
	if (type == 2) numPhi = random<int>()%10;
        size_t numTriangles = 2*2*numPhi*(numPhi-1);
	numTriangles = random<int>()%(numTriangles+1);
        switch (type) {
        case 0: addSphere(thread->device,task->scene,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,0.0f); break;
	case 1: addSphere(thread->device,task->scene,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,1.0f); break;
	case 2: addSubdivSphere(thread->device,task->scene,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,4,numTriangles,0.0f); break;
	case 3: addHair  (thread->device,task->scene,RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,numTriangles,0.0f); break;
	case 4: addHair  (thread->device,task->scene,RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,numTriangles,1.0f); break; 

        case 5: {
	  Sphere* sphere = new Sphere(pos,2.0f); spheres.push_back(sphere); 
	  addUserGeometryEmpty(thread->device,MODE_INTERSECT1,task->scene,sphere); break; // FIXME: imode
        }
	}
        //CountErrors(thread->device);
        if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
          atomic_add(&errorCounter,1);

          hasError = true;
          break;
        }
      }
      
      if (thread->threadCount) {
	task->numActiveThreads = max(size_t(1),random<int>() % thread->threadCount);
	task->barrier.wait();
        if (build_join_test) rtcCommit(task->scene);
        else                 {
          rtcCommitThread(task->scene,thread->threadIndex,task->numActiveThreads);
          rtcCommitThread(task->scene,thread->threadIndex,task->numActiveThreads);          
        }
      } else {
        if (!hasError) {
          rtcCommit(task->scene);
        }
      }
      //CountErrors(thread->device);

      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
        atomic_add(&errorCounter,1);
      }
      else {
        if (!hasError) {
          for (size_t i=0; i<100; i++) {
            shootRays(thread->device,task->scene);
          }
        }
      }

      if (thread->threadCount) 
	task->barrier.wait();

      task->scene = nullptr;
      CountErrors(thread->device);

      for (size_t i=0; i<spheres.size(); i++)
	delete spheres[i];
    }

    delete thread; thread = nullptr;
    return;
  }

  void rtcore_regression_dynamic_thread(void* ptr)
  {
    ThreadRegressionTask* thread = (ThreadRegressionTask*) ptr;
    RegressionTask* task = thread->task;
    if (thread->threadIndex > 0) 
    {
      for (size_t i=0; i<task->sceneCount; i++) 
      {
	task->barrier.wait();
	if (thread->threadIndex < task->numActiveThreads) 
	{
          if (build_join_test) rtcCommit(task->scene);
          else	               rtcCommitThread(task->scene,thread->threadIndex,task->numActiveThreads);
	  //CountErrors(thread->device);
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            atomic_add(&errorCounter,1);
          }
          else {
            for (size_t i=0; i<100; i++)
              shootRays(thread->device,task->scene);
          }
	}
	task->barrier.wait();
      }
      delete thread; thread = nullptr;
      return;
    }
    task->scene = rtcDeviceNewScene(thread->device,RTC_SCENE_DYNAMIC,aflags);
    CountErrors(thread->device);
    if (g_enable_build_cancel) rtcSetProgressMonitorFunction(task->scene,monitorProgressFunction,nullptr);
    int geom[1024];
    int types[1024];
    Sphere spheres[1024];
    size_t numVertices[1024];
    for (size_t i=0; i<1024; i++)  {
      geom[i] = -1;
      types[i] = 0;
      numVertices[i] = 0;
    }

    bool hasError = false;

    for (size_t i=0; i<task->sceneCount; i++) 
    {
      srand(task->sceneIndex*23565+i*2242);
      if (i%20 == 0) std::cout << "." << std::flush;

      for (size_t j=0; j<40; j++) 
      {
        int index = random<int>()%1024;
        if (geom[index] == -1) 
        {
          int type = random<int>()%10;
          Vec3fa pos = 100.0f*Vec3fa(drand48(),drand48(),drand48());
#if !defined(__MIC__)
          switch (random<int>()%16) {
          case 0: pos = Vec3fa(nan); break;
          case 1: pos = Vec3fa(inf); break;
          case 2: pos = Vec3fa(1E30f); break;
          default: break;
          };
#endif
          size_t numPhi = random<int>()%100;
	  if (type >= 3 || type <= 5) numPhi = random<int>()%10;
#if defined(__WIN32__)          
    numPhi = random<int>() % 4;
#endif

          size_t numTriangles = 2*2*numPhi*(numPhi-1);
          numTriangles = random<int>()%(numTriangles+1);
          types[index] = type;
          numVertices[index] = 2*numPhi*(numPhi+1);
          switch (type) {
          case 0: geom[index] = addSphere(thread->device,task->scene,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,0.0f); break;
          case 1: geom[index] = addSphere(thread->device,task->scene,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,numTriangles,0.0f); break;
          case 2: geom[index] = addSphere(thread->device,task->scene,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,numTriangles,0.0f); break;
          case 3: geom[index] = addSubdivSphere(thread->device,task->scene,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,4,numTriangles,0.0f); break;
	  case 4: geom[index] = addSubdivSphere(thread->device,task->scene,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,4,numTriangles,0.0f); break;
	  case 5: geom[index] = addSubdivSphere(thread->device,task->scene,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,4,numTriangles,0.0f); break;
          case 6: geom[index] = addSphere(thread->device,task->scene,RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,1.0f); break;
          case 7: geom[index] = addSphere(thread->device,task->scene,RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,numTriangles,1.0f); break;
          case 8: geom[index] = addSphere(thread->device,task->scene,RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,numTriangles,1.0f); break;
          case 9: spheres[index] = Sphere(pos,2.0f); geom[index] = addUserGeometryEmpty(thread->device,MODE_INTERSECT1,task->scene,&spheres[index]); break; // FIXME: imode
          }; 
	  //CountErrors(thread->device);
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            atomic_add(&errorCounter,1);
            hasError = true;
            break;
          }
        }
        else 
        {
          switch (types[index]) {
          case 0:
          case 3:
          case 6:
	  case 9: {
            rtcDeleteGeometry(task->scene,geom[index]);     
	    CountErrors(thread->device);
            geom[index] = -1; 
            break;
          }
          case 1: 
          case 2:
          case 4: 
          case 5:
	  case 7: 
          case 8: {
            int op = random<int>()%2;
            switch (op) {
            case 0: {
              rtcDeleteGeometry(task->scene,geom[index]);     
	      CountErrors(thread->device);
              geom[index] = -1; 
              break;
            }
            case 1: {
              Vertex3f* vertices = (Vertex3f*) rtcMapBuffer(task->scene,geom[index],RTC_VERTEX_BUFFER);
              if (vertices) { 
                for (size_t i=0; i<numVertices[index]; i++) vertices[i] += Vertex3f(0.1f);
              }
              rtcUnmapBuffer(task->scene,geom[index],RTC_VERTEX_BUFFER);
              
              if (types[index] == 7 || types[index] == 8) {
                Vertex3f* vertices = (Vertex3f*) rtcMapBuffer(task->scene,geom[index],RTC_VERTEX_BUFFER1);
                if (vertices) {
                  for (size_t i=0; i<numVertices[index]; i++) vertices[i] += Vertex3f(0.1f);
                }
                rtcUnmapBuffer(task->scene,geom[index],RTC_VERTEX_BUFFER1);
              }
              break;
            }
            }
            break;
          }
          }
        }
        
        /* entirely delete all objects from time to time */
        if (j%40 == 38) {
          for (size_t i=0; i<1024; i++) {
            if (geom[i] != -1) {
              rtcDeleteGeometry(task->scene,geom[i]);
              CountErrors(thread->device);
              geom[i] = -1;
            }
          }
        }
      }

      if (thread->threadCount) {
	task->numActiveThreads = max(size_t(1),random<int>() % thread->threadCount);
	task->barrier.wait();
        if (build_join_test) rtcCommit(task->scene);
        else                 rtcCommitThread(task->scene,thread->threadIndex,task->numActiveThreads);
      } else {
        if (!hasError) 
          rtcCommit(task->scene);
      }
      //CountErrors(thread->device);

      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR)
        atomic_add(&errorCounter,1);
      else
        if (!hasError)
          for (size_t i=0; i<100; i++)
            shootRays(thread->device,task->scene);

      if (thread->threadCount) 
	task->barrier.wait();
    }

    task->scene = nullptr;
    CountErrors(thread->device);

    delete thread; thread = nullptr;
    return;
  }

  struct IntensiveRegressionTest : public VerifyApplication::Test
  {
    thread_func func;
    int mode;
    
    IntensiveRegressionTest (std::string name, thread_func func, int mode)
      : VerifyApplication::Test(name,VerifyApplication::PASS), func(func), mode(mode) {}
    
    bool run(VerifyApplication* state)
    {
      errorCounter = 0;
      size_t sceneIndex = 0;
      while (sceneIndex < state->regressionN/5) 
      {
        if (mode)
        {
          ClearBuffers clear_before_return;
          build_join_test = (mode == 2);
          size_t numThreads = getNumberOfLogicalThreads();
#if defined (__MIC__)
          numThreads -= 4;
#endif
          
          std::vector<RegressionTask*> tasks;
          
          while (numThreads) 
          {
            size_t N = max(size_t(1),random<int>()%numThreads); numThreads -= N;
            RegressionTask* task = new RegressionTask(sceneIndex++,5,N);
            tasks.push_back(task);
            
            for (size_t i=0; i<N; i++) 
              g_threads.push_back(createThread(func,new ThreadRegressionTask(i,N,state->device,task),DEFAULT_STACK_SIZE,numThreads+i));
          }
          
          for (size_t i=0; i<g_threads.size(); i++)
            join(g_threads[i]);
          for (size_t i=0; i<tasks.size(); i++)
            delete tasks[i];
          
          g_threads.clear();
        }
        else
        {
          ClearBuffers clear_before_return;
          RegressionTask task(sceneIndex++,5,0);
          func(new ThreadRegressionTask(0,0,state->device,&task));
        }	
      }
      return errorCounter == 0;
    }
  };

  ssize_t monitorMemoryBreak = -1;
  atomic_t monitorMemoryBytesUsed = 0;
  atomic_t monitorMemoryInvokations = 0;
  bool monitorMemoryFunction(ssize_t bytes, bool post) 
  {
    atomic_add(&monitorMemoryBytesUsed,bytes);
    if (bytes > 0) {
      size_t n = atomic_add(&monitorMemoryInvokations,1);
      if (n == monitorMemoryBreak) {
        if (!post) atomic_add(&monitorMemoryBytesUsed,-bytes);
        return false;
      }
    }
    return true;
  }

  struct MemoryMonitorTest : public VerifyApplication::Test
  {
    thread_func func;
    
    MemoryMonitorTest (std::string name, thread_func func)
      : VerifyApplication::Test(name,VerifyApplication::PASS), func(func) {}
    
    bool run(VerifyApplication* state)
    {
      g_enable_build_cancel = true;
      rtcDeviceSetMemoryMonitorFunction(state->device,monitorMemoryFunction);
      
      size_t sceneIndex = 0;
      while (sceneIndex < state->regressionN/5) 
      {
        ClearBuffers clear_before_return;
        errorCounter = 0;
        monitorMemoryBreak = -1;
        monitorMemoryBytesUsed = 0;
        monitorMemoryInvokations = 0;
        monitorProgressBreak = -1;
        monitorProgressInvokations = 0;
        RegressionTask task1(sceneIndex,1,0);
        func(new ThreadRegressionTask(0,0,state->device,&task1));
        if (monitorMemoryBytesUsed) {
          rtcDeviceSetMemoryMonitorFunction(state->device,nullptr);
          //rtcDeviceSetProgressMonitorFunction(state->device,nullptr);
          return false;
        }
        monitorMemoryBreak = monitorMemoryInvokations * drand48();
        monitorMemoryBytesUsed = 0;
        monitorMemoryInvokations = 0;
        monitorProgressBreak = monitorProgressInvokations * 2.0f * drand48();
        monitorProgressInvokations = 0;
        RegressionTask task2(sceneIndex,1,0);
        func(new ThreadRegressionTask(0,0,state->device,&task2));
        if (monitorMemoryBytesUsed) { // || (monitorMemoryInvokations != 0 && errorCounter != 1)) { // FIXME: test that rtcCommit has returned with error code
          rtcDeviceSetMemoryMonitorFunction(state->device,nullptr);
          //rtcDeviceSetProgressMonitorFunction(state->device,nullptr);
          return false;
        }
        sceneIndex++;
      }
      g_enable_build_cancel = false;
      rtcDeviceSetMemoryMonitorFunction(state->device,nullptr);
      return true;
    }
  };

  struct GarbageGeometryTest : public VerifyApplication::Test
  {
    GarbageGeometryTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}
    
    bool run(VerifyApplication* state)
    {
      for (size_t i=0; i<5*state->regressionN; i++) 
      {
        ClearBuffers clear_before_return;
        srand(i*23565);
        if (i%20 == 0) std::cout << "." << std::flush;
        
        RTCSceneFlags sflag = getSceneFlag(i); 
        RTCSceneRef scene = rtcDeviceNewScene(state->device,sflag,aflags);
        AssertNoError(state->device);
        
        for (size_t j=0; j<20; j++) 
        {
          size_t numTriangles = random<int>()%256;
          switch (random<int>()%4) {
          case 0: addGarbageTriangles(state->device,scene,RTC_GEOMETRY_STATIC,numTriangles,false); break;
          case 1: addGarbageTriangles(state->device,scene,RTC_GEOMETRY_STATIC,numTriangles,true); break;
          case 2: addGarbageHair     (state->device,scene,RTC_GEOMETRY_STATIC,numTriangles,false); break;
          case 3: addGarbageHair     (state->device,scene,RTC_GEOMETRY_STATIC,numTriangles,true); break;
          }
          AssertNoError(state->device);
        }
        
        rtcCommit(scene);
        AssertNoError(state->device);
        scene = nullptr;
      }
      return true;
    }
  };

  VerifyApplication::VerifyApplication ()
    : Application(Application::FEATURE_RTCORE), device(nullptr), regressionN(200), numFailedTests(0), use_tests_to_run(false)
  {
    device = rtcNewDevice(rtcore.c_str());

    /* create list of all supported intersect modes to test */
    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT1)) intersectModes.push_back(MODE_INTERSECT1);
    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT4)) intersectModes.push_back(MODE_INTERSECT4);
    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT8)) intersectModes.push_back(MODE_INTERSECT8);
    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT16)) intersectModes.push_back(MODE_INTERSECT16);
    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM)) {
      intersectModes.push_back(MODE_INTERSECT1M);
      intersectModes.push_back(MODE_INTERSECTNM1);
      intersectModes.push_back(MODE_INTERSECTNM3);
      intersectModes.push_back(MODE_INTERSECTNM4);
      intersectModes.push_back(MODE_INTERSECTNM8);
      intersectModes.push_back(MODE_INTERSECTNM16);
      intersectModes.push_back(MODE_INTERSECTNp);
    }

    /* create list of all scene flags to test */
    sceneFlags.push_back(RTC_SCENE_STATIC);
    sceneFlags.push_back(RTC_SCENE_STATIC | RTC_SCENE_ROBUST);
    sceneFlags.push_back(RTC_SCENE_STATIC | RTC_SCENE_COMPACT);
    sceneFlags.push_back(RTC_SCENE_DYNAMIC);
    sceneFlags.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_ROBUST);
    sceneFlags.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_COMPACT);

    sceneFlagsRobust.push_back(RTC_SCENE_STATIC  | RTC_SCENE_ROBUST);
    sceneFlagsRobust.push_back(RTC_SCENE_STATIC  | RTC_SCENE_ROBUST | RTC_SCENE_COMPACT);
    sceneFlagsRobust.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_ROBUST);
    sceneFlagsRobust.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_ROBUST | RTC_SCENE_COMPACT);

    /* add all tests */
    addTest(new MultipleDevicesTest("multiple_devices"));
    addTest(new EmptySceneTest("empty_static",RTC_SCENE_STATIC));
    addTest(new EmptySceneTest("empty_dynamic",RTC_SCENE_DYNAMIC));
    // FIXME: add test with empty meshes
    addTest(new BaryDistanceTest("bary_distance_robust"));

    addTest(new FlagsTest("flags_static_static"     ,VerifyApplication::PASS, RTC_SCENE_STATIC, RTC_GEOMETRY_STATIC));
    addTest(new FlagsTest("flags_static_deformable" ,VerifyApplication::FAIL, RTC_SCENE_STATIC, RTC_GEOMETRY_DEFORMABLE));
    addTest(new FlagsTest("flags_static_dynamic"    ,VerifyApplication::FAIL, RTC_SCENE_STATIC, RTC_GEOMETRY_DYNAMIC));
    addTest(new FlagsTest("flags_dynamic_static"    ,VerifyApplication::PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC));
    addTest(new FlagsTest("flags_dynamic_deformable",VerifyApplication::PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DEFORMABLE));
    addTest(new FlagsTest("flags_dynamic_dynamic"   ,VerifyApplication::PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC));
    
    addTest(new StaticSceneTest("static_scene"));
    addTest(new UnmappedBeforeCommitTest("unmapped_before_commit"));
    addTest(new GetBoundsTest("get_bounds"));

    addTest(new DynamicEnableDisableTest("dynamic_enable_disable"));
    addTest(new GetUserDataTest("get_user_data"));

    addTest(new UpdateTest("update_deformable",RTC_GEOMETRY_DEFORMABLE));
    addTest(new UpdateTest("update_dynamic",RTC_GEOMETRY_DYNAMIC));
    addTest(new OverlappingTrianglesTest("overlapping_triangles",100000));
    addTest(new OverlappingHairTest("overlapping_hair",100000));
    addTest(new NewDeleteGeometryTest("new_delete_geometry"));

    addTest(new InterpolateSubdivTest("interpolate_subdiv4",4));
    addTest(new InterpolateSubdivTest("interpolate_subdiv5",5));
    addTest(new InterpolateSubdivTest("interpolate_subdiv8",8));
    addTest(new InterpolateSubdivTest("interpolate_subdiv11",11));
    addTest(new InterpolateSubdivTest("interpolate_subdiv12",12));
    addTest(new InterpolateSubdivTest("interpolate_subdiv15",15));

    addTest(new InterpolateTrianglesTest("interpolate_triangles4",4));
    addTest(new InterpolateTrianglesTest("interpolate_triangles5",5));
    addTest(new InterpolateTrianglesTest("interpolate_triangles8",8));
    addTest(new InterpolateTrianglesTest("interpolate_triangles11",11));
    addTest(new InterpolateTrianglesTest("interpolate_triangles12",12));
    addTest(new InterpolateTrianglesTest("interpolate_triangles15",15));

    addTest(new InterpolateHairTest("interpolate_hair4",4));
    addTest(new InterpolateHairTest("interpolate_hair5",5));
    addTest(new InterpolateHairTest("interpolate_hair8",8));
    addTest(new InterpolateHairTest("interpolate_hair11",11));
    addTest(new InterpolateHairTest("interpolate_hair12",12));
    addTest(new InterpolateHairTest("interpolate_hair15",15));

    addTest(new BuildTest("build"));

    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_BUFFER_STRIDE)) {
      addTest(new BufferStrideTest("buffer_stride"));
    }

    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_RAY_MASK)) {
      addTest(new RayMasksTest("ray_masks"));
    }

    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECTION_FILTER)) {
      addTest(new IntersectionFilterTest("intersection_filter_tris",false));
      addTest(new IntersectionFilterTest("intersection_filter_subdiv",true));
    }

    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_BACKFACE_CULLING)) {
      addTest(new BackfaceCullingTest("backface_culling"));
    }

    for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          addTest(new InactiveRaysTest("inactive_rays_"+to_string(sflags)+"_"+to_string(imode),sflags,RTC_GEOMETRY_STATIC,imode));
    
    const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
    for (auto sflags : sceneFlagsRobust) 
      for (auto imode : intersectModes) 
        for (std::string model : {"sphere", "plane"}) 
          addTest(new WatertightTest("watertight_"+to_string(sflags)+"_"+model+"_"+to_string(imode),sflags,imode,model,watertight_pos));
    
    if (rtcDeviceGetParameter1i(device,RTC_CONFIG_IGNORE_INVALID_RAYS))
    {
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          addTest(new NaNTest("nan_test_"+to_string(sflags)+"_"+to_string(imode),sflags,RTC_GEOMETRY_STATIC,imode));

      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          addTest(new InfTest("inf_test_"+to_string(sflags)+"_"+to_string(imode),sflags,RTC_GEOMETRY_STATIC,imode));
    }
    
    addTest(new IntensiveRegressionTest("regression_static",rtcore_regression_static_thread,0));
    addTest(new IntensiveRegressionTest("regression_dynamic",rtcore_regression_dynamic_thread,0));

    addTest(new IntensiveRegressionTest("regression_static_user_threads", rtcore_regression_static_thread,1));
    addTest(new IntensiveRegressionTest("regression_dynamic_user_threads",rtcore_regression_dynamic_thread,1));

    addTest(new IntensiveRegressionTest("regression_static_build_join", rtcore_regression_static_thread,2));
    addTest(new IntensiveRegressionTest("regression_dynamic_build_join",rtcore_regression_dynamic_thread,2));
    
    addTest(new MemoryMonitorTest("regression_static_memory_monitor", rtcore_regression_static_thread));
    addTest(new MemoryMonitorTest("regression_dynamic_memory_monitor",rtcore_regression_dynamic_thread));
    
    addTest(new GarbageGeometryTest("regression_garbage_geom"));

    rtcDeleteDevice(device);

    /* register all command line options*/
    std::string run_docu = "--run <regexpr>: Runs all tests whose name match the regular expression. Supported tests are:";
    for (auto test : tests) run_docu += "\n  " + test->name;
    registerOption("run", [this] (Ref<ParseStream> cin, const FileName& path) {
        use_tests_to_run = true;
        std::smatch match;
        std::regex regexpr(cin->getString());
        for (auto test : tests) {
          if (std::regex_match(test->name, match, regexpr)) {
            tests_to_run.push_back(test);
          }
        }
      }, run_docu);
    
    registerOption("regressions", [this] (Ref<ParseStream> cin, const FileName& path) {
        regressionN = cin->getInt();
      }, "--regressions <int>: number of regressions to perform");
  }

  int VerifyApplication::main(int argc, char** argv) try
  {
    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    
    /* parse command line options */
    parseCommandLine(argc,argv);
    
    /* perform tests */
    device = rtcNewDevice(rtcore.c_str());
    //error_handler(rtcDeviceGetError(device));

    /* execute specific user tests */
    if (use_tests_to_run) 
    {
      for (auto test : tests_to_run) 
        runTest(test);
    } 
    /* run all tests by default */
    else 
    {
      /* print Embree version */
      rtcInit("verbose=1");
      error_handler(rtcGetError());
      rtcExit();

      /* run all available tests */
      for (auto test : tests) 
        runTest(test);
    }

    rtcDeleteDevice(device);
    return numFailedTests;
  }
  catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return 1;
  }
  catch (...) {
    std::cout << "Error: unknown exception caught." << std::endl;
    return 1;
  }

  void VerifyApplication::addTest(Ref<Test> test) 
  {
    tests.push_back(test);
    name2test[test->name] = test;
  }
  
  void VerifyApplication::runTest(Ref<Test> test)
  {
    bool ok = true;
    std::cout << std::setw(50) << test->name << " ..." << std::flush;
    try {
      ok &= test->run(this);
      AssertNoError(device);
    } catch (...) {
      ok = false;
    }
    if ((test->ty == PASS) == ok) 
      std::cout << GREEN(" [PASSED]") << std::endl << std::flush;
    else {
      std::cout << RED(" [FAILED]") << std::endl << std::flush;
      numFailedTests++;
    }
  }
}

int main(int argc, char** argv)
{
  embree::VerifyApplication app;
  return app.main(argc,argv);
}
