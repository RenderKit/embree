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
#include "../tutorials/common/scenegraph/scenegraph.h"
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
  std::vector<Ref<SceneGraph::Node>> nodes;
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
    nodes.clear();
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
  RTCAlgorithmFlags aflags_all = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8 | RTC_INTERSECT16 | RTC_INTERSECT_STREAM);
  
  bool g_enable_build_cancel = false;

  unsigned addGeometry(const RTCDeviceRef& device, const RTCSceneRef& scene, const RTCGeometryFlags gflag, const Ref<SceneGraph::Node>& node, bool mblur = false)
  {
    g_mutex2.lock();
    nodes.push_back(node);
    g_mutex2.unlock();

    if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
    {
      int numTimeSteps = mblur ? 2 : 1;
      unsigned geomID = rtcNewTriangleMesh (scene, gflag, mesh->triangles.size(), mesh->v.size(), numTimeSteps);
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER ,mesh->triangles.data(),0,sizeof(SceneGraph::TriangleMeshNode::Triangle));
      if (mesh->v .size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER0,mesh->v .data(),0,sizeof(SceneGraph::TriangleMeshNode::Vertex));
      if (mesh->v2.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER1,mesh->v2.data(),0,sizeof(SceneGraph::TriangleMeshNode::Vertex));
      return geomID;
    }
    else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>())
    {
      int numTimeSteps = mblur ? 2 : 1;
      unsigned geomID = rtcNewQuadMesh (scene, gflag, mesh->quads.size(), mesh->v.size(), numTimeSteps);
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER ,mesh->quads.data(),0,sizeof(SceneGraph::QuadMeshNode::Quad  ));
      if (mesh->v .size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER0,mesh->v .data(),0,sizeof(SceneGraph::QuadMeshNode::Vertex));
      if (mesh->v2.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER1,mesh->v2.data(),0,sizeof(SceneGraph::QuadMeshNode::Vertex));
      return geomID;
    } 
    else if (Ref<SceneGraph::SubdivMeshNode> mesh = node.dynamicCast<SceneGraph::SubdivMeshNode>())
    {
      int numTimeSteps = mblur ? 2 : 1;
      unsigned geomID = rtcNewSubdivisionMesh (scene, gflag, 
                                               mesh->verticesPerFace.size(), mesh->position_indices.size(), mesh->positions.size(), 
                                               mesh->edge_creases.size(), mesh->vertex_creases.size(), mesh->holes.size(), numTimeSteps);
      rtcSetBuffer(scene,geomID,RTC_FACE_BUFFER  ,mesh->verticesPerFace.data(), 0,sizeof(int));
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER ,mesh->position_indices.data(),0,sizeof(int));
      if (mesh->positions .size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER0,mesh->positions .data(),0,sizeof(SceneGraph::SubdivMeshNode::Vertex));
      if (mesh->positions2.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER1,mesh->positions2.data(),0,sizeof(SceneGraph::SubdivMeshNode::Vertex));
      if (mesh->edge_creases.size()) rtcSetBuffer(scene,geomID,RTC_EDGE_CREASE_INDEX_BUFFER,mesh->edge_creases.data(),0,2*sizeof(int));
      if (mesh->edge_crease_weights.size()) rtcSetBuffer(scene,geomID,RTC_EDGE_CREASE_WEIGHT_BUFFER,mesh->edge_crease_weights.data(),0,sizeof(float));
      if (mesh->vertex_creases.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_CREASE_INDEX_BUFFER,mesh->vertex_creases.data(),0,sizeof(int));
      if (mesh->vertex_crease_weights.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_CREASE_WEIGHT_BUFFER,mesh->vertex_crease_weights.data(),0,sizeof(float));
      if (mesh->holes.size()) rtcSetBuffer(scene,geomID,RTC_HOLE_BUFFER,mesh->holes.data(),0,sizeof(int));
      rtcSetTessellationRate(scene,geomID,mesh->tessellationRate);
      rtcSetBoundaryMode(scene,geomID,mesh->boundaryMode);
      return geomID;
    }
    else {
      THROW_RUNTIME_ERROR("unknown node type");
    }
    return 0;
  }
  
  unsigned addPlane (const RTCDeviceRef& device, const RTCSceneRef& scene, const RTCGeometryFlags gflag, size_t num, const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy) {
    return addGeometry(device,scene,gflag,SceneGraph::createTrianglePlane(p0,dx,dy,num,num));
  }

  unsigned addSubdivPlane (const RTCDeviceRef& device, const RTCSceneRef& scene, RTCGeometryFlags gflag, size_t num, const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy) {
    return addGeometry(device,scene,gflag,SceneGraph::createSubdivPlane(p0,dx,dy,num,num,2.0f));
  }

  unsigned addSphere (const RTCDeviceRef& device, const RTCSceneRef& scene, RTCGeometryFlags gflag, const Vec3fa& pos, const float r, size_t numPhi, size_t maxTriangles = -1, float motion = 0.0f)
  {
    bool mblur = motion != 0.0f;
    Ref<SceneGraph::Node> node = SceneGraph::createTriangleSphere(pos,r,numPhi);
    if (mblur) SceneGraph::set_motion_vector(node,Vec3fa(motion));
    if (maxTriangles !=   -1) SceneGraph::resize_randomly(node,maxTriangles);
    return addGeometry(device,scene,gflag,node,mblur);
  }

  void addRandomSubdivFeatures(Ref<SceneGraph::SubdivMeshNode> mesh, size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles)
  {
    std::vector<int> offsets;
    std::vector<int>& faces = mesh->verticesPerFace;
    std::vector<int>& indices = mesh->position_indices;
    for (size_t i=0,j=0; i<mesh->verticesPerFace.size(); i++) {
      offsets.push_back(j); j+=mesh->verticesPerFace[i];
    } 

    for (size_t i=0; i<numEdgeCreases; i++) 
    {
      if (faces.size()) {
	int f = random<int>() % faces.size();
	int n = faces[f];
	int e = random<int>() % n;
	mesh->edge_creases.push_back(Vec2i(indices[offsets[f]+(e+0)%n],indices[offsets[f]+(e+1)%n]));
      } else {
        mesh->edge_creases.push_back(Vec2i(0,0));
      }
      mesh->edge_crease_weights.push_back(10.0f*drand48());
    }
    
    for (size_t i=0; i<numVertexCreases; i++) 
    {
      if (faces.size()) {
        size_t f = random<int>() % faces.size();
        size_t e = random<int>()% faces[f];
        mesh->vertex_creases.push_back(indices[offsets[f] + e]);
        mesh->vertex_crease_weights.push_back(10.0f*drand48());
      }
    }
    
    for (size_t i=0; i<numHoles; i++) {
      mesh->holes.push_back(random<int>() % faces.size());
    }
  }    

  /* adds a subdiv sphere to the scene */
  unsigned int addSubdivSphere (const RTCDeviceRef& device, const RTCSceneRef& scene, RTCGeometryFlags gflag, const Vec3fa& pos, const float r, size_t numPhi, float level, size_t maxFaces = -1, float motion = 0.0f)
  {
    bool mblur = motion != 0.0f;
    Ref<SceneGraph::Node> node = SceneGraph::createSubdivSphere(pos,r,numPhi,level);
    if (mblur) SceneGraph::set_motion_vector(node,Vec3fa(motion));
    if (maxFaces != -1) SceneGraph::resize_randomly(node,maxFaces);
    addRandomSubdivFeatures(node.dynamicCast<SceneGraph::SubdivMeshNode>(),10,10,0);
    return addGeometry(device,scene,gflag,node,mblur);
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

  void IntersectFuncN(const int* valid,
                      void* ptr,
                      const RTCIntersectContext* context,
                      RTCRayN* rays,
                      size_t N,
                      size_t item)
  {
  }
  
  
  unsigned addUserGeometryEmpty (RTCDevice g_device, const RTCSceneRef& scene, Sphere* sphere)
  {
    BBox3fa bounds = sphere->bounds(); 
    unsigned geom = rtcNewUserGeometry (scene,1);
    rtcSetBoundsFunction(scene,geom,(RTCBoundsFunc)BoundsFunc);
    rtcSetUserData(scene,geom,sphere);
    rtcSetIntersectFunctionN(scene,geom,IntersectFuncN);
    rtcSetOccludedFunctionN(scene,geom,IntersectFuncN);
    return geom;
  }

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  struct InitExitTest : public VerifyApplication::Test
  {
    InitExitTest (std::string name)
      : VerifyApplication::Test(name,0,VerifyApplication::PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      rtcInit("verbose=1");
      error_handler(rtcGetError());
      rtcExit();
      return VerifyApplication::PASSED;
    }
  };

  struct EmbreeInternalTest : public VerifyApplication::Test
  {
    EmbreeInternalTest (std::string name, size_t testID)
      : VerifyApplication::Test(name,0,VerifyApplication::PASS), testID(testID) {}
  
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      RTCDeviceRef device = rtcNewDevice(state->rtcore.c_str());
      AssertNoError(device);
      return (VerifyApplication::TestReturnValue)rtcDeviceGetParameter1i(device,(RTCParameter)(3000000+testID));
    }

    size_t testID;
  };

  struct MultipleDevicesTest : public VerifyApplication::Test
  {
    MultipleDevicesTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDevice device1 = rtcNewDevice(cfg.c_str());
      AssertNoError(device1);
      RTCDevice device2 = rtcNewDevice(cfg.c_str());
      AssertNoError(device2);
      RTCDevice device3 = rtcNewDevice(cfg.c_str());
      AssertNoError(device3);
      rtcDeleteDevice(device1);
      rtcDeleteDevice(device3);
      rtcDeleteDevice(device2);
      return VerifyApplication::PASSED;
    }
  };

  struct FlagsTest : public VerifyApplication::Test
  {
    RTCSceneFlags sceneFlags;
    RTCGeometryFlags geomFlags;

    FlagsTest (std::string name, int isa, VerifyApplication::TestType type, RTCSceneFlags sceneFlags, RTCGeometryFlags geomFlags)
      : VerifyApplication::Test(name,isa,type), sceneFlags(sceneFlags), geomFlags(geomFlags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,sceneFlags,aflags);
      AssertNoError(device);
      rtcNewTriangleMesh (scene, geomFlags, 0, 0);
      AssertNoError(device);
      rtcNewHairGeometry (scene, geomFlags, 0, 0);
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }
  };
  
  struct UnmappedBeforeCommitTest : public VerifyApplication::Test
  {
    UnmappedBeforeCommitTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,aflags);
      AssertNoError(device);
      unsigned geom0 = addSphere(device,scene,RTC_GEOMETRY_STATIC,zero,1.0f,50);
      unsigned geom1 = addSphere(device,scene,RTC_GEOMETRY_STATIC,zero,1.0f,50);
      AssertNoError(device);
      rtcMapBuffer(scene,geom0,RTC_INDEX_BUFFER);
      rtcMapBuffer(scene,geom0,RTC_VERTEX_BUFFER);
      AssertNoError(device);
      rtcCommit (scene);
      AssertError(device,RTC_INVALID_OPERATION); // error, buffers still mapped
      return VerifyApplication::PASSED;
    }
  };

  struct GetBoundsTest : public VerifyApplication::Test
  {
    GetBoundsTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(device);
      Ref<SceneGraph::Node> node = SceneGraph::createTriangleSphere(zero,1.0f,50);
      BBox3fa bounds0 = node->bounds();
      unsigned geom0 = addGeometry(device,scene,RTC_GEOMETRY_STATIC,node);
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);
      BBox3fa bounds1;
      rtcGetBounds(scene,(RTCBounds&)bounds1);
      return (VerifyApplication::TestReturnValue)(bounds0 == bounds1);
    }
  };

  struct GetUserDataTest : public VerifyApplication::Test
  {
    GetUserDataTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(device);
      unsigned geom0 = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      AssertNoError(device);
      rtcSetUserData(scene,geom0,(void*)1);
      
      unsigned geom1 = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, 0, 0, 0, 0, 0, 0, 1);
      AssertNoError(device);
      rtcSetUserData(scene,geom1,(void*)2);
      
      unsigned geom2 = rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      AssertNoError(device);
      rtcSetUserData(scene,geom2,(void*)3);
      
      unsigned geom3 = rtcNewUserGeometry (scene,0);
      AssertNoError(device);
      rtcSetUserData(scene,geom3,(void*)4);
      
      rtcCommit (scene);
      AssertNoError(device);
      
      if ((size_t)rtcGetUserData(scene,geom0) != 1) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom1) != 2) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom2) != 3) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom3) != 4) return VerifyApplication::FAILED;
      
      return VerifyApplication::PASSED;
    }
  };

  struct BufferStrideTest : public VerifyApplication::Test
  {
    BufferStrideTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,aflags);
      AssertNoError(device);
      unsigned geom = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 16, 16);
      AssertNoError(device);
      avector<char> indexBuffer(8+16*6*sizeof(int));
      avector<char> vertexBuffer(12+16*9*sizeof(float)+4);
      
      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),1,3*sizeof(int));
      AssertError(device,RTC_INVALID_OPERATION);
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),1,3*sizeof(float));
      AssertError(device,RTC_INVALID_OPERATION);

      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int)+3);
      AssertError(device,RTC_INVALID_OPERATION);
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,3*sizeof(float)+3);
      AssertError(device,RTC_INVALID_OPERATION);
      
      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int));
      AssertNoError(device);
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,3*sizeof(float));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),8,6*sizeof(int));
      AssertNoError(device);
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),12,9*sizeof(float));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,4*sizeof(float));
      AssertNoError(device);
      
      return VerifyApplication::PASSED;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  struct EmptySceneTest : public VerifyApplication::Test
  {
    EmptySceneTest (std::string name, int isa, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), sflags(sflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,aflags);
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }

  public:
    RTCSceneFlags sflags;
  };

  struct EmptyGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;

    EmptyGeometryTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,aflags);
      rtcNewTriangleMesh (scene,gflags,0,0,1);
      rtcNewTriangleMesh (scene,gflags,0,0,2);
      rtcNewQuadMesh (scene,gflags,0,0,1);
      rtcNewQuadMesh (scene,gflags,0,0,2);
      rtcNewSubdivisionMesh (scene,gflags,0,0,0,0,0,0,1);
      rtcNewSubdivisionMesh (scene,gflags,0,0,0,0,0,0,2);
      rtcNewHairGeometry (scene,gflags,0,0,1);
      rtcNewHairGeometry (scene,gflags,0,0,2);
      rtcNewCurveGeometry (scene,gflags,0,0,1);
      rtcNewCurveGeometry (scene,gflags,0,0,2);
      rtcNewUserGeometry2 (scene,0,1);
      rtcNewUserGeometry2 (scene,0,2);
      rtcCommit (scene);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }
  };

   struct BuildTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags; 

    BuildTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}
    
    VerifyApplication::TestReturnValue run (VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,aflags);
      addSphere(device,scene,gflags,zero,1E-24f,50);
      addHair(device,scene,gflags,zero,1E-24f,1E-26f,100,1E-26f);
      addSphere(device,scene,gflags,zero,1E-24f,50);
      addHair(device,scene,gflags,zero,1E-24f,1E-26f,100,1E-26f);
      rtcCommit (scene);
      AssertNoError(device);
      if ((sflags & RTC_SCENE_DYNAMIC) == 0) {
        rtcDisable(scene,0);
        AssertAnyError(device);
        rtcDisable(scene,1);
        AssertAnyError(device);
        rtcDisable(scene,2);
        AssertAnyError(device);
        rtcDisable(scene,3);
        AssertAnyError(device);
      }
      return VerifyApplication::PASSED;
    }
  };

  struct OverlappingTrianglesTest : public VerifyApplication::Test
  {
    int N;
    
    OverlappingTrianglesTest (std::string name, int isa, int N)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), N(N) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,aflags);
      AssertNoError(device);
      rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, N, 3);
      AssertNoError(device);
      
      Vertex3fa* vertices = (Vertex3fa*) rtcMapBuffer(scene,0,RTC_VERTEX_BUFFER);
      vertices[0].x = 0.0f; vertices[0].y = 0.0f; vertices[0].z = 0.0f;
      vertices[1].x = 1.0f; vertices[1].y = 0.0f; vertices[1].z = 0.0f;
      vertices[2].x = 0.0f; vertices[2].y = 1.0f; vertices[2].z = 0.0f;
      rtcUnmapBuffer(scene,0,RTC_VERTEX_BUFFER);
      AssertNoError(device);
      
      Triangle* triangles = (Triangle*) rtcMapBuffer(scene,0,RTC_INDEX_BUFFER);
      for (size_t i=0; i<N; i++) {
        triangles[i].v0 = 0;
        triangles[i].v1 = 1;
        triangles[i].v2 = 2;
      }
      rtcUnmapBuffer(scene,0,RTC_INDEX_BUFFER);
      AssertNoError(device);
      
      rtcCommit (scene);
      AssertNoError(device);
      
      return VerifyApplication::PASSED;
    }
  };
    
  struct OverlappingHairTest : public VerifyApplication::Test
  {
    int N;
    
    OverlappingHairTest (std::string name, int isa, int N)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), N(N) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,aflags);
      AssertNoError(device);
      rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, N, 4);
      AssertNoError(device);
      
      Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene,0,RTC_VERTEX_BUFFER);
      vertices[0].x = 0.0f; vertices[0].y = 0.0f; vertices[0].z = 0.0f; vertices[0].w = 0.1f;
      vertices[1].x = 0.0f; vertices[1].y = 0.0f; vertices[1].z = 1.0f; vertices[1].w = 0.1f;
      vertices[2].x = 0.0f; vertices[2].y = 1.0f; vertices[2].z = 1.0f; vertices[2].w = 0.1f;
      vertices[3].x = 0.0f; vertices[3].y = 1.0f; vertices[3].z = 0.0f; vertices[3].w = 0.1f;
      rtcUnmapBuffer(scene,0,RTC_VERTEX_BUFFER);
      AssertNoError(device);
      
      int* indices = (int*) rtcMapBuffer(scene,0,RTC_INDEX_BUFFER);
      for (size_t i=0; i<N; i++) {
        indices[i] = 0;
      }
      rtcUnmapBuffer(scene,0,RTC_INDEX_BUFFER);
      AssertNoError(device);
      
      rtcCommit (scene);
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  struct NewDeleteGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;

    NewDeleteGeometryTest (std::string name, int isa, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), sflags(sflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,aflags_all);
      AssertNoError(device);
      int geom[128];
      for (size_t i=0; i<128; i++) geom[i] = -1;
      Sphere spheres[128];
      memset(spheres,0,sizeof(spheres));
      
      for (size_t i=0; i<size_t(50*state->intensity); i++) 
      {
        for (size_t j=0; j<10; j++) {
          int index = random<int>()%128;
          Vec3fa pos = 100.0f*Vec3fa(drand48(),drand48(),drand48());
          if (geom[index] == -1) {
            switch (random<int>()%4) {
            case 0: geom[index] = addSphere(device,scene,RTC_GEOMETRY_STATIC,pos,2.0f,10); break;
            case 1: geom[index] = addHair  (device,scene,RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,10); break;
            case 2: geom[index] = addSubdivSphere(device,scene,RTC_GEOMETRY_STATIC,pos,2.0f,4,4); break;
            case 3: 
              spheres[index] = Sphere(pos,2.0f);
              geom[index] = addUserGeometryEmpty(device,scene,&spheres[index]); break;
            }
            AssertNoError(device);
          }
          else { 
            rtcDeleteGeometry(scene,geom[index]);     
            AssertNoError(device);
            geom[index] = -1; 
          }
        }
        rtcCommit(scene);
        AssertNoError(device);
        rtcCommit(scene);
        AssertNoError(device);
        if (i%2 == 0) std::cout << "." << std::flush;
      }
      
      /* now delete all geometries */
      for (size_t i=0; i<128; i++) 
        if (geom[i] != -1) rtcDeleteGeometry(scene,geom[i]);
      rtcCommit(scene);
      AssertNoError(device);

      rtcCommit (scene);
      AssertNoError(device);

      return VerifyApplication::PASSED;
    }
  };

  struct EnableDisableGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;

    EnableDisableGeometryTest (std::string name, int isa, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), sflags(sflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,aflags);
      AssertNoError(device);
      unsigned geom0 = addSphere(device,scene,RTC_GEOMETRY_STATIC,Vec3fa(-1,0,-1),1.0f,50);
      //unsigned geom1 = addSphere(device,scene,RTC_GEOMETRY_STATIC,Vec3fa(-1,0,+1),1.0f,50);
      unsigned geom1 = addHair  (device,scene,RTC_GEOMETRY_STATIC,Vec3fa(-1,0,+1),1.0f,1.0f,1);
      unsigned geom2 = addSphere(device,scene,RTC_GEOMETRY_STATIC,Vec3fa(+1,0,-1),1.0f,50);
      //unsigned geom3 = addSphere(device,scene,RTC_GEOMETRY_STATIC,Vec3fa(+1,0,+1),1.0f,50);
      unsigned geom3 = addHair  (device,scene,RTC_GEOMETRY_STATIC,Vec3fa(+1,0,+1),1.0f,1.0f,1);
      AssertNoError(device);
      
      for (size_t i=0; i<16; i++) 
      {
        bool enabled0 = i & 1, enabled1 = i & 2, enabled2 = i & 4, enabled3 = i & 8;
        if (enabled0) rtcEnable(scene,geom0); else rtcDisable(scene,geom0); AssertNoError(device);
        if (enabled1) rtcEnable(scene,geom1); else rtcDisable(scene,geom1); AssertNoError(device);
        if (enabled2) rtcEnable(scene,geom2); else rtcDisable(scene,geom2); AssertNoError(device);
        if (enabled3) rtcEnable(scene,geom3); else rtcDisable(scene,geom3); AssertNoError(device);
        rtcCommit (scene);
        AssertNoError(device);
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
          if (!ok0 || !ok1 || !ok2 || !ok3) return VerifyApplication::FAILED;
        }
      }
      return VerifyApplication::PASSED;
    }
  };
  
  struct UpdateTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;

    UpdateTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}
    
    static void move_mesh_vec3f(const RTCSceneRef& scene, unsigned mesh, size_t numVertices, Vec3fa& pos) 
    {
      Vertex3f* vertices = (Vertex3f*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
      for (size_t i=0; i<numVertices; i++) vertices[i] += Vertex3f(pos);
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
      rtcUpdate(scene,mesh);
    }
    
    static void move_mesh_vec3fa(const RTCSceneRef& scene, unsigned mesh, size_t numVertices, Vec3fa& pos) 
    {
      Vertex3fa* vertices = (Vertex3fa*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
      for (size_t i=0; i<numVertices; i++) vertices[i] += Vertex3fa(pos);
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
      rtcUpdate(scene,mesh);
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;
      
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      AssertNoError(device);
      size_t numPhi = 10;
      size_t numVertices = 2*numPhi*(numPhi+1);
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      unsigned geom0 = addSphere(device,scene,gflags,pos0,1.0f,numPhi);
      unsigned geom1 = addHair  (device,scene,gflags,pos1,1.0f,1.0f,1);
      unsigned geom2 = addSphere(device,scene,gflags,pos2,1.0f,numPhi);
      unsigned geom3 = addHair  (device,scene,gflags,pos3,1.0f,1.0f,1);
      AssertNoError(device);
      
      for (size_t i=0; i<16; i++) 
      {
        bool move0 = i & 1, move1 = i & 2, move2 = i & 4, move3 = i & 8;
        Vec3fa ds(2,0.1f,2);
        if (move0) { move_mesh_vec3f (scene,geom0,numVertices,ds); pos0 += ds; }
        if (move1) { move_mesh_vec3fa(scene,geom1,4,ds); pos1 += ds; }
        if (move2) { move_mesh_vec3f (scene,geom2,numVertices,ds); pos2 += ds; }
        if (move3) { move_mesh_vec3fa(scene,geom3,4,ds); pos3 += ds; }
        rtcCommit (scene);
        AssertNoError(device);

        RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); // hits geomID == 0
        RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); // hits geomID == 1
        RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); // hits geomID == 2
        RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); // hits geomID == 3
        RTCRay testRays[4] = { ray0, ray1, ray2, ray3 };

        const size_t maxRays = 100;
        RTCRay rays[maxRays];
        for (size_t numRays=1; numRays<maxRays; numRays++) {
          for (size_t i=0; i<numRays; i++) rays[i] = testRays[i%4];
          IntersectWithMode(imode,ivariant,scene,rays,numRays);
          for (size_t i=0; i<numRays; i++) if (rays[i].geomID == -1) return VerifyApplication::FAILED;
        }
      }
      return VerifyApplication::PASSED;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

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
    
    InterpolateSubdivTest (std::string name, int isa, size_t N)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), N(N) {}

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
    
    bool checkSubdivInterpolation(const RTCDeviceRef& device, const RTCSceneRef& scene, int geomID, RTCBufferType buffer, float* vertices0, size_t N, size_t N_total)
    {
      rtcSetBoundaryMode(scene,geomID,RTC_BOUNDARY_EDGE_ONLY);
      AssertNoError(device);
      rtcDisable(scene,geomID);
      AssertNoError(device);
      rtcCommit(scene);
      AssertNoError(device);
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
      AssertNoError(device);
      rtcCommit(scene);
      AssertNoError(device);
      
      passed &= checkInterpolationSharpVertex(scene,geomID,0,0.0f,0.0f,0,buffer,vertices0,N,N_total);
      passed &= checkInterpolationSharpVertex(scene,geomID,2,1.0f,0.0f,3,buffer,vertices0,N,N_total);
      return passed;
    }
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(device);
      unsigned int geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, num_interpolation_quad_faces, num_interpolation_quad_faces*4, num_interpolation_vertices, 3, 2, 0, 1);
      AssertNoError(device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_quad_indices , 0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_FACE_BUFFER,   interpolation_quad_faces,    0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,   interpolation_edge_crease_indices,  0, 2*sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,  interpolation_edge_crease_weights,  0, sizeof(float));
      rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER, interpolation_vertex_crease_indices,0, sizeof(unsigned int));
      rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER,interpolation_vertex_crease_weights,0, sizeof(float));
      AssertNoError(device);
      
      float* vertices0 = new float[M];
      for (size_t i=0; i<M; i++) vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0, 0, N*sizeof(float));
      AssertNoError(device);
      
      /*float* vertices1 = new float[M];
        for (size_t i=0; i<M; i++) vertices1[i] = drand48();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1, 0, N*sizeof(float));
        AssertNoError(device);*/
      
      float* user_vertices0 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0, 0, N*sizeof(float));
      AssertNoError(device);
      
      float* user_vertices1 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices1[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1, 0, N*sizeof(float));
      AssertNoError(device);
      
      bool passed = true;
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_VERTEX_BUFFER0,vertices0,N,N);
      //passed &= checkSubdivInterpolation(device,scene,geomID,RTC_VERTEX_BUFFER1,vertices1,N,N);
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0,N,N);
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1,N,N);
      
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_VERTEX_BUFFER0,vertices0,1,N);
      //passed &= checkSubdivInterpolation(device,scene,geomID,RTC_VERTEX_BUFFER1,vertices1,1,N);
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_USER_VERTEX_BUFFER0,user_vertices0,1,N);
      passed &= checkSubdivInterpolation(device,scene,geomID,RTC_USER_VERTEX_BUFFER1,user_vertices1,1,N);
      
      delete[] vertices0;
      //delete[] vertices1;
      delete[] user_vertices0;
      delete[] user_vertices1;

      return (VerifyApplication::TestReturnValue) passed;
    }
  };

  struct InterpolateTrianglesTest : public VerifyApplication::Test
  {
    size_t N;
    
    InterpolateTrianglesTest (std::string name, int isa, size_t N)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), N(N) {}
    
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

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));

      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(device);
      unsigned int geomID = rtcNewTriangleMesh(scene, RTC_GEOMETRY_STATIC, num_interpolation_triangle_faces, num_interpolation_vertices, 1);
      AssertNoError(device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_triangle_indices , 0, 3*sizeof(unsigned int));
      AssertNoError(device);
      
      float* vertices0 = new float[M];
      for (size_t i=0; i<M; i++) vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0, 0, N*sizeof(float));
      AssertNoError(device);
      
      /*float* vertices1 = new float[M];
        for (size_t i=0; i<M; i++) vertices1[i] = drand48();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1, 0, N*sizeof(float));
        AssertNoError(device);*/
      
      float* user_vertices0 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0, 0, N*sizeof(float));
      AssertNoError(device);
      
      float* user_vertices1 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices1[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1, 0, N*sizeof(float));
      AssertNoError(device);
      
      rtcDisable(scene,geomID);
      AssertNoError(device);
      rtcCommit(scene);
      AssertNoError(device);
      
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

      return (VerifyApplication::TestReturnValue) passed;
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
    
    InterpolateHairTest (std::string name, int isa, size_t N)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), N(N) {}
    
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
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));

      size_t M = num_interpolation_vertices*N+16; // padds the arrays with some valid data
      
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,RTC_INTERPOLATE);
      AssertNoError(device);
      unsigned int geomID = rtcNewHairGeometry(scene, RTC_GEOMETRY_STATIC, num_interpolation_hairs, num_interpolation_hair_vertices, 1);
      AssertNoError(device);
      
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  interpolation_hair_indices , 0, sizeof(unsigned int));
      AssertNoError(device);
      
      float* vertices0 = new float[M];
      for (size_t i=0; i<M; i++) vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER0, vertices0, 0, N*sizeof(float));
      AssertNoError(device);
      
      /*float* vertices1 = new float[M];
        for (size_t i=0; i<M; i++) vertices1[i] = drand48();
        rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER1, vertices1, 0, N*sizeof(float));
        AssertNoError(device);*/
      
      float* user_vertices0 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices0[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, user_vertices0, 0, N*sizeof(float));
      AssertNoError(device);
      
      float* user_vertices1 = new float[M];
      for (size_t i=0; i<M; i++) user_vertices1[i] = drand48();
      rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER1, user_vertices1, 0, N*sizeof(float));
      AssertNoError(device);
      
      rtcDisable(scene,geomID);
      AssertNoError(device);
      rtcCommit(scene);
      AssertNoError(device);
      
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

      return (VerifyApplication::TestReturnValue) passed;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  struct TriangleHitTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags; 
    RTCGeometryFlags gflags; 

    TriangleHitTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}

    inline Vec3fa uniformSampleTriangle(const Vec3fa &a, const Vec3fa &b, const Vec3fa &c, float &u, float& v)
    {
      const float su = sqrtf(u);
      v *= su;
      u = 1.0f-su;
      return c + u * (a-c) + v * (b-c);
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;
     
      Vec3f vertices[3] = {
        Vec3f(0.0f,0.0f,0.0f),
        Vec3f(1.0f,0.0f,0.0f),
        Vec3f(0.0f,1.0f,0.0f)
      };
      Triangle triangles[1] = {
        Triangle(0,1,2)
      };
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      int geomID = rtcNewTriangleMesh (scene, gflags, 1, 3);
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER, vertices , 0, sizeof(Vec3f));
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER , triangles, 0, sizeof(Triangle));
      rtcCommit (scene);
      AssertNoError(device);

      float u[256], v[256];
      RTCRay rays[256];
      for (size_t i=0; i<256; i++)
      {
        u[i] = drand48(); 
        v[i] = drand48();
        Vec3fa from(0.0f,0.0f,-1.0f);
        Vec3fa to = uniformSampleTriangle(vertices[1],vertices[2],vertices[0],u[i],v[i]);
        rays[i] = makeRay(from,to-from);
      }
      IntersectWithMode(imode,ivariant,scene,rays,256);

      for (size_t i=0; i<256; i++)
      {
        if (rays[i].geomID != 0) return VerifyApplication::FAILED;
        if (ivariant & VARIANT_OCCLUDED) continue;
        if (rays[i].primID != 0) return VerifyApplication::FAILED;
        if (abs(rays[i].u - u[i]) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        if (abs(rays[i].v - v[i]) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        if (abs(rays[i].tfar - 1.0f) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        const Vec3fa org(rays[i].org[0],rays[i].org[1],rays[i].org[2]);
        const Vec3fa dir(rays[i].dir[0],rays[i].dir[1],rays[i].dir[2]);
        const Vec3fa ht  = org + rays[i].tfar*dir;
        const Vec3fa huv = vertices[0] + rays[i].u*(vertices[1]-vertices[0]) + rays[i].v*(vertices[2]-vertices[0]);
        if (reduce_max(abs(ht-huv)) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        const Vec3fa Ng = normalize(Vec3fa(rays[i].Ng[0],rays[i].Ng[1],rays[i].Ng[2])); // FIXME: some geom normals are scaled!!!??
        if (reduce_max(abs(Ng - Vec3fa(0.0f,0.0f,-1.0f))) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
      }
      return VerifyApplication::PASSED;
    }
  };

  struct QuadHitTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags; 
    RTCGeometryFlags gflags; 

    QuadHitTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;
     
      Vec3f vertices[4] = {
        Vec3f(0.0f,0.0f,0.0f),
        Vec3f(1.0f,0.0f,0.0f),
        Vec3f(1.0f,1.0f,0.0f),
        Vec3f(0.0f,1.0f,0.0f)
      };
      int quads[4] = {
        0,1,2,3
      };
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      int geomID = rtcNewQuadMesh (scene, gflags, 1, 4);
      rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER, vertices , 0, sizeof(Vec3f));
      rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER , quads, 0, 4*sizeof(int));
      rtcCommit (scene);
      AssertNoError(device);

      float u[256], v[256];
      RTCRay rays[256];
      for (size_t i=0; i<256; i++)
      {
        u[i] = drand48(); 
        v[i] = drand48();
        Vec3fa from(0.0f,0.0f,-1.0f);
        Vec3fa to = vertices[0] + u[i]*(vertices[1]-vertices[0]) + v[i]*(vertices[3]-vertices[0]);
        rays[i] = makeRay(from,to-from);
      }
      IntersectWithMode(imode,ivariant,scene,rays,256);

      for (size_t i=0; i<256; i++)
      {
        if (rays[i].geomID != 0) return VerifyApplication::FAILED;
        if (ivariant & VARIANT_OCCLUDED) continue;
        if (rays[i].primID != 0) return VerifyApplication::FAILED;
        if (abs(rays[i].u - u[i]) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        if (abs(rays[i].v - v[i]) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        if (abs(rays[i].tfar - 1.0f) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        const Vec3fa org(rays[i].org[0],rays[i].org[1],rays[i].org[2]);
        const Vec3fa dir(rays[i].dir[0],rays[i].dir[1],rays[i].dir[2]);
        const Vec3fa ht  = org + rays[i].tfar*dir;
        const Vec3fa huv = vertices[0] + rays[i].u*(vertices[1]-vertices[0]) + rays[i].v*(vertices[3]-vertices[0]);
        if (reduce_max(abs(ht-huv)) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
        const Vec3fa Ng = normalize(Vec3fa(rays[i].Ng[0],rays[i].Ng[1],rays[i].Ng[2])); // FIXME: some geom normals are scaled!!!??
        if (reduce_max(abs(Ng - Vec3fa(0.0f,0.0f,-1.0f))) > 16.0f*float(ulp)) return VerifyApplication::FAILED;
      }
      return VerifyApplication::PASSED;
    }
  };
  
  struct RayMasksTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags; 
    RTCGeometryFlags gflags; 

    RayMasksTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      ClearBuffers clear_before_return;
      bool passed = true;
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      unsigned geom0 = addSphere(device,scene,gflags,pos0,1.0f,50);
      //unsigned geom1 = addSphere(device,scene,gflags,pos1,1.0f,50);
      unsigned geom1 = addHair  (device,scene,gflags,pos1,1.0f,1.0f,1);
      unsigned geom2 = addSphere(device,scene,gflags,pos2,1.0f,50);
      //unsigned geom3 = addSphere(device,scene,gflags,pos3,1.0f,50);
      unsigned geom3 = addHair  (device,scene,gflags,pos3,1.0f,1.0f,1);
      rtcSetMask(scene,geom0,1);
      rtcSetMask(scene,geom1,2);
      rtcSetMask(scene,geom2,4);
      rtcSetMask(scene,geom3,8);
      rtcCommit (scene);
      AssertNoError(device);
      
      for (size_t i=0; i<16; i++) 
      {
        int mask0 = i;
        int mask1 = i+1;
        int mask2 = i+2;
        int mask3 = i+3;
        int masks[4] = { mask0, mask1, mask2, mask3 };
        RTCRay ray0 = makeRay(pos0+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray0.mask = mask0;
        RTCRay ray1 = makeRay(pos1+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray1.mask = mask1;
        RTCRay ray2 = makeRay(pos2+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray2.mask = mask2;
        RTCRay ray3 = makeRay(pos3+Vec3fa(0,10,0),Vec3fa(0,-1,0)); ray3.mask = mask3;
        RTCRay rays[4] = { ray0, ray1, ray2, ray3 };
        IntersectWithMode(imode,ivariant,scene,rays,4);
        for (size_t i=0; i<4; i++)
          passed &= masks[i] & (1<<i) ? rays[i].geomID != -1 : rays[i].geomID == -1;
      }
      return (VerifyApplication::TestReturnValue) passed;
    }
  };

  struct BackfaceCullingTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;

    BackfaceCullingTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;
       
      /* create triangle that is front facing for a right handed 
         coordinate system if looking along the z direction */
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
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
      AssertNoError(device);

      const size_t numRays = 1000;
      RTCRay rays[numRays];
      RTCRay backfacing = makeRay(Vec3fa(0.25f,0.25f,1),Vec3fa(0,0,-1)); 
      RTCRay frontfacing = makeRay(Vec3fa(0.25f,0.25f,-1),Vec3fa(0,0,1)); 

      bool passed = true;

      for (size_t i=0; i<numRays; i++) {
        if (i%2) rays[i] = backfacing;
        else     rays[i] = frontfacing;
      }
      
      IntersectWithMode(imode,ivariant,scene,rays,numRays);
      
      for (size_t i=0; i<numRays; i++) 
      {
        if (i%2) passed &= rays[i].geomID == -1;
        else     passed &= rays[i].geomID == 0;
      }
      return (VerifyApplication::TestReturnValue) passed;
    }
  };

  struct IntersectionFilterTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    bool subdiv;

    IntersectionFilterTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, bool subdiv, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags), subdiv(subdiv) {}
    
    static void intersectionFilter1(void* userGeomPtr, RTCRay& ray) 
    {
      if ((size_t)userGeomPtr != 123) 
        return;
      
      if (ray.primID & 2)
        ray.geomID = -1;
    }
    
    static void intersectionFilter4(const void* valid_i, void* userGeomPtr, RTCRay4& ray) 
    {
      if ((size_t)userGeomPtr != 123) 
        return;
      
      int* valid = (int*)valid_i;
      for (size_t i=0; i<4; i++)
        if (valid[i] == -1)
          if (ray.primID[i] & 2) 
            ray.geomID[i] = -1;
    }
    
    static void intersectionFilter8(const void* valid_i, void* userGeomPtr, RTCRay8& ray) 
    {
      if ((size_t)userGeomPtr != 123) 
        return;
      
      int* valid = (int*)valid_i;
      for (size_t i=0; i<8; i++)
        if (valid[i] == -1)
          if (ray.primID[i] & 2) 
            ray.geomID[i] = -1;
    }
    
    static void intersectionFilter16(const void* valid_i, void* userGeomPtr, RTCRay16& ray) 
    {
      if ((size_t)userGeomPtr != 123) 
        return;
      
      int* valid = (int*)valid_i;
      for (size_t i=0; i<16; i++)
	if (valid[i] == -1)
	  if (ray.primID[i] & 2) 
	    ray.geomID[i] = -1;
    }
    
    static void intersectionFilterN(int* valid,
                                    void* userGeomPtr,
                                    const RTCIntersectContext* context,
                                    RTCRayN* ray,
                                    const RTCHitN* potentialHit,
                                    const size_t N)
    {
      if ((size_t)userGeomPtr != 123) 
        return;

      for (size_t i=0; i<N; i++)
      {
	if (valid[i] != -1) continue;

        /* reject hit */
        if (RTCHitN_primID(potentialHit,N,i) & 2) {
          valid[i] = 0;
        }

        /* accept hit */
        else {
          RTCRayN_instID(ray,N,i) = RTCHitN_instID(potentialHit,N,i);
          RTCRayN_geomID(ray,N,i) = RTCHitN_geomID(potentialHit,N,i);
          RTCRayN_primID(ray,N,i) = RTCHitN_primID(potentialHit,N,i);
          
          RTCRayN_u(ray,N,i) = RTCHitN_u(potentialHit,N,i);
          RTCRayN_v(ray,N,i) = RTCHitN_v(potentialHit,N,i);
          RTCRayN_tfar(ray,N,i) = RTCHitN_t(potentialHit,N,i);
          
          RTCRayN_Ng_x(ray,N,i) = RTCHitN_Ng_x(potentialHit,N,i);
          RTCRayN_Ng_y(ray,N,i) = RTCHitN_Ng_y(potentialHit,N,i);
          RTCRayN_Ng_z(ray,N,i) = RTCHitN_Ng_z(potentialHit,N,i);
        }
      }
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      Vec3fa p0(-0.75f,-0.25f,-10.0f), dx(4,0,0), dy(0,4,0);
      int geom0 = 0;
      if (subdiv) geom0 = addSubdivPlane (device,scene, gflags, 4, p0, dx, dy);
      else        geom0 = addPlane (device,scene, gflags, 4, p0, dx, dy);
      rtcSetUserData(scene,geom0,(void*)123);
      
      if (imode == MODE_INTERSECT1 ) {
        rtcSetIntersectionFilterFunction(scene,geom0,intersectionFilter1);
        rtcSetOcclusionFilterFunction   (scene,geom0,intersectionFilter1);
      }
      else if (imode == MODE_INTERSECT4 ) {
        rtcSetIntersectionFilterFunction4(scene,geom0,intersectionFilter4);
        rtcSetOcclusionFilterFunction4   (scene,geom0,intersectionFilter4);
      }
      else if (imode == MODE_INTERSECT8 ) {
        rtcSetIntersectionFilterFunction8(scene,geom0,intersectionFilter8);
        rtcSetOcclusionFilterFunction8   (scene,geom0,intersectionFilter8);
      }
      else if (imode == MODE_INTERSECT16) {
        rtcSetIntersectionFilterFunction16(scene,geom0,intersectionFilter16);
        rtcSetOcclusionFilterFunction16   (scene,geom0,intersectionFilter16);
      }
      else {
        rtcSetIntersectionFilterFunctionN (scene,geom0,intersectionFilterN);
        rtcSetOcclusionFilterFunctionN (scene,geom0,intersectionFilterN);
      }
      rtcCommit (scene);
      AssertNoError(device);
      
      RTCRay rays[16];
      for (size_t iy=0; iy<4; iy++) 
      {
        for (size_t ix=0; ix<4; ix++) 
        {
          int primID = iy*4+ix;
          if (!subdiv) primID *= 2;
          rays[iy*4+ix] = makeRay(Vec3fa(float(ix),float(iy),0.0f),Vec3fa(0,0,-1));
        }
      }
      IntersectWithMode(imode,ivariant,scene,rays,16);
      
      bool passed = true;
      for (size_t iy=0; iy<4; iy++) 
      {
        for (size_t ix=0; ix<4; ix++) 
        {
          int primID = iy*4+ix;
          if (!subdiv) primID *= 2;
          RTCRay& ray = rays[iy*4+ix];
          bool ok = (primID & 2) ? (ray.geomID == -1) : (ray.geomID == 0);
          if (!ok) passed = false;
        }
      }
      return (VerifyApplication::TestReturnValue) passed;
    }
  };
    
  struct InactiveRaysTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;

    static const size_t N = 10;
    static const size_t maxStreamSize = 100;
    
    InactiveRaysTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}
   
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      Vec3fa pos = zero;
      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      addSphere(device,scene,RTC_GEOMETRY_STATIC,pos,2.0f,50); // FIXME: use different geometries too
      rtcCommit (scene);
      AssertNoError(device);

      RTCRay invalid_ray;
      memset(&invalid_ray,-1,sizeof(RTCRay));
      invalid_ray.tnear = pos_inf;
      invalid_ray.tfar  = neg_inf;
      invalid_ray = invalid_ray;
      
      size_t numFailures = 0;
      for (size_t i=0; i<size_t(N*state->intensity); i++) 
      {
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
      return (VerifyApplication::TestReturnValue) (numFailures == 0);
    }
  };

  struct WatertightTest : public VerifyApplication::IntersectTest
  {
    ALIGNED_STRUCT;
    RTCSceneFlags sflags;
    std::string model;
    Vec3fa pos;
    static const size_t N = 10;
    static const size_t maxStreamSize = 100;
    
    WatertightTest (std::string name, int isa, RTCSceneFlags sflags, IntersectMode imode, std::string model, const Vec3fa& pos)
      : VerifyApplication::IntersectTest(name,isa,imode,VARIANT_INTERSECT,VerifyApplication::PASS), sflags(sflags), model(model), pos(pos) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      ClearBuffers clear_before_return;
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      if      (model == "sphere") addSphere(device,scene,RTC_GEOMETRY_STATIC,pos,2.0f,500);
      else if (model == "plane" ) addPlane(device,scene,RTC_GEOMETRY_STATIC,500,Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f));
      bool plane = model == "plane";
      rtcCommit (scene);
      AssertNoError(device);
      
      size_t numTests = 0;
      size_t numFailures = 0;
      for (auto ivariant : state->intersectVariants)
      for (size_t i=0; i<size_t(N*state->intensity); i++) 
      {
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
      AssertNoError(device);

      double failRate = double(numFailures) / double(numTests);
      bool failed = failRate > 0.00002;
      //printf(" (%f%%)", 100.0f*failRate); fflush(stdout);
      return (VerifyApplication::TestReturnValue)(!failed);
    }
  };

  struct NaNTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    
    NaNTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags)  {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      ClearBuffers clear_before_return;
      const size_t numRays = 1000;
      RTCRay rays[numRays];
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      addSphere(device,scene,gflags,zero,2.0f,100);
      addHair  (device,scene,gflags,zero,1.0f,1.0f,100);
      rtcCommit (scene);
      size_t numFailures = 0;

      double c0 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);
    
      double c1 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org+Vec3fa(nan),dir); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);

      double c2 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org+Vec3fa(nan),dir+Vec3fa(nan)); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);

      double c3 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir,nan,nan); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);

      double c4 = getSeconds();
      double d1 = c1-c0;
      double d2 = c2-c1;
      double d3 = c3-c2;
      double d4 = c4-c3;
      
      bool ok = (d2 < 2.5*d1) && (d3 < 2.5*d1) && (d4 < 2.5*d1);
      float f = max(d2/d1,d3/d1,d4/d1);
      //printf(" (%3.2fx)",f); fflush(stdout);
      return (VerifyApplication::TestReturnValue) ok;
    }
  };
    
  struct InfTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    
    InfTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::PASS), sflags(sflags), gflags(gflags) {}
   
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      ClearBuffers clear_before_return;
      const size_t numRays = 1000;
      RTCRay rays[numRays];
      RTCSceneRef scene = rtcDeviceNewScene(device,sflags,to_aflags(imode));
      addSphere(device,scene,gflags,zero,2.0f,100);
      addHair  (device,scene,gflags,zero,1.0f,1.0f,100);
      rtcCommit (scene);
      AssertNoError(device);

      size_t numFailures = 0;
      double c0 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);

      double c1 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org+Vec3fa(inf),dir); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);

      double c2 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir+Vec3fa(inf)); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);

      double c3 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org+Vec3fa(inf),dir+Vec3fa(inf)); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);

      double c4 = getSeconds();
      for (size_t i=0; i<numRays; i++) {
        Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
        rays[i] = makeRay(org,dir,-0.0f,inf); 
      }
      IntersectWithMode(imode,ivariant,scene,rays,numRays);

      double c5 = getSeconds();      
      double d1 = c1-c0;
      double d2 = c2-c1;
      double d3 = c3-c2;
      double d4 = c4-c3;
      double d5 = c5-c4;
      
      bool ok = (d2 < 2.5*d1) && (d3 < 2.5*d1) && (d4 < 2.5*d1) && (d5 < 2.5*d1);
      float f = max(d2/d1,d3/d1,d4/d1,d5/d1);
      //printf(" (%3.2fx)",f); fflush(stdout);
      return (VerifyApplication::TestReturnValue) ok;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  void shootRandomRays (std::vector<IntersectMode>& intersectModes, std::vector<IntersectVariant>& intersectVariants, const RTCSceneRef& scene)
  {
    const size_t numRays = 100;
    for (auto imode : intersectModes)
    {
      for (auto ivariant : intersectVariants)
      {
        RTCRay rays[numRays];
        for (size_t i=0; i<numRays; i++) {
          Vec3fa org(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
          Vec3fa dir(2.0f*drand48()-1.0f,2.0f*drand48()-1.0f,2.0f*drand48()-1.0f);
          rays[i] = makeRay(org,dir); 
        }
        IntersectWithMode(imode,ivariant,scene,rays,numRays);
      }
    }
  }

  static bool build_join_test = false;

  struct RegressionTask
  {
    RegressionTask (size_t sceneIndex, size_t sceneCount, size_t threadCount)
      : sceneIndex(sceneIndex), sceneCount(sceneCount), scene(nullptr), numActiveThreads(0) { barrier.init(threadCount); }

    size_t sceneIndex;
    size_t sceneCount;
    VerifyApplication* state;
    RTCSceneRef scene;
    BarrierSys barrier;
    volatile size_t numActiveThreads;
  };

  struct ThreadRegressionTask
  {
    ThreadRegressionTask (size_t threadIndex, size_t threadCount,
                          VerifyApplication* state, RTCDeviceRef& device, std::vector<IntersectMode>& intersectModes, RegressionTask* task)
      : threadIndex(threadIndex), threadCount(threadCount), state(state), device(device), intersectModes(intersectModes), task(task) {}

    size_t threadIndex;
    size_t threadCount;
    VerifyApplication* state;
    RTCDeviceRef& device;
    std::vector<IntersectMode>& intersectModes;
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
            shootRandomRays(thread->intersectModes,thread->state->intersectVariants,task->scene);
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
      task->scene = rtcDeviceNewScene(thread->device,sflag,aflags_all);
      CountErrors(thread->device);
      if (g_enable_build_cancel) rtcSetProgressMonitorFunction(task->scene,monitorProgressFunction,nullptr);
      avector<Sphere*> spheres;
      
      for (size_t j=0; j<10; j++) 
      {
        Vec3fa pos = 100.0f*Vec3fa(drand48(),drand48(),drand48());
	int type = random<int>()%6;
        switch (random<int>()%16) {
        case 0: pos = Vec3fa(nan); break;
        case 1: pos = Vec3fa(inf); break;
        case 2: pos = Vec3fa(1E30f); break;
        default: break;
        };
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
	  addUserGeometryEmpty(thread->device,task->scene,sphere); break;
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
          shootRandomRays(thread->intersectModes,thread->state->intersectVariants,task->scene);
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
            shootRandomRays(thread->intersectModes,thread->state->intersectVariants,task->scene);
          }
	}
	task->barrier.wait();
      }
      delete thread; thread = nullptr;
      return;
    }
    task->scene = rtcDeviceNewScene(thread->device,RTC_SCENE_DYNAMIC,aflags_all);
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
          switch (random<int>()%16) {
          case 0: pos = Vec3fa(nan); break;
          case 1: pos = Vec3fa(inf); break;
          case 2: pos = Vec3fa(1E30f); break;
          default: break;
          };
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
          case 9: spheres[index] = Sphere(pos,2.0f); geom[index] = addUserGeometryEmpty(thread->device,task->scene,&spheres[index]); break;
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
          shootRandomRays(thread->intersectModes,thread->state->intersectVariants,task->scene);

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
    std::vector<IntersectMode> intersectModes;
    
    IntensiveRegressionTest (std::string name, int isa, thread_func func, int mode)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), func(func), mode(mode) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));

      /* only test supported intersect modes */
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

      errorCounter = 0;
      size_t sceneIndex = 0;
      while (sceneIndex < size_t(30*state->intensity)) 
      {
        if (mode)
        {
          ClearBuffers clear_before_return;
          build_join_test = (mode == 2);
          size_t numThreads = getNumberOfLogicalThreads();
          
          std::vector<RegressionTask*> tasks;
          while (numThreads) 
          {
            size_t N = max(size_t(1),random<int>()%numThreads); numThreads -= N;
            RegressionTask* task = new RegressionTask(sceneIndex++,5,N);
            tasks.push_back(task);
            
            for (size_t i=0; i<N; i++) 
              g_threads.push_back(createThread(func,new ThreadRegressionTask(i,N,state,device,intersectModes,task),DEFAULT_STACK_SIZE,numThreads+i));
          }
          
          for (size_t i=0; i<g_threads.size(); i++)
            join(g_threads[i]);
          for (size_t i=0; i<tasks.size(); i++)
            delete tasks[i];
          
          g_threads.clear();
          clearBuffers();
        }
        else
        {
          ClearBuffers clear_before_return;
          RegressionTask task(sceneIndex++,5,0);
          func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task));
        }	
      }
      return (VerifyApplication::TestReturnValue) (errorCounter == 0);
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
    std::vector<IntersectMode> intersectModes;
    
    MemoryMonitorTest (std::string name, int isa, thread_func func)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS), func(func) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      
      /* only test supported intersect modes */
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
      
      g_enable_build_cancel = true;
      rtcDeviceSetMemoryMonitorFunction(device,monitorMemoryFunction);
      
      size_t sceneIndex = 0;
      while (sceneIndex < size_t(30*state->intensity)) 
      {
        ClearBuffers clear_before_return;
        errorCounter = 0;
        monitorMemoryBreak = -1;
        monitorMemoryBytesUsed = 0;
        monitorMemoryInvokations = 0;
        monitorProgressBreak = -1;
        monitorProgressInvokations = 0;
        RegressionTask task1(sceneIndex,1,0);
        func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task1));
        if (monitorMemoryBytesUsed) {
          rtcDeviceSetMemoryMonitorFunction(device,nullptr);
          //rtcDeviceSetProgressMonitorFunction(device,nullptr);
          return VerifyApplication::FAILED;
        }
        monitorMemoryBreak = monitorMemoryInvokations * drand48();
        monitorMemoryBytesUsed = 0;
        monitorMemoryInvokations = 0;
        monitorProgressBreak = monitorProgressInvokations * 2.0f * drand48();
        monitorProgressInvokations = 0;
        RegressionTask task2(sceneIndex,1,0);
        func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task2));
        if (monitorMemoryBytesUsed) { // || (monitorMemoryInvokations != 0 && errorCounter != 1)) { // FIXME: test that rtcCommit has returned with error code
          rtcDeviceSetMemoryMonitorFunction(device,nullptr);
          //rtcDeviceSetProgressMonitorFunction(device,nullptr);
          return VerifyApplication::FAILED;
        }
        sceneIndex++;
      }
      g_enable_build_cancel = false;
      rtcDeviceSetMemoryMonitorFunction(device,nullptr);
      return VerifyApplication::PASSED;
    }
  };

  struct GarbageGeometryTest : public VerifyApplication::Test // FIXME: move test to front, this is a build test
  {
    GarbageGeometryTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::PASS) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));

      for (size_t i=0; i<size_t(1000*state->intensity); i++) 
      {
        ClearBuffers clear_before_return;
        srand(i*23565);
        if (i%20 == 0) std::cout << "." << std::flush;
        
        RTCSceneFlags sflag = getSceneFlag(i); 
        RTCSceneRef scene = rtcDeviceNewScene(device,sflag,aflags);
        AssertNoError(device);
        
        for (size_t j=0; j<20; j++) 
        {
          size_t numTriangles = random<int>()%256;
          switch (random<int>()%4) {
          case 0: addGarbageTriangles(device,scene,RTC_GEOMETRY_STATIC,numTriangles,false); break;
          case 1: addGarbageTriangles(device,scene,RTC_GEOMETRY_STATIC,numTriangles,true); break;
          case 2: addGarbageHair     (device,scene,RTC_GEOMETRY_STATIC,numTriangles,false); break;
          case 3: addGarbageHair     (device,scene,RTC_GEOMETRY_STATIC,numTriangles,true); break;
          }
          AssertNoError(device);
        }
        
        rtcCommit(scene);
        AssertNoError(device);
      }
      return VerifyApplication::PASSED;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  
  VerifyApplication::VerifyApplication ()
    : Application(Application::FEATURE_RTCORE), intensity(1.0f), numFailedTests(0), user_specified_tests(false), use_groups(true)
  {
    /* create list of all ISAs to test */
    if (hasISA(SSE2)) isas.push_back(SSE2);
    if (hasISA(SSE42)) isas.push_back(SSE42);
    if (hasISA(AVX)) isas.push_back(AVX);
    if (hasISA(AVX2)) isas.push_back(AVX2);
    if (hasISA(AVX512KNL)) isas.push_back(AVX512KNL);
    
    /* create list of all intersect modes to test */
    intersectModes.push_back(MODE_INTERSECT1);
    intersectModes.push_back(MODE_INTERSECT4);
    intersectModes.push_back(MODE_INTERSECT8);
    intersectModes.push_back(MODE_INTERSECT16);
    intersectModes.push_back(MODE_INTERSECT1M);
    intersectModes.push_back(MODE_INTERSECTNM1);
    intersectModes.push_back(MODE_INTERSECTNM3);
    intersectModes.push_back(MODE_INTERSECTNM4);
    intersectModes.push_back(MODE_INTERSECTNM8);
    intersectModes.push_back(MODE_INTERSECTNM16);
    intersectModes.push_back(MODE_INTERSECTNp);
    
    /* create a list of all intersect variants for each intersect mode */
    intersectVariants.push_back(VARIANT_INTERSECT_COHERENT);
    intersectVariants.push_back(VARIANT_OCCLUDED_COHERENT);
    intersectVariants.push_back(VARIANT_INTERSECT_INCOHERENT);
    intersectVariants.push_back(VARIANT_OCCLUDED_INCOHERENT);

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

    sceneFlagsDynamic.push_back(RTC_SCENE_DYNAMIC);
    sceneFlagsDynamic.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_ROBUST);
    sceneFlagsDynamic.push_back(RTC_SCENE_DYNAMIC | RTC_SCENE_COMPACT);

    /**************************************************************************/
    /*                      Smaller API Tests                                 */
    /**************************************************************************/

    RTCDeviceRef device = rtcNewDevice(rtcore.c_str());
    error_handler(rtcDeviceGetError(device));

    addTest(new InitExitTest("init_exit"));

    /* add Embree internal tests */
    for (size_t i=2000000; i<3000000; i++) {
      const char* testName = (const char*) rtcDeviceGetParameter1i(device,(RTCParameter)i);
      if (testName == nullptr) break;
      addTest(new EmbreeInternalTest(testName,i-2000000));
    }

    for (auto isa : isas)
    {
      addTest(new MultipleDevicesTest("multiple_devices",isa));

      beginTestGroup("flags_"+stringOfISA(isa));
      addTest(new FlagsTest("flags_"+stringOfISA(isa)+"_static_static"     ,isa,VerifyApplication::PASS, RTC_SCENE_STATIC, RTC_GEOMETRY_STATIC));
      addTest(new FlagsTest("flags_"+stringOfISA(isa)+"_static_deformable" ,isa,VerifyApplication::FAIL, RTC_SCENE_STATIC, RTC_GEOMETRY_DEFORMABLE));
      addTest(new FlagsTest("flags_"+stringOfISA(isa)+"_static_dynamic"    ,isa,VerifyApplication::FAIL, RTC_SCENE_STATIC, RTC_GEOMETRY_DYNAMIC));
      addTest(new FlagsTest("flags_"+stringOfISA(isa)+"_dynamic_static"    ,isa,VerifyApplication::PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC));
      addTest(new FlagsTest("flags_"+stringOfISA(isa)+"_dynamic_deformable",isa,VerifyApplication::PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DEFORMABLE));
      addTest(new FlagsTest("flags_"+stringOfISA(isa)+"_dynamic_dynamic"   ,isa,VerifyApplication::PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC));    
      endTestGroup();

      addTest(new UnmappedBeforeCommitTest("unmapped_before_commit_"+stringOfISA(isa),isa));
      addTest(new GetBoundsTest("get_bounds_"+stringOfISA(isa),isa));
      addTest(new GetUserDataTest("get_user_data_"+stringOfISA(isa),isa));

      addTest(new BufferStrideTest("buffer_stride_"+stringOfISA(isa),isa));
      
      /**************************************************************************/
      /*                        Builder Tests                                   */
      /**************************************************************************/
      
      beginTestGroup("empty_scene_"+stringOfISA(isa));
      for (auto sflags : sceneFlags) 
        addTest(new EmptySceneTest("empty_scene_"+to_string(isa,sflags),isa,sflags));
      endTestGroup();
      
      beginTestGroup("empty_geometry_"+stringOfISA(isa));
      for (auto sflags : sceneFlags) 
        addTest(new EmptyGeometryTest("empty_geometry_"+to_string(isa,sflags),isa,sflags,RTC_GEOMETRY_STATIC));
      endTestGroup();
      
      beginTestGroup("build_"+stringOfISA(isa));
      for (auto sflags : sceneFlags) 
        addTest(new BuildTest("build_"+to_string(isa,sflags),isa,sflags,RTC_GEOMETRY_STATIC));
      endTestGroup();
      
      addTest(new OverlappingTrianglesTest("overlapping_triangles_"+stringOfISA(isa),isa,100000));
      addTest(new OverlappingHairTest("overlapping_hair_"+stringOfISA(isa),isa,100000));

      beginTestGroup("new_delete_geometry_"+stringOfISA(isa));
      for (auto sflags : sceneFlagsDynamic) 
        addTest(new NewDeleteGeometryTest("new_delete_geometry_"+to_string(isa,sflags),isa,sflags));
      endTestGroup();
      
      beginTestGroup("enable_disable_geometry_"+stringOfISA(isa));
      for (auto sflags : sceneFlagsDynamic) 
        addTest(new EnableDisableGeometryTest("enable_disable_geometry_"+to_string(isa,sflags),isa,sflags));
      endTestGroup();
      
      beginTestGroup("update_"+stringOfISA(isa));
      for (auto sflags : sceneFlagsDynamic) {
        for (auto imode : intersectModes) {
          for (auto ivariant : intersectVariants) {
            addTest(new UpdateTest("update_deformable_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_DEFORMABLE,imode,ivariant));
            addTest(new UpdateTest("update_dynamic_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_DYNAMIC,imode,ivariant));
          }
        }
      }
      endTestGroup();
      
      /**************************************************************************/
      /*                     Interpolation Tests                                */
      /**************************************************************************/
      
      beginTestGroup("interpolate_subdiv_"+stringOfISA(isa));
      int subdivTests[] = { 4,5,8,11,12,15 };
      for (auto s : subdivTests)
        addTest(new InterpolateSubdivTest("interpolate_subdiv_"+stringOfISA(isa)+"_"+std::to_string(long(s)),isa,s));
      endTestGroup();
      
      beginTestGroup("interpolate_triangles_"+stringOfISA(isa));
      for (auto s : subdivTests) 
        addTest(new InterpolateTrianglesTest("interpolate_triangles_"+stringOfISA(isa)+"_"+std::to_string(long(s)),isa,s));
      endTestGroup();

      beginTestGroup("interpolate_hair_"+stringOfISA(isa));
      for (auto s : subdivTests) 
        addTest(new InterpolateHairTest("interpolate_hair_"+stringOfISA(isa)+"_"+std::to_string(long(s)),isa,s));
      endTestGroup();
      
      /**************************************************************************/
      /*                      Intersection Tests                                */
      /**************************************************************************/

      beginTestGroup("triangle_hit_"+stringOfISA(isa));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            addTest(new TriangleHitTest("triangle_hit_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      endTestGroup();

      beginTestGroup("quad_hit_"+stringOfISA(isa));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            addTest(new QuadHitTest("quad_hit_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      endTestGroup();

      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_RAY_MASK)) 
      {
        beginTestGroup("ray_masks_"+stringOfISA(isa));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              addTest(new RayMasksTest("ray_masks_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        endTestGroup();
      }
      
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_BACKFACE_CULLING)) 
      {
        beginTestGroup("backface_culling_"+stringOfISA(isa));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              addTest(new BackfaceCullingTest("backface_culling_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        endTestGroup();
      }
      
      beginTestGroup("intersection_filter_"+stringOfISA(isa));
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECTION_FILTER)) 
      {
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              addTest(new IntersectionFilterTest("intersection_filter_tris_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,false,imode,ivariant));
        
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              addTest(new IntersectionFilterTest("intersection_filter_subdiv_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,true,imode,ivariant));
      }
      endTestGroup();
      
      beginTestGroup("inactive_rays_"+stringOfISA(isa));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            if (imode != MODE_INTERSECT1) // INTERSECT1 does not support disabled rays
              addTest(new InactiveRaysTest("inactive_rays_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      endTestGroup();
      
      beginTestGroup("watertight_"+stringOfISA(isa));
      std::string watertightModels [] = {"sphere", "plane"};
      const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
      for (auto sflags : sceneFlagsRobust) 
        for (auto imode : intersectModes) 
          for (std::string model : watertightModels) 
            addTest(new WatertightTest("watertight_"+to_string(isa,sflags,imode)+"_"+model,isa,sflags,imode,model,watertight_pos));
      endTestGroup();
      
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_IGNORE_INVALID_RAYS))
      {
        beginTestGroup("nan_test_"+stringOfISA(isa));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              addTest(new NaNTest("nan_test_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        endTestGroup();
        
        beginTestGroup("inf_test_"+stringOfISA(isa));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              addTest(new InfTest("inf_test_"+to_string(isa,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        endTestGroup();
      }
    
      /**************************************************************************/
      /*                  Randomized Stress Testing                             */
      /**************************************************************************/
      
      addTest(new IntensiveRegressionTest("regression_static_"+stringOfISA(isa),isa,rtcore_regression_static_thread,0));
      addTest(new IntensiveRegressionTest("regression_dynamic_"+stringOfISA(isa),isa,rtcore_regression_dynamic_thread,0));
      
      addTest(new IntensiveRegressionTest("regression_static_user_threads_"+stringOfISA(isa), isa,rtcore_regression_static_thread,1));
      addTest(new IntensiveRegressionTest("regression_dynamic_user_threads_"+stringOfISA(isa),isa,rtcore_regression_dynamic_thread,1));
      
      addTest(new IntensiveRegressionTest("regression_static_build_join_"+stringOfISA(isa), isa,rtcore_regression_static_thread,2));
      addTest(new IntensiveRegressionTest("regression_dynamic_build_join_"+stringOfISA(isa),isa,rtcore_regression_dynamic_thread,2));
      
      addTest(new MemoryMonitorTest("regression_static_memory_monitor_"+stringOfISA(isa), isa,rtcore_regression_static_thread));
      addTest(new MemoryMonitorTest("regression_dynamic_memory_monitor_"+stringOfISA(isa),isa,rtcore_regression_dynamic_thread));
      
      addTest(new GarbageGeometryTest("regression_garbage_geom_"+stringOfISA(isa),isa));
    }
    
    /* register all command line options*/
    std::string run_docu = "--run <regexpr>: Runs all tests whose name match the regular expression. Supported tests are:";
    for (auto test : tests) run_docu += "\n  " + test->name;
    registerOption("run", [this] (Ref<ParseStream> cin, const FileName& path) {

        std::string r = cin->getString();

        if (!user_specified_tests) 
          for (auto test : tests) 
            test->enabled = false;

        user_specified_tests = true;
#if defined(__MACOSX__) && defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1600) // works around __ZTVNSt3__123__match_any_but_newlineIcEE link error
        for (auto test : tests) 
          if (test->name == r) test->enabled = true;
#else
        std::smatch match;
        std::regex regexpr(r);
        for (auto test : tests) 
          if (std::regex_match(test->name, match, regexpr)) test->enabled = true;
#endif
      }, run_docu);

    registerOption("skip", [this] (Ref<ParseStream> cin, const FileName& path) {

        std::string r = cin->getString();
        
        if (!user_specified_tests) 
          for (auto test : tests) 
            test->enabled = true;

        user_specified_tests = true;
#if defined(__MACOSX__) && defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1600) // works around __ZTVNSt3__123__match_any_but_newlineIcEE link error
        for (auto test : tests)
          if (test->name == r) test->enabled = false;
#else
        std::smatch match;
        std::regex regexpr(r);
        for (auto test : tests)
          if (std::regex_match(test->name, match, regexpr)) test->enabled = false;
#endif
      }, "--skip <regexpr>: Skips all tests whose name matches the regular expression.");
    
    registerOption("no-groups", [this] (Ref<ParseStream> cin, const FileName& path) {
        use_groups = false;
      }, "--no-groups: ignore test groups");

    registerOption("intensity", [this] (Ref<ParseStream> cin, const FileName& path) {
        intensity = cin->getFloat();
      }, "--intensity <float>: intensity of testing to perform");
  }

  int VerifyApplication::main(int argc, char** argv) try
  {
    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    
    /* parse command line options */
    parseCommandLine(argc,argv);

    /* enable all tests if user did not specify any tests */
    if (!user_specified_tests) 
      for (auto test : tests) 
        test->enabled = true;

    /* run all enabled tests */
    for (size_t i=0; i<tests.size(); i++) 
    {
      if (use_groups && tests[i]->ty == GROUP_BEGIN) {
        if (tests[i]->isEnabled())
          runTestGroup(i);
      }
      else {
        if (tests[i]->isEnabled() && tests[i]->ty != GROUP_BEGIN && tests[i]->ty != GROUP_END)
          runTest(tests[i],false);
      }
    }

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
  
  bool VerifyApplication::runTest(Ref<Test> test, bool silent)
  {
    if (!test->isEnabled())
      return true;

    if (!silent)
      std::cout << std::setw(60) << test->name << " ..." << std::flush;
    
    TestReturnValue v = SKIPPED;
    try {
      v = test->run(this);
    } catch (...) {
      v = FAILED;
    }
    TestReturnValue ev = test->ty == PASS ? PASSED : FAILED;
    bool passed = v == ev || v == SKIPPED;

    if (silent) {
      if (v != SKIPPED) {
        if (passed) std::cout << GREEN("+") << std::flush;
        else        std::cout << RED  ("-") << std::flush;
      }
    } else {
      if      (v == SKIPPED) std::cout << GREEN(" [SKIPPED]") << std::endl << std::flush;
      else if (passed      ) std::cout << GREEN(" [PASSED]" ) << std::endl << std::flush;
      else                   std::cout << RED  (" [FAILED]" ) << std::endl << std::flush;
    }
    numFailedTests += !passed;
    return passed;
  }

  void VerifyApplication::runTestGroup(size_t& id)
  {
    bool ok = true;
    Ref<Test> test = tests[id];
    std::cout << std::setw(50) << test->name << " " << std::flush;

    id++;
    for (; id<tests.size() && tests[id]->ty != GROUP_END; id++)
      ok &= runTest(tests[id],true);

    if (ok) std::cout << GREEN(" [PASSED]") << std::endl << std::flush;
    else    std::cout << RED  (" [FAILED]") << std::endl << std::flush;
  }
}

int main(int argc, char** argv)
{
  embree::VerifyApplication app;
  return app.main(argc,argv);
}
