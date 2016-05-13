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
#include "../kernels/algorithms/parallel_for.h"
#include <regex>
#include <stack>

#define DEFAULT_STACK_SIZE 4*1024*1024

#if defined(__WIN32__)
#  define GREEN(x) x
#  define YELLOW(x) x
#  define RED(x) x
#else
#  define GREEN(x) "\033[32m" x "\033[0m"
#  define YELLOW(x) "\033[33m" x "\033[0m"
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
  bool hasISA(const int isa) 
  {
    int cpu_features = getCPUFeatures();
    return (cpu_features & isa) == isa;
  }

  bool regex_match(std::string str, std::string regex)
  {
#if defined(__MACOSX__) && defined(__INTEL_COMPILER) && (__INTEL_COMPILER < 1600) // works around __ZTVNSt3__123__match_any_but_newlineIcEE link error
    return str == regex; 
#else
    std::smatch match; std::regex regexpr(regex);
    return std::regex_match(str, match, regexpr);
#endif
  }

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

  typedef SceneGraph::TriangleMeshNode::Triangle Triangle;

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

  struct VerifyScene : public RefCount
  {
    VerifyScene (const RTCDeviceRef& device, RTCSceneFlags sflags, RTCAlgorithmFlags aflags)
      : device(device), scene(rtcDeviceNewScene(device,sflags,aflags)) {}

    operator RTCScene() const {
      return scene;
    }
    
    unsigned addGeometry(const RTCGeometryFlags gflag, const Ref<SceneGraph::Node>& node, bool mblur = false)
    {
      mutex.lock();
      nodes.push_back(node);
      mutex.unlock();
      
      if (Ref<SceneGraph::TriangleMeshNode> mesh = node.dynamicCast<SceneGraph::TriangleMeshNode>()) 
      {
        int numTimeSteps = mblur ? 2 : 1;
        unsigned geomID = rtcNewTriangleMesh (scene, gflag, mesh->triangles.size(), mesh->v.size(), numTimeSteps);
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER ,mesh->triangles.data(),0,sizeof(SceneGraph::TriangleMeshNode::Triangle));
        if (mesh->v .size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER0,mesh->v .data(),0,sizeof(SceneGraph::TriangleMeshNode::Vertex));
        if (mesh->v2.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER1,mesh->v2.data(),0,sizeof(SceneGraph::TriangleMeshNode::Vertex));
        AssertNoError(device);
        return geomID;
      }
      else if (Ref<SceneGraph::QuadMeshNode> mesh = node.dynamicCast<SceneGraph::QuadMeshNode>())
      {
        int numTimeSteps = mblur ? 2 : 1;
        unsigned geomID = rtcNewQuadMesh (scene, gflag, mesh->quads.size(), mesh->v.size(), numTimeSteps);
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER ,mesh->quads.data(),0,sizeof(SceneGraph::QuadMeshNode::Quad  ));
        if (mesh->v .size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER0,mesh->v .data(),0,sizeof(SceneGraph::QuadMeshNode::Vertex));
        if (mesh->v2.size()) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER1,mesh->v2.data(),0,sizeof(SceneGraph::QuadMeshNode::Vertex));
        AssertNoError(device);
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
        AssertNoError(device);
        return geomID;
      }
      else if (Ref<SceneGraph::HairSetNode> mesh = node.dynamicCast<SceneGraph::HairSetNode>())
      {
        unsigned int geomID = rtcNewHairGeometry (scene, gflag, mesh->hairs.size(), mesh->v.size(), mblur ? 2 : 1);
        rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,mesh->v.data(),0,sizeof(SceneGraph::HairSetNode::Vertex));
        if (mblur) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER1,mesh->v2.data(),0,sizeof(SceneGraph::HairSetNode::Vertex));
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,mesh->hairs.data(),0,sizeof(SceneGraph::HairSetNode::Hair));
        AssertNoError(device);
        return geomID;
      } 
      else if (Ref<SceneGraph::LineSegmentsNode> mesh = node.dynamicCast<SceneGraph::LineSegmentsNode>())
      {
        unsigned int geomID = rtcNewLineSegments (scene, gflag, mesh->indices.size(), mesh->v.size(), mblur ? 2 : 1);
        rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,mesh->v.data(),0,sizeof(SceneGraph::LineSegmentsNode::Vertex));
        if (mblur) rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER1,mesh->v2.data(),0,sizeof(SceneGraph::LineSegmentsNode::Vertex));
        rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,mesh->indices.data(),0,sizeof(int));
        AssertNoError(device);
        return geomID;
      }
      else {
        THROW_RUNTIME_ERROR("unknown node type");
      }
      return 0;
    }
    
    unsigned addPlane (const RTCGeometryFlags gflag, size_t num, const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy) {
      return addGeometry(gflag,SceneGraph::createTrianglePlane(p0,dx,dy,num,num));
    }
    
    unsigned addSubdivPlane (RTCGeometryFlags gflag, size_t num, const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy) {
      return addGeometry(gflag,SceneGraph::createSubdivPlane(p0,dx,dy,num,num,2.0f));
    }
    
    unsigned addSphere (RTCGeometryFlags gflag, const Vec3fa& pos, const float r, size_t numPhi, size_t maxTriangles = -1, float motion = 0.0f)
    {
      bool mblur = motion != 0.0f;
      Ref<SceneGraph::Node> node = SceneGraph::createTriangleSphere(pos,r,numPhi);
      if (mblur) SceneGraph::set_motion_vector(node,Vec3fa(motion));
      if (maxTriangles !=   -1) SceneGraph::resize_randomly(node,maxTriangles);
      return addGeometry(gflag,node,mblur);
    }

    unsigned int addSubdivSphere (RTCGeometryFlags gflag, const Vec3fa& pos, const float r, size_t numPhi, float level, size_t maxFaces = -1, float motion = 0.0f)
    {
      bool mblur = motion != 0.0f;
      Ref<SceneGraph::Node> node = SceneGraph::createSubdivSphere(pos,r,numPhi,level);
      if (mblur) SceneGraph::set_motion_vector(node,Vec3fa(motion));
      if (maxFaces != -1) SceneGraph::resize_randomly(node,maxFaces);
      addRandomSubdivFeatures(node.dynamicCast<SceneGraph::SubdivMeshNode>(),10,10,0);
      return addGeometry(gflag,node,mblur);
    }
    
    unsigned addSphereHair (RTCGeometryFlags gflag, const Vec3fa& center, const float radius, float motion = 0.0f)
    {
      bool mblur = motion != 0.0f;
      Ref<SceneGraph::Node> node = SceneGraph::createSphereShapedHair(center,radius);
      if (mblur) SceneGraph::set_motion_vector(node,Vec3fa(motion));
      return addGeometry(gflag,node,mblur);
    }
    
    unsigned addHair (RTCGeometryFlags gflag, const Vec3fa& pos, const float scale, const float r, size_t numHairs = 1, float motion = 0.0f)
    {
      bool mblur = motion != 0.0f;
      Ref<SceneGraph::Node> node = SceneGraph::createHairyPlane(pos,Vec3fa(1,0,0),Vec3fa(0,0,1),scale,r,numHairs,true);
      if (mblur) SceneGraph::set_motion_vector(node,Vec3fa(motion));
      return addGeometry(gflag,node,mblur);
    }

    unsigned addUserGeometryEmpty (Sphere* sphere)
    {
      BBox3fa bounds = sphere->bounds(); 
      unsigned geom = rtcNewUserGeometry (scene,1);
      rtcSetBoundsFunction(scene,geom,(RTCBoundsFunc)BoundsFunc);
      rtcSetUserData(scene,geom,sphere);
      rtcSetIntersectFunctionN(scene,geom,IntersectFuncN);
      rtcSetOccludedFunctionN(scene,geom,IntersectFuncN);
      return geom;
    }

  public:
    MutexSys mutex;
    const RTCDeviceRef& device;
    RTCSceneRef scene;
    std::vector<Ref<SceneGraph::Node>> nodes;
  };

  VerifyApplication::TestReturnValue VerifyApplication::Test::execute(VerifyApplication* state, bool silent)
  {
    if (!isEnabled())
      return SKIPPED;
    
    if (!silent) 
      std::cout << std::setw(60) << name << " ..." << std::flush;
      
    TestReturnValue v = SKIPPED;
    try {
      v = run(state,silent);
    } catch (...) {
      v = FAILED;
    }
    TestReturnValue ev = ty != TEST_SHOULD_FAIL ? PASSED : FAILED;
    bool passed = v == ev || v == SKIPPED;

    if (silent) {
      if (v != SKIPPED) {
        if      (passed       ) std::cout << GREEN ("+") << std::flush;
        else if (ignoreFailure) std::cout << YELLOW("-") << std::flush;
        else                    std::cout << RED   ("-") << std::flush;
      }
    } 
    else
    {
      if      (v == SKIPPED ) std::cout << GREEN (" [SKIPPED]") << std::endl << std::flush;
      else if (passed       ) std::cout << GREEN (" [PASSED]" ) << std::endl << std::flush;
      else if (ignoreFailure) std::cout << YELLOW(" [FAILED]" ) << std::endl << std::flush;
      else                    std::cout << RED   (" [FAILED]" ) << std::endl << std::flush;
    }

    /* do ignore failures for some specific tests */
    if (ignoreFailure) 
      passed = true;

    state->numFailedTests += !passed;
    return passed ? PASSED : FAILED;
  }

  VerifyApplication::TestReturnValue VerifyApplication::TestGroup::execute(VerifyApplication* state, bool silent_in)
  {
    if (!isEnabled())
      return SKIPPED;

    bool leaftest = state->flatten && !silent_in && silent;
    bool nextsilent = silent_in || (state->flatten && silent);
    if (leaftest) 
      std::cout << std::setw(60) << name << " ..." << std::flush;
    
    std::atomic<int> passed(true);
    if (state->parallel && leaftest) 
    {
      parallel_for(tests.size(),[&] (size_t i) {
          passed.fetch_and(tests[i]->execute(state,nextsilent) != FAILED);
        });
    } 
    else {
      for (auto test : tests)
        passed.fetch_and(test->execute(state,nextsilent) != FAILED);
    }

    if (leaftest) {
      if (passed) std::cout << GREEN(" [PASSED]" ) << std::endl << std::flush;
      else        std::cout << RED  (" [FAILED]" ) << std::endl << std::flush;
    }

    return passed ? PASSED : FAILED;
  }

  RTCAlgorithmFlags aflags = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8 | RTC_INTERSECT16);
  RTCAlgorithmFlags aflags_all = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8 | RTC_INTERSECT16 | RTC_INTERSECT_STREAM);

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  struct InitExitTest : public VerifyApplication::Test
  {
    InitExitTest (std::string name)
      : VerifyApplication::Test(name,0,VerifyApplication::TEST_SHOULD_PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::Test(name,0,VerifyApplication::TEST_SHOULD_PASS), testID(testID) {}
  
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      VerifyScene scene(device,RTC_SCENE_STATIC,aflags);
      AssertNoError(device);
      unsigned geom0 = scene.addSphere(RTC_GEOMETRY_STATIC,zero,1.0f,50);
      unsigned geom1 = scene.addSphere(RTC_GEOMETRY_STATIC,zero,1.0f,50);
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      VerifyScene scene(device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(device);
      Ref<SceneGraph::Node> node = SceneGraph::createTriangleSphere(zero,1.0f,50);
      BBox3fa bounds0 = node->bounds();
      unsigned geom0 = scene.addGeometry(RTC_GEOMETRY_STATIC,node);
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,RTC_INTERSECT1);
      AssertNoError(device);

      unsigned geom0 = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom0,(void*)1);
      unsigned geom1 = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom1,(void*)2);
      unsigned geom2 = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom2,(void*)3);
      unsigned geom3 = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom3,(void*)4);
      unsigned geom4 = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, 0, 0, 0, 0, 0, 0, 1);
      rtcSetUserData(scene,geom4,(void*)5);
      unsigned geom5 = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, 0, 0, 0, 0, 0, 0, 2);
      rtcSetUserData(scene,geom5,(void*)6);
      unsigned geom6 = rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom6,(void*)7);
      unsigned geom7 = rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom7,(void*)8);
      unsigned geom8 = rtcNewCurveGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom8,(void*)9);
      unsigned geom9 = rtcNewCurveGeometry (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom9,(void*)10);
       unsigned geom10 = rtcNewLineSegments (scene, RTC_GEOMETRY_STATIC, 0, 0, 1);
      rtcSetUserData(scene,geom10,(void*)11);
      unsigned geom11 = rtcNewLineSegments (scene, RTC_GEOMETRY_STATIC, 0, 0, 2);
      rtcSetUserData(scene,geom11,(void*)12);
      unsigned geom12 = rtcNewUserGeometry2(scene,0,1);
      rtcSetUserData(scene,geom12,(void*)13);
      unsigned geom13 = rtcNewUserGeometry2(scene,0,2);
      rtcSetUserData(scene,geom13,(void*)14);
      rtcCommit (scene);
      AssertNoError(device);
      
      if ((size_t)rtcGetUserData(scene,geom0 ) !=  1) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom1 ) !=  2) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom2 ) !=  3) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom3 ) !=  4) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom4 ) !=  5) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom5 ) !=  6) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom6 ) !=  7) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom7 ) !=  8) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom8 ) !=  9) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom9 ) != 10) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom10) != 11) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom11) != 12) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom12) != 13) return VerifyApplication::FAILED;
      if ((size_t)rtcGetUserData(scene,geom13) != 14) return VerifyApplication::FAILED;
      return VerifyApplication::PASSED;
    }
  };

  struct BufferStrideTest : public VerifyApplication::Test
  {
    GeometryType gtype;

    BufferStrideTest (std::string name, int isa, GeometryType gtype)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), gtype(gtype) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      RTCSceneRef scene = rtcDeviceNewScene(device,RTC_SCENE_STATIC,aflags);
      AssertNoError(device);

      unsigned geomID = -1;
      switch (gtype) {
      case TRIANGLE_MESH    : geomID = rtcNewTriangleMesh   (scene, RTC_GEOMETRY_STATIC, 16, 16, 1); break;
      case TRIANGLE_MESH_MB : geomID = rtcNewTriangleMesh   (scene, RTC_GEOMETRY_STATIC, 16, 16, 2); break;
      case QUAD_MESH        : geomID = rtcNewQuadMesh       (scene, RTC_GEOMETRY_STATIC, 16, 16, 1); break;
      case QUAD_MESH_MB     : geomID = rtcNewQuadMesh       (scene, RTC_GEOMETRY_STATIC, 16, 16, 2); break;
      case SUBDIV_MESH      : geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC,  0, 16, 16, 0,0,0, 1); break;
      case SUBDIV_MESH_MB   : geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC,  0, 16, 16, 0,0,0, 2); break;
      case HAIR_GEOMETRY    : geomID = rtcNewHairGeometry   (scene, RTC_GEOMETRY_STATIC, 16, 16, 1); break;
      case HAIR_GEOMETRY_MB : geomID = rtcNewHairGeometry   (scene, RTC_GEOMETRY_STATIC, 16, 16, 2); break;
      case CURVE_GEOMETRY   : geomID = rtcNewCurveGeometry   (scene, RTC_GEOMETRY_STATIC, 16, 16, 1); break;
      case CURVE_GEOMETRY_MB: geomID = rtcNewCurveGeometry   (scene, RTC_GEOMETRY_STATIC, 16, 16, 2); break;
      default               : throw std::runtime_error("unknown geometry type: "+to_string(gtype));
      }
      AssertNoError(device);

      avector<char> indexBuffer(8+16*6*sizeof(int));
      avector<char> vertexBuffer(12+16*9*sizeof(float)+4);
      
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),1,3*sizeof(int));
      AssertError(device,RTC_INVALID_OPERATION);
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),1,3*sizeof(float));
      AssertError(device,RTC_INVALID_OPERATION);

      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int)+3);
      AssertError(device,RTC_INVALID_OPERATION);
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,3*sizeof(float)+3);
      AssertError(device,RTC_INVALID_OPERATION);
      
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int));
      AssertNoError(device);
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,3*sizeof(float));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),8,6*sizeof(int));
      AssertNoError(device);
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),12,9*sizeof(float));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geomID,RTC_INDEX_BUFFER,indexBuffer.data(),0,3*sizeof(int));
      AssertNoError(device);
      
      rtcSetBuffer(scene,geomID,RTC_VERTEX_BUFFER,vertexBuffer.data(),0,4*sizeof(float));
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
    
    VerifyApplication::TestReturnValue run (VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags);

      const Vec3fa center = zero;
      const float radius = 1.0f;
      const Vec3fa dx(1,0,0);
      const Vec3fa dy(0,1,0);
      scene.addGeometry(gflags,SceneGraph::createTriangleSphere(center,radius,50),false);
      scene.addGeometry(gflags,SceneGraph::createTriangleSphere(center,radius,50)->set_motion_vector(Vec3fa(1)),true);
      scene.addGeometry(gflags,SceneGraph::createQuadSphere(center,radius,50),false);
      scene.addGeometry(gflags,SceneGraph::createQuadSphere(center,radius,50)->set_motion_vector(Vec3fa(1)),true);
      scene.addGeometry(gflags,SceneGraph::createSubdivSphere(center,radius,8,20),false);
      scene.addGeometry(gflags,SceneGraph::createSubdivSphere(center,radius,8,20)->set_motion_vector(Vec3fa(1)),true);
      scene.addGeometry(gflags,SceneGraph::createHairyPlane(center,dx,dy,0.1f,0.01f,100,true),false);
      scene.addGeometry(gflags,SceneGraph::createHairyPlane(center,dx,dy,0.1f,0.01f,100,true)->set_motion_vector(Vec3fa(1)),true);
      scene.addGeometry(gflags,SceneGraph::createHairyPlane(center,dx,dy,0.1f,0.01f,100,false),false);
      scene.addGeometry(gflags,SceneGraph::createHairyPlane(center,dx,dy,0.1f,0.01f,100,false)->set_motion_vector(Vec3fa(1)),true);
      rtcCommit (scene);
      AssertNoError(device);

      if ((sflags & RTC_SCENE_DYNAMIC) == 0) 
      {
        for (size_t i=0; i<10; i++) {
          rtcDisable(scene,i);
          AssertAnyError(device);
        }
      }
      return VerifyApplication::PASSED;
    }
  };

  struct OverlappingGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags; 
    int N;
    
    OverlappingGeometryTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, int N)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags), N(N) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags);
      AssertNoError(device);

      const Vec3fa p (0,0,0);
      const Vec3fa dx(1,0,0);
      const Vec3fa dy(0,1,0);
      
      Ref<SceneGraph::TriangleMeshNode> trimesh = SceneGraph::createTrianglePlane(p,dx,dy,1,1).dynamicCast<SceneGraph::TriangleMeshNode>();
      for (size_t i=0; i<N; i++) trimesh->triangles.push_back(trimesh->triangles.back());
      scene.addGeometry(gflags,trimesh.dynamicCast<SceneGraph::Node>(),false);

      Ref<SceneGraph::TriangleMeshNode> trimesh2 = SceneGraph::createTrianglePlane(p,dx,dy,1,1)->set_motion_vector(Vec3fa(1)).dynamicCast<SceneGraph::TriangleMeshNode>();
      for (size_t i=0; i<N; i++) trimesh2->triangles.push_back(trimesh2->triangles.back());
      scene.addGeometry(gflags,trimesh2.dynamicCast<SceneGraph::Node>(),true);

      Ref<SceneGraph::QuadMeshNode> quadmesh = SceneGraph::createQuadPlane(p,dx,dy,1,1).dynamicCast<SceneGraph::QuadMeshNode>();
      for (size_t i=0; i<N; i++) quadmesh->quads.push_back(quadmesh->quads.back());
      scene.addGeometry(gflags,quadmesh.dynamicCast<SceneGraph::Node>(),false);

      Ref<SceneGraph::QuadMeshNode> quadmesh2 = SceneGraph::createQuadPlane(p,dx,dy,1,1)->set_motion_vector(Vec3fa(1)).dynamicCast<SceneGraph::QuadMeshNode>();
      for (size_t i=0; i<N; i++) quadmesh2->quads.push_back(quadmesh2->quads.back());
      scene.addGeometry(gflags,quadmesh2.dynamicCast<SceneGraph::Node>(),true);

      Ref<SceneGraph::SubdivMeshNode> subdivmesh = new SceneGraph::SubdivMeshNode(nullptr);
      for (size_t i=0; i<N; i++) {
        subdivmesh->verticesPerFace.push_back(4);
        subdivmesh->position_indices.push_back(4*i+0);
        subdivmesh->position_indices.push_back(4*i+1);
        subdivmesh->position_indices.push_back(4*i+2);
        subdivmesh->position_indices.push_back(4*i+3);
        subdivmesh->positions.push_back(Vec3fa(0,0,0));
        subdivmesh->positions.push_back(Vec3fa(0,1,0));
        subdivmesh->positions.push_back(Vec3fa(1,1,0));
        subdivmesh->positions.push_back(Vec3fa(1,0,0));
        /*subdivmesh->positions2.push_back(Vec3fa(0,0,0));
        subdivmesh->positions2.push_back(Vec3fa(0,1,0));
        subdivmesh->positions2.push_back(Vec3fa(1,1,0));
        subdivmesh->positions2.push_back(Vec3fa(1,0,0));*/
      }
      scene.addGeometry(gflags,subdivmesh.dynamicCast<SceneGraph::Node>(),false);
      //scene.addGeometry(gflags,subdivmesh.dynamicCast<SceneGraph::Node>(),true); // FIXME: enable mblur subdiv test when supported
      
      Ref<SceneGraph::HairSetNode> hairgeom = SceneGraph::createHairyPlane(p,dx,dy,0.2f,0.01f,1,true).dynamicCast<SceneGraph::HairSetNode>();
      for (size_t i=0; i<N; i++) hairgeom->hairs.push_back(hairgeom->hairs.back());
      scene.addGeometry(gflags,hairgeom.dynamicCast<SceneGraph::Node>(),false);

      Ref<SceneGraph::HairSetNode> hairgeom2 = SceneGraph::createHairyPlane(p,dx,dy,0.2f,0.01f,1,true)->set_motion_vector(Vec3fa(1)).dynamicCast<SceneGraph::HairSetNode>();
      for (size_t i=0; i<N; i++) hairgeom2->hairs.push_back(hairgeom2->hairs.back());
      scene.addGeometry(gflags,hairgeom2.dynamicCast<SceneGraph::Node>(),true);

      Ref<SceneGraph::Node> curvegeom = SceneGraph::convert_hair_to_curves(hairgeom.dynamicCast<SceneGraph::Node>());
      scene.addGeometry(gflags,curvegeom,false);

      Ref<SceneGraph::Node> curvegeom2 = SceneGraph::convert_hair_to_curves(hairgeom2.dynamicCast<SceneGraph::Node>());
      scene.addGeometry(gflags,curvegeom2,true);

      Ref<SceneGraph::Node> linegeom = SceneGraph::convert_bezier_to_lines(hairgeom.dynamicCast<SceneGraph::Node>());
      scene.addGeometry(gflags,linegeom,false);

      Ref<SceneGraph::Node> linegeom2 = SceneGraph::convert_bezier_to_lines(hairgeom2.dynamicCast<SceneGraph::Node>());
      scene.addGeometry(gflags,linegeom2,true);
      
      rtcCommit (scene);
      AssertNoError(device);
      return VerifyApplication::PASSED;
    }
  };
    
  struct NewDeleteGeometryTest : public VerifyApplication::Test
  {
    RTCSceneFlags sflags;

    NewDeleteGeometryTest (std::string name, int isa, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags_all);
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
            case 0: geom[index] = scene.addSphere(RTC_GEOMETRY_STATIC,pos,2.0f,10); break;
            case 1: geom[index] = scene.addHair  (RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,10); break;
            case 2: geom[index] = scene.addSubdivSphere(RTC_GEOMETRY_STATIC,pos,2.0f,4,4); break;
            case 3: 
              spheres[index] = Sphere(pos,2.0f);
              geom[index] = scene.addUserGeometryEmpty(&spheres[index]); break;
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      VerifyScene scene(device,sflags,aflags);
      AssertNoError(device);
      unsigned geom0 = scene.addSphere(RTC_GEOMETRY_STATIC,Vec3fa(-1,0,-1),1.0f,50);
      //unsigned geom1 = scene.addSphere(RTC_GEOMETRY_STATIC,Vec3fa(-1,0,+1),1.0f,50);
      unsigned geom1 = scene.addHair  (RTC_GEOMETRY_STATIC,Vec3fa(-1,0,+1),1.0f,1.0f,1);
      unsigned geom2 = scene.addSphere(RTC_GEOMETRY_STATIC,Vec3fa(+1,0,-1),1.0f,50);
      //unsigned geom3 = scene.addSphere(RTC_GEOMETRY_STATIC,Vec3fa(+1,0,+1),1.0f,50);
      unsigned geom3 = scene.addHair  (RTC_GEOMETRY_STATIC,Vec3fa(+1,0,+1),1.0f,1.0f,1);
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
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
    
    static void move_mesh(const VerifyScene& scene, unsigned mesh, size_t numVertices, Vec3fa& pos) 
    {
      Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
      for (size_t i=0; i<numVertices; i++) vertices[i] += Vec3fa(pos);
      rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
      rtcUpdate(scene,mesh);
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;
      
      VerifyScene scene(device,sflags,to_aflags(imode));
      AssertNoError(device);
      size_t numPhi = 10;
      size_t numVertices = 2*numPhi*(numPhi+1);
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      unsigned geom0 = scene.addSphere(gflags,pos0,1.0f,numPhi);
      unsigned geom1 = scene.addSphereHair(gflags,pos1,1.0f);
      unsigned geom2 = scene.addSphere(gflags,pos2,1.0f,numPhi);
      unsigned geom3 = scene.addSphereHair(gflags,pos3,1.0f);
      AssertNoError(device);
      
      for (size_t i=0; i<16; i++) 
      {
        bool move0 = i & 1, move1 = i & 2, move2 = i & 4, move3 = i & 8;
        Vec3fa ds(2,0.1f,2);
        if (move0) { move_mesh(scene,geom0,numVertices,ds); pos0 += ds; }
        if (move1) { move_mesh(scene,geom1,4,ds); pos1 += ds; }
        if (move2) { move_mesh(scene,geom2,numVertices,ds); pos2 += ds; }
        if (move3) { move_mesh(scene,geom3,4,ds); pos3 += ds; }
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

  struct GarbageGeometryTest : public VerifyApplication::Test
  {
    GarbageGeometryTest (std::string name, int isa)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));

      for (size_t i=0; i<size_t(1000*state->intensity); i++) 
      {
        srand(i*23565);
        if (i%20 == 0) std::cout << "." << std::flush;
        
        RTCSceneFlags sflag = getSceneFlag(i); 
        VerifyScene scene(device,sflag,aflags);
        AssertNoError(device);
        
        for (size_t j=0; j<20; j++) 
        {
          size_t numPrimitives = random<int>()%256;
          switch (random<int>()%8) {
          case 0: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageTriangleMesh(numPrimitives,false),false); break;
          case 1: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageTriangleMesh(numPrimitives,true ),true); break;
          case 2: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageQuadMesh(numPrimitives,false),false); break;
          case 3: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageQuadMesh(numPrimitives,true ),true); break;
          case 4: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageHair(numPrimitives,false),false); break;
          case 5: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageHair(numPrimitives,true ),true); break;
          case 6: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageLineSegments(numPrimitives,false),false); break;
          case 7: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageLineSegments(numPrimitives,true ),true); break;
            //case 8: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageSubdivMesh(numPrimitives,false),false); break; // FIXME: not working yet
            //case 9: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createGarbageSubdivMesh(numPrimitives,true ),true); break;
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), N(N) {}

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
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), N(N) {}
    
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

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), N(N) {}
    
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
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}

    inline Vec3fa uniformSampleTriangle(const Vec3fa &a, const Vec3fa &b, const Vec3fa &c, float &u, float& v)
    {
      const float su = sqrtf(u);
      v *= su;
      u = 1.0f-su;
      return c + u * (a-c) + v * (b-c);
    }

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      bool passed = true;
      Vec3fa pos0 = Vec3fa(-10,0,-10);
      Vec3fa pos1 = Vec3fa(-10,0,+10);
      Vec3fa pos2 = Vec3fa(+10,0,-10);
      Vec3fa pos3 = Vec3fa(+10,0,+10);
      
      VerifyScene scene(device,sflags,to_aflags(imode));
      unsigned geom0 = scene.addSphere(gflags,pos0,1.0f,50);
      //unsigned geom1 = scene.addSphere(gflags,pos1,1.0f,50);
      unsigned geom1 = scene.addHair  (gflags,pos1,1.0f,1.0f,1);
      unsigned geom2 = scene.addSphere(gflags,pos2,1.0f,50);
      //unsigned geom3 = scene.addSphere(gflags,pos3,1.0f,50);
      unsigned geom3 = scene.addHair  (gflags,pos3,1.0f,1.0f,1);
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
    GeometryType gtype;

    BackfaceCullingTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, GeometryType gtype, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags), gtype(gtype) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;
       
      /* create triangle that is front facing for a right handed 
         coordinate system if looking along the z direction */
      VerifyScene scene(device,sflags,to_aflags(imode));
      AssertNoError(device);
      const Vec3fa p0 = Vec3fa(0.0f);
      const Vec3fa dx = Vec3fa(1.0f,0.0f,0.0f);
      const Vec3fa dy = Vec3fa(0.0f,1.0f,0.0f);
      switch (gtype) {
      case TRIANGLE_MESH:    scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTrianglePlane(p0,dx,dy,1,1),false); break;
      case TRIANGLE_MESH_MB: scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTrianglePlane(p0,dx,dy,1,1)->set_motion_vector(Vec3fa(1)),true); break;
      case QUAD_MESH:        scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadPlane(p0,dx,dy,1,1),false); break;
      case QUAD_MESH_MB:     scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadPlane(p0,dx,dy,1,1)->set_motion_vector(Vec3fa(1)),true); break;
      case SUBDIV_MESH:      scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivPlane(p0,dx,dy,1,1,4.0f),false); break;
      case SUBDIV_MESH_MB:   scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivPlane(p0,dx,dy,1,1,4.0f)->set_motion_vector(Vec3fa(1)),true); break;
      default:               throw std::runtime_error("unsupported geometry type: "+to_string(gtype)); 
      }
      
      AssertNoError(device);
      rtcCommit (scene);
      AssertNoError(device);

      const size_t numRays = 1000;
      RTCRay rays[numRays];
      bool passed = true;

      for (size_t i=0; i<numRays; i++) {
        if (i%2) rays[i] = makeRay(Vec3fa(random<float>(),random<float>(),+1),Vec3fa(0,0,-1)); 
        else     rays[i] = makeRay(Vec3fa(random<float>(),random<float>(),-1),Vec3fa(0,0,+1)); 
      }
      
      IntersectWithMode(imode,ivariant,scene,rays,numRays);
      
      for (size_t i=0; i<numRays; i++) 
      {
        Vec3fa dir(rays[i].dir[0],rays[i].dir[1],rays[i].dir[2]);
        Vec3fa Ng (rays[i].Ng[0], rays[i].Ng[1], rays[i].Ng[2]);
        if (i%2) passed &= rays[i].geomID == -1;
        else {
          passed &= rays[i].geomID == 0;
          if (ivariant & VARIANT_INTERSECT)
            passed &= dot(dir,Ng) < 0.0f;
        }
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
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags), subdiv(subdiv) {}
    
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

    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      VerifyScene scene(device,sflags,to_aflags(imode));
      Vec3fa p0(-0.75f,-0.25f,-10.0f), dx(4,0,0), dy(0,4,0);
      int geom0 = 0;
      if (subdiv) geom0 = scene.addSubdivPlane (gflags, 4, p0, dx, dy);
      else        geom0 = scene.addPlane (gflags, 4, p0, dx, dy);
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
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
   
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      Vec3fa pos = zero;
      VerifyScene scene(device,sflags,to_aflags(imode));
      scene.addSphere(RTC_GEOMETRY_STATIC,pos,2.0f,50); // FIXME: use different geometries too
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
      : VerifyApplication::IntersectTest(name,isa,imode,VARIANT_INTERSECT,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), model(model), pos(pos) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      VerifyScene scene(device,sflags,to_aflags(imode));
      if      (model == "sphere.triangles") scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTriangleSphere(pos,2.0f,500),false);
      else if (model == "sphere.quads"    ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadSphere    (pos,2.0f,500),false);
      else if (model == "sphere.subdiv"   ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivSphere  (pos,2.0f,4,64),false);
      else if (model == "plane.triangles" ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createTrianglePlane (Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f),500,500),false);
      else if (model == "plane.quads"     ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createQuadPlane     (Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f),500,500),false);
      else if (model == "plane.subdiv"    ) scene.addGeometry(RTC_GEOMETRY_STATIC,SceneGraph::createSubdivPlane   (Vec3fa(pos.x,-6.0f,-6.0f),Vec3fa(0.0f,0.0f,12.0f),Vec3fa(0.0f,12.0f,0.0f),500,500,2),false);
      bool plane = model.compare(0,5,"plane") == 0;
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
      if (!silent) { printf(" (%f%%)", 100.0f*failRate); fflush(stdout); }
      return (VerifyApplication::TestReturnValue)(!failed);
    }
  };

  struct NaNTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    
    NaNTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags)  {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      const size_t numRays = 1000;
      RTCRay rays[numRays];
      VerifyScene scene(device,sflags,to_aflags(imode));
      scene.addSphere(gflags,zero,2.0f,100);
      scene.addHair  (gflags,zero,1.0f,1.0f,100);
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
      if (!silent) { printf(" (%3.2fx)",f); fflush(stdout); }
      return (VerifyApplication::TestReturnValue) ok;
    }
  };
    
  struct InfTest : public VerifyApplication::IntersectTest
  {
    RTCSceneFlags sflags;
    RTCGeometryFlags gflags;
    
    InfTest (std::string name, int isa, RTCSceneFlags sflags, RTCGeometryFlags gflags, IntersectMode imode, IntersectVariant ivariant)
      : VerifyApplication::IntersectTest(name,isa,imode,ivariant,VerifyApplication::TEST_SHOULD_PASS), sflags(sflags), gflags(gflags) {}
   
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
    {
      std::string cfg = state->rtcore + ",isa="+stringOfISA(isa);
      RTCDeviceRef device = rtcNewDevice(cfg.c_str());
      error_handler(rtcDeviceGetError(device));
      if (!supportsIntersectMode(device))
        return VerifyApplication::SKIPPED;

      const size_t numRays = 1000;
      RTCRay rays[numRays];
      VerifyScene scene(device,sflags,to_aflags(imode));
      scene.addSphere(gflags,zero,2.0f,100);
      scene.addHair  (gflags,zero,1.0f,1.0f,100);
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
      if (!silent) { printf(" (%3.2fx)",f); fflush(stdout); }
      return (VerifyApplication::TestReturnValue) ok;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  void shootRandomRays (std::vector<IntersectMode>& intersectModes, std::vector<IntersectVariant>& intersectVariants, const VerifyScene& scene)
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
    RegressionTask (size_t sceneIndex, size_t sceneCount, size_t threadCount, bool cancelBuild)
      : sceneIndex(sceneIndex), sceneCount(sceneCount), scene(nullptr), numActiveThreads(0), cancelBuild(cancelBuild), errorCounter(0) 
    { 
      barrier.init(threadCount); 
    }

    size_t sceneIndex;
    size_t sceneCount;
    VerifyApplication* state;
    Ref<VerifyScene> scene;
    BarrierSys barrier;
    volatile size_t numActiveThreads;
    bool cancelBuild;
    std::atomic<size_t> errorCounter;
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
  std::atomic<size_t> monitorProgressInvokations(0);

  bool monitorProgressFunction(void* ptr, double dn) 
  {
    size_t n = monitorProgressInvokations++;
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
          if (build_join_test) rtcCommit(*task->scene);
          else                 {
            rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
            rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
          }
	  //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            task->errorCounter++;
          }
          else {
            shootRandomRays(thread->intersectModes,thread->state->intersectVariants,*task->scene);
          }
	}
        task->barrier.wait();
      }
      delete thread; thread = nullptr;
      return;
    }

    if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
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
      task->scene = new VerifyScene(thread->device,sflag,aflags_all);
      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
      if (task->cancelBuild) rtcSetProgressMonitorFunction(*task->scene,monitorProgressFunction,nullptr);
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
        case 0: task->scene->addSphere(RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,0.0f); break;
	case 1: task->scene->addSphere(RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,1.0f); break;
	case 2: task->scene->addSubdivSphere(RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,4,numTriangles,0.0f); break;
	case 3: task->scene->addHair  (RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,numTriangles,0.0f); break;
	case 4: task->scene->addHair  (RTC_GEOMETRY_STATIC,pos,1.0f,2.0f,numTriangles,1.0f); break; 

        case 5: {
	  Sphere* sphere = new Sphere(pos,2.0f); spheres.push_back(sphere); 
	  task->scene->addUserGeometryEmpty(sphere); break;
        }
	}
        //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
        if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
          task->errorCounter++;
          hasError = true;
          break;
        }
      }
      
      if (thread->threadCount) {
	task->numActiveThreads = max(size_t(1),random<int>() % thread->threadCount);
	task->barrier.wait();
        if (build_join_test) rtcCommit(*task->scene);
        else                 {
          rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
          rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);          
        }
      } else {
        if (!hasError) {
          rtcCommit(*task->scene);
        }
      }
      //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;

      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
        task->errorCounter++;
      }
      else {
        if (!hasError) {
          shootRandomRays(thread->intersectModes,thread->state->intersectVariants,*task->scene);
        }
      }

      if (thread->threadCount) 
	task->barrier.wait();

      task->scene = nullptr;
      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;

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
          if (build_join_test) rtcCommit(*task->scene);
          else	               rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
	  //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            task->errorCounter++;
          }
          else {
            shootRandomRays(thread->intersectModes,thread->state->intersectVariants,*task->scene);
          }
	}
	task->barrier.wait();
      }
      delete thread; thread = nullptr;
      return;
    }
    task->scene = new VerifyScene(thread->device,RTC_SCENE_DYNAMIC,aflags_all);
    if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
    if (task->cancelBuild) rtcSetProgressMonitorFunction(*task->scene,monitorProgressFunction,nullptr);
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
          case 0: geom[index] = task->scene->addSphere(RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,0.0f); break;
          case 1: geom[index] = task->scene->addSphere(RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,numTriangles,0.0f); break;
          case 2: geom[index] = task->scene->addSphere(RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,numTriangles,0.0f); break;
          case 3: geom[index] = task->scene->addSubdivSphere(RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,4,numTriangles,0.0f); break;
	  case 4: geom[index] = task->scene->addSubdivSphere(RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,4,numTriangles,0.0f); break;
	  case 5: geom[index] = task->scene->addSubdivSphere(RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,4,numTriangles,0.0f); break;
          case 6: geom[index] = task->scene->addSphere(RTC_GEOMETRY_STATIC,pos,2.0f,numPhi,numTriangles,1.0f); break;
          case 7: geom[index] = task->scene->addSphere(RTC_GEOMETRY_DEFORMABLE,pos,2.0f,numPhi,numTriangles,1.0f); break;
          case 8: geom[index] = task->scene->addSphere(RTC_GEOMETRY_DYNAMIC,pos,2.0f,numPhi,numTriangles,1.0f); break;
          case 9: spheres[index] = Sphere(pos,2.0f); geom[index] = task->scene->addUserGeometryEmpty(&spheres[index]); break;
          }; 
	  //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
          if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) {
            task->errorCounter++;
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
            rtcDeleteGeometry(*task->scene,geom[index]);     
	    if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
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
              rtcDeleteGeometry(*task->scene,geom[index]);     
	      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
              geom[index] = -1; 
              break;
            }
            case 1: {
              Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(*task->scene,geom[index],RTC_VERTEX_BUFFER);
              if (vertices) { 
                for (size_t i=0; i<numVertices[index]; i++) vertices[i] += Vec3fa(0.1f);
              }
              rtcUnmapBuffer(*task->scene,geom[index],RTC_VERTEX_BUFFER);
              
              if (types[index] == 7 || types[index] == 8) {
                Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(*task->scene,geom[index],RTC_VERTEX_BUFFER1);
                if (vertices) {
                  for (size_t i=0; i<numVertices[index]; i++) vertices[i] += Vec3fa(0.1f);
                }
                rtcUnmapBuffer(*task->scene,geom[index],RTC_VERTEX_BUFFER1);
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
              rtcDeleteGeometry(*task->scene,geom[i]);
              if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;
              geom[i] = -1;
            }
          }
        }
      }

      if (thread->threadCount) {
	task->numActiveThreads = max(size_t(1),random<int>() % thread->threadCount);
	task->barrier.wait();
        if (build_join_test) rtcCommit(*task->scene);
        else                 rtcCommitThread(*task->scene,thread->threadIndex,task->numActiveThreads);
      } else {
        if (!hasError) 
          rtcCommit(*task->scene);
      }
      //if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;

      if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR)
        task->errorCounter++;
      else
        if (!hasError)
          shootRandomRays(thread->intersectModes,thread->state->intersectVariants,*task->scene);

      if (thread->threadCount) 
	task->barrier.wait();
    }

    task->scene = nullptr;
    if (rtcDeviceGetError(thread->device) != RTC_NO_ERROR) task->errorCounter++;;

    delete thread; thread = nullptr;
    return;
  }

  struct IntensiveRegressionTest : public VerifyApplication::Test
  {
    thread_func func;
    int mode;
    std::vector<IntersectMode> intersectModes;
    std::vector<thread_t> threads;
    
    IntensiveRegressionTest (std::string name, int isa, thread_func func, int mode)
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), func(func), mode(mode) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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

      size_t errorCounter = 0;
      size_t sceneIndex = 0;
      while (sceneIndex < size_t(30*state->intensity)) 
      {
        if (mode)
        {
          build_join_test = (mode == 2);
          size_t numThreads = getNumberOfLogicalThreads();
          
          std::vector<RegressionTask*> tasks;
          while (numThreads) 
          {
            size_t N = max(size_t(1),random<int>()%numThreads); numThreads -= N;
            RegressionTask* task = new RegressionTask(sceneIndex++,5,N,false);
            tasks.push_back(task);
            
            for (size_t i=0; i<N; i++) 
              threads.push_back(createThread(func,new ThreadRegressionTask(i,N,state,device,intersectModes,task),DEFAULT_STACK_SIZE,numThreads+i));
          }
          
          for (size_t i=0; i<threads.size(); i++)
            join(threads[i]);
          for (size_t i=0; i<tasks.size(); i++) {
            errorCounter += tasks[i]->errorCounter;
            delete tasks[i];
          }
          
          threads.clear();
        }
        else
        {
          RegressionTask task(sceneIndex++,5,0,false);
          func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task));
        }	
      }
      return (VerifyApplication::TestReturnValue) (errorCounter == 0);
    }
  };

  ssize_t monitorMemoryBreak = -1;
  std::atomic<size_t> monitorMemoryBytesUsed(0);
  std::atomic<size_t> monitorMemoryInvokations(0);

  bool monitorMemoryFunction(ssize_t bytes, bool post) 
  {
    monitorMemoryBytesUsed += bytes;
    if (bytes > 0) {
      size_t n = monitorMemoryInvokations++;
      if (n == monitorMemoryBreak) {
        if (!post) monitorMemoryBytesUsed -= bytes;
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
      : VerifyApplication::Test(name,isa,VerifyApplication::TEST_SHOULD_PASS), func(func) {}
    
    VerifyApplication::TestReturnValue run(VerifyApplication* state, bool silent)
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
      
      rtcDeviceSetMemoryMonitorFunction(device,monitorMemoryFunction);
      
      size_t sceneIndex = 0;
      while (sceneIndex < size_t(30*state->intensity)) 
      {
        monitorMemoryBreak = -1;
        monitorMemoryBytesUsed = 0;
        monitorMemoryInvokations = 0;
        monitorProgressBreak = -1;
        monitorProgressInvokations = 0;
        RegressionTask task1(sceneIndex,1,0,true);
        func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task1));
        if (monitorMemoryBytesUsed) {
          rtcDeviceSetMemoryMonitorFunction(device,nullptr);
          return VerifyApplication::FAILED;
        }
        monitorMemoryBreak = monitorMemoryInvokations * drand48();
        monitorMemoryBytesUsed = 0;
        monitorMemoryInvokations = 0;
        monitorProgressBreak = monitorProgressInvokations * 2.0f * drand48();
        monitorProgressInvokations = 0;
        RegressionTask task2(sceneIndex,1,0,true);
        func(new ThreadRegressionTask(0,0,state,device,intersectModes,&task2));
        if (monitorMemoryBytesUsed || (monitorMemoryInvokations != 0 && task2.errorCounter != 1)) {
          rtcDeviceSetMemoryMonitorFunction(device,nullptr);
          return VerifyApplication::FAILED;
        }
        sceneIndex++;
      }
      rtcDeviceSetMemoryMonitorFunction(device,nullptr);
      return VerifyApplication::PASSED;
    }
  };

  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////

  VerifyApplication::VerifyApplication ()
    : Application(Application::FEATURE_RTCORE), intensity(1.0f), tests(new TestGroup("")), numFailedTests(0), user_specified_tests(false), flatten(true), parallel(true)
  {
    GeometryType gtypes[] = { TRIANGLE_MESH, TRIANGLE_MESH_MB, QUAD_MESH, QUAD_MESH_MB, SUBDIV_MESH, SUBDIV_MESH_MB };

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

    std::stack<Ref<TestGroup>> groups;
    groups.push(tests.dynamicCast<TestGroup>());
    auto push = [&] (Ref<TestGroup> group) {
      groups.top()->add(group.dynamicCast<Test>());
      groups.push(group);
    };

    RTCDeviceRef device = rtcNewDevice(rtcore.c_str());
    error_handler(rtcDeviceGetError(device));

    groups.top()->add(new InitExitTest("init_exit"));

    /* add Embree internal tests */
    for (size_t i=2000000; i<3000000; i++) {
      const char* testName = (const char*) rtcDeviceGetParameter1i(device,(RTCParameter)i);
      if (testName == nullptr) break;
      groups.top()->add(new EmbreeInternalTest(testName,i-2000000));
    }

    for (auto isa : isas)
    {
      push(new TestGroup(stringOfISA(isa)));
      
      groups.top()->add(new MultipleDevicesTest("multiple_devices",isa));

      push(new TestGroup("flags",true));
      groups.top()->add(new FlagsTest("static_static"     ,isa,VerifyApplication::TEST_SHOULD_PASS, RTC_SCENE_STATIC, RTC_GEOMETRY_STATIC));
      groups.top()->add(new FlagsTest("static_deformable" ,isa,VerifyApplication::TEST_SHOULD_FAIL, RTC_SCENE_STATIC, RTC_GEOMETRY_DEFORMABLE));
      groups.top()->add(new FlagsTest("static_dynamic"    ,isa,VerifyApplication::TEST_SHOULD_FAIL, RTC_SCENE_STATIC, RTC_GEOMETRY_DYNAMIC));
      groups.top()->add(new FlagsTest("dynamic_static"    ,isa,VerifyApplication::TEST_SHOULD_PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC));
      groups.top()->add(new FlagsTest("dynamic_deformable",isa,VerifyApplication::TEST_SHOULD_PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DEFORMABLE));
      groups.top()->add(new FlagsTest("dynamic_dynamic"   ,isa,VerifyApplication::TEST_SHOULD_PASS, RTC_SCENE_DYNAMIC,RTC_GEOMETRY_DYNAMIC));    
      groups.pop();

      groups.top()->add(new UnmappedBeforeCommitTest("unmapped_before_commit",isa));
      groups.top()->add(new GetBoundsTest("get_bounds",isa));
      groups.top()->add(new GetUserDataTest("get_user_data",isa));

      push(new TestGroup("buffer_stride",true));
      for (auto gtype : gtypes)
        groups.top()->add(new BufferStrideTest(to_string(gtype),isa,gtype));
      groups.pop();
      
      /**************************************************************************/
      /*                        Builder Tests                                   */
      /**************************************************************************/
      
      push(new TestGroup("empty_scene",true));
      for (auto sflags : sceneFlags) 
        groups.top()->add(new EmptySceneTest(to_string(sflags),isa,sflags));
      groups.pop();
      
      push(new TestGroup("empty_geometry",true));
      for (auto sflags : sceneFlags) 
        groups.top()->add(new EmptyGeometryTest(to_string(sflags),isa,sflags,RTC_GEOMETRY_STATIC));
      groups.pop();
      
      push(new TestGroup("build",true));
      for (auto sflags : sceneFlags) 
        groups.top()->add(new BuildTest(to_string(sflags),isa,sflags,RTC_GEOMETRY_STATIC));
      groups.pop();
      
      push(new TestGroup("overlapping_primitives",true));
      for (auto sflags : sceneFlags)
        groups.top()->add(new OverlappingGeometryTest(to_string(sflags),isa,sflags,RTC_GEOMETRY_STATIC,100000));
      groups.pop();

      push(new TestGroup("new_delete_geometry",true));
      for (auto sflags : sceneFlagsDynamic) 
        groups.top()->add(new NewDeleteGeometryTest(to_string(sflags),isa,sflags));
      groups.pop();
      
      push(new TestGroup("enable_disable_geometry",true));
      for (auto sflags : sceneFlagsDynamic) 
        groups.top()->add(new EnableDisableGeometryTest(to_string(sflags),isa,sflags));
      groups.pop();
      
      push(new TestGroup("update",true));
      for (auto sflags : sceneFlagsDynamic) {
        for (auto imode : intersectModes) {
          for (auto ivariant : intersectVariants) {
            groups.top()->add(new UpdateTest("deformable."+to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_DEFORMABLE,imode,ivariant));
            groups.top()->add(new UpdateTest("dynamic."+to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_DYNAMIC,imode,ivariant));
          }
        }
      }
      groups.pop();

      groups.top()->add(new GarbageGeometryTest("build_garbage_geom."+stringOfISA(isa),isa));

      /**************************************************************************/
      /*                     Interpolation Tests                                */
      /**************************************************************************/
      
      push(new TestGroup("interpolate"));
      int interpolateTests[] = { 4,5,8,11,12,15 };

      push(new TestGroup("triangles",true));
      for (auto s : interpolateTests) 
        groups.top()->add(new InterpolateTrianglesTest(std::to_string(long(s)),isa,s));
      groups.pop();

      push(new TestGroup("subdiv",true));
      for (auto s : interpolateTests)
        groups.top()->add(new InterpolateSubdivTest(std::to_string(long(s)),isa,s));
      groups.pop();
        
      push(new TestGroup("hair",true));
      for (auto s : interpolateTests) 
        groups.top()->add(new InterpolateHairTest(std::to_string(long(s)),isa,s));
      groups.pop();

      groups.pop();
      
      /**************************************************************************/
      /*                      Intersection Tests                                */
      /**************************************************************************/

      push(new TestGroup("triangle_hit",true));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            groups.top()->add(new TriangleHitTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      groups.pop();
      
      push(new TestGroup("quad_hit",true));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            groups.top()->add(new QuadHitTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      groups.pop();

      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_RAY_MASK)) 
      {
        push(new TestGroup("ray_masks",true));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              groups.top()->add(new RayMasksTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        groups.pop();
      }
      
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_BACKFACE_CULLING)) 
      {
        push(new TestGroup("backface_culling",true));
        for (auto gtype : gtypes)
          for (auto sflags : sceneFlags) 
            for (auto imode : intersectModes) 
              for (auto ivariant : intersectVariants)
                groups.top()->add(new BackfaceCullingTest(to_string(gtype,sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,gtype,imode,ivariant));
        groups.pop();
      }
      
      push(new TestGroup("intersection_filter",true));
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECTION_FILTER)) 
      {
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              groups.top()->add(new IntersectionFilterTest("triangles."+to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,false,imode,ivariant));
        
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              groups.top()->add(new IntersectionFilterTest("subdiv."+to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,true,imode,ivariant));
      }
      groups.pop();
      
      push(new TestGroup("inactive_rays",true));
      for (auto sflags : sceneFlags) 
        for (auto imode : intersectModes) 
          for (auto ivariant : intersectVariants)
            if (imode != MODE_INTERSECT1) // INTERSECT1 does not support disabled rays
              groups.top()->add(new InactiveRaysTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
      groups.pop();
      
      push(new TestGroup("watertight_triangles",true)); {
        std::string watertightModels [] = {"sphere.triangles", "plane.triangles"};
        const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new WatertightTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model,watertight_pos));
        groups.pop();
      }

      push(new TestGroup("watertight_quads",true)); {
        std::string watertightModels [] = {"sphere.quads", "plane.quads"};
        const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new WatertightTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model,watertight_pos));
        groups.pop();
      }

      push(new TestGroup("watertight_subdiv",true)); {
        std::string watertightModels [] = { "sphere.subdiv", "plane.subdiv"};
        const Vec3fa watertight_pos = Vec3fa(148376.0f,1234.0f,-223423.0f);
        for (auto sflags : sceneFlagsRobust) 
          for (auto imode : intersectModes) 
            for (std::string model : watertightModels) 
              groups.top()->add(new WatertightTest(to_string(sflags,imode)+"."+model,isa,sflags,imode,model,watertight_pos));
        groups.pop();
      }
      
      if (rtcDeviceGetParameter1i(device,RTC_CONFIG_IGNORE_INVALID_RAYS))
      {
        push(new TestGroup("nan_test",true));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              groups.top()->add(new NaNTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        groups.pop();
        
        push(new TestGroup("inf_test",true));
        for (auto sflags : sceneFlags) 
          for (auto imode : intersectModes) 
            for (auto ivariant : intersectVariants)
              groups.top()->add(new InfTest(to_string(sflags,imode,ivariant),isa,sflags,RTC_GEOMETRY_STATIC,imode,ivariant));
        groups.pop();
      }
    
      /**************************************************************************/
      /*                  Randomized Stress Testing                             */
      /**************************************************************************/
      
      groups.top()->add(new IntensiveRegressionTest("regression_static",isa,rtcore_regression_static_thread,0));
      groups.top()->add(new IntensiveRegressionTest("regression_dynamic",isa,rtcore_regression_dynamic_thread,0));
      
      groups.top()->add(new IntensiveRegressionTest("regression_static_user_threads", isa,rtcore_regression_static_thread,1));
      groups.top()->add(new IntensiveRegressionTest("regression_dynamic_user_threads",isa,rtcore_regression_dynamic_thread,1));
      
      groups.top()->add(new IntensiveRegressionTest("regression_static_build_join", isa,rtcore_regression_static_thread,2));
      groups.top()->add(new IntensiveRegressionTest("regression_dynamic_build_join",isa,rtcore_regression_dynamic_thread,2));
      
      groups.top()->add(new MemoryMonitorTest("regression_static_memory_monitor", isa,rtcore_regression_static_thread));
      groups.top()->add(new MemoryMonitorTest("regression_dynamic_memory_monitor",isa,rtcore_regression_dynamic_thread));

      groups.pop();
    }
    prefix_test_names(tests);

    /* ignore failure of some tests that are known to fail */
    map_tests(tests, [&] (Ref<Test> test) { 
        if (test->name.find("watertight_subdiv") != std::string::npos) test->ignoreFailure = true;
      });
    
    /**************************************************************************/
    /*                     Command Line Parsing                               */
    /**************************************************************************/
  
    registerOption("run", [this] (Ref<ParseStream> cin, const FileName& path) {
        if (!user_specified_tests) enable_disable_all_tests(tests,false);
        user_specified_tests = true;
        std::string regex = cin->getString();
        enable_disable_some_tests(tests,regex,true);
      }, "--run <regexpr>: Runs all tests whose name match the regular expression.");
    registerOptionAlias("run","enable");

    registerOption("skip", [this] (Ref<ParseStream> cin, const FileName& path) {
        if (!user_specified_tests) enable_disable_all_tests(tests,true);
        user_specified_tests = true;
        std::string regex = cin->getString();
        enable_disable_some_tests(tests,regex,false);
      }, "--skip <regexpr>: Skips all tests whose name matches the regular expression.");
    registerOptionAlias("skip","disable");

    registerOption("skip-before", [this] (Ref<ParseStream> cin, const FileName& path) {
        if (!user_specified_tests) enable_disable_all_tests(tests,true);
        user_specified_tests = true;
        bool found = false;
        std::string regex = cin->getString();
        map_tests(tests, [&] (Ref<Test> test) {
            if (regex_match(test->name,regex)) found = true;
            test->enabled &= found;
          });
      }, "--skip-before <regexpr>: Skips all tests before the first test matching the regular expression.");
    registerOptionAlias("skip-before","disable-before");

    registerOption("flatten", [this] (Ref<ParseStream> cin, const FileName& path) {
        flatten = false;
      }, "--flatten: shows all leaf test names when executing tests");
    
    registerOption("sequential", [this] (Ref<ParseStream> cin, const FileName& path) {
        parallel = false;
      }, "--sequential: execute all tests sequential");

    registerOption("parallel", [this] (Ref<ParseStream> cin, const FileName& path) {
        parallel = true;
      }, "--parallel: parallelized test execution (default)");

    registerOption("print-tests", [this] (Ref<ParseStream> cin, const FileName& path) {
        print_tests(tests,0);
        exit(1);
      }, "--print-tests: prints all enabled tests");
    
    registerOption("intensity", [this] (Ref<ParseStream> cin, const FileName& path) {
        intensity = cin->getFloat();
      }, "--intensity <float>: intensity of testing to perform");
  }

  void VerifyApplication::prefix_test_names(Ref<Test> test, std::string prefix)
  {
    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) 
      for (auto t : group->tests) 
        prefix_test_names(t,group->extend_prefix(prefix));

    test->name = prefix + test->name;
  }

  bool VerifyApplication::update_tests(Ref<Test> test)
  {
    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) 
    {
      bool any_enabled = false;
      for (auto t : group->tests) any_enabled |= update_tests(t);
      return test->enabled = any_enabled;
    } 
    else 
      return test->enabled;
  }

  void VerifyApplication::print_tests(Ref<Test> test, size_t depth)
  {
    if (!test->isEnabled()) return;

    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) 
    {
      std::cout << std::string(2*depth,' ') << group->name << (group->name != "" ? " " : "") << "{" << std::endl;
      for (auto t : group->tests) print_tests(t,depth+1);
      std::cout << std::string(2*depth,' ') << "}" << std::endl;
    } else {
      std::cout << std::string(2*depth,' ') << test->name << std::endl;
    }
  }

  template<typename Function> 
  void VerifyApplication::map_tests(Ref<Test> test, const Function& f)
  {
    if (Ref<TestGroup> group = test.dynamicCast<TestGroup>()) {
      for (auto t : group->tests) map_tests(t,f);
    } else {
      f(test);
    }
  }

  void VerifyApplication::enable_disable_all_tests(Ref<Test> test, bool enabled)
  {
    map_tests(test, [&] (Ref<Test> test) { test->enabled = enabled; });
    update_tests(test);
  }

  void VerifyApplication::enable_disable_some_tests(Ref<Test> test, std::string regex, bool enabled)
  {
    map_tests(test, [&] (Ref<Test> test) { 
        if (regex_match(test->name,regex)) 
          test->enabled = enabled;
      });
   update_tests(test);
  }

  int VerifyApplication::main(int argc, char** argv) try
  {
    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
    
    /* parse command line options */
    parseCommandLine(argc,argv);

    /* run all enabled tests */
    tests->execute(this,false);

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
}

int main(int argc, char** argv)
{
  embree::VerifyApplication app;
  return app.main(argc,argv);
}
