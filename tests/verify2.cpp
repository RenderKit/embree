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

#include "verify2.h"

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
  
#if 0
  struct StaticSceneTest : public VerifyApplication::Test
  {
    StaticSceneTest (std::string name, VerifyApplication::TestType type, RTCSceneFlags sceneFlags, RTCGeometryFlags geomFlags)
      : VerifyApplication::Test(name,type), sceneFlags(sceneFlags), geomFlags(geomFlags) {}

    bool run(VerifyApplication* state)
    {
      bool rtcore_static_scene()
  {
    ClearBuffers clear_before_return;
    RTCSceneRef scene = rtcDeviceNewScene(g_device,RTC_SCENE_STATIC,aflags);
    AssertNoError();
    unsigned geom0 = addSphere(scene,RTC_GEOMETRY_STATIC,zero,1.0f,50);
    AssertNoError();
    unsigned geom1 = addSubdivSphere(scene,RTC_GEOMETRY_STATIC,zero,1.0f,10,16);
    AssertNoError();
    unsigned geom2 = addHair(scene,RTC_GEOMETRY_STATIC,Vec3fa(0,0,0),1.0f,0.5f,100);
    AssertNoError();
    rtcCommit (scene);
    AssertNoError();
    //rtcCommit (scene); // cannot commit static scene twice
    //AssertAnyError();
    rtcDisable(scene,geom0); // static scene cannot get modified anymore after commit
    AssertAnyError();
    scene = nullptr;
    return true;
  }
#endif

  VerifyApplication::VerifyApplication ()
    : device(nullptr), rtcore(""), regressionN(200), numFailedTests(0)
  {
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
    
    //POSITIVE("static_scene",              rtcore_static_scene());
    //POSITIVE("unmapped_before_commit",    rtcore_unmapped_before_commit());
    //POSITIVE("get_bounds",                rtcore_rtcGetBounds());

#if defined(RTCORE_BUFFER_STRIDE)
    //POSITIVE("buffer_stride",             rtcore_buffer_stride());
#endif

    //POSITIVE("dynamic_enable_disable",    rtcore_dynamic_enable_disable());
    //POSITIVE("get_user_data"         ,    rtcore_get_user_data());

    //POSITIVE("update_deformable",         rtcore_update(RTC_GEOMETRY_DEFORMABLE));
    //POSITIVE("update_dynamic",            rtcore_update(RTC_GEOMETRY_DYNAMIC));
    //POSITIVE("overlapping_triangles",     rtcore_overlapping_triangles(100000));
    //POSITIVE("overlapping_hair",          rtcore_overlapping_hair(100000));
    //POSITIVE("new_delete_geometry",       rtcore_new_delete_geometry());

    //POSITIVE("interpolate_subdiv4",                rtcore_interpolate_subdiv(4));
    //POSITIVE("interpolate_subdiv5",                rtcore_interpolate_subdiv(5));
    //POSITIVE("interpolate_subdiv8",                rtcore_interpolate_subdiv(8));
    //POSITIVE("interpolate_subdiv11",               rtcore_interpolate_subdiv(11));
    //POSITIVE("interpolate_subdiv12",               rtcore_interpolate_subdiv(12));
    //POSITIVE("interpolate_subdiv15",               rtcore_interpolate_subdiv(15));

    //POSITIVE("interpolate_triangles4",                rtcore_interpolate_triangles(4));
    //POSITIVE("interpolate_triangles5",                rtcore_interpolate_triangles(5));
    //POSITIVE("interpolate_triangles8",                rtcore_interpolate_triangles(8));
    //POSITIVE("interpolate_triangles11",               rtcore_interpolate_triangles(11));
    //POSITIVE("interpolate_triangles12",               rtcore_interpolate_triangles(12));
    //POSITIVE("interpolate_triangles15",               rtcore_interpolate_triangles(15));

    //POSITIVE("interpolate_hair4",                rtcore_interpolate_hair(4));
    //POSITIVE("interpolate_hair5",                rtcore_interpolate_hair(5));
    //POSITIVE("interpolate_hair8",                rtcore_interpolate_hair(8));
    //POSITIVE("interpolate_hair11",               rtcore_interpolate_hair(11));
    //POSITIVE("interpolate_hair12",               rtcore_interpolate_hair(12));
    //POSITIVE("interpolate_hair15",               rtcore_interpolate_hair(15));

    //rtcore_build();

#if defined(RTCORE_RAY_MASK)
    //rtcore_ray_masks_all();
#endif

#if defined(RTCORE_INTERSECTION_FILTER)
    //rtcore_filter_all(false);
#endif

#if defined(RTCORE_INTERSECTION_FILTER) && !defined(__MIC__) // FIXME: subdiv intersection filters not yet implemented for MIC
    //rtcore_filter_all(true);
#endif


#if defined(RTCORE_BACKFACE_CULLING)
    //rtcore_backface_culling_all();
#endif

    //rtcore_packet_write_test_all();

    //rtcore_watertight_closed1("sphere", pos);
    //rtcore_watertight_closed1("cube",pos);
    //rtcore_watertight_plane1(100000);
#if HAS_INTERSECT4
    //rtcore_watertight_closed4("sphere",pos);
    //rtcore_watertight_closed4("cube",pos);
    //rtcore_watertight_plane4(100000);
#endif

#if HAS_INTERSECT8
    //if (hasISA(AVX)) {
    //  rtcore_watertight_closed8("sphere",pos);
    //  rtcore_watertight_closed8("cube",pos);
    //  rtcore_watertight_plane8(100000);
    //}
#endif

#if HAS_INTERSECT16
    //if (hasISA(AVX512KNL) || hasISA(KNC))
    //{
    //  rtcore_watertight_closed16("sphere",pos);
    //  rtcore_watertight_closed16("cube",pos);
    //  rtcore_watertight_plane16(100000);
    //}

#endif

#if defined(RTCORE_IGNORE_INVALID_RAYS)
    //rtcore_nan("nan_test_1",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,1);
    //rtcore_inf("inf_test_1",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,1);

#if HAS_INTERSECT4
    //rtcore_nan("nan_test_4",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,4);
    //rtcore_inf("inf_test_4",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,4);
#endif

#if HAS_INTERSECT8
    //if (hasISA(AVX)) {
    //  rtcore_nan("nan_test_8",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,8);
    //  rtcore_inf("inf_test_8",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,8);
    //}
#endif

#if HAS_INTERSECT16
    //if (hasISA(AVX512KNL) || hasISA(KNC))
    //{
    //  rtcore_nan("nan_test_16",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,16);
    //  rtcore_inf("inf_test_16",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,16);
    //}
#endif
#endif

    //POSITIVE("regression_static",         rtcore_regression(rtcore_regression_static_thread,0));
    //POSITIVE("regression_dynamic",        rtcore_regression(rtcore_regression_dynamic_thread,0));


#if defined(TASKING_TBB) || defined(TASKING_INTERNAL)
    //POSITIVE("regression_static_user_threads", rtcore_regression(rtcore_regression_static_thread,1));
    //POSITIVE("regression_dynamic_user_threads", rtcore_regression(rtcore_regression_dynamic_thread,1));
#endif

#if defined(TASKING_TBB) || defined(TASKING_INTERNAL)
    //POSITIVE("regression_static_build_join", rtcore_regression(rtcore_regression_static_thread,2));
    //POSITIVE("regression_dynamic_build_join", rtcore_regression(rtcore_regression_dynamic_thread,2));
#endif
      
#if defined(TASKING_TBB) || defined(TASKING_INTERNAL)
    //POSITIVE("regression_static_memory_monitor",  rtcore_regression_memory_monitor(rtcore_regression_static_thread));
    //POSITIVE("regression_dynamic_memory_monitor", rtcore_regression_memory_monitor(rtcore_regression_dynamic_thread));
#endif

#if !defined(__MIC__)
    //POSITIVE("regression_garbage_geom",   rtcore_regression_garbage());
#endif

    /* register all command line options*/
    registerOption("rtcore", [this] (Ref<ParseStream> cin, const FileName& path) {
        rtcore += "," + cin->getString();
      }, "--rtcore <string>: uses <string> to configure Embree device");
    
    registerOption("threads", [this] (Ref<ParseStream> cin, const FileName& path) {
        rtcore += ",threads=" + toString(cin->getInt());
      }, "--threads <int>: number of threads to use");
    
    registerOption("affinity", [this] (Ref<ParseStream> cin, const FileName& path) {
        rtcore += ",set_affinity=1";
      }, "--affinity: affinitize threads");
    
    registerOption("verbose", [this] (Ref<ParseStream> cin, const FileName& path) {
        rtcore += ",verbose=" + toString(cin->getInt());
      }, "--verbose <int>: sets verbosity level");
    
    std::string run_docu = "--run testname: runs specified test, supported tests are:";
    for (auto test : tests) run_docu += "\n  " + test->name;
    registerOption("run", [this] (Ref<ParseStream> cin, const FileName& path) {
        std::string name = cin->getString();
        if (name2test.find(name) == name2test.end()) throw std::runtime_error("Unknown test: "+name);
        tests_to_run.push_back(name2test[name]);
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
    
    /* print Embree version */
    rtcInit("verbose=1");
    error_handler(rtcGetError());
    rtcExit();
    
    /* perform tests */
    device = rtcNewDevice(rtcore.c_str());
    error_handler(rtcDeviceGetError(device));

    /* execute specific user tests */
    if (tests_to_run.size()) {
      for (auto test : tests_to_run) runTest(test);
    } else {
      for (auto test : tests) runTest(test);
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
    std::cout << std::setw(30) << test->name << " ..." << std::flush;
    try {
      ok &= test->run(this);
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
