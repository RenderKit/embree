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

  struct EmptySceneTest : public VerifyApplication::Test
  {
    EmptySceneTest (std::string name, RTCSceneFlags sflags)
      : VerifyApplication::Test(name,VerifyApplication::PASS), sflags(sflags) {}

    void run(VerifyApplication* state)
    {
      RTCSceneRef scene = rtcDeviceNewScene(state->device,sflags,aflags);
      AssertNoError(state->device);
      rtcCommit (scene);
      AssertNoError(state->device);
    }

  public:
    RTCSceneFlags sflags;
  };

  struct MultipleDevicesTest : public VerifyApplication::Test
  {
    MultipleDevicesTest (std::string name)
      : VerifyApplication::Test(name,VerifyApplication::PASS) {}

    void run(VerifyApplication* state)
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
    }
  };

  VerifyApplication::VerifyApplication ()
    : device(nullptr), rtcore(""), regressionN(200)
  {
    /* add all tests */
    addTest(new EmptySceneTest("empty_static",RTC_SCENE_STATIC));
    addTest(new MultipleDevicesTest("multiple_devices"));

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
      for (auto test : tests_to_run) test->run(this);
    } else {
      for (auto test : tests) test->run(this);
    }

    rtcDeleteDevice(device);
    return 0;
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
  app.main(argc,argv);
}
