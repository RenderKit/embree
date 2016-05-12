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

#include "../kernels/common/default.h"
#include "../kernels/common/ray.h"
#include "../include/embree2/rtcore.h"
#include "../include/embree2/rtcore_ray.h"
#include "rtcore_helpers.h"
#include "../tutorials/common/tutorial/application.h"

namespace embree
{
  class VerifyApplication : public Application
  {
  public:
    enum TestType { TEST_SHOULD_PASS, TEST_SHOULD_FAIL, TEST_GROUP };
    enum TestReturnValue { FAILED, PASSED, SKIPPED };
    
    struct Test : public RefCount
    {
      Test (std::string name, int isa, TestType ty) 
        : name(name), isa(isa), ty(ty), enabled(true) {}

      bool isEnabled() { return enabled; }
      virtual TestReturnValue run(VerifyApplication* state, bool silent) { return SKIPPED; }
      virtual TestReturnValue execute(VerifyApplication* state, bool silent);

    public:
      std::string name;
      int isa;
      TestType ty;
      bool enabled;
    };

    struct TestGroup : public Test
    {
      TestGroup (std::string name, bool silent = false)
        : Test(name,0,TEST_GROUP), silent(silent) {}

    public:
      void add(Ref<Test> test) {
        tests.push_back(test);
      }

      std::string extend_prefix(std::string prefix) const {
        return (name != "") ? prefix + name + "." : prefix;
      }
      
      bool isEnabled() { return enabled; }
      TestReturnValue execute(VerifyApplication* state, bool silent);

    public:
      bool silent;
      std::vector<Ref<Test>> tests;
    };

    struct IntersectTest : public Test
    {
      IntersectTest (std::string name, int isa, IntersectMode imode, IntersectVariant ivariant, TestType ty = TEST_SHOULD_PASS)
        : Test(name,isa,ty), imode(imode), ivariant(ivariant) {}

      bool supportsIntersectMode(RTCDevice device)
      { 
        switch (imode) {
        case MODE_INTERSECT_NONE: return true;
        case MODE_INTERSECT1:   return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT1);
        case MODE_INTERSECT4:   return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT4);
        case MODE_INTERSECT8:   return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT8);
        case MODE_INTERSECT16:  return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT16);
        case MODE_INTERSECT1M:  return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
        case MODE_INTERSECTNM1: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
        case MODE_INTERSECTNM3: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
        case MODE_INTERSECTNM4: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
        case MODE_INTERSECTNM8: return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
        case MODE_INTERSECTNM16:return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
        case MODE_INTERSECTNp:  return rtcDeviceGetParameter1i(device,RTC_CONFIG_INTERSECT_STREAM);
        }
        assert(false);
        return false;
      }

    public:
      IntersectMode imode;
      IntersectVariant ivariant;
    };
    
  public:

    VerifyApplication ();
    void prefix_test_names(Ref<Test> test, std::string prefix = "");
    void print_tests(Ref<Test> test, size_t depth);
    bool enable_disable_all_tests(Ref<Test> test, bool enabled);
    bool enable_disable_some_tests(Ref<Test> test, std::string regex, bool enabled);
    int main(int argc, char** argv);
    
  public:
    float intensity;
    std::atomic<size_t> numFailedTests;

  public:
    std::vector<int> isas;
    Ref<TestGroup> tests;

  public:
    std::vector<RTCSceneFlags> sceneFlags;
    std::vector<RTCSceneFlags> sceneFlagsRobust;
    std::vector<RTCSceneFlags> sceneFlagsDynamic;
    std::vector<IntersectMode> intersectModes;
    std::vector<IntersectVariant> intersectVariants;
    bool user_specified_tests;
    bool flatten;
    bool parallel;
  };
}
