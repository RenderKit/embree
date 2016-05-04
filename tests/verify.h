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
    enum TestType { PASS, FAIL, GROUP_BEGIN, GROUP_END };
    enum TestReturnValue { FAILED, PASSED, SKIPPED };
    
    struct Test : public RefCount
    {
      Test (std::string name, int isa, TestType ty) 
        : name(name), isa(isa), ty(ty), enabled(false) {}

      bool isEnabled() { return enabled; }
      virtual TestReturnValue run(VerifyApplication* state) { return SKIPPED; };

    public:
      std::string name;
      int isa;
      TestType ty;
      bool enabled;
    };

    struct IntersectTest : public Test
    {
      IntersectTest (std::string name, int isa, IntersectMode imode, IntersectVariant ivariant, TestType ty = PASS)
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
      }

    public:
      IntersectMode imode;
      IntersectVariant ivariant;
    };
    
  public:

    VerifyApplication ();

    void addTest(Ref<Test> test);
    void beginTestGroup(std::string name) { addTest(new Test(name,0,GROUP_BEGIN)); }
    void endTestGroup  () { addTest(new Test("",0,GROUP_END)); }
    bool runTest(Ref<Test> test, bool silent);
    void runTestGroup(size_t& id);

    int main(int argc, char** argv);
    
  public:
    float intensity;
    size_t numFailedTests;

  public:
    std::vector<int> isas;
    std::vector<Ref<Test>> tests;
    std::map<std::string,Ref<Test>> name2test;

  public:
    std::vector<RTCSceneFlags> sceneFlags;
    std::vector<RTCSceneFlags> sceneFlagsRobust;
    std::vector<RTCSceneFlags> sceneFlagsDynamic;
    std::vector<IntersectMode> intersectModes;
    std::vector<IntersectVariant> intersectVariants;
    bool user_specified_tests;
    bool use_groups;
  };
}
