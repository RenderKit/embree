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
#include "../tutorials/common/math/random_sampler.h"

namespace embree
{
  class VerifyApplication : public Application
  {
  public:
    enum TestType { TEST_SHOULD_PASS, TEST_SHOULD_FAIL, TEST_GROUP, BENCHMARK };
    enum TestReturnValue { FAILED, PASSED, SKIPPED };
    
    struct Test : public RefCount
    {
      Test (std::string name, int isa, TestType ty) 
        : name(name), isa(isa), ty(ty), enabled(true), ignoreFailure(false) 
      {
        RandomSampler_init(sampler,0x23F67E21);
      }

      float  random_float () { return RandomSampler_getFloat(sampler); }
      int    random_int   () { return RandomSampler_getInt  (sampler); }
      Vec3fa random_Vec3fa() { return RandomSampler_get3D   (sampler); }
      bool isEnabled() { return enabled; }
      virtual TestReturnValue run(VerifyApplication* state, bool silent) { return SKIPPED; }
      virtual TestReturnValue execute(VerifyApplication* state, bool silent);

    public:
      std::string name;
      int isa;
      TestType ty;
      bool enabled;
      bool ignoreFailure;
      RandomSampler sampler;
    };

    class Benchmark : public Test
    {
    public:
      const std::string unit;
      Benchmark (const std::string& name, int isa, const std::string& unit)
        : Test(name,isa,BENCHMARK), unit(unit) {}
      
      virtual bool setup(VerifyApplication* state) { return true; }
      virtual double benchmark(VerifyApplication* state) = 0;
      virtual void cleanup(VerifyApplication* state) {}
      virtual TestReturnValue execute(VerifyApplication* state, bool silent);
    };

    struct TestGroup : public Test
    {
      TestGroup (std::string name, bool silent, bool parallel)
        : Test(name,0,TEST_GROUP), silent(silent), parallel(parallel) {}

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
      bool parallel;
      std::vector<Ref<Test>> tests;
    };

    struct IntersectTest : public Test
    {
      IntersectTest (std::string name, int isa, IntersectMode imode, IntersectVariant ivariant, TestType ty = TEST_SHOULD_PASS)
        : Test(name,isa,ty), imode(imode), ivariant(ivariant) {}

    public:
      IntersectMode imode;
      IntersectVariant ivariant;
    };
    
  public:

    VerifyApplication ();
    void prefix_test_names(Ref<Test> test, std::string prefix = "");
    bool update_tests(Ref<Test> test);
    void print_tests(Ref<Test> test, size_t depth);
    template<typename Function> 
      void map_tests(Ref<Test> test, const Function& f);
    void enable_disable_all_tests(Ref<Test> test, bool enabled);
    void enable_disable_some_tests(Ref<Test> test, std::string regex, bool enabled);
    int main(int argc, char** argv);
    
  public:
    float intensity;
    std::atomic<size_t> numPassedTests;
    std::atomic<size_t> numFailedTests;
    std::atomic<size_t> numFailedAndIgnoredTests;

  public:
    MutexSys mutex;
    std::vector<int> isas;
    Ref<Test> tests;

  public:
    RTCDeviceRef device;
    std::vector<RTCSceneFlags> sceneFlags;
    std::vector<RTCSceneFlags> sceneFlagsRobust;
    std::vector<RTCSceneFlags> sceneFlagsDynamic;
    std::vector<IntersectMode> intersectModes;
    std::vector<IntersectVariant> intersectVariants;
    bool user_specified_tests;
    bool flatten;
    bool parallel;

    /* sets terminal colors */
  public:
    std::string green(std::string str) {
      if (usecolors) return "\033[32m" + str + "\033[0m";
      else           return str;
    }
    std::string yellow(std::string str) {
      if (usecolors) return "\033[33m" + str + "\033[0m";
      else           return str;
    }
    std::string red(std::string str) {
      if (usecolors) return "\033[31m" + str + "\033[0m";
      else           return str;
    }
    bool usecolors;
  };
}
