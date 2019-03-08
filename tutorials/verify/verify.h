// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

/* we include the Embree headers the very first to make sure they
 * always compile without any internal Embree specific stuff. */
#include "../../include/embree3/rtcore.h"
#include "../../include/embree3/rtcore_ray.h"
RTC_NAMESPACE_OPEN

/* now we include all Embree internal files we need for testing */
#include "../../kernels/common/default.h"
#include "../../kernels/common/ray.h"
#include "rtcore_helpers.h"
#include "../common/tutorial/application.h"
#include "../common/math/random_sampler.h"
#include "../common/tutorial/statistics.h"

namespace embree
{
  class VerifyApplication : public Application
  {
  public:
    enum TestType { TEST_SHOULD_PASS, TEST_SHOULD_FAIL, TEST_GROUP, BENCHMARK };
    enum TestReturnValue { FAILED, PASSED, SKIPPED };
    
    struct Test : public RefCount
    {
      Test (std::string name, int isa, TestType ty, bool enabled = true) 
        : name(name), isa(isa), ty(ty), enabled(enabled), ignoreFailure(false) 
      {
        RandomSampler_init(sampler,0x23F67E21);
      }

      bool   random_bool  () { return RandomSampler_getInt  (sampler) % 2; }
      float  random_float () { return RandomSampler_getFloat(sampler); }
      int    random_int   () { return RandomSampler_getInt  (sampler); }
      Vec3fa random_Vec3fa() { return RandomSampler_get3D   (sampler); }

      avector<Vec3fa> random_motion_vector2(float f = 1.0f)
      {
        avector<Vec3fa> motion_vector(2);
        motion_vector[0] = f*random_Vec3fa();
        motion_vector[1] = f*random_Vec3fa();
        return motion_vector;
      }

      avector<Vec3fa> random_motion_vector(float f = 1.0f)
      {
        float v = random_float();
        size_t numTimeSteps = clamp(int(v*v*9),2,8); // samples small number of time steps more frequently

        avector<Vec3fa> motion_vector(numTimeSteps);
        for (size_t i=0; i<numTimeSteps; i++)
          motion_vector[i] = f*random_Vec3fa();
        return motion_vector;
      }

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
      Benchmark (const std::string& name, int isa, const std::string& unit, bool higher_is_better, size_t max_attempts)
        : Test(name,isa,BENCHMARK,false), unit(unit), numThreads(getNumberOfLogicalThreads()), higher_is_better(higher_is_better), max_attempts(max_attempts) {}
      
      virtual size_t setNumPrimitives(size_t N) { return 0; }
      virtual void setNumThreads(size_t N) 
      { 
        if (N == 0) numThreads = getNumberOfLogicalThreads(); 
        else numThreads = N; 
      }
      virtual bool setup(VerifyApplication* state) { return true; }
      virtual float benchmark(VerifyApplication* state) = 0;
      Statistics benchmark_loop(VerifyApplication* state);
      virtual void cleanup(VerifyApplication* state) {}
      virtual TestReturnValue execute(VerifyApplication* state, bool silent);
      double readDatabase(VerifyApplication* state);
      void updateDatabase(VerifyApplication* state, Statistics stat, double bestAvg);
      void plotDatabase(VerifyApplication* state);

    public:
      size_t numThreads;
      bool higher_is_better;
      size_t max_attempts;
    };

    struct TestGroup : public Test
    {
      TestGroup (std::string name, bool silent, bool parallel, bool enabled = true)
        : Test(name,0,TEST_GROUP,enabled), silent(silent), parallel(parallel) {}

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
    void print_ctests(Ref<Test> test, size_t depth);
    template<typename Function> 
      void map_tests(Ref<Test> test, const Function& f);
    void enable_disable_all_tests(Ref<Test> test, bool enabled);
    size_t enable_disable_some_tests(Ref<Test> test, std::string regex, bool enabled);
     template<typename Closure>
       void plot(std::vector<Ref<Benchmark>> benchmarks, const FileName outFileName, std::string xlabel, size_t startN, size_t endN, float f, size_t dn, const Closure& test);
    FileName parse_benchmark_list(Ref<ParseStream> cin, std::vector<Ref<Benchmark>>& benchmarks);
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
    std::map<std::string,Ref<Test>> name2test;

  public:
    RTCDeviceRef device;
    std::vector<SceneFlags> sceneFlags;
    std::vector<SceneFlags> sceneFlagsRobust;
    std::vector<SceneFlags> sceneFlagsDynamic;
    std::vector<IntersectMode> intersectModes;
    std::vector<IntersectVariant> intersectVariants;
    bool user_specified_tests;
    bool flatten;
    bool parallel;
    bool cdash;
    FileName database;
    bool update_database;
    float benchmark_tolerance;

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
