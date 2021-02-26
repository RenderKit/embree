// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "buildbench.h"

#include "../common/tutorial/tutorial.h"

#include <iostream>
#include <unordered_map>
#include <iterator>

RTC_NAMESPACE_USE;

namespace embree
{
  uint32_t g_num_user_threads = 0;

  struct Tutorial : public SceneLoadingTutorialApplication
  {
    Tutorial()
      : SceneLoadingTutorialApplication("build_bench",FEATURE_RTCORE)
    {
      interactive = false;

      registerOption("user_threads", [this] (Ref<ParseStream> cin, const FileName& path) {
          g_num_user_threads = cin->getInt();
          rtcore += ",threads=" + toString(g_num_user_threads);
          rtcore += ",user_threads=" + toString(g_num_user_threads);
          rtcore += ",start_threads=0,set_affinity=0";
        }, "--user_threads <int>: invokes user thread benchmark with specified number of application provided build threads");
    }

    void postParseCommandLine() override
    {
      /* load default scene if none specified */
      if (sceneFilename.size() == 0) {
        FileName file = FileName::executableFolder() + FileName("models/cornell_box.ecs");
        parseCommandLine(new ParseStream(new LineCommentFilter(file, "#")), file.path());
      }
    }
  };

  std::unique_ptr<Tutorial> tutorial {};

  static void buildBench(BenchState& state, BenchParams& params, BuildBenchParams& buildParams, int argc, char** argv)
  {
    if (!tutorial) {
      tutorial.reset(new Tutorial());
      tutorial->main(argc,argv);
    }

    if (buildParams.userThreads == 0)
    {
      /* set error handler */
      if (buildParams.buildBenchType & BuildBenchType::UPDATE_DYNAMIC_DEFORMABLE) {
        Benchmark_Dynamic_Update(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_REFIT);
      }
      if (buildParams.buildBenchType & BuildBenchType::UPDATE_DYNAMIC_DYNAMIC) {
        Benchmark_Dynamic_Update(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_LOW);
      }
      if (buildParams.buildBenchType & BuildBenchType::UPDATE_DYNAMIC_STATIC) {
        Benchmark_Dynamic_Update(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_MEDIUM);
      }
      if (buildParams.buildBenchType & BuildBenchType::CREATE_DYNAMIC_DEFORMABLE) {
        Benchmark_Dynamic_Create(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_REFIT);
      }
      if (buildParams.buildBenchType & BuildBenchType::CREATE_DYNAMIC_DYNAMIC) {
        Benchmark_Dynamic_Create(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_LOW);
      }
      if (buildParams.buildBenchType & BuildBenchType::CREATE_DYNAMIC_STATIC) {
        Benchmark_Dynamic_Create(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_MEDIUM);
      }
      if (buildParams.buildBenchType & BuildBenchType::CREATE_STATIC_STATIC) {
        Benchmark_Static_Create(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_MEDIUM,RTC_BUILD_QUALITY_MEDIUM);
      }
      if (buildParams.buildBenchType & BuildBenchType::CREATE_HIGH_QUALITY_STATIC_STATIC) {
        Benchmark_Static_Create(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_MEDIUM,RTC_BUILD_QUALITY_HIGH);
      }
    }
    else
    {
      if (buildParams.buildBenchType & BuildBenchType::CREATE_USER_THREADS_STATIC_STATIC) {
        Benchmark_Static_Create_UserThreads(state, params, buildParams, tutorial->ispc_scene.get(), RTC_BUILD_QUALITY_MEDIUM,RTC_BUILD_QUALITY_MEDIUM);
      }
    }
  }
}

int main(int argc, char **argv)
{
  return embree::TutorialBuildBenchmark(embree::buildBench).main(argc, argv);
}