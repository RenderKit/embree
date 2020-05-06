// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../common/tutorial/benchmark.h"

#include "../../include/embree3/rtcore_common.h"

namespace embree {
  struct ISPCScene;

#if defined(RTC_NAMESPACE)
  using RTC_NAMESPACE::RTCBuildQuality;
#endif

  void Benchmark_Dynamic_Update(BenchState& state, BenchParams& params, BuildBenchParams& buildParams, ISPCScene* ispc_scene, RTCBuildQuality quality = RTCBuildQuality::RTC_BUILD_QUALITY_LOW);
  void Benchmark_Dynamic_Create(BenchState& state, BenchParams& params, BuildBenchParams& buildParams, ISPCScene* ispc_scene, RTCBuildQuality quality);
  void Benchmark_Static_Create(BenchState& state, BenchParams& params, BuildBenchParams& buildParams, ISPCScene* ispc_scene, RTCBuildQuality quality, RTCBuildQuality qflags);
  void Benchmark_Static_Create_UserThreads(BenchState& state, BenchParams& params, BuildBenchParams& buildParams, ISPCScene* ispc_scene, RTCBuildQuality quality, RTCBuildQuality qflags);

  size_t getNumPrimitives(ISPCScene* scene_in);
}