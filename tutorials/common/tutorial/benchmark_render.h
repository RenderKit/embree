// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tutorial.h"
#include "statistics.h"
#include "benchmark.h"

#ifdef USE_GOOGLE_BENCHMARK
#include <benchmark/benchmark.h>
#endif

namespace embree
{

template<class Tutorial>
static void renderBenchmarkLegacy(BenchState& state, BenchParams& params ,int argc, char** argv);

template<class Tutorial>
static void renderBenchFunc(BenchState& state, BenchParams& params ,int argc, char** argv)
{
#ifdef USE_GOOGLE_BENCHMARK
  if (params.legacy) {
    renderBenchmarkLegacy<Tutorial>(state, params, argc, argv);
    return;
  }

  Tutorial tutorial;
  tutorial.interactive = false;
  tutorial.main(argc,argv);

  tutorial.resize(tutorial.width, tutorial.height);
  ISPCCamera ispccamera = tutorial.camera.getISPCCamera(tutorial.width, tutorial.height);

  for (size_t i = 0; i < params.skipIterations; i++)
  {
    tutorial.initRayStats();
    tutorial.render(tutorial.pixels,tutorial.width,tutorial.height,0.0f,ispccamera);
  }

  size_t numRays = 0;
  for (auto _ : *state.state)
  {
    tutorial.initRayStats();
    tutorial.render(tutorial.pixels,tutorial.width,tutorial.height,0.0f,ispccamera);
    numRays += tutorial.getNumRays();
  }

  state.state->SetItemsProcessed(state.state->iterations());
  state.state->counters["Rays/s"] = benchmark::Counter(numRays, benchmark::Counter::kIsRate);
#else
  renderBenchmarkLegacy<Tutorial>(state, params, argc, argv);
#endif
}

template<class Tutorial>
static void renderBenchmarkLegacy(BenchState& state, BenchParams& params ,int argc, char** argv)
{
  Tutorial tutorial;
  tutorial.interactive = false;
  tutorial.main(argc,argv);

  tutorial.resize(tutorial.width, tutorial.height);
  ISPCCamera ispccamera = tutorial.camera.getISPCCamera(tutorial.width, tutorial.height);

  IOStreamStateRestorer cout_state(std::cout);
  std::cout.setf(std::ios::fixed, std::ios::floatfield);
  std::cout.precision(4);

  //Statistics stat;
  FilteredStatistics fpsStat(0.5f,0.0f);
  FilteredStatistics mraypsStat(0.5f,0.0f);
  {
    size_t numTotalFrames = params.skipIterations + params.minTimeOrIterations;
    for (size_t i=0; i<params.skipIterations; i++)
    {
      tutorial.initRayStats();
      double t0 = getSeconds();
      tutorial.render(tutorial.pixels,tutorial.width,tutorial.height,0.0f,ispccamera);
      double t1 = getSeconds();
      std::cout << "frame [" << std::setw(3) << i << " / " << std::setw(3) << numTotalFrames << "]: " <<  std::setw(8) << 1.0/(t1-t0) << " fps (skipped)" << std::endl << std::flush;
    }

    for (size_t i=params.skipIterations; i<numTotalFrames; i++)
    {
      tutorial.initRayStats();
      double t0 = getSeconds();
      tutorial.render(tutorial.pixels,tutorial.width,tutorial.height,0.0f,ispccamera);
      double t1 = getSeconds();

      float fps = float(1.0/(t1-t0));
      fpsStat.add(fps);

      float mrayps = float(double(tutorial.getNumRays())/(1000000.0*(t1-t0)));
      mraypsStat.add(mrayps);

      if (numTotalFrames >= 1024 && (i % 64 == 0))
      {
        double rate = 0;
        if (fpsStat.getAvg()) rate = 100.0f*fpsStat.getSigma()/fpsStat.getAvg();

        std::cout << "frame [" << std::setw(3) << i << " / " << std::setw(3) << numTotalFrames << "]: "
                  << std::setw(8) << fps << " fps, "
                  << "min = " << std::setw(8) << fpsStat.getMin() << " fps, "
                  << "avg = " << std::setw(8) << fpsStat.getAvg() << " fps, "
                  << "max = " << std::setw(8) << fpsStat.getMax() << " fps, "
                  << "sigma = " << std::setw(6) << fpsStat.getSigma() << " (" << rate << "%)" << std::endl << std::flush;
      }
    }

    double rate = 0;
    if (fpsStat.getAvg()) rate = 100.0f*fpsStat.getAvgSigma()/fpsStat.getAvg();

    std::cout << "frame [" << std::setw(3) << params.skipIterations << " - " << std::setw(3) << numTotalFrames << "]: "
              << "              "
              << "min = " << std::setw(8) << fpsStat.getMin() << " fps, "
              << "avg = " << std::setw(8) << fpsStat.getAvg() << " fps, "
              << "max = " << std::setw(8) << fpsStat.getMax() << " fps, "
              << "sigma = " << std::setw(6) << fpsStat.getAvgSigma() << " (" << rate << "%)" << std::endl;
  }

  std::cout << "BENCHMARK_RENDER_MIN " << fpsStat.getMin() << std::endl;
  std::cout << "BENCHMARK_RENDER_AVG " << fpsStat.getAvg() << std::endl;
  std::cout << "BENCHMARK_RENDER_MAX " << fpsStat.getMax() << std::endl;
  std::cout << "BENCHMARK_RENDER_SIGMA " << fpsStat.getSigma() << std::endl;
  std::cout << "BENCHMARK_RENDER_AVG_SIGMA " << fpsStat.getAvgSigma() << std::endl;

#if defined(RAY_STATS)
  std::cout << "BENCHMARK_RENDER_MRAYPS_MIN " << mraypsStat.getMin() << std::endl;
  std::cout << "BENCHMARK_RENDER_MRAYPS_AVG " << mraypsStat.getAvg() << std::endl;
  std::cout << "BENCHMARK_RENDER_MRAYPS_MAX " << mraypsStat.getMax() << std::endl;
  std::cout << "BENCHMARK_RENDER_MRAYPS_SIGMA " << mraypsStat.getSigma() << std::endl;
  std::cout << "BENCHMARK_RENDER_MRAYPS_AVG_SIGMA " << mraypsStat.getAvgSigma() << std::endl;
#endif

  std::cout << std::flush;
}

}