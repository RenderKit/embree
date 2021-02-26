// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "application.h"

namespace benchmark
{
  struct State;
}

namespace embree {

enum BuildBenchType {
  UPDATE_DYNAMIC_DEFORMABLE = 1,
  UPDATE_DYNAMIC_DYNAMIC = 2,
  UPDATE_DYNAMIC_STATIC = 4,
  CREATE_DYNAMIC_DEFORMABLE = 8,
  CREATE_DYNAMIC_DYNAMIC = 16,
  CREATE_DYNAMIC_STATIC = 32,
  CREATE_STATIC_STATIC = 64,
  CREATE_HIGH_QUALITY_STATIC_STATIC = 128,
  CREATE_USER_THREADS_STATIC_STATIC = 256,
  ALL = 511
};

static MAYBE_UNUSED BuildBenchType getBuildBenchType(std::string const& str)
{
  if      (str == "update_dynamic_deformable")         return BuildBenchType::UPDATE_DYNAMIC_DEFORMABLE;
  else if (str == "update_dynamic_dynamic")            return BuildBenchType::UPDATE_DYNAMIC_DYNAMIC;
  else if (str == "update_dynamic_static")             return BuildBenchType::UPDATE_DYNAMIC_STATIC;
  else if (str == "create_dynamic_deformable")         return BuildBenchType::CREATE_DYNAMIC_DEFORMABLE;
  else if (str == "create_dynamic_dynamic")            return BuildBenchType::CREATE_DYNAMIC_DYNAMIC;
  else if (str == "create_dynamic_static")             return BuildBenchType::CREATE_DYNAMIC_STATIC;
  else if (str == "create_static_static")              return BuildBenchType::CREATE_STATIC_STATIC;
  else if (str == "create_high_quality_static_static") return BuildBenchType::CREATE_HIGH_QUALITY_STATIC_STATIC;
  else if (str == "create_user_threads_static_static") return BuildBenchType::CREATE_USER_THREADS_STATIC_STATIC;
  return BuildBenchType::ALL;
}

static MAYBE_UNUSED std::string getBuildBenchTypeString(BuildBenchType type)
{
  if      (type == BuildBenchType::UPDATE_DYNAMIC_DEFORMABLE)         return "update_dynamic_deformable";
  else if (type == BuildBenchType::UPDATE_DYNAMIC_DYNAMIC)            return "update_dynamic_dynamic";
  else if (type == BuildBenchType::UPDATE_DYNAMIC_STATIC)             return "update_dynamic_static";
  else if (type == BuildBenchType::CREATE_DYNAMIC_DEFORMABLE)         return "create_dynamic_deformable";
  else if (type == BuildBenchType::CREATE_DYNAMIC_DYNAMIC)            return "create_dynamic_dynamic";
  else if (type == BuildBenchType::CREATE_DYNAMIC_STATIC)             return "create_dynamic_static";
  else if (type == BuildBenchType::CREATE_STATIC_STATIC)              return "create_static_static";
  else if (type == BuildBenchType::CREATE_HIGH_QUALITY_STATIC_STATIC) return "create_high_quality_static_static";
  else if (type == BuildBenchType::CREATE_USER_THREADS_STATIC_STATIC) return "create_user_threads_static_static";
  return "all";
}

struct BenchState {
  benchmark::State* state;
};

struct BenchParams {
  int skipIterations = 0;
  int minTimeOrIterations = 0;
  int repetitions = 0;
  bool legacy = false;
  std::string name = "";
};

struct BuildBenchParams {
  BuildBenchType buildBenchType = BuildBenchType::ALL;
  int userThreads = 0;
};

using BenchFunc = void(*)(BenchState& state, BenchParams& params, int argc, char** argv);
using BuildBenchFunc = void(*)(BenchState& state, BenchParams& params, BuildBenchParams& buildParams, int argc, char** argv);

struct TutorialBenchmark
{
  static bool benchmark(int argc, char** argv) {
    for (int i = 0; i < argc; ++i)
      if (std::string(argv[i]) == "--benchmark")
        return true;
    return false;
  }

  TutorialBenchmark(BenchFunc func) :
    commandLineParser(CommandLineParser(CommandLineParser::SILENT)),
    func(func)
  {
    commandLineParser.registerOption("help", [this] (Ref<ParseStream> cin, const FileName& path) {
        commandLineParser.printCommandLineHelp();
        exit(1);
      }, "--help: prints help for all supported command line options");
    commandLineParser.registerOption("i", [&] (Ref<ParseStream> cin, const FileName& path) {
        inputFile = cin->getString();
        processedCommandLineOptions.push_back("-i");
        processedCommandLineOptions.push_back("-c");
      }, "-i <filepath>: .xml or .ecs");
    commandLineParser.registerOptionAlias("i", "c");
    commandLineParser.registerOption("benchmark", [&] (Ref<ParseStream> cin, const FileName& path) {
        params.skipIterations = cin->getInt();
        params.minTimeOrIterations = cin->getInt();
        processedCommandLineOptions.push_back("--benchmark");
      }, "--benchmark <N> <M>: run benchmark for M seconds (M iterations in legacy mode) with N iterations warmup");
    commandLineParser.registerOption("legacy", [&] (Ref<ParseStream> cin, const FileName& path) {
        params.legacy = true;
        processedCommandLineOptions.push_back("--legacy");
      }, "--legacy: run old benchmark version (without google benchmark)");
    commandLineParser.registerOption("benchmark_repetitions", [&] (Ref<ParseStream> cin, const FileName& path) {
        params.repetitions = cin->getInt();
        processedCommandLineOptions.push_back("--benchmark_repetitions");
      }, "--benchmark_repetitions <R>: run R repetitions when using google benchmark");
    commandLineParser.registerOption("benchmark_name", [&] (Ref<ParseStream> cin, const FileName& path) {
        params.name = cin->getString();
        processedCommandLineOptions.push_back("--benchmark_name");
      }, "--benchmark_name <string>: override name of the benchmark");
  }

  TutorialBenchmark() : TutorialBenchmark(nullptr)
  {
  }

  int main(int argc, char** argv, std::string name = "default");

protected:
  CommandLineParser commandLineParser;

  // remove the processed commdand line options
  void updateCommandLine(int& argc, char** argv);
  virtual void postParseCommandLine();

  std::string inputFile = "";
  std::vector<std::string> processedCommandLineOptions;
  BenchParams params;

  virtual void registerBenchmark(std::string const& name, int argc, char** argv);

private:
  BenchFunc func;
};

struct TutorialBuildBenchmark : public TutorialBenchmark
{
  TutorialBuildBenchmark(BuildBenchFunc func) : TutorialBenchmark(), buildBenchFunc(func)
  {
    commandLineParser.registerOption("benchmark_type", [&] (Ref<ParseStream> cin, const FileName& path) {
        std::string str = cin->getString();
        buildParams.buildBenchType = getBuildBenchType(str);
        processedCommandLineOptions.push_back("--benchmark_type");
      }, "--benchmark_type <string>: select which build types to benchmark");
    commandLineParser.registerOption("user_threads", [this] (Ref<ParseStream> cin, const FileName& path) {
        buildParams.userThreads = cin->getInt();
      }, "--user_threads <int>: invokes user thread benchmark with specified number of application provided build threads");
  }

private:
  void postParseCommandLine() override;

  void registerBenchmark(std::string const& name, int argc, char** argv) override;
  void registerBuildBenchmark(std::string name, BuildBenchType buildBenchType, int argc, char** argv);

  BuildBenchParams buildParams;
  BuildBenchFunc buildBenchFunc;
};

}