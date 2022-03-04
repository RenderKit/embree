// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <iterator>
#include <sstream>
#include <fstream>
#include <memory>

#include "benchmark.h"

#ifdef USE_GOOGLE_BENCHMARK
#include <benchmark/benchmark.h>
#endif

namespace embree {

//////////////////////////////////////////////
// util functions for command line handling //
//////////////////////////////////////////////
struct CommandLine
{
  typedef std::unordered_set<std::unique_ptr<std::string>> StringPool;
  typedef std::unordered_set<std::unique_ptr<std::vector<char*>>> CommandLinePool;

  static StringPool string_pool;
  static CommandLinePool command_line_pool;

  CommandLine() : cl(nullptr) { }

  CommandLine(int argc, char** argv) : cl(nullptr)
  {
    auto cl_iter = command_line_pool.emplace(std::unique_ptr<std::vector<char*>>(new std::vector<char*>()));
    cl = cl_iter.first->get();
    for (size_t i = 0; i < argc; ++i) {
      auto pair = string_pool.emplace(std::unique_ptr<std::string>{new std::string(argv[i])});
      cl->emplace_back(&(*pair.first->get())[0]);
    }
  }

  void add(std::vector<std::string> const& args)
  {
    for (std::string const& str : args) {
      auto pair = string_pool.emplace(std::unique_ptr<std::string>{new std::string(str)});
      cl->emplace_back(&(*pair.first->get())[0]);
    }
  }

  int argc() { return cl->size(); }
  char** argv() { return cl->data(); }

private:
  std::vector<char*>* cl;
};

CommandLine::StringPool CommandLine::string_pool;
CommandLine::CommandLinePool CommandLine::command_line_pool;

bool endsWith(std::string const &str, std::string const &suffix)
{
  if (str.length() <= suffix.length())
    return false;
  return str.substr(str.length() - suffix.length(), str.length() - 1) == suffix;
}

bool startsWith(std::string const &str, std::string const &prefix)
{
  if (str.length() <= prefix.length())
    return false;
  return (str.rfind(prefix, 0) == 0);
}

std::string removeQuotes(std::string const &str)
{
  std::string res = str;
  if (startsWith(res, "\""))
    res = res.substr(1, res.length());
  if (endsWith(res, "\""))
    res = res.substr(0, res.length() - 1);
  return res;
}

std::string getFileName(std::string const &path)
{
  const size_t b = (path.rfind("/") == std::string::npos) ? 0 : path.rfind("/") + 1;
  return path.substr(b, path.rfind(".") - b);
}

void printCommandLine(int argc, char** argv)
{
  for (int i = 0; i < argc; ++i)
      std::cout << argv[i] << (i < argc-1 ? " " : "");
  if (argc > 0) std::cout <<  std::endl;
}
//////////////////////////////////////////////
//////////////////////////////////////////////

int TutorialBenchmark::main(int argc, char** argv, std::string name)
{
  commandLineParser.parseCommandLine(argc, argv);
  updateCommandLine(argc, argv);

  CommandLine commandLine(argc, argv);

#if USE_GOOGLE_BENCHMARK
  if (!params.legacy && params.minTimeOrIterations > 0)
    commandLine.add({"--benchmark_min_time=" + std::to_string(params.minTimeOrIterations)});
  if (!params.legacy && params.repetitions > 0)
    commandLine.add({"--benchmark_repetitions=" + std::to_string(params.repetitions)});
#endif

  argc = commandLine.argc();
  argv = commandLine.argv();

#ifdef USE_GOOGLE_BENCHMARK
  if (!params.legacy)
    ::benchmark::Initialize(&argc, argv);
#endif

  commandLine = CommandLine(argc, argv);

  postParseCommandLine();

  std::string benchmark_name = name;
  if (endsWith(inputFile, ".xml")) {
    benchmark_name = getFileName(inputFile);
  }
  else if (endsWith(inputFile, ".ecs")) {
    benchmark_name = getFileName(inputFile);
  }

  if (params.name != "")
    benchmark_name = params.name;

  registerBenchmark(benchmark_name, commandLine.argc(), commandLine.argv());

#ifdef USE_GOOGLE_BENCHMARK
  if (!params.legacy)
    ::benchmark::RunSpecifiedBenchmarks();
#endif

  return 0;
}

// remove the processed commdand line options
void TutorialBenchmark::updateCommandLine(int& argc, char** argv)
{
  for (std::string const& str : processedCommandLineOptions)
  {
    for (int i = 0; i < argc; ++i) {
      if (std::string(argv[i]) == str) {
        int remove = 1;
        int j = i+1;
        while(j < argc && !startsWith(std::string(argv[j]), "-")) {
          remove++;
          j++;
        }
        argc -= remove;
        for (j = i; j < argc; ++j) {
          argv[j] = argv[j+remove];
        }
      }
    }
  }
}

void TutorialBenchmark::postParseCommandLine()
{

}

void callBenchFunc(benchmark::State& state, int argc, char** argv, BenchParams benchParams, BenchFunc benchFunc)
{
  BenchState benchState;
  benchState.state = &state;
  benchFunc(benchState, benchParams, argc, argv);
}

void TutorialBenchmark::registerBenchmark(std::string const& name, int argc, char** argv)
{
#ifdef USE_GOOGLE_BENCHMARK
  if (params.legacy) {
    std::cout << "BENCHMARK SCENE: " << name << std::endl;
    BenchState benchState;
    func(benchState, params, argc, argv);
  }
  else {
    ::benchmark::RegisterBenchmark(name.c_str(), callBenchFunc, argc, argv, params, func)->Unit(::benchmark::TimeUnit::kMillisecond);
  }
#else
  std::cout << "BENCHMARK SCENE: " << name << std::endl;
  BenchState benchState;
  func(benchState, params, argc, argv);
#endif
}

void callBuildBenchFunc(benchmark::State& state, int argc, char** argv, BenchParams benchParams, BuildBenchParams buildBenchParams, BuildBenchFunc buildBenchFunc)
{
  BenchState benchState;
  benchState.state = &state;
  buildBenchFunc(benchState, benchParams, buildBenchParams, argc, argv);
}

void TutorialBuildBenchmark::registerBuildBenchmark(std::string name, BuildBenchType buildBenchType, int argc, char** argv)
{
  if (buildParams.buildBenchType & buildBenchType) {
    // attach benchmark name if more than one bit in buildBenchMask is set
    bool attach = (buildParams.buildBenchType & (buildParams.buildBenchType - 1)) != 0;
    if (attach) name += "_" + getBuildBenchTypeString(buildBenchType);
    BuildBenchParams p = buildParams;
    p.buildBenchType = buildBenchType;
#ifdef USE_GOOGLE_BENCHMARK
    if (params.legacy) {
      std::cout << "BENCHMARK SCENE: " << name << std::endl;
      BenchState benchState;
      buildBenchFunc(benchState, params, p, argc, argv);
    }
    else {
      ::benchmark::RegisterBenchmark(name.c_str(), callBuildBenchFunc, argc, argv, params, p, buildBenchFunc)->Unit(::benchmark::TimeUnit::kMillisecond);
    }
#else
    std::cout << "BENCHMARK SCENE: " << name << std::endl;
    BenchState benchState;
    buildBenchFunc(benchState, params, p, argc, argv);
#endif
  }
}

void TutorialBuildBenchmark::registerBenchmark(std::string const& name, int argc, char** argv)
{
  registerBuildBenchmark(name, BuildBenchType::UPDATE_DYNAMIC_DEFORMABLE,         argc, argv);
  registerBuildBenchmark(name, BuildBenchType::UPDATE_DYNAMIC_DYNAMIC,            argc, argv);
  registerBuildBenchmark(name, BuildBenchType::UPDATE_DYNAMIC_STATIC,             argc, argv);
  registerBuildBenchmark(name, BuildBenchType::CREATE_DYNAMIC_DEFORMABLE,         argc, argv);
  registerBuildBenchmark(name, BuildBenchType::CREATE_DYNAMIC_DYNAMIC,            argc, argv);
  registerBuildBenchmark(name, BuildBenchType::CREATE_DYNAMIC_STATIC,             argc, argv);
  registerBuildBenchmark(name, BuildBenchType::CREATE_STATIC_STATIC,              argc, argv);
  registerBuildBenchmark(name, BuildBenchType::CREATE_HIGH_QUALITY_STATIC_STATIC, argc, argv);
  registerBuildBenchmark(name, BuildBenchType::CREATE_USER_THREADS_STATIC_STATIC, argc, argv);
}

void TutorialBuildBenchmark::postParseCommandLine()
{
  if (buildParams.userThreads > 0) {
    buildParams.buildBenchType = BuildBenchType::CREATE_USER_THREADS_STATIC_STATIC;
  } else {
    buildParams.buildBenchType = (BuildBenchType)(buildParams.buildBenchType & ~(BuildBenchType::CREATE_USER_THREADS_STATIC_STATIC));
  }
}

}