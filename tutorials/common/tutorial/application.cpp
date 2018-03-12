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

#include "application.h"

namespace embree
{
  Application* Application::instance = nullptr;
  
  Application::Application(int features)
    : rtcore("start_threads=1,set_affinity=1"), verbosity(0),
      log_delta(false),
      start_time(getSeconds()),
      last_time(start_time),
      last_virtual_memory(0),
      last_resident_memory(0)
  {
    if (instance)
      throw std::runtime_error("internal error: applicaton already created");

    instance = this;

    registerOption("help", [this] (Ref<ParseStream> cin, const FileName& path) {
        printCommandLineHelp();
        exit(1);
      }, "--help: prints help for all supported command line options");

    if (features & FEATURE_RTCORE)
    {
      registerOption("rtcore", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += "," + cin->getString();
        }, "--rtcore <string>: uses <string> to configure Embree device");
    
      registerOption("threads", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += ",threads=" + toString(cin->getInt());
        }, "--threads <int>: number of threads to use");
      
      registerOption("affinity", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += ",set_affinity=1";
        }, "--affinity: affinitize threads");

      registerOption("set_affinity", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += ",set_affinity=" + cin->getString();
        }, "--set_affinity <0/1>: enables or disables affinitizing of threads");
      registerOptionAlias("set_affinity","set-affinity");

      registerOption("start_threads", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += ",start_threads=" + cin->getString();
        }, "--start_threads <0/1>: starts threads at device creation time if set to 1");
      registerOptionAlias("start_threads","start-threads");
      
      registerOption("verbose", [this] (Ref<ParseStream> cin, const FileName& path) {
          verbosity = cin->getInt();
          rtcore += ",verbose=" + toString(verbosity);
        }, "--verbose <int>: sets verbosity level");

      registerOption("delta", [this] (Ref<ParseStream> cin, const FileName& path) {
          log_delta = true;
        }, "--delta: print delta numbers in log");
      
      registerOption("isa", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += ",isa=" + cin->getString();
        }, "--isa <string>: selects instruction set to use:\n"
        "  sse: select SSE codepath\n"
        "  sse2: select SSE2 codepath\n"
        "  sse3: select SSE3 codepath\n"
        "  ssse3: select SSSE3 codepath\n"
        "  sse4.1: select SSE4.1 codepath\n"
        "  sse4.2: select SSE4.2 codepath\n"
        "  avx: select AVX codepath\n"
        "  avxi: select AVXI codepath\n"
        "  avx2: select AVX2 codepath\n"
        "  avx512knl: select AVX512 codepath for KNL\n"
        "  avx512skx: select AVX512 codepath for SKX\n");
    } 
  }

  Application::~Application()
  {
    assert(instance == this);
    instance = nullptr;
  }
  
  void Application::registerOptionAlias(const std::string& name, const std::string& alternativeName) {
    commandLineOptionMap[alternativeName] = commandLineOptionMap[name];
  }
  
  void Application::parseCommandLine(int argc, char** argv)
  {
    /* create stream for parsing */
    Ref<ParseStream> stream = new ParseStream(new CommandLineStream(argc, argv));
    
    /* parse command line */  
    parseCommandLine(stream, FileName());
  }
  
  void Application::parseCommandLine(Ref<ParseStream> cin, const FileName& path)
  {
    while (true)
    {
      std::string tag = cin->getString();
      if (tag == "") return;
      std::string tag0 = tag;
      
      /* remove - or -- and lookup command line option */
      if (tag.find("-") == 0) 
      {
        tag = tag.substr(1);
        if (tag.find("-") == 0) tag = tag.substr(1);
        auto option = commandLineOptionMap.find(tag);
      
        /* process command line option */
        if (option != commandLineOptionMap.end()) {
          option->second->parse(cin,path);
          continue;
        }
      }
      
      /* handle unknown command line options */
      std::cerr << "unknown command line parameter: " << tag0 << " ";
      while (cin->peek() != "" && cin->peek()[0] != '-') std::cerr << cin->getString() << " ";
      std::cerr << std::endl;
    }
  }

  void Application::printCommandLineHelp()
  {
    for (auto& c : commandLineOptionList) {
      std::cout << c->description << std::endl;
    }
  }

  void Application::log(int verbose, const std::string& str)
  {
    if (verbosity < verbose)
      return;
    
    double time = getSeconds();
    ssize_t virtual_memory = getVirtualMemoryBytes();
    ssize_t resident_memory = getResidentMemoryBytes();

    double log_time = log_delta ? time-last_time : time-start_time;
    ssize_t log_virtual_memory = log_delta ? virtual_memory-last_virtual_memory : virtual_memory;
    ssize_t log_resident_memory = log_delta ? resident_memory-last_resident_memory : resident_memory;
      
    std::cout << "[ "
              << std::setw(8) << std::setprecision(3) << std::fixed << log_time << "s, "
              << std::setw(8) << std::setprecision(2) << std::fixed << double(log_virtual_memory)/1E6 << " MB virtual, "
              << std::setw(8) << std::setprecision(2) << std::fixed << double(log_resident_memory)/1E6 << " MB resident ] "
              << str << std::fixed 
              << std::endl << std::flush;
    
    last_time = time;
    last_virtual_memory = virtual_memory;
    last_resident_memory = resident_memory;
  }
}
