// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../default.h"

namespace embree
{
  void waitForKeyPressedUnderWindows();
  
  struct CommandLineParser
  {
    enum Verbosity {
      NORMAL = 0,
      SILENT
    };
    
    CommandLineParser(Verbosity verbosity = NORMAL) : verbosity(verbosity) {}

    /* virtual interface for command line option processing */
    struct CommandLineOption : public RefCount 
    {
      CommandLineOption (const std::string& description) 
        : description(description) {}

      virtual void parse(Ref<ParseStream> cin, const FileName& path) = 0;
      
      std::string description;
    };

    /* helper class to provide parsing function via lambda function */
    template<typename F>
    struct CommandLineOptionClosure : public CommandLineOption
    {
      CommandLineOptionClosure (std::string description, const F& f) 
        : CommandLineOption(description), f(f) {}
      
      virtual void parse(Ref<ParseStream> cin, const FileName& path) {
        f(cin,path);
      }
      
      F f;
    };

    /* registers a command line option */
    template<typename F>
      void registerOption(const std::string& name, const F& f, const std::string& description) 
    {
      Ref<CommandLineOption> closure = new CommandLineOptionClosure<F>(description,f);
      commandLineOptionList.push_back(closure);
      commandLineOptionMap[name] = closure;
    }

    /* registers an alias for a command line option */
    void registerOptionAlias(const std::string& name, const std::string& alternativeName); 
    
    /* command line parsing */
    void parseCommandLine(int argc, char** argv);
    void parseCommandLine(Ref<ParseStream> cin, const FileName& path);

    /* prints help for all supported command line options */
    void printCommandLineHelp();

    /* command line options database */
    std::vector<         Ref<CommandLineOption> > commandLineOptionList;
    std::map<std::string,Ref<CommandLineOption> > commandLineOptionMap;
    
    Verbosity verbosity;
  };
 
  class Application
  {
  public:
    enum Features { FEATURE_RTCORE = 1, FEATURE_STREAM = 2 };

    Application (int features);
    virtual ~Application();

    static Application* instance;
    
    /* print log message */
    void log(int verbose, const std::string& str);
    
    template<typename F>
    void registerOption(const std::string& name, const F& f, const std::string& description) {
      commandLineParser.registerOption<F>(name, f, description);
    }
    
    /* registers an alias for a command line option */
    void registerOptionAlias(const std::string& name, const std::string& alternativeName) {
      commandLineParser.registerOptionAlias(name, alternativeName);
    }
    
    /* command line parsing */
    void parseCommandLine(int argc, char** argv) {
      commandLineParser.parseCommandLine(argc, argv);
    }

    void parseCommandLine(Ref<ParseStream> cin, const FileName& path) {
      commandLineParser.parseCommandLine(cin, path);
    }

    /* prints help for all supported command line options */
    void printCommandLineHelp() {
      commandLineParser.printCommandLineHelp();
    }
 
  public:

    /* virtual main function, contains all tutorial logic */
    virtual int main(int argc, char** argv) = 0;

  public:
    std::string rtcore;      // embree configuration
    int verbosity;           // verbosity of output
    
    CommandLineParser commandLineParser;

  public:
    bool log_delta;       // whether to print delta stats
    double start_time;       // start time of application
    double last_time;        // last log time of application
    ssize_t last_virtual_memory; // last virtual memory usage
    ssize_t last_resident_memory; // last resident memory usage
  };
}
