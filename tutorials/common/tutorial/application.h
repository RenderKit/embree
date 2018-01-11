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

#pragma once

#include "../default.h"

namespace embree
{
  class Application
  {
  public:
    enum Features { FEATURE_RTCORE = 1, FEATURE_STREAM = 2 };

    Application (int features);
    
    /* virtual interface for command line option processing */
    struct CommandLineOption : public RefCount 
    {
    public:
      CommandLineOption (const std::string& description) 
        : description(description) {}

      virtual void parse(Ref<ParseStream> cin, const FileName& path) = 0;
      
    public:
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
      
    public:
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
  public:
    std::vector<         Ref<CommandLineOption> > commandLineOptionList;
    std::map<std::string,Ref<CommandLineOption> > commandLineOptionMap;
 
  public:

    /* virtual main function, contains all tutorial logic */
    virtual int main(int argc, char** argv) = 0;

  public:
    /* embree configuration */
    std::string rtcore;
  };
}
