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

#pragma once

#include "../default.h"
#include "tutorial_device.h"

namespace embree
{
  /* camera */
  extern Camera g_camera;

  class TutorialApplication
  {
  public:
    TutorialApplication (const std::string& tutorialName);
    
  public:
    
    struct CommandLineOption : public RefCount 
    {
    public:
      CommandLineOption (const std::string& description) 
        : description(description) {}

      virtual void parse(Ref<ParseStream> cin, const FileName& path) = 0;
      
    public:
      std::string description;
    };
    
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
    
    template<typename F>
      void registerOption(const std::string& name, const F& f, const std::string& description) 
    {
      Ref<CommandLineOption> closure = new CommandLineOptionClosure<F>(description,f);
      commandLineOptionList.push_back(closure);
      commandLineOptionMap[name] = closure;
    }

    void registerAlternativeOption(const std::string& name, const std::string& alternativeName); 
    
    void parseCommandLine(int argc, char** argv);
    void parseCommandLine(Ref<ParseStream> cin, const FileName& path);
    void printCommandLineHelp();

    void renderBenchmark(const FileName& fileName);
    void renderToFile(const FileName& fileName);
    virtual int main(int argc, char** argv);
    virtual void postParseCommandLine() {}
    
  public:
    std::vector<         Ref<CommandLineOption> > commandLineOptionList;
    std::map<std::string,Ref<CommandLineOption> > commandLineOptionMap;
    
  public:
    std::string tutorialName;
    
    /* configuration */
    std::string rtcore;
    size_t numThreads;
    std::string subdiv_mode;
    
    /* output settings */
    size_t width;
    size_t height;
    bool fullscreen;
    FileName outFilename;
    int skipBenchmarkFrames;
    int numBenchmarkFrames;
    bool interactive;
    int instancing_mode;
    Shader shader;
    bool convert_tris_to_quads;
    bool convert_bezier_to_lines;
    bool convert_hair_to_curves;
    
    /* scene */
    TutorialScene obj_scene;
    Ref<SceneGraph::GroupNode> scene;
    FileName filename;
  };
}

#if defined __WIN32__
inline double drand48() {
  return (double)rand()/(double)RAND_MAX;
}

#endif
