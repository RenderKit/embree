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

#include "../../../common/sys/platform.h"
#include "../../../common/sys/sysinfo.h"
#include "../../../common/sys/ref.h"
#include "../../../common/lexers/streamfilters.h"
#include "../../../common/lexers/parsestream.h"
#include "glutdisplay.h"
#include "../transport/transport_host.h"
#include "scene.h"
#include "tutorial_device.h"

#include <map>
#include <vector>

namespace embree
{
  class TutorialApplication
  {
  public:
    TutorialApplication (const std::string& tutorialName);
    
  public:
    
    struct CommandLineOption : public RefCount {
      virtual void parse(Ref<ParseStream> cin, const FileName& path) = 0;
      std::string description;
    };
    
    template<typename F>
      struct CommandLineOptionClosure : public CommandLineOption
    {
      CommandLineOptionClosure (std::string description, const F& f) 
        : description(description), f(f) {}
      
      virtual void parse(Ref<ParseStream> cin, const FileName& path) {
        f(cin,path);
      }
      
    public:
      std::string description;
      F f;
    };
    
    template<typename F>
      void registerOption(const std::string& name, const F& f, const std::string& description) 
    {
      Ref<CommandLineOption> closure = new CommandLineOptionClosure<F>(description,f);
      commandLineOptionList.push_back(closure);
      commandLineOptionMap[name] = closure;
    }
    
    void parseCommandLine(int argc, char** argv);
    void parseCommandLine(Ref<ParseStream> cin, const FileName& path);
    void printCommandLineHelp();

    void renderBenchmark(const FileName& fileName);
    void renderToFile(const FileName& fileName);
    virtual void main(int argc, char** argv);
    
  public:
    std::vector<         Ref<CommandLineOption> > commandLineOptionList;
    std::map<std::string,Ref<CommandLineOption> > commandLineOptionMap;
    
  public:
    std::string tutorialName;
    
    /* configuration */
    std::string g_rtcore;
    size_t g_numThreads;
    std::string g_subdiv_mode;
    
    /* output settings */
    size_t g_width;
    size_t g_height;
    bool g_fullscreen;
    FileName outFilename;
    int g_skipBenchmarkFrames;
    int g_numBenchmarkFrames;
    bool g_interactive;
    int g_instancing_mode;
    Shader g_shader;//  = SHADER_DEFAULT;
    bool convert_tris_to_quads;
    bool convert_bezier_to_lines;
    bool convert_hair_to_curves;
    
    /* scene */
    TutorialScene g_obj_scene;
    Ref<SceneGraph::GroupNode> g_scene;
    FileName filename;
    
    //std::vector<FileName> keyframeList;
    //std::vector<TutorialScene*> g_keyframes;
  };
}

#if defined __WIN32__
inline double drand48() {
  return (double)rand()/(double)RAND_MAX;
}

#endif
