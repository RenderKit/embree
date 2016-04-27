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

#include "application.h"

namespace embree
{
  Application::Application()
  {
    registerOption("help", [this] (Ref<ParseStream> cin, const FileName& path) {
        printCommandLineHelp();
        exit(1);
      }, "--help: prints help for all supported command line options");
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
    for (auto c : commandLineOptionList) {
      std::cout << c->description << std::endl;
    }
  }
}
