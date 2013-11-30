// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "registry_builder.h"
#include <algorithm>

namespace embree
{
  void BuilderRegistry::add(int cpu_features, const std::string& name, BuilderConstruction create, const size_t minLeafSize, const size_t maxLeafSize) 
  { 
    table[name].push_back(BuildClosure(cpu_features,create,minLeafSize,maxLeafSize));
    std::sort(table[name].begin(),table[name].end(),BuilderRegistry::BuildClosure::compare);
  }

  Builder* BuilderRegistry::get(std::string name, void* accel, BuildSource* source, void* geometry) 
  {
    /*! find specified builder */
    if (table.find(name) == table.end()) 
      throw std::runtime_error("unknown builder \""+name+"\".");
    
    /*! find implementation that matches CPU features */
    int cpu_features = getCPUFeatures();

    const std::vector<BuildClosure>& implementations = table[name];
    for (size_t i=0; i<implementations.size(); i++) 
    {
      if ((implementations[i].cpu_features & cpu_features) == implementations[i].cpu_features) 
      {
        if (g_verbose > 0)
          std::cout << "using builder " << name << " optimized for " << stringOfISA(implementations[i].cpu_features) << std::endl;

        const BuildClosure& closure = implementations[i];
        return closure.create(accel,source,geometry,closure.minLeafSize,closure.maxLeafSize);
      }
    }
    
    /*! fatal error, we did not find a matching implementation */
    throw std::runtime_error("cannot find implementation of builder "+name+" for CPU with features "+stringOfCPUFeatures(cpu_features));
  }

  void BuilderRegistry::print() 
  {
    std::cout << "builder registry:" << std::endl;
    for (std::map<std::string, std::vector<BuildClosure> >::iterator i=table.begin(); i != table.end(); i++) 
    {
      const std::string name = i->first;
      const std::vector<BuildClosure>& implementations = i->second;
      std::cout << "  " << name << " (";
      
      for (size_t j=0; j<implementations.size(); j++) {
        if (j) std::cout << ", ";
        std::cout << stringOfISA(implementations[j].cpu_features);
      }
      std::cout << ")" << std::endl;
    } 
  }

  void BuilderRegistry::clear() {
    table.clear();
  }

  /*! builder registry */
  BuilderRegistry builders; 
}

