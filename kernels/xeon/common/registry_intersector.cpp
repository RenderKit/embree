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

#include "registry_intersector.h"
#include <algorithm>

// FIXME: remove this file

namespace embree
{
  template<typename T>
  void IntersectorRegistry<T>::add(int cpu_features, const std::string& name, const T intersector) 
  {   
    table[name].push_back(IntersectorImplementation(cpu_features,intersector));
    std::sort(table[name].begin(),table[name].end(),IntersectorRegistry<T>::IntersectorImplementation::compare);
  }
    
  template<typename T>
  T IntersectorRegistry<T>::get(std::string name) 
  {
    /*! find specified intersector */
    if (table.find(name) == table.end())
      throw std::runtime_error("cannot find any implementation of " + std::string(T::type) + " " +name);
    
    /*! find implementation that matches CPU features */
    int cpu_features = getCPUFeatures();
    const std::vector<IntersectorImplementation>& implementations = table[name];
    for (size_t i=0; i<implementations.size(); i++) 
    {
      if ((implementations[i].cpu_features & cpu_features) == implementations[i].cpu_features) 
      {
        if (g_verbose > 0)
          std::cout << "using " << T::type << " " << name << " optimized for " << stringOfISA(implementations[i].cpu_features) << std::endl;

        return implementations[i].intersector;
      }
    }
    
    /*! fatal error, we did not find a matching implementation */
    throw std::runtime_error("cannot find implementation of " + std::string(T::type) + " " +name+" for CPU with features "+stringOfCPUFeatures(cpu_features));
    return T();
  }

  template<typename T>
  void IntersectorRegistry<T>::print() 
  {
    std::cout << T::type << " registry:" << std::endl;
    for (typename std::map<std::string, std::vector<IntersectorImplementation> >::iterator i=table.begin(); i != table.end(); i++) 
    {
      const std::string name = i->first;
      const std::vector<IntersectorImplementation>& implementations = i->second;
      std::cout << "  " << name << " (";
      
      for (size_t j=0; j<implementations.size(); j++) {
        if (j) std::cout << ", ";
        std::cout << stringOfISA(implementations[j].cpu_features);
      }
      std::cout << ")" << std::endl;
    } 
  }

  template<typename T>
  void IntersectorRegistry<T>::clear() {
    table.clear();
  }
  
  /*! intersector registries */
  /*IntersectorRegistry<Accel::Intersector1> intersectors1;
  IntersectorRegistry<Accel::Intersector4> intersectors4;
  IntersectorRegistry<Accel::Intersector8> intersectors8;
  IntersectorRegistry<Accel::Intersector16> intersectors16;*/

  /*! name of intersectors */
  const char* Accel::Intersector1::type = "intersector1";
  const char* Accel::Intersector4::type = "intersector4";
  const char* Accel::Intersector8::type = "intersector8";
  const char* Accel::Intersector16::type = "intersector16";

  /*! explicit template instantiations */
  template class IntersectorRegistry<Accel::Intersector1>;
  template class IntersectorRegistry<Accel::Intersector4>;
  template class IntersectorRegistry<Accel::Intersector8>;
  template class IntersectorRegistry<Accel::Intersector16>;
}
