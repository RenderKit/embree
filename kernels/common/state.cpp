// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "state.h"

namespace embree
{
  void skipSpace(const char* str, size_t& pos) {
    while (str[pos] == ' ') pos++;
  }

  int parseInt(const char* str, size_t& pos) 
  {
    skipSpace(str,pos);
    size_t begin = pos;
    while (isdigit(str[pos])) pos++;
    return atoi(str+begin);
  }

  float parseFloat(const char* str, size_t& pos) 
  {
    skipSpace(str,pos);
    size_t begin = pos;
    while (isdigit(str[pos]) || str[pos] == '.') pos++;
    return atof(str+begin);
  }

  std::string parseIdentifier(const char* str, size_t& pos) 
  {
    skipSpace(str,pos);
    size_t begin = pos;
    while (isalnum(str[pos]) || str[pos] == '_' || str[pos] == '.') pos++;
    return std::string(str+begin,str+pos);
  }

  bool parseSymbol(const char* str, char c, size_t& pos) 
  {
    skipSpace(str,pos);
    if (str[pos] == c) { pos++; return true; }
    return false;
  }

  bool findNext(const char* str, char c, size_t& pos) 
  {
    while (str[pos] && str[pos] != c) pos++;
    if (str[pos] == c) { pos++; return true; }
    else return false;
  }

  State State::state;
  
  State::State () {
    clear();
  }
  
  void State::clear()
  {
    g_tri_accel = "default";                 //!< acceleration structure to use for triangles
    g_tri_builder = "default";               //!< builder to use for triangles
    g_tri_traverser = "default";             //!< traverser to use for triangles
    g_tri_builder_replication_factor = 2.0f; //!< maximally factor*N many primitives in accel
    
    g_tri_accel_mb = "default";              //!< acceleration structure to use for motion blur triangles
    g_tri_builder_mb = "default";            //!< builder to use for motion blur triangles
    g_tri_traverser_mb = "default";          //!< traverser to use for triangles
    
    g_hair_accel = "default";                 //!< hair acceleration structure to use
    g_hair_builder = "default";               //!< builder to use for hair
    g_hair_traverser = "default";             //!< traverser to use for hair
    g_hair_builder_replication_factor = 3.0f; //!< maximally factor*N many primitives in accel
    g_memory_preallocation_factor     = 1.0f; 
    g_tessellation_cache_size         = 0;    //!< size of the shared tessellation cache 
    g_subdiv_accel = "default";               //!< acceleration structure to use for subdivision surfaces
    
    g_scene_flags = -1;                               //!< scene flags to use
    g_verbose = 0;                                 //!< verbosity of output
    g_numThreads = 0;                              //!< number of threads to use in builders
    g_benchmark = 0;
    g_regression_testing = 0;                      //!< enables regression tests at startup
  }

  void State::parse(const char* cfg)
  {
    if (cfg == NULL) return;

    size_t pos = 0;
    do {
      std::string tok = parseIdentifier (cfg,pos);
      
      if (tok == "threads" && parseSymbol(cfg,'=',pos)) 
        g_numThreads = parseInt(cfg,pos);

      else if (tok == "isa" && parseSymbol (cfg,'=',pos)) 
      {
        std::string isa = parseIdentifier (cfg,pos);
        if      (isa == "sse" ) setCPUFeatures(SSE);
        else if (isa == "sse2") setCPUFeatures(SSE2);
        else if (isa == "sse3") setCPUFeatures(SSE3);
        else if (isa == "ssse3") setCPUFeatures(SSSE3);
        else if (isa == "sse41") setCPUFeatures(SSE41);
        else if (isa == "sse4.1") setCPUFeatures(SSE41);
        else if (isa == "sse42") setCPUFeatures(SSE42);
        else if (isa == "sse4.2") setCPUFeatures(SSE42);
        else if (isa == "avx") setCPUFeatures(AVX);
        else if (isa == "avxi") setCPUFeatures(AVXI);
        else if (isa == "avx2") setCPUFeatures(AVX2);
      }
      
      else if ((tok == "tri_accel" || tok == "accel") && parseSymbol (cfg,'=',pos))
        g_tri_accel = parseIdentifier (cfg,pos);
      else if ((tok == "tri_builder" || tok == "builder") && parseSymbol (cfg,'=',pos))
        g_tri_builder = parseIdentifier (cfg,pos);
      else if ((tok == "tri_traverser" || tok == "traverser") && parseSymbol (cfg,'=',pos))
        g_tri_traverser = parseIdentifier (cfg,pos);
      else if (tok == "tri_builder_replication_factor" && parseSymbol (cfg,'=',pos))
        g_tri_builder_replication_factor = parseInt (cfg,pos);
      
      else if ((tok == "tri_accel_mb" || tok == "accel_mb") && parseSymbol (cfg,'=',pos))
        g_tri_accel_mb = parseIdentifier (cfg,pos);
      else if ((tok == "tri_builder_mb" || tok == "builder_mb") && parseSymbol (cfg,'=',pos))
        g_tri_builder_mb = parseIdentifier (cfg,pos);
      else if ((tok == "tri_traverser_mb" || tok == "traverser_mb") && parseSymbol (cfg,'=',pos))
        g_tri_traverser_mb = parseIdentifier (cfg,pos);
      
      else if (tok == "hair_accel" && parseSymbol (cfg,'=',pos))
        g_hair_accel = parseIdentifier (cfg,pos);
      else if (tok == "hair_builder" && parseSymbol (cfg,'=',pos))
        g_hair_builder = parseIdentifier (cfg,pos);
      else if (tok == "hair_traverser" && parseSymbol (cfg,'=',pos))
        g_hair_traverser = parseIdentifier (cfg,pos);
      else if (tok == "hair_builder_replication_factor" && parseSymbol (cfg,'=',pos))
        g_hair_builder_replication_factor = parseInt (cfg,pos);
      
      else if (tok == "subdiv_accel" && parseSymbol (cfg,'=',pos))
        g_subdiv_accel = parseIdentifier (cfg,pos);
      
      else if (tok == "verbose" && parseSymbol (cfg,'=',pos))
        g_verbose = parseInt (cfg,pos);
      else if (tok == "benchmark" && parseSymbol (cfg,'=',pos))
        g_benchmark = parseInt (cfg,pos);
      
      else if (tok == "flags") {
        g_scene_flags = 0;
        if (parseSymbol (cfg,'=',pos)) {
          do {
            std::string flag = parseIdentifier (cfg,pos);
            if      (flag == "static" ) g_scene_flags |= RTC_SCENE_STATIC;
            else if (flag == "dynamic") g_scene_flags |= RTC_SCENE_DYNAMIC;
            else if (flag == "compact") g_scene_flags |= RTC_SCENE_COMPACT;
            else if (flag == "coherent") g_scene_flags |= RTC_SCENE_COHERENT;
            else if (flag == "incoherent") g_scene_flags |= RTC_SCENE_INCOHERENT;
            else if (flag == "high_quality") g_scene_flags |= RTC_SCENE_HIGH_QUALITY;
            else if (flag == "robust") g_scene_flags |= RTC_SCENE_ROBUST;
          } while (parseSymbol (cfg,',',pos));
        }
      }
      else if (tok == "memory_preallocation_factor" && parseSymbol (cfg,'=',pos)) 
        g_memory_preallocation_factor = parseFloat (cfg,pos);

      else if (tok == "regression" && parseSymbol (cfg,'=',pos)) 
        g_regression_testing = parseInt (cfg,pos);

      else if (tok == "tessellation_cache_size" && parseSymbol (cfg,'=',pos))
      {
        g_tessellation_cache_size = parseFloat (cfg,pos) * 1024 * 1024;
        
      }        
    } while (findNext (cfg,',',pos));
  }
  
  bool State::verbosity(int N) {
    return N <= g_verbose;
  }
  
  void State::print()
  {
    std::cout << "general:" << std::endl;
    std::cout << "  build threads = " << g_numThreads << std::endl;
    std::cout << "  verbosity     = " << g_verbose << std::endl;
    
    std::cout << "triangles:" << std::endl;
    std::cout << "  accel         = " << g_tri_accel << std::endl;
    std::cout << "  builder       = " << g_tri_builder << std::endl;
    std::cout << "  traverser     = " << g_tri_traverser << std::endl;
    std::cout << "  replications  = " << g_tri_builder_replication_factor << std::endl;
    
    std::cout << "motion blur triangles:" << std::endl;
    std::cout << "  accel         = " << g_tri_accel_mb << std::endl;
    std::cout << "  builder       = " << g_tri_builder_mb << std::endl;
    std::cout << "  traverser     = " << g_tri_traverser_mb << std::endl;
    
    std::cout << "hair:" << std::endl;
    std::cout << "  accel         = " << g_hair_accel << std::endl;
    std::cout << "  builder       = " << g_hair_builder << std::endl;
    std::cout << "  traverser     = " << g_hair_traverser << std::endl;
    std::cout << "  replications  = " << g_hair_builder_replication_factor << std::endl;
    
    std::cout << "subdivision surfaces:" << std::endl;
    std::cout << "  accel         = " << g_subdiv_accel << std::endl;
    
#if defined(__MIC__)
    std::cout << "memory allocation:" << std::endl;
    std::cout << "  preallocation_factor  = " << g_memory_preallocation_factor << std::endl;
#endif
  }
}
