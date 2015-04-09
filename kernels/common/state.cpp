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
#include "lexers/parsestream.h"

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

  /* error flag */
  tls_t State::g_error = NULL; // FIXME: use thread local
  std::vector<RTCError*> State::g_errors; // FIXME: use thread local
  MutexSys State::g_errors_mutex;

  State State::state;

  State::State () {
    clear();
  }
  
  void State::clear()
  {
    tri_accel = "default";
    tri_builder = "default";
    tri_traverser = "default";
    tri_builder_replication_factor = 2.0f;
    
    tri_accel_mb = "default";
    tri_builder_mb = "default";
    tri_traverser_mb = "default";
    
    hair_accel = "default";
    hair_builder = "default";
    hair_traverser = "default";
    hair_builder_replication_factor = 3.0f;
    memory_preallocation_factor     = 1.0f; 
    tessellation_cache_size         = 0;
    subdiv_accel = "default";
    
    scene_flags = -1;
    verbose = 0;
    g_numThreads = 0;
    benchmark = 0;
    regression_testing = 0;

    {
      Lock<MutexSys> lock(g_errors_mutex);
      if (g_error == NULL) 
        g_error = createTls();
    }
    g_error_function = NULL;
    g_memory_monitor_function = NULL;

    //Lock<MutexSys> lock(g_errors_mutex);
    //  for (size_t i=0; i<g_errors.size(); i++)
    //    delete g_errors[i];
    //  destroyTls(g_error);
    //  g_errors.clear();
  }

  void State::parse(const char* cfg)
  {
    if (cfg == NULL) return;

#if 0

    std::vector<std::string> symbols;
    symbols.push_back("=");
    Ref<ParseStream> stream = new TokenStream(new StrStream(cfg),
                                              TokenStream::alpha+TokenStream::ALPHA+TokenStream::numbers+"_",
                                              TokenStream::separators);

    std::string tag = cin->getString();
    if (tag == "") return;

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
        tri_accel = parseIdentifier (cfg,pos);
      else if ((tok == "tri_builder" || tok == "builder") && parseSymbol (cfg,'=',pos))
        tri_builder = parseIdentifier (cfg,pos);
      else if ((tok == "tri_traverser" || tok == "traverser") && parseSymbol (cfg,'=',pos))
        tri_traverser = parseIdentifier (cfg,pos);
      else if (tok == "tri_builder_replication_factor" && parseSymbol (cfg,'=',pos))
        tri_builder_replication_factor = parseInt (cfg,pos);
      
      else if ((tok == "tri_accel_mb" || tok == "accel_mb") && parseSymbol (cfg,'=',pos))
        tri_accel_mb = parseIdentifier (cfg,pos);
      else if ((tok == "tri_builder_mb" || tok == "builder_mb") && parseSymbol (cfg,'=',pos))
        tri_builder_mb = parseIdentifier (cfg,pos);
      else if ((tok == "tri_traverser_mb" || tok == "traverser_mb") && parseSymbol (cfg,'=',pos))
        tri_traverser_mb = parseIdentifier (cfg,pos);
      
      else if (tok == "hair_accel" && parseSymbol (cfg,'=',pos))
        hair_accel = parseIdentifier (cfg,pos);
      else if (tok == "hair_builder" && parseSymbol (cfg,'=',pos))
        hair_builder = parseIdentifier (cfg,pos);
      else if (tok == "hair_traverser" && parseSymbol (cfg,'=',pos))
        hair_traverser = parseIdentifier (cfg,pos);
      else if (tok == "hair_builder_replication_factor" && parseSymbol (cfg,'=',pos))
        hair_builder_replication_factor = parseInt (cfg,pos);
      
      else if (tok == "subdiv_accel" && parseSymbol (cfg,'=',pos))
        subdiv_accel = parseIdentifier (cfg,pos);
      
      else if (tok == "verbose" && parseSymbol (cfg,'=',pos))
        verbose = parseInt (cfg,pos);
      else if (tok == "benchmark" && parseSymbol (cfg,'=',pos))
        benchmark = parseInt (cfg,pos);
      
      else if (tok == "flags") {
        scene_flags = 0;
        if (parseSymbol (cfg,'=',pos)) {
          do {
            std::string flag = parseIdentifier (cfg,pos);
            if      (flag == "static" ) scene_flags |= RTC_SCENE_STATIC;
            else if (flag == "dynamic") scene_flags |= RTC_SCENE_DYNAMIC;
            else if (flag == "compact") scene_flags |= RTC_SCENE_COMPACT;
            else if (flag == "coherent") scene_flags |= RTC_SCENE_COHERENT;
            else if (flag == "incoherent") scene_flags |= RTC_SCENE_INCOHERENT;
            else if (flag == "high_quality") scene_flags |= RTC_SCENE_HIGH_QUALITY;
            else if (flag == "robust") scene_flags |= RTC_SCENE_ROBUST;
          } while (parseSymbol (cfg,',',pos));
        }
      }
      else if (tok == "memory_preallocation_factor" && parseSymbol (cfg,'=',pos)) 
        memory_preallocation_factor = parseFloat (cfg,pos);

      else if (tok == "regression" && parseSymbol (cfg,'=',pos)) 
        regression_testing = parseInt (cfg,pos);

      else if (tok == "tessellation_cache_size" && parseSymbol (cfg,'=',pos))
      {
        tessellation_cache_size = parseFloat (cfg,pos) * 1024 * 1024;
        
      }        
    } while (findNext (cfg,',',pos));

#else
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
        tri_accel = parseIdentifier (cfg,pos);
      else if ((tok == "tri_builder" || tok == "builder") && parseSymbol (cfg,'=',pos))
        tri_builder = parseIdentifier (cfg,pos);
      else if ((tok == "tri_traverser" || tok == "traverser") && parseSymbol (cfg,'=',pos))
        tri_traverser = parseIdentifier (cfg,pos);
      else if (tok == "tri_builder_replication_factor" && parseSymbol (cfg,'=',pos))
        tri_builder_replication_factor = parseInt (cfg,pos);
      
      else if ((tok == "tri_accel_mb" || tok == "accel_mb") && parseSymbol (cfg,'=',pos))
        tri_accel_mb = parseIdentifier (cfg,pos);
      else if ((tok == "tri_builder_mb" || tok == "builder_mb") && parseSymbol (cfg,'=',pos))
        tri_builder_mb = parseIdentifier (cfg,pos);
      else if ((tok == "tri_traverser_mb" || tok == "traverser_mb") && parseSymbol (cfg,'=',pos))
        tri_traverser_mb = parseIdentifier (cfg,pos);
      
      else if (tok == "hair_accel" && parseSymbol (cfg,'=',pos))
        hair_accel = parseIdentifier (cfg,pos);
      else if (tok == "hair_builder" && parseSymbol (cfg,'=',pos))
        hair_builder = parseIdentifier (cfg,pos);
      else if (tok == "hair_traverser" && parseSymbol (cfg,'=',pos))
        hair_traverser = parseIdentifier (cfg,pos);
      else if (tok == "hair_builder_replication_factor" && parseSymbol (cfg,'=',pos))
        hair_builder_replication_factor = parseInt (cfg,pos);
      
      else if (tok == "subdiv_accel" && parseSymbol (cfg,'=',pos))
        subdiv_accel = parseIdentifier (cfg,pos);
      
      else if (tok == "verbose" && parseSymbol (cfg,'=',pos))
        verbose = parseInt (cfg,pos);
      else if (tok == "benchmark" && parseSymbol (cfg,'=',pos))
        benchmark = parseInt (cfg,pos);
      
      else if (tok == "flags") {
        scene_flags = 0;
        if (parseSymbol (cfg,'=',pos)) {
          do {
            std::string flag = parseIdentifier (cfg,pos);
            if      (flag == "static" ) scene_flags |= RTC_SCENE_STATIC;
            else if (flag == "dynamic") scene_flags |= RTC_SCENE_DYNAMIC;
            else if (flag == "compact") scene_flags |= RTC_SCENE_COMPACT;
            else if (flag == "coherent") scene_flags |= RTC_SCENE_COHERENT;
            else if (flag == "incoherent") scene_flags |= RTC_SCENE_INCOHERENT;
            else if (flag == "high_quality") scene_flags |= RTC_SCENE_HIGH_QUALITY;
            else if (flag == "robust") scene_flags |= RTC_SCENE_ROBUST;
          } while (parseSymbol (cfg,',',pos));
        }
      }
      else if (tok == "memory_preallocation_factor" && parseSymbol (cfg,'=',pos)) 
        memory_preallocation_factor = parseFloat (cfg,pos);

      else if (tok == "regression" && parseSymbol (cfg,'=',pos)) 
        regression_testing = parseInt (cfg,pos);

      else if (tok == "tessellation_cache_size" && parseSymbol (cfg,'=',pos))
      {
        tessellation_cache_size = parseFloat (cfg,pos) * 1024 * 1024;
        
      }        
    } while (findNext (cfg,',',pos));
#endif
  }

  RTCError* State::error() 
  {
    RTCError* stored_error = (RTCError*) getTls(g_error);
    if (stored_error == NULL) {
      Lock<MutexSys> lock(g_errors_mutex);
      stored_error = new RTCError(RTC_NO_ERROR);
      g_errors.push_back(stored_error);
      setTls(g_error,stored_error);
    }
    return stored_error;
  }

  bool State::verbosity(int N) {
    return N <= verbose;
  }
  
  void State::print()
  {
    std::cout << "general:" << std::endl;
    std::cout << "  build threads = " << g_numThreads << std::endl;
    std::cout << "  verbosity     = " << verbose << std::endl;
    
    std::cout << "triangles:" << std::endl;
    std::cout << "  accel         = " << tri_accel << std::endl;
    std::cout << "  builder       = " << tri_builder << std::endl;
    std::cout << "  traverser     = " << tri_traverser << std::endl;
    std::cout << "  replications  = " << tri_builder_replication_factor << std::endl;
    
    std::cout << "motion blur triangles:" << std::endl;
    std::cout << "  accel         = " << tri_accel_mb << std::endl;
    std::cout << "  builder       = " << tri_builder_mb << std::endl;
    std::cout << "  traverser     = " << tri_traverser_mb << std::endl;
    
    std::cout << "hair:" << std::endl;
    std::cout << "  accel         = " << hair_accel << std::endl;
    std::cout << "  builder       = " << hair_builder << std::endl;
    std::cout << "  traverser     = " << hair_traverser << std::endl;
    std::cout << "  replications  = " << hair_builder_replication_factor << std::endl;
    
    std::cout << "subdivision surfaces:" << std::endl;
    std::cout << "  accel         = " << subdiv_accel << std::endl;
    
#if defined(__MIC__)
    std::cout << "memory allocation:" << std::endl;
    std::cout << "  preallocation_factor  = " << memory_preallocation_factor << std::endl;
#endif
  }
}
