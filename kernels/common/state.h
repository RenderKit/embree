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

#pragma once

#include "default.h"

namespace embree
{
  extern size_t g_numThreads; // FIXME: move into state?

  struct State
  {
  public:
    /*! state construction */
    State ();

    /*! clears the state to its defaults */
    void clear();

    /*! parses state from a configuration file */
    bool parseFile(const FileName& fileName);

    /*! parses the state from a string */
    void parseString(const char* cfg);

    /*! parses the state from a stream */
    void parse(Ref<TokenStream> cin);

    /*! prints the state */
    void print();

    /*! checks if verbosity level is at least N */
    bool verbosity(int N);

    /*! returns single state instance */
    static __forceinline State* instance() {
      return &state;
    }

    /*! returns thread local error code */
    static RTCError* error();

  private:
    static State state;                      //!< single state object

  public:
    std::string tri_accel;                 //!< acceleration structure to use for triangles
    std::string tri_builder;               //!< builder to use for triangles
    std::string tri_traverser;             //!< traverser to use for triangles
    double      tri_builder_replication_factor; //!< maximally factor*N many primitives in accel

  public:
    std::string tri_accel_mb;              //!< acceleration structure to use for motion blur triangles
    std::string tri_builder_mb;            //!< builder to use for motion blur triangles
    std::string tri_traverser_mb;          //!< traverser to use for triangles

  public:
    std::string hair_accel;                //!< hair acceleration structure to use
    std::string hair_builder;              //!< builder to use for hair
    std::string hair_traverser;            //!< traverser to use for hair
    double      hair_builder_replication_factor; //!< maximally factor*N many primitives in accel

  public:
    float       memory_preallocation_factor; 
    size_t      tessellation_cache_size;   //!< size of the shared tessellation cache 
    std::string subdiv_accel;              //!< acceleration structure to use for subdivision surfaces

  public:
    bool float_exceptions;                 //!< enable floating point exceptions
    int scene_flags;                       //!< scene flags to use
    size_t verbose;                        //!< verbosity of output
    //size_t numThreads;                   //!< number of threads to use in builders
    size_t benchmark;                      //!< true
    size_t regression_testing;             //!< enables regression tests at startup

  public:
    static tls_t g_error; // FIXME: use thread local
    static std::vector<RTCError*> g_errors; // FIXME: use thread local
    static MutexSys g_errors_mutex;

  public:
    RTCErrorFunc g_error_function;
    RTCMemoryMonitorFunc g_memory_monitor_function;
  };
}
