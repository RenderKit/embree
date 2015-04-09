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

    /*! parses the state from a string */
    void parse(const char* cfg);

    /*! prints the state */
    void print();

    /*! checks if verbosity level is at least N */
    bool verbosity(int N);

    /*! returns single state instance */
    static __forceinline State* instance() {
      return &state;
    }

  private:
    static State state;                      //!< single state object

  public:
    std::string g_tri_accel;                 //!< acceleration structure to use for triangles
    std::string g_tri_builder;               //!< builder to use for triangles
    std::string g_tri_traverser;             //!< traverser to use for triangles
    double      g_tri_builder_replication_factor; //!< maximally factor*N many primitives in accel

  public:
    std::string g_tri_accel_mb;              //!< acceleration structure to use for motion blur triangles
    std::string g_tri_builder_mb;            //!< builder to use for motion blur triangles
    std::string g_tri_traverser_mb;          //!< traverser to use for triangles

  public:
    std::string g_hair_accel;                //!< hair acceleration structure to use
    std::string g_hair_builder;              //!< builder to use for hair
    std::string g_hair_traverser;            //!< traverser to use for hair
    double      g_hair_builder_replication_factor; //!< maximally factor*N many primitives in accel

  public:
    float       g_memory_preallocation_factor; 
    size_t      g_tessellation_cache_size;   //!< size of the shared tessellation cache 
    std::string g_subdiv_accel;              //!< acceleration structure to use for subdivision surfaces

  public:
    int g_scene_flags;                       //!< scene flags to use
    size_t g_verbose;                        //!< verbosity of output
    //size_t g_numThreads;                   //!< number of threads to use in builders
    size_t g_benchmark;                      //!< true
    size_t g_regression_testing;             //!< enables regression tests at startup
  };
}
