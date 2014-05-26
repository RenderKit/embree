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

#pragma once

#include "common/scene.h"
#include "primrefalloc.h"
#include "primrefblock.h"
#include "bvh4hair/heuristic_fallback.h"

namespace embree
{
  namespace isa
  {
    /*! Generates a list of triangle build primitives from the scene. */
    class TriRefListGen
    {
      static const size_t maxTasks = 32;
      typedef atomic_set<PrimRefBlockT<PrimRef> > TriRefList;

    public:      
      static void generate(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const Scene* scene, GeometryTy ty, size_t numTimeSteps, TriRefList& prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor that schedules the task */
      TriRefListGen (size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const Scene* scene, GeometryTy ty, size_t numTimeSteps, TriRefList& prims, PrimInfo& pinfo);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(TriRefListGen,task_gen_parallel);
      
      /* input data */
    private:
      const Scene* scene;                  //!< input geometry
      GeometryTy ty;                       //!< types of geometry to generate
      size_t numTimeSteps;                 //!< number of timesteps to generate
      PrimRefBlockAlloc<PrimRef>* alloc;   //!< allocator for build primitive blocks
      size_t numPrimitives;
      
      /* intermediate data */
    private:
      TaskScheduler::Task task;
      PrimInfo pinfos[maxTasks];
      
      /* output data */
    public:
      TriRefList& prims;             //!< list of build primitives
      PrimInfo& pinfo;                  //!< bounding information of primitives
    };

    /*! Generates a list of triangle build primitives from some triangle mesh. */
    class TriRefListGenFromTriangleMesh
    {
      static const size_t maxTasks = 32;
      typedef atomic_set<PrimRefBlockT<PrimRef> > TriRefList;

    public:      
      static void generate(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const TriangleMesh* mesh, TriRefList& prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor that schedules the task */
      TriRefListGenFromTriangleMesh (size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const TriangleMesh* mesh, TriRefList& prims, PrimInfo& pinfo);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(TriRefListGenFromTriangleMesh,task_gen_parallel);
      
      /* input data */
    private:
      const TriangleMesh* mesh;            //!< input geometry
      PrimRefBlockAlloc<PrimRef>* alloc;   //!< allocator for build primitive blocks
      
      /* intermediate data */
    private:
      TaskScheduler::Task task;
      PrimInfo pinfos[maxTasks];
      
      /* output data */
    public:
      TriRefList& prims;             //!< list of build primitives
      PrimInfo& pinfo;                  //!< bounding information of primitives
    };

    /*! Generates an array of triangle build primitives from the scene. */
    class TriRefArrayGen
    {
    public:   
      static void generate_sequential(size_t threadIndex, size_t threadCount, const Scene* scene, PrimRef* prims, PrimInfo& pinfo);
      static void generate_parallel  (size_t threadIndex, size_t threadCount, const Scene* scene, PrimRef* prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor that schedules the task */
      TriRefArrayGen (size_t threadIndex, size_t threadCount, const Scene* scene, PrimRef* prims_o, PrimInfo& pinfo_o, bool parallel);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(TriRefArrayGen,task_gen_parallel);
      
      /* input data */
    private:
      const Scene* scene;                  //!< input geometry
      
      /* output data */
    public:
      PrimRef* prims_o;             //!< list of build primitives
      PrimInfo& pinfo_o;            //!< bounding information of primitives
    };

    /*! Generates an array of triangle build primitives from some triangle mesh. */
    class TriRefArrayGenFromTriangleMesh
    {
    public:   
      static void generate_sequential(size_t threadIndex, size_t threadCount, const TriangleMesh* mesh, PrimRef* prims, PrimInfo& pinfo);
      static void generate_parallel  (size_t threadIndex, size_t threadCount, const TriangleMesh* mesh, PrimRef* prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor */
      TriRefArrayGenFromTriangleMesh (size_t threadIndex, size_t threadCount, const TriangleMesh* mesh, PrimRef* prims_o, PrimInfo& pinfo_o);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(TriRefArrayGenFromTriangleMesh,task_gen_parallel);
      
      /* input data */
    private:
      const TriangleMesh* mesh;      //!< input geometry
      
      /* output data */
    public:
      PrimRef* prims_o;             //!< list of build primitives
      PrimInfo& pinfo_o;            //!< bounding information of primitives
    };
  }
}
