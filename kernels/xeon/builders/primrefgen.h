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
    class PrimRefListGen
    {
      typedef atomic_set<PrimRefBlockT<PrimRef> > PrimRefList;

    public:      
      static void generate(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const Scene* scene, GeometryTy ty, size_t numTimeSteps, PrimRefList& prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor that schedules the task */
      PrimRefListGen (size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const Scene* scene, GeometryTy ty, size_t numTimeSteps, PrimRefList& prims, PrimInfo& pinfo);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(PrimRefListGen,task_gen_parallel);
      
    private:
      const Scene* scene;                  //!< input geometry
      GeometryTy ty;                       //!< types of geometry to generate
      size_t numTimeSteps;                 //!< number of timesteps to generate
      PrimRefBlockAlloc<PrimRef>* alloc;   //!< allocator for build primitive blocks
      size_t numPrimitives;                //!< number of generated primitives
      TaskScheduler::Task task;
      PrimRefList& prims_o;                 //!< list of build primitives
      PrimInfo& pinfo_o;                   //!< bounding information of primitives
    };

    /*! Generates a list of triangle build primitives from some triangle mesh. */
    class PrimRefListGenFromTriangleMesh
    {
      typedef atomic_set<PrimRefBlockT<PrimRef> > PrimRefList;

    public:      
      static void generate(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const TriangleMesh* mesh, PrimRefList& prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor that schedules the task */
      PrimRefListGenFromTriangleMesh (size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const TriangleMesh* mesh, PrimRefList& prims, PrimInfo& pinfo);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(PrimRefListGenFromTriangleMesh,task_gen_parallel);
      
      /* input data */
    private:
      const TriangleMesh* mesh;            //!< input geometry
      PrimRefBlockAlloc<PrimRef>* alloc;   //!< allocator for build primitive blocks
      TaskScheduler::Task task;
      PrimRefList& prims_o;                 //!< list of build primitives
      PrimInfo& pinfo_o;                   //!< bounding information of primitives
    };

    /*! Generates a list of build primitives from some bezier curve set. */
    class PrimRefListGenFromBezierCurves
    {
      typedef atomic_set<PrimRefBlockT<PrimRef> > PrimRefList;

    public:      
      static void generate(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const BezierCurves* set, PrimRefList& prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor that schedules the task */
      PrimRefListGenFromBezierCurves (size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const BezierCurves* set, PrimRefList& prims, PrimInfo& pinfo);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(PrimRefListGenFromBezierCurves,task_gen_parallel);
      
      /* input data */
    private:
      const BezierCurves* geom;             //!< input geometry
      PrimRefBlockAlloc<PrimRef>* alloc;   //!< allocator for build primitive blocks
      TaskScheduler::Task task;
      PrimRefList& prims_o;                //!< list of build primitives
      PrimInfo& pinfo_o;                   //!< bounding information of primitives
    };

    /*! Generates a list of build primitives from some user geometry set. */
    class PrimRefListGenFromUserGeometry
    {
      typedef atomic_set<PrimRefBlockT<PrimRef> > PrimRefList;

    public:      
      static void generate(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const UserGeometryScene::Base* set, PrimRefList& prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor that schedules the task */
      PrimRefListGenFromUserGeometry (size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const UserGeometryScene::Base* set, PrimRefList& prims, PrimInfo& pinfo);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(PrimRefListGenFromUserGeometry,task_gen_parallel);
      
      /* input data */
    private:
      const UserGeometryScene::Base* geom; //!< input geometry
      PrimRefBlockAlloc<PrimRef>* alloc;   //!< allocator for build primitive blocks
      TaskScheduler::Task task;
      PrimRefList& prims_o;                //!< list of build primitives
      PrimInfo& pinfo_o;                   //!< bounding information of primitives
    };

    /*! Generates an array of triangle build primitives from the scene. */
    class PrimRefArrayGen
    {
    public:   
      static void generate_sequential(size_t threadIndex, size_t threadCount, const Scene* scene, GeometryTy ty, size_t numTimeSteps, PrimRef* prims, PrimInfo& pinfo);
      static void generate_parallel  (size_t threadIndex, size_t threadCount, const Scene* scene, GeometryTy ty, size_t numTimeSteps, PrimRef* prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor that schedules the task */
      PrimRefArrayGen (size_t threadIndex, size_t threadCount, const Scene* scene, GeometryTy ty, size_t numTimeSteps, PrimRef* prims_o, PrimInfo& pinfo_o, bool parallel);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(PrimRefArrayGen,task_gen_parallel);
      
      /* input data */
    private:
      const Scene* scene;           //!< input geometry
      GeometryTy ty;                //!< types of geometry to generate
      size_t numTimeSteps;          //!< number of timesteps to generate
      size_t numPrimitives;         //!< number of generated primitives
      PrimRef* prims_o;             //!< list of build primitives
      PrimInfo& pinfo_o;            //!< bounding information of primitives
    };

    /*! Generates an array of triangle build primitives from some triangle mesh. */
    class PrimRefArrayGenFromTriangleMesh
    {
    public:   
      static void generate_sequential(size_t threadIndex, size_t threadCount, const TriangleMesh* geom, PrimRef* prims, PrimInfo& pinfo);
      static void generate_parallel  (size_t threadIndex, size_t threadCount, const TriangleMesh* geom, PrimRef* prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor */
      PrimRefArrayGenFromTriangleMesh (size_t threadIndex, size_t threadCount, const TriangleMesh* geom, PrimRef* prims_o, PrimInfo& pinfo_o);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(PrimRefArrayGenFromTriangleMesh,task_gen_parallel);
      
      /* input data */
    private:
      const TriangleMesh* geom;     //!< input geometry
      PrimRef* prims_o;             //!< list of build primitives
      PrimInfo& pinfo_o;            //!< bounding information of primitives
    };

    /*! Generates an array of triangle build primitives from some triangle mesh. */
    class PrimRefArrayGenFromBezierCurves
    {
    public:   
      static void generate_sequential(size_t threadIndex, size_t threadCount, const BezierCurves* geom, PrimRef* prims, PrimInfo& pinfo);
      static void generate_parallel  (size_t threadIndex, size_t threadCount, const BezierCurves* geom, PrimRef* prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor */
      PrimRefArrayGenFromBezierCurves (size_t threadIndex, size_t threadCount, const BezierCurves* geom, PrimRef* prims_o, PrimInfo& pinfo_o);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(PrimRefArrayGenFromBezierCurves,task_gen_parallel);
      
      /* input data */
    private:
      const BezierCurves* geom;     //!< input geometry
      PrimRef* prims_o;             //!< list of build primitives
      PrimInfo& pinfo_o;            //!< bounding information of primitives
    };

    /*! Generates an array of triangle build primitives from some triangle mesh. */
    class PrimRefArrayGenFromUserGeometry
    {
    public:   
      static void generate_sequential(size_t threadIndex, size_t threadCount, const UserGeometryScene::Base* geom, PrimRef* prims, PrimInfo& pinfo);
      static void generate_parallel  (size_t threadIndex, size_t threadCount, const UserGeometryScene::Base* geom, PrimRef* prims, PrimInfo& pinfo);
      
    private:
      
      /*! standard constructor */
      PrimRefArrayGenFromUserGeometry (size_t threadIndex, size_t threadCount, const UserGeometryScene::Base* geom, PrimRef* prims_o, PrimInfo& pinfo_o);
            
      /*! parallel task to iterate over the primitives */
      TASK_RUN_FUNCTION(PrimRefArrayGenFromUserGeometry,task_gen_parallel);
      
      /* input data */
    private:
      const UserGeometryScene::Base* geom;     //!< input geometry
      PrimRef* prims_o;             //!< list of build primitives
      PrimInfo& pinfo_o;            //!< bounding information of primitives
    };
  }
}
