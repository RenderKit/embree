// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "bvh4.h"
#include "bvh4_builder_util.h"
#include "bvh4_builder_binner.h"
#include "bvh4_builder_fast.h"
#include "common/scene_triangle_mesh.h"

namespace embree
{
  class BuildRecord;
  struct BuildRef;

  namespace isa
  {
    class BVH4BuilderTopLevel : public BVH4TopLevelBuilderFastT
    {
      ALIGNED_CLASS;
    public:
      
      /*! Type shortcuts */
      typedef BVH4::Node    Node;
      typedef BVH4::NodeRef NodeRef;
      static const size_t SIZE_WORK_STACK = 64;

      struct GlobalState
      {
        ALIGNED_CLASS;
              
      public:

        GlobalState () 
	: numThreads(getNumberOfLogicalThreads())
	{
          thread_workStack = new WorkStack<BuildRecord,SIZE_WORK_STACK>[numThreads];
          thread_bounds = new Centroid_Scene_AABB[numThreads];
        }
        
        ~GlobalState () {
          delete[] thread_workStack;
          delete[] thread_bounds;
        }

      public:
	size_t numThreads;
        __aligned(64) WorkStack<BuildRecord,SIZE_WORK_STACK> global_workStack;
        __aligned(64) WorkStack<BuildRecord,SIZE_WORK_STACK>* thread_workStack;
        LinearBarrierActive global_barrier;
        ParallelBinner2<16> parallelBinner;  
        Centroid_Scene_AABB* thread_bounds;
      };

      static std::auto_ptr<GlobalState> g_state;

    public:
      
      /*! Constructor. */
      BVH4BuilderTopLevel (BVH4* bvh, Scene* scene, const createTriangleMeshAccelTy createTriangleMeshAccel);
      
      /*! Destructor */
      ~BVH4BuilderTopLevel ();
      
      /*! builder entry point */
      void build(size_t threadIndex, size_t threadCount);
      
      void build_toplevel(size_t threadIndex, size_t threadCount);
      
      /*! parallel rebuild of geometry */
      TASK_RUN_FUNCTION(BVH4BuilderTopLevel,task_build_parallel);
     
      
      BBox3fa build (size_t threadIndex, size_t threadCount, size_t objectID);
      void create_object(size_t objectID);
      
      void open_sequential();
      
    public:
      BVH4* bvh;      //!< Output BVH4
      std::vector<BVH4*>& objects;
      std::vector<Builder*> builders;
      std::vector<size_t> allThreadBuilds;    
      
    public:
      Scene* scene;
      createTriangleMeshAccelTy createTriangleMeshAccel;
      
      vector_t<BuildRef> refs;
      vector_t<BuildRef> refs1;
      vector_t<PrimRef> prims;
      AlignedAtomicCounter32 nextRef;
    };
  }
}
