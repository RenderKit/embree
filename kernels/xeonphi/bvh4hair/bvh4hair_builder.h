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

#include "bvh4i/bvh4i_builder.h"
#include "bvh4hair.h"
#include "geometry/bezier1i.h"


#define BVH_NODE_PREALLOC_FACTOR                 1.15f

namespace embree
{

  /*! derived binned-SAH builder supporting hair primitives */  
  class BVH4HairBuilderConvert : public BVH4iBuilder
  {
    ALIGNED_CLASS;

  public:
    BVH4Hair *bvh4hair;
    
  BVH4HairBuilderConvert(BVH4Hair* bvh, BuildSource* source, void* geometry) : BVH4iBuilder((BVH4i*)bvh,source,geometry) 
      {
	bvh4hair = bvh;
      }

    virtual void build          (const size_t threadIndex, const size_t threadCount);
    virtual void computePrimRefs(const size_t threadIndex, const size_t threadCount);
    virtual void createAccel    (const size_t threadIndex, const size_t threadCount);
    virtual size_t getNumPrimitives();
    virtual void printBuilderName();

  protected:
    TASK_FUNCTION(BVH4HairBuilderConvert,computePrimRefsBezierCurves);
    TASK_FUNCTION(BVH4HairBuilderConvert,createBezierCurvesAccel);    
  };


  /*! derived binned-SAH builder supporting hair primitives */  
  class BVH4HairBuilder : public ParallelBuilderInterface
  {
    ALIGNED_CLASS;

  protected:
    static const size_t ALLOCATOR_NODE_BLOCK_SIZE = 64;
    typedef AtomicIDBlock<ALLOCATOR_NODE_BLOCK_SIZE> NodeAllocator;    

  public:
    BVH4Hair *bvh4hair;
    Bezier1i *prims;
    Bezier1i *accel;
    BVH4Hair::UnalignedNode*   node;

    size_t size_prims;
    size_t size_accel;
    size_t size_nodes;
    
  BVH4HairBuilder(BVH4Hair* bvh, BuildSource* source, void* geometry) 
    : ParallelBuilderInterface(source,geometry),
      bvh4hair(bvh),
      prims(NULL),
      node(NULL),
      accel(NULL),
      size_prims(0),
      size_nodes(0),
      size_accel(0)
      {
      }

    virtual ~BVH4HairBuilder() 
      {	
      }

    virtual size_t getNumPrimitives();
    virtual void   printBuilderName();
    virtual void   allocateData   (const size_t threadCount, const size_t newNumPrimitives);
    virtual void   computePrimRefs(const size_t threadIndex, const size_t threadCount);
    virtual void   build          (const size_t threadIndex, const size_t threadCount);
    virtual void   createAccel    (const size_t threadIndex, const size_t threadCount);

    virtual void buildSubTree(BuildRecord& current, 
			      NodeAllocator& alloc, 
			      const size_t mode,
			      const size_t threadID, 
			      const size_t numThreads);

    void build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event);

  protected:

    void allocateMemoryPools(const size_t numPrims, const size_t numNodes);

    /*! recursive build function */
    void recurseSAH(BuildRecord& current, NodeAllocator& alloc, const size_t mode, const size_t threadID, const size_t numThreads);

    bool split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads);
  
    void createLeaf(BuildRecord& current, NodeAllocator& alloc,const size_t threadIndex, const size_t threadCount);


    TASK_RUN_FUNCTION(BVH4HairBuilder,build_parallel_hair);
    TASK_FUNCTION(BVH4HairBuilder,computePrimRefsBezierCurves);
  };



}
