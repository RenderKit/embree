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

#include "../builders/parallel_builder.h"
#include "../builders/builder_util.h"
#include "../builders/binning.h"
#include "../builders/priminfo.h"

#include "bvh4i.h"
#include "bvh4i_statistics.h"

#include "../../algorithms/parallel_for_for.h"
#include "../../algorithms/parallel_for_for_prefix_sum.h"

namespace embree
{

  /*! factor to modify size of pre-allocated node array */

#define BVH4I_NODE_PREALLOC_FACTOR               0.715f 


  /*! creates the builder */
  
  class BVH4iBuilder : public ParallelBinnedSAHBuilder
  {
    ALIGNED_CLASS;
  public:
    
    enum { 
      BVH4I_BUILDER_DEFAULT, 
      BVH4I_BUILDER_PRESPLITS, 
      BVH4I_BUILDER_VIRTUAL_GEOMETRY, 
      BVH4I_BUILDER_MEMORY_CONSERVATIVE,
      BVH4I_BUILDER_SUBDIV_MESH
    };

    static Builder* create (void* accel, void* geometry, size_t mode = BVH4I_BUILDER_DEFAULT);
 
    /*! Constructor. */
    BVH4iBuilder (BVH4i* bvh, void* geometry, const size_t bvh4iNodeSize = sizeof(BVH4i::Node));
    virtual ~BVH4iBuilder();


    /* virtual function interface */
    virtual void build            (const size_t threadIndex, const size_t threadCount);
    virtual void allocateData     (const size_t threadCount, const size_t newNumPrimitives);
    virtual void computePrimRefs  (const size_t threadIndex, const size_t threadCount);
    virtual void createAccel      (const size_t threadIndex, const size_t threadCount);
    virtual void finalize         (const size_t threadIndex, const size_t threadCount);

    virtual size_t getNumPrimitives();
    virtual void printBuilderName();

    virtual void buildSubTree(BuildRecord& current, 
			      NodeAllocator& alloc, 
			      const size_t mode,
			      const size_t threadID, 
			      const size_t numThreads);


    virtual void storeNodeDataUpdateParentPtrs(void *ptr,
					       BuildRecord *__restrict__ const br,
					       const size_t numChildren);

    virtual std::string getStatistics();
    
  protected:

    void build_main(size_t threadIndex, size_t threadCount);

    
    void allocateMemoryPools(const size_t numPrims, 
			     const size_t numNodes,
			     const size_t sizeNodeInBytes  = sizeof(BVH4i::Node),
			     const size_t sizeAccelInBytes = sizeof(Triangle1),
			     const float  bvh4iNodePreallocFactor = BVH4I_NODE_PREALLOC_FACTOR);

    void checkBuildRecord(const BuildRecord &current);
    void checkLeafNode(const BVH4i::NodeRef &ref, const BBox3fa &bounds);

    void createTriangle1AccelRange(const size_t startID, const size_t endID);

    TASK_FUNCTION(BVH4iBuilder,computePrimRefsTriangles);
    TASK_FUNCTION(BVH4iBuilder,createTriangle1Accel);
    TASK_FUNCTION(BVH4iBuilder,parallelBinningGlobal);
    LOCAL_TASK_FUNCTION(BVH4iBuilder,parallelBinningLocal);

  public:


    /*! splitting function that selects between sequential and parallel mode */
    bool split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads);

    /*! perform sequential binning and splitting */
    bool splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);
			      
    /*! perform parallel binning and splitting using all threads on all cores*/
    bool splitParallelGlobal(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t threads);

    /*! perform parallel binning and splitting using only the threads per core*/
    bool splitParallelLocal(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID);
    
    /*! creates a leaf node */
    void createLeaf(BuildRecord& current, NodeAllocator& alloc, const size_t threadIndex, const size_t threadCount);

    /*! select between recursion and stack operations */
    void recurse(BuildRecord& current, NodeAllocator& alloc, const size_t mode, const size_t threadID, const size_t numThreads);
    
    /*! recursive build function */
    void recurseSAH(BuildRecord& current, NodeAllocator& alloc, const size_t mode, const size_t threadID, const size_t numThreads);

    /*! brute force splitting */
    bool split_fallback(PrimRef * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);

  protected:
    BVH4i* bvh;                   //!< Output BVH

    /* work record handling */
    PrimRef  *    prims;
    int16    *    node;  // node array in 64 byte blocks
    Triangle1*    accel;

    /* threshold for leaf generation */
    size_t leafItemThreshold;

    /* memory allocation */
    size_t size_prims;
    const size_t num64BytesBlocksPerNode;


    __forceinline void storeNode(void *ptr,
				 BuildRecord *__restrict__ const br,
				 const size_t numChildren)
    {
      float16 lower = broadcast4to16f(&BVH4i::Node::initQBVHNode[0]);
      float16 upper = broadcast4to16f(&BVH4i::Node::initQBVHNode[1]);
      BVH4i::Node &bvh = *(BVH4i::Node*)ptr;

      bool16 m_lane = 0xf;
      for (size_t i=0;i<numChildren;i++)
	{
	  const float16 b_lower = broadcast4to16f(&br[i].bounds.geometry.lower);
	  const float16 b_upper = broadcast4to16f(&br[i].bounds.geometry.upper);
	  lower = select(m_lane,b_lower,lower);
	  upper = select(m_lane,b_upper,upper);
	  m_lane = (unsigned int)m_lane << 4;
	}      

      store16f_ngo((float16*)ptr+0,lower); 
      store16f_ngo((float16*)ptr+1,upper);             
    }


  };

  /*! derived binned-SAH builder supporting triangle pre-splits */  
  class BVH4iBuilderPreSplits : public BVH4iBuilder
  {
  public:

    BVH4iBuilderPreSplits (BVH4i* bvh, void* geometry) : BVH4iBuilder(bvh,geometry) {}

    virtual void allocateData   (const size_t threadCount, const size_t newNumPrimitives);
    virtual void computePrimRefs(const size_t threadIndex, const size_t threadCount);
    virtual void finalize       (const size_t threadIndex, const size_t threadCount);
    virtual void printBuilderName();

  protected:
    size_t numMaxPrimitives;
    size_t numMaxPreSplits;
    float averageBoxTriSAHRatio;

    float PRESPLIT_AREA_THRESHOLD;
    float PRESPLIT_MIN_AREA;

    __aligned(64) AlignedAtomicCounter32 dest0;
    __aligned(64) AlignedAtomicCounter32 dest1;
    __aligned(64) float averageBoxSAH;
    __aligned(64) float averageTriSAH;

    TASK_FUNCTION(BVH4iBuilderPreSplits,getAverageBoundsSAH);
    TASK_FUNCTION(BVH4iBuilderPreSplits,countAndComputePrimRefsPreSplits);
    TASK_FUNCTION(BVH4iBuilderPreSplits,computePrimRefsFromPreSplitIDs);
    
  };


  /*! derived memory conservative binned-SAH builder */  
  class BVH4iBuilderMemoryConservative : public BVH4iBuilder
  {
  public:
    
    BVH4iBuilderMemoryConservative (BVH4i* bvh, void* geometry) : BVH4iBuilder(bvh,geometry,sizeof(BVH4i::QuantizedNode)) {}

    virtual void allocateData     (const size_t threadCount, const size_t newNumPrimitives);
    virtual void printBuilderName();
    virtual void createAccel      (const size_t threadIndex, const size_t threadCount);   
    virtual void finalize(const size_t threadIndex, const size_t threadCount);

    virtual void storeNodeDataUpdateParentPtrs(void *ptr,
					       BuildRecord *__restrict__ const br,
					       const size_t numChildren);

    virtual std::string getStatistics();

  protected:
    TASK_FUNCTION(BVH4iBuilderMemoryConservative,createMemoryConservativeAccel);
 
  };



  /*! derived binned-SAH builder supporting virtual geometry */  
  class BVH4iBuilderVirtualGeometry : public BVH4iBuilder
  {
  public:
    BVH4iBuilderVirtualGeometry (BVH4i* bvh, void* geometry) : BVH4iBuilder(bvh,geometry) {}

    virtual size_t getNumPrimitives();
    virtual void computePrimRefs(const size_t threadIndex, const size_t threadCount);
    virtual void createAccel    (const size_t threadIndex, const size_t threadCount);
    virtual void printBuilderName();

  protected:
    TASK_FUNCTION(BVH4iBuilderVirtualGeometry,computePrimRefsVirtualGeometry);
    TASK_FUNCTION(BVH4iBuilderVirtualGeometry,createVirtualGeometryAccel);
    
  };

  /*! derived binned-SAH builder supporting subdivision surface meshes */  
  class BVH4iBuilderSubdivMesh : public BVH4iBuilder
  {
  protected:
    Scene::Iterator<SubdivMesh> iter;
    ParallelForForPrefixSumState<PrimInfo> pstate;
    bool fastUpdateMode;
    size_t numSubdivEnableDisableEvents;

    static const size_t MAX_SUBTREES = 2048;
    static const size_t GENERATE_SUBTREES_MAX_TREE_DEPTH = 5;

    size_t numSubTrees;
    BVH4i::NodeRef subtree_refs[MAX_SUBTREES];

  public:
    BVH4iBuilderSubdivMesh (BVH4i* bvh, void* geometry) : BVH4iBuilder(bvh,geometry),fastUpdateMode(false),numSubdivEnableDisableEvents(0)
      {}

    virtual void build            (const size_t threadIndex, const size_t threadCount);
    virtual void allocateData     (const size_t threadCount, const size_t totalNumPrimitives);
    virtual size_t getNumPrimitives();
    virtual void computePrimRefs  (const size_t threadIndex, const size_t threadCount);
    virtual void createAccel      (const size_t threadIndex, const size_t threadCount);
    virtual void printBuilderName();
    virtual void finalize         (const size_t threadIndex, const size_t threadCount);

    void extract_refit_subtrees(const BVH4i::NodeRef &ref, 
				const size_t depth);

    BBox3fa refit(const BVH4i::NodeRef &ref);

    BBox3fa refit_top_level(const BVH4i::NodeRef &ref, 
				const size_t depth);

  protected:
    TASK_FUNCTION(BVH4iBuilderSubdivMesh,updateLeaves);    
    TASK_FUNCTION(BVH4iBuilderSubdivMesh,refitSubTrees);    

  };

}
