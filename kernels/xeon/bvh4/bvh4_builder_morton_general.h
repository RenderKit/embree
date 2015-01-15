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

#include "bvh4.h"
#include "builders/heuristic_fallback.h"
#include "builders/workstack.h"
#include "bvh4_statistics.h"
#include "geometry/triangle4.h"

#include "tbb/tbb.h"
#include "algorithms/parallel_for_for.h"


namespace embree
{
  namespace isa
  {
    //template<typename CreateLeafFunc>
    class BVH4BuilderMortonGeneral : public Builder
    {
      ALIGNED_CLASS;
      
    protected:
      /*! Type shortcuts */
      typedef BVH4::Node    Node;
      typedef BVH4::NodeRef NodeRef;
      typedef FastAllocator::ThreadLocal2 Allocator;
      //typedef LinearAllocatorPerThread::ThreadAllocator Allocator;
      
      enum { RECURSE = 1, CREATE_TOP_LEVEL = 2 };
      
      static const size_t MAX_TOP_LEVEL_BINS = 1024;
      static const size_t NUM_TOP_LEVEL_BINS = 1024 + 4*BVH4::maxBuildDepth;

      static const size_t LATTICE_BITS_PER_DIM = 10;
      static const size_t LATTICE_SIZE_PER_DIM = size_t(1) << LATTICE_BITS_PER_DIM;
      
      static const size_t RADIX_BITS = 11;
      static const size_t RADIX_BUCKETS = (1 << RADIX_BITS);
      static const size_t RADIX_BUCKETS_MASK = (RADIX_BUCKETS-1);

    public:
  
      class BuildRecord 
      {
      public:
        unsigned int begin;
        unsigned int end;
        unsigned int depth;
        BVH4::NodeRef* parent;
        
        __forceinline unsigned int size() const {
          return end - begin;
        }
        
        __forceinline void init(const unsigned int _begin, const unsigned int _end)			 
        {
          begin = _begin;
          end = _end;
          depth = 1;
          parent = NULL;
	}
        
	struct Greater {
	  __forceinline bool operator()(const BuildRecord& a, const BuildRecord& b) {
	    return a.size() > b.size();
	  }
	};
      };

      struct __aligned(8) MortonID32Bit
      {
        union {
          struct {
	    unsigned int code;
	    unsigned int index;
	    //uint64 index;
          };
          //int64 all;
        };

        __forceinline operator unsigned() const { return code; }
        
        __forceinline unsigned int get(const unsigned int shift, const unsigned and_mask) const {
          return (code >> shift) & and_mask;
        }
        
        /*__forceinline void operator=(const MortonID32Bit& v) {   
          all = v.all; 
	  };*/  
        
        __forceinline friend std::ostream &operator<<(std::ostream &o, const MortonID32Bit& mc) {
          o << "index " << mc.index << " code = " << mc.code;
          return o;
        }
        
        __forceinline bool operator<(const MortonID32Bit &m) const { return code < m.code; } 
        __forceinline bool operator>(const MortonID32Bit &m) const { return code > m.code; } 
      };

      struct MortonBuilderState
      {
        ALIGNED_CLASS;

      public:

        typedef unsigned int ThreadRadixCountTy[RADIX_BUCKETS];

        MortonBuilderState () 
        {
	  taskCounter = 0;
          numThreads = getNumberOfLogicalThreads();
          startGroup = new unsigned int[numThreads];
          startGroupOffset = new unsigned int[numThreads];
	  dest = new size_t[numThreads];
          radixCount = (ThreadRadixCountTy*) alignedMalloc(numThreads*sizeof(ThreadRadixCountTy));
        }

        ~MortonBuilderState () 
        {
          alignedFree(radixCount);
          delete[] startGroupOffset;
          delete[] startGroup;
	  delete[] dest;
        }

        size_t numThreads;
        unsigned int* startGroup;
        unsigned int* startGroupOffset;
	size_t* dest;
        ThreadRadixCountTy* radixCount;
        
	atomic_t taskCounter;
	std::vector<BuildRecord> buildRecords;
        __aligned(64) WorkStack<BuildRecord,NUM_TOP_LEVEL_BINS> workStack;
        LinearBarrierActive barrier;
      };
      
      /*! Constructor. */
      BVH4BuilderMortonGeneral (BVH4* bvh, Scene* scene, TriangleMesh* mesh, size_t listMode, size_t logBlockSize, bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), state(nullptr), scheduler(&scene->lockstep_scheduler), scene(scene), mesh(mesh), listMode(listMode), logBlockSize(logBlockSize), needVertices(needVertices), primBytes(primBytes), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
	topLevelItemThreshold(0), encodeShift(0), encodeMask(-1), morton(NULL), bytesMorton(0), numGroups(0), numPrimitives(0), numAllocatedPrimitives(0), numAllocatedNodes(0)
      {
        needAllThreads = true;
        //if (mesh) needAllThreads = mesh->numTriangles > 50000;
      }
      
      /*! Destruction */
      ~BVH4BuilderMortonGeneral ()
      {
        if (morton) os_free(morton,bytesMorton);
        bvh->alloc.shrink();
      }
      
      /* build function */
      //void build(size_t threadIndex, size_t threadCount);
      void build(size_t threadIndex, size_t threadCount) 
    {
      if (g_verbose >= 2)
        std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) << "::BVH4BuilderMortonGeneral ... " << std::flush;
      
      /* calculate size of scene */
      size_t numPrimitivesOld = numPrimitives;
      if (mesh) numPrimitives = mesh->numTriangles;
      else      numPrimitives = scene->numTriangles;
      
      bvh->numPrimitives = numPrimitives;
      numGroups = scene->size();
      size_t maxPrimsPerGroup = 0;
      if (mesh) 
        maxPrimsPerGroup = numPrimitives;
      else {
	for (size_t i=0; i<numGroups; i++) // FIXME: encoding unsafe
        {
          Geometry* geom = scene->get(i);
          if (!geom || geom->type != TRIANGLE_MESH) continue;
          TriangleMesh* mesh = (TriangleMesh*) geom;
	  if (mesh->numTimeSteps != 1) continue;
	  maxPrimsPerGroup = max(maxPrimsPerGroup,mesh->numTriangles);
        }

        /* calculate groupID, primID encoding */
        encodeShift = __bsr(maxPrimsPerGroup) + 1;
        encodeMask = ((size_t)1 << encodeShift)-1;
        size_t maxGroups = ((size_t)1 << (31-encodeShift))-1;
      
        if (maxPrimsPerGroup > encodeMask || numGroups > maxGroups) 
          THROW_RUNTIME_ERROR("encoding error in morton builder");
      }
      
      /* preallocate arrays */
      if (numPrimitivesOld != numPrimitives)
      {
	bvh->init(sizeof(BVH4::Node),numPrimitives,threadCount);
        if (morton) os_free(morton,bytesMorton);
	bytesMorton = ((numPrimitives+7)&(-8)) * sizeof(MortonID32Bit);
        morton = (MortonID32Bit* ) os_malloc(bytesMorton); memset(morton,0,bytesMorton);
      }
         
#if defined(PROFILE_MORTON_GENERAL)
      
      double dt_min = pos_inf;
      double dt_avg = 0.0f;
      double dt_max = neg_inf;
      for (size_t i=0; i<20; i++) 
      {
        double t0 = getSeconds();
#endif

        state.reset(new MortonBuilderState);
	//size_t numActiveThreads = threadCount;
	//size_t numActiveThreads = min(threadCount,getNumberOfCores());

      //int numThreads = tbb::task_scheduler_init::default_num_threads();
      //std::cout << "numThreads = " << numThreads << std::endl;
      //tbb::task_scheduler_init init(numThreads);

      //bvh->alloc.init(numPrimitives*sizeof(BVH4::Node),numPrimitives*sizeof(BVH4::Node));
      size_t bytesAllocated = (numPrimitives+7)/8*sizeof(BVH4::Node) + size_t(1.2f*(numPrimitives+3)/4)*sizeof(Triangle4);
      bvh->alloc2.init(bytesAllocated,2*bytesAllocated);

      /* compute scene bounds */
      Scene::Iterator<TriangleMesh,1> iter1(scene);
      global_bounds = parallel_for_for_reduce( iter1, CentGeomBBox3fa(empty), [&](TriangleMesh* mesh, const range<size_t>& r, size_t k) -> CentGeomBBox3fa
      {
        CentGeomBBox3fa bounds(empty);
        for (size_t i=r.begin(); i<r.end(); i++)
        {
          const BBox3fa b = mesh->bounds(i);
          if (!inFloatRange(b)) continue;
          bounds.extend(b);
        }
        return bounds;
      }, [] (CentGeomBBox3fa a, const CentGeomBBox3fa& b) { a.merge(b); return a; });

      bvh->bounds = global_bounds.geomBounds;

      /* calculate initial destination for each thread */
      for (size_t t=0; t<threadCount; t++)
	state->dest[t] = t*numPrimitives/threadCount;

      /* compute morton codes */
      MortonID32Bit* __restrict__ const dest = (MortonID32Bit*) bvh->alloc2.ptr();

      /* compute mapping from world space into 3D grid */
      const ssef base  = (ssef)global_bounds.centBounds.lower;
      const ssef diag  = (ssef)global_bounds.centBounds.upper - (ssef)global_bounds.centBounds.lower;
      const ssef scale = select(diag > ssef(1E-19f), rcp(diag) * ssef(LATTICE_SIZE_PER_DIM * 0.99f),ssef(0.0f));
      
      Scene::Iterator<TriangleMesh,1> iter(scene);
      parallel_for_for( iter, [&](TriangleMesh* mesh, const range<size_t>& r, size_t k)
      {
        size_t currentID = k;
        size_t slots = 0;
        ssei ax = 0, ay = 0, az = 0, ai = 0;
                
        for (size_t i=r.begin(); i<r.end(); i++)	  
        {
          const BBox3fa b = mesh->bounds(i);
          const ssef lower = (ssef)b.lower;
          const ssef upper = (ssef)b.upper;
          const ssef centroid = lower+upper;
          const ssei binID = ssei((centroid-base)*scale);
          unsigned int index = i;
          index |= mesh->id << encodeShift;
          ax[slots] = extract<0>(binID);
          ay[slots] = extract<1>(binID);
          az[slots] = extract<2>(binID);
          ai[slots] = index;
          slots++;
          currentID++;
          
          if (slots == 4)
          {
            const ssei code = bitInterleave(ax,ay,az);
            storeu4i(&dest[currentID-4],unpacklo(code,ai));
            storeu4i(&dest[currentID-2],unpackhi(code,ai));
            slots = 0;
          }
        }
            
        if (slots != 0)
        {
          const ssei code = bitInterleave(ax,ay,az);
          for (size_t i=0; i<slots; i++) {
            dest[currentID-slots+i].index = ai[i];
            dest[currentID-slots+i].code = code[i];
          }
        }
      });

      /* padding */     
      for (size_t i=numPrimitives; i<( (numPrimitives+7)&(-8) ); i++) {
        dest[i].code  = 0xffffffff; 
        dest[i].index = 0;
      }

      /* sort morton codes */
      radix_sort_copy_u32((MortonID32Bit*)bvh->alloc2.ptr(),morton,numPrimitives);

      BuildRecord br;
      br.init(0,numPrimitives);
      br.parent = &bvh->root;
      br.depth = 1;
      
      BBox3fa bounds = empty;
      LockStepTaskScheduler::execute_tbb([&] { bounds = recurse_tbb(br, NULL); });
      //bounds = recurse_tbb(br, NULL);

	//build_parallel_morton(threadIndex,numActiveThreads,0,1);
        state.reset(NULL);


#if defined(PROFILE_MORTON_GENERAL)
        double dt = getSeconds()-t0;
        dt_min = min(dt_min,dt);
        if (i != 0) dt_avg = dt_avg + dt;
        dt_max = max(dt_max,dt);
      }
      dt_avg /= double(19);
      
      std::cout << "[DONE]" << std::endl;
      std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
      std::cout << BVH4Statistics(bvh).str();
#endif

      if (g_verbose >= 2) {
        //std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
        std::cout << "  bvh4::alloc : "; bvh->alloc.print_statistics();
	std::cout << "  bvh4::alloc2: "; bvh->alloc2.print_statistics();
        std::cout << BVH4Statistics(bvh).str();
      }

      /* benchmark mode */
      if (g_benchmark) {
	BVH4Statistics stat(bvh);
	//std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
      }
    }
      
      /*! precalculate some per thread data */
      void initThreadState(const size_t threadID, const size_t numThreads);
      
      /*! single threaded build */
      void build_sequential_morton(size_t threadIndex, size_t threadCount);

      CentGeomBBox3fa computeBounds();

      void computeMortonCodes(const size_t startID, const size_t endID, size_t& destID,
                              const size_t startGroup, const size_t startOffset, 
                              MortonID32Bit* __restrict__ const dest);

      /*! main build task */
      TASK_SET_FUNCTION(BVH4BuilderMortonGeneral,build_parallel_morton);
      TaskScheduler::Task task;
      
      /*! task that calculates the bounding box of the scene */
      TASK_FUNCTION(BVH4BuilderMortonGeneral,computeBounds);
      
      /*! task that calculates the morton codes for each primitive in the scene */
      TASK_FUNCTION(BVH4BuilderMortonGeneral,computeMortonCodes);
      
      /*! parallel sort of the morton codes */
      TASK_FUNCTION(BVH4BuilderMortonGeneral,radixsort);
      
      /*! task that builds a list of sub-trees */
      TASK_FUNCTION(BVH4BuilderMortonGeneral,recurseSubMortonTrees);
      
    public:
      
      /*! creates leaf node */
      void createSmallLeaf(BuildRecord& current, Allocator* alloc, BBox3fa& box_o);

      void splitFallback(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild) const
      {
        const unsigned int center = (current.begin + current.end)/2;
        leftChild.init(current.begin,center);
        rightChild.init(center,current.end);
      }
      
      BBox3fa createLeaf(BuildRecord& current, Allocator* alloc)
      {
#if defined(DEBUG)
        if (current.depth > BVH4::maxBuildDepthLeaf) 
          THROW_RUNTIME_ERROR("ERROR: depth limit reached");
#endif
        
        /* create leaf for few primitives */
        if (current.size() <= minLeafSize) {
          BBox3fa bounds;
          createSmallLeaf(current,alloc,bounds);
          return bounds;
        }
        
        /* first split level */
        BuildRecord record0, record1;
        splitFallback(current,record0,record1);
        
        /* second split level */
        BuildRecord children[4];
        splitFallback(record0,children[0],children[1]);
        splitFallback(record1,children[2],children[3]);
        
        /* allocate node */
        Node* node = (Node*) alloc->alloc0.malloc(sizeof(Node)); node->clear();
        *current.parent = bvh->encodeNode(node);
        
        /* recurse into each child */
        BBox3fa bounds0 = empty;
        for (size_t i=0; i<4; i++) {
          children[i].parent = &node->child(i);
          children[i].depth = current.depth+1;
          BBox3fa bounds = createLeaf(children[i],alloc);
          bounds0.extend(bounds);
          node->set(i,bounds);
        }
        BVH4::compact(node); // move empty nodes to the end
        return bounds0;
      }  
      
      __forceinline void split(BuildRecord& current,
                               BuildRecord& left,
                               BuildRecord& right) const
      {
        const unsigned int code_start = morton[current.begin].code;
        const unsigned int code_end   = morton[current.end-1].code;
        unsigned int bitpos = clz(code_start^code_end);
        
        /* if all items mapped to same morton code, then create new morton codes for the items */
        if (unlikely(bitpos == 32)) 
        {
          recreateMortonCodes(current);
          const unsigned int code_start = morton[current.begin].code;
          const unsigned int code_end   = morton[current.end-1].code;
          bitpos = clz(code_start^code_end);
          
          /* if the morton code is still the same, goto fall back split */
          if (unlikely(bitpos == 32)) 
          {
            size_t center = (current.begin + current.end)/2; 
            left.init(current.begin,center);
            right.init(center,current.end);
            return;
          }
        }
        
        /* split the items at the topmost different morton code bit */
        const unsigned int bitpos_diff = 31-bitpos;
        const unsigned int bitmask = 1 << bitpos_diff;
        
        /* find location where bit differs using binary search */
        size_t begin = current.begin;
        size_t end   = current.end;
        while (begin + 1 != end) {
          const size_t mid = (begin+end)/2;
          const unsigned bit = morton[mid].code & bitmask;
          if (bit == 0) begin = mid; else end = mid;
        }
        size_t center = end;
#if defined(DEBUG)      
        for (unsigned int i=begin;  i<center; i++) assert((morton[i].code & bitmask) == 0);
        for (unsigned int i=center; i<end;    i++) assert((morton[i].code & bitmask) == bitmask);
#endif
        
        left.init(current.begin,center);
        right.init(center,current.end);
      }
      
      BBox3fa recurse(BuildRecord& current, Allocator* alloc, int mode) 
      {
        /* stop toplevel recursion at some number of items */
        if (mode == CREATE_TOP_LEVEL && current.size() <= topLevelItemThreshold) {
          state->buildRecords.push_back(current);
          return empty;
        }
        
        __aligned(64) BuildRecord children[BVH4::N];
        
        /* create leaf node */
        if (unlikely(current.depth >= BVH4::maxBuildDepth || current.size() <= minLeafSize)) {
          return createLeaf(current,alloc);
        }
        
        /* fill all 4 children by always splitting the one with the largest surface area */
        size_t numChildren = 1;
        children[0] = current;
        
        do {
          
          /* find best child with largest bounding box area */
          int bestChild = -1;
          unsigned bestItems = 0;
          for (unsigned int i=0; i<numChildren; i++)
          {
            /* ignore leaves as they cannot get split */
            if (children[i].size() <= minLeafSize)
              continue;
            
            /* remember child with largest area */
            if (children[i].size() > bestItems) { 
              bestItems = children[i].size();
              bestChild = i;
            }
          }
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          __aligned(64) BuildRecord left, right;
          split(children[bestChild],left,right);
          
          /* add new children left and right */
          left.depth = right.depth = current.depth+1;
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < BVH4::N);
        
        /* create leaf node if no split is possible */
        if (unlikely(numChildren == 1)) {
          BBox3fa bounds; createSmallLeaf(current,alloc,bounds); return bounds;
        }
        
        /* allocate node */
        Node* node = (Node*) alloc->alloc0.malloc(sizeof(Node)); node->clear();
        *current.parent = bvh->encodeNode(node);
        
        /* recurse into each child */
        BBox3fa bounds0 = empty;
        for (size_t i=0; i<numChildren; i++) 
        {
          children[i].parent = &node->child(i);
          
          if (children[i].size() <= minLeafSize) {
            const BBox3fa bounds = createLeaf(children[i],alloc);
            bounds0.extend(bounds);
            node->set(i,bounds);
          } else {
            const BBox3fa bounds = recurse(children[i],alloc,mode);
            bounds0.extend(bounds);
            node->set(i,bounds);
          }
        }
        return bounds0;
      }

      BBox3fa recurse_tbb(BuildRecord& current, Allocator* alloc) 
      {
        if (alloc == NULL) 
          alloc = bvh->alloc2.threadLocal2();

        __aligned(64) BuildRecord children[BVH4::N];
        
        /* create leaf node */
        if (unlikely(current.depth >= BVH4::maxBuildDepth || current.size() <= minLeafSize)) {
          return createLeaf(current,alloc);
        }
        
        /* fill all 4 children by always splitting the one with the largest surface area */
        size_t numChildren = 1;
        children[0] = current;
        
        do {
          
          /* find best child with largest bounding box area */
          int bestChild = -1;
          unsigned bestItems = 0;
          for (unsigned int i=0; i<numChildren; i++)
          {
            /* ignore leaves as they cannot get split */
            if (children[i].size() <= minLeafSize)
              continue;
            
            /* remember child with largest area */
            if (children[i].size() > bestItems) { 
              bestItems = children[i].size();
              bestChild = i;
            }
          }
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          __aligned(64) BuildRecord left, right;
          split(children[bestChild],left,right);
          
          /* add new children left and right */
          left.depth = right.depth = current.depth+1;
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < BVH4::N);
        
        /* create leaf node if no split is possible */
        if (unlikely(numChildren == 1)) {
          BBox3fa bounds; createSmallLeaf(current,alloc,bounds); return bounds;
        }
        
        /* allocate node */
        Node* node = (Node*) alloc->alloc0.malloc(sizeof(Node)); node->clear();
        *current.parent = bvh->encodeNode(node);
      
        /* process top parts of tree parallel */
        BBox3fa bounds0 = empty;
        if (current.size() > 4096)
        {
          BBox3fa bounds[4];
          tbb::task_group g;
          for (size_t i=0; i<numChildren; i++) {
            children[i].parent = &node->child(i);
            g.run([&,i]{ bounds[i] = recurse_tbb(children[i],NULL); });
          }
          g.wait();

          for (size_t i=0; i<numChildren; i++) {
            bounds0.extend(bounds[i]);
            node->set(i,bounds[i]);
          }
        } 

        /* finish tree sequential */
        else 
        {
          /* recurse into each child */
          for (size_t i=0; i<numChildren; i++) 
          {
            children[i].parent = &node->child(i);
            
            if (children[i].size() <= minLeafSize) {
              const BBox3fa bounds = createLeaf(children[i],alloc);
              bounds0.extend(bounds);
              node->set(i,bounds);
            } else {
              const BBox3fa bounds = recurse_tbb(children[i],alloc);
              bounds0.extend(bounds);
              node->set(i,bounds);
            }
          }
        }
        return bounds0;
      }
      
      /*! recreates morton codes when reaching a region where all codes are identical */
      void recreateMortonCodes(BuildRecord& current) const
      {
        assert(current.size() > 4);
        CentGeomBBox3fa global_bounds;
        global_bounds.reset();
        
        for (size_t i=current.begin; i<current.end; i++)
        {
          const size_t index  = morton[i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = index >> encodeShift; 
          const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
          global_bounds.extend(mesh->bounds(primID));
        }
        
        /* compute mapping from world space into 3D grid */
        const ssef base  = (ssef)global_bounds.centBounds.lower;
        const ssef diag  = (ssef)global_bounds.centBounds.upper - (ssef)global_bounds.centBounds.lower;
        const ssef scale = select(diag > ssef(1E-19f), rcp(diag) * ssef(LATTICE_SIZE_PER_DIM * 0.99f),ssef(0.0f));
        
        for (size_t i=current.begin; i<current.end; i++)
        {
          const size_t index  = morton[i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = index >> encodeShift; 
          const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
          const BBox3fa b = mesh->bounds(primID);
          const ssef lower = (ssef)b.lower;
          const ssef upper = (ssef)b.upper;
          const ssef centroid = lower+upper;
          const ssei binID = ssei((centroid-base)*scale);
          const unsigned int bx = extract<0>(binID);
          const unsigned int by = extract<1>(binID);
          const unsigned int bz = extract<2>(binID);
          const unsigned int code = bitInterleave(bx,by,bz);
          morton[i].code  = code;
        }
        std::sort(morton+current.begin,morton+current.end);
      }
      
      
    public:
      BVH4* bvh;               //!< Output BVH
      LockStepTaskScheduler* scheduler;
      std::unique_ptr<MortonBuilderState> state;

      Scene* scene;
      TriangleMesh* mesh;
      size_t logBlockSize;
      size_t blocks(size_t N) { return (N+((1<<logBlockSize)-1)) >> logBlockSize; }
      bool needVertices;
      size_t primBytes; 
      size_t minLeafSize;
      size_t maxLeafSize;
      size_t listMode;

      size_t topLevelItemThreshold;
      size_t encodeShift;
      size_t encodeMask;
            
    public:
      MortonID32Bit* __restrict__ morton;
      size_t bytesMorton;
      
    public:
      size_t numGroups;
      size_t numPrimitives;
      size_t numAllocatedPrimitives;
      size_t numAllocatedNodes;
      CentGeomBBox3fa global_bounds;
      Barrier barrier;
    };
  }
}
