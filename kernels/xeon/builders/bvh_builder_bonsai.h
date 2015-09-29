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

#include "heuristic_bonsai.h"
#include "../../common/scene.h"
#include <chrono>

namespace embree
{
  namespace isa
  {
    template<typename Set, typename Split>
      struct GeneralBuildRecord
      {
      public:
	__forceinline GeneralBuildRecord () {}

        __forceinline GeneralBuildRecord (size_t depth)
          : depth(depth), pinfo(empty) {}

        __forceinline GeneralBuildRecord (const PrimInfo& pinfo, size_t depth, size_t* parent)
          : pinfo(pinfo), depth(depth), parent(parent) {}

        __forceinline GeneralBuildRecord (const PrimInfo& pinfo, size_t depth, size_t* parent, const Set &prims)
          : pinfo(pinfo), depth(depth), parent(parent), prims(prims) {}

          __forceinline GeneralBuildRecord (const PrimInfo& pinfo, size_t depth, size_t* parent, const Set &prims, const Set &indexRange)
            : pinfo(pinfo), depth(depth), parent(parent), prims(prims), indexRange(indexRange) {}

        __forceinline friend bool operator< (const GeneralBuildRecord& a, const GeneralBuildRecord& b) { return a.pinfo.size() < b.pinfo.size(); }
	__forceinline friend bool operator> (const GeneralBuildRecord& a, const GeneralBuildRecord& b) { return a.pinfo.size() > b.pinfo.size(); }

        __forceinline BBox3fa bounds() const { return pinfo.geomBounds; }

        __forceinline size_t size() const { return this->pinfo.size(); }

      public:
        size_t* parent;   //!< Pointer to the parent node's reference to us
		  size_t depth;     //!< Depth of the root of this subtree.
		  Set prims;        //!< The list of primitives.
		  PrimInfo pinfo;   //!< Bounding info of primitives.
	//Split split;      //!< The best split for the primitives.
		  Split split;
      Set indexRange;   //!< In memory available space for split partitioning
      };

    template<typename BuildRecord,
      typename Heuristic,
      typename ReductionTy,
      typename Allocator,
      typename CreateAllocFunc,
      typename CreateNodeFunc,
      typename UpdateNodeFunc,
      typename CreateLeafFunc,
      typename ProgressMonitor>

      class GeneralBVHBuilder
      {
        static const size_t MAX_BRANCHING_FACTOR = 16;        //!< maximal supported BVH branching factor
        static const size_t MIN_LARGE_LEAF_LEVELS = 8;        //!< create balanced tree of we are that many levels before the maximal tree depth
        static const size_t SINGLE_THREADED_THRESHOLD = 4096; //!< threshold to switch to single threaded build
		size_t MAX_MINI_TREE_SIZE = 8192;		//!< maximum size of a mini tree
		float  PRUNING_THRESHOLD = 0.1f;			//!< a pruning treshold in the range ]0,1[, anything else results in no pruning
      const bool splitTriangles = false;
      const int numberOfTriangles;
		  volatile int miniTreeCount;
      volatile int splitIndex;

		  int underFilledNodes;
      mvector<unsigned> ttIndicesX;
      mvector<unsigned> ttIndicesY;
      mvector<unsigned> ttIndicesZ;

      mvector<float> ttMidpointsX;
      mvector<float> ttMidpointsY;
      mvector<float> ttMidpointsZ;

      float* __restrict ttMidPoints[3];
      unsigned* __restrict ttIndices[3];

      mvector<PrimRef> ttBounds;
      /*
      mvector<unsigned> ttTemporaryIndices;
		  mvector<unsigned char> ttLeftPartition;
		  mvector<float> ttAccumulatedArea;
      mvector<FastSweepData*> fsd;
      */
      mvector<unsigned> ttTemporaryIndices;
		  mvector<unsigned char> ttLeftPartition;
		  mvector<float> ttAccumulatedArea;
      mvector<FastSweepData*> fsd;
      public:
        mvector<BuildRecord> miniTreeRecords;
		  mvector<BuildRecord> miniTreeRecordsPruned;
      mvector<size_t*> miniTreeRoots;

        GeneralBVHBuilder (Heuristic& heuristic,
                           const ReductionTy& identity,
                           CreateAllocFunc& createAlloc,
                           CreateNodeFunc& createNode,
                           UpdateNodeFunc& updateNode,
                           CreateLeafFunc& createLeaf,
                           ProgressMonitor& progressMonitor,
                           const PrimInfo& pinfo,
                           const size_t branchingFactor, const size_t maxDepth,
                           const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize,
                           const float travCost, const float intCost, const int numberOfTriangles, const bool splitTriangles, Scene* scene)
          : heuristic(heuristic),
          identity(identity),
          createAlloc(createAlloc), createNode(createNode), updateNode(updateNode), createLeaf(createLeaf),
          progressMonitor(progressMonitor),
          pinfo(pinfo),
          branchingFactor(branchingFactor), maxDepth(maxDepth),
          logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
          travCost(travCost), intCost(intCost), numberOfTriangles(numberOfTriangles),splitTriangles(splitTriangles),
          ttTemporaryIndices(scene->device), ttLeftPartition(scene->device), ttAccumulatedArea(scene->device),
          fsd(scene->device), miniTreeRecords(scene->device), miniTreeRecordsPruned(scene->device), miniTreeRoots(scene->device),
          ttBounds(scene->device), ttIndicesX(scene->device), ttIndicesY(scene->device), ttIndicesZ(scene->device),
          ttMidpointsX(scene->device), ttMidpointsY(scene->device), ttMidpointsZ(scene->device)
        {
          if (branchingFactor > MAX_BRANCHING_FACTOR)
            THROW_RUNTIME_ERROR("bvh_builder: branching factor too large");
        }

		  /*! bonsai builder */

		  __forceinline void partition(BuildRecord& brecord, BuildRecord& lrecord, BuildRecord& rrecord) {
			  heuristic.split(brecord.split,brecord.pinfo,brecord.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo, rrecord.prims);
		  }



		  __forceinline int getLargestDim(const BBox3fa& bounds) {
			  Vec3fa dist = bounds.upper - bounds.lower;
        int dim = -1;
        if (dist[0] > 0 || dist[1] > 0 || dist[2] > 0) {
			     dim = dist[0] > dist[1] ? 0 : 1;
			      dim = dist[dim] > dist[2] ? dim : 2;
          }

			  return dim;
		  }
      __forceinline float getLargestDist(const BBox3fa& bounds) {
			  Vec3fa dist = bounds.upper - bounds.lower;
			    float largest = dist[0] > dist[1] ? dist[0] : dist[1];
			      largest = largest > dist[2] ? largest : dist[2];


			  return largest;
		  }
		  void miniTreeSelectRescursive(BuildRecord current) {


        if (getLargestDist(current.pinfo.geomBounds) == 0)
          return;
			  if (current.prims.size() <= MAX_MINI_TREE_SIZE) {
				  int miniTreeIndex = atomic_add(&miniTreeCount, 1);
				  miniTreeRecords[miniTreeIndex] = current;
				  return;
			  }
			  BuildRecord left, right;
			  current.split.dim = getLargestDim(current.pinfo.centBounds);
        current.split.pos = current.pinfo.centBounds.center()[current.split.dim];

			  partition(current, left, right);


			  if (current.size() > SINGLE_THREADED_THRESHOLD*4)
			  {
			  SPAWN_BEGIN;
				SPAWN(([&] { miniTreeSelectRescursive(left); }));
				SPAWN(([&] { miniTreeSelectRescursive(right); }));
			  SPAWN_END;
			  }
			  else {
				  miniTreeSelectRescursive(left);
				  miniTreeSelectRescursive(right);
			  }

		  }

		  __forceinline bool sweep(FastSweepData& data, BuildRecord& current, BuildRecord& left, BuildRecord& right, unsigned offset) {
			  return heuristic.fastSweep(data, current.pinfo, current.split, current.prims, left.pinfo, left.prims, left.split, right.pinfo, right.prims, right.split, maxLeafSize, offset, current.depth);
		  }


		  void buildMiniTreesRecursive(FastSweepData& data, BuildRecord& current, unsigned offset, Allocator alloc) {

			  if (alloc == nullptr)
				 Allocator alloc = createAlloc();
			  /* call memory monitor function to signal progress */
			  //if (toplevel && current.size() <= SINGLE_THREADED_THRESHOLD)
			//	  progressMonitor(current.size());

			  //assert(current.pinfo.size() == 0);

			  /*! initialize child list */
			  BuildRecord children[MAX_BRANCHING_FACTOR];
			  children[0] = current;
			  size_t numChildren = 1;

			  /*! split until node is full or SAH tells us to stop */
			  do {

				  /*! find best child to split */
				  float largestSAH = -1;
				  ssize_t largestChild = -1;
				  for (size_t i=0; i<numChildren; i++)
				  {

					  if (children[i].pinfo.size() <= minLeafSize || children[i].split.isLeaf()) continue;
					  float currentSAH = children[i].pinfo.leafSAH(logBlockSize);
					  if (currentSAH > largestSAH) { largestChild = i; largestSAH = currentSAH; }

				  }
				  if (largestChild == -1) break;

				  /* perform best found split */
				  BuildRecord& brecord = children[largestChild];
				  BuildRecord lrecord(current.depth+1);
				  BuildRecord rrecord(current.depth+1);


				  if (!sweep(data, brecord, lrecord, rrecord, offset)) {
					  children[largestChild] = lrecord;
					  children[numChildren] = rrecord;
					  numChildren++;
				  }

			  } while (numChildren < branchingFactor);

			  /*! create an inner node */
			  if (numChildren > 1) {

				  auto node = createNode(current,children,numChildren,alloc);
			  }
			  else {
				  heuristic.gatherReferences(data, current.prims);
				  //std::sort(data.gatheredRefs.data(),data.gatheredRefs.data() + current.prims.size());
				  createLeaf(current, data.gatheredRefs, alloc);
				  return;
			  }
			  /* spawn tasks */
			  for (ssize_t i=numChildren-1; i>=0; i--) {
				  buildMiniTreesRecursive(data, children[i], offset, alloc);
			  }
		  }

		  void setupIndices(unsigned** indices, const size_t first, const size_t count) {
			  for (int i = 0; i < count; i++)
				  indices[0][i] = indices[1][i] = indices[2][i] = i;// + first;
		  }

		  void buildMiniTrees(mvector<size_t*>& miniTreeRoots) {

			  if (miniTreeCount == 1) {
          FastSweepData data((int)(MAX_MINI_TREE_SIZE), maxLeafSize);
  			  Allocator alloc = createAlloc();
				  BuildRecord& br = miniTreeRecords[0];
				  br.prims = range<size_t>(0, br.prims.size());
				  heuristic.sortIndices(data, br.prims.size(), 0);
				  buildMiniTreesRecursive(data, br, 0, alloc);
			  }
			  else {
#if 0
        FastSweepData data((int)(MAX_MINI_TREE_SIZE), maxLeafSize);
        Allocator alloc = createAlloc();
				  for (int i = 0; i < miniTreeCount; ++i) {
					  BuildRecord& br = miniTreeRecords[i];
					  data.setOffset(br.prims.begin());
            unsigned offset = br.prims.begin();
					  br.parent = (size_t*)&(miniTreeRoots.data()[i]);
					  br.depth = 0;
					  br.prims = range<size_t>(0, br.prims.size());
					  br.pinfo.begin = 0;
					  br.pinfo.end = br.prims.size();
					  heuristic.sortIndices(data, br.prims.size(), offset);
					  buildMiniTreesRecursive(data, br, offset, alloc);
				  }
#else

          int threadCount = TaskSchedulerTBB::threadCount();

          fsd.resize(threadCount);
          FastSweepData** __restrict fastSweepDataPtr = fsd.data();
          for (int i = 0; i < threadCount; i++) {
              fastSweepDataPtr[i] = new FastSweepData(MAX_MINI_TREE_SIZE, maxLeafSize);
          }

  			  unsigned granularity = miniTreeCount/512;// / 128;

          if (granularity < 1) {
            granularity = miniTreeCount/128;
          }
          if (granularity < 1) {
            granularity = 1;
          }


          size_t** __restrict miniTreeRootsPtr = miniTreeRoots.data();
          //std::sort(miniTreeRecords.data(), miniTreeRecords.data() + miniTreeCount, [](BuildRecord& a, BuildRecord& b) { return a.prims.size() > b.prims.size(); });

				  SPAWN_BEGIN;
				  int treeID = 0;
				  for (; treeID < miniTreeCount - granularity + 1; treeID += granularity) {

            SPAWN(([this, treeID, miniTreeRootsPtr, fastSweepDataPtr, granularity] {
              int threadIndex = TaskSchedulerTBB::threadIndex();
              FastSweepData& data = *(fastSweepDataPtr[threadIndex]);

              Allocator alloc = createAlloc();

						  for (int i = treeID; i < treeID + granularity; ++i) {
                BuildRecord& br = miniTreeRecords[i];

                int offset = br.prims.begin();
							  br.parent = (size_t*)&(miniTreeRootsPtr[i]);
							  br.depth = 0;
							  br.prims = range<size_t>(0, br.prims.size());
							  br.pinfo.begin = 0;
							  br.pinfo.end = br.prims.size();
							  heuristic.sortIndices(data, br.prims.size(), offset);
							  buildMiniTreesRecursive(data, br, offset, alloc);
						}
					  }));
				  }
          if (treeID < miniTreeCount) {
				  SPAWN(([this, treeID, miniTreeRootsPtr, fastSweepDataPtr] {
            int threadIndex = TaskSchedulerTBB::threadIndex();
            FastSweepData& data = *(fastSweepDataPtr[threadIndex]);

            Allocator alloc = createAlloc();

					  for (int i = treeID; i < miniTreeCount; ++i) {
						  BuildRecord& br = miniTreeRecords[i];
						  //data.setOffset(br.prims.begin());
              int offset = br.prims.begin();
						  br.parent = (size_t*)&(miniTreeRootsPtr[i]);
						  br.depth = 0;
						  br.prims = range<size_t>(0, br.prims.size());
						  br.pinfo.begin = 0;
						  br.pinfo.end = br.prims.size();
						  heuristic.sortIndices(data, br.prims.size(), offset);
						  buildMiniTreesRecursive(data, br, offset, alloc);
					  }}));
          }
        SPAWN_END;
#endif
			  }
		}



		  void partitionAccordingToRef(unsigned axis, unsigned first, unsigned last) {
			  unsigned* __restrict temporaryIndices = ttTemporaryIndices.data();
			  unsigned char* __restrict leftPartition = ttLeftPartition.data();

			  //unsigned* __restrict indices = ttIndices[axis].data();
        unsigned* __restrict indices = ttIndices[axis];

			  unsigned* source = indices + first;
			  unsigned* end = indices + last;

			  unsigned* left = source;
			  unsigned* right = temporaryIndices + first;

			  while (source < end) {
				  unsigned index = *source;
				  unsigned l = leftPartition[index];

				  *left = index;
				  *right = index;

				  left += l;
				  right += 1-l;

				  ++source;
			  }

			  std::copy(temporaryIndices + first, right, left);
		  }

		  void ttPartition(unsigned axis, unsigned first, unsigned last, unsigned pivot) {
			  //unsigned* __restrict ref = (ttIndices[axis]).data();
        unsigned* __restrict ref = (ttIndices[axis]);
			  unsigned char* __restrict leftPartition = ttLeftPartition.data();

			  for (unsigned i = first; i < pivot; ++i)
				  leftPartition[ref[i]] = 1;

			  for (unsigned i = pivot; i < last; ++i)
				  leftPartition[ref[i]] = 0;

			  partitionAccordingToRef((axis+1)%3, first, last);
			  partitionAccordingToRef((axis+2)%3, first, last);
		  }

		  void topTreeSweep(BuildRecord& current, BuildRecord& lrecord, BuildRecord& rrecord) {

			  unsigned first = current.prims.begin();
			  unsigned last = current.prims.end();

			  float bestSah = std::numeric_limits<float>::infinity();

			  unsigned bestDim = 0xffffffff;
			  unsigned pivot = 0xffffffff;

			  BBox3fa bestBounds;

			  for (unsigned dim = 0; dim < 3; ++dim) {
				  //unsigned* __restrict sortedIndices = ttIndices[dim].data();
          unsigned* __restrict sortedIndices = ttIndices[dim];
				  float* __restrict accumulatedArea = ttAccumulatedArea.data();
				  BBox3fa bounds = miniTreeRecordsPruned[sortedIndices[first]].pinfo.geomBounds;

				  {
					  int i = (int)first;

					  for(; (int)i < (int)last-1; i++) {
						  bounds.extend(miniTreeRecordsPruned[sortedIndices[i]].pinfo.geomBounds);
						  float sa = halfArea(bounds);
						  accumulatedArea[i] =  sa * (float)((int)(i - first + 1));
					  }
				  }

				  bounds = miniTreeRecordsPruned[sortedIndices[last - 1]].pinfo.geomBounds;
				  {
					  int i = (int)last - 1;

					  for (; (int)i > (int)first; i--) {
						  bounds.extend(miniTreeRecordsPruned[sortedIndices[i]].pinfo.geomBounds);
						  float sa = halfArea(bounds);
						  float sah = (accumulatedArea[i - 1] + // left side SAH
									     sa * (float)((int)(last - i))); // right side SAH

						  if (sah < bestSah) {
							  bestSah = sah;
							  bestDim = dim;
							  pivot = i;
							  bestBounds = bounds;
						  }
					  }
				  }
			  }

			  lrecord.prims = range<size_t>(first, pivot);
			  rrecord.prims = range<size_t>(pivot, last);

			  ttPartition(bestDim, first, last, pivot);

			  rrecord.pinfo = PrimInfo(pivot, last, bestBounds, bestBounds);
			  //bestBounds = miniTreeRecordsPruned[ttIndices[0].data()[first]].pinfo.geomBounds;
        bestBounds = miniTreeRecordsPruned[ttIndices[0][first]].pinfo.geomBounds;
			  size_t index = first + 1;
			  for (; index < pivot; index++) {
				  //bestBounds.extend(miniTreeRecordsPruned[ttIndices[0].data()[index]].pinfo.geomBounds);
          bestBounds.extend(miniTreeRecordsPruned[ttIndices[0][index]].pinfo.geomBounds);
			  }

			  lrecord.pinfo = PrimInfo(first, pivot, bestBounds, bestBounds);
		  }

		  void buildTopTreeRecursive(BuildRecord& current, Allocator alloc) {

			  if (alloc == nullptr)
				  alloc = createAlloc();
			  /* call memory monitor function to signal progress */
			  //if (toplevel && current.size() <= SINGLE_THREADED_THRESHOLD)
			  //	  progressMonitor(current.size());

			  assert(current.pinfo.size() > 0);// || leafSAH >= 0 && splitSAH >= 0);

        BuildRecord children[MAX_BRANCHING_FACTOR];
			  /*! create a leaf node when threshold reached or SAH tells us to stop */
			  if (current.pinfo.size() <= branchingFactor) {

				  size_t count = current.pinfo.size();
				  for (int i = 0; i < count; i++) {
					   children[i] = miniTreeRecordsPruned[ttIndices[0][i + current.prims.begin()]];
				  }

				  if (count == 1) {
					  *current.parent = *children[0].parent;

				  }
				  else {
						  createNode(current, children, count, alloc, underFilledNodes);
				  }

				  return;
			  }

			  /*! initialize child list */

			  children[0] = current;
			  size_t numChildren = 1;

			  /*! split until node is full or SAH tells us to stop */
			  do {

				  /*! find best child to split */
				  float largestSAH = -1;
				  ssize_t largestChild = -1;
				  for (size_t i=0; i<numChildren; i++)
				  {
					  //float currentSAH = children[i].pinfo.leafSAH();
             // not clear why, but tree quality is improved. I would like to try
             // the max depth of the set of mini trees times the area as a cost measurement.
            float currentSAH = children[i].pinfo.leafSAH(32);
					  if (children[i].pinfo.size() == 1) continue;
					  if (currentSAH > largestSAH) { largestChild = i; largestSAH = currentSAH; }
				  }
				  if (largestChild == -1) break;

				  /* perform best found split */
				  BuildRecord& brecord = children[largestChild];
				  BuildRecord lrecord(current.depth+1);
				  BuildRecord rrecord(current.depth+1);

				  topTreeSweep(brecord, lrecord, rrecord);

				  children[largestChild] = lrecord;
				  children[numChildren] = rrecord;
				  numChildren++;



			  } while (numChildren < branchingFactor);

			  /*! create an inner node */
			  if (numChildren == 8)
				  auto node = createNode(current,children,numChildren,alloc);

			  /* spawn tasks */
#if 0
			  for (ssize_t i=numChildren-1; i>=0; i--) {
				 // if (children[i].size() > 1)
				  buildTopTreeRecursive(children[i], alloc);

			  }
#else
			  if (current.size() > SINGLE_THREADED_THRESHOLD) {
				  SPAWN_BEGIN;
				  for (ssize_t i=numChildren-1; i>=0; i--)
					  SPAWN(([&,i] { buildTopTreeRecursive(children[i],nullptr); }));
				  SPAWN_END;
			  }
			  else {
				  for (ssize_t i=numChildren-1; i>=0; i--) {
					  // if (children[i].size() > 1)
					  buildTopTreeRecursive(children[i], alloc);

				  }
			  }
#endif
		  }

      void sortTopTree() {

        //if (!ttAlloc) {
          Allocator alloc;
          alloc = createAlloc();

          ttIndicesX.resize(2*miniTreeCount+8);
          ttIndicesY.resize(2*miniTreeCount+8);
          ttIndicesZ.resize(2*miniTreeCount+8);

          ttMidpointsX.resize(miniTreeCount+4);
          ttMidpointsY.resize(miniTreeCount+4);
          ttMidpointsZ.resize(miniTreeCount+4);

          ttIndices[0] = (unsigned* __restrict)ttIndicesX.data();
          ttIndices[1] = ttIndicesY.data();
          ttIndices[2] = ttIndicesZ.data();
/*
          ttIndices[0] = (unsigned*)alloc->alloc0.malloc(2*sizeof(unsigned) * miniTreeCount + 256, 64);
          ttIndices[1] = (unsigned*)alloc->alloc0.malloc(2*sizeof(unsigned) * miniTreeCount + 256, 64);
          ttIndices[2] = (unsigned*)alloc->alloc0.malloc(2*sizeof(unsigned) * miniTreeCount + 256, 64);

          ttMidpoints[0] = (float*)alloc->alloc0.malloc(sizeof(float) * miniTreeCount + 128, 64);
          ttMidpoints[1] = (float*)alloc->alloc0.malloc(sizeof(float) * miniTreeCount + 128, 64);
          ttMidpoints[2] = (float*)alloc->alloc0.malloc(sizeof(float) * miniTreeCount + 128, 64);
*/

        BuildRecord* __restrict mtrp = miniTreeRecordsPruned.data();
        int i = 0;

        float* __restrict mpx = ttMidpointsX.data();
        float* __restrict mpy = ttMidpointsY.data();
        float* __restrict mpz = ttMidpointsZ.data();

        unsigned* __restrict idx = ttIndices[0];
        unsigned* __restrict idy = ttIndices[1];
        unsigned* __restrict idz = ttIndices[2];

        for (; i < miniTreeCount - 3; i += 4) {
          BBox3fa b = mtrp[i].pinfo.geomBounds;
				  __m128 mid0 = _mm_add_ps(b.lower, b.upper);
          b = mtrp[i + 1].pinfo.geomBounds;
					__m128 mid1 = _mm_add_ps(b.lower, b.upper);
          b = mtrp[i + 2].pinfo.geomBounds;
					__m128 mid2 = _mm_add_ps(b.lower, b.upper);
          b = mtrp[i + 3].pinfo.geomBounds;
					__m128 mid3 = _mm_add_ps(b.lower, b.upper);

				  _MM_TRANSPOSE4_PS(mid0, mid1, mid2, mid3);
				  _mm_store_ps(mpx + i, mid0);
				  _mm_store_ps(mpy + i, mid1);
				  _mm_store_ps(mpz + i, mid2);
			  }

			  for (; i < miniTreeCount; i++) {

				  //Vec3fa mid = prims[(i + offset)].center2();
          BBox3fa& b = mtrp[i].pinfo.geomBounds;
					__m128 mid = _mm_add_ps(b.lower, b.upper);
          ALIGNED(32) float midArray[4];
					_mm_store_ps(midArray, mid);
				  //prims[(i + offset)].lower = _mm_xor_ps(prims[(i + offset)].lower, neg);
				  mpx[i] = midArray[0];
				  mpy[i] = midArray[1];
				  mpz[i] = midArray[2];

			  }

        SPAWN_BEGIN;
        SPAWN(([&] {sortedIndicesFromFloats(mpx, idx, miniTreeCount);}));
        SPAWN(([&] {sortedIndicesFromFloats(mpy, idy, miniTreeCount);}));
        SPAWN(([&] {sortedIndicesFromFloats(mpz, idz, miniTreeCount);}));
        SPAWN_END;

      }

		  void buildTopTree(BuildRecord& rootRecord) {

			  ttTemporaryIndices.resize(miniTreeCount);
			  ttLeftPartition.resize(miniTreeCount);
			  ttAccumulatedArea.resize(miniTreeCount);

        sortTopTree();

			  rootRecord.prims = range<size_t>(0, miniTreeCount);
			  rootRecord.pinfo = PrimInfo(0, miniTreeCount, rootRecord.pinfo.geomBounds, rootRecord.pinfo.centBounds);
			  rootRecord.depth = 0;
        buildTopTreeRecursive(rootRecord, nullptr);
		  }

		  void pruneRecursive(int& newMiniTreeCount, const float averageArea, BVH8::NodeRef& nf) {
			  for (int j = 0; j < branchingFactor; j++) {
				  if (nf.node()->child(j).isNode() && (area(nf.node()->bounds(j)) > averageArea)) {
					  pruneRecursive(newMiniTreeCount, averageArea, nf.node()->child(j));
				  }
				  else if (nf.node()->child(j) != BVH8::emptyNode) {
					  BuildRecord br;
					  br.pinfo.geomBounds = nf.node()->bounds(j);
					  br.parent = (size_t*)&nf.node()->child(j);
					  miniTreeRecordsPruned[newMiniTreeCount++] = br;
				  }
			  }
		  }

      // digs in to mini trees and removes too large pieces, which become mini trees of their own
		  void prune() {
			  float averageArea = 0;
			  for (int i = 0; i < miniTreeCount; i++) {
				  averageArea += area(miniTreeRecords[i].pinfo.geomBounds);
			  }
			  averageArea /= (float)miniTreeCount;

			  averageArea *= PRUNING_THRESHOLD;
			  int newMiniTreeCount = 0;
			  int originalMiniTreeCount = miniTreeCount;
			  for (int i = 0; i < originalMiniTreeCount; i++) {

				  BVH8::NodeRef nf = BVH8::NodeRef(*(miniTreeRecords[i].parent));
				  if (nf.isNode() && area(miniTreeRecords[i].pinfo.geomBounds) > averageArea) {
					  for (int j = 0; j < branchingFactor; j++) {

						  if (nf.node()->child(j).isNode() && (area(nf.node()->bounds(j)) > averageArea)) {

							  pruneRecursive(newMiniTreeCount, averageArea, nf.node()->child(j));
						  }
						  else if (nf.node()->child(j) != BVH8::emptyNode) {
							  BuildRecord br;
							  br.pinfo.geomBounds = nf.node()->bounds(j);
							  br.parent = (size_t*)&nf.node()->child(j);
							  miniTreeRecordsPruned[newMiniTreeCount++] = br;
						  }
					  }
				  }
				  else if (nf != BVH8::emptyNode){
					miniTreeRecordsPruned[newMiniTreeCount++] = miniTreeRecords[i];
				  }
			  }
			  miniTreeCount = newMiniTreeCount;

		  }


		  __forceinline void printTime(std::string s, std::chrono::high_resolution_clock::time_point& t0) const {

			  std::chrono::duration<float> time = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::high_resolution_clock::now()
 - t0);
			  std::cout << s << ": " << time.count() * 1000.f  << "ms" << std::endl;
		  }

		  __forceinline std::chrono::high_resolution_clock::time_point startTime() {
			  return std::chrono::high_resolution_clock::now();
		  }



        /*! builder entry function */
        __forceinline const ReductionTy operator() (BuildRecord& record)
        {

      // should be more sofisticated, top tree sweep is performed
      // on an array of buildrecords, but should only be on BBoxes and indices
      int maxMiniTrees = (numberOfTriangles/MAX_MINI_TREE_SIZE) > 0 ? (numberOfTriangles/MAX_MINI_TREE_SIZE) : 1;
			miniTreeRecords.resize(maxMiniTrees*4);
			miniTreeRecordsPruned.resize(maxMiniTrees*4*64);
			miniTreeCount = 0;
			underFilledNodes = 0;



			//auto t = startTime();
			miniTreeSelectRescursive(record);

			//printTime("\nMini tree select", t);
			//std::cout << "Mini tree count: " << miniTreeCount << std::endl;
			miniTreeRoots.resize(miniTreeCount);

		//	t = startTime();
			buildMiniTrees(miniTreeRoots);
		//	printTime("Build mini trees", t);

		//	t = startTime();
			if (PRUNING_THRESHOLD < 1.f && PRUNING_THRESHOLD > 0.f) {
				prune();
				//miniTreeRecordsPruned = miniTreeRecords;
			}
		//	printTime("Pruning", t);
		//	std::cout << "Mini tree count after pruning: " << miniTreeCount << std::endl;
		//	t = startTime();
			if (miniTreeCount > 1)
				buildTopTree(record);
			else
				record = miniTreeRecords[0];
		//	printTime("Build top tree", t);
			//std::cout << "Underfilled nodes: " << underFilledNodes << std::endl;
			return 0;
	    }

      private:
        Heuristic& heuristic;
        const ReductionTy identity;
        CreateAllocFunc& createAlloc;
        CreateNodeFunc& createNode;
        UpdateNodeFunc& updateNode;
        CreateLeafFunc& createLeaf;
        ProgressMonitor& progressMonitor;

      private:
        const PrimInfo& pinfo;
        const size_t branchingFactor;
        const size_t maxDepth;
        const size_t logBlockSize;
        const size_t minLeafSize;
        const size_t maxLeafSize;
        const float travCost;
        const float intCost;
      };

    /* SAH builder that operates on an array of BuildRecords */
    struct BVHBuilderBonsai
    {
      typedef range<size_t> Set;
      typedef HeuristicBonsai<PrimRef> Heuristic;
		typedef GeneralBuildRecord<Set, typename Heuristic::Split> BuildRecord;

      /*! standard build without reduction */
      template<typename NodeRef,
        typename CreateAllocFunc,
        typename CreateNodeFunc,
        typename CreateLeafFunc,
        typename ProgressMonitor>

        static void build(NodeRef& root,
                          CreateAllocFunc createAlloc,
                          CreateNodeFunc createNode,
                          CreateLeafFunc createLeaf,
                          ProgressMonitor progressMonitor,
                          PrimRef* prims, const PrimInfo& pinfo,
                          const size_t branchingFactor, const size_t maxDepth, const size_t blockSize,
                          const size_t minLeafSize, const size_t maxLeafSize,
                          const float travCost, const float intCost, Scene* scene, const bool splitTriangles, int numOfPrimitives)
      {

        /* use dummy reduction over integers */
        int identity = 0;
        auto updateNode = [] (int node, int*, size_t) -> int { return 0; };



        /* initiate builder */
        build_reduce(root,
                     createAlloc,
                     identity,
                     createNode,
                     updateNode,
                     createLeaf,
                     progressMonitor,
                     prims,
                     pinfo,
                     branchingFactor,maxDepth,blockSize,
                     minLeafSize,maxLeafSize,travCost,intCost, scene, splitTriangles, numOfPrimitives);
      }

      /*! special builder that propagates reduction over the tree */
      template<typename NodeRef,
        typename CreateAllocFunc,
        typename ReductionTy,
        typename CreateNodeFunc,
        typename UpdateNodeFunc,
        typename CreateLeafFunc,
        typename ProgressMonitor>

        static ReductionTy build_reduce(NodeRef& root,
                                        CreateAllocFunc createAlloc,
                                        const ReductionTy& identity,
                                        CreateNodeFunc createNode, UpdateNodeFunc updateNode, CreateLeafFunc createLeaf,
                                        ProgressMonitor progressMonitor,
                                        PrimRef* prims, const PrimInfo& pinfo,
                                        const size_t branchingFactor, const size_t maxDepth, const size_t blockSize,
                                        const size_t minLeafSize, const size_t maxLeafSize,
                                        const float travCost, const float intCost, Scene* scene, const bool splitTriangles, int numOfPrimitives)
      {
        /* builder wants log2 of blockSize as input */
        const size_t logBlockSize = __bsr(blockSize);
        //assert((blockSize ^ (size_t(1) << logBlockSize)) == 0);

        /* instantiate array binning heuristic */
        Heuristic heuristic(prims, scene, numOfPrimitives);
        typedef GeneralBVHBuilder<
          BuildRecord,
          Heuristic,
          ReductionTy,
          decltype(createAlloc()),
          CreateAllocFunc,
          CreateNodeFunc,
          UpdateNodeFunc,
          CreateLeafFunc,
          ProgressMonitor> Builder;

        /* instantiate builder */
        Builder builder(heuristic,
                        identity,
                        createAlloc,
                        createNode,
                        updateNode,
                        createLeaf,
                        progressMonitor,
                        pinfo,
                        branchingFactor,maxDepth,logBlockSize,
                        minLeafSize,maxLeafSize,travCost,intCost, numOfPrimitives, splitTriangles, scene);

        /* build hierarchy */
        //BuildRecord br(pinfo,1,(size_t*)&root,Set(0,pinfo.size()));
        BuildRecord br(pinfo,1,(size_t*)&root,Set(0,pinfo.size()), Set(0,pinfo.size()*2));
        return builder(br);
      }
    };

      }
}
