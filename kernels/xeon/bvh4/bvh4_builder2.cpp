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

#include "bvh4.h"
#include "bvh4_builder2.h"
#include "bvh4_refit.h"
#include "bvh4_rotate.h"
#include "bvh4_statistics.h"

#include "builders/heuristics.h"
#include "builders/splitter_fallback.h"

#define ROTATE_TREE 1

#include "common/scene_triangle_mesh.h"

namespace embree
{
  namespace isa
  {
    void BVH4Builder2::build(size_t threadIndex, size_t threadCount) 
    {
      size_t numPrimitives = source->size();
      bvh->init(numPrimitives);
      if (source->isEmpty()) 
	return;
      
      if (g_verbose >= 2) 
	std::cout << "building BVH4<" << bvh->primTy.name << "> with SAH builder2 ... " << std::flush;
      
      double t0 = 0.0, t1 = 0.0f;
      if (g_verbose >= 2 || g_benchmark)
	t0 = getSeconds();
      
      /* generate list of build primitives */
      TriRefList prims; PrimInfo pinfo(empty);
      TriRefListGen::generate(threadIndex,threadCount,&alloc,(Scene*)geometry,prims,pinfo);
      
      const Split split = find(threadIndex,threadCount,prims,pinfo); // FIXME: enable parallel
      tasks.push_back(SplitTask(threadIndex,threadCount,this,bvh->root,1,prims,pinfo,split));

#if 1
      /* work in multithreaded toplevel mode until sufficient subtasks got generated */
      while (tasks.size() < threadCount)
      {
	/* pop largest item for better load balancing */
	SplitTask task = tasks.front();
	pop_heap(tasks.begin(),tasks.end());
	tasks.pop_back();
	
	/* process this item in parallel */
	task.recurse<true>(threadIndex,threadCount,NULL);
      }
#endif

      /*! process each generated subtask in its own thread */
      TaskScheduler::executeTask(threadIndex,threadCount,_buildFunction,this,tasks.size(),"BVH4Builder2::build");
                  
      /* finish build */
#if ROTATE_TREE
      for (int i=0; i<5; i++) 
	BVH4Rotate::rotate(bvh,bvh->root);
#endif

      /* layout top nodes */
      bvh->root = layout_top_nodes(threadIndex,bvh->root);
      //bvh->clearBarrier(bvh->root);
      bvh->numPrimitives = pinfo.size();
      bvh->bounds = pinfo.geomBounds;
      
      /* free all temporary blocks */
      Alloc::global.clear();
      
      if (g_verbose >= 2 || g_benchmark) 
	t1 = getSeconds();
      
      if (g_verbose >= 2) {
	std::cout << "[DONE]" << std::endl;
	std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(source->size())/(t1-t0) << " Mprim/s" << std::endl;
	std::cout << BVH4Statistics(bvh).str();
      }
      
      if (g_benchmark) {
	BVH4Statistics stat(bvh);
	std::cout << "BENCHMARK_BUILD " << 1000.0f*(t1-t0) << " " << 1E-6*double(source->size())/(t1-t0) << " " << stat.bytesUsed() << std::endl;
      }
    }

    BVH4::NodeRef BVH4Builder2::layout_top_nodes(size_t threadIndex, NodeRef node)
    {
      if (node.isBarrier()) {
	node.clearBarrier();
	return node;
      }
      else if (!node.isLeaf()) 
      {
	Node* src = node.node();
	Node* dst = bvh->allocNode(threadIndex);

	//std::pair<float,int> order[4];
	//for (size_t i=0; i<BVH4::N; i++)
	//  order[i] = std::pair<float,int>(=area(src->bounds(i)),i);
	//std::sort(&order[0],&order[4]);

	for (size_t i=0; i<BVH4::N; i++) {
	  dst->set(i,src->bounds(i),layout_top_nodes(threadIndex,src->child(i)));
	}
	return bvh->encodeNode(dst);
      }
      else
	return node;
    }
    
    BVH4Builder2::BVH4Builder2 (BVH4* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize)
      : source(source), geometry(geometry), primTy(bvh->primTy), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize), bvh(bvh),
	taskQueue(/*Heuristic::depthFirst ?*/ TaskScheduler::GLOBAL_BACK /*: TaskScheduler::GLOBAL_FRONT*/)
    {
      size_t maxLeafPrims = BVH4::maxLeafBlocks*primTy.blockSize;
      if (maxLeafPrims < this->maxLeafSize) 
	this->maxLeafSize = maxLeafPrims;
    }
    
    template<>
    void BVH4Builder2::recurse<true>(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event, 
				     NodeRef& node, size_t depth, TriRefList& prims, const PrimInfo& pinfo, const Split& split)
    {
      tasks.push_back(SplitTask(threadIndex,threadCount,this,node,depth,prims,pinfo,split));
      push_heap(tasks.begin(),tasks.end());
    }

    template<>
    void BVH4Builder2::recurse<false>(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event, 
				      NodeRef& node, size_t depth, TriRefList& prims, const PrimInfo& pinfo, const Split& split)
    {
      /* use full single threaded build for small jobs */
      if (pinfo.size() < 4*1024) 
	new BuildTask(threadIndex,threadCount,event,this,node,depth,prims,pinfo,split);
      
      /* use single threaded split for medium size jobs  */
      else
	new SplitTask(threadIndex,threadCount,event,this,node,depth,prims,pinfo,split);
    }

    void BVH4Builder2::buildFunction(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
    {
      SplitTask& task = tasks[taskIndex];
      recurse<false>(threadIndex,threadCount,event,*task.dst,task.depth,task.prims,task.pinfo,task.split);
    }

    BVH4Builder2::BuildTask::BuildTask(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event, 
				       BVH4Builder2* parent, NodeRef& node, size_t depth, 
				       TriRefList& prims, const PrimInfo& pinfo, const Split& split)
      : threadIndex(threadIndex), threadCount(threadCount), parent(parent), dst(node), depth(depth), prims(prims), pinfo(pinfo), split(split)
    {
      new (&task) TaskScheduler::Task(event,_run,this,"build::full");
      TaskScheduler::addTask(threadIndex,parent->taskQueue,&task);
    }
    
    typename BVH4Builder2::NodeRef BVH4Builder2::createLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo)
    {
      /* allocate leaf node */
      size_t blocks = primTy.blocks(pinfo.size());
      char* leaf = bvh->allocPrimitiveBlocks(threadIndex,blocks);
      assert(blocks <= (size_t)BVH4::maxLeafBlocks);
      
      /* insert all triangles */
      TriRefList::block_iterator_unsafe iter(prims);
      for (size_t i=0; i<blocks; i++) {
	primTy.pack(leaf+i*primTy.bytes,iter,geometry);
      }
      assert(!iter);
      
      /* free all primitive blocks */
      while (TriRefList::item* block = prims.take())
	alloc.free(threadIndex,block);
      
      return bvh->encodeLeaf(leaf,blocks);
    }
    
    typename BVH4Builder2::NodeRef BVH4Builder2::createLargeLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo, size_t depth)
    {
#if defined(_DEBUG)
      if (depth >= BVH4::maxBuildDepthLeaf) 
	throw std::runtime_error("ERROR: Loosing primitives during build.");
#endif
      
      /* create leaf for few primitives */
      if (pinfo.size() <= maxLeafSize)
	return createLeaf(threadIndex,prims,pinfo);
      
      /* first level */
      TriRefList prims0, prims1;
      PrimInfo   cinfo0, cinfo1;
      FallBackSplit::find(threadIndex,alloc,prims,prims0,cinfo0,prims1,cinfo1);
      
      /* second level */
      TriRefList cprims[4];
      PrimInfo   cinfo[4];
      FallBackSplit::find(threadIndex,alloc,prims0,cprims[0],cinfo[0],cprims[1],cinfo[1]);
      FallBackSplit::find(threadIndex,alloc,prims1,cprims[2],cinfo[2],cprims[3],cinfo[3]);
      
      /*! create an inner node */
      Node* node = bvh->allocNode(threadIndex);
      for (size_t i=0; i<4; i++) node->set(i,cinfo[i].geomBounds,createLargeLeaf(threadIndex,cprims[i],cinfo[i],depth+1));
      return bvh->encodeNode(node);
    }  
    
    void BVH4Builder2::BuildTask::run(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event)
    {
      this->threadIndex = threadIndex;
      dst = recurse(depth,prims,pinfo,split);
#if ROTATE_TREE
      for (int i=0; i<5; i++) 
	BVH4Rotate::rotate(parent->bvh,dst); 
#endif
      dst.setBarrier();
      delete this;
    }

    const Split BVH4Builder2::find(size_t threadIndex, size_t threadCount, TriRefList& prims, const PrimInfo& pinfo)
    {
      ObjectPartition::Split osplit = ObjectPartition::find<false>(threadIndex,threadCount,      prims,pinfo,2); // FIXME: hardcoded constant
      //return osplit;
      SpatialSplit   ::Split ssplit = SpatialSplit   ::find<false>(threadIndex,threadCount,(Scene*)geometry,prims,pinfo,2); // FIXME: hardcoded constant
      const float bestSAH = min(osplit.sah,ssplit.sah);
      if      (bestSAH == osplit.sah) return osplit; 
      else if (bestSAH == ssplit.sah) return ssplit;
      else                            return Split();
    }
    
    typename BVH4Builder2::NodeRef BVH4Builder2::BuildTask::recurse(size_t depth, TriRefList& prims, const PrimInfo& pinfo, const Split& split)
    {
      /*! compute leaf and split cost */
      const float leafSAH  = parent->primTy.intCost*pinfo.leafSAH(2);
      const float splitSAH = BVH4::travCost*halfArea(pinfo.geomBounds)+parent->primTy.intCost*split.splitSAH();
      assert(TriRefList::block_iterator_unsafe(prims).size() == pinfo.size());
      assert(pinfo.size() == 0 || leafSAH >= 0 && splitSAH >= 0);
      
      /*! create a leaf node when threshold reached or SAH tells us to stop */
      if (pinfo.size() <= parent->minLeafSize || depth > BVH4::maxBuildDepth || (pinfo.size() <= parent->maxLeafSize && leafSAH <= splitSAH)) {
	return parent->createLargeLeaf(threadIndex,prims,pinfo,depth+1);
      }
      
      /*! initialize child list */
      TriRefList cprims[BVH4::N]; cprims[0] = prims;
      PrimInfo   cinfo [BVH4::N]; cinfo [0] = pinfo;
      Split      csplit[BVH4::N]; csplit[0] = split;
      size_t numChildren = 1;
      
      /*! split until node is full or SAH tells us to stop */
      do {
	
	/*! find best child to split */
	float bestSAH = 0; 
	ssize_t bestChild = -1;
	for (size_t i=0; i<numChildren; i++) 
	{
	  float dSAH = csplit[i].splitSAH()-cinfo[i].leafSAH(2);
	  if (cinfo[i].size() <= parent->minLeafSize) continue; 
	  if (cinfo[i].size() > parent->maxLeafSize) dSAH = min(0.0f,dSAH); //< force split for large jobs
	  if (dSAH <= bestSAH) { bestChild = i; bestSAH = dSAH; }
	}
	if (bestChild == -1) break;
	
	/*! perform best found split and find new splits */
	PrimInfo linfo(empty),rinfo(empty);
	TriRefList lprims,rprims;
	csplit[bestChild].split<false>(threadIndex,threadCount,parent->alloc,(Scene*)parent->geometry,cprims[bestChild],lprims,linfo,rprims,rinfo);
	//const Split lsplit = ObjectPartition::find<false>(threadIndex,threadCount,lprims,linfo,2);
	//const Split rsplit = ObjectPartition::find<false>(threadIndex,threadCount,rprims,rinfo,2);
	const Split lsplit = parent->find(threadIndex,threadCount,lprims,linfo);
	const Split rsplit = parent->find(threadIndex,threadCount,rprims,rinfo);
	cprims[bestChild  ] = lprims; cinfo[bestChild  ] = linfo; csplit[bestChild  ] = lsplit;
	cprims[numChildren] = rprims; cinfo[numChildren] = rinfo; csplit[numChildren] = rsplit;
	numChildren++;
	
      } while (numChildren < BVH4::N);
      
      /*! create an inner node */
      Node* node = parent->bvh->allocNode(threadIndex);
      for (size_t i=0; i<numChildren; i++) node->set(i,cinfo[i].geomBounds,recurse(depth+1,cprims[i],cinfo[i],csplit[i]));
      return parent->bvh->encodeNode(node);
    }
    
    BVH4Builder2::SplitTask::SplitTask(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event, 
				       BVH4Builder2* parent, NodeRef& node, size_t depth, 
				       TriRefList& prims, const PrimInfo& pinfo, const Split& split)
      : parent(parent), dst(&node), depth(depth), prims(prims), pinfo(pinfo), split(split)
    {
      new (&task) TaskScheduler::Task(event,_run,this,"build::split");
      TaskScheduler::addTask(threadIndex,parent->taskQueue,&task);
    }
    
    BVH4Builder2::SplitTask::SplitTask(size_t threadIndex, size_t threadCount, 
				       BVH4Builder2* parent, NodeRef& node, size_t depth, 
				       TriRefList& prims, const PrimInfo& pinfo, const Split& split)
      : parent(parent), dst(&node), depth(depth), prims(prims), pinfo(pinfo), split(split) {}
    
    void BVH4Builder2::SplitTask::run(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event) {
      recurse<false>(threadIndex,threadCount,event);
    }
    
    template<bool PARALLEL>
    void BVH4Builder2::SplitTask::recurse(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event)
    {
      /*! compute leaf and split cost */
      const float leafSAH  = parent->primTy.intCost*pinfo.leafSAH(2);
      const float splitSAH = BVH4::travCost*halfArea(pinfo.geomBounds)+parent->primTy.intCost*split.splitSAH();
      assert(TriRefList::block_iterator_unsafe(prims).size() == pinfo.size());
      assert(pinfo.size() == 0 || leafSAH >= 0 && splitSAH >= 0);
      
      /*! create a leaf node when threshold reached or SAH tells us to stop */
      if (pinfo.size() <= parent->minLeafSize || depth > BVH4::maxBuildDepth || (pinfo.size() <= parent->maxLeafSize && leafSAH <= splitSAH)) {
	*dst = parent->createLargeLeaf(threadIndex,prims,pinfo,depth+1); return;
      }
      
      /*! initialize child list */
      TriRefList cprims[BVH4::N]; 
      PrimInfo   cinfo [BVH4::N]; 
      Split      csplit[BVH4::N]; 
      cprims[0] = prims;
      cinfo [0] = pinfo;
      csplit[0] = split;
      size_t numChildren = 1;
      
      /*! split until node is full or SAH tells us to stop */
      do {
	
	/*! find best child to split */
	float bestSAH = 0; 
	ssize_t bestChild = -1;
	for (size_t i=0; i<numChildren; i++) 
	{
	  float dSAH = csplit[i].splitSAH()-cinfo[i].leafSAH(2);
	  if (cinfo[i].size() <= parent->minLeafSize) continue; 
	  if (cinfo[i].size() > parent->maxLeafSize) dSAH = min(0.0f,dSAH); //< force split for large jobs
	  if (dSAH <= bestSAH) { bestChild = i; bestSAH = dSAH; }
	}
	if (bestChild == -1) break;
	
	/*! perform best found split and find new splits */
	PrimInfo linfo(empty), rinfo(empty);
	TriRefList lprims,rprims;
	csplit[bestChild].split<PARALLEL>(threadIndex,threadCount,parent->alloc,(Scene*)parent->geometry,cprims[bestChild],lprims,linfo,rprims,rinfo);
	//const Split lsplit = ObjectPartition::find<PARALLEL>(threadIndex,threadCount,lprims,linfo,2);
	//const Split rsplit = ObjectPartition::find<PARALLEL>(threadIndex,threadCount,rprims,rinfo,2);
	const Split lsplit = parent->find(threadIndex,threadCount,lprims,linfo); // FIXME: not in parallel mode
	const Split rsplit = parent->find(threadIndex,threadCount,rprims,rinfo);
	cprims[bestChild  ] = lprims; cinfo[bestChild  ] = linfo; csplit[bestChild  ] = lsplit;
	cprims[numChildren] = rprims; cinfo[numChildren] = rinfo; csplit[numChildren] = rsplit;
	numChildren++;
	
      } while (numChildren < BVH4::N);
      
      /*! create an inner node */
      Node* node = parent->bvh->allocNode(threadIndex);
      *dst = parent->bvh->encodeNode(node);
      for (size_t i=0; i<numChildren; i++) {
	node->set(i,cinfo[i].geomBounds,0);
	parent->recurse<PARALLEL>(threadIndex,threadCount,event,node->child(i),depth+1,cprims[i],cinfo[i],csplit[i]);
      }
    }
    
#if 1
    /*Builder* BVH4Builder2ObjectSplit1 (void* accel, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize) {
      return new BVH4Builder2((BVH4*)accel,source,geometry,minLeafSize,maxLeafSize);
      }*/
    
    Builder* BVH4Builder2ObjectSplit4 (void* accel, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize) {
      return new BVH4Builder2((BVH4*)accel,source,geometry,minLeafSize,maxLeafSize);
    }
    
    /*Builder* BVH4Builder2ObjectSplit8 (void* accel, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize) {
      return new BVH4Builder2((BVH4*)accel,source,geometry,minLeafSize,maxLeafSize);
    }
    
    Builder* BVH4Builder2SpatialSplit1 (void* accel, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize) {
      return new BVH4Builder2((BVH4*)accel,source,geometry,minLeafSize,maxLeafSize);
    }
    
    Builder* BVH4Builder2SpatialSplit4 (void* accel, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize) {
      return new BVH4Builder2((BVH4*)accel,source,geometry,minLeafSize,maxLeafSize);
    }
    
    Builder* BVH4Builder2SpatialSplit8 (void* accel, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize) {
      return new BVH4Builder2((BVH4*)accel,source,geometry,minLeafSize,maxLeafSize);
      }*/
#endif
  }
}
