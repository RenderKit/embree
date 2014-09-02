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

#include "bvh4.h"
#include "bvh4_builder_hair_mb.h"
#include "bvh4_statistics.h"
#include "common/scene_bezier_curves.h"
#include "../builders/bezierrefgen.h"
#include <algorithm>
#include "geometry/bezier1.h"
#include "geometry/bezier1i.h"

namespace embree
{
  extern double g_hair_builder_replication_factor;
 
  namespace isa
  {
    //template<> BVH4BuilderHairMBT<Bezier1 >::BVH4BuilderHairMBT (BVH4* bvh, Scene* scene, size_t mode) : BVH4BuilderHairMB(bvh,scene,mode) {}
    template<> BVH4BuilderHairMBT<Bezier1iMB>::BVH4BuilderHairMBT (BVH4* bvh, Scene* scene, size_t mode) : BVH4BuilderHairMB(bvh,scene,mode) {}

    BVH4BuilderHairMB::BVH4BuilderHairMB (BVH4* bvh, Scene* scene, size_t mode)
      : scene(scene), minLeafSize(1), maxLeafSize(inf), enableSpatialSplits(mode > 0), bvh(bvh), remainingReplications(0)
    {
      if (BVH4::maxLeafBlocks < this->maxLeafSize) 
	this->maxLeafSize = BVH4::maxLeafBlocks;
    }
    
    void BVH4BuilderHairMB::build(size_t threadIndex, size_t threadCount) 
    {
      /* fast path for empty BVH */
      size_t numPrimitives = scene->numBezierCurves2;
      bvh->init(numPrimitives,numPrimitives+(size_t)(g_hair_builder_replication_factor*numPrimitives));
      if (numPrimitives == 0) return;
      numGeneratedPrims = 0;
      
      //while (true)
      {
	double t0 = 0.0;
	if (g_verbose >= 2) 
	{
	  std::cout << "building ";
#if BVH4HAIR_COMPRESS_UNALIGNED_NODES
	  std::cout << "Compressed";
#endif
	  std::cout << "BVH4<" + bvh->primTy.name + "> using " << TOSTRING(isa) << "::BVH4BuilderHairMB ..." << std::flush;
	  t0 = getSeconds();
	}
	
	size_t N = 0;
	float r = 0;
	
	/* create initial curve list */
	size_t numVertices = 0;
	BezierRefGen gen(threadIndex,threadCount,&alloc,scene,2);
	PrimInfo pinfo = gen.pinfo;
	BezierRefList prims = gen.prims;
	
	bvh->numPrimitives = numPrimitives;
	bvh->numVertices = 0;
	if (&bvh->primTy == &SceneBezier1i::type) bvh->numVertices = numVertices;
	
	/* start recursive build */
	remainingReplications = g_hair_builder_replication_factor*numPrimitives;
	bvh->bounds = pinfo.geomBounds;
	
#if 0
	const Split split = find_split(threadIndex,threadCount,prims,pinfo,pinfo.geomBounds);
	BuildTask task(&bvh->root,0,prims,pinfo,pinfo.geomBounds,split); recurseTask(threadIndex,task);
#else
	const Split split = find_split<true>(threadIndex,threadCount,prims,pinfo,pinfo.geomBounds,pinfo);
	BuildTask task(&bvh->root,0,prims,pinfo,pinfo.geomBounds,pinfo,split);
	numActiveTasks = 1;
	tasks.push_back(task);
	std::push_heap(tasks.begin(),tasks.end());
	
#if 1
	while (tasks.front().pinfo.size() > 200000)
	{
	  BuildTask task = tasks.front();
	  std::pop_heap(tasks.begin(),tasks.end());
	  tasks.pop_back();
	  
	  size_t numChildren;
	  BuildTask ctasks[BVH4::N];
	  processTask<true>(threadIndex,threadCount,task,ctasks,numChildren);
	  
	  for (size_t i=0; i<numChildren; i++) {
	    atomic_add(&numActiveTasks,+1);
	    tasks.push_back(ctasks[i]);
	    std::push_heap(tasks.begin(),tasks.end());
	  }
	  atomic_add(&numActiveTasks,-1);
	}
#endif
	
	TaskScheduler::executeTask(threadIndex,threadCount,_task_build_parallel,this,threadCount,"BVH4Builder::build_parallel");
#endif

	if (g_verbose >= 2) {
	  double t1 = getSeconds();
	  std::cout << " [DONE]" << std::endl;
	  std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(numPrimitives)/(t1-t0) << " Mprim/s" << std::endl;
	  std::cout << BVH4Statistics(bvh).str();
	}
      }
    }

    template<typename Primitive>
    BVH4::NodeRef BVH4BuilderHairMBT<Primitive>::createLeaf(size_t threadIndex, size_t depth, BezierRefList& prims, const PrimInfo& pinfo)
    {
      size_t N = pinfo.size();
      
      if (N > (size_t)BVH4::maxLeafBlocks) {
	//std::cout << "WARNING: Loosing " << N-BVH4::maxLeafBlocks << " primitives during build!" << std::endl;
	std::cout << "!" << std::flush;
	N = (size_t)BVH4::maxLeafBlocks;
      }
      if (g_verbose >= 1) {
        size_t numGeneratedPrimsOld = atomic_add(&numGeneratedPrims,N); 
        if (numGeneratedPrimsOld%10000 > (numGeneratedPrimsOld+N)%10000) std::cout << "." << std::flush; 
      }
      //assert(N <= (size_t)BVH4::maxLeafBlocks);
     
      Primitive* leaf = (Primitive*) bvh->allocPrimitiveBlocks(threadIndex,N);
      BezierRefList::block_iterator_unsafe iter(prims);
      for (size_t i=0; i<N; i++) leaf[i].fill(iter,scene);
      assert(!iter);
      
      /* free all primitive blocks */
      while (BezierRefList::item* block = prims.take())
	alloc.free(threadIndex,block);
      
      return bvh->encodeLeaf((char*)leaf,N);
    }

    BVH4::NodeRef BVH4BuilderHairMB::createLargeLeaf(size_t threadIndex, BezierRefList& prims, const PrimInfo& pinfo, size_t depth)
    {
#if defined(_DEBUG)
      if (depth >= BVH4::maxBuildDepthLeaf) 
	throw std::runtime_error("ERROR: Loosing primitives during build.");
#endif
      
      /* create leaf for few primitives */
      if (pinfo.size() <= BVH4::maxLeafBlocks)
	return createLeaf(threadIndex,depth,prims,pinfo);
      
      /* first level */
      BezierRefList prims0, prims1;
      PrimInfo   cinfo0, cinfo1;
      FallBackSplit::find(threadIndex,alloc,prims,prims0,cinfo0,prims1,cinfo1);
      
      /* second level */
      BezierRefList cprims[4];
      PrimInfo   cinfo[4];
      FallBackSplit::find(threadIndex,alloc,prims0,cprims[0],cinfo[0],cprims[1],cinfo[1]);
      FallBackSplit::find(threadIndex,alloc,prims1,cprims[2],cinfo[2],cprims[3],cinfo[3]);
      
      /*! create an inner node */
      BVH4::Node* node = bvh->allocNode(threadIndex);
      for (size_t i=0; i<4; i++) {
        node->set(i,cinfo[i].geomBounds);
        node->set(i,createLargeLeaf(threadIndex,cprims[i],cinfo[i],depth+1));
      }
      return bvh->encodeNode(node);
    }  

    template<bool Parallel>
    Split BVH4BuilderHairMB::find_split(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo, const NAABBox3fa& bounds, const PrimInfo& sinfo)
    {
      /* variable to track the SAH of the best splitting approach */
      float bestSAH = inf;
      const float leafSAH = BVH4::intCost*float(pinfo.size())*halfArea(bounds.bounds);
      
      const size_t threshold = 100;

      /* perform standard binning in aligned space */
      ObjectPartition::Split alignedObjectSplit;
      float alignedObjectSAH = inf;
#if 1
      if (pinfo.size() > threshold) {
        alignedObjectSplit = ObjectPartition::find<Parallel>(threadIndex,threadCount,prims,pinfo,0); // FIXME: hardcoded 0
        alignedObjectSAH = BVH4::travCostAligned*halfArea(bounds.bounds) + BVH4::intCost*alignedObjectSplit.splitSAH();
        bestSAH = min(bestSAH,alignedObjectSAH);
      }
#endif
      
      /* perform standard binning in unaligned space */
      ObjectPartitionUnaligned::Split unalignedObjectSplit;
      float unalignedObjectSAH = inf;
#if 1
      if (pinfo.size() <= threshold) {
      //if (alignedObjectSAH > 0.7f*leafSAH) {
	if (sinfo.size()) 
	  unalignedObjectSplit = ObjectPartitionUnaligned::find<Parallel>(threadIndex,threadCount,prims,bounds.space,sinfo);
	else {
	  const LinearSpace3fa space = ObjectPartitionUnaligned::computeAlignedSpace(threadIndex,threadCount,prims); 
	  const PrimInfo       sinfo = ObjectPartitionUnaligned::computePrimInfo    <Parallel>(threadIndex,threadCount,prims,space);
	  unalignedObjectSplit = ObjectPartitionUnaligned::find<Parallel>(threadIndex,threadCount,prims,space,sinfo);
	}    	
	unalignedObjectSAH = BVH4::travCostUnaligned*halfArea(bounds.bounds) + BVH4::intCost*unalignedObjectSplit.splitSAH();
	bestSAH = min(bestSAH,unalignedObjectSAH);
      }
#endif
      
      /* perform splitting into two strands */
      StrandSplit::Split strandSplit;
      float strandSAH = inf;
#if 1
      //if (alignedObjectSAH > 0.6f*leafSAH) {
      if (pinfo.size() <= threshold) {
	strandSplit = StrandSplit::find<Parallel>(threadIndex,threadCount,prims);
	strandSAH = BVH4::travCostUnaligned*halfArea(bounds.bounds) + BVH4::intCost*strandSplit.splitSAH();
	bestSAH = min(bestSAH,strandSAH);
      }
#endif
 
      /* return best split */
      if      (bestSAH == float(inf)        ) return Split();
      else if (bestSAH == alignedObjectSAH  ) return Split(alignedObjectSplit);
      else if (bestSAH == unalignedObjectSAH) return Split(unalignedObjectSplit);
      else if (bestSAH == strandSAH         ) return Split(strandSplit);
      else throw std::runtime_error("bvh4hair_builder: internal error");
    }
    
    template<bool Parallel>
    __forceinline void BVH4BuilderHairMB::processTask(size_t threadIndex, size_t threadCount, BuildTask& task, BuildTask task_o[BVH4::N], size_t& numTasks_o)
    {
      /* create enforced leaf */
      const float leafSAH  = BVH4::intCost*task.pinfo.leafSAH();
      const float splitSAH = BVH4::travCostUnaligned*halfArea(task.bounds.bounds)+BVH4::intCost*task.split.splitSAH();

      if (task.pinfo.size() <= minLeafSize || task.depth >= BVH4::maxBuildDepth || (task.pinfo.size() <= maxLeafSize && leafSAH <= splitSAH)) {
	*task.dst = createLargeLeaf(threadIndex,task.prims,task.pinfo,task.depth);
	numTasks_o = 0;
	return;
      }
      
      /*! initialize child list */
      bool isAligned = true;
      PrimInfo cpinfo     [BVH4::N]; cpinfo [0] = task.pinfo; 
      PrimInfo csinfo     [BVH4::N]; csinfo [0] = task.sinfo; 
      NAABBox3fa cbounds  [BVH4::N]; cbounds[0] = task.bounds;
      BezierRefList cprims[BVH4::N]; cprims [0] = task.prims;
      Split csplit        [BVH4::N]; csplit [0] = task.split;        
      size_t numChildren = 1;
      
      /*! split until node is full or SAH tells us to stop */
      do {
	
	/*! find best child to split */
	float bestSAH = 0; 
	ssize_t bestChild = -1;
	for (size_t i=0; i<numChildren; i++) 
	{
	  float dSAH = csplit[i].splitSAH()-cpinfo[i].leafSAH();
	  if (cpinfo[i].size() <= minLeafSize) continue; 
	  if (cpinfo[i].size() > maxLeafSize) dSAH = min(0.0f,dSAH); //< force split for large jobs
	  if (dSAH <= bestSAH) { bestChild = i; bestSAH = dSAH; }
	}
	if (bestChild == -1) break;
	
	/*! split selected child */
	PrimInfo linfo(empty), rinfo(empty);
	BezierRefList lprims, rprims;
	csplit[bestChild].split<Parallel>(threadIndex,threadCount,alloc,scene,cprims[bestChild],lprims,linfo,rprims,rinfo);

	const ssize_t replications = linfo.size()+rinfo.size()-cpinfo[bestChild].size(); assert(replications >= 0);
	isAligned &= csplit[bestChild].isAligned;
	LinearSpace3fa lspace,rspace;
	PrimInfo lsinfo(empty),rsinfo(empty);
	if (!isAligned) {
	  lspace = ObjectPartitionUnaligned::computeAlignedSpace(threadIndex,threadCount,lprims); 
	  rspace = ObjectPartitionUnaligned::computeAlignedSpace(threadIndex,threadCount,rprims); 
	  lsinfo = ObjectPartitionUnaligned::computePrimInfo    <Parallel>(threadIndex,threadCount,lprims,lspace);
	  rsinfo = ObjectPartitionUnaligned::computePrimInfo    <Parallel>(threadIndex,threadCount,rprims,rspace);
	}
	const NAABBox3fa lbounds = isAligned ? linfo.geomBounds : NAABBox3fa(lspace,lsinfo.geomBounds); 
	const NAABBox3fa rbounds = isAligned ? rinfo.geomBounds : NAABBox3fa(rspace,rsinfo.geomBounds); 

	const Split lsplit = find_split<Parallel>(threadIndex,threadCount,lprims,linfo,lbounds,lsinfo);
	const Split rsplit = find_split<Parallel>(threadIndex,threadCount,rprims,rinfo,rbounds,rsinfo);
	cprims[numChildren] = rprims; cpinfo[numChildren] = rinfo; cbounds[numChildren] = rbounds; csinfo[numChildren] = lsinfo; csplit[numChildren] = rsplit;
	cprims[bestChild  ] = lprims; cpinfo[bestChild  ] = linfo; cbounds[bestChild  ] = lbounds; csinfo[bestChild  ] = rsinfo; csplit[bestChild  ] = lsplit;
	if (replications) atomic_add(&remainingReplications,-replications); 
	numChildren++;
	
      } while (numChildren < BVH4::N);
      
      /* create aligned node */
      if (isAligned)
      {
	BVH4::NodeMB* node = bvh->allocNodeMB(threadIndex);
	for (size_t i=0; i<numChildren; i++) 
        {
          std::pair<BBox3fa,BBox3fa> bounds = ObjectPartition::computePrimInfoMB<Parallel>(threadIndex,threadCount,scene,cprims[i]);
          node->set(i,bounds.first,bounds.second);
	  new (&task_o[i]) BuildTask(&node->child(i),task.depth+1,cprims[i],cpinfo[i],cbounds[i],csinfo[i],csplit[i]);
	}
	numTasks_o = numChildren;
	*task.dst = bvh->encodeNode(node);
      }
      
      /* create unaligned node */
      else 
      {
	BVH4::UnalignedNodeMB* node = bvh->allocUnalignedNodeMB(threadIndex);
	for (size_t i=0; i<numChildren; i++) 
        {
          std::pair<AffineSpace3fa,AffineSpace3fa> spaces = ObjectPartitionUnaligned::computeAlignedSpaceMB(threadIndex,threadCount,scene,cprims[i]); 
	  //AffineSpace3fa space01 = 0.5f*(spaces.first+spaces.second); spaces.first = spaces.second = space01;
	  //Vec3fa dir = 0.5f*(spaces.first.l.row2() + spaces.second.l.row2());
	  //spaces.first = spaces.second = frame(dir).transposed();
	  ObjectPartitionUnaligned::PrimInfoMB pinfo1 = ObjectPartitionUnaligned::computePrimInfoMB<Parallel>(threadIndex,threadCount,scene,cprims[i],spaces);

          Vec3fa k0 = 0.5f*(pinfo1.s0t0.lower+pinfo1.s0t0.upper);
          Vec3fa k1 = 0.5f*(pinfo1.s1t1.lower+pinfo1.s1t1.upper);
	  Vec3fa d0(k0.x,k0.y,pinfo1.s0t0.lower.z);
          Vec3fa d1(k1.x,k1.y,pinfo1.s1t1.lower.z);
          spaces.first.p  -= d0; pinfo1.s0t0.lower -= d0; pinfo1.s0t0.upper -= d0;
          spaces.second.p -= d1; pinfo1.s1t1.lower -= d1; pinfo1.s1t1.upper -= d1;

	  ObjectPartitionUnaligned::PrimInfoMB pinfo = ObjectPartitionUnaligned::computePrimInfoMB<Parallel>(threadIndex,threadCount,scene,cprims[i],spaces);
	  
#if BVH4HAIR_MB_VERSION == 2
          Vec3fa a0 = xfmVector(spaces.first.l.transposed(),-spaces.first.p);
          Vec3fa a1 = xfmVector(spaces.second.l.transposed(),-spaces.second.p);

	  Vec3fa b0 = xfmVector(spaces.first .l.transposed(),Vec3fa(0.0f,0.0f,pinfo1.s0t0.upper.z)-spaces.first .p);
          Vec3fa b1 = xfmVector(spaces.second.l.transposed(),Vec3fa(0.0f,0.0f,pinfo1.s1t1.upper.z)-spaces.second.p);

          spaces.first.p = a0;
          spaces.second.p = a1;

	  /*spaces.first.l.vx = a0;
	  spaces.first.l.vy = b0;

	  spaces.second.l.vx = a1;
	  spaces.second.l.vy = b1;*/

	  float r0 = max(abs(pinfo.s0t0.lower.x),abs(pinfo.s0t0.lower.y),abs(pinfo.s0t0.upper.x),abs(pinfo.s0t0.upper.y));
          pinfo.s0t0.lower.x = pinfo.s0t0.lower.y = -r0;
          pinfo.s0t0.upper.x = pinfo.s0t0.upper.y = +r0;

          float r1 = max(abs(pinfo.s1t1.lower.x),abs(pinfo.s1t1.lower.y),abs(pinfo.s1t1.upper.x),abs(pinfo.s1t1.upper.y));
          pinfo.s1t1.lower.x = pinfo.s1t1.lower.y = -r1;
          pinfo.s1t1.upper.x = pinfo.s1t1.upper.y = +r1;

	  /*spaces.first.l.vz.x = r0;
	    spaces.second.l.vz.x = r1;*/

#endif

          node->set(i,spaces.first,spaces.second);
          node->set(i,pinfo.s0t0,pinfo.s0t1_s1t0,pinfo.s1t1);
	  new (&task_o[i]) BuildTask(&node->child(i),task.depth+1,cprims[i],cpinfo[i],cbounds[i],csinfo[i],csplit[i]);
	}
	numTasks_o = numChildren;
	*task.dst = bvh->encodeNode(node);
      }
    }

    void BVH4BuilderHairMB::recurseTask(size_t threadIndex, size_t threadCount, BuildTask& task)
    {
      size_t numChildren;
      BuildTask tasks[BVH4::N];
      processTask<false>(threadIndex,threadCount,task,tasks,numChildren);
      for (size_t i=0; i<numChildren; i++) 
	recurseTask(threadIndex,threadCount,tasks[i]);
    }
    
    void BVH4BuilderHairMB::task_build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
    {
      while (numActiveTasks) 
      {
	taskMutex.lock();
	if (tasks.size() == 0) {
	  taskMutex.unlock();
	  continue;
	}
	
	/* take next task from heap */
	BuildTask task = tasks.back(); tasks.pop_back();
	taskMutex.unlock();
	
	/* recursively finish task */
	if (task.pinfo.size() < 1024) {
	  atomic_add(&numActiveTasks,-1);
	  recurseTask(threadIndex,threadCount,task);
	}
	
	/* execute task and add child tasks */
	else 
	{
	  size_t numChildren;
	  BuildTask ctasks[BVH4::N];
	  processTask<false>(threadIndex,threadCount,task,ctasks,numChildren);
	  taskMutex.lock();
	  for (size_t i=0; i<numChildren; i++) {
	    atomic_add(&numActiveTasks,+1);
	    tasks.push_back(ctasks[i]);
	  }
	  atomic_add(&numActiveTasks,-1);
	  taskMutex.unlock();
	}
      }
    }

    /*! entry functions for the builder */
    //Builder* BVH4Bezier1BuilderMB  (void* bvh, Scene* scene, size_t mode) { return new class BVH4BuilderHairMBT<Bezier1> ((BVH4*)bvh,scene,mode); }
    Builder* BVH4Bezier1iMBBuilder_OBB (void* bvh, Scene* scene, size_t mode) { return new class BVH4BuilderHairMBT<Bezier1iMB> ((BVH4*)bvh,scene,mode); }
  }
}
