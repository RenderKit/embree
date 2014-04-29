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

#include "bvh4hair.h"
#include "bvh4hair_builder2.h"
#include "bvh4hair_statistics.h"
#include "common/scene_bezier_curves.h"

namespace embree
{
  extern double g_hair_builder_replication_factor;
 
  BVH4HairBuilder2::BVH4HairBuilder2 (BVH4Hair* bvh, Scene* scene)
    : scene(scene), minLeafSize(1), maxLeafSize(inf), bvh(bvh), remainingReplications(0)
  {
    if (BVH4Hair::maxLeafBlocks < this->maxLeafSize) 
      this->maxLeafSize = BVH4Hair::maxLeafBlocks;

    enableAlignedObjectSplits = false;
    enableAlignedSpatialSplits = false;
    enableUnalignedObjectSplits = false;
    enableStrandSplits = false;
    enablePreSubdivision = 0;
    
    for (size_t i=0; i<g_hair_accel_mode.size();)
    {
      if      (g_hair_accel_mode.substr(i,2) == "P0" ) { enablePreSubdivision = 0; i+=2; } 
      else if (g_hair_accel_mode.substr(i,2) == "P1" ) { enablePreSubdivision = 1; i+=2; } 
      else if (g_hair_accel_mode.substr(i,2) == "P2" ) { enablePreSubdivision = 2; i+=2; } 
      else if (g_hair_accel_mode.substr(i,2) == "P3" ) { enablePreSubdivision = 3; i+=2; } 
      else if (g_hair_accel_mode.substr(i,2) == "P4" ) { enablePreSubdivision = 4; i+=2; } 
      else if (g_hair_accel_mode.substr(i,2) == "aO" ) { enableAlignedObjectSplits = true; i+=2; } 
      else if (g_hair_accel_mode.substr(i,2) == "uO" ) { enableUnalignedObjectSplits = true; i+=2; } 
      else if (g_hair_accel_mode.substr(i,3) == "auO" ) { enableAlignedObjectSplits = enableUnalignedObjectSplits = true; i+=3; } 
      else if (g_hair_accel_mode.substr(i,3) == "uST") { enableStrandSplits = true; i+=3; } 
      else if (g_hair_accel_mode.substr(i,3) == "aSP") { enableAlignedSpatialSplits = true; i+=3; } 
      else throw std::runtime_error("invalid hair accel mode");
    }
  }

  void BVH4HairBuilder2::build(size_t threadIndex, size_t threadCount) 
  {
    /* fast path for empty BVH */
    size_t numPrimitives = scene->numCurves << enablePreSubdivision;
    bvh->init(numPrimitives,numPrimitives+(size_t)(g_hair_builder_replication_factor*numPrimitives));
    if (numPrimitives == 0) return;
    numGeneratedPrims = 0;

    double t0 = 0.0;
    if (g_verbose >= 2) 
    {
      PRINT(enableAlignedObjectSplits);
      PRINT(enableAlignedSpatialSplits);
      PRINT(enableUnalignedObjectSplits);
      PRINT(enableStrandSplits);
      PRINT(enablePreSubdivision);

      std::cout << "building BVH4Hair<" + bvh->primTy.name + "> using BVH4HairBuilder2 ..." << std::flush;
      t0 = getSeconds();
    }

    size_t N = 0;
    float r = 0;

    /* create initial curve list */
    BBox3fa bounds = empty;
    size_t numVertices = 0;
    BezierRefList prims;
    for (size_t i=0; i<scene->size(); i++) 
    {
      Geometry* geom = scene->get(i);
      if (geom->type != BEZIER_CURVES) continue;
      if (!geom->isEnabled()) continue;
      BezierCurves* set = (BezierCurves*) geom;
      numVertices += set->numVertices;
      for (size_t j=0; j<set->numCurves; j++) {
        const int ofs = set->curve(j);
        const Vec3fa& p0 = set->vertex(ofs+0);
        const Vec3fa& p1 = set->vertex(ofs+1);
        const Vec3fa& p2 = set->vertex(ofs+2);
        const Vec3fa& p3 = set->vertex(ofs+3);
        const Bezier1 bezier(p0,p1,p2,p3,0,1,i,j);
        bounds.extend(subdivideAndAdd(threadIndex,prims,bezier,enablePreSubdivision));
      }
    }

    /* compute primitive info */
    PrimInfo pinfo;
    for (BezierRefList::block_iterator_unsafe i = prims; i; i++) 
      pinfo.add((*i).bounds(),(*i).center());

    bvh->numPrimitives = scene->numCurves;
    bvh->numVertices = 0;
    if (&bvh->primTy == &SceneBezier1i::type) bvh->numVertices = numVertices;

    /* start recursive build */
    remainingReplications = g_hair_builder_replication_factor*numPrimitives;
    bvh->bounds = bounds;

#if 0
    const Split split = find_split(threadIndex,threadCount,prims,pinfo,pinfo.geomBounds);
    BuildTask task(&bvh->root,0,prims,pinfo,pinfo.geomBounds,split); recurseTask(threadIndex,task);
    /*bvh->root = recurse(threadIndex,0,prims,pinfo,pinfo.geomBounds,split);*/
#else
    const Split split = find_split(threadIndex,threadCount,prims,pinfo,pinfo.geomBounds);
    BuildTask task(&bvh->root,0,prims,pinfo,pinfo.geomBounds,split);
    numActiveTasks = 1;
    tasks.push_back(task);
    push_heap(tasks.begin(),tasks.end());
    
#if 0
    while (tasks.front().pinfo.size() > 1000000)
    {
      BuildTask task = tasks.front();
      pop_heap(tasks.begin(),tasks.end());
      tasks.pop_back();

      size_t numChildren;
      BuildTask ctasks[BVH4Hair::N];
      processLargeTask(threadIndex,threadCount,task,ctasks,numChildren);
      
      for (size_t i=0; i<numChildren; i++) {
	atomic_add(&numActiveTasks,+1);
	tasks.push_back(ctasks[i]);
	push_heap(tasks.begin(),tasks.end());
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
      std::cout << BVH4HairStatistics(bvh).str();
    }
  }

  const BBox3fa BVH4HairBuilder2::subdivideAndAdd(size_t threadIndex, BezierRefList& prims, const Bezier1& bezier, size_t depth)
  {
    if (depth == 0) {
      BezierRefList::item* block = prims.head();
      if (block == NULL || !block->insert(bezier)) {
        block = prims.insert(alloc.malloc(threadIndex));
        block->insert(bezier);
      }
      return bezier.bounds();
    }

    Bezier1 bezier0,bezier1;
    bezier.subdivide(bezier0,bezier1);
    const BBox3fa bounds0 = subdivideAndAdd(threadIndex,prims,bezier0,depth-1);
    const BBox3fa bounds1 = subdivideAndAdd(threadIndex,prims,bezier1,depth-1);
    return merge(bounds0,bounds1);
  }

  const NAABBox3fa BVH4HairBuilder2::computeHairSpaceBounds(BezierRefList& prims)
  {
    size_t N = BezierRefList::block_iterator_unsafe(prims).size();
    if (N == 0)
      return NAABBox3fa(empty); // FIXME: can cause problems with compression

    float bestArea = inf;
    LinearSpace3fa bestSpace = one;
    BBox3fa bestBounds = empty;

    size_t k=0;
    for (BezierRefList::block_iterator_unsafe i = prims; i; i++)
    {
      //if ((k++) % ((N+1)/2)) continue;
      if ((k++) % ((N+3)/4)) continue;
      //if ((k++) % ((N+15)/16)) continue;
      const Vec3fa axis = normalize(i->p3 - i->p0);
      if (length(i->p3 - i->p0) < 1E-9) continue;
      const LinearSpace3fa space = clamp(frame(axis).transposed());
      BBox3fa bounds = empty;
      float area = 0.0f;
      for (BezierRefList::block_iterator_unsafe j = prims; j; j++) {
        const BBox3fa cbounds = j->bounds(space);
	//area += halfArea(cbounds);
	area += (cbounds.upper.x-cbounds.lower.x)*(cbounds.upper.y-cbounds.lower.y);
        bounds.extend(cbounds);
      }

      if (area <= bestArea) {
        bestBounds = bounds;
        bestSpace = space;
        bestArea = area;
      }
    }

    /* select world space for some corner cases */
    if (bestArea == float(inf)) 
    {
      bestSpace = one;
      bestBounds = empty;
      for (BezierRefList::block_iterator_unsafe j = prims; j; j++)
        bestBounds.extend(j->bounds());
    }

    return NAABBox3fa(bestSpace,bestBounds);
  }

  BVH4Hair::NodeRef BVH4HairBuilder2::leaf(size_t threadIndex, size_t depth, BezierRefList& prims, const NAABBox3fa& bounds)
  {
    //size_t N = end-begin;
    size_t N = BezierRefList::block_iterator_unsafe(prims).size();

    if (N > (size_t)BVH4Hair::maxLeafBlocks) {
      //std::cout << "WARNING: Loosing " << N-BVH4Hair::maxLeafBlocks << " primitives during build!" << std::endl;
      std::cout << "!" << std::flush;
      N = (size_t)BVH4Hair::maxLeafBlocks;
    }
    size_t numGeneratedPrimsOld = atomic_add(&numGeneratedPrims,N); 
    if (numGeneratedPrimsOld%10000 > (numGeneratedPrimsOld+N)%10000) std::cout << "." << std::flush; 
    //assert(N <= (size_t)BVH4Hair::maxLeafBlocks);
    if (&bvh->primTy == &Bezier1Type::type) {
      Bezier1* leaf = (Bezier1*) bvh->allocPrimitiveBlocks(threadIndex,N);
      BezierRefList::block_iterator_unsafe iter(prims);
      for (size_t i=0; i<N; i++) { leaf[i] = *iter; iter++; }
      assert(!iter);

      /* free all primitive blocks */
      while (BezierRefList::item* block = prims.take())
        alloc.free(threadIndex,block);

      return bvh->encodeLeaf((char*)leaf,N);
    } 
    else if (&bvh->primTy == &SceneBezier1i::type) {
      Bezier1i* leaf = (Bezier1i*) bvh->allocPrimitiveBlocks(threadIndex,N);
      BezierRefList::block_iterator_unsafe iter(prims);
      for (size_t i=0; i<N; i++) {
        const Bezier1& curve = *iter; iter++;
        const BezierCurves* in = (BezierCurves*) scene->get(curve.geomID);
        const Vec3fa& p0 = in->vertex(in->curve(curve.primID));
        leaf[i] = Bezier1i(&p0,curve.geomID,curve.primID,-1); // FIXME: support mask
      }

      /* free all primitive blocks */
      while (BezierRefList::item* block = prims.take())
        alloc.free(threadIndex,block);

      return bvh->encodeLeaf((char*)leaf,N);
    }
    else 
      throw std::runtime_error("unknown primitive type");
  }

  Split BVH4HairBuilder2::find_split(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo, const NAABBox3fa& bounds)
  {
    /* variable to track the SAH of the best splitting approach */
    float bestSAH = inf;
    const float leafSAH = BVH4Hair::intCost*float(pinfo.size())*halfArea(bounds.bounds);
    
    /* perform standard binning in aligned space */
    ObjectPartition::Split alignedObjectSplit;
    float alignedObjectSAH = inf;
    if (enableAlignedObjectSplits) {
      alignedObjectSplit = ObjectPartition::find(threadIndex,threadCount,prims,one);
      alignedObjectSAH = BVH4Hair::travCostAligned*halfArea(bounds.bounds) + BVH4Hair::intCost*alignedObjectSplit.splitSAH();
      bestSAH = min(bestSAH,alignedObjectSAH);
    }

    /* perform spatial split in aligned space */
    SpatialSplit::Split alignedSpatialSplit;
    float alignedSpatialSAH = inf;
    bool enableSpatialSplits = remainingReplications > 0;
    if (enableSpatialSplits && enableAlignedSpatialSplits) {
      alignedSpatialSplit = SpatialSplit::find(threadIndex,threadCount,prims,pinfo);
      alignedSpatialSAH = BVH4Hair::travCostAligned*halfArea(bounds.bounds) + BVH4Hair::intCost*alignedSpatialSplit.splitSAH();
      bestSAH = min(bestSAH,alignedSpatialSAH);
    }

    /* perform standard binning in unaligned space */
    ObjectPartition::Split unalignedObjectSplit;
    float unalignedObjectSAH = inf;
    if (enableUnalignedObjectSplits && alignedObjectSAH > 0.7f*leafSAH) {
      const NAABBox3fa ubounds = computeHairSpaceBounds(prims);
      unalignedObjectSplit = ObjectPartition::find(threadIndex,threadCount,prims,ubounds.space);
      unalignedObjectSAH = BVH4Hair::travCostUnaligned*halfArea(bounds.bounds) + BVH4Hair::intCost*unalignedObjectSplit.splitSAH();
      bestSAH = min(bestSAH,unalignedObjectSAH);
    }

    /* perform splitting into two strands */
    StrandSplit::Split strandSplit;
    float strandSAH = inf;
    if (enableStrandSplits && alignedObjectSAH > 0.6f*leafSAH) {
      strandSplit = StrandSplit::find(threadIndex,threadCount,prims);
      strandSAH = BVH4Hair::travCostUnaligned*halfArea(bounds.bounds) + BVH4Hair::intCost*strandSplit.splitSAH();
      bestSAH = min(bestSAH,strandSAH);
    }

    /* perform fallback split */
    if (bestSAH == float(inf)) {
      return Split();
    }

    /* perform aligned object split */
    else if (bestSAH == alignedObjectSAH) {
      return Split(alignedObjectSplit,true);
    }

    /* perform aligned spatial split */
    else if (bestSAH == alignedSpatialSAH) {
      //atomic_add(&remainingReplications,pinfo.size()-linfo_o.size()-rinfo_o.size()); // FIXME:
      return Split(alignedSpatialSplit,true);
    }

    /* perform unaligned object split */
    else if (bestSAH == unalignedObjectSAH) {
      return Split(unalignedObjectSplit,false);
    }

    /* perform strand split */
    else if (bestSAH == strandSAH) {
      return Split(strandSplit,false);
    }
 
    else {
      throw std::runtime_error("bvh4hair_builder: internal error");
    }
  }

  BVH4Hair::NodeRef BVH4HairBuilder2::recurse(size_t threadIndex, size_t threadCount, size_t depth, BezierRefList& prims, const PrimInfo& pinfo, const NAABBox3fa& bounds, const Split& split)
  {
    /* create enforced leaf */
    const float leafSAH  = BVH4Hair::intCost*pinfo.leafSAH();
    const float splitSAH = BVH4Hair::travCostUnaligned*halfArea(bounds.bounds)+BVH4Hair::intCost*split.splitSAH();
   
    if (pinfo.size() <= minLeafSize || depth >= BVH4Hair::maxBuildDepth || (pinfo.size() <= maxLeafSize && leafSAH <= splitSAH)) {
      return leaf(threadIndex,depth,prims,bounds);
    }

    /*! initialize child list */
    bool isAligned = true;
    PrimInfo cpinfo     [BVH4Hair::N]; cpinfo [0] = pinfo; 
    NAABBox3fa cbounds  [BVH4Hair::N]; cbounds[0] = bounds;
    BezierRefList cprims[BVH4Hair::N]; cprims [0] = prims;
    Split csplit        [BVH4Hair::N]; csplit [0] = split;        
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
      PrimInfo linfo, rinfo;
      BezierRefList lprims, rprims;
      isAligned &= csplit[bestChild].isAligned;
      csplit[bestChild].split(threadIndex,threadCount,alloc,cprims[bestChild],lprims,linfo,rprims,rinfo);
      const NAABBox3fa lbounds = isAligned ? linfo.geomBounds : computeHairSpaceBounds(lprims); 
      const NAABBox3fa rbounds = isAligned ? rinfo.geomBounds : computeHairSpaceBounds(rprims); 
      const Split lsplit = find_split(threadIndex,threadCount,lprims,linfo,lbounds);
      const Split rsplit = find_split(threadIndex,threadCount,rprims,rinfo,rbounds);
      cprims[numChildren] = rprims; cpinfo[numChildren] = rinfo; cbounds[numChildren]= rbounds; csplit[numChildren] = rsplit;
      cprims[bestChild  ] = lprims; cpinfo[bestChild  ] = linfo; cbounds[bestChild  ]= lbounds; csplit[bestChild  ] = lsplit;
      numChildren++;
      
    } while (numChildren < BVH4Hair::N);

    /* create aligned node */
    if (isAligned) 
    {
      BVH4Hair::AlignedNode* node = bvh->allocAlignedNode(threadIndex);
      for (ssize_t i=0; i<numChildren; i++) {
        node->set(i,cpinfo[i].geomBounds);
        node->child(i) = recurse(threadIndex,threadCount,depth+1,cprims[i],cpinfo[i],cbounds[i],csplit[i]);
      }
      return bvh->encodeNode(node);
    }
    
    /* create unaligned node */
    else {
      BVH4Hair::UnalignedNode* node = bvh->allocUnalignedNode(threadIndex);
      for (ssize_t i=numChildren-1; i>=0; i--) {
        node->set(i,cbounds[i]);
        node->child(i) = recurse(threadIndex,threadCount,depth+1,cprims[i],cpinfo[i],cbounds[i],csplit[i]);
      }
      return bvh->encodeNode(node);
    }
  }
  
  __forceinline void BVH4HairBuilder2::processTask(size_t threadIndex, size_t threadCount, BuildTask& task, BuildTask task_o[BVH4Hair::N], size_t& numTasks_o)
  {
    /* create enforced leaf */
    const float leafSAH  = BVH4Hair::intCost*task.pinfo.leafSAH();
    const float splitSAH = BVH4Hair::travCostUnaligned*halfArea(task.bounds.bounds)+BVH4Hair::intCost*task.split.splitSAH();
   
    if (task.pinfo.size() <= minLeafSize || task.depth >= BVH4Hair::maxBuildDepth || (task.pinfo.size() <= maxLeafSize && leafSAH <= splitSAH)) {
      *task.dst = leaf(threadIndex,task.depth,task.prims,task.bounds);
      numTasks_o = 0;
      return;
    }

    /*! initialize child list */
    bool isAligned = true;
    PrimInfo cpinfo     [BVH4Hair::N]; cpinfo [0] = task.pinfo; 
    NAABBox3fa cbounds  [BVH4Hair::N]; cbounds[0] = task.bounds;
    BezierRefList cprims[BVH4Hair::N]; cprims [0] = task.prims;
    Split csplit        [BVH4Hair::N]; csplit [0] = task.split;        
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
      PrimInfo linfo, rinfo;
      BezierRefList lprims, rprims;
      isAligned &= csplit[bestChild].isAligned;
      csplit[bestChild].split(threadIndex,threadCount,alloc,cprims[bestChild],lprims,linfo,rprims,rinfo);
      const NAABBox3fa lbounds = isAligned ? linfo.geomBounds : computeHairSpaceBounds(lprims); 
      const NAABBox3fa rbounds = isAligned ? rinfo.geomBounds : computeHairSpaceBounds(rprims); 
      const Split lsplit = find_split(threadIndex,threadCount,lprims,linfo,lbounds);
      const Split rsplit = find_split(threadIndex,threadCount,rprims,rinfo,rbounds);
      cprims[numChildren] = rprims; cpinfo[numChildren] = rinfo; cbounds[numChildren]= rbounds; csplit[numChildren] = rsplit;
      cprims[bestChild  ] = lprims; cpinfo[bestChild  ] = linfo; cbounds[bestChild  ]= lbounds; csplit[bestChild  ] = lsplit;
      numChildren++;
      
    } while (numChildren < BVH4Hair::N);

    /* create aligned node */
    if (isAligned) 
    {
      BVH4Hair::AlignedNode* node = bvh->allocAlignedNode(threadIndex);
      for (ssize_t i=0; i<numChildren; i++) {
        node->set(i,cpinfo[i].geomBounds);
	new (&task_o[i]) BuildTask(&node->child(i),task.depth+1,cprims[i],cpinfo[i],cbounds[i],csplit[i]);
      }
      numTasks_o = numChildren;
      *task.dst = bvh->encodeNode(node);
    }
    
    /* create unaligned node */
    else {
      BVH4Hair::UnalignedNode* node = bvh->allocUnalignedNode(threadIndex);
      for (ssize_t i=numChildren-1; i>=0; i--) {
        node->set(i,cbounds[i]);
	new (&task_o[i]) BuildTask(&node->child(i),task.depth+1,cprims[i],cpinfo[i],cbounds[i],csplit[i]);
      }
      numTasks_o = numChildren;
      *task.dst = bvh->encodeNode(node);
    }
  }

  void BVH4HairBuilder2::recurseTask(size_t threadIndex, size_t threadCount,BuildTask& task)
  {
    size_t numChildren;
    BuildTask tasks[BVH4Hair::N];
    processTask(threadIndex,threadCount,task,tasks,numChildren);
    for (size_t i=0; i<numChildren; i++) 
      recurseTask(threadIndex,threadCount,tasks[i]);
  }

  void BVH4HairBuilder2::task_build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    while (numActiveTasks) 
    {
      taskMutex.lock();
      if (tasks.size() == 0) {
        taskMutex.unlock();
        continue;
      }

      /* take next task from heap */
      BuildTask task = tasks.front();
      pop_heap(tasks.begin(),tasks.end());
      tasks.pop_back();
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
        BuildTask ctasks[BVH4Hair::N];
        processTask(threadIndex,threadCount,task,ctasks,numChildren);
        taskMutex.lock();
        for (size_t i=0; i<numChildren; i++) {
          atomic_add(&numActiveTasks,+1);
          tasks.push_back(ctasks[i]);
          push_heap(tasks.begin(),tasks.end());
        }
        atomic_add(&numActiveTasks,-1);
        taskMutex.unlock();
      }
    }
  }

  Builder* BVH4HairBuilder2_ (BVH4Hair* accel, Scene* scene) {
    return new BVH4HairBuilder2(accel,scene);
  }
}
