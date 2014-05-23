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

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"

#define ROTATE_TREE 1

#include "common/scene_triangle_mesh.h"

namespace embree
{
  namespace isa
  {
    template<> BVH4Builder2T<Triangle1 >::BVH4Builder2T (BVH4* bvh, Scene* scene, size_t mode) : BVH4Builder2(bvh,scene,NULL,mode,0,0,1.0f,false,sizeof(Triangle1),2,inf) {}
    template<> BVH4Builder2T<Triangle4 >::BVH4Builder2T (BVH4* bvh, Scene* scene, size_t mode) : BVH4Builder2(bvh,scene,NULL,mode,2,2,1.0f,false,sizeof(Triangle4),4,inf) {}
#if defined(__AVX__)
    template<> BVH4Builder2T<Triangle8 >::BVH4Builder2T (BVH4* bvh, Scene* scene, size_t mode) : BVH4Builder2(bvh,scene,NULL,mode,3,2,1.0f,false,sizeof(Triangle8),8,inf) {}
#endif
    template<> BVH4Builder2T<Triangle1v>::BVH4Builder2T (BVH4* bvh, Scene* scene, size_t mode) : BVH4Builder2(bvh,scene,NULL,mode,0,0,1.0f,false,sizeof(Triangle1v),2,inf) {}
    template<> BVH4Builder2T<Triangle4v>::BVH4Builder2T (BVH4* bvh, Scene* scene, size_t mode) : BVH4Builder2(bvh,scene,NULL,mode,2,2,1.0f,false,sizeof(Triangle4v),4,inf) {}
    template<> BVH4Builder2T<Triangle4i>::BVH4Builder2T (BVH4* bvh, Scene* scene, size_t mode) : BVH4Builder2(bvh,scene,NULL,mode,2,2,1.0f,true ,sizeof(Triangle4i),4,inf) {}

    template<> BVH4Builder2T<Triangle1 >::BVH4Builder2T (BVH4* bvh, TriangleMesh* mesh, size_t mode) : BVH4Builder2(bvh,mesh->parent,mesh,mode,0,0,1.0f,false,sizeof(Triangle1),2,inf) {}
    template<> BVH4Builder2T<Triangle4 >::BVH4Builder2T (BVH4* bvh, TriangleMesh* mesh, size_t mode) : BVH4Builder2(bvh,mesh->parent,mesh,mode,2,2,1.0f,false,sizeof(Triangle4),4,inf) {}
#if defined(__AVX__)
    template<> BVH4Builder2T<Triangle8 >::BVH4Builder2T (BVH4* bvh, TriangleMesh* mesh, size_t mode) : BVH4Builder2(bvh,mesh->parent,mesh,mode,3,2,1.0f,false,sizeof(Triangle8),8,inf) {}
#endif
    template<> BVH4Builder2T<Triangle1v>::BVH4Builder2T (BVH4* bvh, TriangleMesh* mesh, size_t mode) : BVH4Builder2(bvh,mesh->parent,mesh,mode,0,0,1.0f,false,sizeof(Triangle1v),2,inf) {}
    template<> BVH4Builder2T<Triangle4v>::BVH4Builder2T (BVH4* bvh, TriangleMesh* mesh, size_t mode) : BVH4Builder2(bvh,mesh->parent,mesh,mode,2,2,1.0f,false,sizeof(Triangle4v),4,inf) {}
    template<> BVH4Builder2T<Triangle4i>::BVH4Builder2T (BVH4* bvh, TriangleMesh* mesh, size_t mode) : BVH4Builder2(bvh,mesh->parent,mesh,mode,2,2,1.0f,true ,sizeof(Triangle4i),4,inf) {}

    BVH4Builder2::BVH4Builder2 (BVH4* bvh, Scene* scene, TriangleMesh* mesh, size_t mode,
				size_t logBlockSize, size_t logSAHBlockSize, float intCost, 
				bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize)
      : scene(scene), mesh(mesh), bvh(bvh), enableSpatialSplits(mode > 0),
	logBlockSize(logBlockSize), logSAHBlockSize(logSAHBlockSize), intCost(intCost), 
	needVertices(needVertices), primBytes(primBytes), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize)
     {
       size_t maxLeafPrims = BVH4::maxLeafBlocks*(1<<logBlockSize);
      if (maxLeafPrims < this->maxLeafSize) 
	this->maxLeafSize = maxLeafPrims;
    }

    template<typename Triangle>
    typename BVH4Builder2::NodeRef BVH4Builder2T<Triangle>::createLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo)
    {
      /* allocate leaf node */
      size_t N = blocks(pinfo.size());
      Triangle* leaf = (Triangle*) bvh->allocPrimitiveBlocks(threadIndex,N);
      assert(N <= (size_t)BVH4::maxLeafBlocks);
      
      /* insert all triangles */
      TriRefList::block_iterator_unsafe iter(prims);
      for (size_t i=0; i<N; i++) leaf[i].fill(iter,scene);
      assert(!iter);
      
      /* free all primitive blocks */
      while (TriRefList::item* block = prims.take())
	alloc.free(threadIndex,block);
      
      return bvh->encodeLeaf(leaf,N);
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

    template<bool PARALLEL>
    const Split BVH4Builder2::find(size_t threadIndex, size_t threadCount, size_t depth, TriRefList& prims, const PrimInfo& pinfo, bool spatial)
    {
      ObjectPartition::Split osplit = ObjectPartition::find<PARALLEL>(threadIndex,threadCount,      prims,pinfo,logSAHBlockSize);
      if (!enableSpatialSplits || !spatial) {
	if (osplit.sah == float(inf)) return Split();
	else return osplit;
      }
      SpatialSplit   ::Split ssplit = SpatialSplit   ::find<PARALLEL>(threadIndex,threadCount,scene,prims,pinfo,logSAHBlockSize);
      const float bestSAH = min(osplit.sah,ssplit.sah);
      if      (bestSAH == osplit.sah) return osplit; 
      else if (bestSAH == ssplit.sah) return ssplit;
      else                            return Split();
    }
    
    template<bool PARALLEL>
    __forceinline size_t BVH4Builder2::createNode(size_t threadIndex, size_t threadCount, BVH4Builder2* parent, BuildRecord& record, BuildRecord records_o[BVH4::N])
    {
      /*! compute leaf and split cost */
      const float leafSAH  = parent->intCost*record.pinfo.leafSAH(parent->logSAHBlockSize);
      const float splitSAH = BVH4::travCost*halfArea(record.pinfo.geomBounds)+parent->intCost*record.split.splitSAH();
      assert(TriRefList::block_iterator_unsafe(prims).size() == record.pinfo.size());
      assert(record.pinfo.size() == 0 || leafSAH >= 0 && splitSAH >= 0);
      
      /*! create a leaf node when threshold reached or SAH tells us to stop */
      if (record.pinfo.size() <= parent->minLeafSize || record.depth > BVH4::maxBuildDepth || (record.pinfo.size() <= parent->maxLeafSize && leafSAH <= splitSAH)) {
	*record.dst = parent->createLargeLeaf(threadIndex,record.prims,record.pinfo,record.depth+1); return 0;
      }
      
      /*! initialize child list */
      records_o[0] = record;
      size_t numChildren = 1;
      
      /*! split until node is full or SAH tells us to stop */
      do {
	
	/*! find best child to split */
	float bestSAH = 0; 
	ssize_t bestChild = -1;
	for (size_t i=0; i<numChildren; i++) 
	{
	  float dSAH = records_o[i].split.splitSAH()-records_o[i].pinfo.leafSAH(parent->logSAHBlockSize);
	  if (records_o[i].pinfo.size() <= parent->minLeafSize) continue; 
	  if (records_o[i].pinfo.size() > parent->maxLeafSize) dSAH = min(0.0f,dSAH); //< force split for large jobs
	  if (dSAH <= bestSAH) { bestChild = i; bestSAH = dSAH; }
	}
	if (bestChild == -1) break;
	
	/* perform best found split */
	BuildRecord lrecord(record.depth+1);
	BuildRecord rrecord(record.depth+1);
	records_o[bestChild].split.split<PARALLEL>(threadIndex,threadCount,parent->alloc,parent->scene,records_o[bestChild].prims,lrecord.prims,lrecord.pinfo,rrecord.prims,rrecord.pinfo);

	/* fallback if spatial split did fail for corner case */
	if (lrecord.pinfo.size() == 0) {
	  rrecord.split = parent->find<PARALLEL>(threadIndex,threadCount,record.depth,rrecord.prims,rrecord.pinfo,false);
	  records_o[bestChild] = rrecord;
	  continue;
	}

	/* fallback if spatial split did fail for corner case */
	if (rrecord.pinfo.size() == 0) {
	  lrecord.split = parent->find<PARALLEL>(threadIndex,threadCount,record.depth,lrecord.prims,lrecord.pinfo,false);
	  records_o[bestChild] = lrecord;
	  continue;
	}
	
	/* count number of replications caused by spatial splits */
	ssize_t remaining = 0;
	const ssize_t replications = lrecord.pinfo.size()+rrecord.pinfo.size()-records_o[bestChild].pinfo.size(); 
	assert(replications >= 0);
	if (replications >= 0) 
	  remaining = atomic_add(&parent->remainingReplications,-replications);

	/* find new splits */
	lrecord.split = parent->find<PARALLEL>(threadIndex,threadCount,record.depth,lrecord.prims,lrecord.pinfo,remaining > 0);
	rrecord.split = parent->find<PARALLEL>(threadIndex,threadCount,record.depth,rrecord.prims,rrecord.pinfo,remaining > 0);
	records_o[bestChild  ] = lrecord;
	records_o[numChildren] = rrecord;
	numChildren++;
	
      } while (numChildren < BVH4::N);
      
      /*! create an inner node */
      Node* node = parent->bvh->allocNode(threadIndex);
      for (size_t i=0; i<numChildren; i++) {
	node->set(i,records_o[i].pinfo.geomBounds);
	records_o[i].dst = &node->child(i);
      }
      *record.dst = parent->bvh->encodeNode(node);
      return numChildren;
    }

    void BVH4Builder2::continue_build(size_t threadIndex, size_t threadCount, BuildRecord& record)
    {
      /* finish small tasks */
      if (record.pinfo.size() < 4*1024) 
      {
	finish_build(threadIndex,threadCount,record);
#if ROTATE_TREE
	for (int i=0; i<5; i++) 
	  BVH4Rotate::rotate(bvh,*record.dst); 
#endif
	record.dst->setBarrier();
      }

      /* and split large tasks */
      else
      {
	BuildRecord children[BVH4::N];
	size_t N = createNode<false>(threadIndex,threadCount,this,record,children);
      	taskMutex.lock();
	for (size_t i=0; i<N; i++) {
	  tasks.push_back(children[i]);
	  atomic_add(&activeBuildRecords,1);
	}
	taskMutex.unlock();
      }
    }

    void BVH4Builder2::finish_build(size_t threadIndex, size_t threadCount, BuildRecord& record)
    {
      BuildRecord children[BVH4::N];
      size_t N = createNode<false>(threadIndex,threadCount,this,record,children);
      for (size_t i=0; i<N; i++)
	finish_build(threadIndex,threadCount,children[i]);
    }
    
    void BVH4Builder2::build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
    {
      while (activeBuildRecords)
      {
	taskMutex.lock();
	if (tasks.size() == 0) {
	  taskMutex.unlock();
	  continue;
	}
	BuildRecord record = tasks.back();
	tasks.pop_back();
	taskMutex.unlock();
	continue_build(threadIndex,threadCount,record);
	atomic_add(&activeBuildRecords,-1);
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
	for (size_t i=0; i<BVH4::N; i++) {
	  dst->set(i,src->bounds(i),layout_top_nodes(threadIndex,src->child(i)));
	}
	return bvh->encodeNode(dst);
      }
      else
	return node;
    }
    
    void BVH4Builder2::build(size_t threadIndex, size_t threadCount) 
    {
      size_t numPrimitives = scene->numTriangles;
      bvh->init(2*numPrimitives); // FIXME: 2x
      remainingReplications = numPrimitives;
      if (numPrimitives == 0) 
	return;
      
      if (g_verbose >= 2) 
	std::cout << "building BVH4<" << bvh->primTy.name << "> with SAH builder2 ... " << std::flush;
      
      double t0 = 0.0, t1 = 0.0f;
      if (g_verbose >= 2 || g_benchmark)
	t0 = getSeconds();
      
      /* generate list of build primitives */
      if (mesh) throw std::runtime_error("BVH4Builder2: generation from triangle mesh not yet implemented"); // FIXME
      TriRefList prims; PrimInfo pinfo(empty);
      TriRefListGen::generate(threadIndex,threadCount,&alloc,scene,prims,pinfo);
      
      const Split split = find<true>(threadIndex,threadCount,1,prims,pinfo,true);
      BuildRecord record(1,prims,pinfo,split,&bvh->root);
      tasks.push_back(record);
      activeBuildRecords=1;

      /* work in multithreaded toplevel mode until sufficient subtasks got generated */
      while (tasks.size() < threadCount)
      {
	/* pop largest item for better load balancing */
	BuildRecord task = tasks.front();
	pop_heap(tasks.begin(),tasks.end());
	tasks.pop_back();
	activeBuildRecords--;
	
	/* process this item in parallel */
	BuildRecord children[BVH4::N];
	size_t N = createNode<true>(threadIndex,threadCount,this,task,children);
	for (size_t i=0; i<N; i++) {
	  tasks.push_back(children[i]);
	  push_heap(tasks.begin(),tasks.end());
	  activeBuildRecords++;
	}
      }
      
      /*! process each generated subtask in its own thread */
      TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel,this,threadCount,"BVH4Builder2::build");
                  
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
	std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(numPrimitives)/(t1-t0) << " Mprim/s" << std::endl;
	std::cout << BVH4Statistics(bvh).str();
      }
    }
    
    Builder* BVH4Triangle1Builder2  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Builder2T<Triangle1> ((BVH4*)bvh,scene,mode); }
    Builder* BVH4Triangle4Builder2  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Builder2T<Triangle4> ((BVH4*)bvh,scene,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8Builder2  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Builder2T<Triangle8> ((BVH4*)bvh,scene,mode); }
#endif
    Builder* BVH4Triangle1vBuilder2 (void* bvh, Scene* scene, size_t mode) { return new class BVH4Builder2T<Triangle1v>((BVH4*)bvh,scene,mode); }
    Builder* BVH4Triangle4vBuilder2 (void* bvh, Scene* scene, size_t mode) { return new class BVH4Builder2T<Triangle4v>((BVH4*)bvh,scene,mode); }
    Builder* BVH4Triangle4iBuilder2 (void* bvh, Scene* scene, size_t mode) { return new class BVH4Builder2T<Triangle4i>((BVH4*)bvh,scene,mode); }

    Builder* BVH4Triangle1MeshBuilder2  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Builder2T<Triangle1> ((BVH4*)bvh,mesh,mode); }
    Builder* BVH4Triangle4MeshBuilder2  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Builder2T<Triangle4> ((BVH4*)bvh,mesh,mode); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilder2  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Builder2T<Triangle8> ((BVH4*)bvh,mesh,mode); }
#endif
    Builder* BVH4Triangle1vMeshBuilder2 (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Builder2T<Triangle1v>((BVH4*)bvh,mesh,mode); }
    Builder* BVH4Triangle4vMeshBuilder2 (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Builder2T<Triangle4v>((BVH4*)bvh,mesh,mode); }
    Builder* BVH4Triangle4iMeshBuilder2 (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4Builder2T<Triangle4i>((BVH4*)bvh,mesh,mode); }
  }
}
