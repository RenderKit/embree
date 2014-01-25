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

#include "bvh4i/bvh4i.h"
#include "bvh4i/bvh4i_builder.h"

#include "bvh8i_builder.h"

#include "limits.h"
#include "sys/sync/barrier.h"

#include "bvh8i/bvh8i.h"

using namespace embree;

#define __ALIGN(x) __declspec(align(x))


#define CONVERT_TO_BVH8

namespace embree
{
  namespace isa
  {
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    BVH8iBuilderTriangle8::BVH8iBuilderTriangle8 (BVH4i* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize) 

    {
      bvh4i_builder;
    } 
    
    void BVH8iBuilderTriangle8::build(size_t threadIndex, size_t threadCount) 
    {
      // bvh->init();
      // allocateData();
      // TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel,this,TaskScheduler::getNumThreads(),"build_parallel");
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
#define DBG2(x) x
#define DBG3(x) 
    
    static void countLeavesButtomUpBVH4(BVHNode *__restrict__ const bvh4,
                                        const size_t index,
                                        size_t &index_tri4,
                                        Triangle4 *const accel4)
    {
      BVHNode &entry = bvh4[index];
      
      if (entry.isLeaf())
      {
#if 0
	unsigned int &min_d = (*(unsigned int*)&entry.lower.a);
	min_d ^= BVH_LEAF_MASK;
	const unsigned int items  = bvhItems(min_d);
	const unsigned int itemOffset = qbvhItemOffset(min_d);
	const Triangle1    * const accel = (Triangle1*)this->accel;
	const Triangle1  * tptr    = (Triangle1*)((char*)accel + itemOffset);
	Triangle4 *dest = &accel4[index_tri4];
	for (size_t j=0;j<4;j++)
        {
          //DBG_PRINT(tptr[j]);
          dest->v0.x[j] = tptr[j].a.x;
          dest->v0.y[j] = tptr[j].a.y;
          dest->v0.z[j] = tptr[j].a.z;
          
          dest->e1.x[j] = tptr[j].a.x - tptr[j].b.x;
          dest->e1.y[j] = tptr[j].a.y - tptr[j].b.y;
          dest->e1.z[j] = tptr[j].a.z - tptr[j].b.z;
          
          dest->e2.x[j] = tptr[j].c.x - tptr[j].a.x;
          dest->e2.y[j] = tptr[j].c.y - tptr[j].a.y;
          dest->e2.z[j] = tptr[j].c.z - tptr[j].a.z;
          
          dest->Ng.x[j] = tptr[j].normal.x;
          dest->Ng.y[j] = tptr[j].normal.y;
          dest->Ng.z[j] = tptr[j].normal.z;
          
          dest->geomID[j] = tptr[j].geomID;
          dest->primID[j] = tptr[j].primID;	      
        }
	// DBG_PRINT(dest->geomID);
	// DBG_PRINT(dest->primID);
        
	min_d = (unsigned int)(sizeof(Triangle4)*index_tri4) | BVH_LEAF_MASK | items;
	index_tri4++;
#endif
	entry.upper.a = 1;
      }
      else
      {
	const size_t childID = entry.firstChildID();
	const size_t children = entry.items();
        
	unsigned int leaves = 0;
	for (size_t i=0;i<children;i++) 
        {
          countLeavesButtomUpBVH4(bvh4,childID+i,index_tri4,accel4);
          leaves += bvh4[childID+i].upper.a;
        }      
	entry.upper.a = leaves;
      }
      
    }
    
    static void convertBVH4toBVH8(const BVHNode *const bvh4,
                                  const unsigned int bvh4_ext_min, 
                                  const unsigned int bvh4_ext_max, 
                                  BVH8i::BVH8iNode *const bvh8,
                                  size_t &index8,
                                  unsigned int &parent_offset,
                                  avxi &bvh8_node_dist)
    {
      size_t bvh8_used_slots = 0;
      const size_t bvh8_node_index = index8++;
      
      DBG3(DBG_PRINT(bvh8_node_index));
      
      bvh8[bvh8_node_index].reset();
      
      {
        const size_t childID = bvhChildID(bvh4_ext_min);
        const size_t children = bvhItems(bvh4_ext_min);
        DBG3(
          DBG_PRINT(childID);
          DBG_PRINT(children);
          );
        
        for (size_t i=0;i<children;i++) 
	{
	  DBG3(std::cout << "Putting " << bvh4[childID+i] << " in slot " << bvh8_used_slots << std::endl);
	  bvh8[bvh8_node_index].set(bvh8_used_slots++,bvh4[childID+i]);
	}
      }
      
      DBG3(DBG_PRINT(bvh8[bvh8_node_index]));
      
      while(bvh8_used_slots < 8)
      {
	DBG3(
          std::cout << std::endl << std::flush;
          DBG_PRINT(bvh8_used_slots);
          DBG_PRINT(bvh8[bvh8_node_index]);
          );
	avxf node_area = bvh8[bvh8_node_index].area();
	DBG3(DBG_PRINT(node_area));
        
        
	ssize_t max_index = -1;
	float max_area = 0.0f;
	const unsigned int free_slots = 8 - bvh8_used_slots;
        
	ssize_t max_index_small = -1;
	ssize_t min_children_small = 8;
	float max_area_small = 0.0f;
        
	for (size_t i=0;i<bvh8_used_slots;i++)
        {
          if (bvhLeaf(bvh8[bvh8_node_index].min_d[i])) continue;
          if ((bvhItems(bvh8[bvh8_node_index].min_d[i]) + bvh8_used_slots - 1) <= 8 && 
              node_area[i] > max_area)
          {      
            //if (bvh8_used_slots >=8)
            if (bvh8[bvh8_node_index].max_d[i] >= 8 && bvh8[bvh8_node_index].max_d[i] <= 8) continue;	      
            
            max_index = i;
            max_area = node_area[i];
            
            DBG3(
              DBG_PRINT(i);
              DBG_PRINT(max_index);
              DBG_PRINT(max_area);
              );
            
          }
	  
          if (bvh8[bvh8_node_index].max_d[i] <= free_slots && 
              bvh8[bvh8_node_index].max_d[i] < min_children_small)// &&
            //node_area[i] > max_area_small)
          {
            min_children_small = bvh8[bvh8_node_index].max_d[i];
            max_index_small = i;
            max_area_small = node_area[i];
            
            DBG3(
              DBG_PRINT(i);
              DBG_PRINT(max_index_small);
              DBG_PRINT(max_area_small);
              );
            
          }	  
          
        }
        
	if (max_index == -1) 
        {
          break;
        }
        
	if (max_index_small != -1)
	  max_index = max_index_small;
        
        
	DBG3(std::cout << "1" << std::endl << std::flush);
	const unsigned int parent_index = bvh8[bvh8_node_index].min_d[max_index];
	DBG3(DBG_PRINT(parent_index));
        
	
	bvh8[bvh8_node_index].shift(max_index);
	bvh8_used_slots--;
        
	assert(!bvhLeaf(parent_index));
	assert( bvhItems(parent_index) + bvh8_used_slots <= 8);
        
	DBG3(std::cout << "2" << std::endl << std::flush);
        
	const size_t childID = bvhChildID(parent_index);
	const size_t children = bvhItems(parent_index);
	for (size_t i=0;i<children;i++) 
        {
          DBG3(std::cout << "Putting node " << childID+i << " -> " << bvh4[childID+i] << " in slot " << bvh8_used_slots << std::endl);
          bvh8[bvh8_node_index].set(bvh8_used_slots++,bvh4[childID+i]);
          
        }
	if (bvh8_used_slots > 8) FATAL("HERE");
        
	DBG3(std::cout << "3" << std::endl << std::flush);
	assert(bvh8_used_slots <= 8);
      }
      
      
      DBG3(
        std::cout << "FINAL: " << std::endl << std::flush;
        DBG_PRINT(bvh8_used_slots);
        DBG_PRINT(bvh8[bvh8_node_index]);
        );
      
      DBG3(DBG_PRINT(bvh8[bvh8_node_index].min_d));
      
      
      DBG3(DBG_PRINT(bvh8_used_slots));
      
      parent_offset = (unsigned int)(sizeof(BVH8i::BVH8iNode) * bvh8_node_index);
      
      bvh8_node_dist[bvh8_used_slots-1]++;
      
      BVH8i::BVH8iNode &b8 = bvh8[bvh8_node_index];
      
      DBG3(DBG_PRINT(b8));
      for (size_t i=0;i<bvh8_used_slots;i++)
        if (!bvhLeaf(b8.min_d[i]))
	{
	  DBG3(std::cout << "RECURSE FOR " << b8.min_d[i] << " " << b8.max_d[i] << std::endl << std::flush);
	  convertBVH4toBVH8(bvh4,
			    b8.min_d[i],
			    b8.max_d[i],
			    bvh8,			      
			    index8,
			    (unsigned int&)b8.min_d[i],
			    bvh8_node_dist);
	}
        else
	{
	  b8.min_d[i] = (b8.min_d[i] ^ BVH_LEAF_MASK) | QBVH_LEAF_MASK;
	}
      
      DBG3(std::cout << "DONE" << std::endl << std::flush);
      
    }
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
//     void BVH8iBuilder::build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
//     {
//       if (threadIndex == 0)
//       {
// 	const size_t maxTri4Nodes = numPrimitives / 4.0f * 1.3f;
// 	accel4 = NULL; // (Triangle4*)os_malloc(sizeof(Triangle4)*maxTri4Nodes);
//       }
      
//       global_barrier.wait(threadIndex,threadCount);
//       //std::cout << "entering threadID " << threadIndex << " of " << threadCount << std::endl;
      
//       if (threadIndex == 0)
//       {
// 	// ==========================          
// 	// === preallocate arrays ===
// 	// ==========================
        
// 	double t0 = 0.0f;
// 	if (g_verbose >= 2) {
// 	  std::cout << "building BVH8i with SAH builder2 ... " << std::flush;
// 	  t0 = getSeconds();
// 	}
        
// 	// ==================================          
// 	// === calculate list of primrefs ===
// 	// ==================================
        
// 	//const Scene *__restrict__ const scene = (Scene*)geometry;
// 	//PrimRef *__restrict__ const aabb = this->prims;
        
        
// 	global_bounds.reset();
// 	LockStepTaskScheduler::dispatchTask( task_computePrimRefs, this, threadIndex, threadCount );
        
        
// 	//DBG_PRINT(global_bounds);
        
// 	// =======================          
// 	// === build QBVH tree ===
// 	// =======================
        
// 	atomicID.reset(4);
// 	//global_bounds.storeSceneAABB((float*)&this->node[0]);
//         this->node[0].lower = global_bounds.geometry.lower;
//         this->node[0].upper = global_bounds.geometry.upper;
        
// 	BuildRecord br;
// 	br.init(global_bounds,0,numPrimitives);
// 	br.parentID = 0;
// 	global_workStack.reset();
        
// 	global_workStack.push_nolock(br);
// 	while(global_workStack.size() < 2*threadCount && global_workStack.size()+1 <= SIZE_WORK_STACK)
//         {
//           bool success = global_workStack.pop(br);
//           assert(success);
//           recurseSAH(br,BUILD_TOP_LEVEL,threadIndex,threadCount);
//         }
// 	LockStepTaskScheduler::dispatchTask(task_buildSubTrees, this, threadIndex, threadCount );
// 	numNodes = atomicID >> 2;
        
// 	// =============================        
// 	// === create triangle accel ===
// 	// =============================
        
// 	LockStepTaskScheduler::dispatchTask( task_createTriangle1, this, threadIndex, threadCount );
	
// 	for (size_t i=0;i<4;i++)
// 	  this->accel[numPrimitives+i] = this->accel[numPrimitives-1];
        
// 	bvh->accel = this->accel;
// 	bvh->qbvh  = this->node;
        
// #ifdef DEBUG
// 	// checkBVH4iTree(this->node,
// 	// 	       this->accel,
// 	// 	       numPrimitives,
// 	// 	       true);
// #endif
        
// #ifdef CONVERT_TO_BVH8
// 	BVHNode  *bvh4 = (BVHNode*)this->node;
        
// 	size_t index_tri4 = 0;
// 	countLeavesButtomUpBVH4(bvh4,0,index_tri4,accel4);
        
// 	BVH8i::BVH8iNode *bvh8 = (BVH8i::BVH8iNode*)this->prims;
// 	bvh8[0].reset();
// 	size_t index8 = 1;
// 	avxi bvh8_node_dist = 0;
// 	convertBVH4toBVH8(bvh4,
// 			  bvh4[0].lower.a,
// 			  bvh4[0].upper.a,
// 			  bvh8,
// 			  index8,
// 			  (unsigned int&)bvh8[0].min_d[0],
// 			  bvh8_node_dist);
        
// 	DBG_PRINT(index8);
// 	DBG_PRINT(index8*sizeof(BVH8i::BVH8iNode));
// 	//DBG_PRINT(index_tri4);
        
// 	{
// 	  unsigned int total = 0;
// 	  float util = 0.0f;
// 	  for (size_t i=0;i<8;i++) {
// 	    util += (float)(i+1) * bvh8_node_dist[i];
// 	    total += bvh8_node_dist[i];
// 	  }
// 	  DBG_PRINT(total);
// 	  std::cout << "node util dist: ";
// 	  for (size_t i=0;i<8;i++) 
//           {
//             std::cout << i+1 << "[" << (float)bvh8_node_dist[i] * 100.0f / total << "] ";
//           }
// 	  std::cout << std::endl;
// 	  DBG_PRINT(100.0f * util / (8.0f * total));
// 	  std::cout << std::endl;
// 	  //exit(0);
// 	}
        
// 	//bvh->accel = (Triangle1*)accel4;
	
// #endif
        
        
	
// 	// ===================================        
// 	// === convert to optimized layout ===
// 	// ===================================
        
// 	LockStepTaskScheduler::dispatchTask( task_convertToSOALayout, this, threadIndex, threadCount );
        
        
// 	const QBVHNode      *const __restrict__ qbvh  = (QBVHNode*)bvh->qbvh;
// 	bvh->root = qbvh[0].min_d[0]; 
// 	bvh->bounds = BBox3f(Vec3fa(qbvh->min_x[0],qbvh->min_y[0],qbvh->min_y[0]),
// 			     Vec3fa(qbvh->max_x[0],qbvh->max_y[0],qbvh->max_y[0]));
        
// #ifdef CONVERT_TO_BVH8
// 	memcpy(bvh4,bvh8,sizeof(BVH8i::BVH8iNode)*index8);
// #endif
        
// 	//for (int i=0; i<5; i++) 
// 	//BVH4iRotate::rotate(bvh,bvh->root);
	
// 	// ==================          
// 	// === stop timer ===
// 	// ==================
        
// 	if (g_verbose >= 2) {
// 	  double t1 = getSeconds();
// 	  std::cout << "[DONE]" << std::endl;
// 	  std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(source->size())/(t1-t0) << " Mprim/s" << std::endl;
// 	}
        
// #ifndef CONVERT_TO_BVH8
// 	if (g_verbose >= 2) 
// 	  std::cout << BVH4iStatistics(bvh).str();
// #endif
// 	LockStepTaskScheduler::releaseThreads(threadCount);
//       }
//       else
//         LockStepTaskScheduler::dispatchTaskMainLoop(threadIndex,threadCount); 
//     };

     Builder* BVH8iTriangle8BuilderObjectSplit (void* bvh, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize) {
       return new BVH8iBuilderTriangle8((BVH4i*)bvh,source,scene,minLeafSize,maxLeafSize);
     }
  }
};
  

