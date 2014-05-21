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

#include "trirefgen.h"

namespace embree
{
  namespace isa
  {
    void TriRefListGen::generate(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const Scene* scene, TriRefList& prims, PrimInfo& pinfo) {
      TriRefListGen gen(threadIndex,threadCount,alloc,scene,prims,pinfo);
    }

    TriRefListGen::TriRefListGen(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const Scene* scene, TriRefList& prims, PrimInfo& pinfo)
      : scene(scene), alloc(alloc), prims(prims), pinfo(pinfo)
    {
      /*! parallel stage */
      size_t numTasks = min(threadCount,maxTasks);
      TaskScheduler::executeTask(threadIndex,threadCount,_task_gen_parallel,this,numTasks,"build::trirefgen");
      
      /*! reduction stage */
      for (size_t i=0; i<numTasks; i++)
	pinfo.merge(pinfos[i]);
    }
    
    void TriRefListGen::task_gen_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
    {
      ssize_t start = (taskIndex+0)*scene->numTriangles/taskCount;
      ssize_t end   = (taskIndex+1)*scene->numTriangles/taskCount;
      ssize_t cur   = 0;
      
      PrimInfo pinfo(empty);
      TriRefList::item* block = prims.insert(alloc->malloc(threadIndex)); 
      for (size_t i=0; i<scene->size(); i++) 
      {
	TriangleMesh* geom = (TriangleMesh*) scene->get(i);
        if (geom == NULL) continue;
	if (geom->type != TRIANGLE_MESH || geom->numTimeSteps != 1 || !geom->isEnabled()) continue;
	ssize_t gstart = 0;
	ssize_t gend = geom->numTriangles;
	ssize_t s = max(start-cur,gstart);
	ssize_t e = min(end  -cur,gend  );
	for (size_t j=s; j<e; j++) {
	  const PrimRef prim(geom->bounds(j),i,j);
	  pinfo.add(prim.bounds(),prim.center2());
	  if (likely(block->insert(prim))) continue; 
	  block = prims.insert(alloc->malloc(threadIndex));
	  block->insert(prim);
	}
	cur += geom->numTriangles;
	if (cur >= end) break;  
      }
      pinfos[taskIndex] = pinfo;
    }

    void TriRefArrayGen::generate_sequential(size_t threadIndex, size_t threadCount, const Scene* scene, PrimRef* prims_o, PrimInfo& pinfo_o) {
      TriRefArrayGen gen(threadIndex,threadCount,scene,prims_o,pinfo_o,false);
    }

    void TriRefArrayGen::generate_parallel(size_t threadIndex, size_t threadCount, const Scene* scene, PrimRef* prims_o, PrimInfo& pinfo_o) {
      TriRefArrayGen gen(threadIndex,threadCount,scene,prims_o,pinfo_o,true);
    }

    TriRefArrayGen::TriRefArrayGen(size_t threadIndex, size_t threadCount, const Scene* scene, PrimRef* prims_o, PrimInfo& pinfo_o, bool parallel)
      : scene(scene), prims_o(prims_o), pinfo_o(pinfo_o)
    {
      pinfo_o.reset();
      if (parallel) TaskScheduler::dispatchTask(_task_gen_parallel, this, threadIndex, threadCount);
      else          task_gen_parallel(threadIndex,threadCount,0,1,NULL);
      assert(pinfo_o.size() == scene->numTriangles);
    }

    void TriRefArrayGen::task_gen_parallel(size_t threadID, size_t numThreads, size_t taskIndex, size_t taskCount, TaskScheduler::Event* taskGroup)
    {
      ssize_t start = (taskIndex+0)*scene->numTriangles/taskCount;
      ssize_t end   = (taskIndex+1)*scene->numTriangles/taskCount;
      ssize_t cur   = 0;
      
      PrimInfo pinfo(empty);
      for (size_t i=0; i<scene->size(); i++) 
      {
	TriangleMesh* geom = (TriangleMesh*) scene->get(i);
        if (geom == NULL) continue;
	if (geom->type != TRIANGLE_MESH || geom->numTimeSteps != 1 || !geom->isEnabled()) continue;
	ssize_t gstart = 0;
	ssize_t gend = geom->numTriangles;
	ssize_t s = max(start-cur,gstart);
	ssize_t e = min(end  -cur,gend  );
	for (size_t j=s; j<e; j++) {
	  const PrimRef prim(geom->bounds(j),i,j);
	  pinfo.add(prim.bounds(),prim.center2());
	  prims_o[cur+j] = prim;
	}
	cur += geom->numTriangles;
	if (cur >= end) break;  
      }
      pinfo_o.atomic_extend(pinfo);
    }

    void TriRefArrayGenFromTriangleMesh::generate_sequential(size_t threadIndex, size_t threadCount, const TriangleMesh* mesh, PrimRef* prims_o, PrimInfo& pinfo_o) 
    {
      pinfo_o.reset();
      TriRefArrayGenFromTriangleMesh gen(threadIndex,threadCount,mesh,prims_o,pinfo_o);
      gen.task_gen_parallel(threadIndex,threadCount,0,1,NULL);
      assert(pinfo_o.size() == mesh->numTriangles);
    }

    void TriRefArrayGenFromTriangleMesh::generate_parallel(size_t threadIndex, size_t threadCount, const TriangleMesh* mesh, PrimRef* prims_o, PrimInfo& pinfo_o) 
    {
      pinfo_o.reset();
      TriRefArrayGenFromTriangleMesh gen(threadIndex,threadCount,mesh,prims_o,pinfo_o);
      TaskScheduler::dispatchTask(_task_gen_parallel, &gen, threadIndex, threadCount);
      assert(pinfo_o.size() == mesh->numTriangles);
    }

    TriRefArrayGenFromTriangleMesh::TriRefArrayGenFromTriangleMesh(size_t threadIndex, size_t threadCount, const TriangleMesh* mesh, PrimRef* prims_o, PrimInfo& pinfo_o)
      : mesh(mesh), prims_o(prims_o), pinfo_o(pinfo_o) {}

    void TriRefArrayGenFromTriangleMesh::task_gen_parallel(size_t threadID, size_t numThreads, size_t taskIndex, size_t taskCount, TaskScheduler::Event* taskGroup)
    {
      ssize_t start = (taskIndex+0)*mesh->numTriangles/taskCount;
      ssize_t end   = (taskIndex+1)*mesh->numTriangles/taskCount;
      ssize_t cur   = 0;
      
      PrimInfo pinfo(empty);
      for (size_t j=start; j<end; j++) {
	const PrimRef prim(mesh->bounds(j),mesh->id,j);
	pinfo.add(prim.bounds(),prim.center2());
	prims_o[j] = prim;
      }
      pinfo_o.atomic_extend(pinfo);
    }
  }
}

