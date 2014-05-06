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
    TriRefGen::TriRefGen(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<PrimRef>* alloc, const Scene* scene)
      : scene(scene), alloc(alloc)
    {
      /*! parallel stage */
      size_t numTasks = min(threadCount,maxTasks);
      TaskScheduler::executeTask(threadIndex,threadCount,_task_gen_parallel,this,numTasks,"build::trirefgen");
      
      /*! reduction stage */
      for (size_t i=0; i<numTasks; i++)
	pinfo.merge(pinfos[i]);
    }
    
    void TriRefGen::task_gen_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
    {
      ssize_t start = (taskIndex+0)*scene->numTriangles/taskCount;
      ssize_t end   = (taskIndex+1)*scene->numTriangles/taskCount;
      ssize_t cur   = 0;
      
      PrimInfo pinfo;
      TriRefList::item* block = prims.insert(alloc->malloc(threadIndex)); 
      for (size_t i=0; i<scene->size(); i++) 
      {
	TriangleMesh* geom = (TriangleMesh*) scene->get(i);
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
  }
}

