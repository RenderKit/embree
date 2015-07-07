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

#include "scene.h"

#if !defined(__MIC__)
#include "../xeon/bvh4/bvh4.h"
#include "../xeon/bvh8/bvh8.h"
#else
#include "../xeonphi/bvh4i/bvh4i.h"
#include "../xeonphi/bvh4mb/bvh4mb.h"
#include "../xeonphi/bvh4hair/bvh4hair.h"
#endif

#define USE_TASK_ARENA 1

namespace embree
{
#if TASKING_TBB
  tbb::task_arena arena;
#endif

  /* error raising rtcIntersect and rtcOccluded functions */
  void missing_rtcCommit()     { throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed"); }
  void invalid_rtcIntersect1() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect and rtcOccluded not enabled"); }
  void invalid_rtcIntersect4() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect4 and rtcOccluded4 not enabled"); }
  void invalid_rtcIntersect8() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect8 and rtcOccluded8 not enabled"); }
  void invalid_rtcIntersect16() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect16 and rtcOccluded16 not enabled"); }

  Scene::Scene (RTCSceneFlags sflags, RTCAlgorithmFlags aflags)
    : Accel(AccelData::TY_UNKNOWN),
      flags(sflags), aflags(aflags), numMappedBuffers(0), is_build(false), modified(true), 
      needTriangleIndices(false), needTriangleVertices(false), 
      needBezierIndices(false), needBezierVertices(false),
      needSubdivIndices(false), needSubdivVertices(false),
      numTriangles(0), numTriangles2(0), 
      numBezierCurves(0), numBezierCurves2(0), 
      numSubdivPatches(0), numSubdivPatches2(0), 
      numUserGeometries1(0), 
      numIntersectionFilters4(0), numIntersectionFilters8(0), numIntersectionFilters16(0),
      commitCounter(0), 
      progress_monitor_function(nullptr), progress_monitor_ptr(nullptr), progress_monitor_counter(0)
  {
#if defined(TASKING_LOCKSTEP) 
    lockstep_scheduler.taskBarrier.init(MAX_MIC_THREADS);
#elif defined(TASKING_TBB_INTERNAL)
    scheduler = nullptr;
#else
    //arena = new tbb::task_arena;
    group = new tbb::task_group;
#endif

    intersectors = Accel::Intersectors(missing_rtcCommit);

    if (State::instance()->scene_flags != -1)
      flags = (RTCSceneFlags) State::instance()->scene_flags;

    if (aflags & RTC_INTERPOLATE) {
      needTriangleIndices = true;
      needBezierIndices = true;
      //needSubdivIndices = true; // not required for interpolation
      needTriangleVertices = true;
      needBezierVertices = true;
      needSubdivVertices = true;
    }

#if defined(__MIC__)
    needBezierVertices = true;

    accels.add( BVH4mb::BVH4mbTriangle1ObjectSplitBinnedSAH(this) );
    accels.add( BVH4i::BVH4iVirtualGeometryBinnedSAH(this, isRobust()));
    accels.add( BVH4Hair::BVH4HairBinnedSAH(this));
    accels.add( BVH4i::BVH4iSubdivMeshBinnedSAH(this, isRobust() ));

    if (State::instance()->verbosity(1))
    {
      std::cout << "scene flags: static " << isStatic() << " compact = " << isCompact() << " high quality = " << isHighQuality() << " robust = " << isRobust() << std::endl;
    }
    
    if (State::instance()->tri_accel == "default" || State::instance()->tri_accel == "bvh4i")   
    {
      if (State::instance()->tri_builder == "default") 
      {
        if (isStatic())
        {
          if (State::instance()->verbosity(1)) std::cout << "STATIC BUILDER MODE" << std::endl;
          if ( isCompact() )
            accels.add(BVH4i::BVH4iTriangle1MemoryConservativeBinnedSAH(this,isRobust()));		    
          else if ( isHighQuality() )
            accels.add(BVH4i::BVH4iTriangle1ObjectSplitBinnedSAH(this,isRobust()));
          else
            accels.add(BVH4i::BVH4iTriangle1ObjectSplitBinnedSAH(this,isRobust()));
        }
        else
        {
          if (State::instance()->verbosity(1)) std::cout << "DYNAMIC BUILDER MODE" << std::endl;
          accels.add(BVH4i::BVH4iTriangle1ObjectSplitMorton(this,isRobust()));
        }
      }
      else
      {
        if (State::instance()->tri_builder == "sah" || State::instance()->tri_builder == "bvh4i" || State::instance()->tri_builder == "bvh4i.sah") {
          accels.add(BVH4i::BVH4iTriangle1ObjectSplitBinnedSAH(this,isRobust()));
        }
        else if (State::instance()->tri_builder == "fast" || State::instance()->tri_builder == "morton") {
          accels.add(BVH4i::BVH4iTriangle1ObjectSplitMorton(this,isRobust()));
        }
        else if (State::instance()->tri_builder == "fast_enhanced" || State::instance()->tri_builder == "morton.enhanced") {
          accels.add(BVH4i::BVH4iTriangle1ObjectSplitEnhancedMorton(this,isRobust()));
        }
        else if (State::instance()->tri_builder == "high_quality" || State::instance()->tri_builder == "presplits") {
          accels.add(BVH4i::BVH4iTriangle1PreSplitsBinnedSAH(this,isRobust()));
        }
        else if (State::instance()->tri_builder == "compact" ||
                 State::instance()->tri_builder == "memory_conservative") {
          accels.add(BVH4i::BVH4iTriangle1MemoryConservativeBinnedSAH(this,isRobust()));
        }
        else if (State::instance()->tri_builder == "morton64") {
          accels.add(BVH4i::BVH4iTriangle1ObjectSplitMorton64Bit(this,isRobust()));
        }
        
        else THROW_RUNTIME_ERROR("unknown builder "+State::instance()->tri_builder+" for BVH4i<Triangle1>");
      }
    }
    else THROW_RUNTIME_ERROR("unknown accel "+State::instance()->tri_accel);
    
#else
    createTriangleAccel();
    accels.add(BVH4::BVH4Triangle4vMB(this));
    accels.add(BVH4::BVH4UserGeometry(this));
    createHairAccel();
    accels.add(BVH4::BVH4OBBBezier1iMB(this,false));
    createSubdivAccel();
#endif
  }

#if !defined(__MIC__)

  void Scene::createTriangleAccel()
  {
    if (State::instance()->tri_accel == "default") 
    {
      if (isStatic()) {
        int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
        switch (mode) {
        case /*0b00*/ 0: 
#if defined (__TARGET_AVX__)
          if (hasISA(AVX))
	  {
            if (isHighQuality()) {
#if defined (__TARGET_AVX512__) && 0
              accels.add(BVH8::BVH8Triangle8SpatialSplit(this)); 
#else
              accels.add(BVH8::BVH8Triangle4SpatialSplit(this)); 
#endif
            }
            else {
#if defined (__TARGET_AVX512__) && 0
              accels.add(BVH8::BVH8Triangle8ObjectSplit(this)); 
#else
              accels.add(BVH8::BVH8Triangle4ObjectSplit(this)); 
#endif
            }
          }
          else 
#endif
          {
            if (isHighQuality()) accels.add(BVH4::BVH4Triangle4SpatialSplit(this));
            else                 accels.add(BVH4::BVH4Triangle4ObjectSplit(this)); 
          }
          break;

          case /*0b01*/ 1: 
#if defined (__TARGET_AVX2__) && !defined(__WIN32__) // FIXME: have to disable under Windows as watertightness tests fail
            if (hasISA(AVX2))
              accels.add(BVH8::BVH8Triangle8vObjectSplit(this)); 
            else
#endif
              accels.add(BVH4::BVH4Triangle4vObjectSplit(this));

            break;
        case /*0b10*/ 2: accels.add(BVH4::BVH4Triangle4iObjectSplit(this)); break;
        case /*0b11*/ 3: accels.add(BVH4::BVH4Triangle4iObjectSplit(this)); break;
        }
      } 
      else 
      {
        int mode =  2*(int)isCompact() + 1*(int)isRobust();
        switch (mode) {
        case /*0b00*/ 0: accels.add(BVH4::BVH4BVH4Triangle4ObjectSplit(this)); break;
        case /*0b01*/ 1: accels.add(BVH4::BVH4BVH4Triangle4vObjectSplit(this)); break;
        case /*0b10*/ 2: accels.add(BVH4::BVH4BVH4Triangle4iObjectSplit(this)); break;
        case /*0b11*/ 3: accels.add(BVH4::BVH4BVH4Triangle4iObjectSplit(this)); break;
        }
      }
    }
    else if (State::instance()->tri_accel == "bvh4.bvh4.triangle4")    accels.add(BVH4::BVH4BVH4Triangle4ObjectSplit(this));
    else if (State::instance()->tri_accel == "bvh4.bvh4.triangle4v")   accels.add(BVH4::BVH4BVH4Triangle4vObjectSplit(this));
    else if (State::instance()->tri_accel == "bvh4.bvh4.triangle4i")   accels.add(BVH4::BVH4BVH4Triangle4iObjectSplit(this));
    else if (State::instance()->tri_accel == "bvh4.triangle4")         accels.add(BVH4::BVH4Triangle4(this));
    else if (State::instance()->tri_accel == "bvh4.triangle4v")        accels.add(BVH4::BVH4Triangle4v(this));
    else if (State::instance()->tri_accel == "bvh4.triangle4i")        accels.add(BVH4::BVH4Triangle4i(this));
#if defined (__TARGET_AVX__)
    else if (State::instance()->tri_accel == "bvh4.bvh4.triangle8")    accels.add(BVH4::BVH4BVH4Triangle8ObjectSplit(this));
    else if (State::instance()->tri_accel == "bvh4.triangle8")         accels.add(BVH4::BVH4Triangle8(this));
    else if (State::instance()->tri_accel == "bvh8.triangle4")         accels.add(BVH8::BVH8Triangle4(this));
    else if (State::instance()->tri_accel == "bvh8.triangle8")         accels.add(BVH8::BVH8Triangle8(this));
    else if (State::instance()->tri_accel == "bvh8.trianglepairs8")    accels.add(BVH8::BVH8TrianglePairs8(this));
    else if (State::instance()->tri_accel == "bvh8.triangle8v")    accels.add(BVH8::BVH8Triangle8v(this));

#endif
    else THROW_RUNTIME_ERROR("unknown triangle acceleration structure "+State::instance()->tri_accel);
  }

  void Scene::createHairAccel()
  {
    if (State::instance()->hair_accel == "default") 
    {
      if (isStatic()) {
        int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
        switch (mode) {
        case /*0b00*/ 0: accels.add(BVH4::BVH4OBBBezier1v(this,isHighQuality())); break;
        case /*0b01*/ 1: accels.add(BVH4::BVH4OBBBezier1v(this,isHighQuality())); break;
        case /*0b10*/ 2: accels.add(BVH4::BVH4OBBBezier1i(this,isHighQuality())); break;
        case /*0b11*/ 3: accels.add(BVH4::BVH4OBBBezier1i(this,isHighQuality())); break;
        }
      } 
      else 
      {
        int mode =  2*(int)isCompact() + 1*(int)isRobust();
        switch (mode) {
	case /*0b00*/ 0: accels.add(BVH4::BVH4Bezier1v(this)); break;
        case /*0b01*/ 1: accels.add(BVH4::BVH4Bezier1v(this)); break;
        case /*0b10*/ 2: accels.add(BVH4::BVH4Bezier1i(this)); break;
        case /*0b11*/ 3: accels.add(BVH4::BVH4Bezier1i(this)); break;
        }
      }   
    }
    else if (State::instance()->hair_accel == "bvh4.bezier1v"    ) accels.add(BVH4::BVH4Bezier1v(this));
    else if (State::instance()->hair_accel == "bvh4.bezier1i"    ) accels.add(BVH4::BVH4Bezier1i(this));
    else if (State::instance()->hair_accel == "bvh4obb.bezier1v" ) accels.add(BVH4::BVH4OBBBezier1v(this,false));
    else if (State::instance()->hair_accel == "bvh4obb.bezier1i" ) accels.add(BVH4::BVH4OBBBezier1i(this,false));
    else THROW_RUNTIME_ERROR("unknown hair acceleration structure "+State::instance()->hair_accel);
  }

  void Scene::createSubdivAccel()
  {
    if (State::instance()->subdiv_accel == "default") 
    {
      if (isIncoherent(flags) && isStatic())
        accels.add(BVH4::BVH4SubdivGridEager(this));
      else
        accels.add(BVH4::BVH4SubdivPatch1Cached(this));
    }
    else if (State::instance()->subdiv_accel == "bvh4.subdivpatch1"      ) accels.add(BVH4::BVH4SubdivPatch1(this));
    else if (State::instance()->subdiv_accel == "bvh4.subdivpatch1cached") accels.add(BVH4::BVH4SubdivPatch1Cached(this));
    else if (State::instance()->subdiv_accel == "bvh4.grid.adaptive"     ) accels.add(BVH4::BVH4SubdivGrid(this));
    else if (State::instance()->subdiv_accel == "bvh4.grid.eager"        ) accels.add(BVH4::BVH4SubdivGridEager(this));
    else if (State::instance()->subdiv_accel == "bvh4.grid.lazy"         ) accels.add(BVH4::BVH4SubdivGridLazy(this));
    else THROW_RUNTIME_ERROR("unknown subdiv accel "+State::instance()->subdiv_accel);
  }

#endif

  Scene::~Scene () 
  {
    for (size_t i=0; i<geometries.size(); i++)
      delete geometries[i];

#if TASKING_TBB
    //delete arena; arena = nullptr;
    delete group; group = nullptr;
#endif
  }

  void Scene::clear() {
  }

  unsigned Scene::newUserGeometry (size_t items) 
  {
    Geometry* geom = new UserGeometry(this,items);
    return geom->id;
  }
  
  unsigned Scene::newInstance (Scene* scene) {
    Geometry* geom = new Instance(this,scene);
    return geom->id;
  }

  unsigned Scene::newTriangleMesh (RTCGeometryFlags gflags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) 
  {
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > 2) {
      throw_RTCError(RTC_INVALID_OPERATION,"only 1 or 2 time steps supported");
      return -1;
    }
    
    Geometry* geom = new TriangleMesh(this,gflags,numTriangles,numVertices,numTimeSteps);
    return geom->id;
  }

  unsigned Scene::newSubdivisionMesh (RTCGeometryFlags gflags, size_t numFaces, size_t numEdges, size_t numVertices, size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps) 
  {
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > 2) {
      throw_RTCError(RTC_INVALID_OPERATION,"only 1 or 2 time steps supported");
      return -1;
    }

    Geometry* geom = nullptr;
#if defined(__TARGET_AVX__)
    if (hasISA(AVX))
      geom = new SubdivMeshAVX(this,gflags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps);
    else 
#endif
      geom = new SubdivMesh(this,gflags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps);
    return geom->id;
  }


  unsigned Scene::newBezierCurves (RTCGeometryFlags gflags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > 2) {
      throw_RTCError(RTC_INVALID_OPERATION,"only 1 or 2 time steps supported");
      return -1;
    }
    
    Geometry* geom = new BezierCurves(this,gflags,numCurves,numVertices,numTimeSteps);
    return geom->id;
  }

  unsigned Scene::add(Geometry* geometry) 
  {
    Lock<AtomicMutex> lock(geometriesMutex);

    if (usedIDs.size()) {
      int id = usedIDs.back(); 
      usedIDs.pop_back();
      geometries[id] = geometry;
      return id;
    } else {
      geometries.push_back(geometry);
      return geometries.size()-1;
    }
  }
  
  void Scene::remove(Geometry* geometry) 
  {
    Lock<AtomicMutex> lock(geometriesMutex);
    usedIDs.push_back(geometry->id);
    geometries[geometry->id] = nullptr;
    delete geometry;
  }

  void Scene::updateInterface()
  {
    /* update bounds */
    is_build = true;
    bounds = accels.bounds;
    intersectors = accels.intersectors;

    /* enable only algorithms choosen by application */
    if ((aflags & RTC_INTERSECT1) == 0) intersectors.intersector1 = Accel::Intersector1(&invalid_rtcIntersect1);
    if ((aflags & RTC_INTERSECT4) == 0) intersectors.intersector4 = Accel::Intersector4(&invalid_rtcIntersect4);
    if ((aflags & RTC_INTERSECT8) == 0) intersectors.intersector8 = Accel::Intersector8(&invalid_rtcIntersect8);
    if ((aflags & RTC_INTERSECT16) == 0) intersectors.intersector16 = Accel::Intersector16(&invalid_rtcIntersect16);

    /* update commit counter */
    commitCounter++;
  }

  void Scene::build_task ()
  {
    progress_monitor_counter = 0;

    /* select fast code path if no intersection filter is present */
    accels.select(numIntersectionFilters4,numIntersectionFilters8,numIntersectionFilters16);
  
    /* build all hierarchies of this scene */
    accels.build(0,0);
    
    /* make static geometry immutable */
    if (isStatic()) 
    {
      accels.immutable();
      for (size_t i=0; i<geometries.size(); i++)
        if (geometries[i]) geometries[i]->immutable();
    }

    /* delete geometry that is scheduled for delete */
    for (size_t i=0; i<geometries.size(); i++) // FIXME: this late deletion is inefficient in case of many geometries
    {
      Geometry* geom = geometries[i];
      if (!geom) continue;
      if (geom->isEnabled()) geom->clearModified(); // FIXME: should builders to this?
      if (geom->isErasing()) remove(geom);
    }

    updateInterface();

    if (State::instance()->verbosity(2)) {
      std::cout << "created scene intersector" << std::endl;
      accels.print(2);
      std::cout << "selected scene intersector" << std::endl;
      intersectors.print(2);
    }
    
    setModified(false);
  }

#if defined(TASKING_LOCKSTEP)

  void Scene::task_build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    LockStepTaskScheduler::Init init(threadIndex,threadCount,&lockstep_scheduler);
    if (threadIndex == 0) accels.build(threadIndex,threadCount);
  }

  void Scene::build (size_t threadIndex, size_t threadCount) 
  {
    LockStepTaskScheduler::Init init(threadIndex,threadCount,&lockstep_scheduler);
    if (threadIndex != 0) return;

    /* allow only one build at a time */
    Lock<MutexSys> lock(buildMutex);

    progress_monitor_counter = 0;

    //if (isStatic() && isBuild()) {
    //  throw_RTCError(RTC_INVALID_OPERATION,"static geometries cannot get committed twice");
    //  return;
    //}

    if (!ready()) {
      throw_RTCError(RTC_INVALID_OPERATION,"not all buffers are unmapped");
      return;
    }

    /* select fast code path if no intersection filter is present */
    accels.select(numIntersectionFilters4,numIntersectionFilters8,numIntersectionFilters16);

    /* if user provided threads use them */
    if (threadCount)
      accels.build(threadIndex,threadCount);

    /* otherwise use our own threads */
    else
    {
      TaskScheduler::EventSync event;
      new (&task) TaskScheduler::Task(&event,_task_build_parallel,this,TaskScheduler::getNumThreads(),nullptr,nullptr,"scene_build");
      TaskScheduler::addTask(-1,TaskScheduler::GLOBAL_FRONT,&task);
      event.sync();
    }

    /* make static geometry immutable */
    if (isStatic()) 
    {
      accels.immutable();
      for (size_t i=0; i<geometries.size(); i++)
        if (geometries[i]) geometries[i]->immutable();
    }

    /* delete geometry that is scheduled for delete */
    for (size_t i=0; i<geometries.size(); i++) // FIXME: this late deletion is inefficient in case of many geometries
    {
      Geometry* geom = geometries[i];
      if (!geom) continue;
      if (geom->isEnabled()) geom->clearModified(); // FIXME: should builders to this?
      if (geom->isErasing()) remove(geom);
    }

    updateInterface();
    setModified(false);

    if (State::instance()->verbosity(2)) {
      std::cout << "created scene intersector" << std::endl;
      accels.print(2);
      std::cout << "selected scene intersector" << std::endl;
      intersectors.print(2);
    }
  }

#endif

#if defined(TASKING_TBB_INTERNAL)

  void Scene::build (size_t threadIndex, size_t threadCount) 
  {
    Ref<TaskSchedulerTBB> scheduler = nullptr;
    {
      Lock<MutexSys> lock(schedulerMutex);
      scheduler = this->scheduler;
      if (scheduler == null) this->scheduler = scheduler = new TaskSchedulerTBB;
    }

    if (threadCount != 0) 
    {
      if (threadIndex > 0) {
        scheduler->join();
        return;
      } else
        scheduler->wait_for_threads(threadCount);
    }

    /* try to obtain build lock */
    TryLock<MutexSys> lock(buildMutex);

    /* join hierarchy build */
    if (!lock.isLocked()) {
      scheduler->join();
      return;
    }

    if (!isModified()) {
      this->scheduler = nullptr;
      return;
    }

    progress_monitor_counter = 0;

    if (!ready()) {
      this->scheduler = nullptr;
      throw_RTCError(RTC_INVALID_OPERATION,"not all buffers are unmapped");
    }

    scheduler->spawn_root  ([&]() { build_task(); }, 1, threadCount == 0);
    this->scheduler = nullptr;
  }

#endif

#if defined(TASKING_TBB)

  void Scene::build (size_t threadIndex, size_t threadCount) 
  {
    /* let threads wait for build to finish in rtcCommitThread mode */
    if (threadCount != 0) {
      if (threadIndex > 0) {
        group_barrier.wait(threadCount); // FIXME: use barrier that waits in condition
        group->wait();
        return;
      }
    }

    /* try to obtain build lock */
    TryLock<MutexSys> lock(buildMutex);

    /* join hierarchy build */
    if (!lock.isLocked()) {
#if USE_TASK_ARENA
      arena.execute([&]{ group->wait(); });
#else
      group->wait();
#endif
      while (!buildMutex.try_lock()) {
        __pause_cpu();
        yield();
#if USE_TASK_ARENA
        arena.execute([&]{ group->wait(); });
#else
        group->wait();
#endif
      }
      buildMutex.unlock();
      return;
    }

    if (!isModified()) {
      return;
    }

    if (!ready()) {
      throw_RTCError(RTC_INVALID_OPERATION,"not all buffers are unmapped");
      return;
    }

    /* for best performance set FTZ and DAZ flags in the MXCSR control and status register */
#if !defined(__MIC__)
    unsigned int mxcsr = _mm_getcsr();
    _mm_setcsr(mxcsr | /* FTZ */ (1<<15) | /* DAZ */ (1<<6));
#endif
    
    try {
    
      tbb::task_group_context ctx( tbb::task_group_context::isolated, tbb::task_group_context::default_traits | tbb::task_group_context::fp_settings );
      //ctx.set_priority(tbb::priority_high);

#if USE_TASK_ARENA
      arena.execute([&]{
#endif
          group->run([&]{
              tbb::parallel_for (size_t(0), size_t(1), [&] (size_t) { build_task(); }, ctx);
            });
          if (threadCount) group_barrier.wait(threadCount);
          group->wait();
#if USE_TASK_ARENA
        }); 
#endif
     
      /* reset MXCSR register again */
#if !defined(__MIC__)
      _mm_setcsr(mxcsr);
#endif
    } 
    catch (...) {

       /* reset MXCSR register again */
#if !defined(__MIC__)
    _mm_setcsr(mxcsr);
#endif

      accels.clear();
      updateInterface();
      throw;
    }
  }
#endif

  void Scene::write(std::ofstream& file)
  {
    int magick = 0x35238765LL;
    file.write((char*)&magick,sizeof(magick));
    int numGroups = size();
    file.write((char*)&numGroups,sizeof(numGroups));
    for (size_t i=0; i<numGroups; i++) {
      if (geometries[i]) geometries[i]->write(file);
      else { int type = -1; file.write((char*)&type,sizeof(type)); }
    }
  }

  void Scene::setProgressMonitorFunction(RTCProgressMonitorFunc func, void* ptr) 
  {
    static MutexSys mutex;
    mutex.lock();
    progress_monitor_function = func;
    progress_monitor_ptr      = ptr;
    mutex.unlock();
  }

  void Scene::progressMonitor(double dn)
  {
    if (progress_monitor_function) {
      size_t n = atomic_t(dn) + atomic_add(&progress_monitor_counter, atomic_t(dn));
      if (!progress_monitor_function(progress_monitor_ptr, n / (double(numPrimitives())))) {
#if defined(TASKING_TBB)
        throw_RTCError(RTC_CANCELLED,"progress monitor forced termination");
#endif
      }
    }
  }
}
