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

#include "scene.h"

#include "rtcore/twolevel_accel.h"

#include "bvh4/bvh4.h"
#include "bvh4mb/bvh4mb.h"

#if !defined(__MIC__)
#include "bvh4/bvh4_builder_toplevel.h"
#include "bvh4/bvh4.h"
#include "bvh4i/bvh4i.h"
#include "bvh8i/bvh8i.h"
#else
#include "../xeonphi/bvh4i/bvh4i.h"
#endif

namespace embree
{
  Scene::Scene (RTCSceneFlags flags, RTCAlgorithmFlags aflags)
    : flags(flags), aflags(aflags), numMappedBuffers(0), is_build(false), needTriangles(false), needVertices(false),
      numTriangleMeshes(0), numTriangleMeshes2(0), numUserGeometries(0),
      flat_triangle_source_1(this,1), flat_triangle_source_2(this,2)
  {
#if defined(__MIC__)
    accels.accel0 = NULL; 
    accels.accel1 = NULL; // BVH4i::BVH4iVirtualGeometryBinnedSAH(this);
    accels.accel2 = NULL;
 
    if (g_builder == "default") 
      {
	if (isStatic())
	  {
	    if (g_verbose >= 1) std::cout << "STATIC BUILDER MODE" << std::endl;
	    accels.accel0 = BVH4i::BVH4iTriangle1ObjectSplitBinnedSAH(this);
	  }
	else
	  {
	    if (g_verbose >= 1) std::cout << "DYNAMIC BUILDER MODE" << std::endl;
	    accels.accel0 = BVH4i::BVH4iTriangle1ObjectSplitMorton(this);
	  }
      }
    else
      {
	if (g_builder == "sah" || g_builder == "objectsplit")
	  {
	    accels.accel0 = BVH4i::BVH4iTriangle1ObjectSplitBinnedSAH(this);
	  }
	else if (g_builder == "fast" || g_builder == "morton")
	  {
	    accels.accel0 = BVH4i::BVH4iTriangle1ObjectSplitMorton(this);
	  }
	else if (g_builder == "fast_enhanced" || g_builder == "morton.enhanced")
	  {
	    accels.accel0 = BVH4i::BVH4iTriangle1ObjectSplitEnhancedMorton(this);
	  }
	else if (g_builder == "high_quality" || g_builder == "presplits")
	  {
	    accels.accel0 = BVH4i::BVH4iTriangle1PreSplitsBinnedSAH(this);
	  }
	else throw std::runtime_error("unknown builder "+g_builder+" for BVH4i<Triangle1>");

      }
#else

    /* create default acceleration structure */
    if (g_top_accel == "default" && g_tri_accel == "default") 
    {
      if (isStatic()) {
        int mode =  4*(int)isCoherent() + 2*(int)isCompact() + 1*(int)isRobust();
        switch (mode) {
        case /*0b000*/ 0: 
#if defined (__TARGET_AVX__)
          if (has_feature(AVX2) && aflags == RTC_INTERSECT1) 
          {
            if (isHighQuality()) accels.accel0 = BVH4::BVH4Triangle8SpatialSplit(this); 
            else                 accels.accel0 = BVH4::BVH4Triangle8ObjectSplit(this); 
          }
          else 
#endif
          {
            if (isHighQuality()) accels.accel0 = BVH4::BVH4Triangle4SpatialSplit(this);
            else                 accels.accel0 = BVH4::BVH4Triangle4ObjectSplit(this); 
          }
          break;

        case /*0b001*/ 1: accels.accel0 = BVH4::BVH4Triangle4vObjectSplit(this); break;
        case /*0b010*/ 2: accels.accel0 = BVH4::BVH4Triangle4iObjectSplit(this); break;
        case /*0b011*/ 3: accels.accel0 = BVH4::BVH4Triangle4iObjectSplit(this); break;
        case /*0b100*/ 4: 
          if (isHighQuality()) accels.accel0 = BVH4::BVH4Triangle1SpatialSplit(this);
          else                 accels.accel0 = BVH4::BVH4Triangle1ObjectSplit(this); 
          break;
        case /*0b101*/ 5: accels.accel0 = BVH4::BVH4Triangle1vObjectSplit(this); break;
        case /*0b110*/ 6: accels.accel0 = BVH4::BVH4Triangle4iObjectSplit(this); break;
        case /*0b111*/ 7: accels.accel0 = BVH4::BVH4Triangle4iObjectSplit(this); break;
        }
        accels.accel1 = BVH4MB::BVH4MBTriangle1v(this); 
        accels.accel2 = new TwoLevelAccel("bvh4",this); 
      } else {
        int mode =  4*(int)isCoherent() + 2*(int)isCompact() + 1*(int)isRobust();
        switch (mode) {
        case /*0b000*/ 0: accels.accel0 = BVH4::BVH4BVH4Triangle4ObjectSplit(this); break;
        case /*0b001*/ 1: accels.accel0 = BVH4::BVH4BVH4Triangle4vObjectSplit(this); break;
        case /*0b010*/ 2: accels.accel0 = BVH4::BVH4BVH4Triangle4vObjectSplit(this); break;
        case /*0b011*/ 3: accels.accel0 = BVH4::BVH4BVH4Triangle4vObjectSplit(this); break;
        case /*0b100*/ 4: accels.accel0 = BVH4::BVH4BVH4Triangle1ObjectSplit(this); break;
        case /*0b101*/ 5: accels.accel0 = BVH4::BVH4BVH4Triangle1vObjectSplit(this); break;
        case /*0b110*/ 6: accels.accel0 = BVH4::BVH4BVH4Triangle1vObjectSplit(this); break;
        case /*0b111*/ 7: accels.accel0 = BVH4::BVH4BVH4Triangle1vObjectSplit(this); break;
        }
        accels.accel1 = BVH4MB::BVH4MBTriangle1v(this);
        accels.accel2 = new TwoLevelAccel("bvh4",this);
      }
    }

    /* create user specified acceleration structure */
    else if (g_top_accel == "default") 
    {
      if      (g_tri_accel == "bvh4.bvh4.triangle1.morton") accels.accel0 = BVH4::BVH4BVH4Triangle1Morton(this);
      else if (g_tri_accel == "bvh4.bvh4.triangle1")    accels.accel0 = BVH4::BVH4BVH4Triangle1ObjectSplit(this);
      else if (g_tri_accel == "bvh4.bvh4.triangle4")    accels.accel0 = BVH4::BVH4BVH4Triangle4ObjectSplit(this);
      else if (g_tri_accel == "bvh4.bvh4.triangle1v")   accels.accel0 = BVH4::BVH4BVH4Triangle1vObjectSplit(this);
      else if (g_tri_accel == "bvh4.bvh4.triangle4v")   accels.accel0 = BVH4::BVH4BVH4Triangle4vObjectSplit(this);
      else if (g_tri_accel == "bvh4.triangle1")         accels.accel0 = BVH4::BVH4Triangle1(this);
      else if (g_tri_accel == "bvh4.triangle4")         accels.accel0 = BVH4::BVH4Triangle4(this);
#if defined (__TARGET_AVX__)
      else if (g_tri_accel == "bvh4.triangle8")         accels.accel0 = BVH4::BVH4Triangle8(this);
#endif
      else if (g_tri_accel == "bvh4.triangle1v")        accels.accel0 = BVH4::BVH4Triangle1v(this);
      else if (g_tri_accel == "bvh4.triangle4v")        accels.accel0 = BVH4::BVH4Triangle4v(this);
      else if (g_tri_accel == "bvh4.triangle4i")        accels.accel0 = BVH4::BVH4Triangle4i(this);
      else if (g_tri_accel == "bvh4i.triangle1")        accels.accel0 = BVH4i::BVH4iTriangle1(this);
      else if (g_tri_accel == "bvh4i.triangle4")        accels.accel0 = BVH4i::BVH4iTriangle4(this);
      else if (g_tri_accel == "bvh4i.triangle1.v1")     accels.accel0 = BVH4i::BVH4iTriangle1_v1(this);
      else if (g_tri_accel == "bvh4i.triangle1.v2")     accels.accel0 = BVH4i::BVH4iTriangle1_v2(this);
      else if (g_tri_accel == "bvh4i.triangle1.morton") accels.accel0 = BVH4i::BVH4iTriangle1_morton(this);
      else if (g_tri_accel == "bvh4i.triangle1.morton.enhanced") accels.accel0 = BVH4i::BVH4iTriangle1_morton_enhanced(this);
#if defined (__TARGET_AVX__)
      else if (g_tri_accel == "bvh8i.triangle1")        accels.accel0 = BVH8i::BVH8iTriangle1(this);
#endif
      else throw std::runtime_error("unknown triangle acceleration structure "+g_tri_accel);

      accels.accel1 = new TwoLevelAccel(g_top_accel,this);
    }
    else {
      accels.accel0 = new TwoLevelAccel(g_top_accel,this);
      accels.accel1 = NULL;
    }
#endif
  }
  
  Scene::~Scene () 
  {
    for (size_t i=0; i<geometries.size(); i++)
      delete geometries[i];
  }

  unsigned Scene::newUserGeometry () 
  {
    Geometry* geom = new UserGeometryScene::UserGeometry(this);
    return geom->id;
  }
  
  unsigned Scene::newInstance (Scene* scene) {
    Geometry* geom = new UserGeometryScene::Instance(this,scene);
    return geom->id;
  }

  unsigned Scene::newTriangleMesh (RTCGeometryFlags gflags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) 
  {
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      recordError(RTC_INVALID_OPERATION);
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > 2) {
      recordError(RTC_INVALID_OPERATION);
      return -1;
    }
    
    Geometry* geom = new TriangleMeshScene::TriangleMesh(this,gflags,numTriangles,numVertices,numTimeSteps);
    return geom->id;
  }

  unsigned Scene::newQuadraticBezierCurves (RTCGeometryFlags gflags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      recordError(RTC_INVALID_OPERATION);
      return -1;
    }
    
    Geometry* geom = new QuadraticBezierCurvesScene::QuadraticBezierCurves(this,gflags,numCurves,numVertices);
    return geom->id;
  }

  unsigned Scene::add(Geometry* geometry) 
  {
#if !defined(__MIC__)
    Lock<AtomicMutex> lock(geometriesMutex);
#endif
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
#if !defined(__MIC__)
    Lock<AtomicMutex> lock(geometriesMutex);
#endif
    usedIDs.push_back(geometry->id);
    geometries[geometry->id] = NULL;
    delete geometry;
  }

  void Scene::build (size_t threadIndex, size_t threadCount) {
    accels.build(threadIndex,threadCount);
  }

  void Scene::task_build(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event) {
    build(threadIndex,threadCount);
  }

  void Scene::build () 
  {
#if !defined(__MIC__)
    Lock<MutexSys> lock(mutex);
#endif

    if ((isStatic() && isBuild()) || !ready()) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }

    /* verify geometry in debug mode  */
#if defined(DEBUG)
    for (size_t i=0; i<geometries.size(); i++) {
      if (geometries[i]) {
        if (!geometries[i]->verify()) {
          std::cerr << "RTCore: invalid geometry specified" << std::endl;
          throw std::runtime_error("invalid geometry");
        }
      }
    }
#endif

    /* spawn build task */
    TaskScheduler::EventSync event;
    new (&task) TaskScheduler::Task(&event,NULL,NULL,1,_task_build,this,"scene_build");
    TaskScheduler::addTask(-1,TaskScheduler::GLOBAL_FRONT,&task);
    event.sync();

    /* make static geometry immutable */
    if (isStatic()) 
    {
      accels.immutable();
      for (size_t i=0; i<geometries.size(); i++)
        geometries[i]->immutable();
    }

    /* delete geometry that is scheduled for delete */
    for (size_t i=0; i<geometries.size(); i++) 
    {
      Geometry* geom = geometries[i];
      if (geom == NULL || geom->state != Geometry::ERASING) continue;
      remove(geom);
    }

    /* update bounds */
    bounds = accels.bounds;
    intersectors = accels.intersectors;
    is_build = true;

    /* enable only algorithms choosen by application */
    if ((aflags & RTC_INTERSECT1) == 0) {
      intersectors.intersector1.intersect = NULL;
      intersectors.intersector1.occluded = NULL;
    }
    if ((aflags & RTC_INTERSECT4) == 0) {
      intersectors.intersector4.intersect = NULL;
      intersectors.intersector4.occluded = NULL;
    }
    if ((aflags & RTC_INTERSECT8) == 0) {
      intersectors.intersector8.intersect = NULL;
      intersectors.intersector8.occluded = NULL;
    }
    if ((aflags & RTC_INTERSECT16) == 0) {
      intersectors.intersector16.intersect = NULL;
      intersectors.intersector16.occluded = NULL;
    }

    if (g_verbose >= 2) {
      std::cout << "created scene intersector" << std::endl;
      accels.print(2);
      std::cout << "selected scene intersector" << std::endl;
      intersectors.print(2);
    }
  }
}
