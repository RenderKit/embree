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
#include "../xeon/bvh/bvh4_factory.h"
#include "../xeon/bvh/bvh8_factory.h"
#else
#include "../xeonphi/bvh4i/bvh4i_factory.h"
#include "../xeonphi/bvh4mb/bvh4mb_factory.h"
#include "../xeonphi/bvh4hair/bvh4hair_factory.h"
#endif
 
namespace embree
{
  /* error raising rtcIntersect and rtcOccluded functions */
  void missing_rtcCommit()     { throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed"); }
  void invalid_rtcIntersect1() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect and rtcOccluded not enabled"); }
  void invalid_rtcIntersect4() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect4 and rtcOccluded4 not enabled"); }
  void invalid_rtcIntersect8() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect8 and rtcOccluded8 not enabled"); }
  void invalid_rtcIntersect16() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect16 and rtcOccluded16 not enabled"); }

 
/* number of created scene */
  AtomicCounter Scene::numScenes = 0;

  Scene::Scene (Device* device, RTCSceneFlags sflags, RTCAlgorithmFlags aflags)
    : device(device), 
      Accel(AccelData::TY_UNKNOWN),
      flags(sflags), aflags(aflags), numMappedBuffers(0), is_build(false), modified(true), 
      needTriangleIndices(false), needTriangleVertices(false), 
      needQuadIndices(false), needQuadVertices(false), 
      needBezierIndices(false), needBezierVertices(false),
      needLineIndices(false), needLineVertices(false),
      needPointVertices(false),
      needSubdivIndices(false), needSubdivVertices(false),
      numIntersectionFilters4(0), numIntersectionFilters8(0), numIntersectionFilters16(0),
      commitCounter(0), commitCounterSubdiv(0), 
      progress_monitor_function(nullptr), progress_monitor_ptr(nullptr), progress_monitor_counter(0),
      progressInterface(this)
  {
#if defined(TASKING_LOCKSTEP) 
    lockstep_scheduler.taskBarrier.init(MAX_THREADS);
#elif defined(TASKING_TBB_INTERNAL)
    scheduler = nullptr;
#else
    group = new tbb::task_group;
#endif

    intersectors = Accel::Intersectors(missing_rtcCommit);

    if (device->scene_flags != -1)
      flags = (RTCSceneFlags) device->scene_flags;

    if (aflags & RTC_INTERPOLATE) {
      needTriangleIndices = true;
      needQuadIndices = true;
      needBezierIndices = true;
      needLineIndices = true;
      //needSubdivIndices = true; // not required for interpolation
      needTriangleVertices = true;
      needQuadVertices = true;      
      needBezierVertices = true;
      needLineVertices = true;
      needSubdivVertices = true;
    }

#if defined(__MIC__)
    needBezierVertices = true;
    needSubdivVertices = true;

    accels.add( BVH4mbFactory::BVH4mbTriangle1ObjectSplitBinnedSAH(this) );
    accels.add( BVH4iFactory::BVH4iVirtualGeometryBinnedSAH(this, isRobust()));
    accels.add( BVH4HairFactory::BVH4HairBinnedSAH(this));
    accels.add( BVH4iFactory::BVH4iSubdivMeshBinnedSAH(this, isRobust() ));

    if (device->verbosity(1))
    {
      std::cout << "scene flags: static " << isStatic() << " compact = " << isCompact() << " high quality = " << isHighQuality() << " robust = " << isRobust() << std::endl;
    }
    
    if (device->tri_accel == "default" || device->tri_accel == "bvh4i")   
    {
      if (device->tri_builder == "default") 
      {
        if (isStatic())
        {
          if (device->verbosity(1)) std::cout << "STATIC BUILDER MODE" << std::endl;
          if ( isCompact() )
            accels.add(BVH4iFactory::BVH4iTriangle1MemoryConservativeBinnedSAH(this,isRobust()));
          else if ( isHighQuality() )
            accels.add(BVH4iFactory::BVH4iTriangle1ObjectSplitBinnedSAH(this,isRobust()));
          else
            accels.add(BVH4iFactory::BVH4iTriangle1ObjectSplitBinnedSAH(this,isRobust()));
        }
        else
        {
          if (device->verbosity(1)) std::cout << "DYNAMIC BUILDER MODE" << std::endl;
          accels.add(BVH4iFactory::BVH4iTriangle1ObjectSplitMorton(this,isRobust()));
        }
      }
      else
      {
        if (device->tri_builder == "sah" || device->tri_builder == "bvh4i" || device->tri_builder == "bvh4i.sah") {
          accels.add(BVH4iFactory::BVH4iTriangle1ObjectSplitBinnedSAH(this,isRobust()));
        }
        else if (device->tri_builder == "fast" || device->tri_builder == "morton") {
          accels.add(BVH4iFactory::BVH4iTriangle1ObjectSplitMorton(this,isRobust()));
        }
        else if (device->tri_builder == "fast_enhanced" || device->tri_builder == "morton.enhanced") {
          accels.add(BVH4iFactory::BVH4iTriangle1ObjectSplitEnhancedMorton(this,isRobust()));
        }
        else if (device->tri_builder == "high_quality" || device->tri_builder == "presplits") {
          accels.add(BVH4iFactory::BVH4iTriangle1PreSplitsBinnedSAH(this,isRobust()));
        }
        else if (device->tri_builder == "compact" ||
                 device->tri_builder == "memory_conservative") {
          accels.add(BVH4iFactory::BVH4iTriangle1MemoryConservativeBinnedSAH(this,isRobust()));
        }
        else if (device->tri_builder == "morton64") {
          accels.add(BVH4iFactory::BVH4iTriangle1ObjectSplitMorton64Bit(this,isRobust()));
        }
        
        else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+device->tri_builder+" for BVH4i<Triangle1>");
      }
    }
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown accel "+device->tri_accel);
    
#else
    createTriangleAccel();
    createTriangleMBAccel();
    createQuadAccel();
    createQuadMBAccel();
    createHairAccel();
    createHairMBAccel();
    createLineAccel();
    createLineMBAccel();
    createPointAccel();
    createPointMBAccel();
    createSubdivAccel();
    accels.add(device->bvh4_factory->BVH4InstancedBVH4Triangle4ObjectSplit(this));
    accels.add(device->bvh4_factory->BVH4UserGeometry(this)); // has to be the last as the instID field of a hit instance is not invalidated by other hit geometry
    accels.add(device->bvh4_factory->BVH4UserGeometryMB(this)); // has to be the last as the instID field of a hit instance is not invalidated by other hit geometry
#endif

    /* increment number of scenes */
    numScenes++;
  }

#if !defined(__MIC__)

  void Scene::createTriangleAccel()
  {
    if (device->tri_accel == "default") 
    {
      if (isStatic()) {
        int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
        switch (mode) {
        case /*0b00*/ 0: 
#if defined (__TARGET_AVX__)
          if (device->hasISA(AVX))
	  {
            if (isHighQuality()) accels.add(device->bvh8_factory->BVH8Triangle4SpatialSplit(this));
            else                 accels.add(device->bvh8_factory->BVH8Triangle4ObjectSplit(this));
          }
          else 
#endif
          {
            if (isHighQuality()) accels.add(device->bvh4_factory->BVH4Triangle4SpatialSplit(this));
            else accels.add(device->bvh4_factory->BVH4Triangle4ObjectSplit(this));            
          }
          break;

        case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Triangle4vObjectSplit(this)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4iObjectSplit(this)); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4iObjectSplit(this)); break;
        }
      }
      else 
      {
        int mode =  2*(int)isCompact() + 1*(int)isRobust();
        switch (mode) {
        case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4Triangle4Twolevel(this)); break;
        case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Triangle4Twolevel(this)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4iTwolevel(this)); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4iTwolevel(this)); break;
        }
      }
    }
    else if (device->tri_accel == "bvh4.triangle4")       accels.add(device->bvh4_factory->BVH4Triangle4(this));
    else if (device->tri_accel == "bvh4.triangle4v")      accels.add(device->bvh4_factory->BVH4Triangle4v(this));
    else if (device->tri_accel == "bvh4.triangle4i")      accels.add(device->bvh4_factory->BVH4Triangle4i(this));

#if defined (__TARGET_AVX__)
    else if (device->tri_accel == "bvh4.triangle8")       accels.add(device->bvh4_factory->BVH4Triangle8(this));
    else if (device->tri_accel == "bvh8.triangle4")       accels.add(device->bvh8_factory->BVH8Triangle4(this));
    else if (device->tri_accel == "bvh8.triangle8")       accels.add(device->bvh8_factory->BVH8Triangle8(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown triangle acceleration structure "+device->tri_accel);
  }


  void Scene::createQuadAccel()
  {
    if (device->quad_accel == "default") 
    {
      int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
      switch (mode) {
      case /*0b00*/ 0:
      case /*0b01*/ 1:
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8Quad4v(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Quad4v(this));
        break;

      case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Quad4i(this)); break;
      case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4i(this)); break;
      }
    }
    else if (device->quad_accel == "bvh4.quad4v")       accels.add(device->bvh4_factory->BVH4Quad4v(this));
    else if (device->quad_accel == "bvh4.quad4i")       accels.add(device->bvh4_factory->BVH4Quad4i(this));
#if defined (__TARGET_AVX__)
    else if (device->quad_accel == "bvh8.quad4v")       accels.add(device->bvh8_factory->BVH8Quad4v(this));
    else if (device->quad_accel == "bvh8.quad4i")       accels.add(device->bvh8_factory->BVH8Quad4i(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown quad acceleration structure "+device->quad_accel);
  }


  void Scene::createQuadMBAccel()
  {
    if (device->quad_accel_mb == "default") 
    {
      int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
      switch (mode) {
      case /*0b00*/ 0:
      case /*0b01*/ 1:
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8Quad4iMB(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Quad4iMB(this));
        break;

      case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Quad4iMB(this)); break;
      case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4iMB(this)); break;
      }
    }
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown quad acceleration structure "+device->quad_accel);
  }


  void Scene::createTriangleMBAccel()
  {
    if (device->tri_accel_mb == "default")
    {
#if defined (__TARGET_AVX__)
      if (device->hasISA(AVX))
      {
        accels.add(device->bvh8_factory->BVH8Triangle4vMB(this));
      }
      else
#endif
      {
        accels.add(device->bvh4_factory->BVH4Triangle4vMB(this));
      }
    }
    else if (device->tri_accel_mb == "bvh4.triangle4vmb") accels.add(device->bvh4_factory->BVH4Triangle4vMB(this));
#if defined (__TARGET_AVX__)
    else if (device->tri_accel_mb == "bvh8.triangle4vmb") accels.add(device->bvh8_factory->BVH8Triangle4vMB(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown motion blur triangle acceleration structure "+device->tri_accel_mb);
  }

  void Scene::createHairAccel()
  {
    if (device->hair_accel == "default")
    {
      int mode = 2*(int)isCompact() + 1*(int)isRobust();
      if (isStatic())
      {
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX))
        {
          switch (mode) {
          case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8OBBBezier1v(this,isHighQuality())); break;
          case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8OBBBezier1v(this,isHighQuality())); break;
          case /*0b10*/ 2: accels.add(device->bvh8_factory->BVH8OBBBezier1i(this,isHighQuality())); break;
          case /*0b11*/ 3: accels.add(device->bvh8_factory->BVH8OBBBezier1i(this,isHighQuality())); break;
          }
        }
        else
  #endif
        {
          switch (mode) {
          case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4OBBBezier1v(this,isHighQuality())); break;
          case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4OBBBezier1v(this,isHighQuality())); break;
          case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4OBBBezier1i(this,isHighQuality())); break;
          case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4OBBBezier1i(this,isHighQuality())); break;
          }
        }
      }
      else
      {
        switch (mode) {
        case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4Bezier1v(this)); break;
        case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Bezier1v(this)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Bezier1i(this)); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Bezier1i(this)); break;
        }
      }
    }
    else if (device->hair_accel == "bvh4.bezier1v"    ) accels.add(device->bvh4_factory->BVH4Bezier1v(this));
    else if (device->hair_accel == "bvh4.bezier1i"    ) accels.add(device->bvh4_factory->BVH4Bezier1i(this));
    else if (device->hair_accel == "bvh4obb.bezier1v" ) accels.add(device->bvh4_factory->BVH4OBBBezier1v(this,false));
    else if (device->hair_accel == "bvh4obb.bezier1i" ) accels.add(device->bvh4_factory->BVH4OBBBezier1i(this,false));
#if defined (__TARGET_AVX__)
    else if (device->hair_accel == "bvh8obb.bezier1v" ) accels.add(device->bvh8_factory->BVH8OBBBezier1v(this,false));
    else if (device->hair_accel == "bvh8obb.bezier1i" ) accels.add(device->bvh8_factory->BVH8OBBBezier1i(this,false));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown hair acceleration structure "+device->hair_accel);
  }

  void Scene::createHairMBAccel()
  {
    if (device->hair_accel_mb == "default")
    {
#if defined (__TARGET_AVX__)
      if (device->hasISA(AVX))
      {
        accels.add(device->bvh8_factory->BVH8OBBBezier1iMB(this,false));
      }
      else
#endif
      {
        accels.add(device->bvh4_factory->BVH4OBBBezier1iMB(this,false));
      }
    }
    else if (device->hair_accel_mb == "bvh4obb.bezier1imb") accels.add(device->bvh4_factory->BVH4OBBBezier1iMB(this,false));
#if defined (__TARGET_AVX__)
    else if (device->hair_accel_mb == "bvh8obb.bezier1imb") accels.add(device->bvh8_factory->BVH8OBBBezier1iMB(this,false));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown motion blur hair acceleration structure "+device->tri_accel_mb);
  }

  void Scene::createLineAccel()
  {
    if (device->line_accel == "default")
    {
      if (isStatic())
      {
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX) && !isCompact())
          accels.add(device->bvh8_factory->BVH8Line4i(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Line4i(this));
      }
      else
      {
        accels.add(device->bvh4_factory->BVH4Line4iTwolevel(this));
      }
    }
    else if (device->line_accel == "bvh4.line4i") accels.add(device->bvh4_factory->BVH4Line4i(this));
#if defined (__TARGET_AVX__)
    else if (device->line_accel == "bvh8.line4i") accels.add(device->bvh8_factory->BVH8Line4i(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown line segment acceleration structure "+device->line_accel);
  }

  void Scene::createLineMBAccel()
  {
    if (device->line_accel_mb == "default")
    {
#if defined (__TARGET_AVX__)
      if (device->hasISA(AVX) && !isCompact())
        accels.add(device->bvh8_factory->BVH8Line4iMB(this));
      else
#endif
        accels.add(device->bvh4_factory->BVH4Line4iMB(this));
    }
    else if (device->line_accel_mb == "bvh4.line4imb") accels.add(device->bvh4_factory->BVH4Line4iMB(this));
#if defined (__TARGET_AVX__)
    else if (device->line_accel_mb == "bvh8.line4imb") accels.add(device->bvh8_factory->BVH8Line4iMB(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown motion blur line segment acceleration structure "+device->line_accel_mb);
  }

  void Scene::createPointAccel()
  {
    if (device->line_accel == "default")
    {
      if (isStatic())
      {
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX) && !isCompact())
          accels.add(device->bvh8_factory->BVH8Point4i(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Point4i(this));
      }
      else
      {
        accels.add(device->bvh4_factory->BVH4Point4iTwolevel(this));
      }
    }
    else if (device->line_accel == "bvh4.point4i") accels.add(device->bvh4_factory->BVH4Point4i(this));
#if defined (__TARGET_AVX__)
    else if (device->line_accel == "bvh8.point4i") accels.add(device->bvh8_factory->BVH8Point4i(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown point acceleration structure "+device->line_accel);
  }

  void Scene::createPointMBAccel()
  {
    if (device->line_accel_mb == "default")
    {
#if defined (__TARGET_AVX__)
      if (device->hasISA(AVX) && !isCompact())
        accels.add(device->bvh8_factory->BVH8Point4iMB(this));
      else
#endif
        accels.add(device->bvh4_factory->BVH4Point4iMB(this));
    }
    else if (device->line_accel_mb == "bvh4.point4imb") accels.add(device->bvh4_factory->BVH4Point4iMB(this));
#if defined (__TARGET_AVX__)
    else if (device->line_accel_mb == "bvh8.point4imb") accels.add(device->bvh8_factory->BVH8Point4iMB(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown motion blur point acceleration structure "+device->line_accel_mb);
  }

  void Scene::createSubdivAccel()
  {
    if (device->subdiv_accel == "default") 
    {
      if (isIncoherent(flags) && isStatic())
      {
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX))
        {
          accels.add(device->bvh8_factory->BVH8SubdivGridEager(this));
        }
        else
#endif
        {
          accels.add(device->bvh4_factory->BVH4SubdivGridEager(this));
        }
      }
      else
      {
        accels.add(device->bvh4_factory->BVH4SubdivPatch1Cached(this));
      }
    }
    else if (device->subdiv_accel == "bvh4.subdivpatch1cached") accels.add(device->bvh4_factory->BVH4SubdivPatch1Cached(this));
    else if (device->subdiv_accel == "bvh4.grid.eager"        ) accels.add(device->bvh4_factory->BVH4SubdivGridEager(this));
#if defined (__TARGET_AVX__)
    else if (device->subdiv_accel == "bvh8.grid.eager"        ) accels.add(device->bvh8_factory->BVH8SubdivGridEager(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown subdiv accel "+device->subdiv_accel);
  }

#endif

  Scene::~Scene () 
  {
    for (size_t i=0; i<geometries.size(); i++)
      delete geometries[i];

#if TASKING_TBB
    delete group; group = nullptr;
#endif

    /* decrement number of scenes */
    numScenes--;
  }

  void Scene::clear() {
  }

  unsigned Scene::newUserGeometry (size_t items, size_t numTimeSteps) 
  {
    Geometry* geom = new UserGeometry(this,items,numTimeSteps);
    return geom->id;
  }

  unsigned Scene::newInstance (Scene* scene) {
    Geometry* geom = new Instance(this,scene);
    return geom->id;
  }
  
  unsigned Scene::newGeometryInstance (Geometry* geom) {
    Geometry* instance = new GeometryInstance(this,geom);
    return instance->id;
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

  unsigned Scene::newQuadMesh (RTCGeometryFlags gflags, size_t numQuads, size_t numVertices, size_t numTimeSteps) 
  {
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > 2) {
      throw_RTCError(RTC_INVALID_OPERATION,"only 1 or 2 time steps supported");
      return -1;
    }
    
    Geometry* geom = new QuadMesh(this,gflags,numQuads,numVertices,numTimeSteps);
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
    if (device->hasISA(AVX))
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

  unsigned Scene::newLineSegments (RTCGeometryFlags gflags, size_t numSegments, size_t numVertices, size_t numTimeSteps)
  {
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > 2) {
      throw_RTCError(RTC_INVALID_OPERATION,"only 1 or 2 time steps supported");
      return -1;
    }

    Geometry* geom = new LineSegments(this,gflags,numSegments,numVertices,numTimeSteps);
    return geom->id;
  }

  unsigned Scene::newPoints(RTCGeometryFlags gflags, size_t numPoints, size_t numTimeSteps)
  {
	  if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
		  throw_RTCError(RTC_INVALID_OPERATION, "static scenes can only contain static geometries");
		  return -1;
	  }

	  if (numTimeSteps == 0 || numTimeSteps > 2) {
		  throw_RTCError(RTC_INVALID_OPERATION, "only 1 or 2 time steps supported");
		  return -1;
	  }

	  Geometry* geom = new Points(this, gflags, numPoints, numTimeSteps);
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

  void Scene::deleteGeometry(size_t geomID)
  {
    Lock<AtomicMutex> lock(geometriesMutex);
    
    if (isStatic())
      throw_RTCError(RTC_INVALID_OPERATION,"rtcDeleteGeometry cannot get called in static scenes");
    if (geomID >= geometries.size())
      throw_RTCError(RTC_INVALID_OPERATION,"invalid geometry ID");

    Geometry* geometry = geometries[geomID];
    if (geometry == nullptr)
      throw_RTCError(RTC_INVALID_OPERATION,"invalid geometry");
    
    geometry->disable();
    accels.deleteGeometry(geomID);
    usedIDs.push_back(geomID);
    geometries[geomID] = nullptr;
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

    /* do only reset tessellation cache on MIC */
#if defined(__MIC__)
    if (numScenes == 1)
      resetTessellationCache();
#endif
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
    }

    updateInterface();

    if (device->verbosity(2)) {
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
    }

    updateInterface();
    setModified(false);

    if (device->verbosity(2)) {
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
    AutoUnlock<MutexSys> buildLock(buildMutex);

    /* allocates own taskscheduler for each build */
    Ref<TaskSchedulerTBB> scheduler = nullptr;
    { 
      Lock<MutexSys> lock(schedulerMutex);
      scheduler = this->scheduler;
      if (scheduler == null) {
        buildLock.lock();
        this->scheduler = scheduler = new TaskSchedulerTBB;
      }
    }

    /* worker threads join build */
    if (!buildLock.isLocked()) {
      scheduler->join();
      return;
    }

    /* wait for all threads in rtcCommitThread mode */
    if (threadCount != 0)
      scheduler->wait_for_threads(threadCount);

    /* fast path for unchanged scenes */
    if (!isModified()) {
      scheduler->spawn_root([&]() { this->scheduler = nullptr; }, 1, threadCount == 0);
      return;
    }

    /* report error if scene not ready */
    if (!ready()) {
      scheduler->spawn_root([&]() { this->scheduler = nullptr; }, 1, threadCount == 0);
      throw_RTCError(RTC_INVALID_OPERATION,"not all buffers are unmapped");
    }

    /* initiate build */
    try {
      scheduler->spawn_root([&]() { build_task(); this->scheduler = nullptr; }, 1, threadCount == 0);
    }
    catch (...) {
      accels.clear();
      updateInterface();
      throw;
    }
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
      device->arena->execute([&]{ group->wait(); });
#else
      group->wait();
#endif
      while (!buildMutex.try_lock()) {
        __pause_cpu();
        yield();
#if USE_TASK_ARENA
        device->arena->execute([&]{ group->wait(); });
#else
        group->wait();
#endif
      }
      buildMutex.unlock();
      return;
    }

    if (!isModified()) {
      if (threadCount) group_barrier.wait(threadCount);
      return;
    }

    if (!ready()) {
      if (threadCount) group_barrier.wait(threadCount);
      throw_RTCError(RTC_INVALID_OPERATION,"not all buffers are unmapped");
      return;
    }

    /* for best performance set FTZ and DAZ flags in the MXCSR control and status register */
#if !defined(__MIC__)
    unsigned int mxcsr = _mm_getcsr();
    _mm_setcsr(mxcsr | /* FTZ */ (1<<15) | /* DAZ */ (1<<6));
#endif
    
    try {

#if TBB_INTERFACE_VERSION_MAJOR < 8    
      tbb::task_group_context ctx( tbb::task_group_context::isolated, tbb::task_group_context::default_traits);
#else
      tbb::task_group_context ctx( tbb::task_group_context::isolated, tbb::task_group_context::default_traits | tbb::task_group_context::fp_settings );
#endif
      //ctx.set_priority(tbb::priority_high);

#if USE_TASK_ARENA
      device->arena->execute([&]{
#endif
          group->run([&]{
              tbb::parallel_for (size_t(0), size_t(1), size_t(1), [&] (size_t) { build_task(); }, ctx);
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
#if !defined(TASKING_LOCKSTEP)
        throw_RTCError(RTC_CANCELLED,"progress monitor forced termination");
#endif
      }
    }
  }
}
