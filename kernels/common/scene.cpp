// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "../bvh/bvh4_factory.h"
#include "../bvh/bvh8_factory.h"
 
namespace embree
{
  /* error raising rtcIntersect and rtcOccluded functions */
  void missing_rtcCommit()      { throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed"); }
  void invalid_rtcIntersect1()  { throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect and rtcOccluded not enabled"); }
  void invalid_rtcIntersect4()  { throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect4 and rtcOccluded4 not enabled"); }
  void invalid_rtcIntersect8()  { throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect8 and rtcOccluded8 not enabled"); }
  void invalid_rtcIntersect16() { throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect16 and rtcOccluded16 not enabled"); }
  void invalid_rtcIntersectN()  { throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersectN and rtcOccludedN not enabled"); }

  Scene::Scene (Device* device)
    : Accel(AccelData::TY_UNKNOWN),
      device(device),
      flags_modified(true),
      scene_flags(RTC_SCENE_FLAG_NONE),
      quality_flags(RTC_BUILD_QUALITY_MEDIUM),
      is_build(false), modified(true),
      progressInterface(this), progress_monitor_function(nullptr), progress_monitor_ptr(nullptr), progress_monitor_counter(0), 
      numIntersectionFiltersN(0)
  {
    device->refInc();
    
#if defined(TASKING_INTERNAL) 
    scheduler = nullptr;
#elif defined(TASKING_TBB)
    group = new tbb::task_group;
#elif defined(TASKING_PPL)
    group = new concurrency::task_group;
#endif

    intersectors = Accel::Intersectors(missing_rtcCommit);

    /* one can overwrite flags through device for debugging */
    if (device->quality_flags != -1)
      quality_flags = (RTCBuildQuality) device->quality_flags;
    if (device->scene_flags != -1)
      scene_flags = (RTCSceneFlags) device->scene_flags;
  }

  Scene::~Scene () 
  {
#if defined(TASKING_TBB) || defined(TASKING_PPL)
    delete group; group = nullptr;
#endif

    /* detach all geometries */
    for (auto& geometry : geometries)
      if (geometry)
        geometry->detach();

    device->refDec();
  }
  
  void Scene::printStatistics()
  {
    /* calculate maximum number of time segments */
    unsigned max_time_steps = 0;
    for (size_t i=0; i<size(); i++)
      max_time_steps = max(max_time_steps,get(i)->numTimeSteps);

    /* initialize vectors*/
    std::vector<size_t> statistics[Geometry::NUM_TYPES];
    for (size_t i=0; i<Geometry::NUM_TYPES; i++)
      statistics[i].resize(max_time_steps);

    /* gather statistics */
    for (size_t i=0; i<size(); i++) 
    {
      int ty = __bsf(get(i)->type); 
      assert(ty<Geometry::NUM_TYPES);
      int timesegments = get(i)->numTimeSegments(); 
      assert((unsigned int)timesegments < max_time_steps);
      statistics[ty][timesegments] += get(i)->size();
    }

    /* print statistics */
    const char* names[Geometry::NUM_TYPES] = {
      "triangles",
      "quads",
      "curves",
      "segments",
      "subdivs",
      "usergeom",
      "instance",
      "group"
    };

    std::cout << "  segments: ";
    for (size_t t=0; t<max_time_steps; t++)
      std::cout << std::setw(10) << t;
    std::cout << std::endl;

    std::cout << "------------";
    for (size_t t=0; t<max_time_steps; t++)
      std::cout << "----------";
    std::cout << std::endl;
    
    for (size_t p=0; p<Geometry::NUM_TYPES; p++)
    {
      std::cout << std::setw(10) << names[p] << ": ";
      for (size_t t=0; t<max_time_steps; t++)
        std::cout << std::setw(10) << statistics[p][t];
      std::cout << std::endl;
    }
  }

  void Scene::createTriangleAccel()
  {
#if defined(EMBREE_GEOMETRY_TRIANGLES)
    if (device->tri_accel == "default") 
    {
      if (quality_flags != RTC_BUILD_QUALITY_LOW)
      {
        int mode =  2*(int)isCompactAccel() + 1*(int)isRobustAccel(); 
        switch (mode) {
        case /*0b00*/ 0: 
#if defined (EMBREE_TARGET_SIMD8)
          if (device->hasISA(AVX))
	  {
            if (quality_flags == RTC_BUILD_QUALITY_HIGH) 
              accels.add(device->bvh8_factory->BVH8Triangle4(this,BVHFactory::BuildVariant::HIGH_QUALITY,BVHFactory::IntersectVariant::FAST));
            else
              accels.add(device->bvh8_factory->BVH8Triangle4(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST));
          }
          else 
#endif
          { 
            if (quality_flags == RTC_BUILD_QUALITY_HIGH) 
              accels.add(device->bvh4_factory->BVH4Triangle4(this,BVHFactory::BuildVariant::HIGH_QUALITY,BVHFactory::IntersectVariant::FAST));
            else 
              accels.add(device->bvh4_factory->BVH4Triangle4(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST));
          }
          break;

        case /*0b01*/ 1: 
#if defined (EMBREE_TARGET_SIMD8)
          if (device->hasISA(AVX)) 
            accels.add(device->bvh8_factory->BVH8Triangle4v(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST));
          else
#endif
            accels.add(device->bvh4_factory->BVH4Triangle4v(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST));

          break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST  )); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST)); break;
        }
      }
      else /* dynamic */
      {
#if defined (EMBREE_TARGET_SIMD8)
          if (device->hasISA(AVX))
	  {
            int mode =  2*(int)isCompactAccel() + 1*(int)isRobustAccel();
            switch (mode) {
            case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8Triangle4 (this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::FAST  )); break;
            case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8Triangle4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::ROBUST)); break;
            case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::FAST  )); break;
            case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::ROBUST)); break;
            }
          }
          else
#endif
          {
            int mode =  2*(int)isCompactAccel() + 1*(int)isRobustAccel();
            switch (mode) {
            case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4Triangle4 (this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::FAST  )); break;
            case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Triangle4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::ROBUST)); break;
            case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::FAST  )); break;
            case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::ROBUST)); break;
            }
          }
      }
    }
    else if (device->tri_accel == "bvh4.triangle4")       accels.add(device->bvh4_factory->BVH4Triangle4 (this));
    else if (device->tri_accel == "bvh4.triangle4v")      accels.add(device->bvh4_factory->BVH4Triangle4v(this));
    else if (device->tri_accel == "bvh4.triangle4i")      accels.add(device->bvh4_factory->BVH4Triangle4i(this));
    else if (device->tri_accel == "qbvh4.triangle4i")     accels.add(device->bvh4_factory->BVH4QuantizedTriangle4i(this));

#if defined (EMBREE_TARGET_SIMD8)
    else if (device->tri_accel == "bvh8.triangle4")       accels.add(device->bvh8_factory->BVH8Triangle4 (this));
    else if (device->tri_accel == "bvh8.triangle4v")      accels.add(device->bvh8_factory->BVH8Triangle4v(this));
    else if (device->tri_accel == "bvh8.triangle4i")      accels.add(device->bvh8_factory->BVH8Triangle4i(this));
    else if (device->tri_accel == "qbvh8.triangle4i")     accels.add(device->bvh8_factory->BVH8QuantizedTriangle4i(this));
    else if (device->tri_accel == "qbvh8.triangle4")      accels.add(device->bvh8_factory->BVH8QuantizedTriangle4(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown triangle acceleration structure "+device->tri_accel);
#endif
  }

  void Scene::createTriangleMBAccel()
  {
#if defined(EMBREE_GEOMETRY_TRIANGLES)
    if (device->tri_accel_mb == "default")
    {
      int mode =  2*(int)isCompactAccel() + 1*(int)isRobustAccel(); 
      
#if defined (EMBREE_TARGET_SIMD8)
      if (device->hasISA(AVX2)) // BVH8 reduces performance on AVX only-machines
      {
        switch (mode) {
        case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8Triangle4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST  )); break;
        case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8Triangle4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST  )); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST)); break;
        }
      }
      else
#endif
      {
        switch (mode) {
        case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST  )); break;
        case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST  )); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST)); break;
        }
      }
    }
    else if (device->tri_accel_mb == "bvh4.triangle4imb") accels.add(device->bvh4_factory->BVH4Triangle4iMB(this));
    else if (device->tri_accel_mb == "bvh4.triangle4vmb") accels.add(device->bvh4_factory->BVH4Triangle4vMB(this));
#if defined (EMBREE_TARGET_SIMD8)
    else if (device->tri_accel_mb == "bvh8.triangle4imb") accels.add(device->bvh8_factory->BVH8Triangle4iMB(this));
    else if (device->tri_accel_mb == "bvh8.triangle4vmb") accels.add(device->bvh8_factory->BVH8Triangle4vMB(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown motion blur triangle acceleration structure "+device->tri_accel_mb);
#endif
  }

  void Scene::createQuadAccel()
  {
#if defined(EMBREE_GEOMETRY_QUADS)
    if (device->quad_accel == "default") 
    {
      if (quality_flags != RTC_BUILD_QUALITY_LOW)
      {
        /* static */
        int mode =  2*(int)isCompactAccel() + 1*(int)isRobustAccel(); 
        switch (mode) {
        case /*0b00*/ 0:
#if defined (EMBREE_TARGET_SIMD8)
          if (device->hasISA(AVX))
          {
            if (quality_flags == RTC_BUILD_QUALITY_HIGH) 
              accels.add(device->bvh8_factory->BVH8Quad4v(this,BVHFactory::BuildVariant::HIGH_QUALITY,BVHFactory::IntersectVariant::FAST));
            else
              accels.add(device->bvh8_factory->BVH8Quad4v(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST));
          }
          else
#endif
          {
            if (quality_flags == RTC_BUILD_QUALITY_HIGH) 
              accels.add(device->bvh4_factory->BVH4Quad4v(this,BVHFactory::BuildVariant::HIGH_QUALITY,BVHFactory::IntersectVariant::FAST));
            else
              accels.add(device->bvh4_factory->BVH4Quad4v(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST));
          }
          break;

        case /*0b01*/ 1:
#if defined (EMBREE_TARGET_SIMD8)
          if (device->hasISA(AVX))
            accels.add(device->bvh8_factory->BVH8Quad4v(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST));
          else
#endif
            accels.add(device->bvh4_factory->BVH4Quad4v(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST));
          break;

        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Quad4i(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST)); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4i(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST)); break;
        }
      }
      else /* dynamic */
      {
#if defined (EMBREE_TARGET_SIMD8)
          if (device->hasISA(AVX))
	  {
            int mode =  2*(int)isCompactAccel() + 1*(int)isRobustAccel();
            switch (mode) {
            case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8Quad4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::FAST)); break;
            case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8Quad4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::ROBUST)); break;
            case /*0b10*/ 2: accels.add(device->bvh8_factory->BVH8Quad4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::FAST)); break;
            case /*0b11*/ 3: accels.add(device->bvh8_factory->BVH8Quad4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::ROBUST)); break;
            }
          }
          else
#endif
          {
            int mode =  2*(int)isCompactAccel() + 1*(int)isRobustAccel();
            switch (mode) {
            case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4Quad4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::FAST)); break;
            case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Quad4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::ROBUST)); break;
            case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Quad4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::FAST)); break;
            case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4v(this,BVHFactory::BuildVariant::DYNAMIC,BVHFactory::IntersectVariant::ROBUST)); break;
            }
          }
      }
    }
    else if (device->quad_accel == "bvh4.quad4v")       accels.add(device->bvh4_factory->BVH4Quad4v(this));
    else if (device->quad_accel == "bvh4.quad4i")       accels.add(device->bvh4_factory->BVH4Quad4i(this));
    else if (device->quad_accel == "qbvh4.quad4i")      accels.add(device->bvh4_factory->BVH4QuantizedQuad4i(this));

#if defined (EMBREE_TARGET_SIMD8)
    else if (device->quad_accel == "bvh8.quad4v")       accels.add(device->bvh8_factory->BVH8Quad4v(this));
    else if (device->quad_accel == "bvh8.quad4i")       accels.add(device->bvh8_factory->BVH8Quad4i(this));
    else if (device->quad_accel == "qbvh8.quad4i")      accels.add(device->bvh8_factory->BVH8QuantizedQuad4i(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown quad acceleration structure "+device->quad_accel);
#endif
  }

  void Scene::createQuadMBAccel()
  {
#if defined(EMBREE_GEOMETRY_QUADS)
    if (device->quad_accel_mb == "default") 
    {
      int mode =  2*(int)isCompactAccel() + 1*(int)isRobustAccel(); 
      switch (mode) {
      case /*0b00*/ 0:
#if defined (EMBREE_TARGET_SIMD8)
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8Quad4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Quad4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST));
        break;

      case /*0b01*/ 1:
#if defined (EMBREE_TARGET_SIMD8)
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8Quad4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Quad4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST));
        break;

      case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Quad4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::FAST  )); break;
      case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4iMB(this,BVHFactory::BuildVariant::STATIC,BVHFactory::IntersectVariant::ROBUST)); break;
      }
    }
    else if (device->quad_accel_mb == "bvh4.quad4imb") accels.add(device->bvh4_factory->BVH4Quad4iMB(this));
#if defined (EMBREE_TARGET_SIMD8)
    else if (device->quad_accel_mb == "bvh8.quad4imb") accels.add(device->bvh8_factory->BVH8Quad4iMB(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown quad motion blur acceleration structure "+device->quad_accel_mb);
#endif
  }

  void Scene::createHairAccel()
  {
#if defined(EMBREE_GEOMETRY_CURVES)
    if (device->hair_accel == "default")
    {
      int mode = 2*(int)isCompactAccel() + 1*(int)isRobustAccel();
#if defined (EMBREE_TARGET_SIMD8)
      if (device->hasISA(AVX2)) // only enable on HSW machines, for SNB this codepath is slower
      {
        switch (mode) {
        case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8OBBBezier8v(this)); break;
        case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8OBBBezier8v(this)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4OBBBezier8i(this)); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4OBBBezier8i(this)); break;
        }
      }
      else
#endif
      {
        switch (mode) {
        case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4OBBBezier4v(this)); break;
        case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4OBBBezier4v(this)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4OBBBezier4i(this)); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4OBBBezier4i(this)); break;
        }
      }
    }
    else if (device->hair_accel == "bvh4.bezier1v"    ) accels.add(device->bvh4_factory->BVH4Bezier1v(this));
    else if (device->hair_accel == "bvh4.bezier1i"    ) accels.add(device->bvh4_factory->BVH4Bezier1i(this));
    else if (device->hair_accel == "bvh4obb.bezier1v" ) accels.add(device->bvh4_factory->BVH4OBBBezier1v(this));
    else if (device->hair_accel == "bvh4obb.bezier1i" ) accels.add(device->bvh4_factory->BVH4OBBBezier1i(this));
    else if (device->hair_accel == "bvh4obb.bezier4v" ) accels.add(device->bvh4_factory->BVH4OBBBezier4v(this));
    else if (device->hair_accel == "bvh4obb.bezier4i" ) accels.add(device->bvh4_factory->BVH4OBBBezier4i(this));
#if defined (EMBREE_TARGET_SIMD8)
    else if (device->hair_accel == "bvh8obb.bezier1v" ) accels.add(device->bvh8_factory->BVH8OBBBezier1v(this));
    else if (device->hair_accel == "bvh8obb.bezier1i" ) accels.add(device->bvh8_factory->BVH8OBBBezier1i(this));
    else if (device->hair_accel == "bvh8obb.bezier8v" ) accels.add(device->bvh8_factory->BVH8OBBBezier8v(this));
    else if (device->hair_accel == "bvh4obb.bezier8i" ) accels.add(device->bvh4_factory->BVH4OBBBezier8i(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown hair acceleration structure "+device->hair_accel);
#endif
  }

  void Scene::createHairMBAccel()
  {
#if defined(EMBREE_GEOMETRY_CURVES)
    if (device->hair_accel_mb == "default")
    {
#if defined (EMBREE_TARGET_SIMD8)
      if (device->hasISA(AVX2)) // only enable on HSW machines, on SNB this codepath is slower
      {
        accels.add(device->bvh4_factory->BVH4OBBBezier8iMB(this));
      }
      else
#endif
      {
        accels.add(device->bvh4_factory->BVH4OBBBezier4iMB(this));
      }
    }
    else if (device->hair_accel_mb == "bvh4.bezier1imb") accels.add(device->bvh4_factory->BVH4OBBBezier1iMB(this));
    else if (device->hair_accel_mb == "bvh4.bezier4imb") accels.add(device->bvh4_factory->BVH4OBBBezier4iMB(this));
#if defined (EMBREE_TARGET_SIMD8)
    else if (device->hair_accel_mb == "bvh8.bezier1imb") accels.add(device->bvh8_factory->BVH8OBBBezier1iMB(this));
    else if (device->hair_accel_mb == "bvh4.bezier8imb") accels.add(device->bvh4_factory->BVH4OBBBezier8iMB(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown motion blur hair acceleration structure "+device->hair_accel_mb);
#endif
  }

  void Scene::createLineAccel()
  {
#if defined(EMBREE_GEOMETRY_CURVES)
    if (device->line_accel == "default")
    {
      if (quality_flags != RTC_BUILD_QUALITY_LOW)
      {
#if defined (EMBREE_TARGET_SIMD8)
        if (device->hasISA(AVX) && !isCompactAccel())
          accels.add(device->bvh8_factory->BVH8Line4i(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Line4i(this,BVHFactory::BuildVariant::STATIC));
      }
      else
      {
        accels.add(device->bvh4_factory->BVH4Line4i(this,BVHFactory::BuildVariant::DYNAMIC));
      }
    }
    else if (device->line_accel == "bvh4.line4i") accels.add(device->bvh4_factory->BVH4Line4i(this));
#if defined (EMBREE_TARGET_SIMD8)
    else if (device->line_accel == "bvh8.line4i") accels.add(device->bvh8_factory->BVH8Line4i(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown line segment acceleration structure "+device->line_accel);
#endif
  }

  void Scene::createLineMBAccel()
  {
#if defined(EMBREE_GEOMETRY_CURVES)
    if (device->line_accel_mb == "default")
    {
#if defined (EMBREE_TARGET_SIMD8)
      if (device->hasISA(AVX) && !isCompactAccel())
        accels.add(device->bvh8_factory->BVH8Line4iMB(this));
      else
#endif
        accels.add(device->bvh4_factory->BVH4Line4iMB(this));
    }
    else if (device->line_accel_mb == "bvh4.line4imb") accels.add(device->bvh4_factory->BVH4Line4iMB(this));
#if defined (EMBREE_TARGET_SIMD8)
    else if (device->line_accel_mb == "bvh8.line4imb") accels.add(device->bvh8_factory->BVH8Line4iMB(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown motion blur line segment acceleration structure "+device->line_accel_mb);
#endif
  }

  void Scene::createSubdivAccel()
  {
#if defined(EMBREE_GEOMETRY_SUBDIVISION)
    if (device->subdiv_accel == "default") 
    {
      //if (quality_flags != RTC_BUILD_QUALITY_LOW)
        accels.add(device->bvh4_factory->BVH4SubdivPatch1Eager(this));
      //else
      //accels.add(device->bvh4_factory->BVH4SubdivPatch1(this,true));
    }
    else if (device->subdiv_accel == "bvh4.grid.eager" ) accels.add(device->bvh4_factory->BVH4SubdivPatch1Eager(this));
    else if (device->subdiv_accel == "bvh4.subdivpatch1eager" ) accels.add(device->bvh4_factory->BVH4SubdivPatch1Eager(this));
    //else if (device->subdiv_accel == "bvh4.subdivpatch1"      ) accels.add(device->bvh4_factory->BVH4SubdivPatch1(this,false));
    //else if (device->subdiv_accel == "bvh4.subdivpatch1cached") accels.add(device->bvh4_factory->BVH4SubdivPatch1(this,true));
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown subdiv accel "+device->subdiv_accel);
#endif
  }

  void Scene::createSubdivMBAccel()
  {
#if defined(EMBREE_GEOMETRY_SUBDIVISION)
    if (device->subdiv_accel_mb == "default") 
    {
      //if (quality_flags != RTC_BUILD_QUALITY_LOW)
      accels.add(device->bvh4_factory->BVH4SubdivPatch1EagerMB(this));
      //else
      //accels.add(device->bvh4_factory->BVH4SubdivPatch1MB(this,true));
    }
    // else if (device->subdiv_accel_mb == "bvh4.subdivpatch1"      ) accels.add(device->bvh4_factory->BVH4SubdivPatch1MB(this,false));
    // else if (device->subdiv_accel_mb == "bvh4.subdivpatch1cached") accels.add(device->bvh4_factory->BVH4SubdivPatch1MB(this,true));
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown subdiv mblur accel "+device->subdiv_accel_mb);
#endif
  }

  void Scene::createUserGeometryAccel()
  {
#if defined(EMBREE_GEOMETRY_USER)
    if (device->object_accel == "default") 
    {
#if defined (EMBREE_TARGET_SIMD8)
      if (device->hasISA(AVX) && !isCompactAccel())
      {
        //if (quality_flags != RTC_BUILD_QUALITY_LOW) {
        accels.add(device->bvh8_factory->BVH8UserGeometry(this,BVHFactory::BuildVariant::STATIC));
        //} else {
        //accels.add(device->bvh8_factory->BVH8UserGeometry(this,BVHFactory::BuildVariant::DYNAMIC));
        //}
      }
      else
#endif
      {
        //if (quality_flags != RTC_BUILD_QUALITY_LOW) {
        accels.add(device->bvh4_factory->BVH4UserGeometry(this,BVHFactory::BuildVariant::STATIC));
        //} else {
        //accels.add(device->bvh4_factory->BVH4UserGeometry(this,BVHFactory::BuildVariant::DYNAMIC)); // FIXME: only enable when memory consumption issue with instancing is solved
        //}
      }
    }
    else if (device->object_accel == "bvh4.object") accels.add(device->bvh4_factory->BVH4UserGeometry(this));
#if defined (EMBREE_TARGET_SIMD8)
    else if (device->object_accel == "bvh8.object") accels.add(device->bvh8_factory->BVH8UserGeometry(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown user geometry accel "+device->object_accel);
#endif
  }

  void Scene::createUserGeometryMBAccel()
  {
#if defined(EMBREE_GEOMETRY_USER)
    if (device->object_accel_mb == "default"    ) {
#if defined (EMBREE_TARGET_SIMD8)
      if (device->hasISA(AVX) && !isCompactAccel())
        accels.add(device->bvh8_factory->BVH8UserGeometryMB(this));
      else
#endif
        accels.add(device->bvh4_factory->BVH4UserGeometryMB(this));
    }
    else if (device->object_accel_mb == "bvh4.object") accels.add(device->bvh4_factory->BVH4UserGeometryMB(this));
#if defined (EMBREE_TARGET_SIMD8)
    else if (device->object_accel_mb == "bvh8.object") accels.add(device->bvh8_factory->BVH8UserGeometryMB(this));
#endif
    else throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown user geometry mblur accel "+device->object_accel_mb);
#endif
  }
  
  void Scene::clear() {
  }

  unsigned Scene::bind(unsigned geomID, Ref<Geometry> geometry) 
  {
    Lock<SpinLock> lock(geometriesMutex);
    if (geomID == RTC_INVALID_GEOMETRY_ID) {
      geomID = id_pool.allocate();
      if (geomID == RTC_INVALID_GEOMETRY_ID)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION,"too many geometries inside scene");
    }
    else
    {
      if (!id_pool.add(geomID))
        throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid geometry ID provided");
    }
    if (geomID >= geometries.size()) {
      geometries.resize(geomID+1);
      vertices.resize(geomID+1);
    }
    geometries[geomID] = geometry->attach(this,geomID);
    return geomID;
  }

  void Scene::detachGeometry(size_t geomID)
  {
    Lock<SpinLock> lock(geometriesMutex);
    
    if (geomID >= geometries.size())
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid geometry ID");

    Ref<Geometry>& geometry = geometries[geomID];
    if (geometry == null)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid geometry");
    
    geometry->detach();
    accels.deleteGeometry(unsigned(geomID));
    id_pool.deallocate((unsigned)geomID);
    geometries[geomID] = null;
    vertices[geomID] = nullptr;
  }

  void Scene::updateInterface()
  {
    /* update bounds */
    is_build = true;
    bounds = accels.bounds;
    intersectors = accels.intersectors;
  }

  void Scene::commit_task ()
  {
    /* print scene statistics */
    if (device->verbosity(2))
      printStatistics();

    progress_monitor_counter = 0;

    /* call preCommit function of each geometry */
    parallel_for(geometries.size(), [&] ( const size_t i ) {
        if (geometries[i] && geometries[i]->isEnabled())
          geometries[i]->preCommit();
      });

    /* select acceleration structures to build */
    if (flags_modified)
    {    
      accels.init();
      createTriangleAccel();
      createTriangleMBAccel();
      createQuadAccel();
      createQuadMBAccel();
      createSubdivAccel();
      createSubdivMBAccel();
      createHairAccel();
      createHairMBAccel();
      createLineAccel();
      createLineMBAccel();
      
#if defined(EMBREE_GEOMETRY_TRIANGLES)
      accels.add(device->bvh4_factory->BVH4InstancedBVH4Triangle4ObjectSplit(this));
#endif
      
      // has to be the last as the instID field of a hit instance is not invalidated by other hit geometry
      createUserGeometryAccel();
      createUserGeometryMBAccel();
      flags_modified = false;
    }
    
    /* select fast code path if no filter function is present */
    accels.select(hasFilterFunction());
  
    /* build all hierarchies of this scene */
    accels.build();

    /* make static geometry immutable */
    if (!isDynamicAccel()) {
      accels.immutable();
      flags_modified = true; // in non-dynamic mode we have to re-create accels
    }

    /* call postCommit function of each geometry */
    parallel_for(geometries.size(), [&] ( const size_t i ) {
        if (geometries[i] && geometries[i]->isEnabled())
          geometries[i]->postCommit();
      });
      
    updateInterface();

    if (device->verbosity(2)) {
      std::cout << "created scene intersector" << std::endl;
      accels.print(2);
      std::cout << "selected scene intersector" << std::endl;
      intersectors.print(2);
    }
    
    setModified(false);
  }

  void Scene::setBuildQuality(RTCBuildQuality quality_flags_i)
  {
    if (quality_flags == quality_flags_i) return;
    quality_flags = quality_flags_i;
    flags_modified = true;
  }

  RTCBuildQuality Scene::getBuildQuality() const {
    return quality_flags;
  }

  void Scene::setSceneFlags(RTCSceneFlags scene_flags_i)
  {
    if (scene_flags == scene_flags_i) return;
    scene_flags = scene_flags_i;
    flags_modified = true;
  }

  RTCSceneFlags Scene::getSceneFlags() const {
    return scene_flags;
  }
                   
#if defined(TASKING_INTERNAL)

  void Scene::commit (bool join) 
  {
    Lock<MutexSys> buildLock(buildMutex,false);

    /* allocates own taskscheduler for each build */
    Ref<TaskScheduler> scheduler = nullptr;
    { 
      Lock<MutexSys> lock(schedulerMutex);
      scheduler = this->scheduler;
      if (scheduler == null) {
        buildLock.lock();
        this->scheduler = scheduler = new TaskScheduler;
      }
    }

    /* worker threads join build */
    if (!buildLock.isLocked())
    {
      if (!join) 
        throw_RTCError(RTC_ERROR_INVALID_OPERATION,"use rtcJoinCommitScene to join a build operation");
      
      scheduler->join();
      return;
    }

    /* fast path for unchanged scenes */
    if (!isModified()) {
      scheduler->spawn_root([&]() { Lock<MutexSys> lock(schedulerMutex); this->scheduler = nullptr; }, 1, !join);
      return;
    }

    /* initiate build */
    try {
      scheduler->spawn_root([&]() { commit_task(); Lock<MutexSys> lock(schedulerMutex); this->scheduler = nullptr; }, 1, !join);
    }
    catch (...) {
      accels.clear();
      updateInterface();
      Lock<MutexSys> lock(schedulerMutex);
      this->scheduler = nullptr;
      throw;
    }
  }

#endif

#if defined(TASKING_TBB) || defined(TASKING_PPL)

  void Scene::commit (bool join) 
  {
#if defined(TASKING_PPL)
    if (join)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcJoinCommitScene not supported with PPL");
#endif
    
#if defined(TASKING_TBB) && (TBB_INTERFACE_VERSION_MAJOR < 8)
    if (join)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcJoinCommitScene not supported with this TBB version");
#endif

    /* try to obtain build lock */
    Lock<MutexSys> lock(buildMutex,buildMutex.try_lock());

    /* join hierarchy build */
    if (!lock.isLocked())
    {
      if (!join) 
        throw_RTCError(RTC_ERROR_INVALID_OPERATION,"use rtcJoinCommitScene to join a build operation");
      
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
      return;
    }

    /* for best performance set FTZ and DAZ flags in the MXCSR control and status register */
    unsigned int mxcsr = _mm_getcsr();
    _mm_setcsr(mxcsr | /* FTZ */ (1<<15) | /* DAZ */ (1<<6));
    
    try {
#if defined(TASKING_TBB)
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
              tbb::parallel_for (size_t(0), size_t(1), size_t(1), [&] (size_t) { commit_task(); }, ctx);
            });
          group->wait();
#if USE_TASK_ARENA
        }); 
#endif
     
      /* reset MXCSR register again */
      _mm_setcsr(mxcsr);
#else
      group->run([&]{
          concurrency::parallel_for(size_t(0), size_t(1), size_t(1), [&](size_t) { commit_task(); });
        });
      group->wait();

#endif
    } 
    catch (...) {

      /* reset MXCSR register again */
      _mm_setcsr(mxcsr);
      
      accels.clear();
      updateInterface();
      throw;
    }
  }
#endif

  void Scene::setProgressMonitorFunction(RTCProgressMonitorFunction func, void* ptr) 
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
      size_t n = size_t(dn) + progress_monitor_counter.fetch_add(size_t(dn));
      if (!progress_monitor_function(progress_monitor_ptr, n / (double(numPrimitives())))) {
        throw_RTCError(RTC_ERROR_CANCELLED,"progress monitor forced termination");
      }
    }
  }
}
