// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
#include "../../include/embree2/rtcore_ray.h"

#include "../bvh/bvh4_factory.h"
#include "../bvh/bvh8_factory.h"
 
namespace embree
{
namespace isa
{
  /* error raising rtcIntersect and rtcOccluded functions */
  void missing_rtcCommit()      { throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed"); }
  void invalid_rtcIntersect1()  { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect and rtcOccluded not enabled"); }
  void invalid_rtcIntersect4()  { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect4 and rtcOccluded4 not enabled"); }
  void invalid_rtcIntersect8()  { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect8 and rtcOccluded8 not enabled"); }
  void invalid_rtcIntersect16() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect16 and rtcOccluded16 not enabled"); }
  void invalid_rtcIntersectN()  { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersectN and rtcOccludedN not enabled"); }

  Scene::Scene (Device* device, RTCSceneFlags sflags, RTCAlgorithmFlags aflags)
    : Accel(AccelData::TY_UNKNOWN),
      device(device), 
      commitCounterSubdiv(0), 
      numMappedBuffers(0),
      flags(sflags), aflags(aflags), 
      needTriangleIndices(false), needTriangleVertices(false), 
      needQuadIndices(false), needQuadVertices(false), 
      needBezierIndices(false), needBezierVertices(false),
      needLineIndices(false), needLineVertices(false),
      needSubdivIndices(false), needSubdivVertices(false),
      is_build(false), modified(true),
      progressInterface(this), progress_monitor_function(nullptr), progress_monitor_ptr(nullptr), progress_monitor_counter(0), 
      numIntersectionFilters1(0), numIntersectionFilters4(0), numIntersectionFilters8(0), numIntersectionFilters16(0), numIntersectionFiltersN(0)
  {
#if defined(TASKING_INTERNAL) 
    scheduler = nullptr;
#elif defined(TASKING_TBB)
    group = new tbb::task_group;
#elif defined(TASKING_PPL)
    group = new concurrency::task_group;
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
  }

  void Scene::createTriangleAccel()
  {
#if defined(EMBREE_GEOMETRY_TRIANGLES)
    if (device->tri_accel == "default") 
    {
      if (isStatic()) {
        int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
        switch (mode) {
        case /*0b00*/ 0: 
#if defined (__AVX__)
          if (device->hasISA(AVX))
	  {
            if (isHighQuality()) 
              accels.add(device->bvh8_factory->BVH8Triangle4(this,BVH8Factory::BuildVariant::HIGH_QUALITY,BVH8Factory::IntersectVariant::FAST)); 
            else
              accels.add(device->bvh8_factory->BVH8Triangle4(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::FAST));
          }
          else 
#endif
          { 
            if (isHighQuality()) 
              accels.add(device->bvh4_factory->BVH4Triangle4(this,BVH4Factory::BuildVariant::HIGH_QUALITY,BVH4Factory::IntersectVariant::FAST));
            else 
              accels.add(device->bvh4_factory->BVH4Triangle4(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST));
          }
          break;

        case /*0b01*/ 1: 
#if defined (__AVX__)
          if (device->hasISA(AVX)) 
            accels.add(device->bvh8_factory->BVH8Triangle4v(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::ROBUST)); 
          else
#endif
            accels.add(device->bvh4_factory->BVH4Triangle4v(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST)); 

          break;
        case /*0b10*/ 2: 
#if defined (__AVX__)
          if (device->hasISA(AVX)) 
            accels.add(device->bvh8_factory->BVH8Triangle4i(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::FAST  )); 
          else
#endif
            accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST  )); 
          break;
        case /*0b11*/ 3: 
#if defined (__AVX__)
          if (device->hasISA(AVX)) 
            accels.add(device->bvh8_factory->BVH8Triangle4i(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::ROBUST)); 
          else
#endif
            accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST)); 
          break;
        }
      }
      else /* dynamic */
      {
#if defined (__AVX__)
          if (device->hasISA(AVX))
	  {
            int mode =  2*(int)isCompact() + 1*(int)isRobust();
            switch (mode) {
            case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8Triangle4 (this,BVH8Factory::BuildVariant::DYNAMIC,BVH8Factory::IntersectVariant::FAST  )); break;
            case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8Triangle4v(this,BVH8Factory::BuildVariant::DYNAMIC,BVH8Factory::IntersectVariant::ROBUST)); break; 
            case /*0b10*/ 2: accels.add(device->bvh8_factory->BVH8Triangle4i(this,BVH8Factory::BuildVariant::DYNAMIC,BVH8Factory::IntersectVariant::FAST  )); break;
            case /*0b11*/ 3: accels.add(device->bvh8_factory->BVH8Triangle4i(this,BVH8Factory::BuildVariant::DYNAMIC,BVH8Factory::IntersectVariant::ROBUST)); break;
            }
          }
          else
#endif
          {
            int mode =  2*(int)isCompact() + 1*(int)isRobust();
            switch (mode) {
            case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4Triangle4 (this,BVH4Factory::BuildVariant::DYNAMIC,BVH4Factory::IntersectVariant::FAST  )); break;
            case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Triangle4v(this,BVH4Factory::BuildVariant::DYNAMIC,BVH4Factory::IntersectVariant::ROBUST)); break;
            case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVH4Factory::BuildVariant::DYNAMIC,BVH4Factory::IntersectVariant::FAST  )); break;
            case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4i(this,BVH4Factory::BuildVariant::DYNAMIC,BVH4Factory::IntersectVariant::ROBUST)); break;
            }
          }
      }
    }
    else if (device->tri_accel == "bvh4.triangle4")       accels.add(device->bvh4_factory->BVH4Triangle4 (this));
    else if (device->tri_accel == "bvh4.triangle4v")      accels.add(device->bvh4_factory->BVH4Triangle4v(this));
    else if (device->tri_accel == "bvh4.triangle4i")      accels.add(device->bvh4_factory->BVH4Triangle4i(this));
    else if (device->tri_accel == "qbvh4.triangle4i")     accels.add(device->bvh4_factory->BVH4QuantizedTriangle4i(this));

#if defined (__AVX__)
    else if (device->tri_accel == "bvh8.triangle4")       accels.add(device->bvh8_factory->BVH8Triangle4 (this));
    else if (device->tri_accel == "bvh8.triangle4i")      accels.add(device->bvh8_factory->BVH8Triangle4i(this));
    else if (device->tri_accel == "qbvh8.triangle4i")     accels.add(device->bvh8_factory->BVH8QuantizedTriangle4i(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown triangle acceleration structure "+device->tri_accel);
#endif
  }

  void Scene::createTriangleMBAccel()
  {
#if defined(EMBREE_GEOMETRY_TRIANGLES)
    if (device->tri_accel_mb == "default")
    {
      int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
      
#if defined (__AVX__)
      if (device->hasISA(AVX2)) // BVH8 reduces performance on AVX only-machines
      {
        switch (mode) {
        case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8Triangle4iMB(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::FAST  )); break;
        case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8Triangle4iMB(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::ROBUST)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST  )); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST)); break;
        }
      }
      else
#endif
      {
        switch (mode) {
        case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST  )); break;
        case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST  )); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST)); break;
        }
      }
    }
    else if (device->tri_accel_mb == "bvh4.triangle4vmb") accels.add(device->bvh4_factory->BVH4Triangle4vMB(this));
    else if (device->tri_accel_mb == "bvh4.triangle4imb") accels.add(device->bvh4_factory->BVH4Triangle4iMB(this));
#if defined (__AVX__)
    else if (device->tri_accel_mb == "bvh8.triangle4vmb") accels.add(device->bvh8_factory->BVH8Triangle4vMB(this));
    else if (device->tri_accel_mb == "bvh8.triangle4imb") accels.add(device->bvh8_factory->BVH8Triangle4iMB(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown motion blur triangle acceleration structure "+device->tri_accel_mb);
#endif
  }

  void Scene::createQuadAccel()
  {
#if defined(EMBREE_GEOMETRY_QUADS)
    if (device->quad_accel == "default") 
    {
      if (isStatic())
      {
        /* static */
        int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
        switch (mode) {
        case /*0b00*/ 0:
#if defined (__AVX__)
          if (device->hasISA(AVX))
          {
            if (isHighQuality()) 
              accels.add(device->bvh8_factory->BVH8Quad4v(this,BVH8Factory::BuildVariant::HIGH_QUALITY,BVH8Factory::IntersectVariant::FAST));
            else
              accels.add(device->bvh8_factory->BVH8Quad4v(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::FAST));
          }
          else
#endif
          {
            if (isHighQuality()) 
              accels.add(device->bvh4_factory->BVH4Quad4v(this,BVH4Factory::BuildVariant::HIGH_QUALITY,BVH4Factory::IntersectVariant::FAST));
            else
              accels.add(device->bvh4_factory->BVH4Quad4v(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST));
          }
          break;

        case /*0b01*/ 1:
#if defined (__AVX__)
          if (device->hasISA(AVX))
            accels.add(device->bvh8_factory->BVH8Quad4v(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::ROBUST));
          else
#endif
            accels.add(device->bvh4_factory->BVH4Quad4v(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST));
          break;

        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Quad4i(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST)); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4i(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST)); break;
        }
      }
      else /* dynamic */
      {
#if defined (__AVX__)
          if (device->hasISA(AVX))
	  {
            int mode =  2*(int)isCompact() + 1*(int)isRobust();
            switch (mode) {
            case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8Quad4v(this,BVH8Factory::BuildVariant::DYNAMIC,BVH8Factory::IntersectVariant::FAST)); break;
            case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8Quad4v(this,BVH8Factory::BuildVariant::DYNAMIC,BVH8Factory::IntersectVariant::ROBUST)); break; 
            case /*0b10*/ 2: accels.add(device->bvh8_factory->BVH8Quad4v(this,BVH8Factory::BuildVariant::DYNAMIC,BVH8Factory::IntersectVariant::FAST)); break;
            case /*0b11*/ 3: accels.add(device->bvh8_factory->BVH8Quad4v(this,BVH8Factory::BuildVariant::DYNAMIC,BVH8Factory::IntersectVariant::ROBUST)); break;
            }
          }
          else
#endif
          {
            int mode =  2*(int)isCompact() + 1*(int)isRobust();
            switch (mode) {
            case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4Quad4v(this,BVH4Factory::BuildVariant::DYNAMIC,BVH4Factory::IntersectVariant::FAST)); break;
            case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Quad4v(this,BVH4Factory::BuildVariant::DYNAMIC,BVH4Factory::IntersectVariant::ROBUST)); break; 
            case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Quad4v(this,BVH4Factory::BuildVariant::DYNAMIC,BVH4Factory::IntersectVariant::FAST)); break;
            case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4v(this,BVH4Factory::BuildVariant::DYNAMIC,BVH4Factory::IntersectVariant::ROBUST)); break;
            }
          }
      }
    }
    else if (device->quad_accel == "bvh4.quad4v")       accels.add(device->bvh4_factory->BVH4Quad4v(this));
    else if (device->quad_accel == "bvh4.quad4i")       accels.add(device->bvh4_factory->BVH4Quad4i(this));
    else if (device->quad_accel == "qbvh4.quad4i")      accels.add(device->bvh4_factory->BVH4QuantizedQuad4i(this));

#if defined (__AVX__)
    else if (device->quad_accel == "bvh8.quad4v")       accels.add(device->bvh8_factory->BVH8Quad4v(this));
    else if (device->quad_accel == "bvh8.quad4i")       accels.add(device->bvh8_factory->BVH8Quad4i(this));
    else if (device->quad_accel == "qbvh8.quad4i")      accels.add(device->bvh8_factory->BVH8QuantizedQuad4i(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown quad acceleration structure "+device->quad_accel);
#endif
  }

  void Scene::createQuadMBAccel()
  {
#if defined(EMBREE_GEOMETRY_QUADS)
    if (device->quad_accel_mb == "default") 
    {
      int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
      switch (mode) {
      case /*0b00*/ 0:
#if defined (__AVX__)
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8Quad4iMB(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::FAST));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Quad4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST));
        break;

      case /*0b01*/ 1:
#if defined (__AVX__)
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8Quad4iMB(this,BVH8Factory::BuildVariant::STATIC,BVH8Factory::IntersectVariant::ROBUST));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Quad4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST));
        break;

      case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Quad4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::FAST  )); break;
      case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4iMB(this,BVH4Factory::BuildVariant::STATIC,BVH4Factory::IntersectVariant::ROBUST)); break;
      }
    }
    else if (device->quad_accel_mb == "bvh4.quad4imb") accels.add(device->bvh4_factory->BVH4Quad4iMB(this));
#if defined (__AVX__)
    else if (device->quad_accel_mb == "bvh8.quad4imb") accels.add(device->bvh8_factory->BVH8Quad4iMB(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown quad acceleration structure "+device->quad_accel_mb);
#endif
  }

  void Scene::createHairAccel()
  {
#if defined(EMBREE_GEOMETRY_HAIR)
    if (device->hair_accel == "default")
    {
      int mode = 2*(int)isCompact() + 1*(int)isRobust();
      if (isStatic())
      {
#if defined (__AVX__)
        if (device->hasISA(AVX2)) // only enable on HSW machines, for SNB this codepath is slower
        {
          switch (mode) {
          case /*0b00*/ 0: accels.add(device->bvh8_factory->BVH8OBBBezier1v(this)); break;
          case /*0b01*/ 1: accels.add(device->bvh8_factory->BVH8OBBBezier1v(this)); break;
          case /*0b10*/ 2: accels.add(device->bvh8_factory->BVH8OBBBezier1i(this)); break;
          case /*0b11*/ 3: accels.add(device->bvh8_factory->BVH8OBBBezier1i(this)); break;
          }
        }
        else
#endif
        {
          switch (mode) {
          case /*0b00*/ 0: accels.add(device->bvh4_factory->BVH4OBBBezier1v(this)); break;
          case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4OBBBezier1v(this)); break;
          case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4OBBBezier1i(this)); break;
          case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4OBBBezier1i(this)); break;
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
    else if (device->hair_accel == "bvh4obb.bezier1v" ) accels.add(device->bvh4_factory->BVH4OBBBezier1v(this));
    else if (device->hair_accel == "bvh4obb.bezier1i" ) accels.add(device->bvh4_factory->BVH4OBBBezier1i(this));
#if defined (__AVX__)
    else if (device->hair_accel == "bvh8obb.bezier1v" ) accels.add(device->bvh8_factory->BVH8OBBBezier1v(this));
    else if (device->hair_accel == "bvh8obb.bezier1i" ) accels.add(device->bvh8_factory->BVH8OBBBezier1i(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown hair acceleration structure "+device->hair_accel);
#endif
  }

  void Scene::createHairMBAccel()
  {
#if defined(EMBREE_GEOMETRY_HAIR)
    if (device->hair_accel_mb == "default")
    {
#if defined (__AVX__)
      if (device->hasISA(AVX2)) // only enable on HSW machines, on SNB this codepath is slower
      {
        accels.add(device->bvh8_factory->BVH8OBBBezier1iMB(this));
      }
      else
#endif
      {
        accels.add(device->bvh4_factory->BVH4OBBBezier1iMB(this));
      }
    }
    else if (device->hair_accel_mb == "bvh4obb.bezier1imb") accels.add(device->bvh4_factory->BVH4OBBBezier1iMB(this));
#if defined (__AVX__)
    else if (device->hair_accel_mb == "bvh8obb.bezier1imb") accels.add(device->bvh8_factory->BVH8OBBBezier1iMB(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown motion blur hair acceleration structure "+device->tri_accel_mb);
#endif
  }

  void Scene::createLineAccel()
  {
#if defined(EMBREE_GEOMETRY_LINES)
    if (device->line_accel == "default")
    {
      if (isStatic())
      {
#if defined (__AVX__)
        if (device->hasISA(AVX) && !isCompact())
          accels.add(device->bvh8_factory->BVH8Line4i(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Line4i(this,BVH4Factory::BuildVariant::STATIC));
      }
      else
      {
        accels.add(device->bvh4_factory->BVH4Line4i(this,BVH4Factory::BuildVariant::DYNAMIC));
      }
    }
    else if (device->line_accel == "bvh4.line4i") accels.add(device->bvh4_factory->BVH4Line4i(this));
#if defined (__AVX__)
    else if (device->line_accel == "bvh8.line4i") accels.add(device->bvh8_factory->BVH8Line4i(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown line segment acceleration structure "+device->line_accel);
#endif
  }

  void Scene::createLineMBAccel()
  {
#if defined(EMBREE_GEOMETRY_LINES)
    if (device->line_accel_mb == "default")
    {
#if defined (__AVX__)
      if (device->hasISA(AVX) && !isCompact())
        accels.add(device->bvh8_factory->BVH8Line4iMB(this));
      else
#endif
        accels.add(device->bvh4_factory->BVH4Line4iMB(this));
    }
    else if (device->line_accel_mb == "bvh4.line4imb") accels.add(device->bvh4_factory->BVH4Line4iMB(this));
#if defined (__AVX__)
    else if (device->line_accel_mb == "bvh8.line4imb") accels.add(device->bvh8_factory->BVH8Line4iMB(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown motion blur line segment acceleration structure "+device->line_accel_mb);
#endif
  }

  void Scene::createSubdivAccel()
  {
#if defined(EMBREE_GEOMETRY_SUBDIV)
    if (device->subdiv_accel == "default") 
    {
      if (isIncoherent(flags) && isStatic())
        accels.add(device->bvh4_factory->BVH4SubdivPatch1Eager(this));
      else
        accels.add(device->bvh4_factory->BVH4SubdivPatch1(this,true));
    }
    else if (device->subdiv_accel == "bvh4.subdivpatch1eager" ) accels.add(device->bvh4_factory->BVH4SubdivPatch1Eager(this));
    else if (device->subdiv_accel == "bvh4.subdivpatch1"      ) accels.add(device->bvh4_factory->BVH4SubdivPatch1(this,false));
    else if (device->subdiv_accel == "bvh4.subdivpatch1cached") accels.add(device->bvh4_factory->BVH4SubdivPatch1(this,true));
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown subdiv accel "+device->subdiv_accel);
#endif
  }

  void Scene::createSubdivMBAccel()
  {
#if defined(EMBREE_GEOMETRY_SUBDIV)
    if (device->subdiv_accel_mb == "default") 
    {
      if (isIncoherent(flags) && isStatic())
        accels.add(device->bvh4_factory->BVH4SubdivPatch1MB(this,false));
      else
        accels.add(device->bvh4_factory->BVH4SubdivPatch1MB(this,true));
    }
    else if (device->subdiv_accel_mb == "bvh4.subdivpatch1"      ) accels.add(device->bvh4_factory->BVH4SubdivPatch1MB(this,false));
    else if (device->subdiv_accel_mb == "bvh4.subdivpatch1cached") accels.add(device->bvh4_factory->BVH4SubdivPatch1MB(this,true));
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown subdiv mblur accel "+device->subdiv_accel_mb);
#endif
  }

  void Scene::createUserGeometryAccel()
  {
#if defined(EMBREE_GEOMETRY_USER)
    if (device->object_accel == "default") 
    {
      if (isStatic()) {
        accels.add(device->bvh4_factory->BVH4UserGeometry(this,BVH4Factory::BuildVariant::STATIC));
      } else {
        accels.add(device->bvh4_factory->BVH4UserGeometry(this,BVH4Factory::BuildVariant::DYNAMIC));
      }
    }
    else if (device->object_accel == "bvh4.object") accels.add(device->bvh4_factory->BVH4UserGeometry(this)); 
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown user geometry accel "+device->object_accel);
#endif
  }

  void Scene::createUserGeometryMBAccel()
  {
#if defined(EMBREE_GEOMETRY_USER)
    accels.add(device->bvh4_factory->BVH4UserGeometryMB(this));
#endif
  }
  
  Scene::~Scene () 
  {
    for (size_t i=0; i<geometries.size(); i++)
      delete geometries[i];

#if defined(TASKING_TBB) || defined(TASKING_PPL)
    delete group; group = nullptr;
#endif
  }

  void Scene::clear() {
  }

  unsigned Scene::newUserGeometry (RTCGeometryFlags gflags, size_t items, size_t numTimeSteps) 
  {
#if defined(EMBREE_GEOMETRY_USER)
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > RTC_MAX_TIME_STEPS) {
      throw_RTCError(RTC_INVALID_OPERATION,"maximal number of timesteps exceeded");
      return -1;
    }

    return add(new UserGeometry(this,gflags,items,numTimeSteps));
#else
    return -1;
#endif
  }

  unsigned Scene::newInstance (SceneInterface* scene, size_t numTimeSteps) 
  {
#if defined(EMBREE_GEOMETRY_USER)
    if (numTimeSteps == 0 || numTimeSteps > RTC_MAX_TIME_STEPS) {
      throw_RTCError(RTC_INVALID_OPERATION,"maximal number of timesteps exceeded");
      return -1;
    }

    return add(Instance::create(this,dynamic_cast<Scene*>(scene),numTimeSteps));
#else
    return -1;
#endif
  }

  unsigned Scene::newGeometryInstance (Geometry* geom_in) 
  {
    Geometry* geom = new GeometryInstance(this,geom_in);
    unsigned id = add(geom);
    geom->id = id;
    return id;
  }

  unsigned Scene::newGeometryInstance (unsigned geomID) {
    return newGeometryInstance(get_locked(geomID));
  }

  unsigned int Scene::newGeometryGroup (RTCGeometryFlags gflags, const std::vector<Geometry*> geometries)
  {
    Geometry* geom = new GeometryGroup(this,gflags,geometries);
    unsigned id = add(geom);
    geom->id = id;
    return id;
  }

  unsigned int Scene::newGeometryGroup (RTCGeometryFlags gflags, unsigned* geomIDs, size_t N)
  {
    std::vector<Geometry*> geometries(N);
    for (size_t i=0; i<N; i++) {
      geometries[i] = get_locked(geomIDs[i]);
      if (geometries[i]->getType() == Geometry::GROUP)
        throw_RTCError(RTC_INVALID_ARGUMENT,"geometry groups cannot contain other geometry groups");
      if (geometries[i]->getType() != geometries[0]->getType())
        throw_RTCError(RTC_INVALID_ARGUMENT,"geometries inside group have to be of same type");
    }
    return newGeometryGroup(gflags,geometries);
  }

  unsigned Scene::newTriangleMesh (RTCGeometryFlags gflags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) 
  {
#if defined(EMBREE_GEOMETRY_TRIANGLES)

    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > RTC_MAX_TIME_STEPS) {
      throw_RTCError(RTC_INVALID_OPERATION,"maximal number of timesteps exceeded");
      return -1;
    }
    
    return add(new TriangleMesh(this,gflags,numTriangles,numVertices,numTimeSteps));
#else
    return -1;
#endif
  }

  unsigned Scene::newQuadMesh (RTCGeometryFlags gflags, size_t numQuads, size_t numVertices, size_t numTimeSteps) 
  {
#if defined(EMBREE_GEOMETRY_QUADS)

    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > RTC_MAX_TIME_STEPS) {
      throw_RTCError(RTC_INVALID_OPERATION,"maximal number of timesteps exceeded");
      return -1;
    }
    
    return add(new QuadMesh(this,gflags,numQuads,numVertices,numTimeSteps));
#else
    return -1;
#endif
  }

  unsigned Scene::newSubdivisionMesh (RTCGeometryFlags gflags, size_t numFaces, size_t numEdges, size_t numVertices, size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps) 
  {
#if defined(EMBREE_GEOMETRY_SUBDIV)

    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > RTC_MAX_TIME_STEPS) {
      throw_RTCError(RTC_INVALID_OPERATION,"maximal number of timesteps exceeded");
      return -1;
    }

#if defined(__AVX__)
    if (device->hasISA(AVX))
      return add(new SubdivMeshAVX(this,gflags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps));
    else 
#endif
      return add(new SubdivMesh(this,gflags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps));
#else
    return -1;
#endif
  }

  unsigned Scene::newCurves (int subtype_i, int basis_i, RTCGeometryFlags gflags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    NativeCurves::SubType subtype = (NativeCurves::SubType) subtype_i;
    NativeCurves::Basis basis = (NativeCurves::Basis) basis_i;
    
#if defined(EMBREE_GEOMETRY_HAIR)

    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > RTC_MAX_TIME_STEPS) {
      throw_RTCError(RTC_INVALID_OPERATION,"maximal number of timesteps exceeded");
      return -1;
    }
    
    Geometry* geom = nullptr;
#if defined(EMBREE_NATIVE_CURVE_BSPLINE)
    switch (basis) {
    case NativeCurves::BEZIER : geom = new CurvesBezier(this,subtype,basis,gflags,numCurves,numVertices,numTimeSteps); break;
    case NativeCurves::BSPLINE: geom = new NativeCurves(this,subtype,basis,gflags,numCurves,numVertices,numTimeSteps); break;
    }
#else
    switch (basis) {
    case NativeCurves::BEZIER : geom = new NativeCurves (this,subtype,basis,gflags,numCurves,numVertices,numTimeSteps); break;
    case NativeCurves::BSPLINE: geom = new CurvesBSpline(this,subtype,basis,gflags,numCurves,numVertices,numTimeSteps); break;
    }
#endif
    return add(geom);
#else
    return -1;
#endif
  }

  unsigned Scene::newLineSegments (RTCGeometryFlags gflags, size_t numSegments, size_t numVertices, size_t numTimeSteps)
  {
#if defined(EMBREE_GEOMETRY_LINES)

    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > RTC_MAX_TIME_STEPS) {
      throw_RTCError(RTC_INVALID_OPERATION,"maximal number of timesteps exceeded");
      return -1;
    }

    return add(new LineSegments(this,gflags,numSegments,numVertices,numTimeSteps));
#else
    return -1;
#endif
  }

  unsigned Scene::add(Geometry* geometry) 
  {
    Lock<SpinLock> lock(geometriesMutex);
    unsigned id = id_pool.allocate();
    if (id >= geometries.size()) 
      geometries.resize(id+1);
    geometries[id] = geometry;
    geometry->id = id;
    return id;
  }

  void Scene::deleteGeometry(size_t geomID)
  {
    Lock<SpinLock> lock(geometriesMutex);
    
    if (isStatic())
      throw_RTCError(RTC_INVALID_OPERATION,"rtcDeleteGeometry cannot get called in static scenes");
    if (geomID >= geometries.size())
      throw_RTCError(RTC_INVALID_OPERATION,"invalid geometry ID");

    Geometry* geometry = geometries[geomID];
    if (geometry == nullptr)
      throw_RTCError(RTC_INVALID_OPERATION,"invalid geometry");
    
    geometry->disable();
    accels.deleteGeometry(unsigned(geomID));
    id_pool.deallocate((unsigned)geomID);
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
    if ((aflags & RTC_INTERSECT_STREAM) == 0) 
    {
      intersectors.intersectorN = Accel::IntersectorN(&invalid_rtcIntersectN);
      if ((aflags & RTC_INTERSECT1) == 0) intersectors.intersector1 = Accel::Intersector1(&invalid_rtcIntersect1);
      if ((aflags & RTC_INTERSECT4) == 0) intersectors.intersector4 = Accel::Intersector4(&invalid_rtcIntersect4);
      if ((aflags & RTC_INTERSECT8) == 0) intersectors.intersector8 = Accel::Intersector8(&invalid_rtcIntersect8);
      if ((aflags & RTC_INTERSECT16) == 0) intersectors.intersector16 = Accel::Intersector16(&invalid_rtcIntersect16);
    }
  }

  void Scene::commit_task ()
  {
    progress_monitor_counter = 0;

    /* call preCommit function of each geometry */
    parallel_for(geometries.size(), [&] ( const size_t i ) {
        if (geometries[i]) geometries[i]->preCommit();
      });

    /* select fast code path if no intersection filter is present */
    accels.select(numIntersectionFiltersN+numIntersectionFilters4,
                  numIntersectionFiltersN+numIntersectionFilters8,
                  numIntersectionFiltersN+numIntersectionFilters16,
                  numIntersectionFiltersN);
  
    /* build all hierarchies of this scene */
    accels.build();

    /* make static geometry immutable */
    if (isStatic()) accels.immutable();

    /* call postCommit function of each geometry */
    parallel_for(geometries.size(), [&] ( const size_t i ) {
        if (geometries[i]) geometries[i]->postCommit();
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

#if defined(TASKING_INTERNAL)

  void Scene::commit (size_t threadIndex, size_t threadCount, bool useThreadPool) 
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
    if (!buildLock.isLocked()) {
      scheduler->join();
      return;
    }

    /* wait for all threads in rtcCommitThread mode */
    if (threadCount != 0)
      scheduler->wait_for_threads(threadCount);

    /* fast path for unchanged scenes */
    if (!isModified()) {
      scheduler->spawn_root([&]() { this->scheduler = nullptr; }, 1, useThreadPool);
      return;
    }

    /* report error if scene not ready */
    if (!ready()) {
      scheduler->spawn_root([&]() { this->scheduler = nullptr; }, 1, useThreadPool);
      throw_RTCError(RTC_INVALID_OPERATION,"not all buffers are unmapped");
    }

    /* initiate build */
    try {
      scheduler->spawn_root([&]() { commit_task(); this->scheduler = nullptr; }, 1, useThreadPool);
    }
    catch (...) {
      accels.clear();
      updateInterface();
      throw;
    }
  }

#endif

#if defined(TASKING_TBB) || defined(TASKING_PPL)

  void Scene::commit (size_t threadIndex, size_t threadCount, bool useThreadPool) 
  {
    /* let threads wait for build to finish in rtcCommitThread mode */
    if (threadCount != 0) {
#if defined(TASKING_TBB) && (TBB_INTERFACE_VERSION_MAJOR < 8)
      throw_RTCError(RTC_INVALID_OPERATION,"rtcCommitThread not supported");
#endif
      if (threadIndex > 0) {
        group_barrier.wait(threadCount); // FIXME: use barrier that waits in condition
        group->wait();
        return;
      }
    }

    /* try to obtain build lock */
    Lock<MutexSys> lock(buildMutex,buildMutex.try_lock());

    /* join hierarchy build */
    if (!lock.isLocked()) {
#if defined(TASKING_TBB) && (TBB_INTERFACE_VERSION_MAJOR < 8)
      throw_RTCError(RTC_INVALID_OPERATION,"join not supported");
#endif
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
          if (threadCount) group_barrier.wait(threadCount);
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
      if (threadCount) group_barrier.wait(threadCount);
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

  void Scene::setProgressMonitorFunction(RTCProgressMonitorFunc func, void* ptr) 
  {
    progress_monitor_function = func;
    progress_monitor_ptr      = ptr;
  }

  void Scene::progressMonitor(double dn)
  {
    if (progress_monitor_function) {
      size_t n = size_t(dn) + progress_monitor_counter.fetch_add(size_t(dn));
      if (!progress_monitor_function(progress_monitor_ptr, n / (double(numPrimitives())))) {
        throw_RTCError(RTC_CANCELLED,"progress monitor forced termination");
      }
    }
  }

  DeviceInterface* Scene::getDevice() {
    return device;
  }

  GeometryInterface* Scene::getGeometry (unsigned geomID) {
    return get(geomID);
  }

  GeometryInterface* Scene::getGeometryLocked (unsigned geomID) {
    return get_locked(geomID);
  }

  void Scene::rtcGetBounds(RTCBounds& bounds_o)
  {
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    BBox3fa bounds = this->bounds.bounds();
    bounds_o.lower_x = bounds.lower.x;
    bounds_o.lower_y = bounds.lower.y;
    bounds_o.lower_z = bounds.lower.z;
    bounds_o.align0  = 0;
    bounds_o.upper_x = bounds.upper.x;
    bounds_o.upper_y = bounds.upper.y;
    bounds_o.upper_z = bounds.upper.z;
    bounds_o.align1  = 0;
  }

  void Scene::rtcGetLinearBounds(RTCBounds* bounds_o)
  {
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    bounds_o[0].lower_x = bounds.bounds0.lower.x;
    bounds_o[0].lower_y = bounds.bounds0.lower.y;
    bounds_o[0].lower_z = bounds.bounds0.lower.z;
    bounds_o[0].align0  = 0;
    bounds_o[0].upper_x = bounds.bounds0.upper.x;
    bounds_o[0].upper_y = bounds.bounds0.upper.y;
    bounds_o[0].upper_z = bounds.bounds0.upper.z;
    bounds_o[0].align1  = 0;
    bounds_o[1].lower_x = bounds.bounds1.lower.x;
    bounds_o[1].lower_y = bounds.bounds1.lower.y;
    bounds_o[1].lower_z = bounds.bounds1.lower.z;
    bounds_o[1].align0  = 0;
    bounds_o[1].upper_x = bounds.bounds1.upper.x;
    bounds_o[1].upper_y = bounds.bounds1.upper.y;
    bounds_o[1].upper_z = bounds.bounds1.upper.z;
    bounds_o[1].align1  = 0;
  }
  
  void Scene::rtcIntersect (RTCRay& ray) 
  {
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)&ray) & 0x0F        ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT3(normal.travs,1,1,1);
    IntersectContext context(this,nullptr);
    intersect(ray,&context);
  }

  void Scene::rtcIntersect1Ex (const RTCIntersectContext* user_context, RTCRay& ray) 
  {
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)&ray) & 0x0F        ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT3(normal.travs,1,1,1);
    IntersectContext context(this,user_context);
    intersect(ray,&context);
  }

  void Scene::rtcIntersect4 (const void* valid, RTCRay4& ray) 
  {
#if defined(__TARGET_SIMD4__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)&ray ) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,4);
    IntersectContext context(this,nullptr);
    intersect4(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect4 not supported");  
#endif
  }

  void Scene::rtcIntersect4Ex (const void* valid, const RTCIntersectContext* user_context, RTCRay4& ray) 
  {
#if defined(__TARGET_SIMD4__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)&ray ) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,4);
    IntersectContext context(this,user_context);
    intersect4(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect4Ex not supported");  
#endif
  }
  
  void Scene::rtcIntersect8 (const void* valid, RTCRay8& ray) 
  {
#if defined(__TARGET_SIMD8__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)&ray ) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,8);
    IntersectContext context(this,nullptr);
    intersect8(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect8 not supported");      
#endif
  }

  void Scene::rtcIntersect8Ex (const void* valid, const RTCIntersectContext* user_context, RTCRay8& ray) 
  {
#if defined(__TARGET_SIMD8__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)&ray ) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,8);
    IntersectContext context(this,user_context);
    intersect8(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect8Ex not supported");
#endif
  }
  
  void Scene::rtcIntersect16 (const void* valid, RTCRay16& ray) 
  {
#if defined(__TARGET_SIMD16__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)&ray ) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,16);
    IntersectContext context(this,nullptr);
    intersect16(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect16 not supported");
#endif
  }

  void Scene::rtcIntersect16Ex (const void* valid, const RTCIntersectContext* user_context, RTCRay16& ray) 
  {
#if defined(__TARGET_SIMD16__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)&ray ) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,16);
    IntersectContext context(this,user_context);
    intersect16(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect16Ex not supported");
#endif
  }

  void Scene::rtcIntersect1M (const RTCIntersectContext* user_context, RTCRay* rays, const size_t M, const size_t stride) 
  {
#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays ) & 0x03) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,M,M,M);
    IntersectContext context(this,user_context);

    /* fast codepath for single rays */
    if (likely(M == 1)) {
      if (likely(rays->tnear <= rays->tfar)) 
        intersect(*rays,&context);
    } 

    /* codepath for streams */
    else {
      device->rayStreamFilters.filterAOS(this,rays,M,stride,&context,true);   
    }
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect1M not supported");
#endif
  }

  void Scene::rtcIntersect1Mp (const RTCIntersectContext* user_context, RTCRay** rays, const size_t M) 
  {
#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays ) & 0x03) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,M,M,M);
    IntersectContext context(this,user_context);

    /* fast codepath for single rays */
    if (likely(M == 1)) {
      if (likely(rays[0]->tnear <= rays[0]->tfar)) 
        intersect(*rays[0],&context);
    } 

    /* codepath for streams */
    else {
      device->rayStreamFilters.filterAOP(this,rays,M,&context,true);   
    }
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect1Mp not supported");
#endif
  }

  void Scene::rtcIntersectNM (const RTCIntersectContext* user_context, struct RTCRayN* rays, const size_t N, const size_t M, const size_t stride) 
  {
#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays ) & 0x03) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,N*M,N*M,N*M);
    IntersectContext context(this,user_context);

    /* code path for single ray streams */
    if (likely(N == 1))
    {
      /* fast code path for streams of size 1 */
      if (likely(M == 1)) {
        if (likely(((RTCRay*)rays)->tnear <= ((RTCRay*)rays)->tfar))
          intersect(*(RTCRay*)rays,&context);
      } 
      /* normal codepath for single ray streams */
      else {
        device->rayStreamFilters.filterAOS(this,(RTCRay*)rays,M,stride,&context,true);
      }
    }
    /* code path for ray packet streams */
    else {
      device->rayStreamFilters.filterSOA(this,(char*)rays,N,M,stride,&context,true);
    }
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersectNM not supported");
#endif
  }

  void Scene::rtcIntersectNp (const RTCIntersectContext* user_context, const RTCRayNp& rays, const size_t N) 
  {
#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays.orgx   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.orgx not aligned to 4 bytes");   
    if (((size_t)rays.orgy   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.orgy not aligned to 4 bytes");   
    if (((size_t)rays.orgz   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.orgz not aligned to 4 bytes");   
    if (((size_t)rays.dirx   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.dirx not aligned to 4 bytes");   
    if (((size_t)rays.diry   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.diry not aligned to 4 bytes");   
    if (((size_t)rays.dirz   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.dirz not aligned to 4 bytes");   
    if (((size_t)rays.tnear  ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.dirx not aligned to 4 bytes");   
    if (((size_t)rays.tfar   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.tnear not aligned to 4 bytes");   
    if (((size_t)rays.time   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.time not aligned to 4 bytes");   
    if (((size_t)rays.mask   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.mask not aligned to 4 bytes");   
    if (((size_t)rays.Ngx    ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.Ngx not aligned to 4 bytes");   
    if (((size_t)rays.Ngy    ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.Ngy not aligned to 4 bytes");   
    if (((size_t)rays.Ngz    ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.Ngz not aligned to 4 bytes");   
    if (((size_t)rays.u      ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.u not aligned to 4 bytes");   
    if (((size_t)rays.v      ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.v not aligned to 4 bytes");   
    if (((size_t)rays.geomID ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.geomID not aligned to 4 bytes");   
    if (((size_t)rays.primID ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.primID not aligned to 4 bytes");   
    if (((size_t)rays.instID ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.instID not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,N,N,N);
    IntersectContext context(this,user_context);
    device->rayStreamFilters.filterSOP(this,rays,N,&context,true);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersectNp not supported");
#endif
  }
  
  void Scene::rtcOccluded (RTCRay& ray) 
  {
    STAT3(shadow.travs,1,1,1);
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)&ray) & 0x0F        ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    IntersectContext context(this,nullptr);
    occluded(ray,&context);
  }

  void Scene::rtcOccluded1Ex (const RTCIntersectContext* user_context, RTCRay& ray) 
  {
    STAT3(shadow.travs,1,1,1);
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)&ray) & 0x0F        ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    IntersectContext context(this,user_context);
    occluded(ray,&context);
  }
  
  void Scene::rtcOccluded4 (const void* valid, RTCRay4& ray) 
  {
#if defined(__TARGET_SIMD4__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)&ray ) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,4);
    IntersectContext context(this,nullptr);
    occluded4(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded4 not supported");
#endif
  }

  void Scene::rtcOccluded4Ex (const void* valid, const RTCIntersectContext* user_context, RTCRay4& ray) 
  {
#if defined(__TARGET_SIMD4__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)&ray ) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,4);
    IntersectContext context(this,user_context);
    occluded4(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded4Ex not supported");
#endif
  }
 
  void Scene::rtcOccluded8 (const void* valid, RTCRay8& ray) 
  {
#if defined(__TARGET_SIMD8__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)&ray ) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,8);
    IntersectContext context(this,nullptr);
    occluded8(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded8 not supported");
#endif
  }

  void Scene::rtcOccluded8Ex (const void* valid, const RTCIntersectContext* user_context, RTCRay8& ray) 
  {
#if defined(__TARGET_SIMD8__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)&ray ) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,8);
    IntersectContext context(this,user_context);
    occluded8(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded8Ex not supported");
#endif
  }
  
  void Scene::rtcOccluded16 (const void* valid, RTCRay16& ray) 
  {
#if defined(__TARGET_SIMD16__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)&ray ) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,16);
    IntersectContext context(this,nullptr);
    occluded16(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded16 not supported");
#endif
  }

  void Scene::rtcOccluded16Ex (const void* valid, const RTCIntersectContext* user_context, RTCRay16& ray) 
  {
#if defined(__TARGET_SIMD16__) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)&ray ) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,16);
    IntersectContext context(this,user_context);
    occluded16(valid,ray,&context);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded16Ex not supported");
#endif
  }
  
  void Scene::rtcOccluded1M(const RTCIntersectContext* user_context, RTCRay* rays, const size_t M, const size_t stride) 
  {
#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays ) & 0x03) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,M,M,M);
    IntersectContext context(this,user_context);

    /* fast codepath for streams of size 1 */
    if (likely(M == 1)) {
      if (likely(rays->tnear <= rays->tfar)) 
        occluded (*rays,&context);
    } 
    /* codepath for normal streams */
    else {
      device->rayStreamFilters.filterAOS(this,rays,M,stride,&context,false);
    }
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded1M not supported");
#endif
  }

  void Scene::rtcOccluded1Mp(const RTCIntersectContext* user_context, RTCRay** rays, const size_t M) 
  {
#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays ) & 0x03) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,M,M,M);
    IntersectContext context(this,user_context);

    /* fast codepath for streams of size 1 */
    if (likely(M == 1)) {
      if (likely(rays[0]->tnear <= rays[0]->tfar)) 
        occluded (*rays[0],&context);
    } 
    /* codepath for normal streams */
    else {
      device->rayStreamFilters.filterAOP(this,rays,M,&context,false);
    }
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded1Mp not supported");
#endif
  }

  void Scene::rtcOccludedNM(const RTCIntersectContext* user_context, RTCRayN* rays, const size_t N, const size_t M, const size_t stride) 
  {
#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (stride < sizeof(RTCRay)) throw_RTCError(RTC_INVALID_OPERATION,"stride too small");
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays ) & 0x03) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,N*M,N*N,N*N);
    IntersectContext context(this,user_context);

    /* codepath for single rays */
    if (likely(N == 1))
    {
      /* fast path for streams of size 1 */
      if (likely(M == 1)) {
        if (likely(((RTCRay*)rays)->tnear <= ((RTCRay*)rays)->tfar))
          occluded (*(RTCRay*)rays,&context);
      } 
      /* codepath for normal ray streams */
      else {
        device->rayStreamFilters.filterAOS(this,(RTCRay*)rays,M,stride,&context,false);
      }
    }
    /* code path for ray packet streams */
    else {
      device->rayStreamFilters.filterSOA(this,(char*)rays,N,M,stride,&context,false);
    }
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccludedNM not supported");
#endif
  }

  void Scene::rtcOccludedNp(const RTCIntersectContext* user_context, const RTCRayNp& rays, const size_t N) 
  {
#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    if (isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays.orgx   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.orgx not aligned to 4 bytes");   
    if (((size_t)rays.orgy   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.orgy not aligned to 4 bytes");   
    if (((size_t)rays.orgz   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.orgz not aligned to 4 bytes");   
    if (((size_t)rays.dirx   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.dirx not aligned to 4 bytes");   
    if (((size_t)rays.diry   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.diry not aligned to 4 bytes");   
    if (((size_t)rays.dirz   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.dirz not aligned to 4 bytes");   
    if (((size_t)rays.tnear  ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.dirx not aligned to 4 bytes");   
    if (((size_t)rays.tfar   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.tnear not aligned to 4 bytes");   
    if (((size_t)rays.time   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.time not aligned to 4 bytes");   
    if (((size_t)rays.mask   ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.mask not aligned to 4 bytes");   
    if (((size_t)rays.Ngx    ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.Ngx not aligned to 4 bytes");   
    if (((size_t)rays.Ngy    ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.Ngy not aligned to 4 bytes");   
    if (((size_t)rays.Ngz    ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.Ngz not aligned to 4 bytes");   
    if (((size_t)rays.u      ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.u not aligned to 4 bytes");   
    if (((size_t)rays.v      ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.v not aligned to 4 bytes");   
    if (((size_t)rays.geomID ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.geomID not aligned to 4 bytes");   
    if (((size_t)rays.primID ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.primID not aligned to 4 bytes");   
    if (((size_t)rays.instID ) & 0x03 ) throw_RTCError(RTC_INVALID_ARGUMENT, "rays.instID not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,N,N,N);
    IntersectContext context(this,user_context);
    device->rayStreamFilters.filterSOP(this,rays,N,&context,false);
#else
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccludedNp not supported");
#endif
  }
}
}
