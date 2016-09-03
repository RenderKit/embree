// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
  void missing_rtcCommit()      { throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed"); }
  void invalid_rtcIntersect1()  { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect and rtcOccluded not enabled"); }
  void invalid_rtcIntersect4()  { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect4 and rtcOccluded4 not enabled"); }
  void invalid_rtcIntersect8()  { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect8 and rtcOccluded8 not enabled"); }
  void invalid_rtcIntersect16() { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect16 and rtcOccluded16 not enabled"); }
  void invalid_rtcIntersectN()  { throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersectN and rtcOccludedN not enabled"); }

  Scene::Scene (Device* device, RTCSceneFlags sflags, RTCAlgorithmFlags aflags)
    : Accel(AccelData::TY_UNKNOWN),
      device(device), 
      commitCounter(0), 
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

    createTriangleAccel();
    createTriangleMBAccel();
    createQuadAccel();
    createQuadMBAccel();
    createSubdivAccel();
    createHairAccel();
    createHairMBAccel();
    createLineAccel();
    createLineMBAccel();

#if defined(EMBREE_GEOMETRY_TRIANGLES)
    accels.add(device->bvh4_factory->BVH4InstancedBVH4Triangle4ObjectSplit(this));
#endif

#if defined(EMBREE_GEOMETRY_USER)
    accels.add(device->bvh4_factory->BVH4UserGeometry(this)); // has to be the last as the instID field of a hit instance is not invalidated by other hit geometry
    accels.add(device->bvh4_factory->BVH4UserGeometryMB(this)); // has to be the last as the instID field of a hit instance is not invalidated by other hit geometry
#endif
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
#if defined (__TARGET_AVX__)
          if (device->hasISA(AVX))
	  {
            if (isHighQuality()) 
            {
              /* new spatial split builder is now active per default */
              accels.add(device->bvh8_factory->BVH8Triangle4SpatialSplit(this)); 
            }
            else
              accels.add(device->bvh8_factory->BVH8Triangle4ObjectSplit(this));
          }
          else 
#endif
          {
            
            if (isHighQuality()) 
            {
              /* new spatial split builder is now active per default */
              accels.add(device->bvh4_factory->BVH4Triangle4SpatialSplit(this));
            }
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
        case /*0b01*/ 1: accels.add(device->bvh4_factory->BVH4Triangle4vTwolevel(this)); break;
        case /*0b10*/ 2: accels.add(device->bvh4_factory->BVH4Triangle4iTwolevel(this)); break;
        case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Triangle4iTwolevel(this)); break;
        }
      }
    }
    else if (device->tri_accel == "bvh4.triangle4")       accels.add(device->bvh4_factory->BVH4Triangle4(this));
    else if (device->tri_accel == "bvh4.triangle4v")      accels.add(device->bvh4_factory->BVH4Triangle4v(this));
    else if (device->tri_accel == "bvh4.triangle4i")      accels.add(device->bvh4_factory->BVH4Triangle4i(this));

#if defined (__TARGET_AVX__)
    else if (device->tri_accel == "bvh8.triangle4")       accels.add(device->bvh8_factory->BVH8Triangle4(this));
    else if (device->tri_accel == "qbvh8.triangle4i")      accels.add(device->bvh8_factory->BVH8QuantizedTriangle4i(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown triangle acceleration structure "+device->tri_accel);
#endif
  }

  void Scene::createQuadAccel()
  {
#if defined(EMBREE_GEOMETRY_QUADS)
    if (device->quad_accel == "default") 
    {
      int mode =  2*(int)isCompact() + 1*(int)isRobust(); 
      switch (mode) {
      case /*0b00*/ 0:
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8Quad4v(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Quad4v(this));
        break;

      case /*0b01*/ 1:
#if defined (__TARGET_AVX__) && 1
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8Quad4i(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4Quad4i(this));
        break;

      case /*0b10*/ 2: 
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX))
        {
          // FIXME: reduce performance overhead of 10% compared to uncompressed bvh8
          // if (isExclusiveIntersect1Mode())
          //   accels.add(device->bvh8_factory->BVH8QuantizedQuad4i(this)); 
          // else
          accels.add(device->bvh8_factory->BVH8Quad4i(this)); 
        }
        else
#endif
        {
          accels.add(device->bvh4_factory->BVH4Quad4i(this));
        }
        break;

      case /*0b11*/ 3: accels.add(device->bvh4_factory->BVH4Quad4i(this)); break;

      }
    }
    else if (device->quad_accel == "bvh4.quad4v")       accels.add(device->bvh4_factory->BVH4Quad4v(this));
    else if (device->quad_accel == "bvh4.quad4i")       accels.add(device->bvh4_factory->BVH4Quad4i(this));
    else if (device->quad_accel == "qbvh4.quad4i")      accels.add(device->bvh4_factory->BVH4QuantizedQuad4i(this));

#if defined (__TARGET_AVX__)
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
#endif
  }

  void Scene::createTriangleMBAccel()
  {
#if defined(EMBREE_GEOMETRY_TRIANGLES)
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
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX2)) // only enable on HSW machines, for SNB this codepath is slower
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
#endif
  }

  void Scene::createHairMBAccel()
  {
#if defined(EMBREE_GEOMETRY_HAIR)
    if (device->hair_accel_mb == "default")
    {
#if defined (__TARGET_AVX__)
      if (device->hasISA(AVX2)) // only enable on HSW machines, on SNB this codepath is slower
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
#endif
  }

  void Scene::createLineAccel()
  {
#if defined(EMBREE_GEOMETRY_LINES)
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
#endif
  }

  void Scene::createLineMBAccel()
  {
#if defined(EMBREE_GEOMETRY_LINES)
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
#endif
  }

  void Scene::createSubdivAccel()
  {
#if defined(EMBREE_GEOMETRY_SUBDIV)
    if (device->subdiv_accel == "default") 
    {
      if (isIncoherent(flags) && isStatic())
      {
#if defined (__TARGET_AVX__)
        if (device->hasISA(AVX))
          accels.add(device->bvh8_factory->BVH8SubdivGridEager(this));
        else
#endif
          accels.add(device->bvh4_factory->BVH4SubdivGridEager(this));
      }
      else
        accels.add(device->bvh4_factory->BVH4SubdivPatch1Cached(this));
    }
    else if (device->subdiv_accel == "bvh4.subdivpatch1cached") accels.add(device->bvh4_factory->BVH4SubdivPatch1Cached(this));
    else if (device->subdiv_accel == "bvh4.grid.eager"        ) accels.add(device->bvh4_factory->BVH4SubdivGridEager(this));
#if defined (__TARGET_AVX__)
    else if (device->subdiv_accel == "bvh8.grid.eager"        ) accels.add(device->bvh8_factory->BVH8SubdivGridEager(this));
#endif
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown subdiv accel "+device->subdiv_accel);
#endif
  }

  Scene::~Scene () 
  {
    for (size_t i=0; i<geometries.size(); i++)
      delete geometries[i];

#if TASKING_TBB
    delete group; group = nullptr;
#endif
  }

  void Scene::clear() {
  }

  unsigned Scene::newUserGeometry (size_t items, size_t numTimeSteps) 
  {
    Geometry* geom = new UserGeometry(this,items,numTimeSteps);
    return geom->id;
  }

  unsigned Scene::newInstance (Scene* scene, size_t numTimeSteps) 
  {
    Geometry* geom = new Instance(this,scene,numTimeSteps);
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


  unsigned Scene::newBezierCurves (BezierCurves::SubType subtype, RTCGeometryFlags gflags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    if (isStatic() && (gflags != RTC_GEOMETRY_STATIC)) {
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes can only contain static geometries");
      return -1;
    }

    if (numTimeSteps == 0 || numTimeSteps > 2) {
      throw_RTCError(RTC_INVALID_OPERATION,"only 1 or 2 time steps supported");
      return -1;
    }
    
    Geometry* geom = new BezierCurves(this,subtype,gflags,numCurves,numVertices,numTimeSteps);
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

  unsigned Scene::add(Geometry* geometry) 
  {
    Lock<SpinLock> lock(geometriesMutex);

    if (usedIDs.size()) {
      unsigned id = usedIDs.back(); 
      usedIDs.pop_back();
      geometries[id] = geometry;
      return id;
    } else {
      geometries.push_back(geometry);
      return unsigned(geometries.size()-1);
    }
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
    usedIDs.push_back(unsigned(geomID));
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

    /* update commit counter */
    commitCounter++;
  }

  void Scene::build_task ()
  {
    progress_monitor_counter = 0;

    /* select fast code path if no intersection filter is present */
    accels.select(numIntersectionFiltersN+numIntersectionFilters4,
                  numIntersectionFiltersN+numIntersectionFilters8,
                  numIntersectionFiltersN+numIntersectionFilters16,
                  numIntersectionFiltersN);
  
    /* build all hierarchies of this scene */
    accels.build(0,0);
    
    /* make static geometry immutable */
    if (isStatic()) 
    {
      accels.immutable();
      for (size_t i=0; i<geometries.size(); i++)
        if (geometries[i]) geometries[i]->immutable();
    }

    /* clear modified flag */
    for (size_t i=0; i<geometries.size(); i++)
    {
      Geometry* geom = geometries[i];
      if (!geom) continue;
      if (geom->isEnabled()) geom->clearModified(); // FIXME: should builders do this?
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

#if defined(TASKING_INTERNAL)

  void Scene::build (size_t threadIndex, size_t threadCount) 
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
    Lock<MutexSys> lock(buildMutex,buildMutex.try_lock());

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
    unsigned int mxcsr = _mm_getcsr();
    _mm_setcsr(mxcsr | /* FTZ */ (1<<15) | /* DAZ */ (1<<6));
    
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
      _mm_setcsr(mxcsr);
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

  void Scene::write(std::ofstream& file)
  {
    int magick = 0x35238765LL;
    file.write((char*)&magick,sizeof(magick));
    size_t numGroups = size();
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
      size_t n = size_t(dn) + progress_monitor_counter.fetch_add(size_t(dn));
      if (!progress_monitor_function(progress_monitor_ptr, n / (double(numPrimitives())))) {
        throw_RTCError(RTC_CANCELLED,"progress monitor forced termination");
      }
    }
  }
}
