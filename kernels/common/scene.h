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

#pragma once

#include "default.h"
#include "device.h"
#include "scene_triangle_mesh.h"
#include "scene_quad_mesh.h"
#include "scene_user_geometry.h"
#include "scene_instance.h"
#include "scene_geometry_instance.h"
#include "scene_bezier_curves.h"
#include "scene_line_segments.h"
#include "scene_subdiv_mesh.h"

#include "subdiv/tessellation_cache.h"

#include "acceln.h"
#include "geometry.h"

namespace embree
{
  /*! decoding of geometry flags */
  __forceinline bool isStatic    (RTCSceneFlags flags) { return (flags & 1) == RTC_SCENE_STATIC; }
  __forceinline bool isDynamic   (RTCSceneFlags flags) { return (flags & 1) == RTC_SCENE_DYNAMIC; }

  __forceinline bool isCompact   (RTCSceneFlags flags) { return flags & RTC_SCENE_COMPACT; }
  __forceinline bool isRobust    (RTCSceneFlags flags) { return flags & RTC_SCENE_ROBUST; }
  __forceinline bool isCoherent  (RTCSceneFlags flags) { return flags & RTC_SCENE_COHERENT; }
  __forceinline bool isIncoherent(RTCSceneFlags flags) { return flags & RTC_SCENE_INCOHERENT; }
  __forceinline bool isHighQuality(RTCSceneFlags flags) { return flags & RTC_SCENE_HIGH_QUALITY; }
  __forceinline bool isInterpolatable(RTCAlgorithmFlags flags) { return flags & RTC_INTERPOLATE; }

  /*! Base class all scenes are derived from */
  class Scene : public Accel
  {
    ALIGNED_CLASS;

  public:
    template<typename Ty, size_t timeSteps = 1>
      class Iterator
      {
      public:
      Iterator ()  {}
      
      Iterator (Scene* scene, bool all = false) 
      : scene(scene), all(all) {}
      
      __forceinline Ty* at(const size_t i)
      {
        Geometry* geom = scene->geometries[i];
        if (geom == nullptr) return nullptr;
        if (!all && !geom->isEnabled()) return nullptr;
        if (geom->getType() != Ty::geom_type) return nullptr;
        if (geom->numTimeSteps != timeSteps) return nullptr;
        return (Ty*) geom;
      }

      __forceinline Ty* operator[] (const size_t i) {
        return at(i);
      }

      __forceinline size_t size() const {
        return scene->size();
      }
      
      __forceinline size_t numPrimitives() const {
        return scene->getNumPrimitives<Ty,timeSteps>();
      }

      __forceinline size_t maxPrimitivesPerGeometry() 
      {
        size_t ret = 0;
        for (size_t i=0; i<scene->size(); i++) {
          Ty* mesh = at(i);
          if (mesh == nullptr) continue;
          ret = max(ret,mesh->size());
        }
        return ret;
      }
      
    private:
      Scene* scene;
      bool all;
      };

  public:
    
    /*! Scene construction */
    Scene (Device* device, RTCSceneFlags flags, RTCAlgorithmFlags aflags);

    void createTriangleAccel();
    void createQuadAccel();
    void createTriangleMBAccel();
    void createQuadMBAccel();
    void createHairAccel();
    void createHairMBAccel();
    void createLineAccel();
    void createLineMBAccel();
    void createSubdivAccel();

    /*! Scene destruction */
    ~Scene ();
    
    /*! clears the scene */
    void clear();

    /*! Creates new user geometry. */
    unsigned int newUserGeometry (size_t items, size_t numTimeSteps);

    /*! Creates a new scene instance. */
    unsigned int newInstance (Scene* scene);

    /*! Creates a new geometry instance. */
    unsigned int newGeometryInstance (Geometry* geom);

    /*! Creates a new triangle mesh. */
    unsigned int newTriangleMesh (RTCGeometryFlags flags, size_t maxTriangles, size_t maxVertices, size_t numTimeSteps);

    /*! Creates a new quad mesh. */
    unsigned int newQuadMesh (RTCGeometryFlags flags, size_t maxQuads, size_t maxVertices, size_t numTimeSteps);

    /*! Creates a new collection of quadratic bezier curves. */
    unsigned int newBezierCurves (RTCGeometryFlags flags, size_t maxCurves, size_t maxVertices, size_t numTimeSteps);

    /*! Creates a new collection of line segments. */
    unsigned int newLineSegments (RTCGeometryFlags flags, size_t maxSegments, size_t maxVertices, size_t numTimeSteps);

    /*! Creates a new subdivision mesh. */
    unsigned int newSubdivisionMesh (RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps);

    /*! deletes some geometry */
    void deleteGeometry(size_t geomID);

    /*! Builds acceleration structure for the scene. */
    void build (size_t threadIndex, size_t threadCount);
    void build_task ();

    /*! stores scene into binary file */
    void write(std::ofstream& file);

    void updateInterface();

    /*! build task */
#if defined(TASKING_LOCKSTEP)
    TASK_RUN_FUNCTION(Scene,task_build_parallel);
    TaskScheduler::Task task;
#endif

    /* return number of geometries */
    __forceinline size_t size() const { return geometries.size(); }
    
    /* add user geometry to scene */
    unsigned int add (Geometry* geometry);
    
    /* determines of the scene is ready to get build */
    bool ready() { return numMappedBuffers == 0; }

    /* determines if scene is modified */
    __forceinline bool isModified() const { return modified; }

    /* sets modified flag */
    __forceinline void setModified(bool f = true) { 
      modified = f; 
    }

    /* get mesh by ID */
    __forceinline       Geometry* get(size_t i)       { assert(i < geometries.size()); return geometries[i]; }
    __forceinline const Geometry* get(size_t i) const { assert(i < geometries.size()); return geometries[i]; }

    template<typename Mesh>
    __forceinline Mesh* getSafe(size_t i) {
      assert(i < geometries.size());
      if (geometries[i] == nullptr) return nullptr;
      if (geometries[i]->getType() != Mesh::geom_type) return nullptr;
      else return (Mesh*) geometries[i];
    }

    __forceinline Geometry* get_locked(size_t i)  {
      Lock<AtomicMutex> lock(geometriesMutex);
      Geometry *g = geometries[i]; 
      assert(i < geometries.size()); 
      return g; 
    }

    /* get triangle mesh by ID */
    __forceinline TriangleMesh* getTriangleMesh(size_t i) { 
      assert(i < geometries.size()); 
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::TRIANGLE_MESH);
      return (TriangleMesh*) geometries[i]; 
    }
    __forceinline const TriangleMesh* getTriangleMesh(size_t i) const { 
      assert(i < geometries.size()); 
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::TRIANGLE_MESH);
      return (TriangleMesh*) geometries[i]; 
    }
    __forceinline TriangleMesh* getTriangleMeshSafe(size_t i) { 
      assert(i < geometries.size()); 
      if (geometries[i] == nullptr) return nullptr;
      if (geometries[i]->getType() != Geometry::TRIANGLE_MESH) return nullptr;
      else return (TriangleMesh*) geometries[i]; 
    }

    /* get quad mesh by ID */
    __forceinline QuadMesh* getQuadMesh(size_t i) { 
      assert(i < geometries.size()); 
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::QUAD_MESH);
      return (QuadMesh*) geometries[i]; 
    }
    __forceinline const QuadMesh* getQuadMesh(size_t i) const { 
      assert(i < geometries.size()); 
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::QUAD_MESH);
      return (QuadMesh*) geometries[i]; 
    }

    /* get subdiv mesh by ID */
    __forceinline SubdivMesh* getSubdivMesh(size_t i) { 
      assert(i < geometries.size()); 
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::SUBDIV_MESH);
      return (SubdivMesh*) geometries[i]; 
    }
    __forceinline const SubdivMesh* getSubdivMesh(size_t i) const { 
      assert(i < geometries.size()); 
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::SUBDIV_MESH);
      return (SubdivMesh*) geometries[i]; 
    }

    /* get user geometry by ID */
    __forceinline AccelSet* getUserGeometrySafe(size_t i) { 
      assert(i < geometries.size()); 
      if (geometries[i] == nullptr) return nullptr;
      if (geometries[i]->getType() != Geometry::USER_GEOMETRY) return nullptr;
      else return (AccelSet*) geometries[i]; 
    }

    __forceinline BezierCurves* getBezierCurves(size_t i) { 
      assert(i < geometries.size()); 
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::BEZIER_CURVES);
      return (BezierCurves*) geometries[i]; 
    }

    __forceinline LineSegments* getLineSegments(size_t i) {
      assert(i < geometries.size());
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::LINE_SEGMENTS);
      return (LineSegments*) geometries[i];
    }
    __forceinline const LineSegments* getLineSegments(size_t i) const {
      assert(i < geometries.size());
      assert(geometries[i]);
      assert(geometries[i]->getType() == Geometry::LINE_SEGMENTS);
      return (LineSegments*) geometries[i];
    }

    /* test if this is a static scene */
    __forceinline bool isStatic() const { return embree::isStatic(flags); }

    /* test if this is a dynamic scene */
    __forceinline bool isDynamic() const { return embree::isDynamic(flags); }

    __forceinline bool isCompact() const { return embree::isCompact(flags); }
    __forceinline bool isCoherent() const { return embree::isCoherent(flags); }
    __forceinline bool isRobust() const { return embree::isRobust(flags); }
    __forceinline bool isHighQuality() const { return embree::isHighQuality(flags); }
    __forceinline bool isInterpolatable() const { return embree::isInterpolatable(aflags); }

    /* test if scene got already build */
    __forceinline bool isBuild() const { return is_build; }

  public:
    std::vector<int> usedIDs; // FIXME: encapsulate this functionality into own class
    std::vector<Geometry*> geometries; //!< list of all user geometries

    static AtomicCounter numScenes;
    
  public:
    Device* device;
    AccelN accels;
    unsigned int commitCounter;
    atomic_t commitCounterSubdiv;
    atomic_t numMappedBuffers;         //!< number of mapped buffers
    RTCSceneFlags flags;
    RTCAlgorithmFlags aflags;
    bool needTriangleIndices; 
    bool needTriangleVertices; 
    bool needQuadIndices; 
    bool needQuadVertices; 
    bool needBezierIndices;
    bool needBezierVertices;
    bool needLineIndices;
    bool needLineVertices;
    bool needSubdivIndices;
    bool needSubdivVertices;
    bool is_build;
    MutexSys buildMutex;
    AtomicMutex geometriesMutex;
    bool modified;                   //!< true if scene got modified
    
    /*! global lock step task scheduler */
#if defined(TASKING_LOCKSTEP)
    __aligned(64) LockStepTaskScheduler lockstep_scheduler;
#elif defined(TASKING_TBB_INTERNAL)
    MutexSys schedulerMutex;
    Ref<TaskSchedulerTBB> scheduler;
#else
    tbb::task_group* group;
    BarrierActiveAutoReset group_barrier;
#endif
    
  public:
    struct BuildProgressMonitorInterface : public BuildProgressMonitor {
      BuildProgressMonitorInterface(Scene* scene) 
      : scene(scene) {}
      void operator() (size_t dn) { scene->progressMonitor(dn); }
    private:
      Scene* scene;
    };
    BuildProgressMonitorInterface progressInterface;
    RTCProgressMonitorFunc progress_monitor_function;
    void* progress_monitor_ptr;
    atomic_t progress_monitor_counter;
    void progressMonitor(double nprims);
    void setProgressMonitorFunction(RTCProgressMonitorFunc func, void* ptr);

  public:
    struct GeometryCounts 
    {
      __forceinline GeometryCounts()
        : numTriangles(0), numQuads(0), numBezierCurves(0), numLineSegments(0), numSubdivPatches(0), numUserGeometries(0) {}

      __forceinline size_t size() const {
        return numTriangles + numQuads + numBezierCurves + numLineSegments + numSubdivPatches + numUserGeometries;
      }

      atomic_t numTriangles;             //!< number of enabled triangles
      atomic_t numQuads;                 //!< number of enabled quads
      atomic_t numBezierCurves;          //!< number of enabled curves
      atomic_t numLineSegments;          //!< number of enabled line segments
      atomic_t numSubdivPatches;         //!< number of enabled subdivision patches
      atomic_t numUserGeometries;        //!< number of enabled user geometries
    };
    
    GeometryCounts world1;               //!< counts for non-motion blurred geometry
    GeometryCounts world2;               //!< counts for motion blurred geometry
    GeometryCounts instanced1;           //!< instance counts for non-motion blurred geometry
    GeometryCounts instanced2;           //!< instance counts for motion blurred geometry

    atomic_t numSubdivEnableDisableEvents; //!< number of enable/disable calls for any subdiv geometry

    __forceinline size_t numPrimitives() const {
      return world1.size() + world2.size();
    }

    template<typename Mesh, int timeSteps> __forceinline size_t getNumPrimitives() const;
   
    atomic_t numIntersectionFilters4;   //!< number of enabled intersection/occlusion filters for 4-wide ray packets
    atomic_t numIntersectionFilters8;   //!< number of enabled intersection/occlusion filters for 8-wide ray packets
    atomic_t numIntersectionFilters16;  //!< number of enabled intersection/occlusion filters for 16-wide ray packets
  };

  template<> __forceinline size_t Scene::getNumPrimitives<TriangleMesh,1>() const { return world1.numTriangles; } 
  template<> __forceinline size_t Scene::getNumPrimitives<TriangleMesh,2>() const { return world2.numTriangles; } 
  template<> __forceinline size_t Scene::getNumPrimitives<QuadMesh,1>() const { return world1.numQuads; } 
  template<> __forceinline size_t Scene::getNumPrimitives<QuadMesh,2>() const { return world2.numQuads; } 
  template<> __forceinline size_t Scene::getNumPrimitives<BezierCurves,1>() const { return world1.numBezierCurves; } 
  template<> __forceinline size_t Scene::getNumPrimitives<BezierCurves,2>() const { return world2.numBezierCurves; } 
  template<> __forceinline size_t Scene::getNumPrimitives<LineSegments,1>() const { return world1.numLineSegments; }
  template<> __forceinline size_t Scene::getNumPrimitives<LineSegments,2>() const { return world2.numLineSegments; }
  template<> __forceinline size_t Scene::getNumPrimitives<SubdivMesh,1>() const { return world1.numSubdivPatches; } 
  template<> __forceinline size_t Scene::getNumPrimitives<SubdivMesh,2>() const { return world2.numSubdivPatches; } 
  template<> __forceinline size_t Scene::getNumPrimitives<AccelSet,1>() const { return world1.numUserGeometries; } 
  template<> __forceinline size_t Scene::getNumPrimitives<AccelSet,2>() const { return world2.numUserGeometries; } 
}
