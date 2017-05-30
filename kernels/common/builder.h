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

#pragma once

#include "default.h"
#include "accel.h"

namespace embree
{
#define MODE_HIGH_QUALITY (1<<8)

  /*! settings for msmblur builder */
  struct CommonBuildSettings
  {
    /*! default settings */
    CommonBuildSettings ()
    : branchingFactor(2), maxDepth(32), logBlockSize(0), minLeafSize(1), maxLeafSize(8),
      travCost(1.0f), intCost(1.0f), singleLeafTimeSegment(false),
      singleThreadThreshold(1024) {}
    
  public:
    size_t branchingFactor;  //!< branching factor of BVH to build
    size_t maxDepth;         //!< maximal depth of BVH to build
    size_t logBlockSize;     //!< log2 of blocksize for SAH heuristic
    size_t minLeafSize;      //!< minimal size of a leaf
    size_t maxLeafSize;      //!< maximal size of a leaf
    float travCost;          //!< estimated cost of one traversal step
    float intCost;           //!< estimated cost of one primitive intersection
    bool singleLeafTimeSegment; //!< split time to single time range
    size_t singleThreadThreshold; //!< threshold when we switch to single threaded build
  };

  /*! virtual interface for all hierarchy builders */
  class Builder : public RefCount {
  public:

    static const size_t DEFAULT_SINGLE_THREAD_THRESHOLD = 1024;

    /*! initiates the hierarchy builder */
    virtual void build() = 0;

    /*! notifies the builder about the deletion of some geometry */
    virtual void deleteGeometry(size_t geomID) {};

    /*! clears internal builder state */
    virtual void clear() = 0;
  };

  /*! virtual interface for progress monitor class */
  struct BuildProgressMonitor {
    virtual void operator() (size_t dn) const = 0;
  };

  /*! build the progress monitor interface from a closure */
  template<typename Closure>
    struct ProgressMonitorClosure : BuildProgressMonitor
  {
  public:
    ProgressMonitorClosure (const Closure& closure) : closure(closure) {}
    void operator() (size_t dn) const { closure(dn); }
  private:
    const Closure closure;
  };
  template<typename Closure> __forceinline const ProgressMonitorClosure<Closure> BuildProgressMonitorFromClosure(const Closure& closure) {
    return ProgressMonitorClosure<Closure>(closure);
  }

  struct LineSegments;
  struct TriangleMesh;
  struct QuadMesh;
  class AccelSet;

  class Scene;

  typedef void (*createLineSegmentsAccelTy)(LineSegments* mesh, AccelData*& accel, Builder*& builder);
  typedef void (*createTriangleMeshAccelTy)(TriangleMesh* mesh, AccelData*& accel, Builder*& builder);
  typedef void (*createQuadMeshAccelTy)(QuadMesh* mesh, AccelData*& accel, Builder*& builder);
  typedef void (*createAccelSetAccelTy)(AccelSet* mesh, AccelData*& accel, Builder*& builder);

}
