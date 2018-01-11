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

#pragma once

#include "geometry.h"

namespace embree
{
  /*! Instanced geometry */
  struct GeometryInstance : public Geometry
  {
    ALIGNED_STRUCT;
  public:

    /*! type of this geometry */
    static const Geometry::Type geom_type = Geometry::INSTANCE;

    GeometryInstance (Scene* scene, Geometry* geom); 
    virtual void build() {}
    virtual void enabling ();
    virtual void disabling();
    virtual void setMask (unsigned mask);
    virtual void setTransform(const AffineSpace3fa& local2world, size_t timeStep);
    
    void count(Geometry* geom, ssize_t f);
  public:
    AffineSpace3fa local2world; //!< transforms from local space to world space
    AffineSpace3fa world2local; //!< transforms from world space to local space
    Geometry* geom;             //!< pointer to instanced acceleration structure
  };

  /*! Geometry group */
  struct GeometryGroup : public Geometry
  {
    ALIGNED_STRUCT;
  public:

    /*! type of this geometry */
    static const Geometry::Type geom_type = Geometry::GROUP;

    GeometryGroup (Scene* scene, RTCGeometryFlags gflags, const std::vector<Geometry*>& geometries); 
    virtual void build() {}
    virtual void enabling ();
    virtual void disabling();
    virtual void setMask (unsigned mask);

    __forceinline       Geometry* operator[] ( const size_t i )       {  return geometries[i]; }
    __forceinline const Geometry* operator[] ( const size_t i ) const {  return geometries[i]; }

  public:
    std::vector<Geometry*> geometries; 
  };
}
