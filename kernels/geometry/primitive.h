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

#pragma once

#include "../common/default.h"
#include "../common/scene.h"
#include "../builders/primrefblock.h"

namespace embree
{
  struct PrimitiveType
  {
    /*! constructs the primitive type */
    PrimitiveType (const std::string& name, size_t bytes, size_t blockSize) 
    : name(name), bytes(bytes), blockSize(blockSize) {} 

    /*! Returns the number of stored primitives in a block. */
    virtual size_t size(const char* This) const = 0;

  public:
    std::string name;       //!< name of this primitive type
    size_t bytes;           //!< number of bytes of the triangle data
    size_t blockSize;       //!< block size
  };

  class RayPrecalculations
  {
  public:
    __forceinline RayPrecalculations() {}
    __forceinline RayPrecalculations(const Ray& ray, const void* ptr, const Scene* scene) {}

    __forceinline int itime() const { return 0; }
    __forceinline float ftime() const { return 0.0f; }
  };

  class RayMBPrecalculations
  {
  public:
    __forceinline RayMBPrecalculations() {}

    __forceinline RayMBPrecalculations(const Ray& ray, const void* ptr, const Scene* scene)
    {
      /* calculate time segment itime and fractional time ftime */
      const int time_segments = scene->numTimeSteps-1;
      const float time = ray.time*float(time_segments);
      itime_ = clamp(int(floor(time)),0,time_segments-1);
      ftime_ = time - float(itime_);
    }

    __forceinline int itime() const { return itime_; }
    __forceinline float ftime() const { return ftime_; }

  private:
    /* used for msmblur implementation */
    int itime_;
    float ftime_;
  };

  template<typename Precalculations>
  struct Intersector1Precalculations : public RayPrecalculations, public Precalculations
  {
    __forceinline Intersector1Precalculations() {}

    __forceinline Intersector1Precalculations(const Ray& ray, const void* ptr, const Scene* scene)
      : RayPrecalculations(ray, ptr, scene), Precalculations(ray, ptr) {}
  };

  template<typename Precalculations>
  struct MBIntersector1Precalculations : public RayMBPrecalculations, public Precalculations
  {
    __forceinline MBIntersector1Precalculations() {}

    __forceinline MBIntersector1Precalculations(const Ray& ray, const void* ptr, const Scene* scene)
      : RayMBPrecalculations(ray, ptr, scene), Precalculations(ray, ptr) {}
  };
}
