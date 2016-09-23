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

#include "accelset.h"

namespace embree
{
  class InstanceFactory
  {
  public:
    InstanceFactory(int features);
    DEFINE_SYMBOL2(RTCBoundsFunc3,InstanceBoundsFunc);
    DEFINE_SYMBOL2(AccelSet::Intersector1,InstanceIntersector1);
    DEFINE_SYMBOL2(AccelSet::Intersector4,InstanceIntersector4);
    DEFINE_SYMBOL2(AccelSet::Intersector8,InstanceIntersector8);
    DEFINE_SYMBOL2(AccelSet::Intersector16,InstanceIntersector16);
    DEFINE_SYMBOL2(AccelSet::Intersector1M,InstanceIntersector1M);
  };

  /*! Instanced acceleration structure */
  struct Instance : public AccelSet
  {
    ALIGNED_STRUCT;
  public:
    static Instance* create (Scene* parent, Scene* object, size_t numTimeSteps) {
      return ::new (alignedMalloc(sizeof(Instance)+(numTimeSteps-1)*sizeof(AffineSpace3fa))) Instance(parent,object,numTimeSteps);
    }
  private:
    Instance (Scene* parent, Scene* object, size_t numTimeSteps); 
  public:
    virtual void setTransform(const AffineSpace3fa& local2world, size_t timeStep);
    virtual void setMask (unsigned mask);
    virtual void build(size_t threadIndex, size_t threadCount) {}

  public:

    __forceinline AffineSpace3fa getWorld2Local() const {
      return world2local0;
    }

    __forceinline AffineSpace3fa getWorld2Local(float t) const 
    {
      /* calculate time segment itime and fractional time ftime */
      const int time_segments = numTimeSteps-1;
      const float time = t*float(time_segments);
      const int itime = clamp(int(floor(time)),0,time_segments-1);
      const float ftime = time - float(itime);
      return rcp(lerp(local2world[itime+0],local2world[itime+1],ftime));
    }

    template<int K>
      __forceinline AffineSpaceT<LinearSpace3<Vec3<vfloat<K>>>> getWorld2Local(const vbool<K>& valid, const vfloat<K>& t) const
    {
      typedef AffineSpaceT<LinearSpace3<Vec3<vfloat<K>>>> AffineSpace3vfK;
      
      /* calculate time segment itime and fractional time ftime */
      const int time_segments = numTimeSteps-1;
      const vfloat<K> time = t*vfloat<K>(time_segments);
      const vint<K> itime_k = clamp(vint<K>(floor(time)),vint<K>(0),vint<K>(time_segments-1));
      const vfloat<K> ftime = time - vfloat<K>(itime_k);
#if 1
      assert(any(valid));
      const size_t index = __bsf(movemask(valid));
      const int itime = itime_k[index];
      const vfloat<K> t0 = vfloat<K>(1.0f)-ftime;
      const vfloat<K> t1 = ftime;
      return rcp(t0*AffineSpace3vfK(local2world[itime+0])+t1*AffineSpace3vfK(local2world[itime+1]));
#else
      AffineSpaceT<Vec3<vfloat<K>>> result = one;
      const vfloat<K> t0 = vfloat<K>(1.0f)-ftime;
      const vfloat<K> t1 = ftime;
      foreach_unique(valid,itime_k,[&] (const vbool<K>& valid, int itime) {
          AffineSpaceT<Vec3<vfloat<K>>> m(rcp(t0*AffineSpace3vfK(local2world[itime+0])+t1*AffineSpace3vfK(local2world[itime+1])));
          result = select(valid,m,result);
        });
      return result;
#endif
    }
    
  public:
    Scene* object;                 //!< pointer to instanced acceleration structure
    AffineSpace3fa world2local0;   //!< transformation from world space to local space for timestep 0
    AffineSpace3fa local2world[1]; //!< transformation from local space to world space for each timestep
  };
}
