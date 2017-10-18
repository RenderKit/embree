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
#include "builder.h"
#include "geometry.h"
#include "ray.h"

namespace embree
{
  /*! Base class for set of acceleration structures. */
  class AccelSet : public Geometry
  {
    ALIGNED_CLASS;
  public:

    /*! type of this geometry */
    static const Geometry::Type geom_type = Geometry::USER_GEOMETRY;

    typedef RTCIntersectFuncN IntersectFuncN;  
    typedef RTCOccludedFuncN OccludedFuncN;
    typedef void (*ErrorFunc) ();

      struct IntersectorN
      {
        IntersectorN (ErrorFunc error = nullptr) ;
        IntersectorN (IntersectFuncN intersect, OccludedFuncN occluded, const char* name);
        
        operator bool() const { return name; }
        
      public:
        static const char* type;
        IntersectFuncN intersect;
        OccludedFuncN occluded; 
        const char* name;
      };
      
    public:
      
      /*! construction */
      AccelSet (Device* device, RTCGeometryFlags gflags, size_t items, size_t numTimeSteps);
      
      /*! makes the acceleration structure immutable */
      virtual void immutable () {}
      
      /*! build accel */
      virtual void build () = 0;

      /*! check if the i'th primitive is valid between the specified time range */
      __forceinline bool valid(size_t i, const range<size_t>& itime_range) const
      {
        for (size_t itime = itime_range.begin(); itime <= itime_range.end(); itime++)
          if (!isvalid(bounds(i,itime))) return false;
        
        return true;
      }

      /*! Calculates the bounds of an item */
      __forceinline BBox3fa bounds(size_t i, size_t itime = 0) const
      {
        BBox3fa box;
        assert(i < size());
        boundsFunc(boundsFuncUserPtr,intersectors.ptr,i,itime,(RTCBounds&)box);
        return box;
      }

      /*! calculates the linear bounds of the i'th item at the itime'th time segment */
      __forceinline LBBox3fa linearBounds(size_t i, size_t itime) const
      {
        BBox3fa box[2];
        assert(i < size());
        boundsFunc(boundsFuncUserPtr,intersectors.ptr,i,itime+0,(RTCBounds&)box[0]);
        boundsFunc(boundsFuncUserPtr,intersectors.ptr,i,itime+1,(RTCBounds&)box[1]);
        return LBBox3fa(box[0],box[1]);
      }

      /*! calculates the build bounds of the i'th item, if it's valid */
      __forceinline bool buildBounds(size_t i, BBox3fa* bbox = nullptr) const
      {
        const BBox3fa b = bounds(i);
        if (bbox) *bbox = b;
        return isvalid(b);
      }

      /*! calculates the build bounds of the i'th item at the itime'th time segment, if it's valid */
      __forceinline bool buildBounds(size_t i, size_t itime, BBox3fa& bbox) const
      {
        const BBox3fa bounds0 = bounds(i,itime+0);
        const BBox3fa bounds1 = bounds(i,itime+1);
        bbox = bounds0; // use bounding box of first timestep to build BVH
        return isvalid(bounds0) && isvalid(bounds1);
      }

      /*! calculates the linear bounds of the i'th primitive for the specified time range */
      __forceinline LBBox3fa linearBounds(size_t primID, const BBox1f& time_range) const {
        return LBBox3fa([&] (size_t itime) { return bounds(primID, itime); }, time_range, fnumTimeSegments);
      }
      
      /*! calculates the linear bounds of the i'th primitive for the specified time range */
      __forceinline bool linearBounds(size_t i, const BBox1f& time_range, LBBox3fa& bbox) const  {
        if (!valid(i, getTimeSegmentRange(time_range, fnumTimeSegments))) return false;
        bbox = linearBounds(i, time_range);
        return true;
      }

      void enabling ();
      void disabling();

  public:

      /*! Intersects a single ray with the scene. */
      __forceinline void intersect (Ray& ray, size_t item, IntersectContext* context) 
      {
        int mask = -1;
        assert(intersectors.intersectorN.intersect);
        intersectors.intersectorN.intersect((int*)&mask,intersectors.ptr,context->user,(RTCRayN*)&ray,1,item);
      }
   
      /*! Intersects a packet of 4 rays with the scene. */
#if defined(__SSE__)   
      __forceinline void intersect (const vbool4& valid, Ray4& ray, size_t item, IntersectContext* context) 
      {
        assert(item < size());
        vint4 mask = valid.mask32();
        assert(intersectors.intersectorN.intersect);          
        intersectors.intersectorN.intersect((int*)&mask,intersectors.ptr,context->user,(RTCRayN*)&ray,4,item);
      }
#endif
      
#if defined(__AVX__)
      /*! Intersects a packet of 8 rays with the scene. */
      __forceinline void intersect (const vbool8& valid, Ray8& ray, size_t item, IntersectContext* context) 
      {
        assert(item < size());
        vint8 mask = valid.mask32();
        assert(intersectors.intersectorN.intersect);
        intersectors.intersectorN.intersect((int*)&mask,intersectors.ptr,context->user,(RTCRayN*)&ray,8,item);
      }
#endif

      /*! Intersects a packet of 16 rays with the scene. */
#if defined(__AVX512F__)
      __forceinline void intersect (const vbool16& valid, Ray16& ray, size_t item, IntersectContext* context) 
      {
        assert(item < size());
        vint16 mask = valid.mask32();
        assert(intersectors.intersectorN.intersect);
        intersectors.intersectorN.intersect((int*)&mask,intersectors.ptr,context->user,(RTCRayN*)&ray,16,item);
      }
#endif
      
      /*! Tests if single ray is occluded by the scene. */
      __forceinline void occluded (Ray& ray, size_t item, IntersectContext* context) 
      {
        int mask = -1;
        assert(intersectors.intersectorN.occluded);          
        intersectors.intersectorN.occluded((int*)&mask,intersectors.ptr,context->user,(RTCRayN*)&ray,1,item);
      }
      
      /*! Tests if a packet of 4 rays is occluded by the scene. */
#if defined(__SSE__)
      __forceinline void occluded (const vbool4& valid, Ray4& ray, size_t item, IntersectContext* context) 
      {
        assert(item < size());
        vint4 mask = valid.mask32();
        assert(intersectors.intersectorN.occluded);          
        intersectors.intersectorN.occluded((int*)&mask,intersectors.ptr,context->user,(RTCRayN*)&ray,4,item);
      }
#endif
      
      /*! Tests if a packet of 8 rays is occluded by the scene. */
#if defined(__AVX__)
      __forceinline void occluded (const vbool8& valid, Ray8& ray, size_t item, IntersectContext* context) 
      {
        assert(item < size());
        vint8 mask = valid.mask32();
        assert(intersectors.intersectorN.occluded);          
        intersectors.intersectorN.occluded((int*)&mask,intersectors.ptr,context->user,(RTCRayN*)&ray,8,item);
      }
#endif
      
      /*! Tests if a packet of 16 rays is occluded by the scene. */
#if defined(__AVX512F__)
      __forceinline void occluded (const vbool16& valid, Ray16& ray, size_t item, IntersectContext* context) 
      {
        assert(item < size());
        vint16 mask = valid.mask32();
        assert(intersectors.intersectorN.occluded);          
        intersectors.intersectorN.occluded((int*)&mask,intersectors.ptr,context->user,(RTCRayN*)&ray,16,item);
      }
#endif


    public:
      RTCBoundsFunc boundsFunc;
      void* boundsFuncUserPtr;

      struct Intersectors 
      {
        Intersectors() : ptr(nullptr) {}
      public:
        void* ptr;
        IntersectorN intersectorN;
      } intersectors;
  };
  
#define DEFINE_SET_INTERSECTORN(symbol,intersector)                     \
  AccelSet::IntersectorN symbol() {                                     \
    return AccelSet::IntersectorN(intersector::intersect, \
                                  intersector::occluded, \
                                  TOSTRING(isa) "::" TOSTRING(symbol)); \
  }
}
