// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#ifndef __EMBREE_ACCELSET_H__
#define __EMBREE_ACCELSET_H__

#include "common/default.h"
#include "common/builder.h"

namespace embree
{
#define DEFINE_SET_INTERSECTOR1(symbol,intersector)                     \
  AccelSet::Intersector1 symbol((IntersectSetFunc)intersector::intersect, \
                                (OccludedSetFunc )intersector::occluded, \
                                TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_SET_INTERSECTOR4(symbol,intersector)                         \
  AccelSet::Intersector4 symbol((IntersectSetFunc4)intersector::intersect, \
                                (OccludedSetFunc4)intersector::occluded, \
                                TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_SET_INTERSECTOR8(symbol,intersector)                         \
  AccelSet::Intersector8 symbol((IntersectSetFunc8)intersector::intersect, \
                                (OccludedSetFunc8)intersector::occluded, \
                                TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_SET_INTERSECTOR16(symbol,intersector)                         \
  AccelSet::Intersector16 symbol((IntersectSetFunc16)intersector::intersect, \
                                 (OccludedSetFunc16)intersector::occluded, \
                                 TOSTRING(isa) "::" TOSTRING(symbol));  

  typedef RTCIntersectFunc IntersectSetFunc;
  typedef RTCIntersectFunc4 IntersectSetFunc4;
  typedef RTCIntersectFunc8 IntersectSetFunc8;
  typedef RTCIntersectFunc16 IntersectSetFunc16;

  typedef RTCOccludedFunc OccludedSetFunc;
  typedef RTCOccludedFunc4 OccludedSetFunc4;
  typedef RTCOccludedFunc8 OccludedSetFunc8;
  typedef RTCOccludedFunc16 OccludedSetFunc16;

  /*! Base class for set of acceleration structures. */
  class AccelSet : public RefCount 
  {
    ALIGNED_CLASS;
  public:
    struct Intersector1
    {
      Intersector1 (ErrorFunc error = NULL) 
      : intersect((IntersectSetFunc)error), occluded((OccludedSetFunc)error), name(NULL) {}

      Intersector1 (IntersectSetFunc intersect, OccludedSetFunc occluded, const char* name)
      : intersect(intersect), occluded(occluded), name(name) {}
      
        operator bool() const { return name; }
        
      public:
        static const char* type;
        const char* name;
        IntersectSetFunc intersect;
        OccludedSetFunc occluded;  
      };
      
      struct Intersector4 
      {
        Intersector4 (ErrorFunc error = NULL) 
        : intersect((IntersectSetFunc4)error), occluded((OccludedSetFunc4)error), name(NULL) {}

        Intersector4 (IntersectSetFunc4 intersect, OccludedSetFunc4 occluded, const char* name)
        : intersect(intersect), occluded(occluded), name(name) {}
        
        operator bool() const { return name; }
        
      public:
        static const char* type;
        const char* name;
        IntersectSetFunc4 intersect;
        OccludedSetFunc4 occluded;
      };
      
      struct Intersector8 
      {
        Intersector8 (ErrorFunc error = NULL) 
        : intersect((IntersectSetFunc8)error), occluded((OccludedSetFunc8)error), name(NULL) {}

        Intersector8 (IntersectSetFunc8 intersect, OccludedSetFunc8 occluded, const char* name)
        : intersect(intersect), occluded(occluded), name(name) {}
        
        operator bool() const { return name; }
        
      public:
        static const char* type;
        const char* name;
        IntersectSetFunc8 intersect;
        OccludedSetFunc8 occluded;
      };
      
      struct Intersector16 
      {
        Intersector16 (ErrorFunc error = NULL) 
        : intersect((IntersectSetFunc16)error), occluded((OccludedSetFunc16)error), name(NULL) {}

        Intersector16 (IntersectSetFunc16 intersect, OccludedSetFunc16 occluded, const char* name)
        : intersect(intersect), occluded(occluded), name(name) {}
        
        operator bool() const { return name; }
        
      public:
        static const char* type;
        const char* name;
        IntersectSetFunc16 intersect;
        OccludedSetFunc16 occluded;
      };
      
    public:
      
      /*! Construction */
      AccelSet (size_t numItems) : numItems(numItems) {
        intersectors.ptr = NULL; 
        intersectors.boundsPtr = NULL;
      }
      
      /*! Virtual destructor */
      virtual ~AccelSet() {}
      
      /*! makes the acceleration structure immutable */
      virtual void immutable () {};
      
      /*! build accel */
      virtual void build (size_t threadIndex, size_t threadCount) = 0;

      /*! return number of items in set */
      __forceinline size_t size() const {
        return numItems;
      }

      /*! Calculates the bounds of an item */
      __forceinline BBox3f bounds (size_t item) 
      {
        BBox3f box; 
        boundsFunc(intersectors.boundsPtr,item,(RTCBounds&)box);
        return box;
      }
      
      /*! Intersects a single ray with the scene. */
      __forceinline void intersect (RTCRay& ray, size_t item) {
        assert(intersectors.intersector1.intersect);
        intersectors.intersector1.intersect(intersectors.ptr,ray,item);
      }
      
      /*! Intersects a packet of 4 rays with the scene. */
      __forceinline void intersect4 (const void* valid, RTCRay4& ray, size_t item) {
        assert(intersectors.intersector4.intersect);
        intersectors.intersector4.intersect(valid,intersectors.ptr,ray,item);
      }
      
      /*! Intersects a packet of 8 rays with the scene. */
      __forceinline void intersect8 (const void* valid, RTCRay8& ray, size_t item) {
        assert(intersectors.intersector8.intersect);
        intersectors.intersector8.intersect(valid,intersectors.ptr,ray,item);
      }

      /*! Intersects a packet of 16 rays with the scene. */
      __forceinline void intersect16 (const void* valid, RTCRay16& ray, size_t item) {
        assert(intersectors.intersector16.intersect);
        intersectors.intersector16.intersect(valid,intersectors.ptr,ray,item);
      }
      
      /*! Tests if single ray is occluded by the scene. */
      __forceinline void occluded (RTCRay& ray, size_t item) {
        assert(intersectors.intersector1.occluded);
        intersectors.intersector1.occluded(intersectors.ptr,ray,item);
      }
      
      /*! Tests if a packet of 4 rays is occluded by the scene. */
      __forceinline void occluded4 (const void* valid, RTCRay4& ray, size_t item) {
        assert(intersectors.intersector4.occluded);
        intersectors.intersector4.occluded(valid,intersectors.ptr,ray,item);
      }
      
      /*! Tests if a packet of 8 rays is occluded by the scene. */
      __forceinline void occluded8 (const void* valid, RTCRay8& ray, size_t item) {
        assert(intersectors.intersector8.occluded);
        intersectors.intersector8.occluded(valid,intersectors.ptr,ray,item);
      }
      
      /*! Tests if a packet of 16 rays is occluded by the scene. */
      __forceinline void occluded16 (const void* valid, RTCRay16& ray, size_t item) {
        assert(intersectors.intersector16.occluded);
        intersectors.intersector16.occluded(valid,intersectors.ptr,ray,item);
      }
      
    public:
      size_t numItems;
      RTCBoundsFunc boundsFunc;

      struct Intersectors 
      {
      public:
        void* ptr;
        void* boundsPtr;
        Intersector1 intersector1;
        Intersector4 intersector4;
        Intersector8 intersector8;
        Intersector16 intersector16;
      } intersectors;
  };

  struct AccelSetItem {
    AccelSet* accel;
    size_t item;
  };
}

#endif
