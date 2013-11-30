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

#ifndef __EMBREE_ACCEL_H__
#define __EMBREE_ACCEL_H__

#include "common/default.h"
#include "builders/builder.h"

namespace embree
{
  /*! Base class for bounded geometry. */
  class Bounded : public RefCount {
  public:
    Bounded () : bounds(empty) {}
  public:
    BBox3f bounds;
  };
  
  /*! Base class for all intersectable and buildable acceleration structures. */
  class Accel : public Bounded
  {
    ALIGNED_CLASS;
  public:
    struct Intersector1
    {
      Intersector1 (RTCIntersectFunc intersect = NULL, RTCOccludedFunc occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}

      operator bool() const { return intersect != NULL && occluded != NULL; }

    public:
      static const char* type;
      const char* name;
      RTCIntersectFunc intersect;
      RTCOccludedFunc occluded;  
    };
    
    struct Intersector4 
    {
      Intersector4 (RTCIntersectFunc4 intersect = NULL, RTCOccludedFunc4 occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}

      operator bool() const { return intersect != NULL && occluded != NULL; }
      
    public:
      static const char* type;
      const char* name;
      RTCIntersectFunc4 intersect;
      RTCOccludedFunc4 occluded;
    };
    
    struct Intersector8 
    {
      Intersector8 (RTCIntersectFunc8 intersect = NULL, RTCOccludedFunc8 occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}

      operator bool() const { return intersect != NULL && occluded != NULL; }
      
    public:
      static const char* type;
      const char* name;
      RTCIntersectFunc8 intersect;
      RTCOccludedFunc8 occluded;
    };
    
    struct Intersector16 
    {
      Intersector16 (RTCIntersectFunc16 intersect = NULL, RTCOccludedFunc16 occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}

      operator bool() const { return intersect != NULL && occluded != NULL; }
      
    public:
      static const char* type;
      const char* name;
      RTCIntersectFunc16 intersect;
      RTCOccludedFunc16 occluded;
    };
  
  public:

    /*! Construction */
    Accel () { intersectors.ptr = NULL; }

    /*! Virtual destructor */
    virtual ~Accel() {}

    /*! makes the acceleration structure immutable */
    virtual void immutable () {};
    
    /*! build accel */
    virtual void build (size_t threadIndex, size_t threadCount) = 0;
    
    /*! Intersects a single ray with the scene. */
    __forceinline void intersect (RTCRay& ray) {
	  assert(intersectors.intersector1.intersect);
      intersectors.intersector1.intersect(intersectors.ptr,ray);
    }

    /*! Intersects a packet of 4 rays with the scene. */
    __forceinline void intersect4 (const void* valid, RTCRay4& ray) {
	  assert(intersectors.intersector4.intersect);
      intersectors.intersector4.intersect(valid,intersectors.ptr,ray);
    }

    /*! Intersects a packet of 8 rays with the scene. */
    __forceinline void intersect8 (const void* valid, RTCRay8& ray) {
	  assert(intersectors.intersector8.intersect);
      intersectors.intersector8.intersect(valid,intersectors.ptr,ray);
    }

    /*! Intersects a packet of 16 rays with the scene. */
    __forceinline void intersect16 (const void* valid, RTCRay16& ray) {
	  assert(intersectors.intersector16.intersect);
      intersectors.intersector16.intersect(valid,intersectors.ptr,ray);
    }

    /*! Tests if single ray is occluded by the scene. */
    __forceinline void occluded (RTCRay& ray) {
	  assert(intersectors.intersector1.occluded);
      intersectors.intersector1.occluded(intersectors.ptr,ray);
    }
    
    /*! Tests if a packet of 4 rays is occluded by the scene. */
    __forceinline void occluded4 (const void* valid, RTCRay4& ray) {
      assert(intersectors.intersector4.occluded);
      intersectors.intersector4.occluded(valid,intersectors.ptr,ray);
    }

    /*! Tests if a packet of 8 rays is occluded by the scene. */
    __forceinline void occluded8 (const void* valid, RTCRay8& ray) {
	  assert(intersectors.intersector8.occluded);
      intersectors.intersector8.occluded(valid,intersectors.ptr,ray);
    }

    /*! Tests if a packet of 16 rays is occluded by the scene. */
    __forceinline void occluded16 (const void* valid, RTCRay16& ray) {
	  assert(intersectors.intersector16.occluded);
      intersectors.intersector16.occluded(valid,intersectors.ptr,ray);
    }

  public:
    struct Intersectors 
    {
      Intersectors () {}

      void print(size_t ident) 
      {
        for (size_t i=0; i<ident; i++) std::cout << " ";
        std::cout << "intersector1  = " << intersector1.name << std::endl;
        for (size_t i=0; i<ident; i++) std::cout << " ";
        std::cout << "intersector4  = " << intersector4.name << std::endl;
        for (size_t i=0; i<ident; i++) std::cout << " ";
        std::cout << "intersector8  = " << intersector8.name << std::endl;
        for (size_t i=0; i<ident; i++) std::cout << " ";
        std::cout << "intersector16 = " << intersector16.name << std::endl;
      }

    public:
      void* ptr;
      Intersector1 intersector1;
      Intersector4 intersector4;
      Intersector8 intersector8;
      Intersector16 intersector16;
    } intersectors;
  };

  class AccelInstance : public Accel
  {
  public:
    AccelInstance (Bounded* accel, Builder* builder, Intersectors& intersectors)
      : accel(accel), builder(builder) 
    {
      this->intersectors = intersectors;
    }

    void immutable () {
      delete builder; builder = NULL;
    }

    ~AccelInstance() {
      delete builder; builder = NULL; // delete builder first!
      delete accel; accel = NULL;
    }

  public:
    void build (size_t threadIndex, size_t threadCount) {
      if (builder) builder->build(threadIndex,threadCount);
      bounds = accel->bounds;
    }

  private:
    Bounded* accel;
    Builder* builder;
  };
}

#endif
