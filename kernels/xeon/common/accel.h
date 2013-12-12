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
#define DECLARE_INTERSECTOR1(intersector) \
  namespace isa { extern Accel::Intersector1 intersector; }   \
  namespace sse41 { extern Accel::Intersector1 intersector; } \
  namespace avx { extern Accel::Intersector1 intersector; }   \
  namespace avx2 { extern Accel::Intersector1 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  Accel::Intersector1 intersector((RTCIntersectFunc)intersector##_error,(RTCOccludedFunc)intersector##_error);

#define DECLARE_INTERSECTOR4(intersector) \
  namespace isa { extern Accel::Intersector4 intersector; }   \
  namespace sse41 { extern Accel::Intersector4 intersector; } \
  namespace avx { extern Accel::Intersector4 intersector; }   \
  namespace avx2 { extern Accel::Intersector4 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  Accel::Intersector4 intersector((RTCIntersectFunc4)intersector##_error,(RTCOccludedFunc4)intersector##_error);

#define DECLARE_INTERSECTOR8(intersector) \
  namespace isa { extern Accel::Intersector8 intersector; }   \
  namespace sse41 { extern Accel::Intersector8 intersector; } \
  namespace avx { extern Accel::Intersector8 intersector; }   \
  namespace avx2 { extern Accel::Intersector8 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  Accel::Intersector8 intersector((RTCIntersectFunc8)intersector##_error,(RTCOccludedFunc8)intersector##_error);

#define DECLARE_INTERSECTOR16(intersector) \
  namespace isa { extern Accel::Intersector16 intersector; }   \
  namespace sse41 { extern Accel::Intersector16 intersector; } \
  namespace avx { extern Accel::Intersector16 intersector; }   \
  namespace avx2 { extern Accel::Intersector16 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  Accel::Intersector16 intersector((RTCIntersectFunc16)intersector##_error,(RTCOccludedFunc16)intersector##_error);

#define DEFINE_INTERSECTOR1(symbol,intersector)                        \
  Accel::Intersector1 symbol((RTCIntersectFunc)intersector::intersect, \
                             (RTCOccludedFunc )intersector::occluded,  \
                             TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR4(symbol,intersector)                         \
  Accel::Intersector4 symbol((RTCIntersectFunc4)intersector::intersect, \
                             (RTCOccludedFunc4)intersector::occluded,   \
                             TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR4TO8(symbol,intersector,BVH)                  \
  Accel::Intersector8 symbol((RTCIntersectFunc8)Intersector4To8<BVH,intersector >::intersect8, \
                             (RTCOccludedFunc8 )Intersector4To8<BVH,intersector >::occluded8, \
                             TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR4TO16(symbol,intersector,BVH)                 \
  Accel::Intersector16 symbol((RTCIntersectFunc16)Intersector4To16<BVH,intersector >::intersect16, \
                              (RTCOccludedFunc16 )Intersector4To16<BVH,intersector >::occluded16, \
                              TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR8(symbol,intersector)                         \
  Accel::Intersector8 symbol((RTCIntersectFunc8)intersector::intersect, \
                             (RTCOccludedFunc8)intersector::occluded,   \
                             TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR8TO16(symbol,intersector,BVH)                 \
  Accel::Intersector16 symbol((RTCIntersectFunc16)Intersector8To16<BVH,intersector >::intersect16, \
                              (RTCOccludedFunc16 )Intersector8To16<BVH,intersector >::occluded16, \
                              TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR16(symbol,intersector)                         \
  Accel::Intersector16 symbol((RTCIntersectFunc16)intersector::intersect, \
                              (RTCOccludedFunc16)intersector::occluded,\
                              TOSTRING(isa) "::" TOSTRING(symbol));

#define SELECT_DEFAULT(features,intersector) \
  intersector = isa::intersector;

#if defined(__TARGET_SSE41__)
#define SELECT_SSE41(features,intersector) \
  if ((features & SSE41) == SSE41) intersector = sse41::intersector;
#else
#define SELECT_SSE41(features,intersector)
#endif

#if defined(__TARGET_AVX__)
#define SELECT_AVX(features,intersector) \
  if ((features & AVX) == AVX) intersector = avx::intersector;
#else
#define SELECT_AVX(features,intersector)
#endif

#if defined(__TARGET_AVX2__)
#define SELECT_AVX2(features,intersector) \
  if ((features & AVX2) == AVX2) intersector = avx2::intersector;
#else
#define SELECT_AVX2(features,intersector)
#endif

#if defined(__MIC__)
#define SELECT_KNC(features,intersector) \
  intersector = knc::intersector;
#else
#define SELECT_KNC(features,intersector)
#endif

#define SELECT_TEST(intersector)                               \
  if (!intersector) printf("WARNING: could not select code for " TOSTRING(intersector) "\n")
  
#define SELECT_DEFAULT_SSE41(features,intersector) \
  SELECT_DEFAULT(features,intersector);                                 \
  SELECT_SSE41(features,intersector);                                   \
  SELECT_TEST(intersector);

#define SELECT_DEFAULT_AVX(features,intersector) \
  SELECT_DEFAULT(features,intersector);                     \
  SELECT_AVX(features,intersector);                         \
  SELECT_TEST(intersector);

#define SELECT_AVX_AVX2(features,intersector) \
  SELECT_AVX(features,intersector);                         \
  SELECT_AVX2(features,intersector);

#define SELECT_DEFAULT_AVX_AVX2(features,intersector) \
  SELECT_DEFAULT(features,intersector);                     \
  SELECT_AVX(features,intersector);                         \
  SELECT_AVX2(features,intersector);                        \
  SELECT_TEST(intersector);

#define SELECT_DEFAULT_SSE41_AVX_AVX2(features,intersector) \
  SELECT_DEFAULT(features,intersector);                     \
  SELECT_SSE41(features,intersector);                       \
  SELECT_AVX(features,intersector);                         \
  SELECT_AVX2(features,intersector);                        \
  SELECT_TEST(intersector);

#define SELECT_DEFAULT_SSE41_AVX(features,intersector) \
  SELECT_DEFAULT(features,intersector);                     \
  SELECT_SSE41(features,intersector);                       \
  SELECT_AVX(features,intersector);                         \
  SELECT_TEST(intersector);

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
