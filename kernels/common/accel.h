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
#define DECLARE_BOUNDS_FUNC(bounds) \
  namespace isa { extern RTCBoundsFunc bounds; }   \
  namespace sse41 { extern RTCBoundsFunc bounds; } \
  namespace avx { extern RTCBoundsFunc bounds; }   \
  namespace avx2 { extern RTCBoundsFunc bounds; }  \
  void bounds##_error() { std::cerr << "Error: bounds " << TOSTRING(bounds) << " not supported no your CPU" << std::endl; } \
  RTCBoundsFunc bounds((RTCBoundsFunc)bounds##_error);

#define DECLARE_SET_INTERSECTOR1(intersector) \
  namespace isa { extern AccelSet::Intersector1 intersector; }   \
  namespace sse41 { extern AccelSet::Intersector1 intersector; } \
  namespace avx { extern AccelSet::Intersector1 intersector; }   \
  namespace avx2 { extern AccelSet::Intersector1 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  AccelSet::Intersector1 intersector((IntersectSetFunc)intersector##_error,(OccludedSetFunc)intersector##_error);

#define DECLARE_SET_INTERSECTOR4(intersector) \
  namespace isa { extern AccelSet::Intersector4 intersector; }   \
  namespace sse41 { extern AccelSet::Intersector4 intersector; } \
  namespace avx { extern AccelSet::Intersector4 intersector; }   \
  namespace avx2 { extern AccelSet::Intersector4 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  AccelSet::Intersector4 intersector((IntersectSetFunc4)intersector##_error,(OccludedSetFunc4)intersector##_error);

#define DECLARE_SET_INTERSECTOR8(intersector) \
  namespace isa { extern AccelSet::Intersector8 intersector; }   \
  namespace sse41 { extern AccelSet::Intersector8 intersector; } \
  namespace avx { extern AccelSet::Intersector8 intersector; }   \
  namespace avx2 { extern AccelSet::Intersector8 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  AccelSet::Intersector8 intersector((IntersectSetFunc8)intersector##_error,(OccludedSetFunc8)intersector##_error);

#define DECLARE_SET_INTERSECTOR16(intersector) \
  namespace isa { extern AccelSet::Intersector16 intersector; }   \
  namespace sse41 { extern AccelSet::Intersector16 intersector; } \
  namespace avx { extern AccelSet::Intersector16 intersector; }   \
  namespace avx2 { extern AccelSet::Intersector16 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  AccelSet::Intersector16 intersector((IntersectSetFunc16)intersector##_error,(OccludedSetFunc16)intersector##_error);

#define DECLARE_INTERSECTOR1(intersector) \
  namespace isa { extern Accel::Intersector1 intersector; }   \
  namespace sse41 { extern Accel::Intersector1 intersector; } \
  namespace avx { extern Accel::Intersector1 intersector; }   \
  namespace avx2 { extern Accel::Intersector1 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  Accel::Intersector1 intersector((IntersectFunc)intersector##_error,(OccludedFunc)intersector##_error);

#define DECLARE_INTERSECTOR4(intersector) \
  namespace isa { extern Accel::Intersector4 intersector; }   \
  namespace sse41 { extern Accel::Intersector4 intersector; } \
  namespace avx { extern Accel::Intersector4 intersector; }   \
  namespace avx2 { extern Accel::Intersector4 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  Accel::Intersector4 intersector((IntersectFunc4)intersector##_error,(OccludedFunc4)intersector##_error);

#define DECLARE_INTERSECTOR8(intersector) \
  namespace isa { extern Accel::Intersector8 intersector; }   \
  namespace sse41 { extern Accel::Intersector8 intersector; } \
  namespace avx { extern Accel::Intersector8 intersector; }   \
  namespace avx2 { extern Accel::Intersector8 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  Accel::Intersector8 intersector((IntersectFunc8)intersector##_error,(OccludedFunc8)intersector##_error);

#define DECLARE_INTERSECTOR16(intersector) \
  namespace isa { extern Accel::Intersector16 intersector; }   \
  namespace sse41 { extern Accel::Intersector16 intersector; } \
  namespace avx { extern Accel::Intersector16 intersector; }   \
  namespace avx2 { extern Accel::Intersector16 intersector; }  \
  void intersector##_error() { std::cerr << "Error: intersector " << TOSTRING(intersector) << " not supported no your CPU" << std::endl; } \
  Accel::Intersector16 intersector((IntersectFunc16)intersector##_error,(OccludedFunc16)intersector##_error);

#define DEFINE_SET_INTERSECTOR1(symbol,intersector)                        \
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

#define DEFINE_INTERSECTOR1(symbol,intersector)                        \
  Accel::Intersector1 symbol((IntersectFunc)intersector::intersect, \
                             (OccludedFunc )intersector::occluded,  \
                             TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR4(symbol,intersector)                         \
  Accel::Intersector4 symbol((IntersectFunc4)intersector::intersect, \
                             (OccludedFunc4)intersector::occluded,   \
                             TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR4TO8(symbol,intersector,BVH)                  \
  Accel::Intersector8 symbol((IntersectFunc8)Intersector4To8<BVH,intersector >::intersect8, \
                             (OccludedFunc8 )Intersector4To8<BVH,intersector >::occluded8, \
                             TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR4TO16(symbol,intersector,BVH)                 \
  Accel::Intersector16 symbol((IntersectFunc16)Intersector4To16<BVH,intersector >::intersect16, \
                              (OccludedFunc16 )Intersector4To16<BVH,intersector >::occluded16, \
                              TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR8(symbol,intersector)                         \
  Accel::Intersector8 symbol((IntersectFunc8)intersector::intersect, \
                             (OccludedFunc8)intersector::occluded,   \
                             TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR8TO16(symbol,intersector,BVH)                 \
  Accel::Intersector16 symbol((IntersectFunc16)Intersector8To16<BVH,intersector >::intersect16, \
                              (OccludedFunc16 )Intersector8To16<BVH,intersector >::occluded16, \
                              TOSTRING(isa) "::" TOSTRING(symbol));

#define DEFINE_INTERSECTOR16(symbol,intersector)                         \
  Accel::Intersector16 symbol((IntersectFunc16)intersector::intersect, \
                              (OccludedFunc16)intersector::occluded,\
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

  /*! Type of intersect function pointer for single rays. */
  typedef void (*IntersectFunc)(void* ptr,           /*!< pointer to user data */
                                RTCRay& ray          /*!< ray to intersect */);
  
  /*! Type of intersect function pointer for ray packets of size 4. */
  typedef void (*IntersectFunc4)(const void* valid,  /*!< pointer to valid mask */
                                 void* ptr,          /*!< pointer to user data */
                                 RTCRay4& ray        /*!< ray packet to intersect */);
  
  /*! Type of intersect function pointer for ray packets of size 8. */
  typedef void (*IntersectFunc8)(const void* valid,  /*!< pointer to valid mask */
                                 void* ptr,          /*!< pointer to user data */
                                 RTCRay8& ray        /*!< ray packet to intersect */);
  
  /*! Type of intersect function pointer for ray packets of size 16. */
  typedef void (*IntersectFunc16)(const void* valid, /*!< pointer to valid mask */
                                  void* ptr,         /*!< pointer to user data */
                                  RTCRay16& ray      /*!< ray packet to intersect */);
  
  /*! Type of occlusion function pointer for single rays. */
  typedef void (*OccludedFunc) (void* ptr,           /*!< pointer to user data */ 
                                RTCRay& ray          /*!< ray to test occlusion */);
  
  /*! Type of occlusion function pointer for ray packets of size 4. */
  typedef void (*OccludedFunc4) (const void* valid,  /*! pointer to valid mask */
                                 void* ptr,          /*!< pointer to user data */
                                 RTCRay4& ray        /*!< Ray packet to test occlusion. */);
  
  /*! Type of occlusion function pointer for ray packets of size 8. */
  typedef void (*OccludedFunc8) (const void* valid,  /*! pointer to valid mask */
                                 void* ptr,          /*!< pointer to user data */
                                 RTCRay8& ray        /*!< Ray packet to test occlusion. */);
  
  /*! Type of occlusion function pointer for ray packets of size 16. */
  typedef void (*OccludedFunc16) (const void* valid, /*! pointer to valid mask */
                                  void* ptr,         /*!< pointer to user data */
                                  RTCRay16& ray      /*!< Ray packet to test occlusion. */);
  
  /*! Base class for all intersectable and buildable acceleration structures. */
  class Accel : public Bounded
  {
    ALIGNED_CLASS;
  public:
    struct Intersector1
    {
      Intersector1 (IntersectFunc intersect = NULL, OccludedFunc occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}

      operator bool() const { return intersect != NULL && occluded != NULL; }

    public:
      static const char* type;
      const char* name;
      IntersectFunc intersect;
      OccludedFunc occluded;  
    };
    
    struct Intersector4 
    {
      Intersector4 (IntersectFunc4 intersect = NULL, OccludedFunc4 occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}

      operator bool() const { return intersect != NULL && occluded != NULL; }
      
    public:
      static const char* type;
      const char* name;
      IntersectFunc4 intersect;
      OccludedFunc4 occluded;
    };
    
    struct Intersector8 
    {
      Intersector8 (IntersectFunc8 intersect = NULL, OccludedFunc8 occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}

      operator bool() const { return intersect != NULL && occluded != NULL; }
      
    public:
      static const char* type;
      const char* name;
      IntersectFunc8 intersect;
      OccludedFunc8 occluded;
    };
    
    struct Intersector16 
    {
      Intersector16 (IntersectFunc16 intersect = NULL, OccludedFunc16 occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}

      operator bool() const { return intersect != NULL && occluded != NULL; }
      
    public:
      static const char* type;
      const char* name;
      IntersectFunc16 intersect;
      OccludedFunc16 occluded;
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
      Intersector1 (IntersectSetFunc intersect = NULL, OccludedSetFunc occluded = NULL, const char* name = "unknown")
      : intersect(intersect), occluded(occluded), name(name) {}
      
        operator bool() const { return intersect != NULL && occluded != NULL; }
        
      public:
        static const char* type;
        const char* name;
        IntersectSetFunc intersect;
        OccludedSetFunc occluded;  
      };
      
      struct Intersector4 
      {
        Intersector4 (IntersectSetFunc4 intersect = NULL, OccludedSetFunc4 occluded = NULL, const char* name = "unknown")
        : intersect(intersect), occluded(occluded), name(name) {}
        
        operator bool() const { return intersect != NULL && occluded != NULL; }
        
      public:
        static const char* type;
        const char* name;
        IntersectSetFunc4 intersect;
        OccludedSetFunc4 occluded;
      };
      
      struct Intersector8 
      {
        Intersector8 (IntersectSetFunc8 intersect = NULL, OccludedSetFunc8 occluded = NULL, const char* name = "unknown")
        : intersect(intersect), occluded(occluded), name(name) {}
        
        operator bool() const { return intersect != NULL && occluded != NULL; }
        
      public:
        static const char* type;
        const char* name;
        IntersectSetFunc8 intersect;
        OccludedSetFunc8 occluded;
      };
      
      struct Intersector16 
      {
        Intersector16 (IntersectSetFunc16 intersect = NULL, OccludedSetFunc16 occluded = NULL, const char* name = "unknown")
        : intersect(intersect), occluded(occluded), name(name) {}
        
        operator bool() const { return intersect != NULL && occluded != NULL; }
        
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
