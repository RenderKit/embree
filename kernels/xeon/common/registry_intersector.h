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

#ifndef __EMBREE_INTERSECTOR_REGISTRY_H__
#define __EMBREE_INTERSECTOR_REGISTRY_H__

#include "default.h"
#include "primitive.h"
#include "common/accel.h"
#include "embree2/rtcore.h"

namespace embree
{
  /*! Registry for intersectors */
  template<typename T>
    class IntersectorRegistry 
  {
  public:

    struct IntersectorImplementation 
    {
      IntersectorImplementation (int cpu_features, T intersector)
      : cpu_features(cpu_features), intersector(intersector) {}

      static bool compare (const IntersectorImplementation& a, const IntersectorImplementation& b) {
        return a.cpu_features > b.cpu_features;
      }

    public:
      int cpu_features;
      T intersector;
    };

    /*! adds a new intersector */
    void add(int cpu_features, const std::string& name, const T intersector);
    
    /*! get registered intersector by name */
    T get(std::string name);

    /*! prints the registry */
    void print();
    
    /*! clears the registry */
    void clear();

  private:
    std::map<std::string, std::vector<IntersectorImplementation> > table; 
  }; 

  /*! intersector registrys */
  extern IntersectorRegistry<Accel::Intersector1> intersectors1;
  extern IntersectorRegistry<Accel::Intersector4> intersectors4;
  extern IntersectorRegistry<Accel::Intersector8> intersectors8;
  extern IntersectorRegistry<Accel::Intersector16> intersectors16;

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

}
#endif
