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

#ifdef _WIN32
#  define RTCORE_API extern "C" //__declspec(dllexport)
#else
#  define RTCORE_API extern "C" __attribute__ ((visibility ("default")))
#endif

#include "common/default.h"
#include "embree2/rtcore.h"
#include "rtcore/scene.h"
#include "sys/taskscheduler.h"
#include "sys/thread.h"

#include "common/registry_intersector.h"
#include "common/registry_builder.h"
#include "common/alloc.h"

#if defined(__EXIT_ON_ERROR__)
#define VERBOSE 1
#else
#define VERBOSE g_verbose
#endif

#define TRACE(x) //std::cout << #x << std::endl;

namespace embree
{
#define CATCH_BEGIN try {
#define CATCH_END                                                       \
  } catch (std::bad_alloc&) {                                         \
  if (VERBOSE) std::cerr << "RTCore: Out of memory" << std::endl;     \
  recordError(RTC_OUT_OF_MEMORY);                                       \
 } catch (std::exception& e) {                                          \
  if (VERBOSE) std::cerr << "RTCore: " << e.what() << std::endl;      \
  recordError(RTC_UNKNOWN_ERROR);                                       \
 } catch (...) {                                                        \
  if (VERBOSE) std::cerr << "RTCore: Unknown exception caught." << std::endl; \
  recordError(RTC_UNKNOWN_ERROR);                                       \
 }

#define VERIFY_HANDLE(handle) \
  if (handle == NULL) {                                                 \
    if (VERBOSE) std::cerr << "RTCore: invalid argument" << std::endl; \
    recordError(RTC_INVALID_ARGUMENT);                                  \
  }
  
  /* register functions for accels */
  void BVH4Register();
  void BVH4iRegister();
  void BVH8iRegister();
  void BVH4MBRegister();

  /*! intersector registration functions */
  DECLARE_INTERSECTOR1(InstanceIntersector1);
  DECLARE_INTERSECTOR4(InstanceIntersector4);
  DECLARE_INTERSECTOR8(InstanceIntersector8);
  DECLARE_INTERSECTOR16(InstanceIntersector16);
  
  /* global settings */
  std::string g_top_accel = "default";    //!< toplevel acceleration structure to use
  std::string g_tri_accel = "default";    //!< triangle acceleration structure to use
  std::string g_builder = "default";      //!< builder to use
  std::string g_traverser = "default";    //!< traverser to use
  size_t g_verbose = 0;                   //!< verbosity of output
  size_t g_numThreads = 0;                //!< number of threads to use in builders
  size_t g_benchmark = 0;

  /* error flag */
  static tls_t g_error = NULL;
  std::vector<RTCError*> g_errors;
  
  /* mutex to make API thread safe */
  static MutexSys g_mutex;

  /* set if embree got initialized */
  static bool g_initialized = false;

  void skipSpace(const char* str, size_t& pos) {
    while (str[pos] == ' ') pos++;
  }

  int parseInt(const char* str, size_t& pos) 
  {
    skipSpace(str,pos);
    size_t begin = pos;
    while (isdigit(str[pos])) pos++;
    return atoi(str+begin);
  }

  std::string parseIdentifier(const char* str, size_t& pos) 
  {
    skipSpace(str,pos);
    size_t begin = pos;
    while (isalnum(str[pos]) || str[pos] == '_' || str[pos] == '.') pos++;
    return std::string(str+begin,str+pos);
  }

  bool parseSymbol(const char* str, char c, size_t& pos) 
  {
    skipSpace(str,pos);
    if (str[pos] == c) { pos++; return true; }
    return false;
  }

  bool findNext(const char* str, char c, size_t& pos) 
  {
    while (str[pos] && str[pos] != c) pos++;
    if (str[pos] == c) { pos++; return true; }
    else return false;
  }

  RTCORE_API void rtcInit(const char* cfg) 
  {
    Lock<MutexSys> lock(g_mutex);
    TRACE(rtcInit);
    CATCH_BEGIN;

    if (g_initialized) {
      recordError(RTC_INVALID_OPERATION);
      return;
    }
    g_initialized = true;

    if (cfg != NULL) 
    {
      size_t pos = 0;
      do {
        std::string tok = parseIdentifier (cfg,pos);

        if (tok == "threads") {
          if (parseSymbol(cfg,'=',pos))
            g_numThreads = parseInt(cfg,pos);
        }
        else if (tok == "isa") {
          if (parseSymbol (cfg,'=',pos)) {
            std::string isa = parseIdentifier (cfg,pos);
            if      (isa == "sse" ) cpu_features = SSE;
            else if (isa == "sse2") cpu_features = SSE2;
            else if (isa == "sse3") cpu_features = SSE3;
            else if (isa == "ssse3") cpu_features = SSSE3;
            else if (isa == "sse41") cpu_features = SSE41;
            else if (isa == "sse42") cpu_features = SSE42;
            else if (isa == "avx") cpu_features = AVX;
            else if (isa == "avxi") cpu_features = AVXI;
            else if (isa == "avx2") cpu_features = AVX2;
          }
        }
        else if (tok == "accel") {
          if (parseSymbol (cfg,'=',pos))
            g_tri_accel = parseIdentifier (cfg,pos);
        } 
        else if (tok == "triaccel") {
          if (parseSymbol (cfg,'=',pos))
            g_tri_accel = parseIdentifier (cfg,pos);
        } 
        else if (tok == "topaccel") {
          if (parseSymbol (cfg,'=',pos))
            g_top_accel = parseIdentifier (cfg,pos);
        } 
        else if (tok == "builder") {
          if (parseSymbol (cfg,'=',pos))
            g_builder = parseIdentifier (cfg,pos);
        }
        else if (tok == "traverser") {
          if (parseSymbol (cfg,'=',pos))
            g_traverser = parseIdentifier (cfg,pos);
        }
        else if (tok == "verbose") {
          if (parseSymbol (cfg,'=',pos))
            g_verbose = parseInt (cfg,pos);
        }
        else if (tok == "benchmark") {
          if (parseSymbol (cfg,'=',pos))
            g_benchmark = parseInt (cfg,pos);
        }
        
      } while (findNext (cfg,',',pos));
    }

    g_error = createTls();

#if !defined(__MIC__)
    BVH4Register();
#endif
    BVH4MBRegister();
    BVH4iRegister();
    
#if defined(__TARGET_AVX__)
    BVH8iRegister();
#endif

    int features = getCPUFeatures();
    SELECT_DEFAULT_AVX_AVX2(features,InstanceIntersector1);
    SELECT_DEFAULT_AVX_AVX2(features,InstanceIntersector4);
    SELECT_DEFAULT_AVX_AVX2(features,InstanceIntersector8);
    SELECT_DEFAULT_AVX_AVX2(features,InstanceIntersector16);
#if defined(__MIC__)
    SELECT_KNC(features,InstanceIntersector16);
#endif
    
    if (g_verbose) 
    {
      PRINT(cfg);
      PRINT(g_numThreads);
      PRINT(g_verbose);
      PRINT(g_top_accel);
      PRINT(g_tri_accel);
      PRINT(g_builder);
      PRINT(g_traverser);
      std::cout << "cpu features = " << stringOfCPUFeatures(getCPUFeatures()) << std::endl;
      builders.print();
    }
    TaskScheduler::create(g_numThreads);

    CATCH_END;
  }
  
  RTCORE_API void rtcExit() 
  {
    Lock<MutexSys> lock(g_mutex);
    TRACE(rtcExit);
    CATCH_BEGIN;
    if (!g_initialized) {
      return;
    }
    TaskScheduler::destroy();
    for (size_t i=0; i<g_errors.size(); i++)
      delete g_errors[i];
    destroyTls(g_error);
    Alloc::global.clear();
    CATCH_END;
  }

  RTCError* getThreadError() 
  {
    RTCError* stored_error = (RTCError*) getTls(g_error);
    if (stored_error == NULL) {
      Lock<MutexSys> lock(g_mutex);
      stored_error = new RTCError(RTC_NO_ERROR);
      g_errors.push_back(stored_error);
      setTls(g_error,stored_error);
    }
    return stored_error;
  }

  void recordError(RTCError error)
  {    
    RTCError* stored_error = getThreadError();

    if (VERBOSE) 
    {
      switch (error) {
      case RTC_NO_ERROR         : std::cerr << "RTCore: No error" << std::endl; break;
      case RTC_UNKNOWN_ERROR    : std::cerr << "RTCore: Unknown error" << std::endl; break;
      case RTC_INVALID_ARGUMENT : std::cerr << "RTCore: Invalid argument" << std::endl; break;
      case RTC_INVALID_OPERATION: std::cerr << "RTCore: Invalid operation" << std::endl; break;
      case RTC_OUT_OF_MEMORY    : std::cerr << "RTCore: Out of memory" << std::endl; break;
      };
    }
    if (*stored_error == RTC_NO_ERROR)
      *stored_error = error;

#if defined(__EXIT_ON_ERROR__)
    exit(error);
#endif
  }

  RTCORE_API RTCError rtcGetError() 
  {
    TRACE(rtcGetError);
    RTCError* stored_error = getThreadError();
    RTCError error = *stored_error;
    *stored_error = RTC_NO_ERROR;
    return error;
  }

  RTCORE_API void rtcDebug()
  {
    Lock<MutexSys> lock(g_mutex);

    TRACE(rtcDebug);
#if defined(__USE_STAT_COUNTERS__)
    Stat::print(std::cout);
    Stat::clear();
#endif
  }
  
  RTCORE_API RTCScene rtcNewScene (RTCFlags flags, RTCAlgorithmFlags aflags) 
  {
    CATCH_BEGIN;
    TRACE(rtcNewScene);
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCFlags(flags | RTC_INCOHERENT);
    if (isDeformable(flags)) {
      recordError(RTC_INVALID_ARGUMENT);
      return NULL;
    }
    return (RTCScene) new Scene(flags,aflags);
    CATCH_END;
    return NULL;
  }
  
  RTCORE_API void rtcCommit (RTCScene scene) 
  {
    CATCH_BEGIN;
    TRACE(rtcCommit);
    VERIFY_HANDLE(scene);
    ((Scene*)scene)->build();
    CATCH_END;
  }
  
  RTCORE_API void rtcIntersect (RTCScene scene, RTCRay& ray) {
    TRACE(rtcIntersect);
    STAT3(normal.travs,1,1,1);
    ((Scene*)scene)->intersect(ray);
  }
  
  RTCORE_API void rtcIntersect4 (const void* valid, RTCScene scene, RTCRay4& ray) {
    TRACE(rtcIntersect4);
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,4);
    ((Scene*)scene)->intersect4(valid,ray);
  }
  
  RTCORE_API void rtcIntersect8 (const void* valid, RTCScene scene, RTCRay8& ray) {
    TRACE(rtcIntersect8);
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,8);
    ((Scene*)scene)->intersect8(valid,ray);
  }
  
  RTCORE_API void rtcIntersect16 (const void* valid, RTCScene scene, RTCRay16& ray) {
    TRACE(rtcIntersect16);
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,16);
    ((Scene*)scene)->intersect16(valid,ray);
  }
  
  RTCORE_API void rtcOccluded (RTCScene scene, RTCRay& ray) {
    TRACE(rtcOccluded);
    STAT3(shadow.travs,1,1,1);
    ((Scene*)scene)->occluded(ray);
  }
  
  RTCORE_API void rtcOccluded4 (const void* valid, RTCScene scene, RTCRay4& ray) {
    TRACE(rtcOccluded4);
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,4);
    ((Scene*)scene)->occluded4(valid,ray);
  }
  
  RTCORE_API void rtcOccluded8 (const void* valid, RTCScene scene, RTCRay8& ray) {
    TRACE(rtcOccluded8);
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,8);
    ((Scene*)scene)->occluded8(valid,ray);
  }
  
  RTCORE_API void rtcOccluded16 (const void* valid, RTCScene scene, RTCRay16& ray) {
    TRACE(rtcOccluded16);
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,16);
    ((Scene*)scene)->occluded16(valid,ray);
  }
  
  RTCORE_API void rtcDeleteScene (RTCScene scene) {
    CATCH_BEGIN;
    TRACE(rtcDeleteScene);
    VERIFY_HANDLE(scene);
    delete (Scene*) scene;
    CATCH_END;
  }

  RTCORE_API unsigned rtcNewInstance (RTCScene target, RTCScene source) 
  {
    CATCH_BEGIN;
    TRACE(rtcNewInstance);
    VERIFY_HANDLE(target);
    VERIFY_HANDLE(source);
    return ((Scene*) target)->newInstance((Scene*) source);
    CATCH_END;
    return -1;
  }

  RTCORE_API void rtcSetTransform (RTCScene scene, unsigned geomID, RTCMatrixType layout, float* xfm) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetTransform);
    VERIFY_HANDLE(scene);

    AffineSpace3f transform = one;
    switch (layout) 
    {
    case RTC_MATRIX_ROW_MAJOR:
      transform = AffineSpace3f(Vec3fa(xfm[ 0],xfm[ 4],xfm[ 8]),
                                Vec3fa(xfm[ 1],xfm[ 5],xfm[ 9]),
                                Vec3fa(xfm[ 2],xfm[ 6],xfm[10]),
                                Vec3fa(xfm[ 3],xfm[ 7],xfm[11]));
      break;

    case RTC_MATRIX_COLUMN_MAJOR:
      transform = AffineSpace3f(Vec3fa(xfm[ 0],xfm[ 1],xfm[ 2]),
                                Vec3fa(xfm[ 3],xfm[ 4],xfm[ 5]),
                                Vec3fa(xfm[ 6],xfm[ 7],xfm[ 8]),
                                Vec3fa(xfm[ 9],xfm[10],xfm[11]));
      break;

    case RTC_MATRIX_COLUMN_MAJOR_ALIGNED16:
      transform = AffineSpace3f(Vec3fa(xfm[ 0],xfm[ 1],xfm[ 2]),
                                Vec3fa(xfm[ 4],xfm[ 5],xfm[ 6]),
                                Vec3fa(xfm[ 8],xfm[ 9],xfm[10]),
                                Vec3fa(xfm[12],xfm[13],xfm[14]));
      break;

    default: 
      ERROR("Unknown matrix type");
      break;
    }
    ((Scene*) scene)->get_locked(geomID)->setTransform(transform);

    CATCH_END;
  }

  RTCORE_API unsigned rtcNewUserGeometry (RTCScene scene) 
  {
    CATCH_BEGIN;
    TRACE(rtcNewUserGeometry);
    VERIFY_HANDLE(scene);
    return ((Scene*)scene)->newUserGeometry();
    CATCH_END;
    return -1;
  }

  RTCORE_API unsigned rtcNewTriangleMesh (RTCScene scene, RTCFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) 
  {
    CATCH_BEGIN;
    TRACE(rtcNewTriangleMesh);
    VERIFY_HANDLE(scene);
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCFlags(flags |  RTC_INCOHERENT);
    return ((Scene*)scene)->newTriangleMesh(flags,numTriangles,numVertices,numTimeSteps);
    CATCH_END;
    return -1;
  }

  RTCORE_API unsigned rtcNewQuadraticBezierCurves (RTCScene scene, RTCFlags flags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    CATCH_BEGIN;
    TRACE(rtcNewQuadraticBezierCurves);
    VERIFY_HANDLE(scene);
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCFlags(flags | RTC_INCOHERENT);
    return ((Scene*)scene)->newQuadraticBezierCurves(flags,numCurves,numVertices,numTimeSteps);
    CATCH_END;
    return -1;
  }

  RTCORE_API void rtcSetMask (RTCScene scene, unsigned geomID, int mask) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetMask);
    ((Scene*)scene)->get_locked(geomID)->setMask(mask);
    CATCH_END;
  }

  RTCORE_API void* rtcMapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type) 
  {
    CATCH_BEGIN;
    TRACE(rtcMapBuffer);
    return ((Scene*)scene)->get_locked(geomID)->map(type);
    CATCH_END;
    return NULL;
  }

  RTCORE_API void rtcUnmapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type) 
  {
    CATCH_BEGIN;
    TRACE(rtcUnmapBuffer);
    ((Scene*)scene)->get_locked(geomID)->unmap(type);
    CATCH_END;
  }

  RTCORE_API void rtcEnable (RTCScene scene, unsigned geomID) 
  {
    CATCH_BEGIN;
    TRACE(rtcEnable);
    ((Scene*)scene)->get_locked(geomID)->enable();
    CATCH_END;
  }

  RTCORE_API void rtcUpdate (RTCScene scene, unsigned geomID) 
  {
    CATCH_BEGIN;
    TRACE(rtcUpdate);
    ((Scene*)scene)->get_locked(geomID)->update();
    CATCH_END;
  }

  RTCORE_API void rtcDisable (RTCScene scene, unsigned geomID) 
  {
    CATCH_BEGIN;
    TRACE(rtcDisable);
    ((Scene*)scene)->get_locked(geomID)->disable();
    CATCH_END;
  }

  RTCORE_API void rtcDeleteGeometry (RTCScene scene, unsigned geomID) 
  {
    CATCH_BEGIN;
    TRACE(rtcDeleteGeometry);
    ((Scene*)scene)->get_locked(geomID)->erase();
    CATCH_END;
  }

  RTCORE_API void rtcSetBounds (RTCScene scene, unsigned geomID, 
                                float lower_x, float lower_y, float lower_z,
                                float upper_x, float upper_y, float upper_z)
  {
    CATCH_BEGIN;
    TRACE(rtcSetBounds);
    Vec3fa lower = Vec3fa(lower_x,lower_y,lower_z);
    Vec3fa upper = Vec3fa(upper_x,upper_y,upper_z);
    BBox3f box;
    box.lower = lower;
    box.upper = upper;
    ((Scene*)scene)->get_locked(geomID)->setBounds(box);
    CATCH_END;
  }

  RTCORE_API void rtcSetUserData (RTCScene scene, unsigned geomID, void* ptr) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetUserData);
    ((Scene*)scene)->get_locked(geomID)->setUserData(ptr);
    CATCH_END;
  }

  RTCORE_API void rtcSetIntersectFunction (RTCScene scene, unsigned geomID, RTCIntersectFunc intersect) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetIntersectFunction);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction(intersect);
    CATCH_END;
  }

  RTCORE_API void rtcSetIntersectFunction4 (RTCScene scene, unsigned geomID, RTCIntersectFunc4 intersect4) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetIntersectFunction4);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction4(intersect4);
    CATCH_END;
  }

  RTCORE_API void rtcSetIntersectFunction8 (RTCScene scene, unsigned geomID, RTCIntersectFunc8 intersect8) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetIntersectFunction8);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction8(intersect8);
    CATCH_END;
  }
  
  RTCORE_API void rtcSetIntersectFunction16 (RTCScene scene, unsigned geomID, RTCIntersectFunc16 intersect16) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetIntersectFunction16);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction16(intersect16);
    CATCH_END;
  }

  RTCORE_API void rtcSetOccludedFunction (RTCScene scene, unsigned geomID, RTCOccludedFunc occluded) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetOccludedFunction);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction(occluded);
    CATCH_END;
  }

  RTCORE_API void rtcSetOccludedFunction4 (RTCScene scene, unsigned geomID, RTCOccludedFunc4 occluded4) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetOccludedFunction4);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction4(occluded4);
    CATCH_END;
  }

  RTCORE_API void rtcSetOccludedFunction8 (RTCScene scene, unsigned geomID, RTCOccludedFunc8 occluded8) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetOccludedFunction8);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction8(occluded8);
    CATCH_END;
  }

  RTCORE_API void rtcSetOccludedFunction16 (RTCScene scene, unsigned geomID, RTCOccludedFunc16 occluded16) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetOccludedFunction16);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction16(occluded16);
    CATCH_END;
  }
}
