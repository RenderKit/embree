// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "device.h"
#include "version.h"
#include "scene_triangle_mesh.h"
#include "scene_user_geometry.h"
#include "scene_instance.h"
#include "scene_bezier_curves.h"
#include "scene_subdiv_mesh.h"

#include "subdiv/tessellation_cache.h"

#include "acceln.h"
#include "geometry.h"

#if !defined(__MIC__)
#include "../xeon/bvh/bvh4_factory.h"
#include "../xeon/bvh/bvh8_factory.h"
#endif

#if defined(TASKING_LOCKSTEP)
#  include "../../common/tasking/taskscheduler_mic.h"
#elif defined(TASKING_TBB_INTERNAL)
#  include "../../common/tasking/taskscheduler_tbb.h"
#endif

namespace embree
{
  /* functions to initialize global constants */
  void init_globals();

#if defined(__MIC__)
  void BVH4iRegister();
  void BVH4MBRegister();
  void BVH4HairRegister();
#endif

#if defined(TASKING_TBB)

  bool g_tbb_threads_initialized = false;
  tbb::task_scheduler_init g_tbb_threads(tbb::task_scheduler_init::deferred);

  class TBBAffinity: public tbb::task_scheduler_observer
  {
    tbb::atomic<int> threadCount;

    void on_scheduler_entry( bool ) {
      ++threadCount;
      setAffinity(TaskSchedulerTBB::threadIndex()); // FIXME: use threadCount?
    }

    void on_scheduler_exit( bool ) { 
      --threadCount; 
    }
  public:
    
    TBBAffinity() { threadCount = 0; }

    int  get_concurrency()      { return threadCount; }
    void set_concurrency(int i) { threadCount = i; }

  } tbb_affinity;
#endif

  static MutexSys g_mutex;
  static std::map<Device*,size_t> g_cache_size_map;
  static std::map<Device*,size_t> g_num_threads_map;

  Device::Device (const char* cfg, bool singledevice)
    : State(singledevice)
  {
    /* initialize global state */
    init_globals();
    State::parseString(cfg);
    if (FileName::executableFolder() != FileName(""))
      State::parseFile(FileName::executableFolder()+FileName(".embree" TOSTRING(__EMBREE_VERSION_MAJOR__)));
    if (FileName::homeFolder() != FileName("")) // home folder is not available on KNC
      State::parseFile(FileName::homeFolder()+FileName(".embree" TOSTRING(__EMBREE_VERSION_MAJOR__)));
    State::verify();
    
    /*! set tessellation cache size */
    setCacheSize( State::tessellation_cache_size );

    /*! enable some floating point exceptions to catch bugs */
    if (State::float_exceptions)
    {
      int exceptions = _MM_MASK_MASK;
      //exceptions &= ~_MM_MASK_INVALID;
      exceptions &= ~_MM_MASK_DENORM;
      exceptions &= ~_MM_MASK_DIV_ZERO;
      //exceptions &= ~_MM_MASK_OVERFLOW;
      //exceptions &= ~_MM_MASK_UNDERFLOW;
      //exceptions &= ~_MM_MASK_INEXACT;
      _MM_SET_EXCEPTION_MASK(exceptions);
    }

    /* print info header */
    if (State::verbosity(1))
      print();
    if (State::verbosity(2)) 
      State::print();

    /* register all algorithms */
    instance_factory = new InstanceFactory(enabled_cpu_features);

#if !defined(__MIC__)
    bvh4_factory = new BVH4Factory(enabled_cpu_features);
#endif

#if defined(__TARGET_AVX__)
    bvh8_factory = new BVH8Factory(enabled_cpu_features);
#endif

#if defined(__MIC__)
    BVH4iRegister();
    BVH4MBRegister();
    BVH4HairRegister();
#endif 

    /* setup tasking system */
    initTaskingSystem(numThreads);

    /* execute regression tests */
#if !defined(__MIC__)
    if (State::regression_testing) 
      runRegressionTests();
#endif
  }

  Device::~Device ()
  {
    delete instance_factory;
#if !defined(__MIC__)
    delete bvh4_factory;
#if defined(__TARGET_AVX__)
    delete bvh8_factory;
#endif
#endif
    setCacheSize(0);
    exitTaskingSystem();
  }

  std::string getEnabledTargets()
  {
    std::string v = std::string(ISA_STR) + " ";
#if defined(__TARGET_SSE41__)
    v += "SSE4.1 ";
#endif
#if defined(__TARGET_SSE42__)
    v += "SSE4.2 ";
#endif
#if defined(__TARGET_AVX__)
    v += "AVX ";
#endif
#if defined(__TARGET_AVX2__)
    v += "AVX2 ";
#endif
#if defined(__TARGET_AVX512KNL__)
    v += "AVX512KNL ";
#endif
    return v;
  }

  std::string getEmbreeFeatures()
  {
    std::string v;
#if defined(RTCORE_RAY_MASK)
    v += "raymasks ";
#endif
#if defined (RTCORE_BACKFACE_CULLING)
    v += "backfaceculling ";
#endif
#if defined(RTCORE_INTERSECTION_FILTER)
    v += "intersection_filter ";
#endif
#if defined(RTCORE_BUFFER_STRIDE)
    v += "bufferstride ";
#endif
    return v;
  }

  void Device::print()
  {
    const int cpu_features = getCPUFeatures();
    std::cout << "Embree Ray Tracing Kernels " << __EMBREE_VERSION__ << " (" << __DATE__ << ")" << std::endl;
    std::cout << "  Compiler  : " << getCompilerName() << std::endl;
    std::cout << "  Build     : ";
#if defined(DEBUG)
    std::cout << "Debug " << std::endl;
#else
    std::cout << "Release " << std::endl;
#endif
    std::cout << "  Platform  : " << getPlatformName() << std::endl;
    std::cout << "  CPU       : " << stringOfCPUModel(getCPUModel()) << " (" << getCPUVendor() << ")" << std::endl;
    std::cout << "   Threads  : " << getNumberOfLogicalThreads() << std::endl;
    std::cout << "   ISA      : " << stringOfCPUFeatures(cpu_features) << std::endl;
    std::cout << "   Targets  : " << supportedTargetList(cpu_features) << std::endl;
#if !defined(__MIC__)
    const bool hasFTZ = _mm_getcsr() & _MM_FLUSH_ZERO_ON;
    const bool hasDAZ = _mm_getcsr() & _MM_DENORMALS_ZERO_ON;
    std::cout << "   MXCSR    : " << "FTZ=" << hasFTZ << ", DAZ=" << hasDAZ << std::endl;
#endif
    std::cout << "  Config" << std::endl;
    std::cout << "    Threads : " << (numThreads ? toString(numThreads) : std::string("default")) << std::endl;
    std::cout << "    ISA     : " << stringOfCPUFeatures(enabled_cpu_features) << std::endl;
    std::cout << "    Targets : " << supportedTargetList(enabled_cpu_features) << " (supported)" << std::endl;
    std::cout << "              " << getEnabledTargets() << " (compile time enabled)" << std::endl;
    std::cout << "    Features: " << getEmbreeFeatures() << std::endl;
    std::cout << "    Tasking : ";
#if defined(TASKING_TBB)
    std::cout << "TBB" << TBB_VERSION_MAJOR << "." << TBB_VERSION_MINOR << " ";
    std::cout << "TBB_header_interface_" << TBB_INTERFACE_VERSION << " TBB_lib_interface_" << tbb::TBB_runtime_interface_version() << " ";
#endif
#if defined(TASKING_TBB_INTERNAL)
      std::cout << "internal_tasking_system ";
#endif
#if defined(TASKING_LOCKSTEP)
    std::cout << "internal_tasking_system ";
#endif
    std::cout << std::endl;

    /* check of FTZ and DAZ flags are set in CSR */
#if !defined(__MIC__)
    if (!hasFTZ || !hasDAZ) {
#if !defined(_DEBUG)
      if (State::verbosity(1)) 
#endif
      {
        std::cout << std::endl;
        std::cout << "================================================================================" << std::endl;
        std::cout << "  WARNING: \"Flush to Zero\" or \"Denormals are Zero\" mode not enabled "         << std::endl 
                  << "           in the MXCSR control and status register. This can have a severe "     << std::endl
                  << "           performance impact. Please enable these modes for each application "   << std::endl
                  << "           thread the following way:" << std::endl
                  << std::endl 
                  << "           #include \"xmmintrin.h\"" << std::endl 
                  << "           #include \"pmmintrin.h\"" << std::endl 
                  << std::endl 
                  << "           _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);" << std::endl 
                  << "           _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);" << std::endl;
        std::cout << "================================================================================" << std::endl;
        std::cout << std::endl;
      }
    }
#endif

#if defined (__MIC__) && defined(RTCORE_BUFFER_STRIDE)
    if (State::verbosity(1))
      std::cout << "  WARNING: enabled 'bufferstride' support will lower BVH build performance" << std::endl;
#endif

    std::cout << std::endl;
  }

  RTCError* Device::getError()
  {
    if (this) return errorHandler.error();
    else      return g_errorHandler.error();
  }

  void Device::setErrorCode(RTCError error)
  {
    RTCError* stored_error = getError();
    if (*stored_error == RTC_NO_ERROR)
      *stored_error = error;
  }

  RTCError Device::getErrorCode()
  {
    RTCError* stored_error = getError();
    RTCError error = *stored_error;
    *stored_error = RTC_NO_ERROR;
    return error;
  }

  void Device::process_error(RTCError error, const char* str)
  { 
    /* store global error code when device construction failed */
    if (this == nullptr)
      return setErrorCode(error);

    /* print error when in verbose mode */
    if (State::verbosity(1)) 
    {
      switch (error) {
      case RTC_NO_ERROR         : std::cerr << "Embree: No error"; break;
      case RTC_UNKNOWN_ERROR    : std::cerr << "Embree: Unknown error"; break;
      case RTC_INVALID_ARGUMENT : std::cerr << "Embree: Invalid argument"; break;
      case RTC_INVALID_OPERATION: std::cerr << "Embree: Invalid operation"; break;
      case RTC_OUT_OF_MEMORY    : std::cerr << "Embree: Out of memory"; break;
      case RTC_UNSUPPORTED_CPU  : std::cerr << "Embree: Unsupported CPU"; break;
      default                   : std::cerr << "Embree: Invalid error code"; break;                   
      };
      if (str) std::cerr << ", (" << str << ")";
      std::cerr << std::endl;
    }

    /* call user specified error callback */
    if (State::error_function) 
      State::error_function(error,str); 

    /* record error code */
    setErrorCode(error);
  }

  void Device::memoryMonitor(ssize_t bytes, bool post)
  {
    if (State::memory_monitor_function && bytes != 0) {
      if (!State::memory_monitor_function(bytes,post)) {
#if !defined(TASKING_LOCKSTEP)
        if (bytes > 0) { // only throw exception when we allocate memory to never throw inside a destructor
          throw_RTCError(RTC_OUT_OF_MEMORY,"memory monitor forced termination");
        }
#endif
      }
    }
  }
 
  void Device::setCacheSize(size_t bytes) 
  {
    Lock<MutexSys> lock(g_mutex);
    if (bytes == 0) g_cache_size_map.erase(this);
    else            g_cache_size_map[this] = bytes;
    
    size_t maxCacheSize = 0;
    for (std::map<Device*,size_t>::iterator i=g_cache_size_map.begin(); i!= g_cache_size_map.end(); i++)
      maxCacheSize = max(maxCacheSize, (*i).second);
    
    resizeTessellationCache(max(size_t(1024*1024),maxCacheSize));
  }

  void Device::initTaskingSystem(size_t numThreads) 
  {
    Lock<MutexSys> lock(g_mutex);
    if (numThreads == 0) g_num_threads_map[this] = -1;
    else                 g_num_threads_map[this] = numThreads;
    configureTaskingSystem();
  }

  void Device::configureTaskingSystem() 
  {
    /* terminate tasking system */
    if (g_num_threads_map.size() == 0)
    {
#if defined(TASKING_LOCKSTEP)
      TaskScheduler::destroy();
#endif
      
#if defined(TASKING_TBB_INTERNAL)
      TaskSchedulerTBB::destroy();
#endif
      
#if defined(TASKING_TBB)
      if (g_tbb_threads_initialized) {
        g_tbb_threads.terminate();
        g_tbb_threads_initialized = false;
      }
#endif
      return;
    }

    /*! get maximal configured number of threads */
    size_t maxNumThreads = 0;
    for (std::map<Device*,size_t>::iterator i=g_num_threads_map.begin(); i != g_num_threads_map.end(); i++)
      maxNumThreads = max(maxNumThreads, (*i).second);
    if (maxNumThreads == -1) 
      maxNumThreads = 0;

#if defined(TASKING_LOCKSTEP)
    TaskScheduler::create(maxNumThreads,State::set_affinity);
#endif

#if defined(TASKING_TBB_INTERNAL)
    TaskSchedulerTBB::create(maxNumThreads,State::set_affinity);
#endif

#if defined(TASKING_TBB)

    /* first terminate threads in case we configured them */
    if (g_tbb_threads_initialized) {
      g_tbb_threads.terminate();
      g_tbb_threads_initialized = false;
    }

    /* only set affinity if requested by the user */
    if (State::set_affinity) {
      tbb_affinity.set_concurrency(0);
      tbb_affinity.observe(true); 
    }

    /* now either keep default settings are configure number of threads */
    if (maxNumThreads == 0) 
    {
      g_tbb_threads_initialized = false;
      TaskSchedulerTBB::g_numThreads = tbb::task_scheduler_init::default_num_threads();
    } else {
      g_tbb_threads_initialized = true;
      g_tbb_threads.initialize(maxNumThreads);
      TaskSchedulerTBB::g_numThreads = maxNumThreads;
    }
#if USE_TASK_ARENA
    arena = new tbb::task_arena(maxNumThreads);
#endif
#endif
  }

  void Device::exitTaskingSystem() 
  {
    Lock<MutexSys> lock(g_mutex);
    g_num_threads_map.erase(this);
    configureTaskingSystem();
#if USE_TASK_ARENA
    delete arena; arena = nullptr;
#endif
  }

  void Device::setParameter1i(const RTCParameter parm, ssize_t val)
  {
    switch (parm) {
    case RTC_SOFTWARE_CACHE_SIZE: setCacheSize(val); break;
    default: throw_RTCError(RTC_INVALID_ARGUMENT, "unknown parameter"); break;
    };
  }
}
