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

#include "../include/embree2/rtcore.h"
#include "../include/embree2/rtcore_ray.h"
#include "../kernels/common/default.h"
#include "../kernels/algorithms/parallel_for.h"
#include <vector>

#if defined(RTCORE_RAY_PACKETS) && !defined(__MIC__)
#  define HAS_INTERSECT4 1
#else
#  define HAS_INTERSECT4 0
#endif

#if defined(RTCORE_RAY_PACKETS) && (defined(__TARGET_AVX__) || defined(__TARGET_AVX2__))
#  define HAS_INTERSECT8 1
#else
#  define HAS_INTERSECT8 0
#endif

#if defined(RTCORE_RAY_PACKETS) && (defined(__MIC__) || defined(__TARGET_AVX512KNL__))
#  define HAS_INTERSECT16 1
#else
#  define HAS_INTERSECT16 0
#endif

namespace embree
{
  bool hasISA(const int isa) 
  {
    int cpu_features = getCPUFeatures();
    return (cpu_features & isa) == isa;
  }

  /* error reporting function */
  void error_handler(const RTCError code, const char* str = nullptr)
  {
    if (code == RTC_NO_ERROR) 
      return;

    printf("Embree: ");
    switch (code) {
    case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
    case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
    case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
    case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
    case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
    case RTC_CANCELLED        : printf("RTC_CANCELLED"); break;
    default                   : printf("invalid error code"); break;
    }
    if (str) { 
      printf(" ("); 
      while (*str) putchar(*str++); 
      printf(")\n"); 
    }
    exit(1);
  }

  RTCAlgorithmFlags aflags = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8 | RTC_INTERSECT16);

  /* configuration */
  static std::string g_rtcore = "";

  static size_t g_plot_min = 0;
  static size_t g_plot_max = 0;
  static size_t g_plot_step= 0;
  static std::string g_plot_test = "";
  static bool executed_benchmarks = false;

  /* vertex and triangle layout */
  struct Vertex   { float x,y,z,a; };
  struct Triangle { int v0, v1, v2; };

#define AssertNoError() \
  if (rtcGetError() != RTC_NO_ERROR) return false;
#define AssertAnyError() \
  if (rtcGetError() == RTC_NO_ERROR) return false;
#define AssertError(code) \
  if (rtcGetError() != code) return false;

  std::vector<thread_t> g_threads;

  MutexSys g_mutex;
  BarrierSys g_barrier;
  LinearBarrierActive g_barrier_active;
  size_t g_num_mutex_locks = 100000;
  size_t g_num_threads = 0;
  ssize_t g_num_threads_init = -1;
  atomic_t g_atomic_cntr = 0;

  class Benchmark
  {
  public:
    const std::string name;
    const std::string unit;
    Benchmark (const std::string& name, const std::string& unit)
      : name(name), unit(unit) {}

    virtual double run(size_t numThreads) = 0;

    void print(size_t numThreads, size_t N) 
    {
      double pmin = inf, pmax = -float(inf), pavg = 0.0f;
      for (size_t j=0; j<N; j++) {
	double p = run(numThreads);
	pmin = min(pmin,p);
	pmax = max(pmax,p);
	pavg = pavg + p/double(N);
      }

      printf("%40s ... [%f / %f / %f] %s\n",name.c_str(),pmin,pavg,pmax,unit.c_str());
      fflush(stdout);
    }
  };

  class benchmark_mutex_sys : public Benchmark
  {
  public:
    benchmark_mutex_sys () 
     : Benchmark("mutex_sys","ms") {}

    static void benchmark_mutex_sys_thread(void* ptr) 
    {
      g_barrier.wait();      
      while (true)
	{
	  if (atomic_add(&g_atomic_cntr,-1) < 0) break;
	  g_mutex.lock();
	  g_mutex.unlock();
	}
    }
    
    double run (size_t numThreads)
    {
      g_barrier.init(numThreads);
      g_atomic_cntr = g_num_mutex_locks;
      for (size_t i=1; i<numThreads; i++)
	g_threads.push_back(createThread(benchmark_mutex_sys_thread,nullptr,1000000,i));
      //setAffinity(0);

      double t0 = getSeconds();
      benchmark_mutex_sys_thread(nullptr);
      double t1 = getSeconds();
      
      for (size_t i=0; i<g_threads.size(); i++)	join(g_threads[i]);
      g_threads.clear();
      
      //printf("%40s ... %f ms (%f k/s)\n","mutex_sys",1000.0f*(t1-t0)/double(g_num_mutex_locks),1E-3*g_num_mutex_locks/(t1-t0));
      //fflush(stdout);
      return 1000.0f*(t1-t0)/double(g_num_mutex_locks);
    }
  };

  class benchmark_barrier_sys : public Benchmark
  {
  public:
    enum { N = 100 };

    benchmark_barrier_sys () 
     : Benchmark("barrier_sys","ms") {}

    static void benchmark_barrier_sys_thread(void* ptr) 
    {
      g_barrier.wait();
      for (size_t i=0; i<N; i++) 
	g_barrier.wait();
    }
    
    double run (size_t numThreads)
    {
      g_barrier.init(numThreads);
      for (size_t i=1; i<numThreads; i++)
	g_threads.push_back(createThread(benchmark_barrier_sys_thread,(void*)i,1000000,i));
      //setAffinity(0);
      
      g_barrier.wait();
      double t0 = getSeconds();
      for (size_t i=0; i<N; i++) g_barrier.wait();
      double t1 = getSeconds();
      
      for (size_t i=0; i<g_threads.size(); i++)	join(g_threads[i]);
      g_threads.clear();
    
      //printf("%40s ... %f ms (%f k/s)\n","barrier_sys",1000.0f*(t1-t0)/double(N),1E-3*N/(t1-t0));
      //fflush(stdout);
      return 1000.0f*(t1-t0)/double(N);
    }
  };

  class benchmark_barrier_active : public Benchmark
  {
    enum { N = 1000 };

  public:
    benchmark_barrier_active () 
      : Benchmark("barrier_active","ns") {}

    static void benchmark_barrier_active_thread(void* ptr) 
    {
      size_t threadIndex = (size_t) ptr;
      size_t threadCount = g_num_threads;
      g_barrier_active.wait(threadIndex);
      for (size_t i=0; i<N; i++) 
	g_barrier_active.wait(threadIndex);
    }
  
    double run (size_t numThreads)
    {
      g_num_threads = numThreads;
      g_barrier_active.init(numThreads);
      for (size_t i=1; i<numThreads; i++)
	g_threads.push_back(createThread(benchmark_barrier_active_thread,(void*)i,1000000,i));
      setAffinity(0);
      
      g_barrier_active.wait(0);
      double t0 = getSeconds();
      for (size_t i=0; i<N; i++) 
	g_barrier_active.wait(0);
      double t1 = getSeconds();
      
      for (size_t i=0; i<g_threads.size(); i++)
	join(g_threads[i]);
      
      g_threads.clear();
      
      //printf("%40s ... %f ms (%f k/s)\n","barrier_active",1000.0f*(t1-t0)/double(N),1E-3*N/(t1-t0));
      //fflush(stdout);
      return 1E9*(t1-t0)/double(N);
    }
  };

  class benchmark_atomic_inc : public Benchmark
  {
  public:
    enum { N = 10000000 };

    benchmark_atomic_inc () 
     : Benchmark("atomic_inc","ns") {}

    static void benchmark_atomic_inc_thread(void* arg) 
    {
      size_t threadIndex = (size_t) arg;
      size_t threadCount = g_num_threads;
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
      __memory_barrier();
      while (atomic_add(&g_atomic_cntr,-1) > 0);
      __memory_barrier();
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
      
    }
    
    double run (size_t numThreads)
    {
      g_atomic_cntr = N;

      g_num_threads = numThreads;
      g_barrier_active.init(numThreads);
      for (size_t i=1; i<numThreads; i++)
	g_threads.push_back(createThread(benchmark_atomic_inc_thread,(void*)i,1000000,i));
      setAffinity(0);
      
      g_barrier_active.wait(0);
      double t0 = getSeconds();
      //size_t c0 = rdtsc();
      benchmark_atomic_inc_thread(nullptr);
      //size_t c1 = rdtsc();
      double t1 = getSeconds();
      g_barrier_active.wait(0);
      
      for (size_t i=0; i<g_threads.size(); i++)	join(g_threads[i]);
      g_threads.clear();
      //PRINT(double(c1-c0)/N);

      //printf("%40s ... %f ms (%f k/s)\n","mutex_sys",1000.0f*(t1-t0)/double(g_num_mutex_locks),1E-3*g_num_mutex_locks/(t1-t0));
      //fflush(stdout);
      return 1E9*(t1-t0)/double(N);
    }
  };

  class benchmark_pagefaults : public Benchmark
  {
  public:
    enum { N = 1024*1024*1024 };

    static char* ptr;

    benchmark_pagefaults () 
     : Benchmark("pagefaults","GB/s") {}

    static void benchmark_pagefaults_thread(void* arg) 
    {
      size_t threadIndex = (size_t) arg;
      size_t threadCount = g_num_threads;
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
      size_t start = (threadIndex+0)*N/threadCount;
      size_t end   = (threadIndex+1)*N/threadCount;
      for (size_t i=start; i<end; i+=64)
	ptr[i] = 0;
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
    }
    
    double run (size_t numThreads)
    {
#if !defined(__WIN32__)
      ptr = (char*) os_reserve(N);
#else
      ptr = (char*)os_malloc(N);
#endif
      g_num_threads = numThreads;
      g_barrier_active.init(numThreads);
      for (size_t i=1; i<numThreads; i++)
	g_threads.push_back(createThread(benchmark_pagefaults_thread,(void*)i,1000000,i));
      //setAffinity(0);
      
      g_barrier_active.wait(0);
      double t0 = getSeconds();
      benchmark_pagefaults_thread(0);
      double t1 = getSeconds();
      g_barrier_active.wait(0);
      
      for (size_t i=0; i<g_threads.size(); i++)	join(g_threads[i]);
      g_threads.clear();
      os_free(ptr,N);
      
      return 1E-9*double(N)/(t1-t0);
    }
  };

  char* benchmark_pagefaults::ptr = nullptr;


#if defined(__UNIX__)

#include <sys/mman.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#endif

  class benchmark_osmalloc_with_page_commit : public Benchmark
  {
  public:
    enum { N = 1024*1024*1024 };

    static char* ptr;

    benchmark_osmalloc_with_page_commit () 
     : Benchmark("osmalloc_with_page_commit","GB/s") {}

    static void benchmark_osmalloc_with_page_commit_thread(void* arg) 
    {
      size_t threadIndex = (size_t) arg;
      size_t threadCount = g_num_threads;
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
      size_t start = (threadIndex+0)*N/threadCount;
      size_t end   = (threadIndex+1)*N/threadCount;
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
    }
    
    double run (size_t numThreads)
    {
      int flags = 0;
#if defined(__UNIX__) && !defined(__APPLE__)
      flags |= MAP_POPULATE;
#endif
      size_t startN = 1024*1024*16;

      double t = 0.0f;
      size_t iterations = 0;
      for (size_t i=startN;i<=N;i*=2,iterations++)
      {
        double t0 = getSeconds();
        char *ptr = (char*) os_malloc(i,flags);
        double t1 = getSeconds();
        os_free(ptr,i);
        t += 1E-9*double(i)/(t1-t0);        
      }      
      
      return t / (double)iterations;
    }
  };


  // -------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------





  class benchmark_bandwidth : public Benchmark
  {
  public:
    enum { N = 300000000 };

    static char* ptr;

    benchmark_bandwidth () 
      : Benchmark("bandwidth","GB/s") {}

    static void benchmark_bandwidth_thread(void* arg) 
    {
      size_t threadIndex = (size_t) arg;
      size_t threadCount = g_num_threads;
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
      size_t start = (threadIndex+0)*N/threadCount;
      size_t end   = (threadIndex+1)*N/threadCount;
      char p = 0;
      for (size_t i=start; i<end; i+=64)
	p += ptr[i];
      volatile char out = p;
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
    }
    
    double run (size_t numThreads)
    {
      ptr = (char*) os_malloc(N);
      for (size_t i=0; i<N; i+=4096) ptr[i] = 0;

      g_num_threads = numThreads;
      g_barrier_active.init(numThreads);
      for (size_t i=1; i<numThreads; i++)
	g_threads.push_back(createThread(benchmark_bandwidth_thread,(void*)i,1000000,i));
      //setAffinity(0);
      
      g_barrier_active.wait(0);
      double t0 = getSeconds();
      benchmark_bandwidth_thread(0);
      double t1 = getSeconds();
      g_barrier_active.wait(0);

      for (size_t i=0; i<g_threads.size(); i++)	join(g_threads[i]);
      g_threads.clear();
      os_free(ptr,N);
      
      return 1E-9*double(N)/(t1-t0);
    }
  };

  char* benchmark_bandwidth::ptr = nullptr;

  RTCRay makeRay(Vec3f org, Vec3f dir) 
  {
    RTCRay ray;
    ray.org[0] = org.x; ray.org[1] = org.y; ray.org[2] = org.z;
    ray.dir[0] = dir.x; ray.dir[1] = dir.y; ray.dir[2] = dir.z;
    ray.tnear = 0.0f; ray.tfar = inf;
    ray.time = 0; ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
    return ray;
  }

  RTCRay makeRay(Vec3f org, Vec3f dir, float tnear, float tfar) 
  {
    RTCRay ray;
    ray.org[0] = org.x; ray.org[1] = org.y; ray.org[2] = org.z;
    ray.dir[0] = dir.x; ray.dir[1] = dir.y; ray.dir[2] = dir.z;
    ray.tnear = tnear; ray.tfar = tfar;
    ray.time = 0; ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
    return ray;
  }
  
  void setRay(RTCRay4& ray_o, int i, const RTCRay& ray_i)
  {
    ray_o.orgx[i] = ray_i.org[0];
    ray_o.orgy[i] = ray_i.org[1];
    ray_o.orgz[i] = ray_i.org[2];
    ray_o.dirx[i] = ray_i.dir[0];
    ray_o.diry[i] = ray_i.dir[1];
    ray_o.dirz[i] = ray_i.dir[2];
    ray_o.tnear[i] = ray_i.tnear;
    ray_o.tfar[i] = ray_i.tfar;
    ray_o.time[i] = ray_i.time;
    ray_o.mask[i] = ray_i.mask;
    ray_o.geomID[i] = ray_i.geomID;
    ray_o.primID[i] = ray_i.primID;
    ray_o.instID[i] = ray_i.instID;
  }

  void setRay(RTCRay8& ray_o, int i, const RTCRay& ray_i)
  {
    ray_o.orgx[i] = ray_i.org[0];
    ray_o.orgy[i] = ray_i.org[1];
    ray_o.orgz[i] = ray_i.org[2];
    ray_o.dirx[i] = ray_i.dir[0];
    ray_o.diry[i] = ray_i.dir[1];
    ray_o.dirz[i] = ray_i.dir[2];
    ray_o.tnear[i] = ray_i.tnear;
    ray_o.tfar[i] = ray_i.tfar;
    ray_o.time[i] = ray_i.time;
    ray_o.mask[i] = ray_i.mask;
    ray_o.geomID[i] = ray_i.geomID;
    ray_o.primID[i] = ray_i.primID;
    ray_o.instID[i] = ray_i.instID;
  }

  void setRay(RTCRay16& ray_o, int i, const RTCRay& ray_i)
  {
    ray_o.orgx[i] = ray_i.org[0];
    ray_o.orgy[i] = ray_i.org[1];
    ray_o.orgz[i] = ray_i.org[2];
    ray_o.dirx[i] = ray_i.dir[0];
    ray_o.diry[i] = ray_i.dir[1];
    ray_o.dirz[i] = ray_i.dir[2];
    ray_o.tnear[i] = ray_i.tnear;
    ray_o.tfar[i] = ray_i.tfar;
    ray_o.time[i] = ray_i.time;
    ray_o.mask[i] = ray_i.mask;
    ray_o.geomID[i] = ray_i.geomID;
    ray_o.primID[i] = ray_i.primID;
    ray_o.instID[i] = ray_i.instID;
  }

  struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
  };

  void createSphereMesh (const Vec3f pos, const float r, size_t numPhi, Mesh& mesh_o)
  {
    /* create a triangulated sphere */
    size_t numTheta = 2*numPhi;
    mesh_o.vertices.resize(numTheta*(numPhi+1));
    mesh_o.triangles.resize(2*numTheta*(numPhi-1));
    
    /* map triangle and vertex buffer */
    Vertex*   vertices  = (Vertex*  ) &mesh_o.vertices[0];
    Triangle* triangles = (Triangle*) &mesh_o.triangles[0];
    
    /* create sphere geometry */
    int tri = 0;
    const float rcpNumTheta = 1.0f/float(numTheta);
    const float rcpNumPhi   = 1.0f/float(numPhi);
    for (size_t phi=0; phi<=numPhi; phi++)
    {
      for (size_t theta=0; theta<numTheta; theta++)
      {
        const float phif   = phi*float(pi)*rcpNumPhi;
        const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
        Vertex& v = vertices[phi*numTheta+theta];
        v.x = pos.x + r*sin(phif)*sin(thetaf);
        v.y = pos.y + r*cos(phif);
        v.z = pos.z + r*sin(phif)*cos(thetaf);
      }
      if (phi == 0) continue;
      
      for (size_t theta=1; theta<=numTheta; theta++) 
      {
        int p00 = (phi-1)*numTheta+theta-1;
        int p01 = (phi-1)*numTheta+theta%numTheta;
        int p10 = phi*numTheta+theta-1;
        int p11 = phi*numTheta+theta%numTheta;
        
        if (phi > 1) {
          triangles[tri].v0 = p10; 
          triangles[tri].v1 = p00; 
          triangles[tri].v2 = p01; 
          tri++;
        }
        
        if (phi < numPhi) {
          triangles[tri].v0 = p11; 
          triangles[tri].v1 = p10;
          triangles[tri].v2 = p01; 
        tri++;
        }
      }
    }
  }

  unsigned addSphere (RTCScene scene, RTCGeometryFlags flag, const Vec3f pos, const float r, size_t numPhi)
  {
    Mesh mesh; createSphereMesh (pos, r, numPhi, mesh);
    unsigned geom = rtcNewTriangleMesh (scene, flag, mesh.triangles.size(), mesh.vertices.size());
    memcpy(rtcMapBuffer(scene,geom,RTC_VERTEX_BUFFER), &mesh.vertices[0], mesh.vertices.size()*sizeof(Vertex));
    memcpy(rtcMapBuffer(scene,geom,RTC_INDEX_BUFFER ), &mesh.triangles[0], mesh.triangles.size()*sizeof(Triangle));
    rtcUnmapBuffer(scene,geom,RTC_VERTEX_BUFFER);
    rtcUnmapBuffer(scene,geom,RTC_INDEX_BUFFER);
    return geom;
  }

  struct LineSegments {
    avector<Vec3fa> vertices;
    avector<int> lines;
  };

  Vec3fa uniformSampleSphere(const float& u, const float& v)
  {
    const float phi = float(two_pi) * u;
    const float cosTheta = 1.0f - 2.0f * v, sinTheta = 2.0f * sqrt(v * (1.0f - v));
    return Vec3fa(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
  }

  inline unsigned int rngInt(unsigned int& state)
  {
    const unsigned int m = 1664525;
    const unsigned int n = 1013904223;

    state = state * m + n;
    return state;
  }

  inline float rngFloat(unsigned int& state)
  {
    rngInt(state);
    return (float)(int)(state >> 1) * 4.656612873077392578125e-10f;
  }

  void createHairball(const Vec3fa& pos, const float r, size_t numLines, LineSegments& hairset_o)
  {
    const float thickness = 0.001f*r;

    unsigned int state = 1;

    for (size_t t=0; t<numLines/3; t++)
    {
      Vec3fa dp = uniformSampleSphere(rngFloat(state), rngFloat(state));

      Vec3fa l0 = pos + r*(dp + 0.00f*dp); l0.w = thickness;
      Vec3fa l1 = pos + r*(dp + 0.25f*dp); l1.w = thickness;
      Vec3fa l2 = pos + r*(dp + 0.50f*dp); l2.w = thickness;
      Vec3fa l3 = pos + r*(dp + 0.75f*dp); l3.w = thickness;

      const unsigned int v_index = hairset_o.vertices.size();
      hairset_o.vertices.push_back(l0);
      hairset_o.vertices.push_back(l1);
      hairset_o.vertices.push_back(l2);
      hairset_o.vertices.push_back(l3);

      hairset_o.lines.push_back(v_index);
      hairset_o.lines.push_back(v_index+1);
      hairset_o.lines.push_back(v_index+2);
    }
  }

  struct Points {
    avector<Vec3fa> vertices;
  };

  void createPointball(const Vec3fa& pos, const float r, size_t numPoints, Points& pointset_o)
  {
    const float thickness = 0.001f*r;

    unsigned int state = 1;

    for (size_t t=0; t<numPoints; t++)
    {
      Vec3fa dp = uniformSampleSphere(rngFloat(state), rngFloat(state));

      Vec3fa l0 = pos + r*dp; l0.w = thickness;

      pointset_o.vertices.push_back(l0);
    }
  }

  class create_geometry : public Benchmark
  {
  public:
    RTCSceneFlags sflags; RTCGeometryFlags gflags; size_t numPhi; size_t numMeshes;
    create_geometry (const std::string& name, RTCSceneFlags sflags, RTCGeometryFlags gflags, size_t numPhi, size_t numMeshes)
      : Benchmark(name,"Mtris/s"), sflags(sflags), gflags(gflags), numPhi(numPhi), numMeshes(numMeshes) {}

    double run(size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      Mesh mesh; createSphereMesh (Vec3f(0,0,0), 1, numPhi, mesh);

      const size_t sizeVertexData = mesh.vertices.size()*sizeof(Vertex);
      
      Vertex** vertices = new Vertex*[numMeshes];
      for (size_t i=0; i<numMeshes; i++)
      {
        vertices[i] = (Vertex*)os_malloc(sizeVertexData);
        assert(vertices[i]);
        for (size_t j=0;j<mesh.vertices.size();j++)
        {
          Vertex &v = vertices[i][j];
          v.x = mesh.vertices[j].x * (float)i;
          v.y = mesh.vertices[j].y * (float)i;
          v.z = mesh.vertices[j].z * (float)i;
          v.a = 0.0f;
        }
      }

      double t0 = getSeconds();
      RTCScene scene = rtcDeviceNewScene(device,sflags,aflags);

      for (size_t i=0; i<numMeshes; i++)
      {
        unsigned geom = rtcNewTriangleMesh (scene, gflags, mesh.triangles.size(), mesh.vertices.size());
#if 1
        rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,&vertices[i][0] ,0,sizeof(Vertex));
        rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER ,&mesh.triangles[0],0,sizeof(Triangle));
#else
        memcpy(rtcMapBuffer(scene,geom,RTC_VERTEX_BUFFER), &mesh.vertices[0], mesh.vertices.size()*sizeof(Vertex));
        memcpy(rtcMapBuffer(scene,geom,RTC_INDEX_BUFFER ), &mesh.triangles[0], mesh.triangles.size()*sizeof(Triangle));
        rtcUnmapBuffer(scene,geom,RTC_VERTEX_BUFFER);
        rtcUnmapBuffer(scene,geom,RTC_INDEX_BUFFER);
        for (size_t i=0; i<mesh.vertices.size(); i++) {
          mesh.vertices[i].x += 1.0f;
          mesh.vertices[i].y += 1.0f;
          mesh.vertices[i].z += 1.0f;
        }
#endif
      }
      rtcCommit (scene);
      double t1 = getSeconds();
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);

      for (size_t i=0; i<numMeshes; i++)
        os_free(vertices[i],sizeVertexData);

      delete[] vertices;

      size_t numTriangles = mesh.triangles.size() * numMeshes;
      return 1E-6*double(numTriangles)/(t1-t0);
    }
  };

  class create_geometry_line : public Benchmark
  {
  public:
    RTCSceneFlags sflags; RTCGeometryFlags gflags; size_t numLines; size_t numMeshes;
    create_geometry_line (const std::string& name, RTCSceneFlags sflags, RTCGeometryFlags gflags, size_t numLines, size_t numMeshes)
      : Benchmark(name,"Mlines/s"), sflags(sflags), gflags(gflags), numLines(numLines), numMeshes(numMeshes) {}

    double run(size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      LineSegments hairset; createHairball (Vec3f(0,0,0), 1, numLines, hairset);

      const size_t sizeVertexData = hairset.vertices.size()*sizeof(Vec3fa);

      Vec3fa** vertices = new Vec3fa*[numMeshes];
      for (size_t i=0; i<numMeshes; i++)
      {
        vertices[i] = (Vec3fa*)os_malloc(sizeVertexData);
        assert(vertices[i]);
        for (size_t j=0;j<hairset.vertices.size();j++)
        {
          Vec3fa &v = vertices[i][j];
          v.x = hairset.vertices[j].x * (float)i;
          v.y = hairset.vertices[j].y * (float)i;
          v.z = hairset.vertices[j].z * (float)i;
          v.a = hairset.vertices[j].a;
        }
      }

      double t0 = getSeconds();
      RTCScene scene = rtcDeviceNewScene(device,sflags,aflags);

      for (size_t i=0; i<numMeshes; i++)
      {
        unsigned geom = rtcNewLineSegments (scene, gflags, hairset.lines.size(), hairset.vertices.size());
#if 1
        rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,&vertices[i][0],0,sizeof(Vec3fa));
        rtcSetBuffer(scene,geom,RTC_INDEX_BUFFER ,&hairset.lines[0],0,sizeof(int));
#else
        memcpy(rtcMapBuffer(scene,geom,RTC_VERTEX_BUFFER), &hairset.vertices[0], hairset.vertices.size()*sizeof(Vec3fa));
        memcpy(rtcMapBuffer(scene,geom,RTC_INDEX_BUFFER ), &hairset.lines[0], hairset.lines.size()*sizeof(int));
        rtcUnmapBuffer(scene,geom,RTC_VERTEX_BUFFER);
        rtcUnmapBuffer(scene,geom,RTC_INDEX_BUFFER);
        for (size_t i=0; i<hairset.vertices.size(); i++) {
          mesh.vertices[i].x += 1.0f;
          mesh.vertices[i].y += 1.0f;
          mesh.vertices[i].z += 1.0f;
        }
#endif
      }
      rtcCommit (scene);
      double t1 = getSeconds();
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);

      for (size_t i=0; i<numMeshes; i++)
        os_free(vertices[i],sizeVertexData);

      delete[] vertices;

      size_t numLines = hairset.lines.size() * numMeshes;
      return 1E-6*double(numLines)/(t1-t0);
    }
  };
  

  class create_geometry_point : public Benchmark
  {
  public:
    RTCSceneFlags sflags; RTCGeometryFlags gflags; size_t numPoints; size_t numMeshes;
    create_geometry_point (const std::string& name, RTCSceneFlags sflags, RTCGeometryFlags gflags, size_t numPoints, size_t numMeshes)
      : Benchmark(name,"Mpoints/s"), sflags(sflags), gflags(gflags), numPoints(numPoints), numMeshes(numMeshes) {}

    double run(size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      Points pointset; createPointball (Vec3f(0,0,0), 1, numPoints, pointset);

      const size_t sizeVertexData = pointset.vertices.size()*sizeof(Vec3fa);

      Vec3fa** vertices = new Vec3fa*[numMeshes];
      for (size_t i=0; i<numMeshes; i++)
      {
        vertices[i] = (Vec3fa*)os_malloc(sizeVertexData);
        assert(vertices[i]);
        for (size_t j=0;j<pointset.vertices.size();j++)
        {
          Vec3fa &v = vertices[i][j];
          v.x = pointset.vertices[j].x * (float)i;
          v.y = pointset.vertices[j].y * (float)i;
          v.z = pointset.vertices[j].z * (float)i;
          v.a = pointset.vertices[j].a;
        }
      }

      double t0 = getSeconds();
      RTCScene scene = rtcDeviceNewScene(device,sflags,aflags);

      for (size_t i=0; i<numMeshes; i++)
      {
        unsigned geom = rtcNewPoints (scene, gflags, pointset.vertices.size());
#if 1
        rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,&vertices[i][0],0,sizeof(Vec3fa));
#else
        memcpy(rtcMapBuffer(scene,geom,RTC_VERTEX_BUFFER), &pointset.vertices[0], pointset.vertices.size()*sizeof(Vec3fa));
        rtcUnmapBuffer(scene,geom,RTC_VERTEX_BUFFER);
        for (size_t i=0; i<pointset.vertices.size(); i++) {
          mesh.vertices[i].x += 1.0f;
          mesh.vertices[i].y += 1.0f;
          mesh.vertices[i].z += 1.0f;
        }
#endif
      }
      rtcCommit (scene);
      double t1 = getSeconds();
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);

      for (size_t i=0; i<numMeshes; i++)
        os_free(vertices[i],sizeVertexData);

      delete[] vertices;

      size_t numPoints = pointset.vertices.size() * numMeshes;
      return 1E-6*double(numPoints)/(t1-t0);
    }
  };

  class update_geometry : public Benchmark
  {
  public:
    RTCGeometryFlags flags; size_t numPhi; size_t numMeshes;
    update_geometry(const std::string& name, RTCGeometryFlags flags, size_t numPhi, size_t numMeshes)
      : Benchmark(name,"Mtris/s"), flags(flags), numPhi(numPhi), numMeshes(numMeshes) {}
  
    double run(size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      Mesh mesh; createSphereMesh (Vec3f(0,0,0), 1, numPhi, mesh);
      RTCScene scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,aflags);
      
      for (size_t i=0; i<numMeshes; i++) 
      {
	unsigned geom = rtcNewTriangleMesh (scene, flags, mesh.triangles.size(), mesh.vertices.size());
	memcpy(rtcMapBuffer(scene,geom,RTC_VERTEX_BUFFER), &mesh.vertices[0], mesh.vertices.size()*sizeof(Vertex));
	memcpy(rtcMapBuffer(scene,geom,RTC_INDEX_BUFFER ), &mesh.triangles[0], mesh.triangles.size()*sizeof(Triangle));
	rtcUnmapBuffer(scene,geom,RTC_VERTEX_BUFFER);
	rtcUnmapBuffer(scene,geom,RTC_INDEX_BUFFER);
	for (size_t i=0; i<mesh.vertices.size(); i++) {
	  mesh.vertices[i].x += 1.0f;
	  mesh.vertices[i].y += 1.0f;
	  mesh.vertices[i].z += 1.0f;
	}
      }
      rtcCommit (scene);
      double t0 = getSeconds();
      for (size_t i=0; i<numMeshes; i++) rtcUpdate(scene,i);
      rtcCommit (scene);
      double t1 = getSeconds();
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);
      
      //return 1000.0f*(t1-t0);
      size_t numTriangles = mesh.triangles.size() * numMeshes;
      return 1E-6*double(numTriangles)/(t1-t0);
    }
  };

  class update_geometry_line : public Benchmark
  {
  public:
    RTCGeometryFlags flags; size_t numLines; size_t numMeshes;
    update_geometry_line(const std::string& name, RTCGeometryFlags flags, size_t numLines, size_t numMeshes)
      : Benchmark(name,"Mlines/s"), flags(flags), numLines(numLines), numMeshes(numMeshes) {}

    double run(size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      LineSegments hairset; createHairball (Vec3f(0,0,0), 1, numLines, hairset);
      RTCScene scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,aflags);

      for (size_t i=0; i<numMeshes; i++)
      {
        unsigned geom = rtcNewLineSegments (scene, flags, hairset.lines.size(), hairset.vertices.size());
        memcpy(rtcMapBuffer(scene,geom,RTC_VERTEX_BUFFER), &hairset.vertices[0], hairset.vertices.size()*sizeof(Vec3fa));
        memcpy(rtcMapBuffer(scene,geom,RTC_INDEX_BUFFER ), &hairset.lines[0], hairset.lines.size()*sizeof(int));
        rtcUnmapBuffer(scene,geom,RTC_VERTEX_BUFFER);
        rtcUnmapBuffer(scene,geom,RTC_INDEX_BUFFER);
        for (size_t i=0; i<hairset.vertices.size(); i++) {
          hairset.vertices[i].x += 1.0f;
          hairset.vertices[i].y += 1.0f;
          hairset.vertices[i].z += 1.0f;
        }
      }
      rtcCommit (scene);
      double t0 = getSeconds();
      for (size_t i=0; i<numMeshes; i++) rtcUpdate(scene,i);
      rtcCommit (scene);
      double t1 = getSeconds();
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);

      //return 1000.0f*(t1-t0);
      size_t numLines = hairset.lines.size() * numMeshes;
      return 1E-6*double(numLines)/(t1-t0);
    }
  };

  class update_geometry_point : public Benchmark
  {
  public:
    RTCGeometryFlags flags; size_t numPoints; size_t numMeshes;
    update_geometry_point(const std::string& name, RTCGeometryFlags flags, size_t numPoints, size_t numMeshes)
      : Benchmark(name,"Mpoints/s"), flags(flags), numPoints(numPoints), numMeshes(numMeshes) {}

    double run(size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      Points pointset; createPointball (Vec3f(0,0,0), 1, numPoints, pointset);
      RTCScene scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,aflags);

      for (size_t i=0; i<numMeshes; i++)
      {
        unsigned geom = rtcNewPoints (scene, flags, pointset.vertices.size());
        memcpy(rtcMapBuffer(scene,geom,RTC_VERTEX_BUFFER), &pointset.vertices[0], pointset.vertices.size()*sizeof(Vec3fa));
        rtcUnmapBuffer(scene,geom,RTC_VERTEX_BUFFER);
        for (size_t i=0; i<pointset.vertices.size(); i++) {
          pointset.vertices[i].x += 1.0f;
          pointset.vertices[i].y += 1.0f;
          pointset.vertices[i].z += 1.0f;
        }
      }
      rtcCommit (scene);
      double t0 = getSeconds();
      for (size_t i=0; i<numMeshes; i++) rtcUpdate(scene,i);
      rtcCommit (scene);
      double t1 = getSeconds();
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);

      //return 1000.0f*(t1-t0);
      size_t numPoints = pointset.vertices.size() * numMeshes;
      return 1E-6*double(numPoints)/(t1-t0);
    }
  };

  class update_scenes : public Benchmark
  {
  public:
    RTCGeometryFlags flags; size_t numPhi; size_t numMeshes;
    update_scenes(const std::string& name, RTCGeometryFlags flags, size_t numPhi, size_t numMeshes)
      : Benchmark(name,"Mtris/s"), flags(flags), numPhi(numPhi), numMeshes(numMeshes) {}
  
    double run(size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      Mesh mesh; createSphereMesh (Vec3f(0,0,0), 1, numPhi, mesh);
      std::vector<RTCScene> scenes;
      
      for (size_t i=0; i<numMeshes; i++) 
      {
        RTCScene scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,aflags);
	unsigned geom = rtcNewTriangleMesh (scene, flags, mesh.triangles.size(), mesh.vertices.size());
	memcpy(rtcMapBuffer(scene,geom,RTC_VERTEX_BUFFER), &mesh.vertices[0], mesh.vertices.size()*sizeof(Vertex));
	memcpy(rtcMapBuffer(scene,geom,RTC_INDEX_BUFFER ), &mesh.triangles[0], mesh.triangles.size()*sizeof(Triangle));
	rtcUnmapBuffer(scene,geom,RTC_VERTEX_BUFFER);
	rtcUnmapBuffer(scene,geom,RTC_INDEX_BUFFER);
	for (size_t i=0; i<mesh.vertices.size(); i++) {
	  mesh.vertices[i].x += 1.0f;
	  mesh.vertices[i].y += 1.0f;
	  mesh.vertices[i].z += 1.0f;
	}
        scenes.push_back(scene);
        rtcCommit (scene);
      }
            
      double t0 = getSeconds();
#if defined(__MIC__)
      for (size_t i=0; i<scenes.size(); i++) {
        rtcUpdate(scenes[i],0);
        rtcCommit (scenes[i]);
      }
#else
      parallel_for( scenes.size(), [&](size_t i) { 
          rtcUpdate(scenes[i],0); 
          rtcCommit (scenes[i]);
        });
#endif
      
      double t1 = getSeconds();
      for (size_t i=0; i<scenes.size(); i++)
        rtcDeleteScene(scenes[i]);
      rtcDeleteDevice(device);
      
      //return 1000.0f*(t1-t0);
      size_t numTriangles = mesh.triangles.size() * numMeshes;
      return 1E-6*double(numTriangles)/(t1-t0);
    }
  };



  class update_keyframe_scenes : public Benchmark
  {
  public:
    RTCGeometryFlags flags; size_t numPhi; size_t numMeshes;
    update_keyframe_scenes(const std::string& name, RTCGeometryFlags flags, size_t numPhi, size_t numMeshes)
      : Benchmark(name,"Mtris/s"), flags(flags), numPhi(numPhi), numMeshes(numMeshes) {}
  
    double run(size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      Mesh mesh; 
      createSphereMesh (Vec3f(0,0,0), 1, numPhi, mesh);
      std::vector<RTCScene> scenes;
      
      const size_t sizeVertexData = mesh.vertices.size()*sizeof(Vertex);
      Vertex *key_frame_vertices[2];
      key_frame_vertices[0] = (Vertex*)os_malloc(sizeVertexData);
      key_frame_vertices[1] = (Vertex*)os_malloc(sizeVertexData);
      memcpy(key_frame_vertices[0],&mesh.vertices[0],sizeVertexData);
      memcpy(key_frame_vertices[1],&mesh.vertices[0],sizeVertexData);

      RTCScene scene = rtcDeviceNewScene(device,RTC_SCENE_DYNAMIC,aflags);
      unsigned geom = rtcNewTriangleMesh (scene, flags, mesh.triangles.size(), mesh.vertices.size());
      rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,key_frame_vertices[1],0,sizeof(Vec3fa));

      memcpy(rtcMapBuffer(scene,geom,RTC_INDEX_BUFFER ), &mesh.triangles[0], mesh.triangles.size()*sizeof(Triangle));
      rtcUnmapBuffer(scene,geom,RTC_INDEX_BUFFER);
      rtcCommit (scene);
      
      static const size_t anim_runs = 4;
      //static const size_t anim_runs = 4000000;

      double t0 = getSeconds();
      for (size_t i=0;i<anim_runs;i++)
      {
        rtcSetBuffer(scene,geom,RTC_VERTEX_BUFFER,key_frame_vertices[i%2],0,sizeof(Vec3fa));
        rtcUpdate(scene,0);
        rtcCommit(scene);
      }
      
      double t1 = getSeconds();
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);
      
      os_free(key_frame_vertices[0],sizeVertexData);
      os_free(key_frame_vertices[1],sizeVertexData);
      
      size_t numTriangles = mesh.triangles.size() * numMeshes;
      return 1E-6*double(numTriangles)*anim_runs/((t1-t0));;
    }
  };

  class benchmark_rtcore_intersect1_throughput : public Benchmark
  {
  public:
    enum { N = 1024*128 };
    static RTCScene scene;

    benchmark_rtcore_intersect1_throughput () 
      : Benchmark("incoherent_intersect1_throughput","MRays/s (all HW threads)") {}

    virtual void trace(Vec3f* numbers)
    {
    }

    static double benchmark_rtcore_intersect1_throughput_thread(void* arg) 
    {
      size_t threadIndex = (size_t) arg;
      size_t threadCount = g_num_threads;

      srand48(threadIndex*334124);
      Vec3f* numbers = new Vec3f[N];

#if 1
      for (size_t i=0; i<N; i++) {
        float x = 2.0f*drand48()-1.0f;
        float y = 2.0f*drand48()-1.0f;
        float z = 2.0f*drand48()-1.0f;
        numbers[i] = Vec3f(x,y,z);
      }
#else
#define NUM 512
      float rx[NUM];
      float ry[NUM];
      float rz[NUM];

      for (size_t i=0; i<NUM; i++) {
        rx[i] = drand48();
        ry[i] = drand48();
        rz[i] = drand48();
      }

      for (size_t i=0; i<N; i++) {
        float x = 2.0f*rx[i%NUM]-1.0f;
        float y = 2.0f*ry[i%NUM]-1.0f;
        float z = 2.0f*rz[i%NUM]-1.0f;
        numbers[i] = Vec3f(x,y,z);
      }      
#endif

      g_barrier_active.wait(threadIndex);
      double t0 = getSeconds();
      
      for (size_t i=0; i<N; i++) {
        RTCRay ray = makeRay(zero,numbers[i]);
        rtcIntersect(scene,ray);
      }        

      g_barrier_active.wait(threadIndex);
      double t1 = getSeconds();

      delete [] numbers;
      return t1-t0;
    }
    
    double run (size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      int numPhi = 501;
      //int numPhi = 61;
      //int numPhi = 1601;

      RTCSceneFlags flags = RTC_SCENE_STATIC;
      scene = rtcDeviceNewScene(device,flags,aflags);
      addSphere (scene, RTC_GEOMETRY_STATIC, zero, 1, numPhi);
      rtcCommit (scene);


      g_num_threads = numThreads;
      g_barrier_active.init(numThreads);
      for (size_t i=1; i<numThreads; i++)
	g_threads.push_back(createThread((thread_func)benchmark_rtcore_intersect1_throughput_thread,(void*)i,1000000,i));
      setAffinity(0);
      
      //g_barrier_active.wait(0);
      //double t0 = getSeconds();

      double delta = benchmark_rtcore_intersect1_throughput_thread(0);

      //g_barrier_active.wait(0);
      //double t1 = getSeconds();
      
      for (size_t i=0; i<g_threads.size(); i++)	join(g_threads[i]);
      g_threads.clear();
      
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);
      return 1E-6*double(N)/(delta)*double(numThreads);
    }
  };

  RTCScene benchmark_rtcore_intersect1_throughput::scene;




#if HAS_INTERSECT16

  class benchmark_rtcore_intersect16_throughput : public Benchmark
  {
  public:
    enum { N = 1024*128 };
    static RTCScene scene;

    benchmark_rtcore_intersect16_throughput () 
      : Benchmark("incoherent_intersect16_throughput","MRays/s (all HW threads)") {}

    static double benchmark_rtcore_intersect16_throughput_thread(void* arg) 
    {
      size_t threadIndex = (size_t) arg;
      size_t threadCount = g_num_threads;

      srand48(threadIndex*334124);
      Vec3f* numbers = new Vec3f[N];
      for (size_t i=0; i<N; i++) {
        float x = 2.0f*drand48()-1.0f;
        float y = 2.0f*drand48()-1.0f;
        float z = 2.0f*drand48()-1.0f;
        numbers[i] = Vec3f(x,y,z);
      }
      __aligned(16) int valid16[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };

      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
      double t0 = getSeconds();

      //while(1)
      for (size_t i=0; i<N; i+=16) {
        RTCRay16 ray16;
        for (size_t j=0;j<16;j++)        
          setRay(ray16,j,makeRay(zero,numbers[i+j]));
        rtcIntersect16(valid16,scene,ray16);
      }        
      if (threadIndex != 0) g_barrier_active.wait(threadIndex);
      double t1 = getSeconds();

      delete [] numbers;
      return t1-t0;
    }
    
    double run (size_t numThreads)
    {
      RTCDevice device = rtcNewDevice((g_rtcore+",threads="+toString(numThreads)).c_str());
      error_handler(rtcDeviceGetError(device));

      int numPhi = 501;

      RTCSceneFlags flags = RTC_SCENE_STATIC;
      scene = rtcDeviceNewScene(device,flags,aflags);
      addSphere (scene, RTC_GEOMETRY_STATIC, zero, 1, numPhi);
      rtcCommit (scene);


      g_num_threads = numThreads;
      g_barrier_active.init(numThreads);
      for (size_t i=1; i<numThreads; i++)
	g_threads.push_back(createThread((thread_func)benchmark_rtcore_intersect16_throughput_thread,(void*)i,1000000,i));
      setAffinity(0);
      
      g_barrier_active.wait(0);
      double t0 = getSeconds();

      double delta = benchmark_rtcore_intersect16_throughput_thread(0);

      g_barrier_active.wait(0);
      double t1 = getSeconds();
      
      for (size_t i=0; i<g_threads.size(); i++)	join(g_threads[i]);
      g_threads.clear();
      
      rtcDeleteScene(scene);
      rtcDeleteDevice(device);
      return 1E-6*double(N)/(delta)*double(numThreads);
    }
  };

  RTCScene benchmark_rtcore_intersect16_throughput::scene;
#endif


  void rtcore_coherent_intersect1(RTCScene scene)
  {
    size_t width = 1024;
    size_t height = 1024;
    float rcpWidth = 1.0f/1024.0f;
    float rcpHeight = 1.0f/1024.0f;
    double t0 = getSeconds();
    for (size_t y=0; y<height; y++) {
      for (size_t x=0; x<width; x++) {
        RTCRay ray = makeRay(zero,Vec3f(float(x)*rcpWidth,1,float(y)*rcpHeight));
        rtcIntersect(scene,ray);
      }
    }
    double t1 = getSeconds();

    printf("%40s ... %f Mrps\n","coherent_intersect1",1E-6*(double)(width*height)/(t1-t0));
    fflush(stdout);
  }

#if HAS_INTERSECT4
  void rtcore_coherent_intersect4(RTCScene scene)
  {
    size_t width = 1024;
    size_t height = 1024;
    float rcpWidth = 1.0f/1024.0f;
    float rcpHeight = 1.0f/1024.0f;
    double t0 = getSeconds();
    for (size_t y=0; y<height; y+=2) {
      for (size_t x=0; x<width; x+=2) {
        RTCRay4 ray4; 
        for (size_t dy=0; dy<2; dy++) {
          for (size_t dx=0; dx<2; dx++) {
            setRay(ray4,2*dy+dx,makeRay(zero,Vec3f(float(x+dx)*rcpWidth,1,float(y+dy)*rcpHeight)));
          }
        }
        __aligned(16) int valid4[4] = { -1,-1,-1,-1 };
        rtcIntersect4(valid4,scene,ray4);
      }
    }
    double t1 = getSeconds();

    printf("%40s ... %f Mrps\n","coherent_intersect4",1E-6*(double)(width*height)/(t1-t0));
	fflush(stdout);
  }
#endif

#if HAS_INTERSECT8
  void rtcore_coherent_intersect8(RTCScene scene)
  {
    size_t width = 1024;
    size_t height = 1024;
    float rcpWidth = 1.0f/1024.0f;
    float rcpHeight = 1.0f/1024.0f;
    double t0 = getSeconds();
    for (size_t y=0; y<height; y+=4) {
      for (size_t x=0; x<width; x+=2) {
        RTCRay8 ray8; 
        for (size_t dy=0; dy<4; dy++) {
          for (size_t dx=0; dx<2; dx++) {
            setRay(ray8,2*dy+dx,makeRay(zero,Vec3f(float(x+dx)*rcpWidth,1,float(y+dy)*rcpHeight)));
          }
        }
        __aligned(32) int valid8[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
        rtcIntersect8(valid8,scene,ray8);
      }
    }
    double t1 = getSeconds();

    printf("%40s ... %f Mrps\n","coherent_intersect8",1E-6*(double)(width*height)/(t1-t0));
	fflush(stdout);
  }
#endif

#if HAS_INTERSECT16
  void rtcore_coherent_intersect16(RTCScene scene)
  {
    size_t width = 1024;
    size_t height = 1024;
    float rcpWidth = 1.0f/1024.0f;
    float rcpHeight = 1.0f/1024.0f;
    double t0 = getSeconds();
    for (size_t y=0; y<height; y+=4) {
      for (size_t x=0; x<width; x+=4) {
        RTCRay16 ray16; 
        for (size_t dy=0; dy<4; dy++) {
          for (size_t dx=0; dx<4; dx++) {
            setRay(ray16,4*dy+dx,makeRay(zero,Vec3f(float(x+dx)*rcpWidth,1,float(y+dy)*rcpHeight)));
          }
        }
        __aligned(64) int valid16[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
        rtcIntersect16(valid16,scene,ray16);
      }
    }
    double t1 = getSeconds();

    printf("%40s ... %f Mrps\n","coherent_intersect16",1E-6*(double)(width*height)/(t1-t0));
	fflush(stdout);
  }
#endif

  void rtcore_incoherent_intersect1(RTCScene scene, Vec3f* numbers, size_t N)
  {
    double t0 = getSeconds();
    for (size_t i=0; i<N; i++) {
      RTCRay ray = makeRay(zero,numbers[i]);
      rtcIntersect(scene,ray);
    }
    double t1 = getSeconds();

    printf("%40s ... %f Mrps\n","incoherent_intersect1",1E-6*(double)N/(t1-t0));
	fflush(stdout);
  }

#if HAS_INTERSECT4
  void rtcore_incoherent_intersect4(RTCScene scene, Vec3f* numbers, size_t N)
  {
    double t0 = getSeconds();
    for (size_t i=0; i<N; i+=4) {
      RTCRay4 ray4;
      for (size_t j=0; j<4; j++) {
        setRay(ray4,j,makeRay(zero,numbers[i+j]));
      }
      __aligned(16) int valid4[4] = { -1,-1,-1,-1 };
      rtcIntersect4(valid4,scene,ray4);
    }
    double t1 = getSeconds();

    printf("%40s ... %f Mrps\n","incoherent_intersect4",1E-6*(double)N/(t1-t0));
	fflush(stdout);
  }
#endif

#if HAS_INTERSECT8
  void rtcore_incoherent_intersect8(RTCScene scene, Vec3f* numbers, size_t N)
  {
    double t0 = getSeconds();
    for (size_t i=0; i<N; i+=8) {
      RTCRay8 ray8;
      for (size_t j=0; j<8; j++) {
        setRay(ray8,j,makeRay(zero,numbers[i+j]));
      }
      __aligned(32) int valid8[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
      rtcIntersect8(valid8,scene,ray8);
    }
    double t1 = getSeconds();

    printf("%40s ... %f Mrps\n","incoherent_intersect8",1E-6*(double)N/(t1-t0));
	fflush(stdout);
  }
#endif

#if HAS_INTERSECT16
  void rtcore_incoherent_intersect16(RTCScene scene, Vec3f* numbers, size_t N)
  {
    double t0 = getSeconds();
    for (size_t i=0; i<N; i+=16) {
      RTCRay16 ray16;
      for (size_t j=0; j<16; j++) {
        setRay(ray16,j,makeRay(zero,numbers[i+j]));
      }
      __aligned(64) int valid16[16] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
      rtcIntersect16(valid16,scene,ray16);
    }
    double t1 = getSeconds();

    printf("%40s ... %f Mrps\n","incoherent_intersect16",1E-6*(double)N/(t1-t0));
	fflush(stdout);
  }
#endif

  void rtcore_intersect_benchmark(RTCSceneFlags flags, size_t numPhi)
  {
    RTCDevice device = rtcNewDevice(g_rtcore.c_str());
    error_handler(rtcDeviceGetError(device));

    RTCScene scene = rtcDeviceNewScene(device,flags,aflags);
    addSphere (scene, RTC_GEOMETRY_STATIC, zero, 1, numPhi);
    rtcCommit (scene);
    
    rtcore_coherent_intersect1(scene);
#if HAS_INTERSECT4
    rtcore_coherent_intersect4(scene);
#endif

#if HAS_INTERSECT8
    if (hasISA(AVX)) {
      rtcore_coherent_intersect8(scene);
    }
#endif

#if HAS_INTERSECT16
    if (hasISA(AVX512KNL) || hasISA(KNC)) {    
      rtcore_coherent_intersect16(scene);
    }
#endif

    size_t N = 1024*1024;
    Vec3f* numbers = new Vec3f[N];
    for (size_t i=0; i<N; i++) {
      float x = 2.0f*drand48()-1.0f;
      float y = 2.0f*drand48()-1.0f;
      float z = 2.0f*drand48()-1.0f;
      numbers[i] = Vec3f(x,y,z);
    }

    rtcore_incoherent_intersect1(scene,numbers,N);
#if HAS_INTERSECT4
    rtcore_incoherent_intersect4(scene,numbers,N);
#endif

#if HAS_INTERSECT8
    if (hasISA(AVX)) {
      rtcore_incoherent_intersect8(scene,numbers,N);
    }
#endif

#if HAS_INTERSECT16
    if (hasISA(AVX512KNL) || hasISA(KNC)) {
      rtcore_incoherent_intersect16(scene,numbers,N);
    }
#endif

    delete[] numbers;

    rtcDeleteScene(scene);
    rtcDeleteDevice(device);
  }



  
  std::vector<Benchmark*> benchmarks;

  void create_benchmarks()
  {
#if 1
    benchmarks.push_back(new benchmark_rtcore_intersect1_throughput());

#if HAS_INTERSECT16
    if (hasISA(AVX512KNL) || hasISA(KNC)) {
      benchmarks.push_back(new benchmark_rtcore_intersect16_throughput());
    }
#endif

    benchmarks.push_back(new benchmark_mutex_sys());
    benchmarks.push_back(new benchmark_barrier_sys());
    benchmarks.push_back(new benchmark_barrier_active());

    benchmarks.push_back(new benchmark_atomic_inc());
#if defined(__X86_64__)
    benchmarks.push_back(new benchmark_osmalloc_with_page_commit());
    benchmarks.push_back(new benchmark_pagefaults());
    benchmarks.push_back(new benchmark_bandwidth());
#endif

 
    benchmarks.push_back(new create_geometry ("create_static_geometry_120",      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,6,1));
    benchmarks.push_back(new create_geometry ("create_static_geometry_1k" ,      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,17,1));
    benchmarks.push_back(new create_geometry ("create_static_geometry_10k",      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,51,1));
    benchmarks.push_back(new create_geometry ("create_static_geometry_100k",     RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,159,1));
    benchmarks.push_back(new create_geometry ("create_static_geometry_1000k_1",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,501,1));
    benchmarks.push_back(new create_geometry ("create_static_geometry_100k_10",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,159,10));
    benchmarks.push_back(new create_geometry ("create_static_geometry_10k_100",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,51,100));
    benchmarks.push_back(new create_geometry ("create_static_geometry_1k_1000" , RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,17,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new create_geometry ("create_static_geometry_120_10000",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,6,8334));
#endif

    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_120",      RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,6,1));
    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_1k" ,      RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,17,1));
    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_10k",      RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,51,1));
    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_100k",     RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,159,1));
    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_1000k_1",  RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,501,1));
    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_100k_10",  RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,159,10));
    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_10k_100",  RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,51,100));
    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_1k_1000" , RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,17,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new create_geometry ("create_dynamic_geometry_120_10000",RTC_SCENE_DYNAMIC,RTC_GEOMETRY_STATIC,6,8334));
#endif

#endif

    benchmarks.push_back(new update_geometry ("refit_geometry_120",      RTC_GEOMETRY_DEFORMABLE,6,1));
    benchmarks.push_back(new update_geometry ("refit_geometry_1k" ,      RTC_GEOMETRY_DEFORMABLE,17,1));
    benchmarks.push_back(new update_geometry ("refit_geometry_10k",      RTC_GEOMETRY_DEFORMABLE,51,1));
    benchmarks.push_back(new update_geometry ("refit_geometry_100k",     RTC_GEOMETRY_DEFORMABLE,159,1));
    benchmarks.push_back(new update_geometry ("refit_geometry_1000k_1",  RTC_GEOMETRY_DEFORMABLE,501,1));


    benchmarks.push_back(new update_geometry ("refit_geometry_100k_10",  RTC_GEOMETRY_DEFORMABLE,159,10));
    benchmarks.push_back(new update_geometry ("refit_geometry_10k_100",  RTC_GEOMETRY_DEFORMABLE,51,100));
    benchmarks.push_back(new update_geometry ("refit_geometry_1k_1000" , RTC_GEOMETRY_DEFORMABLE,17,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new update_geometry ("refit_geometry_120_10000",RTC_GEOMETRY_DEFORMABLE,6,8334));
#endif

#if 1
    
    benchmarks.push_back(new update_geometry ("update_geometry_120",      RTC_GEOMETRY_DYNAMIC,6,1));
    benchmarks.push_back(new update_geometry ("update_geometry_1k" ,      RTC_GEOMETRY_DYNAMIC,17,1));
    benchmarks.push_back(new update_geometry ("update_geometry_10k",      RTC_GEOMETRY_DYNAMIC,51,1));
    benchmarks.push_back(new update_geometry ("update_geometry_100k",     RTC_GEOMETRY_DYNAMIC,159,1));
    benchmarks.push_back(new update_geometry ("update_geometry_1000k_1",  RTC_GEOMETRY_DYNAMIC,501,1));
    benchmarks.push_back(new update_geometry ("update_geometry_100k_10",  RTC_GEOMETRY_DYNAMIC,159,10));
    benchmarks.push_back(new update_geometry ("update_geometry_10k_100",  RTC_GEOMETRY_DYNAMIC,51,100));
    benchmarks.push_back(new update_geometry ("update_geometry_1k_1000" , RTC_GEOMETRY_DYNAMIC,17,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new update_geometry ("update_geometry_120_10000",RTC_GEOMETRY_DYNAMIC,6,8334));
#endif


    benchmarks.push_back(new update_scenes ("refit_scenes_120",      RTC_GEOMETRY_DEFORMABLE,6,1));
    benchmarks.push_back(new update_scenes ("refit_scenes_1k" ,      RTC_GEOMETRY_DEFORMABLE,17,1));
    benchmarks.push_back(new update_scenes ("refit_scenes_10k",      RTC_GEOMETRY_DEFORMABLE,51,1));
    benchmarks.push_back(new update_scenes ("refit_scenes_100k",     RTC_GEOMETRY_DEFORMABLE,159,1));
    benchmarks.push_back(new update_scenes ("refit_scenes_1000k_1",  RTC_GEOMETRY_DEFORMABLE,501,1));
#if defined(__X86_64__)
    benchmarks.push_back(new update_scenes ("refit_scenes_8000k_1",  RTC_GEOMETRY_DEFORMABLE,1420,1));
#endif
    benchmarks.push_back(new update_scenes ("refit_scenes_100k_10",  RTC_GEOMETRY_DEFORMABLE,159,10));
#if !defined(__MIC__)
    benchmarks.push_back(new update_scenes ("refit_scenes_10k_100",  RTC_GEOMETRY_DEFORMABLE,51,100));
    benchmarks.push_back(new update_scenes ("refit_scenes_1k_1000" , RTC_GEOMETRY_DEFORMABLE,17,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new update_scenes ("refit_scenes_120_10000",RTC_GEOMETRY_DEFORMABLE,6,8334));
#endif

#endif

#if defined(__X86_64__)
    benchmarks.push_back(new update_keyframe_scenes ("refit_keyframe_scenes_1000k_1",  RTC_GEOMETRY_DEFORMABLE,501,1));
    benchmarks.push_back(new update_keyframe_scenes ("refit_keyframe_scenes_8000k_1",  RTC_GEOMETRY_DEFORMABLE,1420,1));
#endif    

    benchmarks.push_back(new update_scenes ("update_scenes_120",      RTC_GEOMETRY_DYNAMIC,6,1));
    benchmarks.push_back(new update_scenes ("update_scenes_1k" ,      RTC_GEOMETRY_DYNAMIC,17,1));
    benchmarks.push_back(new update_scenes ("update_scenes_10k",      RTC_GEOMETRY_DYNAMIC,51,1));
    benchmarks.push_back(new update_scenes ("update_scenes_100k",     RTC_GEOMETRY_DYNAMIC,159,1));
    benchmarks.push_back(new update_scenes ("update_scenes_1000k_1",  RTC_GEOMETRY_DYNAMIC,501,1));
#if defined(__X86_64__)
    benchmarks.push_back(new update_scenes ("update_scenes_8000k_1",  RTC_GEOMETRY_DYNAMIC,1420,1));
#endif
    benchmarks.push_back(new update_scenes ("update_scenes_100k_10",  RTC_GEOMETRY_DYNAMIC,159,10));
#if !defined(__MIC__)
    benchmarks.push_back(new update_scenes ("update_scenes_10k_100",  RTC_GEOMETRY_DYNAMIC,51,100));
    benchmarks.push_back(new update_scenes ("update_scenes_1k_1000" , RTC_GEOMETRY_DYNAMIC,17,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new update_scenes ("update_scenes_120_10000",RTC_GEOMETRY_DYNAMIC,6,8334));
#endif

#if defined(__X86_64__)
    benchmarks.push_back(new update_keyframe_scenes ("update_keyframe_scenes_1000k_1",  RTC_GEOMETRY_DYNAMIC,501,1));
    benchmarks.push_back(new update_keyframe_scenes ("update_keyframe_scenes_8000k_1",  RTC_GEOMETRY_DYNAMIC,1420,1));
#endif    

#endif

#endif

    //benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_120",      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,120,1));
    //benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_1k" ,      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,1*1000,1));
    //benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_10k",      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,10*1000,1));
    benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_100k",     RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,100*1000,1));
    benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_1000k_1",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,1000*1000,1));
    benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_100k_10",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,100*1000,10));
    benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_10k_100",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,10*1000,100));
    benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_1k_1000" , RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,1*1000,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new create_geometry_line ("create_static_geometry_line_120_10000",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,120,10000));
#endif


    //benchmarks.push_back(new update_geometry_line ("refit_geometry_line_120",      RTC_GEOMETRY_DEFORMABLE,120,1));
    //benchmarks.push_back(new update_geometry_line ("refit_geometry_line_1k" ,      RTC_GEOMETRY_DEFORMABLE,1*1000,1));
    //benchmarks.push_back(new update_geometry_line ("refit_geometry_line_10k",      RTC_GEOMETRY_DEFORMABLE,10*1000,1));
    benchmarks.push_back(new update_geometry_line ("refit_geometry_line_100k",     RTC_GEOMETRY_DEFORMABLE,100*1000,1));
    benchmarks.push_back(new update_geometry_line ("refit_geometry_line_1000k_1",  RTC_GEOMETRY_DEFORMABLE,1000*1000,1));
    benchmarks.push_back(new update_geometry_line ("refit_geometry_line_8000k_1",  RTC_GEOMETRY_DEFORMABLE,1000*1000*8,1));

    benchmarks.push_back(new update_geometry_line ("refit_geometry_line_100k_10",  RTC_GEOMETRY_DEFORMABLE,100*1000,10));
    benchmarks.push_back(new update_geometry_line ("refit_geometry_line_10k_100",  RTC_GEOMETRY_DEFORMABLE,10*1000,100));
    benchmarks.push_back(new update_geometry_line ("refit_geometry_line_1k_1000" , RTC_GEOMETRY_DEFORMABLE,1*1000,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new update_geometry_line ("refit_geometry_line_120_10000",RTC_GEOMETRY_DEFORMABLE,120,10000));
#endif

    //benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_120",      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,120,1));
    //benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_1k" ,      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,1*1000,1));
    //benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_10k",      RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,10*1000,1));
    benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_100k",     RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,100*1000,1));
    benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_1000k_1",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,1000*1000,1));
    benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_100k_10",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,100*1000,10));
    benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_10k_100",  RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,10*1000,100));
    benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_1k_1000" , RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,1*1000,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new create_geometry_point ("create_static_geometry_point_120_10000",RTC_SCENE_STATIC,RTC_GEOMETRY_STATIC,120,10000));
#endif


    //benchmarks.push_back(new update_geometry_point ("refit_geometry_point_120",      RTC_GEOMETRY_DEFORMABLE,120,1));
    //benchmarks.push_back(new update_geometry_point ("refit_geometry_point_1k" ,      RTC_GEOMETRY_DEFORMABLE,1*1000,1));
    //benchmarks.push_back(new update_geometry_point ("refit_geometry_point_10k",      RTC_GEOMETRY_DEFORMABLE,10*1000,1));
    benchmarks.push_back(new update_geometry_point ("refit_geometry_point_100k",     RTC_GEOMETRY_DEFORMABLE,100*1000,1));
    benchmarks.push_back(new update_geometry_point ("refit_geometry_point_1000k_1",  RTC_GEOMETRY_DEFORMABLE,1000*1000,1));
    benchmarks.push_back(new update_geometry_point ("refit_geometry_point_8000k_1",  RTC_GEOMETRY_DEFORMABLE,1000*1000*8,1));

    benchmarks.push_back(new update_geometry_point ("refit_geometry_point_100k_10",  RTC_GEOMETRY_DEFORMABLE,100*1000,10));
    benchmarks.push_back(new update_geometry_point ("refit_geometry_point_10k_100",  RTC_GEOMETRY_DEFORMABLE,10*1000,100));
    benchmarks.push_back(new update_geometry_point ("refit_geometry_point_1k_1000" , RTC_GEOMETRY_DEFORMABLE,1*1000,1000));
#if defined(__X86_64__)
    benchmarks.push_back(new update_geometry_point ("refit_geometry_point_120_10000",RTC_GEOMETRY_DEFORMABLE,120,10000));
#endif
  }

  Benchmark* getBenchmark(const std::string& str)
  {
    for (size_t i=0; i<benchmarks.size(); i++) 
      if (benchmarks[i]->name == str) 
	return benchmarks[i];

    std::cout << "unknown benchmark: " << str << std::endl;
    exit(1);
  }

  void plot_scalability()
  {
    Benchmark* benchmark = getBenchmark(g_plot_test);
    //std::cout << "set terminal gif" << std::endl;
    //std::cout << "set output\"" << benchmark->name << "\"" << std::endl;
    std::cout << "set key inside right top vertical Right noreverse enhanced autotitles box linetype -1 linewidth 1.000" << std::endl;
    std::cout << "set samples 50, 50" << std::endl;
    std::cout << "set title \"" << benchmark->name << "\"" << std::endl; 
    std::cout << "set xlabel \"threads\"" << std::endl;
    std::cout << "set ylabel \"" << benchmark->unit << "\"" << std::endl;
    std::cout << "plot \"-\" using 0:2 title \"" << benchmark->name << "\" with lines" << std::endl;
	
    for (size_t i=g_plot_min; i<=g_plot_max; i+= g_plot_step) 
    {
      double pmin = inf, pmax = -float(inf), pavg = 0.0f;
      size_t N = 8;
      for (size_t j=0; j<N; j++) {
	double p = benchmark->run(i);
	pmin = min(pmin,p);
	pmax = max(pmax,p);
	pavg = pavg + p/double(N);
      }
      //std::cout << "threads = " << i << ": [" << pmin << " / " << pavg << " / " << pmax << "] " << benchmark->unit << std::endl;
      std::cout << " " << i << " " << pmin << " " << pavg << " " << pmax << std::endl;
    }
    std::cout << "EOF" << std::endl;
  }

  static void parseCommandLine(int argc, char** argv)
  {
    for (int i=1; i<argc; i++)
    {
      std::string tag = argv[i];
      if (tag == "") return;

      /* rtcore configuration */
      else if (tag == "-rtcore" && i+1<argc) {
        g_rtcore = argv[++i];
      }

      /* plots scalability graph */
      else if (tag == "-plot" && i+4<argc) {
	g_plot_min = atoi(argv[++i]);
	g_plot_max = atoi(argv[++i]);
	g_plot_step= atoi(argv[++i]);
	g_plot_test= argv[++i];
	plot_scalability();
      }

      /* run single benchmark */
      else if (tag == "-run" && i+2<argc) 
      {
	size_t numThreads = atoi(argv[++i]);
	std::string name = argv[++i];
	Benchmark* benchmark = getBenchmark(name);
	benchmark->print(numThreads,16);
        executed_benchmarks = true;
      }

      else if (tag == "-threads" && i+1<argc) 
      {
	g_num_threads_init = atoi(argv[++i]);
      }

      /* skip unknown command line parameter */
      else {
        std::cerr << "unknown command line parameter: " << tag << " ";
        std::cerr << std::endl;
      }
    }
  }

  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    create_benchmarks();

    /* parse command line */  
    parseCommandLine(argc,argv);

    if (!executed_benchmarks) 
    {
      size_t numThreads = getNumberOfLogicalThreads();
      printf("%40s ... %d \n","#HW threads ",(int)getNumberOfLogicalThreads());
      
#if defined (__MIC__)
      numThreads -= 4;
#endif
      if (g_num_threads_init != -1)
      {
        numThreads = g_num_threads_init;
        PRINT(numThreads);
      }

      rtcore_intersect_benchmark(RTC_SCENE_STATIC, 501);

      for (size_t i=0; i<benchmarks.size(); i++) benchmarks[i]->print(numThreads,4);
    }

    return 0;
  }
}

int main(int argc, char** argv)
{
  try {
    return embree::main(argc, argv);
  }
  catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return 1;
  }
  catch (...) {
    std::cout << "Error: unknown exception caught." << std::endl;
    return 1;
  }
}
