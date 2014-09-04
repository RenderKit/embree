// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "sys/platform.h"
#include "sys/ref.h"
#include "embree2/rtcore.h"
#include "embree2/rtcore_ray.h"
#include "../kernels/common/default.h"
#include "../kernels/common/raystream_log.h"
#include "sys/thread.h"
#include "sys/sysinfo.h"
#include "sys/sync/barrier.h"
#include "sys/sync/mutex.h"
#include "sys/sync/condition.h"
#include <vector>
#include <iostream>
#include <fstream>

#define DBG(x) x

namespace embree
{
  struct RayStreamStats
  {
    size_t numTotalRays;
    size_t numRayPackets;
    size_t numIntersectRayPackets;
    size_t numOccludedRayPackets;
    size_t numIntersectRays;
    size_t numOccludedRays;
    size_t num4widePackets;
    size_t num8widePackets;

    RayStreamStats() {
      memset(this,0,sizeof(RayStreamStats));
    }

    void add(RayStreamLogger::LogRay16 &r)
    {
      size_t numRays = popcnt((int)r.m_valid);
      numRayPackets++;
      numTotalRays += numRays;
      if (r.type == RayStreamLogger::RAY_INTERSECT)
	{
	  numIntersectRayPackets++;
	  numIntersectRays += numRays;
	}
      else if (r.type == RayStreamLogger::RAY_OCCLUDED)
	{
	  numOccludedRayPackets++;
	  numOccludedRays += numRays;
	}
      else
	FATAL("unknown log ray type");
      num4widePackets += (numRays+3)/4;
      num8widePackets += (numRays+7)/8;
    }

    void add(RayStreamLogger::LogRay1 &r)
    {
      numRayPackets++;
      numTotalRays ++;
      if (r.type == RayStreamLogger::RAY_INTERSECT)
	{
	  numIntersectRayPackets++;
	  numIntersectRays++;
	}
      else if (r.type == RayStreamLogger::RAY_OCCLUDED)
	{
	  numOccludedRayPackets++;
	  numOccludedRays++;
	}
      else
	FATAL("unknown log ray type");
      num4widePackets += 1;
      num8widePackets += 1;
    }

    void print(size_t simd_width)
    {
      std::cout << "numTotalRays                      = " << numTotalRays << std::endl;
      std::cout << "numRayPackets                     = " << numRayPackets << std::endl;
      std::cout << "numIntersectionRays               = " << numIntersectRays << " [" << 100. * (double)numIntersectRays / numTotalRays << "%]" << std::endl;
      std::cout << "numOcclusionRays                  = " << numOccludedRays << " [" << 100. * (double)numOccludedRays / numTotalRays << "%]" << std::endl;
      if (simd_width > 1)
        {
          std::cout << "avg. intersect packet utilization = " << 100. *  (double)numIntersectRays / (numIntersectRayPackets * (double)simd_width) << "%" << std::endl;
          std::cout << "avg. occluded  packet utilization = " << 100. *  (double)numOccludedRays  / (numOccludedRayPackets  * (double)simd_width) << "%" << std::endl;
          std::cout << "avg. total packet utilization     = " << 100. * (double)numTotalRays / (numRayPackets * (double)simd_width)  << "%" << std::endl;
        }
      if (simd_width == 16)
        {
          std::cout << "avg. 4-wide packet utilization    = " << 100. * (double)numTotalRays / (num4widePackets * 4.)  << "%" << std::endl;
          std::cout << "avg. 8-wide packet utilization    = " << 100. * (double)numTotalRays / (num8widePackets * 8.)  << "%" << std::endl;
        }
    } 
  };


  struct RetraceTask
  {
    RTCScene scene;
    void *raydata;
    void *raydata_verify;
    size_t numLogRayStreamElements;
    bool check;
  };  

  /* configuration */

#if !defined(__MIC__)
  RTCAlgorithmFlags aflags = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8);
  static std::string g_rtcore = "verbose=2,threads=1";
#else
  RTCAlgorithmFlags aflags = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT16);
  static std::string g_rtcore = "verbose=2,threads=1";
#endif

  /* vertex and triangle layout */
  struct Vertex   { float x,y,z,a; };
  struct Triangle { int v0, v1, v2; };

  static AlignedAtomicCounter32 g_counter = 0;
  static bool g_check = false;
  static size_t g_numThreads = 1;
  static size_t g_frames = 1;
  static size_t g_simd_width = 0;
  static AlignedAtomicCounter32 g_rays_traced = 0;
  static std::vector<thread_t> g_threads;
  static LinearBarrierActive g_barrier;
  static bool g_exitThreads = false;
  static RetraceTask g_retraceTask;
  static MutexSys g_mutex;

    
#if defined(__MIC__)
  static std::string g_binaries_path = "/home/micuser/";
#else
  static std::string g_binaries_path = "./";
#endif


#define AssertNoError() \
  if (rtcGetError() != RTC_NO_ERROR) return false;
#define AssertAnyError() \
  if (rtcGetError() == RTC_NO_ERROR) return false;
#define AssertError(code) \
  if (rtcGetError() != code) return false;

  static void parseCommandLine(int argc, char** argv)
  {
    for (int i=1; i<argc; i++)
    {
      std::string tag = argv[i];
      if (tag == "") return;

      else if (tag == "-rtcore" && i+1<argc) {
        g_rtcore += std::stringOf(',') + argv[++i];
      }
      /* rtcore configuration */
      else if (tag == "-check") {
        g_check = true;
      }      
      else if (tag == "-threads" && i+1<argc) {
        g_numThreads = atoi(argv[++i]);
      }
      else if (tag == "-frames" && i+1<argc) {
        g_frames = atoi(argv[++i]);
      }
      else if (tag == "-simd_width" && i+1<argc) {
        g_simd_width = atoi(argv[++i]);
      }

      /* skip unknown command line parameter */
      else {

	g_binaries_path = tag;
        // std::cerr << "unknown command line parameter: " << tag << " ";
        // std::cerr << std::std::endl;
      }
    }
  }


  bool existsFile(std::string &filename)
  {
    std::ifstream file;
    file.open(filename.c_str(),std::ios::in | std::ios::binary);
    if (!file) return false;
    file.close();
    return true;    
  }

  void *loadGeometryData(std::string &geometryFile)
  {
    std::ifstream geometryData;

    geometryData.open(geometryFile.c_str(),std::ios::in | std::ios::binary);
    if (!geometryData) { FATAL("could not open geometry data file"); }
    geometryData.seekg(0, std::ios::beg);
    std::streampos begin = geometryData.tellg();
    geometryData.seekg(0, std::ios::end);
    std::streampos end = geometryData.tellg();
    size_t fileSize = end - begin;
    char *ptr = (char*)os_malloc(fileSize);
    geometryData.seekg(0, std::ios::beg);
    geometryData.read(ptr,fileSize);
    //geometryData.close();
    return ptr;
  }

  template<class T>
  void *loadRayStreamData(std::string &rayStreamFile, size_t &numLogRayStreamElements)
  {
    std::ifstream rayStreamData;

    rayStreamData.open(rayStreamFile.c_str(),std::ios::in | std::ios::binary);
    if (!rayStreamData) { FATAL("could not open raystream data file"); }
    rayStreamData.seekg(0, std::ios::beg);
    std::streampos begin = rayStreamData.tellg();
    rayStreamData.seekg(0, std::ios::end);
    std::streampos end = rayStreamData.tellg();
    size_t fileSize = end - begin;
    char *ptr = (char*)os_malloc(fileSize);
    rayStreamData.seekg(0, std::ios::beg);
    rayStreamData.read(ptr,fileSize);
    numLogRayStreamElements = fileSize / sizeof(T);
    //rayStreamData.close();
    return ptr;
  }

  template<class T>
  RayStreamStats analyseRayStreamData(void *r, size_t numLogRayStreamElements)
  {
    RayStreamStats stats;
    std::cout << "numLogRayStreamElements " << numLogRayStreamElements << std::endl;
    for (size_t i=0;i<numLogRayStreamElements;i++)
      stats.add(((T*)r)[i]);
    return stats;
  }

  RTCScene transferGeometryData(char *g)
  {
    RTCScene scene = rtcNewScene(RTC_SCENE_STATIC,aflags);

    size_t numGroups = *(size_t*)g;
    g += sizeof(size_t);
    size_t numTotalTriangles = *(size_t*)g;
    g += sizeof(size_t);
    DBG_PRINT(numGroups);
    DBG_PRINT(numTotalTriangles);

    for (size_t i=0; i<numGroups; i++) 
      {
	size_t numVertices = *(size_t*)g;
	g += sizeof(size_t);
	size_t numTriangles = *(size_t*)g;

        DBG(
            DBG_PRINT(numVertices);
            DBG_PRINT(numTriangles);
            );

	g += sizeof(size_t);
	Vertex *vtx = (Vertex*)g;
	g += sizeof(Vertex)*numVertices;
	Triangle *tri = (Triangle *)g;
	g += sizeof(Triangle)*numTriangles;
	if (((size_t)g % 16) != 0)
	  g += 16 - ((size_t)g % 16);

	if ((size_t)vtx % 16 != 0)
	  FATAL("vtx array alignment");

	unsigned int geometry = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, numTriangles, numVertices);
	rtcSetBuffer(scene, geometry, RTC_VERTEX_BUFFER, vtx, 0, sizeof(Vec3fa      ));
	rtcSetBuffer(scene, geometry, RTC_INDEX_BUFFER,  tri, 0, sizeof(Triangle));
      }

    rtcCommit(scene);
    return scene;
  }

  size_t check_ray1_packets(RTCRay &start, RTCRay &end)
  {

    DBG(
        DBG_PRINT(start.org[0]);
        DBG_PRINT(start.org[1]);
        DBG_PRINT(start.org[2]);

        DBG_PRINT(start.dir[0]);
        DBG_PRINT(start.dir[1]);
        DBG_PRINT(start.dir[2]);

        DBG_PRINT(start.tnear);
        DBG_PRINT(start.tfar);
        DBG_PRINT(start.u);
        DBG_PRINT(start.v);
        DBG_PRINT(start.primID);
        DBG_PRINT(start.geomID);

        DBG_PRINT(end.org[0]);
        DBG_PRINT(end.org[1]);
        DBG_PRINT(end.org[2]);

        DBG_PRINT(end.dir[0]);
        DBG_PRINT(end.dir[1]);
        DBG_PRINT(end.dir[2]);

        DBG_PRINT(end.tnear);
        DBG_PRINT(end.tfar);
        DBG_PRINT(end.u);
        DBG_PRINT(end.v);
        DBG_PRINT(end.primID);
        DBG_PRINT(end.geomID);
        );

    if (start.primID != end.primID) return 1;
    if (start.geomID != end.geomID) return 1;
    if (start.u != end.u) return 1;
    if (start.v != end.v) return 1;
    if (start.tfar != end.tfar) return 1;
    return 0;
  }

  size_t check_ray16_packets(const unsigned int i_valid, RTCRay16 &start, RTCRay16 &end)
  {
#if defined(__MIC__)
    const mic_m m_valid = (mic_m)i_valid;
    mic_i start_primID = load16i(start.primID);
    mic_i end_primID   = load16i(end.primID);

    mic_i start_geomID = load16i(start.geomID);
    mic_i end_geomID   = load16i(end.geomID);

    mic_f start_u = load16f(start.u);
    mic_f end_u   = load16f(end.u);

    mic_f start_v = load16f(start.v);
    mic_f end_v   = load16f(end.v);

    mic_f start_t = load16f(start.tfar);
    mic_f end_t   = load16f(end.tfar);

    const mic_m m_primID = eq(m_valid,start_primID,end_primID);
    const mic_m m_geomID = eq(m_valid,start_geomID,end_geomID);
    const mic_m m_u      = eq(m_valid,start_u,end_u);
    const mic_m m_v      = eq(m_valid,start_v,end_v);
    const mic_m m_t      = eq(m_valid,start_t,end_t);

    if ( m_primID != m_valid )
      {
	DBG(
	    DBG_PRINT(m_valid);
	    DBG_PRINT(m_primID);
	    DBG_PRINT(start_primID);
	    DBG_PRINT(end_primID);
	    );
	return countbits(m_primID^m_valid);
      }

    if ( m_geomID != m_valid )
      {
	DBG(
	    DBG_PRINT( m_valid );
	    DBG_PRINT( m_geomID );
	    DBG_PRINT( start_geomID );
	    DBG_PRINT( end_geomID );
	    );
	return countbits(m_geomID^m_valid);
      }

    if ( m_u != m_valid )
      {
	DBG(
	    DBG_PRINT( m_valid );
	    DBG_PRINT( m_u );
	    DBG_PRINT( start_u );
	    DBG_PRINT( end_u );
	    );
	return countbits(m_u^m_valid);
      }

    if ( m_v != m_valid )
      {
	DBG(
	    DBG_PRINT( m_valid );
	    DBG_PRINT( m_v );
	    DBG_PRINT( start_v );
	    DBG_PRINT( end_v );
	    );
	return countbits(m_v^m_valid);
      }

    if ( m_t != m_valid )
      {
	DBG(
	    DBG_PRINT( m_valid );
	    DBG_PRINT( m_t );
	    DBG_PRINT( start_t );
	    DBG_PRINT( end_t );
	    );
	return countbits(m_t^m_valid);
      }
#endif
    return 0;
  }

#define RAY_BLOCK_SIZE 16

  template<size_t SIMD_WIDTH>
  void retrace_loop()
  {
    size_t rays = 0;
    size_t diff = 0;
    
    while(1)
      {
	size_t global_index = g_counter.add(RAY_BLOCK_SIZE);
	if (global_index >= g_retraceTask.numLogRayStreamElements) break;
	size_t startID = global_index;
	size_t endID   = min(g_retraceTask.numLogRayStreamElements,startID+RAY_BLOCK_SIZE);

	for (size_t index=startID;index<endID;index++)
	  {
            if (SIMD_WIDTH == 1)
              {
                RayStreamLogger::LogRay1 *raydata        = (RayStreamLogger::LogRay1 *)g_retraceTask.raydata;
                RayStreamLogger::LogRay1 *raydata_verify = (RayStreamLogger::LogRay1 *)g_retraceTask.raydata_verify;

                RTCRay &ray = raydata[index].ray;

                DBG_PRINT(index);

                rays ++;
                if (raydata[index].type == RayStreamLogger::RAY_INTERSECT)
                  {
                    rtcIntersect(g_retraceTask.scene,ray);
                  }
                else 
                  rtcOccluded(g_retraceTask.scene,ray);

                if (unlikely(g_check))
                  {
                    diff += check_ray1_packets(ray, raydata_verify[index].ray);                    
                    if (diff != 0) FATAL("HERE");
                  }
              }
            else if (SIMD_WIDTH == 16)
              {
                RayStreamLogger::LogRay16 *raydata        = (RayStreamLogger::LogRay16 *)g_retraceTask.raydata;
                RayStreamLogger::LogRay16 *raydata_verify = (RayStreamLogger::LogRay16 *)g_retraceTask.raydata_verify;
#if defined(__MIC__)
                RTCRay16 &ray16 = raydata[index].ray16;
                mic_i valid = select((mic_m)r[index].m_valid,mic_i(-1),mic_i(0));
                rays += countbits( (mic_m)raydata[index].m_valid );

                raydata[index+1].prefetchL2();

                if (raydata[index].type == RayStreamLogger::RAY_INTERSECT)
                  rtcIntersect16(&valid,g_retraceTask.scene,ray16);
                else 
                  rtcOccluded16(&valid,g_retraceTask.scene,ray16);
#endif
                if (unlikely(g_check))
                  diff += check_ray16_packets(raydata[index].m_valid,  raydata[index].ray16, raydata_verify[index].ray16);
               
              }


	  }
      }
    if (unlikely(g_check && diff))
      {
	DBG_PRINT(diff);
	DBG_PRINT(100. * diff / rays);
      }
    g_rays_traced.add(rays);
  }



  void renderMainLoop(size_t id)
  {

    // g_mutex.lock();        
    // std::cout << "thread " << id << std::endl;
    // g_mutex.unlock();    
    switch(g_simd_width)
      {
      case 1:
        retrace_loop<1>();
        break;
      case 16:
        retrace_loop<16>();
        break;
      };
  }

  void threadMainLoop(void *ptr)
  {
    size_t id = (size_t)ptr;
    setAffinity(id);

    g_mutex.lock();
    DBG_PRINT(id);
    g_mutex.unlock();

    while(1)
      {
        g_barrier.wait(id,g_numThreads);
    
        if (g_exitThreads) break;

        renderMainLoop(id);

        g_barrier.wait(id,g_numThreads);
      }

    g_mutex.lock();
    std::cout << "thread exit " << id << std::endl;
    g_mutex.unlock();
  }

  void createThreads(size_t numThreads)
  {
    for (size_t i=1; i<numThreads; i++)
      g_threads.push_back(createThread(threadMainLoop,(void*)i,1000000,i));
  }

  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
#if defined(__ENABLE_RAYSTREAM_LOGGER__)
    FATAL("ray stream logger still active, disable to run 'retrace'");
#endif

    g_numThreads = getNumberOfLogicalThreads(); 
#if defined (__MIC__)
    g_numThreads -= 4;
#endif

    setAffinity(0);
    /* parse command line */  
    parseCommandLine(argc,argv);

    DBG_PRINT( g_binaries_path );

    /* perform tests */
    DBG_PRINT(g_rtcore.c_str());
    rtcInit(g_rtcore.c_str());


    DBG_PRINT(g_numThreads);

    /* load geometry file */
    std::string geometryFileName = g_binaries_path + "geometry.bin";

    std::cout << "loading geometry data..." << std::flush;    
    void *g = loadGeometryData(geometryFileName);
    std::cout <<  "done" << std::endl << std::flush;

    /* looking for ray stream file */

    std::string rayStreamFileName;
    std::string rayStreamVerifyFileName;

    if (g_simd_width != 0)
      {
        rayStreamFileName = g_binaries_path + "ray" + std::stringOf(g_simd_width) + ".bin";
        rayStreamVerifyFileName = g_binaries_path + "ray" + std::stringOf(g_simd_width) + "_verify.bin";
      }
    else
      {
        /* looking for stream files in the following order: ray1.bin, ray4.bin, ray8.bin, ray16.bin */
        for (size_t shift=0;shift<=4;shift++)
          {
            g_simd_width = (size_t)1 << shift;
            DBG_PRINT(g_simd_width);
            rayStreamFileName = "ray" + std::stringOf(g_simd_width) + ".bin";
            rayStreamVerifyFileName = g_binaries_path + "ray" + std::stringOf(g_simd_width) + "_verify.bin";
            if (existsFile( rayStreamFileName )) break;

          }
      }
   
    if (g_simd_width == 0)
      FATAL("no valid ray stream data files found");

    DBG_PRINT( rayStreamFileName );
    DBG_PRINT( rayStreamVerifyFileName );

    if (!existsFile( rayStreamFileName )) FATAL("ray stream file does not exists!");
    if (!existsFile( rayStreamVerifyFileName )) FATAL("ray stream verify file does not exists!");


    /* load ray stream data */
    std::cout << "loading ray stream data..." << std::flush;    
    size_t numLogRayStreamElements       = 0;
    size_t numLogRayStreamElementsVerify = 0;

    void *raydata        = NULL;
    void *raydata_verify = NULL;

    switch(g_simd_width)
      {
      case 1:
        raydata = loadRayStreamData<RayStreamLogger::LogRay1>(rayStreamFileName, numLogRayStreamElements);
        if (g_check)
          raydata_verify = loadRayStreamData<RayStreamLogger::LogRay1>(rayStreamVerifyFileName, numLogRayStreamElementsVerify); 
        break;
      case 16:
        raydata = loadRayStreamData<RayStreamLogger::LogRay16>(rayStreamFileName, numLogRayStreamElements);
        if (g_check)
          raydata_verify = loadRayStreamData<RayStreamLogger::LogRay16>(rayStreamVerifyFileName, numLogRayStreamElementsVerify); 
        break;
      default:
        FATAL("unknown SIMD width");
      }

    std::cout <<  "done" << std::endl << std::flush;

    if (g_check)
      if (numLogRayStreamElements != numLogRayStreamElementsVerify)
        FATAL("numLogRayStreamElements != numLogRayStreamElementsVerify");

    /* analyse ray stream data */
    std::cout << "analyse ray stream:" << std::endl << std::flush;    
    RayStreamStats stats;
    switch(g_simd_width)
      {
      case 1:
        stats = analyseRayStreamData<RayStreamLogger::LogRay1>(raydata,numLogRayStreamElements);
        break;
      case 16:
        stats = analyseRayStreamData<RayStreamLogger::LogRay16>(raydata,numLogRayStreamElements);
        break;
      }

    stats.print(g_simd_width);

    /* transfer geometry data */
    std::cout << "transfering geometry data:" << std::endl << std::flush;
    RTCScene scene = transferGeometryData((char*)g);

    std::cout << "using " << g_numThreads << " threads for retracing rays" << std::endl << std::flush;
    createThreads(g_numThreads);


    /* retrace ray packets */
    DBG_PRINT( g_numThreads );

    std::cout << "Retracing logged rays:" << std::endl << std::flush;
    g_retraceTask.scene                   = scene;
    g_retraceTask.raydata                 = raydata;
    g_retraceTask.raydata_verify          = raydata_verify;
    g_retraceTask.numLogRayStreamElements = numLogRayStreamElements;
    g_retraceTask.check                   = g_check;

    
    double avg_time = 0;
    double mrays_sec = 0;
    for (size_t i=0;i<g_frames;i++)
      {
	double dt = getSeconds();
	g_rays_traced = 0;

        /* retrace rays using all threads */
	g_counter = 0;
        g_barrier.wait(0,g_numThreads);
        renderMainLoop(0);

	dt = getSeconds()-dt;
	mrays_sec += (double)g_rays_traced / dt / 1000000.;
#if 1
	std::cout << "frame " << i << " => time " << 1000. * dt << " " << 1. / dt << " fps " << "ms " << stats.numTotalRays / dt / 1000000. << " mrays/sec" << std::endl;
#endif
      }
    std::cout << "rays " << g_rays_traced << " avg. mrays/sec = " << mrays_sec / (double)g_frames << std::endl;

    std::cout << "freeing threads..." << std::flush;
    g_exitThreads = true;
    g_barrier.wait(0,g_numThreads);
    std::cout << "done" << std::endl << std::flush;


    /* done */
    rtcExit();
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
