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
#include "sys/thread.h"
#include "sys/sysinfo.h"
#include "sys/sync/barrier.h"
#include "sys/sync/mutex.h"
#include "sys/sync/condition.h"
#include "math/vec3.h"
#include "math/bbox.h"
#include "embree2/rtcore.h"
#include "embree2/rtcore_ray.h"
#include "../kernels/common/default.h"
#include "../kernels/common/raystream_log.h"
#include <vector>
#include <iostream>
#include <fstream>

namespace embree
{
  using namespace std;

  struct RayStreamStats
  {
    size_t numTotalRays;
    size_t numRayPackets;
    size_t numIntersectRayPackets;
    size_t numOccludedRayPackets;
    size_t numIntersectRays;
    size_t numOccludedRays;

    RayStreamStats() {
      memset(this,0,sizeof(RayStreamStats));
    }

    void add(RayStreamLogger::LogRay16 &r)
    {
      size_t numRays = countbits(r.m_valid);
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

    }

  };

  __forceinline std::ostream &operator<<(std::ostream &o, const RayStreamStats &s)
    {
      o << "numTotalRays                      = " << s.numTotalRays << endl;
      o << "numRayPackets                     = " << s.numRayPackets << endl;
      o << "numIntersectionRays               = " << s.numIntersectRays << " [" << 100. * (double)s.numIntersectRays / s.numTotalRays << "%]" << endl;
      o << "numOcclusionRays                  = " << s.numOccludedRays << " [" << 100. * (double)s.numOccludedRays / s.numTotalRays << "%]" << endl;
      o << "avg. intersect packet utilization = " << 100. *  (double)s.numIntersectRays / (s.numIntersectRayPackets * 16.) << "%" << endl;
      o << "avg. occluded  packet utilization = " << 100. *  (double)s.numOccludedRays  / (s.numOccludedRayPackets  * 16.) << "%" << endl;
      o << "avg. total packet utilization     = " << 100. * (double)s.numTotalRays / (s.numRayPackets * 16.)  << "%" << endl;
      return o;
    } 
  

#if !defined(__MIC__)
  RTCAlgorithmFlags aflags = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT4 | RTC_INTERSECT8);
#else
  RTCAlgorithmFlags aflags = (RTCAlgorithmFlags) (RTC_INTERSECT1 | RTC_INTERSECT16);
#endif
  /* configuration */
  static std::string g_rtcore = "";

  /* vertex and triangle layout */
  struct Vertex   { float x,y,z,a; };
  struct Triangle { int v0, v1, v2; };

  std::vector<thread_t> g_threads;
  size_t numFailedTests = 0;

#if defined(__MIC__)
  std::string g_binaries_path = "/home/micuser/";
#else
  std::string g_binaries_path = "./";
#endif


#define AssertNoError() \
  if (rtcGetError() != RTC_NO_ERROR) return false;
#define AssertAnyError() \
  if (rtcGetError() == RTC_NO_ERROR) return false;
#define AssertError(code) \
  if (rtcGetError() != code) return false;

  const size_t numSceneFlags = 64;

  RTCSceneFlags getSceneFlag(size_t i) 
  {
    int flag = 0;                               
    if (i & 1) flag |= RTC_SCENE_DYNAMIC;
    if (i & 2) flag |= RTC_SCENE_COMPACT;
    if (i & 4) flag |= RTC_SCENE_COHERENT;
    if (i & 8) flag |= RTC_SCENE_INCOHERENT;
    if (i & 16) flag |= RTC_SCENE_HIGH_QUALITY;
    if (i & 32) flag |= RTC_SCENE_ROBUST;
    return (RTCSceneFlags) flag;
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
      
      /* skip unknown command line parameter */
      else {

	g_binaries_path = tag;
        // std::cerr << "unknown command line parameter: " << tag << " ";
        // std::cerr << std::endl;
      }
    }
  }


  void *loadGeometryData(std::string &geometryFile)
  {
    std::ifstream geometryData;

    geometryData.open(geometryFile.c_str(),ios::in | ios::binary);
    if (!geometryData) { FATAL("could not open geometry data file"); }
      geometryData.seekg(0, ios::beg);
    streampos begin = geometryData.tellg();
    geometryData.seekg(0, ios::end);
    streampos end = geometryData.tellg();
    size_t fileSize = end - begin;
    char *ptr = (char*)os_malloc(fileSize);
    geometryData.seekg(0, ios::beg);
    geometryData.read(ptr,fileSize);
    return ptr;
  }

  void *loadRayStreamData(std::string &rayStreamFile, size_t &numLogRayStreamElements)
  {
    std::ifstream rayStreamData;

    rayStreamData.open(rayStreamFile.c_str(),ios::in | ios::binary);
    if (!rayStreamData) { FATAL("could not open raystream data file"); }
    rayStreamData.seekg(0, ios::beg);
    streampos begin = rayStreamData.tellg();
    rayStreamData.seekg(0, ios::end);
    streampos end = rayStreamData.tellg();
    size_t fileSize = end - begin;
    char *ptr = (char*)os_malloc(fileSize);
    rayStreamData.seekg(0, ios::beg);
    rayStreamData.read(ptr,fileSize);
    numLogRayStreamElements = fileSize / sizeof(RayStreamLogger::LogRay16);
    return ptr;
  }

  void analyseRayStreamData(RayStreamLogger::LogRay16 *r, size_t numLogRayStreamElements)
  {
    cout << "numLogRayStreamElements " << numLogRayStreamElements << endl;
    RayStreamStats stats;
    for (size_t i=0;i<numLogRayStreamElements;i++)
      stats.add(r[i]);
    cout << stats << endl;
  }

  void transferGeometryData(char *g)
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
	DBG_PRINT(numVertices);
	size_t numTriangles = *(size_t*)g;
	g += sizeof(size_t);
	DBG_PRINT(numTriangles);
	Vertex *vtx = (Vertex*)g;
	g += sizeof(Vertex)*numVertices;
	Triangle *tri = (Triangle *)g;
	g += sizeof(Triangle)*numTriangles;
	if (((size_t)g % 16) != 0)
	  g += 16 - ((size_t)g % 16);
      }
    rtcCommit(scene);
  }


  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    /* parse command line */  
    parseCommandLine(argc,argv);

    DBG_PRINT( g_binaries_path );

    /* perform tests */
    rtcInit(g_rtcore.c_str());

    RayStreamLogger::rayStreamLogger.deactivate();

    std::string geometryFileName = g_binaries_path + "geometry.bin";
    std::string rayStreamFileName = g_binaries_path + "ray16.bin";

    /* load geometry file */
    cout << "loading geometry data..." << flush;    
    void *g = loadGeometryData(geometryFileName);
    cout <<  "done" << endl << flush;
    
    /* load ray stream data */
    cout << "loading ray stream data..." << flush;    
    size_t numLogRayStreamElements = 0;
    RayStreamLogger::LogRay16 *r = (RayStreamLogger::LogRay16 *)loadRayStreamData(rayStreamFileName, numLogRayStreamElements);
    cout <<  "done" << endl << flush;

    /* analyse ray stream data */
    cout << "analyse ray stream:" << endl << flush;    
    analyseRayStreamData(r,numLogRayStreamElements);

    /* transfer geometry data */
    cout << "transfering geometry data:" << endl << flush;
    transferGeometryData((char*)g);

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
