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
#include <vector>

namespace embree
{
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

  RTCRay makeRay(const Vec3fa& org, const Vec3fa& dir) 
  {
    RTCRay ray;
    ray.org[0] = org.x; ray.org[1] = org.y; ray.org[2] = org.z;
    ray.dir[0] = dir.x; ray.dir[1] = dir.y; ray.dir[2] = dir.z;
    ray.tnear = 0.0f; ray.tfar = inf;
    ray.time = 0; ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
    return ray;
  }

  RTCRay makeRay(const Vec3fa& org, const Vec3fa& dir, float tnear, float tfar) 
  {
    RTCRay ray;
    ray.org[0] = org.x; ray.org[1] = org.y; ray.org[2] = org.z;
    ray.dir[0] = dir.x; ray.dir[1] = dir.y; ray.dir[2] = dir.z;
    ray.tnear = tnear; ray.tfar = tfar;
    ray.time = 0; ray.mask = -1;
    ray.geomID = ray.primID = ray.instID = -1;
    return ray;
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
    ray_o.Ngx[i] = ray_i.Ng[0];
    ray_o.Ngy[i] = ray_i.Ng[1];
    ray_o.Ngz[i] = ray_i.Ng[2];
    ray_o.time[i] = ray_i.time;
    ray_o.mask[i] = ray_i.mask;
    ray_o.geomID[i] = ray_i.geomID;
    ray_o.primID[i] = ray_i.primID;
    ray_o.instID[i] = ray_i.instID;
  }


  RTCRay getRay(RTCRay16& ray_i, int i)
  {
    RTCRay ray_o;
    ray_o.org[0] = ray_i.orgx[i];
    ray_o.org[1] = ray_i.orgy[i];
    ray_o.org[2] = ray_i.orgz[i];
    ray_o.dir[0] = ray_i.dirx[i];
    ray_o.dir[1] = ray_i.diry[i];
    ray_o.dir[2] = ray_i.dirz[i];
    ray_o.tnear = ray_i.tnear[i];
    ray_o.tfar = ray_i.tfar[i];
    ray_o.Ng[0] = ray_i.Ngx[i];
    ray_o.Ng[1] = ray_i.Ngy[i];
    ray_o.Ng[2] = ray_i.Ngz[i];
    ray_o.time = ray_i.time[i];
    ray_o.mask = ray_i.mask[i];
    ray_o.geomID = ray_i.geomID[i];
    ray_o.primID = ray_i.primID[i];
    ray_o.instID = ray_i.instID[i];
    return ray_o;
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
        std::cerr << "unknown command line parameter: " << tag << " ";
        std::cerr << std::endl;
      }
    }
  }


  /* main function in embree namespace */
  int main(int argc, char** argv) 
  {
    /* parse command line */  
    parseCommandLine(argc,argv);


    /* perform tests */
    rtcInit(g_rtcore.c_str());

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
