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

#pragma once

#include "../kernels/common/default.h"
#include "embree2/rtcore_ray.h"
#include <iostream>
#include <fstream>
#include <pthread.h>

namespace embree
{
  class RayStreamLogger
  {
  private:
    pthread_mutex_t mutex;

    bool initialized;

    std::ofstream rayData;

    void storeGeometry(void* scene);

    void openRayDataStream();

  public:
    RayStreamLogger();
    ~RayStreamLogger();

    struct __aligned(64) LogRay16  {
      unsigned int type;
      unsigned int m_valid;
      unsigned int dummy[14];
      RTCRay16 start;
      RTCRay16 end;

      LogRay16() {
	memset(this,0,sizeof(LogRay16));
      }
    };
      
  static RayStreamLogger rayStreamLogger;

  void logRay16Intersect(void* valid, void* scene, RTCRay16& start, RTCRay16& end);
  void logRay16Occluded (void* valid, void* scene, RTCRay16& start, RTCRay16& end);
    
  };
};
