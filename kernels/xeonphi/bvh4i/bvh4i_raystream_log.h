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

#include "bvh4i.h"
#include "bvh4i_traversal.h"
#include "common/ray16.h" 
#include <iostream>
#include <fstream>

namespace embree
{
  class RayStreamLogger
  {
  private:
    bool initialized;
    bool storedBVH4i;
    std::ofstream rayData;
    std::ofstream bvh4iData;

    void storeBVH4i(BVH4i* bvh);

  public:
  RayStreamLogger() : initialized(false), storedBVH4i(false)
      {
      }

  ~RayStreamLogger()
    {
      if (initialized)
	{
	  rayData.close();
	}
    }

  static RayStreamLogger rayStreamLogger;

  void logRayIntersect(mic_i* valid, BVH4i* bvh, Ray16& ray);
  void logRayOccluded (mic_i* valid, BVH4i* bvh, Ray16& ray);
    
  };
};
