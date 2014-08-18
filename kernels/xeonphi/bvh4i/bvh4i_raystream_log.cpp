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

#include "bvh4i_raystream_log.h"
#include "sys/filename.h"


namespace embree
{

  void RayStreamLogger::logRayIntersect(mic_i* valid_i, BVH4i* bvh, Ray16& ray)
  {
    if (!initialized)
      {
	initialized = true;
	FileName filename("raylog.txt");
	rayData.open(filename.c_str());
      }

    const mic_m m_valid     = *(mic_i*)valid_i != mic_i(0);

    rayData << "INTERSECT" << std::endl;
    rayData << m_valid << std::endl;
    rayData << ray << std::endl;
  }

  void RayStreamLogger::logRayOccluded(mic_i* valid_i, BVH4i* bvh, Ray16& ray)
  {
    if (!initialized)
      {
	initialized = true;
	FileName filename("raylog.txt");
	rayData.open(filename.c_str());
      }

    const mic_m m_valid     = *(mic_i*)valid_i != mic_i(0);

    rayData << "OCCLUDED" << std::endl;
    rayData << m_valid << std::endl;
    rayData << ray << std::endl;
  }

  RayStreamLogger RayStreamLogger::rayStreamLogger;

};
