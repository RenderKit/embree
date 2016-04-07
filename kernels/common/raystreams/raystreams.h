// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "../../common/default.h"
#include "../../common/ray.h"

namespace embree
{
  class Scene;

  typedef void (*filterAOS_func)(Scene *scene, 
                                 RTCRay* rayN, 
                                 const size_t M,
                                 const size_t N, 
                                 const size_t stride, 
                                 const size_t flags, 
                                 const bool intersect);

  typedef void (*filterSOA_func)(Scene *scene, 
                                 RTCRaySOA& rayN, 
                                 const size_t N, 
                                 const size_t streams, 
                                 const size_t offset, 
                                 const size_t flags, 
                                 const bool intersect);

  typedef void (*filterAOS_Single_func)(Scene *scene, RTCRay* _rayN, const size_t N, const size_t stride, const size_t flags, const bool intersect);
  typedef void (*filterSOA_Packet_func)(Scene *scene, char* rayN, const size_t N, const size_t streams, const size_t stream_offset, const size_t flags, const bool intersect);

  struct RayStreamFilterFuncs
  {
    RayStreamFilterFuncs()
    : filterAOS(nullptr), filterSOA(nullptr), filterAOSSingle(nullptr), filterSOAPacket(nullptr) {}

    RayStreamFilterFuncs(void (*ptr) ()) {
      filterAOS = (filterAOS_func) filterAOS;
      filterSOA = (filterSOA_func) filterSOA;
      filterAOSSingle = (filterAOS_Single_func) filterAOSSingle;
      filterSOAPacket = (filterSOA_Packet_func) filterSOAPacket;
    }

    RayStreamFilterFuncs(filterAOS_func aos, filterSOA_func soa, filterAOS_Single_func aos_s, filterSOA_Packet_func aos_p) { 
      filterAOS = aos;
      filterSOA = soa;
      filterAOSSingle = aos_s;
      filterSOAPacket = aos_p;
    }

  public:
    filterAOS_func filterAOS;
    filterSOA_func filterSOA;
    filterAOS_Single_func filterAOSSingle;
    filterSOA_Packet_func filterSOAPacket;
  };
  

  namespace isa
  {
    class RayStream
    {

    public:
      static void filterAOS_Single(Scene *scene, RTCRay* _rayN, const size_t N, const size_t stride, const size_t flags, const bool intersect);
      static void filterSOA_Packet(Scene *scene, char* rayN, const size_t N, const size_t streams, const size_t stream_offset, const size_t flags, const bool intersect);

      static void filterAOS(Scene *scene, RTCRay* rayN, const size_t M, const size_t N, const size_t stride, const size_t flags, const bool intersect);

      static void filterSOA(Scene *scene, RTCRaySOA& rayN, const size_t N, const size_t streams, const size_t offset, const size_t flags, const bool intersect);
    
    };
  }
};
