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

#include "acceln.h"
#include "../../include/embree2/rtcore_ray.h"
#include "../algorithms/parallel_for.h"

namespace embree
{
  AccelN::AccelN () 
    : Accel(AccelData::TY_ACCELN), accels(nullptr), validAccels(nullptr) {}

  AccelN::~AccelN() 
  {
    for (size_t i=0; i<accels.size(); i++)
      delete accels[i];
  }

  void AccelN::add(Accel* accel) 
  {
    assert(accel);
    if (accels.size() == accels.max_size())
      throw_RTCError(RTC_UNKNOWN_ERROR,"internal error: AccelN too small");
    
    accels.push_back(accel);
  }
  
  void AccelN::intersect (void* ptr, RTCRay& ray) 
  {
    AccelN* This = (AccelN*)ptr;
    for (size_t i=0; i<This->validAccels.size(); i++)
      This->validAccels[i]->intersect(ray);
  }

  void AccelN::intersect4 (const void* valid, void* ptr, RTCRay4& ray) 
  {
    AccelN* This = (AccelN*)ptr;
    for (size_t i=0; i<This->validAccels.size(); i++)
      This->validAccels[i]->intersect4(valid,ray);
  }

  void AccelN::intersect8 (const void* valid, void* ptr, RTCRay8& ray) 
  {
    AccelN* This = (AccelN*)ptr;
    for (size_t i=0; i<This->validAccels.size(); i++)
      This->validAccels[i]->intersect8(valid,ray);
  }

  void AccelN::intersect16 (const void* valid, void* ptr, RTCRay16& ray) 
  {
    AccelN* This = (AccelN*)ptr;
    for (size_t i=0; i<This->validAccels.size(); i++)
      This->validAccels[i]->intersect16(valid,ray);
  }

  void AccelN::occluded (void* ptr, RTCRay& ray) 
  {
    AccelN* This = (AccelN*)ptr;
    for (size_t i=0; i<This->validAccels.size(); i++) {
      This->validAccels[i]->occluded(ray); 
      if (ray.geomID == 0) break;
    }
  }

  void AccelN::occluded4 (const void* valid, void* ptr, RTCRay4& ray) 
  {
    AccelN* This = (AccelN*)ptr;
    for (size_t i=0; i<This->validAccels.size(); i++) {
      This->validAccels[i]->occluded4(valid,ray);
#if defined(__SSE2__)
      bool4 valid0 = ((bool4*)valid)[0];
      bool4 hit0   = ((int4*)ray.geomID)[0] == int4(0);
      if (all(valid0,hit0)) break;
#endif
    }
  }

  void AccelN::occluded8 (const void* valid, void* ptr, RTCRay8& ray) 
  {
    AccelN* This = (AccelN*)ptr;
    for (size_t i=0; i<This->validAccels.size(); i++) {
      This->validAccels[i]->occluded8(valid,ray);
#if defined(__SSE2__)
      bool4 valid0 = ((bool4*)valid)[0];
      bool4 hit0   = ((int4*)ray.geomID)[0] == int4(0);
      bool4 valid1 = ((bool4*)valid)[1];
      bool4 hit1   = ((int4*)ray.geomID)[1] == int4(0);
      if (all(valid0,hit0) && all(valid1,hit1)) break;
#endif
    }
  }

  void AccelN::occluded16 (const void* valid, void* ptr, RTCRay16& ray) 
  {
    AccelN* This = (AccelN*)ptr;
    for (size_t i=0; i<This->validAccels.size(); i++) {
      This->validAccels[i]->occluded16(valid,ray);
#if defined(__MIC__) || defined(__AVX512F__)
      bool16 valid0 = ((bool16*)valid)[0];
      bool16 hit0   = ((int16*)ray.geomID)[0] == int16(0);
      if (all(valid0,hit0)) break;
#endif
    }
  }

  void AccelN::print(size_t ident)
  {
    for (size_t i=0; i<validAccels.size(); i++)
    {
      for (size_t j=0; j<ident; j++) std::cout << " "; 
      std::cout << "accels[" << i << "]" << std::endl;
      validAccels[i]->intersectors.print(ident+2);
    }
  }

  void AccelN::immutable()
  {
    for (size_t i=0; i<accels.size(); i++)
      accels[i]->immutable();
  }
  
  void AccelN::build (size_t threadIndex, size_t threadCount) 
  {
    /* build all acceleration structures in parallel */
#if defined(__MIC__)
    for (size_t i=0; i<accels.size(); i++) 
        accels[i]->build(threadIndex,threadCount);
#else
    parallel_for (accels.size(), [&] (size_t i) { 
        accels[i]->build(threadIndex,threadCount);
      });
#endif

    /* create list of non-empty acceleration structures */
    validAccels.clear();
    for (size_t i=0; i<accels.size(); i++) {
      if (accels[i]->bounds.empty()) continue;
      validAccels.push_back(accels[i]);
    }

    if (validAccels.size() == 1) {
      intersectors = validAccels[0]->intersectors;
    }
    else 
    {
      intersectors.ptr = this;
      intersectors.intersector1  = Intersector1(&intersect,&occluded,"AccelN::intersector1");
      intersectors.intersector4  = Intersector4(&intersect4,&occluded4,"AccelN::intersector4");
      intersectors.intersector8  = Intersector8(&intersect8,&occluded8,"AccelN::intersector8");
      intersectors.intersector16 = Intersector16(&intersect16,&occluded16,"AccelN::intersector16");
    }
    
    /*! calculate bounds */
    bounds = empty;
    for (size_t i=0; i<validAccels.size(); i++) 
      bounds.extend(validAccels[i]->bounds);
  }

  void AccelN::select(bool filter4, bool filter8, bool filter16)
  {
    for (size_t i=0; i<accels.size(); i++) 
      accels[i]->intersectors.select(filter4,filter8,filter16);
  }

  void AccelN::clear()
  {
    for (size_t i=0; i<accels.size(); i++) 
      accels[i]->clear();
  }
}
