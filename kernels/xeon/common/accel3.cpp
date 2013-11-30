// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "accel3.h"

namespace embree
{
  Accel3::Accel3 (Accel* accel0, Accel* accel1, Accel* accel2) 
  : accel0(accel0), accel1(accel1), accel2(accel2) {}

  Accel3::~Accel3() {
    delete accel0;
    delete accel1;
    delete accel2;
  }

  void Accel3::intersect (void* ptr, RTCRay& ray) 
  {
    Accel3* This = (Accel3*)ptr;
    if (This->accel0) This->accel0->intersect(ray);
    if (This->accel1) This->accel1->intersect(ray);
    if (This->accel2) This->accel2->intersect(ray);
  }

  void Accel3::intersect4 (const void* valid, void* ptr, RTCRay4& ray) 
  {
    Accel3* This = (Accel3*)ptr;
    if (This->accel0) This->accel0->intersect4(valid,ray);
    if (This->accel1) This->accel1->intersect4(valid,ray);
    if (This->accel2) This->accel2->intersect4(valid,ray);
  }

  void Accel3::intersect8 (const void* valid, void* ptr, RTCRay8& ray) 
  {
    Accel3* This = (Accel3*)ptr;
    if (This->accel0) This->accel0->intersect8(valid,ray);
    if (This->accel1) This->accel1->intersect8(valid,ray);
    if (This->accel2) This->accel2->intersect8(valid,ray);
  }

  void Accel3::intersect16 (const void* valid, void* ptr, RTCRay16& ray) 
  {
    Accel3* This = (Accel3*)ptr;
    if (This->accel0) This->accel0->intersect16(valid,ray);
    if (This->accel1) This->accel1->intersect16(valid,ray);
    if (This->accel2) This->accel2->intersect16(valid,ray);
  }

  void Accel3::occluded (void* ptr, RTCRay& ray) 
  {
    Accel3* This = (Accel3*)ptr;
    if (This->accel0) This->accel0->occluded(ray);
    if (This->accel1) This->accel1->occluded(ray);
    if (This->accel2) This->accel2->occluded(ray);
  }

  void Accel3::occluded4 (const void* valid, void* ptr, RTCRay4& ray) 
  {
    Accel3* This = (Accel3*)ptr;
    if (This->accel0) This->accel0->occluded4(valid,ray);
    if (This->accel1) This->accel1->occluded4(valid,ray);
    if (This->accel2) This->accel2->occluded4(valid,ray);
  }

  void Accel3::occluded8 (const void* valid, void* ptr, RTCRay8& ray) 
  {
    Accel3* This = (Accel3*)ptr;
    if (This->accel0) This->accel0->occluded8(valid,ray);
    if (This->accel1) This->accel1->occluded8(valid,ray);
    if (This->accel2) This->accel2->occluded8(valid,ray);
  }

  void Accel3::occluded16 (const void* valid, void* ptr, RTCRay16& ray) 
  {
    Accel3* This = (Accel3*)ptr;
    if (This->accel0) This->accel0->occluded16(valid,ray);
    if (This->accel1) This->accel1->occluded16(valid,ray);
    if (This->accel2) This->accel2->occluded16(valid,ray);
  }

  void Accel3::print(size_t ident)
  {
    if (accel0) {
      for (size_t i=0; i<ident; i++) std::cout << " "; 
      std::cout << "accel0" << std::endl;
      accel0->intersectors.print(ident+2);
    }
    if (accel1) {
      for (size_t i=0; i<ident; i++) std::cout << " "; 
      std::cout << "accel1" << std::endl;
      accel1->intersectors.print(ident+2);
    }
    if (accel2) {
      for (size_t i=0; i<ident; i++) std::cout << " "; 
      std::cout << "accel2" << std::endl;
      accel2->intersectors.print(ident+2);
    }
  }

  void Accel3::immutable()
  {
    if (accel0) accel0->immutable();
    if (accel1) accel1->immutable();
    if (accel2) accel2->immutable();
  }

  void Accel3::build (size_t threadIndex, size_t threadCount) 
  {
    bool has_accel0 = false;
    if (accel0) {
      accel0->build(threadIndex,threadCount);
      has_accel0 = !accel0->bounds.empty();
    }

    bool has_accel1 = false;
    if (accel1) {
      accel1->build(threadIndex,threadCount);
      has_accel1 = !accel1->bounds.empty();
    }

    bool has_accel2 = false;
    if (accel2) { 
      accel2->build(threadIndex,threadCount);
      has_accel2 = !accel2->bounds.empty();
    }

    size_t num = has_accel0 + has_accel1 + has_accel2;
    
    if (num == 1) 
    {
      if (has_accel0)
        intersectors = accel0->intersectors;
      else if (has_accel1)
        intersectors = accel1->intersectors;
      else if (has_accel2)
        intersectors = accel2->intersectors;
    }
    else 
    {
      intersectors.ptr = this;
      intersectors.intersector1 = Intersector1(&intersect,&occluded,"Accel3::intersector1");
      intersectors.intersector4 = Intersector4(&intersect4,&occluded4,"Accel3::intersector4");
      intersectors.intersector8 = Intersector8(&intersect8,&occluded8,"Accel3::intersector8");
      intersectors.intersector16= Intersector16(&intersect16,&occluded16,"Accel3::intersector16");
    }
    
    /*! calculate bounds */
    bounds = empty;
    if (accel0) bounds.grow(accel0->bounds);
    if (accel1) bounds.grow(accel1->bounds);
    if (accel2) bounds.grow(accel2->bounds);
  }
}
