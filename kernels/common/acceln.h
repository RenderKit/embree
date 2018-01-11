// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "accel.h"

namespace embree
{
  /*! merges N acceleration structures together, by processing them in order */
  class AccelN : public Accel
  {
  public:
    AccelN ();
    ~AccelN();

  public:
    void add(Accel* accel);

  public:
    static void intersect (Accel::Intersectors* This, RTCRay& ray, IntersectContext* context);
    static void intersect4 (const void* valid, Accel::Intersectors* This, RTCRay4& ray, IntersectContext* context);
    static void intersect8 (const void* valid, Accel::Intersectors* This, RTCRay8& ray, IntersectContext* context);
    static void intersect16 (const void* valid, Accel::Intersectors* This, RTCRay16& ray, IntersectContext* context);
    static void intersectN (Accel::Intersectors* This, RayK<VSIZEX>** ray, const size_t N, IntersectContext* context);

  public:
    static void occluded (Accel::Intersectors* This, RTCRay& ray, IntersectContext* context);
    static void occluded4 (const void* valid, Accel::Intersectors* This, RTCRay4& ray, IntersectContext* context);
    static void occluded8 (const void* valid, Accel::Intersectors* This, RTCRay8& ray, IntersectContext* context);
    static void occluded16 (const void* valid, Accel::Intersectors* This, RTCRay16& ray, IntersectContext* context);
    static void occludedN (Accel::Intersectors* This, RayK<VSIZEX>** ray, const size_t N, IntersectContext* context);

  public:
    void print(size_t ident);
    void immutable();
    void build ();
    void select(bool filter4, bool filter8, bool filter16, bool filterN);
    void deleteGeometry(size_t geomID);
    void clear ();

  public:
    darray_t<Accel*,16> accels;
    darray_t<Accel*,16> validAccels;
  };
}
