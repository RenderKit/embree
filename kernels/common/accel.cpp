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

#include "accel.h"

namespace embree
{
  /* This is a workaround for some Clang 3.8.0-2ubuntu4 optimization
   * that creates issues with the ISA selection. This code essentially
   * is compiled with non-SSE2 ISA, but has to run on SSE2
   * machines. Clang 3.8.0-2ubuntu4 issues AVX-128 instuctions to
   * update intersect and occluded pointers at once. Only providing
   * these functions for SSE2 ISA fixes this issue. */

  Accel::Intersector1::Intersector1 (ErrorFunc error) 
    : intersect((IntersectFunc)error), occluded((OccludedFunc)error), name(nullptr) {}
  
  Accel::Intersector1::Intersector1 (IntersectFunc intersect, OccludedFunc occluded, const char* name)
    : intersect(intersect), occluded(occluded), name(name) {}
  
  Accel::Intersector4::Intersector4 (ErrorFunc error) 
    : intersect((IntersectFunc4)error), occluded((OccludedFunc4)error), name(nullptr) {}
  
  Accel::Intersector4::Intersector4 (IntersectFunc4 intersect, OccludedFunc4 occluded, const char* name)
    : intersect(intersect), occluded(occluded), name(name) {}
  
  Accel::Intersector8::Intersector8 (ErrorFunc error) 
    : intersect((IntersectFunc8)error), occluded((OccludedFunc8)error), name(nullptr) {}
  
  Accel::Intersector8::Intersector8 (IntersectFunc8 intersect, OccludedFunc8 occluded, const char* name)
    : intersect(intersect), occluded(occluded), name(name) {}
  
  Accel::Intersector16::Intersector16 (ErrorFunc error) 
    : intersect((IntersectFunc16)error), occluded((OccludedFunc16)error), name(nullptr) {}
  
  Accel::Intersector16::Intersector16 (IntersectFunc16 intersect, OccludedFunc16 occluded, const char* name)
    : intersect(intersect), occluded(occluded), name(name) {}
  
  Accel::IntersectorN::IntersectorN (ErrorFunc error) 
    : intersect((IntersectFuncN)error), occluded((OccludedFuncN)error), name(nullptr) {}
  
  Accel::IntersectorN::IntersectorN (IntersectFuncN intersect, OccludedFuncN occluded, const char* name)
    : intersect(intersect), occluded(occluded), name(name) {}
}
