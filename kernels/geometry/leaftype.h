// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "../common/default.h"

namespace embree
{
  struct Leaf
  {
    static const unsigned SHIFT = (32-5);
    static const unsigned MASK = 0x07FFFFFF;

    enum Type
    {
      TY_NULL = 0,
      TY_ONE = 1,
      TY_TRIANGLE = 0,
      TY_TRIANGLE_MB = 1,
      TY_QUAD = 2,
      TY_QUAD_MB = 3,
      TY_HAIR = 4,
      TY_HAIR_MB = 5,
      TY_LINE = 6,
      TY_LINE_MB = 7,
      TY_OBJECT = 8,
      TY_OBJECT_MB = 9,
      TY_SUBDIV = 10,
      TY_SUBDIV_MB = 11,
      TY_GRID = 12,
      TY_GRID_MB = 13
    };

    static __forceinline unsigned typeMaskMBlur() 
    {
      unsigned mask = 0;
      mask |= typeMask(TY_TRIANGLE_MB);
      mask |= typeMask(TY_QUAD_MB);
      mask |= typeMask(TY_HAIR_MB);
      mask |= typeMask(TY_OBJECT_MB);
      mask |= typeMask(TY_LINE_MB);
      mask |= typeMask(TY_SUBDIV_MB);
      mask |= typeMask(TY_GRID_MB);
      return mask;
    }
    
    static __forceinline unsigned typeMask(Type ty) {
      return 1 << ty;
    }

    static __forceinline unsigned encode(Type ty, const unsigned& ID, bool last = false) {
      return (((unsigned)ty) << SHIFT) | (((unsigned)last) << 31) | ID;
    }

    template<typename T>
    static __forceinline T vencode(Type ty, const T& ID, bool last = false)
    {
      T r =  ID;
      r[0] |= unsigned(ty) << SHIFT;
      r[0] |= unsigned(last) << 31;
      return r;
    }

    static __forceinline Type loadTy(const void* leaf) {
      return decodeTy(*(unsigned*)leaf);
    }

    static __forceinline unsigned decodeLast(unsigned ID) {
      return ID & 0x80000000;
    }
    
    static __forceinline Type decodeTy(unsigned ID) {
      return (Type) ((ID >> SHIFT) & 0xF);
    }

    static __forceinline unsigned decodeID(unsigned ID) {
      return ID & MASK;
    }

    static __forceinline Type selectTy(unsigned mask) {
      assert(mask);
      return (Type) __bsf(mask);
    }
  };
}
