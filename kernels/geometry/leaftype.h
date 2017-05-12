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
    static const unsigned SHIFT = (32-4);
    static const unsigned MASK = 0x0FFFFFFF;

    enum Type
    {
      TY_TRIANGLE = 0,
      TY_TRIANGLE_MB = 1,
      TY_QUAD = 2,
      TY_QUAD_MB = 3,
      TY_OBJECT = 4,
      TY_LINE = 5,
      TY_LINE_MB = 6,
      TY_SUBDIV = 7,
      TY_SUBDIV_MB = 8,
      TY_HAIR = 9,
      TY_HAIR_MB = 10,
      TY_GRID = 11,
      TY_GRID_MB = 12
    };

    template<typename T>
    static __forceinline T encode(Type ty, const T& ID) {
      return (((T)ty) << SHIFT) | ID;
    }

    static __forceinline Type decodeTy(unsigned ID) {
      return (Type) (ID >> SHIFT);
    }

    static __forceinline unsigned decodeID(unsigned ID) {
      return ID & MASK;
    }
  };
}
