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
#include "../common/scene.h"
#include "../common/simd/simd.h"
#include "../common/primref.h"
#include "../common/primref_mb.h"

namespace embree
{
  struct Leaf
  {
    static const unsigned SHIFT = (32-4);
    static const unsigned MASK = 0x0FFFFFFF;

    enum Type
    {
      TY_TRIANGLE = 1,
      TY_TRIANGLE_MB = 2,
      TY_QUAD = 3,
      TY_QUAD_MB = 4,
      TY_OBJECT = 5,
      TY_LINE = 6,
      TY_LINE_MB = 7,
      TY_SUBDIV = 8,
      TY_SUBDIV_MB = 9,
      TY_HAIR = 10,
      TY_HAIR_MB = 11
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
  

  struct PrimitiveType
  {
    /*! constructs the primitive type */
    PrimitiveType (const std::string& name, size_t bytes, size_t blockSize) 
    : name(name), bytes(bytes), blockSize(blockSize) {} 

    /*! Returns the number of stored primitives in a block. */
    virtual size_t size(const char* This) const = 0;

  public:
    std::string name;       //!< name of this primitive type
    size_t bytes;           //!< number of bytes of the triangle data
    size_t blockSize;       //!< block size
  };
}
