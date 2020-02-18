// ======================================================================== //
// Copyright 2009-2020 Intel Corporation                                    //
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

#include "bvh_node_ref.h"

namespace embree
{
  
  /*! BVHN Base Node */
  template<int N>
    struct BaseNode_t
  {
    /*! Clears the node. */
    __forceinline void clear()
    {
      for (size_t i=0; i<N; i++)
        children[i] = NodeRefPtr<N>::emptyNode;
    }
    
    /*! Returns reference to specified child */
    __forceinline       NodeRefPtr<N>& child(size_t i)       { assert(i<N); return children[i]; }
    __forceinline const NodeRefPtr<N>& child(size_t i) const { assert(i<N); return children[i]; }
    
    /*! verifies the node */
    __forceinline bool verify() const
    {
      for (size_t i=0; i<N; i++) {
        if (child(i) == NodeRefPtr<N>::emptyNode) {
          for (; i<N; i++) {
            if (child(i) != NodeRefPtr<N>::emptyNode)
              return false;
          }
          break;
        }
      }
      return true;
    }
    
    NodeRefPtr<N> children[N];    //!< Pointer to the N children (can be a node or leaf)
  };
}
