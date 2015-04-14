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

#pragma once

namespace embree
{
  /*! An item on the stack holds the node ID and distance of that node. */
  template<typename T>
    struct __aligned(16) StackItemInt32
  {
    __forceinline static void xchg(StackItemInt32& a, StackItemInt32& b) 
    { 
      const ssef sse_a = load4f(&a); 
      const ssef sse_b = load4f(&b);
      store4f(&a,sse_b);
      store4f(&b,sse_a);
    }

    /*! Sort 2 stack items. */
    __forceinline friend void sort(StackItemInt32& s1, StackItemInt32& s2) {
      if (s2.dist < s1.dist) xchg(s2,s1);
    }
    
    /*! Sort 3 stack items. */
    __forceinline friend void sort(StackItemInt32& s1, StackItemInt32& s2, StackItemInt32& s3)
    {
      if (s2.dist < s1.dist) xchg(s2,s1);
      if (s3.dist < s2.dist) xchg(s3,s2);
      if (s2.dist < s1.dist) xchg(s2,s1);
    }
    
    /*! Sort 4 stack items. */
    __forceinline friend void sort(StackItemInt32& s1, StackItemInt32& s2, StackItemInt32& s3, StackItemInt32& s4)
    {
      if (s2.dist < s1.dist) xchg(s2,s1);
      if (s4.dist < s3.dist) xchg(s4,s3);
      if (s3.dist < s1.dist) xchg(s3,s1);
      if (s4.dist < s2.dist) xchg(s4,s2);
      if (s3.dist < s2.dist) xchg(s3,s2);
    }
    
  public:
    T ptr; 
    unsigned int dist;
  };
}
