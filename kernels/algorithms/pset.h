// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "common/default.h"
#include "common/buffer.h"
#include "sort.h"

namespace embree
{
  template<typename T>
  class pset
  {
  public:
    pset () {}

    __forceinline void init(const std::vector<T>& in) 
    {
      vec.resize(in.size());
      temp.resize(in.size());
      radix_sort<T>(&in[0],&temp[0],&vec[0],in.size());
    }

    __forceinline void init(const BufferT<T>& in) 
    {
      vec.resize(in.size());
      temp.resize(in.size());
      for (size_t i=0; i<in.size(); i++) temp[i] = in[i]; // FIXME: parallel copy
      radix_sort<T>(&temp[0],&temp[0],&vec[0],in.size()); // FIXME: support BufferT in radix sort source
    }

    __forceinline bool lookup(const T& elt) const
    {
      typename std::vector<T>::const_iterator i = std::lower_bound(vec.begin(), vec.end(), elt);
      if (i == vec.end()) return false;
      return *i == elt;
    }

  private:
    std::vector<T> vec;
    std::vector<T> temp;
  };
}
