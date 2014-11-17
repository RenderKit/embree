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
  template<typename Key, typename Val>
  class sorted_map
  {
    struct KeyValue
    {
      __forceinline KeyValue () {}

      __forceinline KeyValue (const Key key, const Val val)
	: key(key), val(val) {}

      __forceinline operator Key() const {
	return key;
      }

    public:
      Key key;
      Val val;
    };

  public:
    sorted_map () {}

    template<typename SourceKey>
    __forceinline void init(const std::vector<SourceKey>& keys, const std::vector<Val>& values) 
    {
      assert(keys.size() == values.size());
      vec.resize(keys.size());
      temp.resize(keys.size());
      for (size_t i=0; i<keys.size(); i++) {
	temp[i] = KeyValue((Key)keys[i],values[i]); // FIXME: parallel copy
      }
      radix_sort<KeyValue,Key>(&temp[0],&temp[0],&vec[0],keys.size());
    }

    template<typename SourceKey>
    __forceinline void init(const BufferT<SourceKey>& keys, const BufferT<Val>& values) 
    {
      assert(keys.size() == values.size());
      vec.resize(keys.size());
      temp.resize(keys.size());
      for (size_t i=0; i<keys.size(); i++) {
	temp[i] = KeyValue((Key)keys[i],values[i]); // FIXME: parallel copy
      }
      radix_sort<KeyValue,Key>(&temp[0],&temp[0],&vec[0],keys.size());
    }

    __forceinline const Val* lookup(const Key& key) const 
    {
      typename std::vector<KeyValue>::const_iterator i = std::lower_bound(vec.begin(), vec.end(), key);
      if (i == vec.end()) return NULL;
      if (i->key != key) return NULL;
      return &i->val;
    }

    __forceinline Val lookup(const Key& key, const Val& def) const 
    {
      typename std::vector<KeyValue>::const_iterator i = std::lower_bound(vec.begin(), vec.end(), key);
      if (i == vec.end()) return def;
      if (i->key != key) return def;
      return i->val;
    }

  private:
    std::vector<KeyValue> vec;
    std::vector<KeyValue> temp;
  };
}
