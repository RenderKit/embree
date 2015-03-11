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

#include "default.h"

namespace embree
{
  void memoryMonitor(ssize_t bytes, bool post);

  template<class T> // FIXME: use os_malloc in vector for large allocations
    class vector : public RefCount
    {
    public:

      vector () : m_size(0), alloced(0), t(NULL) {}

      void clear() {
        if (t) {
          alignedFree(t);
          memoryMonitor(-alloced*sizeof(T),true);
        }
        m_size = alloced = 0;
        t = NULL;
      };

      vector(size_t sz) {
        m_size = 0; alloced = 0; t = NULL;
        if (sz) resize(sz);
      }

      vector(const vector<T> &other)
      {
        m_size = other.m_size;
        alloced = other.alloced;
        memoryMonitor(alloced*sizeof(T),false);
        t = (T*)alignedMalloc(alloced*sizeof(T),64);
        for (size_t i=0; i<m_size; i++) t[i] = other.t[i];
      }
      
      ~vector() {
        clear();
      }

      inline bool empty() const { return m_size == 0; }
      inline size_t size() const { return m_size; };

      T* begin() const { return t; };
      T* end() const { return t+m_size; };

	  __forceinline       T* data()       { return t; };
	  __forceinline const T* data() const { return t; };


      inline T& front() const { return t[0]; };
      inline T& back () const { return t[m_size-1]; };

      void push_back(const T &nt) {
        T v = nt; // need local copy as input reference could point to this vector
        reserve(m_size+1);
        t[m_size] = v;
        m_size++;
      }

	  void pop_back() {
        m_size--;
      }

      vector<T> &operator=(const vector<T> &other) {
        resize(other.m_size);
        for (size_t i=0;i<m_size;i++) t[i] = other.t[i];
        return *this;
      }

      __forceinline T& operator[](size_t i) {
        assert(t);
        assert(i < m_size);
        return t[i];
      };

      __forceinline const T& operator[](size_t i) const {
        assert(t);
        assert(i < m_size);
        return t[i];
      };

      void resize(size_t new_sz, bool exact = false)
      {
        if (new_sz < m_size) {
          if (exact) {
            T *old_t = t;
            memoryMonitor(new_sz*sizeof(T),false);
            t = (T*)alignedMalloc(new_sz*sizeof(T),64);
            for (size_t i=0;i<new_sz;i++) t[i] = old_t[i];
            if (old_t) {
              alignedFree(old_t);
              memoryMonitor(-alloced*sizeof(T),true);
            }
            alloced = new_sz;
          }
        } else {
          reserve(new_sz,exact);
        }
        m_size = new_sz;
      };

      void reserve(size_t sz, bool exact = false)
      {
        if (sz <= alloced) return;

        size_t newAlloced = alloced;
        if (exact) newAlloced = sz;
        else
          while (newAlloced < sz)
            newAlloced = (1 < (newAlloced * 2)) ? newAlloced * 2 : 1;

        T* old_t = t;
        assert(newAlloced > 0);
        memoryMonitor(newAlloced*sizeof(T),false);
        t = (T*)alignedMalloc(newAlloced*sizeof(T),64);

        for (size_t i=0;i<m_size;i++) t[i] = old_t[i];

        if (old_t) {
          alignedFree(old_t);
          memoryMonitor(-alloced*sizeof(T),true);
        }
        alloced = newAlloced;
      }

    public:
      size_t m_size;    // number of valid items
      size_t alloced;   // number of items allocated
      T *t;             // data array
    };
}
