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

#include "intrinsics.h"
#include "mutex.h"

namespace embree
{
  struct AtomicCounter {
  public:
    __forceinline AtomicCounter( void ) : data(0) {}
    __forceinline AtomicCounter( const atomic_t data ) : data(data) {}
    __forceinline AtomicCounter& operator =( const atomic_t input    ) { *(volatile atomic_t*)&data = input; return *this; }
    __forceinline void reset               ( const atomic_t input = 0) { *(volatile atomic_t*)&data = input; }
    __forceinline operator atomic_t() const { return data; }

  public:
    __forceinline atomic_t sub( const atomic_t input ) { return atomic_add(&data,-input); }
    __forceinline atomic_t add( const atomic_t input ) { return atomic_add(&data, input); }

    __forceinline friend atomic_t operator +=( AtomicCounter& value, const atomic_t input ) { return atomic_add(&value.data, +input) + input; }
    __forceinline friend atomic_t operator -=( AtomicCounter& value, const atomic_t input ) { return atomic_add(&value.data, -input) - input; }
    __forceinline friend atomic_t operator ++( AtomicCounter& value ) { return atomic_add(&value.data,  1) + 1; }
    __forceinline friend atomic_t operator --( AtomicCounter& value ) { return atomic_add(&value.data, -1) - 1; }
    __forceinline friend atomic_t operator ++( AtomicCounter& value, int ) { return atomic_add(&value.data,  1); }
    __forceinline friend atomic_t operator --( AtomicCounter& value, int ) { return atomic_add(&value.data, -1); }
    __forceinline atomic_t inc() { return atomic_add(&data,+1); }
    __forceinline atomic_t dec() { return atomic_add(&data,-1); }

  private:
    volatile atomic_t data;
  };

  class __aligned(64) AlignedAtomicCounter32
  {
  public:
    __forceinline AlignedAtomicCounter32 () : data(0) {}
    __forceinline AlignedAtomicCounter32 (const atomic32_t v) {  *(volatile atomic32_t*)&data = v; }
    __forceinline AlignedAtomicCounter32& operator =( const atomic32_t input    ) { *(volatile atomic32_t*)&data = input; return *this; }
    __forceinline void reset                        ( const atomic32_t input = 0) { *(volatile atomic32_t*)&data = input; }
    __forceinline operator atomic32_t() const { return data; }

  public:
    __forceinline unsigned int sub(const atomic32_t i) { return atomic_add(&data, -i); }
    __forceinline unsigned int add(const atomic32_t i) { return atomic_add(&data, +i); }
    __forceinline friend atomic32_t operator +=( AlignedAtomicCounter32& value, const atomic32_t input ) { return atomic_add(&value.data, +input) + input; }
    __forceinline friend atomic32_t operator -=( AlignedAtomicCounter32& value, const atomic32_t input ) { return atomic_add(&value.data, -input) - input; }
    __forceinline friend atomic32_t operator ++( AlignedAtomicCounter32& value ) { return atomic_add(&value.data,  1) + 1; }
    __forceinline friend atomic32_t operator --( AlignedAtomicCounter32& value ) { return atomic_add(&value.data, -1) - 1; }
    __forceinline friend atomic32_t operator ++( AlignedAtomicCounter32& value, int ) { return atomic_add(&value.data,  1); }
    __forceinline friend atomic32_t operator --( AlignedAtomicCounter32& value, int ) { return atomic_add(&value.data, -1); }
    __forceinline atomic32_t inc() { return atomic_add(&data,+1); }
    __forceinline atomic32_t dec() { return atomic_add(&data,-1); }

    __forceinline void min(const unsigned int i) { return atomic_min_ui32((volatile unsigned int*)&data, i); }

    __forceinline void max(const unsigned int i) { return atomic_max_ui32((volatile unsigned int*)&data, i); }


  private:
    volatile atomic32_t data;
    char align[64-sizeof(atomic32_t)]; // one counter per cache line
  };

  class __aligned(64) AlignedAtomicCounter64
  {
#if !defined(__X86_64__)

    typedef int64_t atomic64_t;

    __forceinline atomic64_t atomic_add(volatile atomic64_t* ptr, atomic_t x) {
      mutex.lock();
      atomic64_t old = data;
      data += x;
      mutex.unlock();
      return old;
    }
#endif

  public:
    __forceinline AlignedAtomicCounter64 () : data(0) {}
    __forceinline AlignedAtomicCounter64 (const atomic64_t v) {  *(volatile atomic64_t*)&data = v; }
    __forceinline AlignedAtomicCounter64& operator =( const atomic64_t input    ) { *(volatile atomic64_t*)&data = input; return *this; }
    __forceinline void reset                        ( const atomic64_t input = 0) { *(volatile atomic64_t*)&data = input; }
    __forceinline operator atomic64_t() const { return data; }

  public:
    __forceinline size_t sub(const atomic64_t i) { return atomic_add(&data, -i); }
    __forceinline size_t add(const atomic64_t i) { return atomic_add(&data, +i); }
    __forceinline friend atomic64_t operator +=( AlignedAtomicCounter64& value, const atomic64_t input ) { return value.add(+input) + input; }
    __forceinline friend atomic64_t operator -=( AlignedAtomicCounter64& value, const atomic64_t input ) { return value.add(-input) - input; }
    __forceinline friend atomic64_t operator ++( AlignedAtomicCounter64& value ) { return value.add(1) + 1; }
    __forceinline friend atomic64_t operator --( AlignedAtomicCounter64& value ) { return value.add(-1) - 1; }
    __forceinline friend atomic64_t operator ++( AlignedAtomicCounter64& value, int ) { return value.add(1); }
    __forceinline friend atomic64_t operator --( AlignedAtomicCounter64& value, int ) { return value.add(-1); }
    __forceinline atomic64_t inc() { return atomic_add(&data,+1); }
    __forceinline atomic64_t dec() { return atomic_add(&data,-1); }

  private:
    volatile atomic64_t data;
    AtomicMutex mutex;
    char align[64-sizeof(atomic64_t)-sizeof(AtomicMutex)]; // one counter per cache line
  };
}
