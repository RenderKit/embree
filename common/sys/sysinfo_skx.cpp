// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "intrinsics.h"
#include "sysinfo_skx.h"

namespace embree
{
  volatile char fmaLoopDummy; // dummy to force code generation

  __noinline uint64_t fmaShuffleLoop(size_t count)
  {
    // initialize vectors with dummy values
    __m512d a0  = _mm512_set1_pd(1);
    __m512d a1  = _mm512_set1_pd(2);
    __m512d a2  = _mm512_set1_pd(3);
    __m512d a3  = _mm512_set1_pd(4);
    __m512d a4  = _mm512_set1_pd(5);
    __m512d a5  = _mm512_set1_pd(6);
    __m512d a6  = _mm512_set1_pd(7);
    __m512d a7  = _mm512_set1_pd(8);
    __m512d a8  = _mm512_set1_pd(9);
    __m512d a9  = _mm512_set1_pd(10);
    __m512d a10 = _mm512_set1_pd(11);
    __m512d a11 = _mm512_set1_pd(12);

    __m512i b0  = _mm512_set_epi32(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15);
    __m512i b1  = _mm512_set_epi32(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0);
    __m512i b2  = _mm512_set_epi32(2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,1);
    __m512i b3  = _mm512_set_epi32(3,4,5,6,7,8,9,10,11,12,13,14,15,0,1,2);
    __m512i b4  = _mm512_set_epi32(4,5,6,7,8,9,10,11,12,13,14,15,0,1,2,3);
    __m512i b5  = _mm512_set_epi32(5,6,7,8,9,10,11,12,13,14,15,0,1,2,3,4);
    __m512i b6  = _mm512_set_epi32(6,7,8,9,10,11,12,13,14,15,0,1,2,3,4,5);
    __m512i b7  = _mm512_set_epi32(7,8,9,10,11,12,13,14,15,0,1,2,3,4,5,6);
    __m512i b8  = _mm512_set_epi32(8,9,10,11,12,13,14,15,0,1,2,3,4,5,6,7);
    __m512i b9  = _mm512_set_epi32(9,10,11,12,13,14,15,0,1,2,3,4,5,6,7,8);
    __m512i b10 = _mm512_set_epi32(10,11,12,13,14,15,0,1,2,3,4,5,6,7,8,9);
    __m512i b11 = _mm512_set_epi32(11,12,13,14,15,0,1,2,3,4,5,6,7,8,9,10);

    const uint64_t start = read_tsc();

    // loop
    #pragma nounroll
    for (size_t i = 0; i < count; i++)
    {
      a0  = _mm512_fmadd_pd(a0,  a0,  a0);
      a1  = _mm512_fmadd_pd(a1,  a1,  a1);
      a2  = _mm512_fmadd_pd(a2,  a2,  a2);
      a3  = _mm512_fmadd_pd(a3,  a3,  a3);
      a4  = _mm512_fmadd_pd(a4,  a4,  a4);
      a5  = _mm512_fmadd_pd(a5,  a5,  a5);
      a6  = _mm512_fmadd_pd(a6,  a6,  a6);
      a7  = _mm512_fmadd_pd(a7,  a7,  a7);
      a8  = _mm512_fmadd_pd(a8,  a8,  a8);
      a9  = _mm512_fmadd_pd(a9,  a9,  a9);
      a10 = _mm512_fmadd_pd(a10, a10, a10);
      a11 = _mm512_fmadd_pd(a11, a11, a11);

      b0  = _mm512_permutexvar_epi32(b0,  b0);
      b1  = _mm512_permutexvar_epi32(b1,  b1);
      b2  = _mm512_permutexvar_epi32(b2,  b2);
      b3  = _mm512_permutexvar_epi32(b3,  b3);
      b4  = _mm512_permutexvar_epi32(b4,  b4);
      b5  = _mm512_permutexvar_epi32(b5,  b5);
      b6  = _mm512_permutexvar_epi32(b6,  b6);
      b7  = _mm512_permutexvar_epi32(b7,  b7);
      b8  = _mm512_permutexvar_epi32(b8,  b8);
      b9  = _mm512_permutexvar_epi32(b9,  b9);
      b10 = _mm512_permutexvar_epi32(b10, b10);
      b11 = _mm512_permutexvar_epi32(b11, b11);
    }

    const uint64_t stop = read_tsc();

    // force code generation
    __aligned(64) char temp[64*24];

    _mm512_store_pd(temp + 64*0, a0);
    _mm512_store_pd(temp + 64*1, a1);
    _mm512_store_pd(temp + 64*2, a2);
    _mm512_store_pd(temp + 64*3, a3);
    _mm512_store_pd(temp + 64*4, a4);
    _mm512_store_pd(temp + 64*5, a5);
    _mm512_store_pd(temp + 64*6, a6);
    _mm512_store_pd(temp + 64*7, a7);
    _mm512_store_pd(temp + 64*8, a8);
    _mm512_store_pd(temp + 64*9, a9);
    _mm512_store_pd(temp + 64*10, a10);
    _mm512_store_pd(temp + 64*11, a11);

    _mm512_store_epi32(temp + 64*12, b0);
    _mm512_store_epi32(temp + 64*13, b1);
    _mm512_store_epi32(temp + 64*14, b2);
    _mm512_store_epi32(temp + 64*15, b3);
    _mm512_store_epi32(temp + 64*16, b4);
    _mm512_store_epi32(temp + 64*17, b5);
    _mm512_store_epi32(temp + 64*18, b6);
    _mm512_store_epi32(temp + 64*19, b7);
    _mm512_store_epi32(temp + 64*20, b8);
    _mm512_store_epi32(temp + 64*21, b9);
    _mm512_store_epi32(temp + 64*22, b10);
    _mm512_store_epi32(temp + 64*23, b11);

    fmaLoopDummy = 0;
    for (int i = 0; i < 64*24; ++i)
      fmaLoopDummy ^= temp[i];

    return stop-start;
  }

  __noinline uint64_t fmaOnlyLoop(size_t count)
  {
    // initialize vectors with dummy values
    __m512d a0  = _mm512_set1_pd(1);
    __m512d a1  = _mm512_set1_pd(2);
    __m512d a2  = _mm512_set1_pd(3);
    __m512d a3  = _mm512_set1_pd(4);
    __m512d a4  = _mm512_set1_pd(5);
    __m512d a5  = _mm512_set1_pd(6);
    __m512d a6  = _mm512_set1_pd(7);
    __m512d a7  = _mm512_set1_pd(8);
    __m512d a8  = _mm512_set1_pd(9);
    __m512d a9  = _mm512_set1_pd(10);
    __m512d a10 = _mm512_set1_pd(11);
    __m512d a11 = _mm512_set1_pd(12);

    const uint64_t start = read_tsc();

    // loop
    #pragma nounroll
    for (size_t i = 0; i < count; i++)
    {
      a0  = _mm512_fmadd_pd(a0,  a0,  a0);
      a1  = _mm512_fmadd_pd(a1,  a1,  a1);
      a2  = _mm512_fmadd_pd(a2,  a2,  a2);
      a3  = _mm512_fmadd_pd(a3,  a3,  a3);
      a4  = _mm512_fmadd_pd(a4,  a4,  a4);
      a5  = _mm512_fmadd_pd(a5,  a5,  a5);
      a6  = _mm512_fmadd_pd(a6,  a6,  a6);
      a7  = _mm512_fmadd_pd(a7,  a7,  a7);
      a8  = _mm512_fmadd_pd(a8,  a8,  a8);
      a9  = _mm512_fmadd_pd(a9,  a9,  a9);
      a10 = _mm512_fmadd_pd(a10, a10, a10);
      a11 = _mm512_fmadd_pd(a11, a11, a11);
    }

    const uint64_t stop = read_tsc();

    // force code generation
    __aligned(64) char temp[64*12];

    _mm512_store_pd(temp + 64*0, a0);
    _mm512_store_pd(temp + 64*1, a1);
    _mm512_store_pd(temp + 64*2, a2);
    _mm512_store_pd(temp + 64*3, a3);
    _mm512_store_pd(temp + 64*4, a4);
    _mm512_store_pd(temp + 64*5, a5);
    _mm512_store_pd(temp + 64*6, a6);
    _mm512_store_pd(temp + 64*7, a7);
    _mm512_store_pd(temp + 64*8, a8);
    _mm512_store_pd(temp + 64*9, a9);
    _mm512_store_pd(temp + 64*10, a10);
    _mm512_store_pd(temp + 64*11, a11);

    fmaLoopDummy = 0;
    for (int i = 0; i < 64*12; ++i)
      fmaLoopDummy ^= temp[i];

    return stop-start;
  }

  unsigned int getNumberOfAVX512FMAUnits()
  {
    // warmup
    fmaOnlyLoop(100000);

    // execute FMA and shuffle throughput test
    uint64_t fmaShuffleCycles[3];
    for(int i = 0; i < 3; i++)
      fmaShuffleCycles[i] = fmaShuffleLoop(1000);

    // execute FMA-only throughput test
    uint64_t fmaOnlyCycles[3];
    for (int i = 0; i < 3; i++)
      fmaOnlyCycles[i] = fmaOnlyLoop(1000);

    // decide number of FMA units
    uint64_t fmaShuffleCyclesMin = fmaShuffleCycles[0];
    uint64_t fmaOnlyCyclesMin = fmaOnlyCycles[0];
    for (int i = 1; i < 3; i++) {
      if (fmaShuffleCycles[i] < fmaShuffleCyclesMin) fmaShuffleCyclesMin = fmaShuffleCycles[i];
      if (fmaOnlyCycles[i] < fmaOnlyCyclesMin) fmaOnlyCyclesMin = fmaOnlyCycles[i];
    }

    if (((double)fmaShuffleCyclesMin / (double)fmaOnlyCyclesMin) < 1.5)
      return 1;
    else
      return 2;
  }
}

/*
int main()
{
  printf("FMA units: %d\n", embree::getNumberOfAVX512FMAUnits());
  return 0;
}
*/
