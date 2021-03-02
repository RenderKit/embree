// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/algorithms/parallel_sort.h"

using namespace embree;

namespace parallel_sort_unit_test {

template<typename Key>
bool run_sort_test()
{
  bool passed = true;
  const size_t M = 10;

  for (size_t N = 10; N < 1000000; N = size_t(2.1 * N))
  {
    std::vector<Key> src(N);
    memset(src.data(), 0, N * sizeof(Key));
    std::vector<Key> tmp(N);
    memset(tmp.data(), 0, N * sizeof(Key));
    for (size_t i = 0; i < N; i++)
      src[i] = uint64_t(rand()) * uint64_t(rand());

    /* calculate checksum */
    Key sum0 = 0;
    for (size_t i = 0; i < N; i++)
      sum0 += src[i];

    /* sort numbers */
    for (size_t i = 0; i < M; i++)
    {
      radix_sort<Key>(src.data(), tmp.data(), N);
    }

    /* calculate checksum */
    Key sum1 = 0;
    for (size_t i = 0; i < N; i++)
      sum1 += src[i];
    if (sum0 != sum1)
      passed = false;

    /* check if numbers are sorted */
    for (size_t i = 1; i < N; i++)
      passed &= src[i - 1] <= src[i];
  }

  return passed;
}

TEST_CASE("Test parallel_sort (uint32_t)", "[parallel_sort_uint32_t]")
{
  REQUIRE(run_sort_test<uint64_t>());
}

TEST_CASE("Test parallel_sort (uint64_t)", "[parallel_sort_uint64_t]")
{
  REQUIRE(run_sort_test<uint64_t>());
}

}
