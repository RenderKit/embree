// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/algorithms/parallel_prefix_sum.h"

using namespace embree;

namespace parallel_prefix_sum_unit_test {

TEST_CASE("Test parallel_prefix_sum", "[parallel_prefix_sum")
{
  bool passed = true;
  const size_t M = 10;

  for (size_t N = 10; N < 10000000; N = size_t(2.1 * N))
  {
    /* initialize array with random numbers */
    uint32_t sum0 = 0;
    std::vector<uint32_t> src(N);
    for (size_t i = 0; i < N; i++)
    {
      sum0 += src[i] = rand();
    }

    /* calculate parallel prefix sum */
    std::vector<uint32_t> dst(N);
    for (auto &v : dst)
      v = 0;

    for (size_t i = 0; i < M; i++)
    {
      uint32_t sum1 = parallel_prefix_sum(src, dst, N, 0, std::plus<uint32_t>());
      passed &= (sum0 == sum1);
    }

    /* check if prefix sum is correct */
    for (size_t i = 0, sum = 0; i < N; sum += src[i++])
      passed &= ((uint32_t)sum == dst[i]);
  }

  REQUIRE(passed);
}

}
