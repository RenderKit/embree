// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/algorithms/parallel_for_for.h"

#include <atomic>

using namespace embree;

namespace parallel_for_for_unit_test {

TEST_CASE("Test parallel_for_for", "[parallel_for_for]")
{
  bool passed = true;

  /* create vector with random numbers */
  size_t sum0 = 0;
  size_t K = 0;
  const size_t M = 1000;
  std::vector<std::vector<size_t> *> array2(M);
  for (size_t i = 0; i < M; i++)
  {
    const size_t N = rand() % 1024;
    K += N;
    array2[i] = new std::vector<size_t>(N);
    for (size_t j = 0; j < N; j++)
      sum0 += (*array2[i])[j] = rand();
  }

  /* array to test global index */
  std::vector<std::atomic<size_t>> verify_k(K);
  for (size_t i = 0; i < K; i++)
    verify_k[i].store(0);

  /* add all numbers using parallel_for_for */
  std::atomic<size_t> sum1(0);
  parallel_for_for(array2, size_t(1),
      [&](std::vector<size_t> *v, const range<size_t> &r, size_t k) -> size_t
  {
    size_t s = 0;
    for (size_t i = r.begin(); i < r.end(); i++)
    {
      s += (*v)[i];
      verify_k[k++]++;
    }
    sum1 += s;
    return sum1;
  });
  passed &= (sum0 == sum1);

  /* check global index */
  for (size_t i = 0; i < K; i++)
    passed &= (verify_k[i] == 1);

  /* delete vectors again */
  for (size_t i = 0; i < array2.size(); i++)
    delete array2[i];

  REQUIRE(passed);
}

}
