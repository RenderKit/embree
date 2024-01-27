// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/algorithms/parallel_for_for_prefix_sum.h"

#include <atomic>

using namespace embree;

namespace parallel_for_for_prefix_sum_unit_test {

TEST_CASE("Test parallel_for_for_prefix_sum", "[parallel_for_for_prefix_sum]")
{
  bool passed = true;

  /* create vector with random numbers */
  const size_t M = 10;
#if !defined(TASKING_HPX)
  std::vector<std::atomic<size_t>> flattened;
#endif
  typedef std::vector<std::vector<size_t> *> ArrayArray;
  ArrayArray array2(M);
  size_t K = 0;
  for (size_t i = 0; i < M; i++)
  {
    const size_t N = rand() % 10;
    K += N;
    array2[i] = new std::vector<size_t>(N);
    for (size_t j = 0; j < N; j++)
      (*array2[i])[j] = rand() % 10;
  }

  /* array to test global index */
  std::vector<std::atomic<size_t>> verify_k(K);
  for (size_t i = 0; i < K; i++)
    verify_k[i].store(0);

  ParallelForForPrefixSumState<size_t> state(array2, size_t(1));

  /* dry run only counts */
  size_t S = parallel_for_for_prefix_sum0( state, array2, size_t(0),
      [&](std::vector<size_t> *v, const range<size_t> &r, size_t k, size_t i) -> size_t
  {
    size_t s = 0;
    for (size_t i=r.begin(); i<r.end(); i++) {
      s += (*v)[i];
      verify_k[k++]++;
    }
    return s;
  }, [](size_t v0, size_t v1) { return v0 + v1; });

  /* create properly sized output array */
#if !defined(TASKING_HPX)
  flattened.resize(S);
#else
  std::vector<std::atomic<size_t>> flattened(S);
#endif
  for (auto &a : flattened)
    a.store(0);

  /* now we actually fill the flattened array */
  parallel_for_for_prefix_sum1(state, array2, size_t(0),
      [&](std::vector<size_t> *v, const range<size_t> &r, size_t k, size_t i, const size_t base) -> size_t
  {
    size_t s = 0;
    for (size_t i=r.begin(); i<r.end(); i++) {
      for (size_t j=0; j<(*v)[i]; j++) {
        flattened[base+s+j]++;
      }
      s += (*v)[i];
      verify_k[k++]++;
    }
    return s;
  }, [](size_t v0, size_t v1) { return v0 + v1; });

  /* check global index */
  for (size_t i = 0; i < K; i++)
    passed &= (verify_k[i] == 2);

  /* check if each element was assigned exactly once */
  for (size_t i = 0; i < flattened.size(); i++)
    passed &= (flattened[i] == 1);

  /* delete arrays again */
  for (size_t i = 0; i < array2.size(); i++)
    delete array2[i];

  REQUIRE(passed);
}
}
