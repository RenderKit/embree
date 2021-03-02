// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/algorithms/parallel_set.h"

using namespace embree;

namespace parallel_set_unit_test {

TEST_CASE("Test parallel_set", "[parallel_set]")
{
  bool passed = true;

  /* create vector with random numbers */
  const size_t N = 10000;
  std::vector<uint32_t> unsorted(N);
  for (size_t i=0; i<N; i++) unsorted[i] = 2*rand();

  /* created set from numbers */
  parallel_set<uint32_t> sorted;
  sorted.init(unsorted);

  /* check that all elements are in the set */
  for (size_t i=0; i<N; i++) {
    passed &= sorted.lookup(unsorted[i]);
  }

  /* check that these elements are not in the set */
  for (size_t i=0; i<N; i++) {
    passed &= !sorted.lookup(unsorted[i]+1);
  }

  REQUIRE(passed);
}

}
