// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/algorithms/parallel_reduce.h"


using namespace embree;

namespace parallel_reduce_unit_test {

TEST_CASE ("Test parallel_reduce", "[parallel_reduce]")
{
  bool passed = false;

  const size_t M = 10;
  for (size_t N=10; N<10000000; N=size_t(2.1*N))
  {
    /* sequentially calculate sum of squares */
    size_t sum0 = 0;
    for (size_t i=0; i<N; i++) {
      sum0 += i*i;
    }

    /* parallel calculation of sum of squares */
    for (size_t m=0; m<M; m++)
    {
      size_t sum1 = parallel_reduce(size_t(0), size_t(N), size_t(1024), size_t(0), [&](const range<size_t>& r) -> size_t
      {
        size_t s = 0;
        for (size_t i=r.begin(); i<r.end(); i++)
          s += i*i;
        return s;
      },
      [](const size_t v0, const size_t v1) {
        return v0+v1;
      });
      passed = sum0 == sum1;
    }
  }

  REQUIRE(passed);
}

}
