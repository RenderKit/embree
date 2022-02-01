// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/algorithms/parallel_partition.h"

using namespace embree;

namespace parallel_partition_unit_test {

TEST_CASE("Test parallel_partition", "[parallel_partition")
{
  bool passed = true;

  for (size_t i=0; i<100; i++)
  {
    /* create random permutation */
    size_t N = std::rand() % 1000000;
    std::vector<unsigned> array(N);
    for (unsigned i=0; i<N; i++) array[i] = i;
    for (auto& v : array) std::swap(v,array[std::rand()%array.size()]);
    size_t split = std::rand() % (N+1);

    /* perform parallel partitioning */
    uint64_t left_sum = 0, right_sum = 0;
    size_t mid = parallel_partitioning(array.data(),0,array.size(),0,left_sum,right_sum,
                                       [&] ( size_t i ) { return i < split; },
                                       []  ( uint64_t& sum, unsigned v) { sum += v; },
                                       []  ( uint64_t& sum, size_t v) { sum += v; },
                                       128);

    /*serial_partitioning(array.data(),0,array.size(),left_sum,right_sum,
                        [&] ( size_t i ) { return i < split; },
                        []  ( size_t& left_sum, int v) { left_sum += v; });*/

    /* verify result */
    passed &= mid == split;
    passed &= left_sum == split*(split-1)/2;
    passed &= right_sum == N*(N-1)/2-left_sum;
    for (size_t i=0; i<split; i++) passed &= array[i] < split;
    for (size_t i=split; i<N; i++) passed &= array[i] >= split;
  }

  REQUIRE(passed);
}

}
