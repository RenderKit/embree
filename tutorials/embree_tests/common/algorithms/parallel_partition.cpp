// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/algorithms/parallel_partition.h"

using namespace embree;

namespace parallel_partition_unit_test {

TEST_CASE("Test parallel_partition", "[parallel_partition")
{
  bool passed = true;

  for (uint64_t i=0; i<100; i++)
  {
    /* create random permutation */
    uint64_t N = std::rand() % 1000000;
    std::vector<unsigned> array(N);
    for (unsigned j=0; j<N; j++) array[j] = j;
    for (auto& v : array) std::swap(v,array[std::rand()%array.size()]);
    uint64_t split = std::rand() % (N+1);

    /* perform parallel partitioning */
    uint64_t left_sum = 0, right_sum = 0;
    uint64_t mid = parallel_partitioning(array.data(),0,array.size(),0,left_sum,right_sum,
                                       [&] ( uint64_t i ) { return i < split; },
                                       []  ( uint64_t& sum, unsigned v) { sum += v; },
                                       []  ( uint64_t& sum, uint64_t v) { sum += v; },
                                       128);

    /*serial_partitioning(array.data(),0,array.size(),left_sum,right_sum,
                        [&] ( uint64_t i ) { return i < split; },
                        []  ( uint64_t& left_sum, int v) { left_sum += v; });*/

    /* verify result */
    passed &= mid == split;
    passed &= left_sum == split*(split-1)/2;
    passed &= right_sum == N*(N-1)/2-left_sum;
    for (uint64_t j=0; j<split; j++) passed &= array[j] < split;
    for (uint64_t j=split; j<N; j++) passed &= array[j] >= split;
  }

  REQUIRE(passed);
}

}
