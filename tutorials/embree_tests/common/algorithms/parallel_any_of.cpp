// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../common/tasking/taskscheduler.h"
#include "../common/algorithms/parallel_any_of.h"

#include <vector>
#include <numeric>
#include <thread>

using namespace embree;

namespace parallel_any_of_unit_tests {

TEST_CASE ("Test parallel_any_of", "[parallel_any_of]")
{
  const size_t num_threads = std::thread::hardware_concurrency();
  TaskScheduler::create(num_threads, true, false);

  std::vector<int> data(1024);
  std::iota(data.begin(), data.end(), 0);

  std::shuffle(data.begin(), data.end(), std::mt19937{7777});

  auto unaryPredicateTrue  = [&](size_t i) -> bool { return data[i] == 512; };
  auto unaryPredicateFalse = [&](size_t i) -> bool { return data[i] == 1048; };

  bool resultUnaryPredicateTrue  = parallel_any_of(size_t(0), data.size(), unaryPredicateTrue);
  bool resultUnaryPredicateFalse = parallel_any_of(size_t(0), data.size(), unaryPredicateFalse);

  REQUIRE(resultUnaryPredicateTrue);
  REQUIRE(!resultUnaryPredicateFalse);
}

}
