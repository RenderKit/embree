// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "parallel_partition.h"

namespace embree
{
  struct parallel_partition_regression_test : public RegressionTest
  {
    parallel_partition_regression_test(const char* name) : RegressionTest(name) {
      registerRegressionTest(this);
    }
    
    bool run ()
    {
      bool passed = true;

      for (size_t i=0; i<100; i++)
      {
        /* create random permutation */
        size_t N = std::rand() % 1000000;
        std::vector<int> array(N);
        std::iota(array.begin(),array.end(),0);
        for (auto& v : array) std::swap(v,array[std::rand()%array.size()]);
        size_t split = std::rand() % (N+1);

        /* perform parallel partitioning */
        size_t left_sum = 0, right_sum = 0;
        parallel_in_place_partitioning_static<1024>(array.data(),array.size(),0,left_sum,right_sum,
                                                    [&] ( size_t i ) { return i < split; },
                                                    []  ( size_t& sum, int v) { sum += v; },
                                                    []  ( size_t& sum, size_t v) { sum += v; });

        /*serial_partitioning(array.data(),0,array.size(),left_sum,right_sum,
                            [&] ( size_t i ) { return i < split; },
                            []  ( size_t& left_sum, int v) { left_sum += v; });*/

        /* verify result */
        passed &= left_sum == split*(split-1)/2;
        passed &= right_sum == N*(N-1)/2-left_sum;
        for (size_t i=0; i<split; i++) passed &= array[i] < split;
        for (size_t i=split; i<N; i++) passed &= array[i] >= split;
      }
      
      return passed;
    }
  };

  parallel_partition_regression_test parallel_partition_regression("parallel_partition_regression_test");
}
