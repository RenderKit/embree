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

#include "prefix.h"

namespace embree
{
  struct prefix_sum_regression_test : public RegressionTest
  {
    prefix_sum_regression_test(const char* name) : RegressionTest(name) {
      registerRegressionTest(this);
    }
    
    bool run ()
    {
      bool passed = true;
      const size_t M = 10;
      
      for (size_t N=10; N<10000000; N=size_t(2.1*N))
      {
	/* initialize array with random numbers */
        uint32_t sum0 = 0;
	std::vector<uint32_t> src(N);
	for (size_t i=0; i<N; i++) {
	  sum0 += src[i] = rand();
        }
        
	/* calculate parallel prefix sum */
	std::vector<uint32_t> dst(N);
	memset(dst.data(),0,N*sizeof(uint32_t));
	
	for (size_t i=0; i<M; i++) {
	  uint32_t sum1 = parallel_prefix_sum(src,dst,N,std::plus<uint32_t>());
          passed &= (sum0 == sum1);
        }
        
	/* check if prefix sum is correct */
	for (size_t i=0, sum=0; i<N; sum+=src[i++])
	  passed &= ((uint32_t)sum == dst[i]);
      }
      
      return passed;
    }
  };

  prefix_sum_regression_test prefix_sum_regression("prefix_sum_regression");
}
