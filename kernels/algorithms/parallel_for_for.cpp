// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "parallel_for_for.h"

namespace embree
{
  struct parallel_for_for_regression_test : public RegressionTest
  {
    parallel_for_for_regression_test(const char* name) : name(name) {
      registerRegressionTest(this);
    }
    
    bool operator() ()
    {
      bool passed = true;
      printf("%s::%s ... ",TOSTRING(isa),name);
      fflush(stdout);

      /* create vector with random numbers */
      size_t sum0 = 0;
      const size_t M = 4;
      std::vector<std::vector<size_t> > array2(M);
      for (size_t i=0; i<M; i++) {
        const size_t N = ::random() % 1024;
        array2[i].resize(N);
        for (size_t j=0; j<N; j++) 
          sum0 += array2[i][j] = ::random() % 1024;
      }

      /* add all numbers using parallel_for_for */
      AtomicCounter sum1 = 0;
      parallel_for_for( array2, size_t(1), [&](const std::vector<size_t>& v, const range<size_t>& r) 
      {
        size_t s = 0;
	for (size_t i=r.begin(); i<r.end(); i++) 
	  s += v[i];
        sum1 += s;
      });
      passed = sum0 == sum1;
      
      /* output if test passed or not */
      if (passed) printf("[passed]\n");
      else        printf("[failed]\n");
      
      return passed;
    }

    const char* name;
  };

  parallel_for_for_regression_test parallel_for_for_regression("parallel_for_for_regression_test");
}
