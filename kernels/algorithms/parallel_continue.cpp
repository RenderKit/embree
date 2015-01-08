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

#include "parallel_continue.h"

namespace embree
{
  struct parallel_continue_regression_test : public RegressionTest
  {
    parallel_continue_regression_test(const char* name) : name(name) {
      registerRegressionTest(this);
    }

    struct Continuation 
    {
      size_t N;
      __forceinline Continuation () : N(0) {}
      __forceinline Continuation(size_t N) : N(N) {}

      __forceinline size_t size() const {
        return N;
      }
    };
    
    bool operator() ()
    {
      bool passed = true;
      printf("%s::%s ... ",TOSTRING(isa),name);
      fflush(stdout);

      atomic_t cntr = 0;

      for (size_t N=10; N<160000; N*=2.1f)
      {
        size_t N = 10000;
        std::vector<Continuation> continuations;
        for (size_t i=0; i<1; i++) {
          continuations.push_back(N);
          atomic_add(&cntr,1);
        }

        parallel_continue<1000>( continuations.data(), continuations.size(), [&](const Continuation& c, int& tl, ParallelContinue<Continuation>& cont) 
        {
          size_t N = c.N;
          atomic_add(&cntr,-1);
          if (N < 5) return;
          const size_t N0 = N/5, N1 = N-N0;
          atomic_add(&cntr,2);
          cont(Continuation(N0));
          cont(Continuation(N1));
        }, []() { return 0; });
        passed = cntr == 0;
      }
      
      /* output if test passed or not */
      if (passed) printf("[passed]\n");
      else        printf("[failed]\n");
      
      return passed;
    }

    const char* name;
  };

  parallel_continue_regression_test parallel_continue_regression("parallel_continue_regression_test");
}
