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

#include "sort.h"

namespace embree
{
//  namespace isa
//  {
    struct ParallelSortRegressionTest : public RegressionTest
    {
      ParallelSortRegressionTest() {
        registerRegressionTest(this);
      }

      bool operator() ()
      {
        bool passed = true;
        printf("%s::ParallelSortRegressionTest ... ",TOSTRING(isa));
        fflush(stdout);

        /* instantiate parallel sort */
        ParallelSortUInt32 sort;

	const size_t M = 10;
        for (size_t N=10; N<10000000; N*=2.1f)
        {
          std::vector<unsigned> src(N); memset(&src[0],0,N*sizeof(unsigned));
          std::vector<unsigned> tmp(N); memset(&tmp[0],0,N*sizeof(unsigned));
          std::vector<unsigned> dst(N); memset(&dst[0],0,N*sizeof(unsigned));
          for (size_t i=0; i<N; i++) src[i] = random();

          /* calculate checksum */
          size_t sum0 = 0; for (size_t i=0; i<N; i++) sum0 += src[i];
          
          /* sort numbers */
          double t0 = getSeconds();
	  for (size_t i=0; i<M; i++)
	    sort(&src[0],&tmp[0],&dst[0],N);
          double t1 = getSeconds();
          printf("%zu/%3.2fM ",N,1E-6*double(N*M)/(t1-t0));

          /* calculate checksum */
          size_t sum1 = 0; for (size_t i=0; i<N; i++) sum1 += dst[i];
          if (sum0 != sum1) {
            printf("c");
            passed = false;
          }
          
          /* check if numbers are sorted */
          for (size_t i=1; i<N; i++) {
            if (dst[i-1] <= dst[i]) continue;
            printf("s");
            break;
          }
        }

        /* output if test passed or not */
        if (passed) printf("[passed]\n");
        else        printf("[failed]\n");
        
        return passed;
      }
    } ParallelSortRegressionTest;
//  }
}
