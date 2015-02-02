// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#pragma once

#include "sys/platform.h"

namespace embree
{
  struct ProfileTimer
  {
    static const size_t N = 20;
    
    ProfileTimer () {}

    ProfileTimer (const size_t numSkip) : i(0), j(0), maxJ(0), numSkip(numSkip), t0(0)
    {
      for (size_t i=0; i<N; i++) names[i] = "unknown";
      for (size_t i=0; i<N; i++) dt_fst[i] = 0.0;
      for (size_t i=0; i<N; i++) dt_min[i] = pos_inf;
      for (size_t i=0; i<N; i++) dt_avg[i] = 0.0;
      for (size_t i=0; i<N; i++) dt_max[i] = neg_inf;
    }
    
    __forceinline void begin() 
    {
      j=0;
      t0 = getSeconds();
    }

    __forceinline void end() {
      i++;
    }

    __forceinline void operator() (const char* name) 
    {
      const double t1 = getSeconds();
      const double dt = t1-t0;
      t0 = t1;
      names[j] = name;
      if (i == 0) dt_fst[j] = dt;
      if (i>=numSkip) {
        dt_min[j] = min(dt_min[j],dt);
        dt_avg[j] = dt_avg[j] + dt;
        dt_max[j] = max(dt_max[j],dt);
      }
      j++;
      maxJ = max(maxJ,j);
    }

    void print(size_t numElements) 
    {
      printf("  profile:\n");

      for (size_t k=0; k<N; k++) 
        dt_avg[k] /= double(i-numSkip);

      for (size_t j=0; j<maxJ; j++) {
        printf("%20s:  fst = %7.2f M/s, min = %7.2f M/s, avg = %7.2f M/s, max = %7.2f M/s\n",
               names[j],numElements/dt_fst[j]*1E-6,numElements/dt_max[j]*1E-6,numElements/dt_avg[j]*1E-6,numElements/dt_min[j]*1E-6);
      }
    }

    void print() 
    {
      printf("  profile:\n");

      for (size_t k=0; k<N; k++) 
        dt_avg[k] /= double(i-numSkip);

      for (size_t j=0; j<maxJ; j++) {
        printf("%20s:  fst = %7.2f ms, min = %7.2f ms, avg = %7.2f ms, max = %7.2fms\n",
               names[j],1000.0*dt_fst[j],1000.0*dt_min[j],1000.0*dt_avg[j],1000.0*dt_max[j]);
      }
    }
    
  private:
    size_t i;
    size_t j;
    size_t maxJ;
    size_t numSkip;
    double t0;
    const char* names[N];
    double dt_fst[N];
    double dt_min[N];
    double dt_avg[N];
    double dt_max[N];
  };

  template<typename Closure>
    void profile(const size_t numSkip, const size_t numIter, const size_t numElements, const Closure& closure) 
    {
      ProfileTimer timer(numSkip);
      
      for (size_t i=0; i<numSkip+numIter; i++) 
      {
        timer.begin();
	closure(timer);
        timer.end();
      }
      timer.print(numElements);
    }

  template<typename Closure>
    void profile(ProfileTimer& timer, const size_t numSkip, const size_t numIter, const size_t numElements, const Closure& closure) 
    {
      new (&timer) ProfileTimer(numSkip);
      
      for (size_t i=0; i<numSkip+numIter; i++) 
      {
        timer.begin();
	closure(timer);
        timer.end();
      }
      timer.print(numElements);
    }
}
