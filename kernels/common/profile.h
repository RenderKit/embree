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
  template<typename Closure>
    void profile(const std::string& name, const size_t numSkip, const size_t numIter, const size_t numElements, const Closure& closure) 
    {
      std::cout << "profiling " << name << ": " << std::endl;
      double dt_fst = 0.0f;
      double dt_min = pos_inf;
      double dt_avg = 0.0f;
      double dt_max = neg_inf;
      for (size_t i=0; i<numSkip+numIter; i++) 
      {
	double t0 = getSeconds();
	closure();
	double dt = getSeconds()-t0;
	if (i == 0) dt_fst = dt;
	if (i<numSkip) continue;
	dt_min = min(dt_min,dt);
	dt_avg = dt_avg + dt;
	dt_max = max(dt_max,dt);
      }
      dt_avg /= double(numIter);
      
      std::cout << "  fst = " << 1000.0f*dt_fst << "ms (" << numElements/dt_fst*1E-6 << " M/s)" << std::endl;
      std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numElements/dt_min*1E-6 << " M/s)" << std::endl;
      std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numElements/dt_avg*1E-6 << " M/s)" << std::endl;
      std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numElements/dt_max*1E-6 << " M/s)" << std::endl;
    }
}
