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

#include "../geometry.h"
#include "../scene_subdiv_mesh.h"

namespace embree
{
  class CatmullClarkPrecomputedCoefficients {

  private:
    
    static const size_t MAX_VALENCE = SubdivMesh::MAX_RING_EDGE_VALENCE;
    
    float table_cos_2PI_div_n[MAX_VALENCE];

    float *table_limittangent_a[MAX_VALENCE];
    float *table_limittangent_b[MAX_VALENCE];

    __forceinline float set_cos_2PI_div_n(const size_t n) { return cosf(2.0f*M_PI/(float)n); }

    __forceinline float set_limittangent_a(const size_t i, const size_t n)  { 
      const float c0 = 1.0f/(float)n * 1.0f / sqrtf(4.0f + cosf(M_PI/(float)n)*cosf(M_PI/(float)n));  
      const float c1 = (1.0f/(float)n + cosf(M_PI/(float)n) * c0); 
      return cosf(2.0f*M_PI*(float)i/(float)n) * c1;
    }

    __forceinline float set_limittangent_b(const size_t i, const size_t n)  { 
      const float c0 = 1.0f/(float)n * 1.0f / sqrtf(4.0f + cosf(M_PI/(float)n)*cosf(M_PI/(float)n));  
      return cosf((2.0f*M_PI*i+M_PI)/(float)n) * c0;
    }

  public:

    __forceinline float cos_2PI_div_n(const size_t n)
    {
      assert(n < MAX_VALENCE);

      if (likely(n < MAX_VALENCE))
        return table_cos_2PI_div_n[n];
      else
        return set_cos_2PI_div_n(n);
    }

    __forceinline float limittangent_a(const size_t i, const size_t n)
    {
      assert(n < MAX_VALENCE);
      assert(i < n);
      return table_limittangent_a[n][i];
    }

    __forceinline float limittangent_b(const size_t i, const size_t n)
    {
      assert(n < MAX_VALENCE);
      assert(i < n);
      return table_limittangent_b[n][i];
    }

    static CatmullClarkPrecomputedCoefficients table;
 
    CatmullClarkPrecomputedCoefficients();
    
  };

};
