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

    __forceinline float set_cos_2PI_div_n(const size_t n) { return cosf(2.0f*M_PI/(float)n); }

  public:

    __forceinline float cos_2PI_div_n(const size_t n)
    {
      assert(n < MAX_VALENCE);

      if (likely(n < MAX_VALENCE))
        return table_cos_2PI_div_n[n];
      else
        return set_cos_2PI_div_n(n);
    }

    static CatmullClarkPrecomputedCoefficients table;
 
    CatmullClarkPrecomputedCoefficients();
    
  };

};
