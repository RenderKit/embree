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

#pragma once

namespace embree
{
  struct FractionalTessellationPattern
  {
    FractionalTessellationPattern (const float tess_factor)
    : tess_factor(tess_factor), 
      Nl(floor(tess_factor)), 
      Nh(Nl+1), 
      t0(tess_factor-float(Nh)), 
      t1(float(Nl)-tess_factor) {}

    const float operator() (const int i)
    {
      const int Nl2 = Nl/2;
      if (unlikely(t1 == 0.0f))
        return float(i)*rcp(tess_factor);

      if (Nl % 2 == 0) {
        const float f0 = float(i > Nl2) * t0;
        return (float(i) + f0)*rcp(tess_factor);
      } 
      else {
        const float f0 = (i > (Nl2+0)) * t0;
        const float f1 = (i > (Nl2+1)) * t1;
        const float f2 = (i > (Nl2+2)) * t0;
        return (float(i) + f0 + f1 + f2)*rcp(tess_factor);
      }

    public:
      const float tess_factor;
      const int Nl;
      const float t0;
      const float t1;
    };

#if 0
  const ssef fractional_tessellation_pattern(const ssei i, const float tess_factor)
  {
    const int Nl = (int) floor(tess_factor), Nh = Nl+1;
    const float rcp_tess_factor = 1.0f/tess_factor;
    if (Nl % 2 == 0) {
      const float f0 = select(i > ssei(Nl/2),ssef(tess_factor-float(Nh)),ssef(0.0f));
      return (ssef(i) + f0)*rcp_tess_factor;
    } else {
      const float t0 = tess_factor-float(Nh);
      const float t1 = float(Nl)-tess_factor;
      const float f0 = select(i > ssei(Nl/2+0),ssef(t0),ssef(0.0f));
      const float f1 = select(i > ssei(Nl/2+1),ssef(t1),ssef(0.0f));
      const float f2 = select(i > ssei(Nl/2+2),ssef(t0),ssef(0.0f));
      return (ssef(i) + f0 + f1 + f2)*rcp_tess_factor;
    }
  }
#endif
}
