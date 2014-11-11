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
    FractionalTessellationPattern (const float tess)
    : tess(tess), rcp_tess(1.0f/tess),
      N(0),
      Nl(floor(tess)), 
      Nh(Nl+1), 
      th(tess-float(Nh)), 
      tl(float(Nl)-tess) 
      {
        if (unlikely(tl > -16.0f*float(ulp))) N = Nl;
        else N = Nl+1+(Nl % 2);
      }

    __forceinline int size() const {
      return N;
    }

    __forceinline float operator() (const int i) const
    {
      const int Nl2 = Nl/2;
      if (unlikely(tl > -16.0f*float(ulp)))
        return min(float(i)*rcp_tess,1.0f);

      if (Nl % 2 == 0) {
        const float f0 = float(i > Nl2) * th;
        return min((float(i) + f0)*rcp_tess,1.0f);
      } 
      else {
        const float f0 = (i > (Nl2+0)) * th;
        const float f1 = (i > (Nl2+1)) * tl;
        const float f2 = (i > (Nl2+2)) * th;
        return min((float(i) + f0 + f1 + f2)*rcp_tess,1.0f);
      }
    }

  private:
    const float tess, rcp_tess;
    const int   Nl, Nh;
    int N;
    const float tl, th;
  };

#if 0
  const ssef fractional_tessellation_pattern(const ssei i, const float tess)
  {
    const int Nl = (int) floor(tess), Nh = Nl+1;
    const float rcp_tess = 1.0f/tess;
    if (Nl % 2 == 0) {
      const float f0 = select(i > ssei(Nl/2),ssef(tess-float(Nh)),ssef(0.0f));
      return (ssef(i) + f0)*rcp_tess;
    } else {
      const float th = tess-float(Nh);
      const float tl = float(Nl)-tess;
      const float f0 = select(i > ssei(Nl/2+0),ssef(th),ssef(0.0f));
      const float f1 = select(i > ssei(Nl/2+1),ssef(tl),ssef(0.0f));
      const float f2 = select(i > ssei(Nl/2+2),ssef(th),ssef(0.0f));
      return (ssef(i) + f0 + f1 + f2)*rcp_tess;
    }
  }
#endif
}
