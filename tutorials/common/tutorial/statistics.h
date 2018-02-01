// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "../default.h"
#include <algorithm>

namespace embree
{
  /* calculates min/avg/max and sigma */
  struct Statistics
  {
  public:
    Statistics() 
      : v(0.0f), v2(0.0f), vmin(pos_inf), vmax(neg_inf), N(0) {}

    void add(float a) 
    {
      v += a;
      v2 += a*a;
      vmin = min(vmin,a);
      vmax = max(vmax,a);
      N++;
    }

    float getSigma() const 
    {
      if (N == 0) return 0.0f;
      else return (float) sqrt(max(0.0,v2/N - sqr(v/N)));
    }

    float getAvgSigma() const // standard deviation of average
    {
      if (N == 0) return 0.0f;
      else return getSigma()/sqrt(float(N));
    }

    float getMin() const { return vmin; }
    float getMax() const { return vmax; }
    float getAvg() const { return float(v/N); }

  private:
    double v;   // sum of all values
    double v2;  // sum of squares of all values
    float vmin; // min of all values
    float vmax; // max of all values
    size_t N;   // number of values
  };

  /* filters outlyers out */
  struct FilteredStatistics
  {
  public:
    FilteredStatistics(float fskip_small, float fskip_large) 
      : fskip_small(fskip_small), fskip_large(fskip_large) {}

    void add(float a) 
    {
      v.push_back(a);
      std::sort(v.begin(),v.end(),std::less<float>());
      size_t skip_small = (size_t) floor(0.5*fskip_small*double(v.size()));
      size_t skip_large = (size_t) floor(0.5*fskip_large*double(v.size()));

      stat = Statistics();
      for (size_t i=skip_small; i<v.size()-skip_large; i++)
        stat.add(v[i]);
    }

    float getSigma() const { return stat.getSigma(); }
    float getAvgSigma() const { return stat.getAvgSigma(); }
    float getMin() const { return stat.getMin(); }
    float getMax() const { return stat.getMax(); }
    float getAvg() const { return stat.getAvg(); }
    Statistics getStatistics() const { return stat; }

  private:
    float fskip_small; // fraction of small outlyers to filter out
    float fskip_large; // fraction of large outlyers to filter out
    std::vector<float> v;
    Statistics stat;
  };
}
