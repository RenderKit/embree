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

#include "distribution2d.h"

namespace embree
{
  Distribution2D::Distribution2D()
    : width(0), height(0) {}

  Distribution2D::Distribution2D(const float* f, const size_t width, const size_t height)
    : width(width), height(height)
  {
    init(f, width, height);
  }

  void Distribution2D::init(const float* f, const size_t w, const size_t h)
  {
    /*! create arrays */
    width = w; height = h;
    xDists.resize(height);
    std::vector<float> fy(height);

    /*! compute y distribution and initialize row distributions */
    for (size_t y=0; y<height; y++)
    {
      /*! accumulate row to compute y distribution */
      fy[y] = 0.0f;
      for (size_t x=0; x<width; x++)
        fy[y] += f[y*w+x];

      /*! initialize distribution for current row */
      xDists[y].init(f+y*width, width);
    }

    /*! initializes the y distribution */
    yDist.init(fy.data(), height);
  }

  Vec2f Distribution2D::sample(const Vec2f& u) const
  {
    /*! use u.y to sample a row */
    float sy = yDist.sample(u.y);
    int idx = clamp(int(sy),0,int(height)-1);

    /*! use u.x to sample inside the row */
    float sx = xDists[idx].sample(u.x);
    return Vec2f(sx,sy);
  }

  float Distribution2D::pdf(const Vec2f& p) const {
    int idx = clamp(int(p.y*height),0,int(height)-1);
    return xDists[idx].pdf(p.x) * yDist.pdf(p.y);
  }
}
