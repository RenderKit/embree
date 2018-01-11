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

#include "distribution1d.h"

namespace embree
{
  /*! 2D probability distribution. The probability distribution
   *  function (PDF) can be initialized with arbitrary data and be
   *  sampled. */
  class Distribution2D
  {
  public:

    /*! Default construction. */
    Distribution2D();

    /*! Construction from 2D distribution array f. */
    Distribution2D(const float* f, const size_t width, const size_t height);

    /*! Initialized the PDF and CDF arrays. */
    void init(const float* f, const size_t width, const size_t height);

  public:

    /*! Draws a sample from the distribution. \param u is a pair of
     *  random numbers to use for sampling */
    Vec2f sample(const Vec2f& u) const;

    /*! Returns the probability density a sample would be drawn from
     *  location p. */
    float pdf(const Vec2f& p) const;

    size_t width;            //!< Number of elements in x direction
    size_t height;           //!< Number of elements in y direction
  private:
    Distribution1D yDist;    //!< Distribution to select between rows
    std::vector<Distribution1D> xDists;  //!< One 1D Distribution per row
  };
}

