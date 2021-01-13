// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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

