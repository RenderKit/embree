// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "default.h"

namespace embree
{
  /*! 1D probability distribution. The probability distribution
   *  function (PDF) can be initialized with arbitrary data and be
   *  sampled. */
  class Distribution1D
  {
  public:

    /*! Default construction. */
    Distribution1D();

    /*! Construction from distribution array f. */
    Distribution1D(const float* f, const size_t size);

    /*! Initialized the PDF and CDF arrays. */
    void init(const float* f, const size_t size);

  public:

    /*! Draws a sample from the distribution. \param u is a random
     *  number to use for sampling */
    float sample(const float u) const;

    /*! Returns the probability density a sample would be drawn from location p. */
    float pdf(const float p) const;

  private:
    size_t size;  //!< Number of elements in the PDF
    std::vector<float> PDF;   //!< Probability distribution function
    std::vector<float> CDF;   //!< Cumulative distribution function (required for sampling)
  };
}

