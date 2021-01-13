// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <vector>
#include "constraints.h"
#include "../common/tutorial/tutorial_device.h"

namespace embree { namespace collide2 {

class Constraint;

struct Mesh {
    
    virtual ~Mesh() {};
    
    std::vector<vec_t>          x_;
    std::vector<Triangle>       tris_;
};

struct ClothModel : public Mesh {

    // particle system
    std::vector<vec_t>          x_0_;
    std::vector<vec_t>          x_old_;
    std::vector<vec_t>          x_last_;
    std::vector<vec_t>          v_;
    std::vector<vec_t>          a_;
    std::vector<float>          m_;
    std::vector<float>          m_inv_;

    // simulation constraints
    std::vector<Constraint*>    m_constraints_;
    std::vector<Constraint*>    c_constraints_;

    // material parameters
    float                       k_stretch_  = 1.f;
    float                       k_damp_     = 1.f;

    void clearMotionConstraints()
    {
      for (auto c : m_constraints_) {
        delete c;
      }
      m_constraints_.clear ();
    }

    void clearCollisionConstraints()
    {
      for (auto c : c_constraints_) {
        delete c;
      }
      c_constraints_.clear ();
    }
      
    virtual ~ClothModel () {
      clearMotionConstraints();
      clearCollisionConstraints();
    }
};

} // namespace collide2

  void initializeClothPositions (collide2::ClothModel & cloth);

} // namespace embree
