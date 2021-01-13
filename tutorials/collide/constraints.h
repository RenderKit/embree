// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stddef.h>
#include "../common/math/vec.h"

namespace embree { namespace collide2 {

using vec_t = Vec3fa;

struct ClothModel;

class Constraint {
public:

    Constraint (size_t const numConstainedBodies) 
    : numConstrainedBodies_ (numConstainedBodies)
    {
        bodyIDs_ = new size_t[numConstrainedBodies_];
    }

    virtual ~Constraint () { delete[] bodyIDs_; }

    virtual void solvePositionConstraint (ClothModel & model, float timeStep, size_t iter) = 0;

private:
    Constraint (const Constraint& other) DELETED; // do not implement
    Constraint& operator= (const Constraint& other) DELETED; // do not implement

protected:
    size_t  numConstrainedBodies_ = 0;
    size_t* bodyIDs_ = nullptr;
};

class DistanceConstraint : public Constraint {
public:

    DistanceConstraint ()
    :
        Constraint (2)
    {}

    virtual void initConstraint             (ClothModel const & model, size_t p0ID, size_t p1ID);
    virtual void solvePositionConstraint    (ClothModel & model, float timeStep, size_t iter);

protected:
    float rl_ {0.f};
    float lambda_old_0_ {0.f};
    float lambda_old_1_ {0.f};
};

class CollisionConstraint : public Constraint {
  ALIGNED_CLASS_(16);
public:

    CollisionConstraint ()
      : Constraint (1) {}

    virtual void initConstraint             (size_t qID, const vec_t& x0, const vec_t& n, float d);
    virtual void solvePositionConstraint    (ClothModel & model, float timeStep, size_t iter);

protected:
    vec_t x0_   = vec_t(0.f, 0.f, 0.f);
    vec_t n_    = vec_t(0.f, 0.f, 0.f);
    float d_    = 1.e5f;
};

} // namespace collide2
} // namespace embree
