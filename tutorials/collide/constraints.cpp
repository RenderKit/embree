// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "constraints.h"
#include "clothModel.h"
#include <cmath>

namespace embree { namespace collide2 {

void DistanceConstraint::initConstraint (ClothModel const & model, size_t p0ID, size_t p1ID) {
    
    bodyIDs_[0] = p0ID;
    bodyIDs_[1] = p1ID;

    auto & x0 = model.x_0_[p0ID];
    auto & x1 = model.x_0_[p1ID];

    rl_ = distance (x1, x0);
}

void DistanceConstraint::solvePositionConstraint (ClothModel & model, float timeStep, size_t iter) {

    auto mi0 = model.m_inv_[bodyIDs_[0]];
    auto mi1 = model.m_inv_[bodyIDs_[1]];
    auto wSum = mi0 + mi1;
    if (0.f == wSum) {
        return;
    }

    auto & x0 = model.x_[bodyIDs_[0]];
    auto & x1 = model.x_[bodyIDs_[1]];
    auto alpha = 1.f / (model.k_stretch_ * timeStep * timeStep);
    auto gamma = model.k_damp_ / (model.k_stretch_ * timeStep);

    auto d = distance (x1, x0);
    auto delta = d - rl_;
    auto normd = 1.f / d;

    if (iter == 0) {
        lambda_old_0_ = 0.f;
        lambda_old_1_ = 0.f;
    }
    auto norml = 1.f / ((1.f+gamma)*wSum + alpha);

    vec_t jac;
    jac.x = normd * (x1.x - x0.x);
    jac.y = normd * (x1.y - x0.y);
    jac.z = normd * (x1.z - x0.z);

    if (mi0 != 0.f) {
        auto & xl = model.x_old_[bodyIDs_[0]];
        auto dot = jac.x*(x0.x-xl.x) + jac.y*(x0.y-xl.y) + jac.z*(x0.z-xl.z);
        auto lambda = norml * (delta - alpha*lambda_old_0_ - gamma*dot);
        x0.x += mi0 * jac.x * lambda;
        x0.y += mi0 * jac.y * lambda;
        x0.z += mi0 * jac.z * lambda;
        lambda_old_0_ = lambda;
    }

    if (mi1 != 0.f) {
        auto & xl = model.x_old_[bodyIDs_[1]];
        auto dot = jac.x*(x1.x-xl.x) + jac.y*(x1.y-xl.y) + jac.z*(x1.z-xl.z);
        auto lambda = norml * (-delta - alpha*lambda_old_1_ - gamma*dot);
        x1.x += mi1 * jac.x * lambda;
        x1.y += mi1 * jac.y * lambda;
        x1.z += mi1 * jac.z * lambda;
        lambda_old_1_ = lambda;
    }
}

void CollisionConstraint::initConstraint (size_t qID, const vec_t& x0, const vec_t& n, float d) {

    bodyIDs_[0] = qID;
    x0_ = x0;
    n_ = n;
    d_ = d;
}

void CollisionConstraint::solvePositionConstraint (ClothModel & model, float timeStep, size_t iter) {

    if (0.f == model.m_inv_[bodyIDs_[0]]) {
        return;
    }

    auto & q = model.x_[bodyIDs_[0]];
    float proj = dot (q-x0_, n_) - d_;//(q.x - x0_.x) * n_.x + (q.y - x0_.y) * n_.y + (q.z - x0_.z) * n_.z - d_;
    if (proj < -d_) {
        q += -n_*proj;
    }
}

} // namespace collide2
} // namespace embree
