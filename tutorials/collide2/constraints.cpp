// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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

#include "constraints.h"
#include "clothModel.h"
#include <cmath>

namespace embree { namespace collide2 {

float dot (vec_t const & v1, vec_t const & v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}
vec_t cross (vec_t const & v1, vec_t const & v2) {
    vec_t ret;
    ret.x = v1.y*v2.z - v1.z*v2.y;
    ret.y = v1.z*v2.x - v1.x*v2.z;
    ret.z = v1.x*v2.y - v1.y*v2.x;
    return ret;
}

void normalize (vec_t & v) {
    auto norm = 1.f / sqrtf (dot(v,v));
    v.x *= norm;
    v.y *= norm;
    v.z *= norm;
}

float distance (Vertex const & v1, Vertex const & v2) {
  return sqrtf ((v1.x-v2.x)*(v1.x-v2.x)+(v1.y-v2.y)*(v1.y-v2.y)+(v1.z-v2.z)*(v1.z-v2.z));
}

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

void CollisionConstraint::initConstraint (size_t qID, vec_t x0, vec_t n, float d) {

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
    float proj = (q.x - x0_.x) * n_.x + (q.y - x0_.y) * n_.y + (q.z - x0_.z) * n_.z - d_;
    if (proj < -d_) {
        q.x += -n_.x * proj;
        q.y += -n_.y * proj;
        q.z += -n_.z * proj;
    }
}

} // namespace collide2
} // namespace embree