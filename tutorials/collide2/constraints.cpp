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

namespace embree { namespace collide2 {

float distance (Vertex const & v1, Vertex const & v2) {
  return std::sqrt ((v1.x-v2.x)*(v1.x-v2.x)+(v1.y-v2.y)*(v1.y-v2.y)+(v1.z-v2.z)*(v1.z-v2.z));
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

    auto d = distance (x1, x0);
    auto delta = d - rl_;
    auto normd = 1.f / d;

    if (iter == 0) {
        lambda_old_ = 0.f;
    }
    auto lambda = (delta - alpha * lambda_old_) / (wSum + alpha);
    lambda_old_ = lambda;

    vec_t c;
    c.x = lambda * normd * (x1.x - x0.x);
    c.y = lambda * normd * (x1.y - x0.y);
    c.z = lambda * normd * (x1.z - x0.z);

    if (mi0 != 0.f) {
        x0.x += mi0 * c.x;
        x0.y += mi0 * c.y;
        x0.z += mi0 * c.z;
    }

    if (mi1 != 0.f) {
        x1.x -= mi1 * c.x;
        x1.y -= mi1 * c.y;
        x1.z -= mi1 * c.z;
    }
}

} // namespace collide2
} // namespace embree