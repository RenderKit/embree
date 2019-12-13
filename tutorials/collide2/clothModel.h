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

#pragma once

#include <vector>
#include "constraints.h"
#include "../common/tutorial/tutorial_device.h"

namespace embree { namespace collide2 {

using vec_t = Vertex;

class Constraint;

struct ClothModel {

    // particle system
    std::vector<Vertex>         x_;
    std::vector<Vertex>         x_0_;
    std::vector<Vertex>         x_old_;
    std::vector<Vertex>         x_last_;
    std::vector<vec_t>          v_;
    std::vector<vec_t>          a_;
    std::vector<float>          m_;
    std::vector<float>          m_inv_;

    // mesh connectivity
    std::vector<Triangle>       tris_;

    // simulation constraints
    std::vector<Constraint*>    constraints_;

    // material parameters
    float                       k_stretch_ = 1.f;
    //float                       k_bending_ = 1.f;

    virtual ~ClothModel () {
        for (auto c : constraints_) {
            delete c;
        }
        constraints_.clear ();
    }
};

} // namespace collide2
} // namespace embree