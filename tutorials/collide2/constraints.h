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

#pragma once;

namespace embree { namespace collide2 {

class ClothModel;

class Constraint {
public:

    Constraint (size_t const numConstainedBodies) 
    :
        numConstrainedBodies_ (numConstainedBodies)
    {
        bodyIDs_ = new size_t[numConstrainedBodies_];
    }

    virtual ~Constraint () {
        delete[] bodyIDs_;
    }

    virtual bool solvePositionConstraint (ClothModel & model, size_t iter) = 0;

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

    virtual bool initConstraint (ClothModel & model, size_t p0ID, size_t p1ID);
    virtual bool solvePositionConstraint (ClothModel & model, size_t iter);

protected:

    float rl_ = 0.f;
};

} // namespace collide2
} // namespace embree