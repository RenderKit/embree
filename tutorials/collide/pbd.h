// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "clothModel.h"
#include "constraints.h"
#include "../../common/algorithms/parallel_for.h"

namespace embree { namespace collide2 {

void updatePositions (ClothModel & model, float h) {

    parallel_for ((size_t)0, model.x_.size (), [&] (const range<size_t>& r) {
        for (size_t i=r.begin(); i<r.end(); i++) {

            model.x_last_[i].x = model.x_old_[i].x;
            model.x_last_[i].y = model.x_old_[i].y;
            model.x_last_[i].z = model.x_old_[i].z;

            model.x_old_[i].x = model.x_[i].x;
            model.x_old_[i].y = model.x_[i].y;
            model.x_old_[i].z = model.x_[i].z;

            model.v_[i].x += model.a_[i].x * h;
            model.v_[i].y += model.a_[i].y * h;
            model.v_[i].z += model.a_[i].z * h;

            model.x_[i].x += model.v_[i].x * h;
            model.x_[i].y += model.v_[i].y * h;
            model.x_[i].z += model.v_[i].z * h;
        }
    });
}

void constrainPositions (ClothModel & model, float timeStep, size_t maxNumIterations) {

    size_t nIters = 0;

    while (nIters < maxNumIterations) {
        //parallel_for (0, model.constraints_.size(), [&] (const range<size_t>& r) {
        //    for (size_t i=r.begin(); i<r.end(); i++) {
            for (size_t i=0; i<model.m_constraints_.size (); ++i) {
                model.m_constraints_[i]->solvePositionConstraint (model, timeStep, nIters);
            }
        //});
        ++nIters;
    }

    nIters = 0;

    while (nIters < 1) {
        //parallel_for (0, model.constraints_.size(), [&] (const range<size_t>& r) {
        //    for (size_t i=r.begin(); i<r.end(); i++) {
            for (size_t i=0; i<model.c_constraints_.size (); ++i) {
                model.c_constraints_[i]->solvePositionConstraint (model, timeStep, nIters);
            }
        //});
        ++nIters;
    }
}

void updateVelocities (ClothModel & model, float h) {

    float hinv = 1.f/h;
    parallel_for ((size_t)0, model.x_.size (), [&] (const range<size_t>& r) {
        for (size_t i=r.begin(); i<r.end(); i++) {
            if (model.m_[i] != 0.f) {

                // first order
                model.v_[i].x = hinv * (model.x_[i].x - model.x_old_[i].x);
                model.v_[i].y = hinv * (model.x_[i].y - model.x_old_[i].y);
                model.v_[i].z = hinv * (model.x_[i].z - model.x_old_[i].z);

                // second order
                // model.v_[i].x = hinv * (1.5f * model.x_[i].x - 2.f * model.x_old_[i].x + .5f * model.x_last_[i].x);
                // model.v_[i].y = hinv * (1.5f * model.x_[i].y - 2.f * model.x_old_[i].y + .5f * model.x_last_[i].y);
                // model.v_[i].z = hinv * (1.5f * model.x_[i].z - 2.f * model.x_old_[i].z + .5f * model.x_last_[i].z);
            }
        }
    });    
}

} // namespace collide2
} // namespace embree
