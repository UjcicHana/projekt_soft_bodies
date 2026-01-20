//
// Created by hana on 12/1/25.
//

#include "PBD.h"

#include <iostream>


void PBD::step()
{
    // --- External forces ---
    so.calculateForces();

    // --- Semi-implicit Euler ---
    for (auto& p : so.particles) {
        p->v += timeStep * p->w * p->F;
        p->p = p->x + timeStep * p->v;
    }

    // --- Constraint solver ---
    for (int iter = 0; iter < solverIterations; ++iter) {

        for (auto& c : so.distanceConstraints)
            c->project();

        for (auto& c : so.collisionConstraints)
            c->project();

        // Shape matching constraint (restores volume/shape)
        so.projectShapeMatching();

        for (auto& c : so.volumeConstraints)
            c->project();
    }

    // --- Velocity update ---
    for (auto& p : so.particles) {
        p->v = (p->p - p->x) / timeStep;
        p->x = p->p;
    }
}

void PBD::resetSimulation(Object& obj) {
    so.initializeFromObject(obj);
}



