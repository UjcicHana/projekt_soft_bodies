//
// Created by hana on 5/7/26.
//

#include "Solver.h"

void Solver::step() const {
    if (so.algorithmType == AlgorithmType::XPBD)
    {
        for (auto& c : so.constraints)
            c->lambda = 0.0;
    }

    for (auto& p : so.particles) {
        p->v += timeStep * p->w * p->F;
        p->p = p->x + timeStep * p->v;
    }

    for (int iter = 0; iter < solverIterations; ++iter) {

        for (auto& c : so.constraints)
            c->project(so.algorithmType, 0.0);

        for (auto& c : so.collisionConstraints)
            c->project(so.algorithmType, 0.0);
    }

    for (auto& p : so.particles) {
        p->v = (p->p - p->x) / timeStep;
        p->x = p->p;
    }
}

void Solver::resetSimulation(Object& obj, double ground) {
    so.initializeFromObject(obj, ground);
}


