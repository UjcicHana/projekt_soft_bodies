//
// Created by hana on 5/7/26.
//

#include "Solver.h"

void Solver::step() {
    if (so.algorithmType == AlgorithmType::XPBD) {
        so.resetLambdaConstraints();
    }

    for (auto& p : so.particles) {
        p->v += timeStep * p->w * p->F;
        p->p = p->x + timeStep * p->v;
    }

    for (int iter = 0; iter < solverIterations; ++iter) {
        so.projectConstraints();
    }

    for (auto& p : so.particles) {
        p->v = (p->p - p->x) / timeStep;
        p->x = p->p;
    }

    //for (auto& c : so.constraints) c->print();
}

void Solver::resetSimulation(Object& obj, float ground) {
    so.initializeFromObject(obj, ground);
}


