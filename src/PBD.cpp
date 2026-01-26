//
// Created by hana on 12/1/25.
//

#include "PBD.h"

#include <iostream>

void PBD::step() const {

    for (auto& p : so.particles) {
        p->v += timeStep * p->w * p->F;
        p->p = p->x + timeStep * p->v;
    }

    for (int iter = 0; iter < solverIterations; ++iter) {

        for (auto& c : so.constraints)
            c->project();

        for (auto& c : so.collisionConstraints)
            c->project();
    }

    for (auto& p : so.particles) {
        p->v = (p->p - p->x) / timeStep;
        p->x = p->p;
    }
}

void PBD::resetSimulation(Object& obj, double ground) {
    so.initializeFromObject(obj, ground);
}



