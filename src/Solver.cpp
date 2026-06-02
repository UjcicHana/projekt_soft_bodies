//
// Created by hana on 5/7/26.
//

#include "Solver.h"

bool overlaps(
    const AABB& a,
    const AABB& b
)
{
    return
        (a.min.x() <= b.max.x() &&
         a.max.x() >= b.min.x()) &&

        (a.min.y() <= b.max.y() &&
         a.max.y() >= b.min.y()) &&

        (a.min.z() <= b.max.z() &&
         a.max.z() >= b.min.z());
}

void Solver::step() {
    for (auto so : simObjects) {
        if (so->algorithmType == AlgorithmType::XPBD) {
            so->resetLambdaConstraints();
        }

        for (auto& p : so->particles) {
            p->v += timeStep * p->w * p->F;
            p->p = p->x + timeStep * p->v;
        }
    }
    for (auto so : simObjects) {
        //so->collisionConstraints.clear();

    }


    for (auto so : simObjects) {
        for (int iter = 0; iter < solverIterations; ++iter) {
            so->projectConstraints();
        }

        for (auto& p : so->particles) {
            p->v = (p->p - p->x) / timeStep;
            p->x = p->p;
        }

        //velocity damping
        for (auto& p : so->particles) {
            p->v *= 0.98f;
        }
    }

    //for (auto& c : so.constraints) c->print();
}

void Solver::resetSimulation() {
    for (auto so : simObjects)
        so->reset();
}


