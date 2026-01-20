//
// Created by hana on 12/1/25.
//

#ifndef PBD_H
#define PBD_H

#include "SimulatedObject.h"

class PBD {
    double timeStep;
    int solverIterations = 10;
    SimulatedObject so;
    public:
    explicit PBD(Object& obj, double ground, double time_step) : timeStep(time_step) {
        so.initializeFromObject(obj, ground);
    };
    void resetSimulation(Object& obj);
    void step();

    const std::vector<std::shared_ptr<Particle>> getParticles() { return so.particles; };
    void setTimeStep(double time_step) { timeStep = time_step; };
    void setSolverIterations(int solver_iterations) { solverIterations = solver_iterations; };
};



#endif //PBD_H
