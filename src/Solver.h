//
// Created by hana on 5/7/26.
//

#ifndef SOLVER_H
#define SOLVER_H

#include "SimulatedObject.h"

class Solver {
    double timeStep;
    int solverIterations = 10;
    SimulatedObject so;
public:
    explicit Solver(Object& obj, double ground, double time_step) : timeStep(time_step) {
        so.initializeFromObject(obj, ground);
    };
    void resetSimulation(Object& obj, double ground);
    void step() const;

    [[nodiscard]] std::vector<std::shared_ptr<Particle>> getParticles() const { return so.particles; };
    void setTimeStep(const double time_step) { timeStep = time_step; };
    void setSolverIterations(const int solver_iterations) { solverIterations = solver_iterations; };
    void setDistanceStiffness(const double dist_stiffness) { so.distanceStiffness = dist_stiffness; };
    void setVolumeStiffness(const double volume_stiffness) { so.volumeStiffness = volume_stiffness; };
    void setShapeMatchingStiffness(const double shape_m_stiffness) { so.shapeMatchingStiffness = shape_m_stiffness; };
    void setOutsideForces(const Eigen::Vector3d& outside_forces) { so.outsideForces = outside_forces; };
    void setAlgorithmType(const AlgorithmType algorithm_type) {so.algorithmType = algorithm_type; };
};


#endif //SOLVER_H
