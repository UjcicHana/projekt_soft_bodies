//
// Created by hana on 5/7/26.
//

#ifndef SOLVER_H
#define SOLVER_H

#include "SimulatedObject.h"

class Solver {
    float timeStep;
    int solverIterations = 10;
    SimulatedObject so;
public:
    explicit Solver(Object& obj, float ground, float time_step) : timeStep(time_step) {
        so.initializeFromObject(obj, ground);
    };
    void resetSimulation(Object& obj, float ground);
    void step();

    [[nodiscard]] std::vector<std::shared_ptr<Particle>> getParticles() const { return so.particles; };
    void setTimeStep(const float time_step) { timeStep = time_step; so.timeStep = time_step; };
    void setSolverIterations(const int solver_iterations) { solverIterations = solver_iterations; };
    void setAlgorithmType(const AlgorithmType algorithm_type) { so.algorithmType = algorithm_type; };

    void setDistanceStiffness(const float dist_stiffness) { so.distanceStiffness = dist_stiffness; };
    void setVolumeStiffness(const float volume_stiffness) { so.volumeStiffness = volume_stiffness; };
    void setShapeMatchingStiffness(const float shape_m_stiffness) { so.shapeMatchingStiffness = shape_m_stiffness; };

    void setDistanceCompliance (const float distance_compliance) { so.distanceCompliance = distance_compliance; };
    void setVolumeCompliance (const float volume_compliance) { so.volumeCompliance = volume_compliance; };

    void setOutsideForces(const Eigen::Vector3f& outside_forces) { so.outsideForces = outside_forces; };
};


#endif //SOLVER_H
