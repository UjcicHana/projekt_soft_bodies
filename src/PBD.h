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
    void resetSimulation(Object& obj, double ground);
    void step() const;

    [[nodiscard]] std::vector<std::shared_ptr<Particle>> getParticles() const { return so.particles; };
    void setTimeStep(const double time_step) { timeStep = time_step; };
    void setSolverIterations(const int solver_iterations) { solverIterations = solver_iterations; };
    void setDistanceStiffness(const double dist_stiffness) { so.distanceStiffness = dist_stiffness; };
    void setVolumeStiffness(const double volume_stiffness) { so.volumeStiffness = volume_stiffness; };
    void setShapeMatchingStiffness(const double shape_m_stiffness) { so.shapeMatchingStiffness = shape_m_stiffness; };
    void setOutsideForces(const Eigen::Vector3d& outside_forces) { so.outsideForces = outside_forces; };
};



#endif //PBD_H
