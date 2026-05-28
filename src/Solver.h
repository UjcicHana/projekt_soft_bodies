//
// Created by hana on 5/7/26.
//

#ifndef SOLVER_H
#define SOLVER_H

#include "SimulatedObject.h"

class Solver {
    float timeStep;
    int solverIterations = 10;
    Eigen::Vector3f ground;
    std::vector<std::shared_ptr<SimulatedObject>> simObjects;
public:
    explicit Solver(int iterations, float time_step) : solverIterations(iterations), timeStep(time_step) {};
    void addSoftBody(Object &obj, float ground) {
        std::shared_ptr<SimulatedObject> so = std::make_shared<SimulatedObject>();
        so->initializeFromObject(obj, ground);
        simObjects.push_back(so);
    }
    void resetSimulation();
    void step();
    //void groundCollision();

    [[nodiscard]] std::vector<std::shared_ptr<Particle>> getParticles(unsigned int pos) const {
        assert(pos < simObjects.size());
        return simObjects[pos]->particles;
    };
    void setTimeStep(const float time_step) {
        timeStep = time_step;
        for (auto so : simObjects) {
            so->timeStep = time_step;
        }
    };
    void setSolverIterations(const int solver_iterations) { solverIterations = solver_iterations; };
    void setAlgorithmType(const AlgorithmType algorithm_type, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->algorithmType = algorithm_type;
    };

    void setDistanceStiffness(const float dist_stiffness, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->distanceStiffness = dist_stiffness;
    };
    void setVolumeStiffness(const float volume_stiffness, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->volumeStiffness = volume_stiffness;
    };
    void setShapeMatchingStiffness(const float shape_m_stiffness, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->shapeMatchingStiffness = shape_m_stiffness;
    };

    void setDistanceCompliance (const float distance_compliance, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->distanceCompliance = distance_compliance;
    };
    void setVolumeCompliance (const float volume_compliance, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->volumeCompliance = volume_compliance;
    };

    void setOutsideForces(const Eigen::Vector3f& outside_forces, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->outsideForces = outside_forces;
    };
};


#endif //SOLVER_H
