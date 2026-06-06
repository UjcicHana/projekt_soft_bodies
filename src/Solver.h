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
    void clearObjects() {
        simObjects.clear();
    }
    void addSimulatedObject(const std::shared_ptr<SimulatedObject>& so) {
        simObjects.push_back(so);
    }
    [[nodiscard]] size_t objectCount() const {
        return simObjects.size();
    }

    [[nodiscard]] std::shared_ptr<SimulatedObject> getObject(unsigned int pos) const {
        assert(pos < simObjects.size());
        return simObjects[pos];
    }

    [[nodiscard]] const std::vector<std::shared_ptr<SimulatedObject>>& getObjects() const {
        return simObjects;
    }

    void resetSimulation();
    void step();
    //void groundCollision();

    [[nodiscard]] std::vector<std::shared_ptr<Particle>> getParticles(unsigned int pos) const {
        assert(pos < simObjects.size());
        return simObjects[pos]->particles;
    };

    [[nodiscard]] std::vector<Eigen::Vector3i> getFaces(unsigned int pos) const {
        assert(pos < simObjects.size());
        return simObjects[pos]->faces;
    }

    void setTimeStep(const float time_step) {
        timeStep = time_step;
        for (auto so : simObjects) {
            so->simulation.timeStep = time_step;
        }
    };
    void setSolverIterations(const int solver_iterations) { solverIterations = solver_iterations; };
    void setAlgorithmType(const AlgorithmType algorithm_type, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->simulation.algorithmType = algorithm_type;
    };
    void setOutsideForces(const Eigen::Vector3f& outside_forces, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->simulation.externalAcceleration = outside_forces;
    };

    // distance constraint parameters
    void setDistanceStiffness(const float dist_stiffness, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.distanceStiffness = dist_stiffness;
    };

    void setDistanceCompliance(const float distance_compliance, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.distanceCompliance = distance_compliance;
    };

    // volume constraint parameters
    void setVolumeStiffness(const float volume_stiffness, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.volumeStiffness = volume_stiffness;
    };

    void setVolumeCompliance(const float volume_compliance, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.volumeCompliance = volume_compliance;
    };

    // triangle continuum constraint parameters
    void setContinuumStiffness(const float continuum_stiffness, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.continuumStiffness = continuum_stiffness;
    };

    void setContinuumCompliance(const float continuum_compliance, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.continuumCompliance = continuum_compliance;
    };

    void setYoungsModulus(const float youngs_modulus, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.youngsModulus = youngs_modulus;
    };

    void setPoissonRatio(const float poisson_ratio, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.poissonRatio = poisson_ratio;
    };

    // bending constraint parameters
    void setBendingStiffness(const float bending_stiffness, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.bendingStiffness = bending_stiffness;
    };

    void setBendingCompliance(const float bending_compliance, unsigned int pos) {
        assert(pos < simObjects.size());
        simObjects[pos]->material.bendingCompliance = bending_compliance;
    };
};


#endif //SOLVER_H
