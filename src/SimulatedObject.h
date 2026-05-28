//
// Created by hana on 1/19/26.
//

#ifndef SIMULATEDOBJECT_H
#define SIMULATEDOBJECT_H

#include "Object.h"
#include "Particle.h"
#include "Constraint.h"

struct AABB {
    Eigen::Vector3f min;
    Eigen::Vector3f max;
};

class SimulatedObject {
public:
    // object (changes through time)
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3f> initialPositions;
    std::vector<Eigen::Vector3i> faces;
    // constraints
    std::vector<std::shared_ptr<DistanceConstraint>> distanceConstraints;
    std::vector<std::shared_ptr<VolumeConstraint>> volumeConstraints;
    std::vector<std::shared_ptr<ShapeMatchingConstraint>> shapeMatchingConstraints;
    std::vector<std::shared_ptr<FixedPointConstraint>> fixedPointConstraints;
    std::vector<std::shared_ptr<GroundCollisionConstraint>> collisionConstraints;

    float timeStep = 1.0 / 120.0;
    AlgorithmType algorithmType;
    float distanceStiffness = 0.7;
    float volumeStiffness = 0.2;
    float shapeMatchingStiffness = 0.1;
    Eigen::Vector3f outsideForces = Eigen::Vector3f(0, -9.8, 0); // default just gravity
    float distanceCompliance = 5e-5;
    float volumeCompliance = 5e-6;

    void initializeFromObject(
        const Object& obj,
        float ground = 0.0,
        float mass = 1.0,
        const Eigen::Vector3f& initialVelocity = Eigen::Vector3f::Zero(),
        const Eigen::Vector3f& initialTranslation = Eigen::Vector3f(1.0f, 1.5f, 0),
        AlgorithmType at = AlgorithmType::PBD);

    void reset();

    // build constraints
    void generateDistanceConstraints(float stiffness, float compliance, float dt);
    void generateCollisionConstraints(float ground);

    // utils
    void calculateForces(const Eigen::Vector3f& outside_forces) const;
    void resetConstraints();
    void resetLambdaConstraints();
    void projectConstraints();

    AABB computeAABB() const;

};



#endif //SIMULATEDOBJECT_H
