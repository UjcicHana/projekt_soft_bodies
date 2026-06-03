//
// Created by hana on 1/19/26.
//

#ifndef SIMULATEDOBJECT_H
#define SIMULATEDOBJECT_H

#include "Object.h"
#include "Particle.h"
#include "Constraint.h"

class SimulatedObject {
public:
    struct SimulationSettings {
        float timeStep = 1.0f / 120.0f;
        AlgorithmType algorithmType = AlgorithmType::PBD;
        Eigen::Vector3f externalAcceleration = Eigen::Vector3f(0.0f, -9.8f, 0.0f);
    };

    struct MaterialSettings {
        float distanceStiffness = 0.7f;
        float distanceCompliance = 5e-5f;

        float volumeStiffness = 0.2f;
        float volumeCompliance = 5e-6f;

        float continuumStiffness = 0.01f;
        float continuumCompliance = 1e-4f;
        float youngsModulus = 100.0f;
        float poissonRatio = 0.3f;

        float bendingStiffness = 0.01f;
        float bendingCompliance = 1e-4f;
    };

    struct ConstraintConfig {
        bool useDistance = true;
        bool useVolume = false;
        bool useFixedPoints = false;
        bool useBending = false;
        bool useContinuumTriangle = false;
        bool useShapeMatching = false;
        bool useGroundCollision = true;
    };

    // object (changes through time)
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3f> initialPositions;
    std::vector<Eigen::Vector3i> faces;
    // constraints
    std::vector<std::shared_ptr<DistanceConstraint>> distanceConstraints;
    std::vector<std::shared_ptr<VolumeConstraint>> volumeConstraints;
    std::vector<std::shared_ptr<FixedPointConstraint>> fixedPointConstraints;
    std::vector<std::shared_ptr<EnvironmentalCollisionConstraint>> collisionConstraints;
    std::vector<std::shared_ptr<IsometricBendingConstraint>> isometricBendingConstraints;
    std::vector<std::shared_ptr<ContinuumTriangleConstraint>> continuumTriangleConstraints;

    SimulationSettings simulation;
    MaterialSettings material;
    ConstraintConfig config;

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
    void generateBendingConstraints(float stiffness, float compliance, float dt);
    void generateCollisionConstraints(float ground);
    void generateClothFixedPointConstraints();
    void generateContinuumTriangleConstraints(float stiffness, float compliance, float dt,
        float youngsModulus, float poissonRatio);

    // utils
    void calculateForces(const Eigen::Vector3f& externalAcceleration) const;
    void resetConstraints();
    void resetLambdaConstraints();
    void projectConstraints();

private:
    void clearState();
    void createParticlesFromObject(
        const Object& obj,
        float mass,
        const Eigen::Vector3f& initialVelocity,
        const Eigen::Vector3f& initialTranslation
    );

    void buildConstraints(float ground);
};



#endif //SIMULATEDOBJECT_H
