//
// Created by hana on 1/19/26.
//

#ifndef SIMULATEDOBJECT_H
#define SIMULATEDOBJECT_H

#include "Object.h"
#include "Particle.h"
#include "Constraint.h"
#include <set>



class SimulatedObject {
public:
    // object
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3i> faces;
    // constraints
    std::vector<std::shared_ptr<DistanceConstraint>> distanceConstraints;
    std::vector<std::shared_ptr<CollisionConstraint>> collisionConstraints;
    // shape matching (should be replaced with volume constraints)
    std::vector<Eigen::Vector3d> restPositions;
    double shapeMatchingStiffness = 0.5;

    void initializeFromObject(
        const Object& obj,
        double mass = 1.0,
        const Eigen::Vector3d& initialVelocity = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& initialTranslation = Eigen::Vector3d(0, 1.5, 0)
    );

    // build constraints
    void generateDistanceConstraints(double stiffness);
    void generateCollisionConstraints();

    // utils
    void calculateForces(const Eigen::Vector3d& gravity = Eigen::Vector3d(0, -9.8, 0));
    void projectShapeMatching();

};



#endif //SIMULATEDOBJECT_H
