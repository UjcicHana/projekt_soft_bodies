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
    // object (changes through time)
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3i> faces;
    // constraints
    std::vector<std::shared_ptr<Constraint>> constraints;
    std::vector<std::shared_ptr<CollisionConstraint>> collisionConstraints;

    double distanceStiffness = 0.7;
    double volumeStiffness = 0.8;
    std::vector<Eigen::Vector3d> restPositions;
    double shapeMatchingStiffness = 0.5;
    Eigen::Vector3d outsideForces = Eigen::Vector3d(0, -9.8, 0); // default just gravity

    void initializeFromObject(
        const Object& obj,
        double ground = 0.0,
        double mass = 1.0,
        const Eigen::Vector3d& initialVelocity = Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& initialTranslation = Eigen::Vector3d(1.0, 1.5, 0)
    );

    // build constraints
    void generateDistanceConstraints(double stiffness);
    void generateCollisionConstraints(double ground);

    // utils
    void calculateForces(const Eigen::Vector3d& outsideForces) const;

};



#endif //SIMULATEDOBJECT_H
