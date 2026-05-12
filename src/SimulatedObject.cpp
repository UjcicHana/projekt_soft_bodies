//
// Created by hana on 1/19/26.
//

#include "SimulatedObject.h"
#include <set>

Eigen::Matrix3d rotationX(double angleRadians)
{
    double c = std::cos(angleRadians);
    double s = std::sin(angleRadians);

    Eigen::Matrix3d R;
    R << 1,  0, 0,
         0,  c, -s,
         0,  s,  c;
    return R;
}

void SimulatedObject::initializeFromObject(
    const Object& obj,
    double ground,
    double mass,
    const Eigen::Vector3d& initialVelocity,
    const Eigen::Vector3d& initialTranslation,
    AlgorithmType at)
{
    algorithmType = at;
    particles.clear();
    restPositions.clear();

    double angleDeg = 0.0;
    double angleRad = angleDeg * M_PI / 180.0;
    Eigen::Matrix3d R = rotationX(angleRad);

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (auto& v : obj.getVertices())
        center += v;
    center /= obj.getVertices().size();

    for (const auto& vertex : obj.getVertices())
    {
        auto p = std::make_shared<Particle>();

        Eigen::Vector3d rotated = R * (vertex - center) + center;
        p->x = rotated + initialTranslation;
        p->p = p->x;

        p->v = initialVelocity;
        p->F.setZero();

        p->m = mass;
        p->w = (p->m > 0.0) ? 1.0 / p->m : 0.0;

        particles.push_back(p);
        restPositions.push_back(p->p);
    }

    faces = obj.getFaces();

    constraints.clear();

    generateDistanceConstraints(distanceStiffness, distanceCompliance, timeStep);
    generateCollisionConstraints(ground);
    constraints.push_back(std::make_shared<VolumeConstraint>(
    particles, faces, volumeStiffness, volumeCompliance, timeStep));
    constraints.push_back(std::make_shared<ShapeMatchingConstraint>(
    particles, restPositions, shapeMatchingStiffness));

    //for (const auto &c : constraints) c->print();

    calculateForces(outsideForces);
}

void SimulatedObject::generateDistanceConstraints(double stiffness, double compliance, double dt) {

    std::set<std::pair<int,int>> edges;

    for (auto& f : faces) {
        int ids[3] = { f.x(), f.y(), f.z() };

        for (int i = 0; i < 3; ++i) {
            int a = ids[i];
            int b = ids[(i + 1) % 3];
            if (a > b) std::swap(a, b);
            edges.insert({a, b});
        }
    }

    for (auto& e : edges) {
        constraints.push_back(
            std::make_shared<DistanceConstraint>(
                particles[e.first], particles[e.second],
                stiffness, compliance, dt));
    }
}


void SimulatedObject::generateCollisionConstraints(double ground) {
    collisionConstraints.clear();
    Eigen::Vector3d normal(0, 1, 0);

    for (const auto& p : particles) {
        collisionConstraints.push_back(
            std::make_shared<CollisionConstraint>(
                p, normal, ground
            )
        );
    }
}

void SimulatedObject::calculateForces(const Eigen::Vector3d& outside_forces) const {

    for (const auto& particle : particles)
    {
        particle->F = particle->m * outside_forces;
    }
}
