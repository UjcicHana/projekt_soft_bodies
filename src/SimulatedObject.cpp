//
// Created by hana on 1/19/26.
//

#include "SimulatedObject.h"

#include <iostream>
#include <set>

Eigen::Matrix3f rotationX(float angleRadians)
{
    float c = std::cos(angleRadians);
    float s = std::sin(angleRadians);

    Eigen::Matrix3f R;
    R << 1,  0, 0,
         0,  c, -s,
         0,  s,  c;
    return R;
}

void SimulatedObject::initializeFromObject(
    const Object& obj,
    float ground,
    float mass,
    const Eigen::Vector3f& initialVelocity,
    const Eigen::Vector3f& initialTranslation,
    AlgorithmType at)
{
    algorithmType = at;
    particles.clear();
    initialPositions.clear();

    float angleDeg = 0.0f;
    auto angleRad = static_cast<float>(angleDeg * M_PI / 180.0);
    Eigen::Matrix3f R = rotationX(angleRad);

    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for (auto& v : obj.getVertices())
        center += v;
    center /= obj.getVertices().size();

    for (const auto& vertex : obj.getVertices())
    {
        auto p = std::make_shared<Particle>();

        Eigen::Vector3f rotated = R * (vertex - center) + center;
        p->x = rotated + initialTranslation;
        p->p = p->x;

        p->v = initialVelocity;
        p->F.setZero();

        p->m = mass;
        p->w = (p->m > 0.0) ? 1.0f / p->m : 0;

        particles.push_back(p);
        initialPositions.push_back(p->p);
    }

    faces = obj.getFaces();

    resetConstraints();

    generateDistanceConstraints(distanceStiffness, distanceCompliance, timeStep);
    generateCollisionConstraints(ground);
    volumeConstraints.push_back(std::make_shared<VolumeConstraint>(
    particles, faces, volumeStiffness, volumeCompliance, timeStep));
    //shapeMatchingConstraints.push_back(std::make_shared<ShapeMatchingConstraint>(particles, restPositions, shapeMatchingStiffness));
    //fixedPointConstraints.push_back((std::make_shared<FixedPointConstraint>(particles[0], restPositions[0])));

    //for (const auto &c : constraints) c->print();

    calculateForces(outsideForces);
}

void SimulatedObject::reset() {
    for (int i = 0; i < particles.size(); i++) {
        particles[i]->x = initialPositions[i];
        particles[i]->p = particles[i]->x;
    }
}

void SimulatedObject::generateDistanceConstraints(float stiffness, float compliance, float dt) {

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
        distanceConstraints.push_back(
            std::make_shared<DistanceConstraint>(
                particles[e.first], particles[e.second],
                stiffness, compliance, dt));
    }
}


void SimulatedObject::generateCollisionConstraints(float ground) {
    collisionConstraints.clear();
    Eigen::Vector3f normal(0, 1, 0);

    for (const auto& p : particles) {
        collisionConstraints.push_back(
            std::make_shared<GroundCollisionConstraint>(p, normal, ground));
    }
}

void SimulatedObject::calculateForces(const Eigen::Vector3f& outside_forces) const {

    for (const auto& particle : particles)
    {
        particle->F = particle->m * outside_forces;
    }
}

void SimulatedObject::resetConstraints() {
    distanceConstraints.clear();
    volumeConstraints.clear();
    shapeMatchingConstraints.clear();
    fixedPointConstraints.clear();
    collisionConstraints.clear();
}

void SimulatedObject::resetLambdaConstraints() {
    for (const auto& c : distanceConstraints) {
        c->lambda = 0.0f;
    }
    for (const auto& c : volumeConstraints) {
        c->lambda = 0.0f;
    }
    // for (const auto& c : shapeMatchingConstraints) {
    //     c->lambda = 0.0f;
    // }
    for (const auto& c : collisionConstraints) {
        c->lambda = 0.0f;
    }
    for (const auto& c : fixedPointConstraints) {
        c->lambda = 0.0f;
    }
}

void SimulatedObject::projectConstraints() {
    for (const auto& c : distanceConstraints) {
        c->solve(algorithmType);
    }
    for (const auto& c : volumeConstraints) {
        c->solve(algorithmType);
    }
    // for (const auto& c : shapeMatchingConstraints) {
    //     c->project(algorithmType);
    // }
    for (const auto& c : collisionConstraints) {
        c->solve(algorithmType);
    }
    for (const auto& c : fixedPointConstraints) {
        c->solve(algorithmType);
    }
}

AABB SimulatedObject::computeAABB() const
{
    AABB box;

    box.min = Eigen::Vector3f(std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max());

    box.max = -box.min;

    for (const auto& p : particles)
    {
        box.min = box.min.cwiseMin(p->p);

        box.max = box.max.cwiseMax(p->p);
    }

    return box;
}
