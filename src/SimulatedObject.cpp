//
// Created by hana on 1/19/26.
//

#include "SimulatedObject.h"

void SimulatedObject::initializeFromObject(
    const Object& obj,
    double mass,
    const Eigen::Vector3d& initialVelocity,
    const Eigen::Vector3d& initialTranslation)
{
    particles.clear();
    restPositions.clear();

    for (const auto& vertex : obj.getVertices())
    {
        auto p = std::make_shared<Particle>();

        p->x = vertex + initialTranslation;
        p->p = p->x;

        p->v = initialVelocity;
        p->f.setZero();

        p->m = mass;
        p->w = (p->m > 0.0) ? 1.0 / p->m : 0.0;

        particles.push_back(p);
        restPositions.push_back(p->p);
    }

    faces = obj.getFaces();

    generateDistanceConstraints(0.7);
    generateCollisionConstraints();
}

void SimulatedObject::generateDistanceConstraints(double stiffness) {

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

    distanceConstraints.clear();

    for (auto& e : edges) {
        distanceConstraints.push_back(
            std::make_shared<DistanceConstraint>(
                particles[e.first],
                particles[e.second],
                stiffness
            )
        );
    }

    /*std::cout << "Distance constraints:\n";

    int i = 0;
    for (const auto& c : distanceConstraints) {
        std::cout << "  [" << i++ << "] ";
        c->print();
    }

    std::cout << "Total distance constraints: "
              << distanceConstraints.size()
              << "\n";*/
}


void SimulatedObject::generateCollisionConstraints() {
    collisionConstraints.clear();
    Eigen::Vector3d normal(0, 1, 0);

    for (const auto& p : particles) {
        collisionConstraints.push_back(
            std::make_shared<CollisionConstraint>(
                p, normal, 0.0
            )
        );
    }

    if (collisionConstraints.empty()) return;

    /*std::cout << "Collision constraints:\n";

    int i = 0;
    for (const auto& c : collisionConstraints) {
        std::cout << "  [" << i++ << "] ";
        c->print();
    }

    std::cout << "Total collision constraints: "
              << collisionConstraints.size()
              << "\n";*/
}

void SimulatedObject::projectShapeMatching()
{
    // Compute center of mass of current positions
    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (auto& p : particles)
        cm += p->p;
    cm /= particles.size();

    // Compute center of mass of rest positions
    Eigen::Vector3d cmRest = Eigen::Vector3d::Zero();
    for (auto& r : restPositions)
        cmRest += r;
    cmRest /= restPositions.size();

    // Compute goal positions (rigid transform approximation)
    for (size_t i = 0; i < particles.size(); ++i)
    {
        Eigen::Vector3d goal = particles[i]->p + shapeMatchingStiffness * (restPositions[i] + (cm - cmRest) - particles[i]->p);
        particles[i]->p = goal;
    }
}

void SimulatedObject::calculateForces(const Eigen::Vector3d& gravity) {
    // in this case just gravity

    for (auto particle : particles)
    {
        particle->f = particle->m * gravity;
    }
}
