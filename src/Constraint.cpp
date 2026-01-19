//
// Created by hana on 12/1/25.
//

#include "Constraint.h"

#include <iostream>

Constraint::Constraint() = default;
Constraint::~Constraint() = default;

void DistanceConstraint::project() {

    Eigen::Vector3d dir = p1->p - p2->p;
    double len = dir.norm();

    if (len < 1e-8) return;

    Eigen::Vector3d n = dir / len;
    double C = len - restLength;

    double w1 = p1->w;
    double w2 = p2->w;
    double wSum = w1 + w2;

    if (wSum == 0.0) return;

    Eigen::Vector3d correction = stiffness * (C / wSum) * n;

    p1->p -= w1 * correction;
    p2->p += w2 * correction;
}

void DistanceConstraint::print() const
{
    std::cout << "DistanceConstraint: "
              << "p1 = " << p1->x.transpose()
              << ", p2 = " << p2->x.transpose()
              << ", restLength = " << restLength
              << ", stiffness = " << stiffness
              << "\n";
}

void CollisionConstraint::project() {

    double penetration = normal.dot(p->p) - offset;

    if (penetration >= 0.0)
        return;

    // Push particle out of the plane
    p->p -= penetration * normal;
}


void CollisionConstraint::print() const
{
    std::cout << "CollisionConstraint: "
              << "particle = " << p->x.transpose()
              << ", normal = " << normal.transpose()
              << ", offset = " << offset
              << "\n";
}

void ShapeMatchingConstraint::project()  {
    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (auto& p : particles) cm += p->p;
    cm /= particles.size();

    for (size_t i = 0; i < particles.size(); ++i) {
        Eigen::Vector3d goal = cm + restPositions[i];
        particles[i]->p += stiffness * (goal - particles[i]->p);
    }
}

void ShapeMatchingConstraint::print() const {

}


