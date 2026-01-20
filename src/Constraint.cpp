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

double VolumeConstraint::computeVolume() const
{
    double volume = 0.0;

    for (const auto& f : faces) {
        const Eigen::Vector3d& x0 = particles[f.x()]->p;
        const Eigen::Vector3d& x1 = particles[f.y()]->p;
        const Eigen::Vector3d& x2 = particles[f.z()]->p;

        volume += x0.dot(x1.cross(x2));
    }

    return volume / 6.0;
}

void VolumeConstraint::project()
{
    double currentVolume = computeVolume();
    double C = currentVolume - restVolume;

    if (std::abs(C) < 1e-6)
        return;

    std::vector<Eigen::Vector3d> gradients(particles.size(),
                                           Eigen::Vector3d::Zero());

    // Compute gradients
    for (const auto& f : faces) {
        int i0 = f.x();
        int i1 = f.y();
        int i2 = f.z();

        const Eigen::Vector3d& p0 = particles[i0]->p;
        const Eigen::Vector3d& p1 = particles[i1]->p;
        const Eigen::Vector3d& p2 = particles[i2]->p;

        Eigen::Vector3d g0 = (p1.cross(p2)) / 6.0;
        Eigen::Vector3d g1 = (p2.cross(p0)) / 6.0;
        Eigen::Vector3d g2 = (p0.cross(p1)) / 6.0;

        gradients[i0] += g0;
        gradients[i1] += g1;
        gradients[i2] += g2;
    }

    double denom = 0.0;
    for (size_t i = 0; i < particles.size(); ++i) {
        denom += particles[i]->w * gradients[i].squaredNorm();
    }

    if (denom < 1e-9)
        return;

    double lambda = -C / denom;

    // Apply position corrections
    for (size_t i = 0; i < particles.size(); ++i) {
        particles[i]->p += stiffness * particles[i]->w * lambda * gradients[i];
    }
}

void VolumeConstraint::print() const
{
    std::cout << "VolumeConstraint | restVolume = "
              << restVolume
              << ", stiffness = "
              << stiffness
              << "\n";
}



