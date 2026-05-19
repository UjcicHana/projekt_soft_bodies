//
// Created by hana on 12/1/25.
//

#include "Constraint.h"

#include <iostream>

Constraint::~Constraint() = default;

void Constraint::solve(AlgorithmType algorithmType) {
    // If the constraint is already satisfied, no need to solve it
    if (isSatisfied()) return;

    calculateGradient();
    float C = calculateValue();

    // compute delta lambda
    float alphaTilde = compliance / (dt * dt);
    float numerator = -C - alphaTilde * lambda;
    float denominator = alphaTilde;

    for (unsigned int i = 0; i < particles.size(); i++) {
        denominator += particles[i]->w * gradient.col(i).dot(gradient.col(i));
    }

    float deltaLambda = 0.0;
    if (denominator < 1e-6) {
        deltaLambda = 0.0;
    } else {
        deltaLambda = numerator / denominator;
    }

    if(std::isnan(deltaLambda)) {
        std::cout << "deltaLambda is NAN" << std::endl;
        deltaLambda = 0.0;
    }

    for (unsigned int i = 0; i < particles.size(); i++) {
        if(denominator < 1e-6) {
            continue;
        }
        particles[i]->p += (deltaLambda * particles[i]->w) * gradient.col(i);
    }

    lambda += deltaLambda;
}

bool Constraint::isSatisfied() {
    switch (constraintType) {
        case INEQUALITY:
            // >= 0
            return calculateValue() >= 0;
        case EQUALITY:
            // == 0
            return fabsf(calculateValue()) <= 1e-6;
    }

    throw std::runtime_error("Constraint type not handled");
}

void DistanceConstraint::project(AlgorithmType algorithmType) {
    solve(algorithmType);
    /*auto& p0 = particles[0];
    auto& p1 = particles[1];

    Eigen::Vector3d dir = p0->p - p1->p;
    double len = dir.norm();

    if (len < 1e-8) return;

    Eigen::Vector3d n = dir / len;
    double C = len - restLength;

    double w1 = p0->w;
    double w2 = p1->w;
    double wSum = w1 + w2;

    if (wSum == 0.0) return;
    Eigen::Vector3d correction;

    switch (algorithmType) {
        case AlgorithmType::PBD:
            correction = stiffness * (C / wSum) * n;

            p0->p -= w1 * correction;
            p1->p += w2 * correction;

            break;
        case AlgorithmType::XPBD:
            //std::cout << "compliance: " << compliance << " dt: " << dt << "\n";
            double alphaTilde = compliance / (dt * dt);
            // std::cout << "C: " << C << " alphaTilde: " << alphaTilde << " lambda: " << lambda << " wSum: " << wSum << "\n";
            double denom = wSum + alphaTilde;
            if (std::abs(denom) < 1e-9)
                break;
            double deltaLambda = (-C - alphaTilde * lambda) / denom;

            lambda += deltaLambda;

            p0->p += w1 * deltaLambda * n;
            p1->p -= w2 * deltaLambda * n;
            break;
    }*/
}

void DistanceConstraint::print() const
{
    std::cout << "DistanceConstraint: "
              << "p1 = " << particles[0]->x.transpose()
              << ", p2 = " << particles[1]->x.transpose()
              << ", restLength = " << restLength
              << ", stiffness = " << stiffness
              << ", compliance = " << compliance
              << ", lambda = " << lambda
              << ", deltaTime = " << dt
              << "\n";
}

float DistanceConstraint::calculateValue() {
    auto& p0 = particles[0];
    auto& p1 = particles[1];
    return static_cast<float>((p0->p - p1->p).norm() - restLength);
}

void DistanceConstraint::calculateGradient() {
    auto g1 = (particles[0]->p - particles[1]->p).normalized();
    auto g2 = -g1;

    gradient.col(0) = g1;
    gradient.col(1) = g2;
}

void CollisionConstraint::project(AlgorithmType algorithmType) {

    double penetration = normal.dot(particles[0]->p) - offset;

    if (penetration >= 0.0)
        return;

    // Push particle out of the plane
    particles[0]->p -= penetration * normal;
}


void CollisionConstraint::print() const
{
    std::cout << "CollisionConstraint: "
              << "particle = " << p->x.transpose()
              << ", normal = " << normal.transpose()
              << ", offset = " << offset
              << "\n";
}

float CollisionConstraint::calculateValue() {
    /*auto oldQ = particles[0]->x;
    auto q = particles[0]->p;
    auto p1 = particles[1]->p;
    auto p2 = particles[2]->p;
    auto p3 = particles[3]->p;

    auto n = (p2 -p1).cross(p3 -p1);
    // if triangle is degenerate, normalization would result in NaN values
    if (n.norm() < 1e-3f) {
        return 0.0f;
    }
    n.normalize();

    return static_cast<float>((q - p1).dot(n) - h);*/
    return 0;
}

void CollisionConstraint::calculateGradient() {
    /*auto q = particles[0]->p;
    auto p1 = particles[1]->p;
    auto p2 = particles[2]->p;
    auto p3 = particles[3]->p;

    auto e1 = p2 - p1;
    auto e2 = p3 - p1;

    auto n = e1.cross(e2);

    double area2 = n.norm();

    if (area2 < 1e-6f) {
        gradient = Eigen::Matrix3Xd::Zero(3, 4);
        return;
    }

    n.normalize();

    // Compute barycentric coordinates of projected point
    auto qp = q - p1;

    auto d00 = e1.dot(e1);
    auto d01 = e1.dot(e2);
    auto d11 = e2.dot(e2);
    auto d20 = qp.dot(e1);
    auto d21 = qp.dot(e2);

    auto denom = d00 * d11 - d01 * d01;

    auto v = (d11 * d20 - d01 * d21) / denom;
    auto w = (d00 * d21 - d01 * d20) / denom;
    auto u = 1.0f - v - w;

    // Gradients
    gradient.col(0) = n;          // point q
    gradient.col(1) = -u * n;     // p1
    gradient.col(2) = -v * n;     // p2
    gradient.col(3) = -w * n;     // p3*/
}

void ShapeMatchingConstraint::project(AlgorithmType algorithmType) {
    if (particles.empty()) return;

    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (const auto& p : particles)
        cm += p->p;
    cm = cm / particles.size();

    Eigen::Vector3d cmRest = Eigen::Vector3d::Zero();
    for (const auto& r : restPositions)
        cmRest += r;
    cmRest = cmRest / restPositions.size();

    double alpha = 1.0;

    if (algorithmType == AlgorithmType::XPBD) {
        alpha = 1.0 / (1.0 + compliance / (dt * dt));
    }

    for (size_t i = 0; i < particles.size(); ++i) {
        Eigen::Vector3d goal =
            cm + (restPositions[i] - cmRest);

        particles[i]->p += alpha * stiffness * (goal - particles[i]->p);
    }
}

void ShapeMatchingConstraint::print() const {
        std::cout << "ShapeMatchingConstraint: "
                  << particles.size()
                  << " particles, stiffness = "
                  << stiffness << "\n";
    }

float ShapeMatchingConstraint::calculateValue() {
    return 0;
}

void ShapeMatchingConstraint::calculateGradient() {

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

void VolumeConstraint::project(AlgorithmType algorithmType)
{
    double currentVolume = computeVolume();
    double C = currentVolume - restVolume;

    if (std::abs(C) < 1e-6)
        return;

    std::vector<Eigen::Vector3d> gradients(particles.size(),
                                           Eigen::Vector3d::Zero());

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

    double deltaLambda;
    switch (algorithmType) {
        case AlgorithmType::PBD:
            deltaLambda = -C / denom;

            for (size_t i = 0; i < particles.size(); ++i) {
                particles[i]->p += stiffness * particles[i]->w * deltaLambda * gradients[i];
            }

            break;
        case AlgorithmType::XPBD:
            double alphaTilde =
            compliance / (dt * dt);

            deltaLambda = (-C - alphaTilde * lambda)
                / (denom + alphaTilde);

            lambda += deltaLambda;

            for (size_t i = 0; i < particles.size(); ++i) {
                particles[i]->p += particles[i]->w * deltaLambda * gradients[i];
            }
            break;
    }

}

void VolumeConstraint::print() const
{
    std::cout << "VolumeConstraint | restVolume = " << restVolume
    << ", stiffness = " << stiffness
    << ", compliance = " << compliance
    << ", lambda = " << lambda
    << ", deltaTime = " << dt
              << "\n";
}

float VolumeConstraint::calculateValue() {
    return 0;
}

void VolumeConstraint::calculateGradient() {

}

void FixedPointConstraint::project(
    AlgorithmType algorithmType
)
{
    particles[0]->p = fixedPoint;
}

void FixedPointConstraint::print() const
{
    std::cout
        << "FixedPointConstraint | fixedPosition = "
        << fixedPoint.transpose()
        << "\n";
}

float FixedPointConstraint::calculateValue() {
    return 0;
}

void FixedPointConstraint::calculateGradient() {

}



