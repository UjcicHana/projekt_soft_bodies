//
// Created by hana on 12/1/25.
//

#include "Constraint.h"

#include <iostream>

Constraint::~Constraint() = default;

void Constraint::solve(AlgorithmType algorithmType) {

    // If the constraint is already satisfied, no need to solve it
    if (isSatisfied()) return;

    //if (constraintType == ConstraintType::INEQUALITY) std::cout << "Do something\n";
    calculateGradient();
    float C = calculateValue();

    if (algorithmType == AlgorithmType::PBD) {
        float denominator = 0.0f;

        for (unsigned int i = 0; i < particles.size(); i++) {
            denominator += particles[i]->w * gradient.col(i).dot(gradient.col(i));
        }
        if(denominator > 1e-6) {
            lambda = - stiffness * (C / denominator);
        }

        lambda = std::clamp(lambda, -0.01f, 0.01f);
        for (unsigned int i = 0; i < particles.size(); i++) {
            particles[i]->p += (lambda * particles[i]->w) * gradient.col(i);
            //auto added = (lambda * particles[i]->w) * gradient.col(i);
            //std::cout << "Solve PBD = " << lambda << std::endl;
        }
    } else if (algorithmType == AlgorithmType::XPBD) {
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

        if (constraintType == ConstraintType::INEQUALITY) {
            float oldLambda = lambda;

            lambda = std::max(0.0f, lambda + deltaLambda);
            deltaLambda = lambda - oldLambda;
        } else {
            lambda += deltaLambda;
        }

        for (unsigned int i = 0; i < particles.size(); i++) {
            if(denominator < 1e-6) {
                continue;
            }
            particles[i]->p += (deltaLambda * particles[i]->w) * gradient.col(i);
            //auto added = (deltaLambda * particles[i]->w) * gradient.col(i);
            //std::cout << "Solve XPBD = " << lambda << std::endl;
        }

        /*lambda += deltaLambda;
        if (constraintType == ConstraintType::INEQUALITY) {
            lambda = std::max(0.0f, lambda);
        }*/
    }
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
    return (p0->p - p1->p).norm() - restLength;
}

void DistanceConstraint::calculateGradient() {
    auto g1 = (particles[0]->p - particles[1]->p).normalized();
    auto g2 = -g1;

    gradient.col(0) = g1;
    gradient.col(1) = g2;
}

void EnvironmentalCollisionConstraint::print() const
{
    std::cout << "CollisionConstraint: "
              << "particle = " << p->x.transpose()
              << ", normal = " << normal.transpose()
              << ", offset = " << offset
              << "\n";
}

float EnvironmentalCollisionConstraint::calculateValue() {
    return normal.dot(particles[0]->p) - offset;
}

void EnvironmentalCollisionConstraint::calculateGradient() {
    gradient.resize(3, 1);
    gradient.col(0) = normal;
}

float VolumeConstraint::computeVolume() const
{
    float volume = 0.0;

    for (const auto& f : faces) {
        const Eigen::Vector3f& x0 = particles[f.x()]->p;
        const Eigen::Vector3f& x1 = particles[f.y()]->p;
        const Eigen::Vector3f& x2 = particles[f.z()]->p;

        volume += x0.dot(x1.cross(x2));
    }

    return volume / 6.0f;
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

    float currentVolume = 0.0;

    for (const auto& f : faces) {
        const Eigen::Vector3f& x0 = particles[f.x()]->p;
        const Eigen::Vector3f& x1 = particles[f.y()]->p;
        const Eigen::Vector3f& x2 = particles[f.z()]->p;

        currentVolume += x0.dot(x1.cross(x2));
    }

    return currentVolume / 6.0f - restVolume;

}

void VolumeConstraint::calculateGradient() {
    gradient = Eigen::MatrixXf::Zero(3, static_cast<long>(particles.size()));
    for (const auto& f : faces) {
        int i0 = f.x();
        int i1 = f.y();
        int i2 = f.z();

        const Eigen::Vector3f& p0 = particles[i0]->p;
        const Eigen::Vector3f& p1 = particles[i1]->p;
        const Eigen::Vector3f& p2 = particles[i2]->p;

        Eigen::Vector3f g0 = (p1.cross(p2)) / 6.0;
        Eigen::Vector3f g1 = (p2.cross(p0)) / 6.0;
        Eigen::Vector3f g2 = (p0.cross(p1)) / 6.0;

        gradient.col(i0) += g0;
        gradient.col(i1) += g1;
        gradient.col(i2) += g2;
    }
}

void FixedPointConstraint::print() const
{
    std::cout
        << "FixedPointConstraint | fixedPosition = "
        << fixedPoint.transpose()
        << "\n";
}

float FixedPointConstraint::calculateValue() {
    return (particles[0]->p - fixedPoint).norm();
}

void FixedPointConstraint::calculateGradient() {
    gradient.col(0) = (particles[0]->p - fixedPoint).normalized();
}

void CollisionConstraint::print() const
{
    std::cout << "CollisionConstraint: "
    << "q = " << particles[0]->x.transpose()
    << ", p1 = " << particles[1]->x.transpose()
    << ", p2 = " << particles[2]->x.transpose()
    << ", p3 = " << particles[3]->x.transpose()
    << ", h = " << h
    << "\n";
}

float CollisionConstraint::calculateValue() {
    const auto& q  = particles[0]->p;
    const auto& p1 = particles[1]->p;
    const auto& p2 = particles[2]->p;
    const auto& p3 = particles[3]->p;

    Eigen::Vector3f e1 = p2 - p1;
    Eigen::Vector3f e2 = p3 - p1;

    Eigen::Vector3f n = e1.cross(e2);

    float len = n.norm();

    if (len < 1e-6f)
        return 1.0f;

    n /= len;

    // Project point onto plane
    Eigen::Vector3f projected =
        q - n * (q - p1).dot(n);

    // Barycentric coordinates
    Eigen::Vector3f v2 = projected - p1;

    float d00 = e1.dot(e1);
    float d01 = e1.dot(e2);
    float d11 = e2.dot(e2);
    float d20 = v2.dot(e1);
    float d21 = v2.dot(e2);

    float denom = d00 * d11 - d01 * d01;

    if (std::abs(denom) < 1e-8f)
        return 1.0f;

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    // Outside triangle -> no collision
    if (u < 0.0f || v < 0.0f || w < 0.0f)
        return 1.0f;

    // Signed penetration depth
    auto C = (q - p1).dot(n) - h;
    //std::cout << "n= " << n.transpose() << ", C = " << C << std::endl;
    return C;
}

void CollisionConstraint::calculateGradient() {
    const auto& q  = particles[0]->p;
    const auto& p1 = particles[1]->p;
    const auto& p2 = particles[2]->p;
    const auto& p3 = particles[3]->p;

    Eigen::Vector3f e1 = p2 - p1;
    Eigen::Vector3f e2 = p3 - p1;

    Eigen::Vector3f n = e1.cross(e2);

    float len = n.norm();

    if (len < 1e-6f) {
        gradient.setZero();
        return;
    }

    n /= len;

    // Project q onto triangle plane
    Eigen::Vector3f projected =
        q - n * (q - p1).dot(n);

    Eigen::Vector3f qp = projected - p1;

    // Compute barycentrics
    float d00 = e1.dot(e1);
    float d01 = e1.dot(e2);
    float d11 = e2.dot(e2);
    float d20 = qp.dot(e1);
    float d21 = qp.dot(e2);

    float denom = d00 * d11 - d01 * d01;

    if (std::abs(denom) < 1e-8f) {
        gradient.setZero();
        return;
    }

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    // Outside triangle -> disable constraint
    if (u < 0.0f || v < 0.0f || w < 0.0f) {
        gradient.setZero();
        return;
    }

    // Gradients
    gradient.col(0) = n;
    gradient.col(1) = -u * n;
    gradient.col(2) = -v * n;
    gradient.col(3) = -w * n;

    //std::cout << gradient << std::endl;
}

float IsometricBendingConstraint::calculateCotTheta(const Eigen::Vector3f& x, const Eigen::Vector3f& y)
{
    float denom = x.cross(y).norm();

    if (denom < 1e-8f)
        return 0.0f;

    return x.dot(y) / denom;
}

void IsometricBendingConstraint::print() const {
    std::cout << "BendingConstraint: "
    << "p0 = " << particles[0]->x.transpose()
    << ", p1 = " << particles[1]->x.transpose()
    << ", p2 = " << particles[2]->x.transpose()
    << ", p3 = " << particles[3]->x.transpose()
    << ", Q = " << Q
    << "\n";
}

float IsometricBendingConstraint::computeEnergy() const {
    float sum = 0.0f;

    for (unsigned int i = 0; i < 4; ++i) {
        for (unsigned int j = 0; j < 4; ++j) {
            sum += Q(i, j) *
                particles[i]->p.dot(particles[j]->p);
        }
    }

    return 0.5f * sum;
}

float IsometricBendingConstraint::calculateValue() {
    return computeEnergy() - restValue;
}

void IsometricBendingConstraint::calculateGradient() {
    gradient.setZero();

    for (unsigned int i = 0; i < 4; ++i) {
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for (unsigned int j = 0; j < 4; ++j) {
            sum += Q(i, j) * particles[j]->p;
        }
        gradient.col(i) = sum;
    }
}

void BendingConstraint::print() const {
    std::cout << "BendingConstraint: "
    << "p0 = " << particles[0]->x.transpose()
    << ", p1 = " << particles[1]->x.transpose()
    << ", p2 = " << particles[2]->x.transpose()
    << ", p3 = " << particles[3]->x.transpose()
    << ", phi = " << restCosPhi
    << "\n";
}

float BendingConstraint::calculateValue() {
    const Eigen::Vector3f& x0 = particles[0]->p;
    const Eigen::Vector3f& x1 = particles[1]->p;
    const Eigen::Vector3f& x2 = particles[2]->p;
    const Eigen::Vector3f& x3 = particles[3]->p;

    const Eigen::Vector3f n0 = (x1 - x0).cross(x2 - x0).normalized();
    const Eigen::Vector3f n1 = (x1 - x0).cross(x3 - x0).normalized();

    float cosPhi = std::clamp(n0.dot(n1), -1.0f, 1.0f);
    std::cout << "restCosPhi: " << restCosPhi << " new cosPhi: " << cosPhi << std::endl;

    return cosPhi - restCosPhi;
}

void BendingConstraint::calculateGradient() {
    Eigen::Vector3f x0 = particles[0]->p;
    Eigen::Vector3f x1 = particles[1]->p;
    Eigen::Vector3f x2 = particles[2]->p;
    Eigen::Vector3f x3 = particles[3]->p;

    Eigen::Vector3f el = x2 - x0;
    Eigen::Vector3f em = x1 - x0;
    Eigen::Vector3f er = x3 - x0;

    // Compute n1 = p1 x p2, n2 = p1 x p3 and cosPhi = n1.n2
    Eigen::Vector3f n0 = (x1 - x0).cross(x2 - x0);
    Eigen::Vector3f n1 = (x1 - x0).cross(x3 - x0);

    auto l1 = n0.norm();
    auto l2 = n1.norm();

    // Check if n1 or n2 are null vectors (i.e. the triangle is degenerate)
    if (l1 < 1e-8f || l2 < 1e-8f) return;

    // Normalize n1 and n2
    n0 /= l1;
    n1 /= l2;

    float cosPhi = n0.dot(n1);
    if (cosPhi > 1.0f) cosPhi = 1.0f;
    if (cosPhi < -1.0f) cosPhi = -1.0f;

    float cosPhi2 = cosPhi * cosPhi;

    if (1.0f - cosPhi2 < 1e-6f) return;

    float arcosDerivative = -1.0f / sqrtf(1.0f - cosPhi2);
    if((n0).cross(n1).dot(em) < 0.0f)
        arcosDerivative = -arcosDerivative;

    Eigen::Vector3f dp1 = (er.cross(n0) + cosPhi * n1.cross(er)) / l2
                    + (el.cross(n1) + cosPhi * n0.cross(el)) / l1;
    Eigen::Vector3f dp2 = (n1.cross(em) - cosPhi * n0.cross(em)) / l1;
    Eigen::Vector3f dp3 = (n0.cross(em) - cosPhi * n1.cross(em)) / l2;

    //std::cout << "DERIVATIVES: " << toString(dp1) << toString(dp2) << toString(dp3) << std::endl;

    Eigen::Vector3f grad1 = arcosDerivative * dp1;
    Eigen::Vector3f grad2 = arcosDerivative * dp2;
    Eigen::Vector3f grad3 = arcosDerivative * dp3;
    Eigen::Vector3f grad0 = -grad1 - grad2 - grad3;

    gradient.col(0) = grad0;
    gradient.col(1) = grad1;
    gradient.col(2) = grad2;
    gradient.col(3) = grad3;

    std::cout
    << grad0.norm() << " "
    << grad1.norm() << " "
    << grad2.norm() << " "
    << grad3.norm() << "\n";

}




