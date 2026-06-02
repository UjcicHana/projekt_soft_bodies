//
// Created by hana on 12/1/25.
//

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <iostream>
#include <utility>

#include "Particle.h"
#include "FEMUtils.h"

enum AlgorithmType {
    PBD,
    XPBD
};

enum ConstraintType {
    EQUALITY,
    INEQUALITY
};

class Constraint {
public:
    Constraint(const std::vector<std::shared_ptr<Particle>> &particles,
        float stiffness,
        float compliance,
        float dt,
        ConstraintType ct);;
    virtual ~Constraint();

    virtual float calculateValue() = 0;
    virtual void calculateGradient() = 0;
    virtual void print() const = 0;

    void solve(AlgorithmType algorithmType);
    bool isSatisfied();

    float stiffness; // used in PBD, [0,1]
    float lambda; // used in XPBD
    float compliance; // used in XPBD
    float dt; // used in XPBD
    ConstraintType constraintType;
protected:
    std::vector<std::shared_ptr<Particle>> particles;
    Eigen::MatrixXf gradient;
};

class DistanceConstraint final : public Constraint {
public:
    DistanceConstraint(
        const std::shared_ptr<Particle>& p0,
        const std::shared_ptr<Particle>& p1,
        float stiffness = 1.0,
        float compliance = 0.0,
        float dt = 1.0 / 120.0);

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;
private:
    float restLength;
};

class EnvironmentalCollisionConstraint final : public Constraint {
public:
    EnvironmentalCollisionConstraint(
        std::shared_ptr<Particle> particle,
        const Eigen::Vector3f& n,
        float d,
        float stiffness = 1.0,
        float compliance = 0.0,
        float dt = 1.0 / 120.0);

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;

private:
    std::shared_ptr<Particle> p;
    Eigen::Vector3f normal;
    float offset;
};

class VolumeConstraint final : public Constraint {
public:
    VolumeConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        const std::vector<Eigen::Vector3i>& fs,
        float stiffness = 1.0,
        float compliance = 0.0,
        float dt = 1.0 / 120.0);

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;

private:
    std::vector<Eigen::Vector3i> faces;
    float restVolume;

    [[nodiscard]] float computeVolume() const;
};

class FixedPointConstraint : public Constraint {
public:
    FixedPointConstraint(
        std::shared_ptr<Particle>& p,
        Eigen::Vector3f fp,
        float stiffness = 1.0,
        float compliance = 0.0,
        float dt = 1.0 / 120.0);

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;

private:
    Eigen::Vector3f fixedPoint;
};

class IsometricBendingConstraint final : public Constraint {
public:
    IsometricBendingConstraint(
        std::shared_ptr<Particle>& p0,
        std::shared_ptr<Particle>& p1,
        std::shared_ptr<Particle>& p2,
        std::shared_ptr<Particle>& p3,
        float stiffness = 1.0f,
        float compliance = 0.0f,
        float dt = 1.0f / 120.0f);

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;

private:
    Eigen::Matrix4f Q;
    float restValue;

    static float calculateCotTheta(const Eigen::Vector3f& x, const Eigen::Vector3f& y);
};

class ContinuumTriangleConstraint final : public Constraint {
public:
    ContinuumTriangleConstraint(
        std::shared_ptr<Particle>& p0,
        std::shared_ptr<Particle>& p1,
        std::shared_ptr<Particle>& p2,
        float stiffness = 1.0f,
        float compliance = 0.0f,
        float dt = 1.0f / 120.0f,
        float youngus_model = 1.0f,
        float poisson_ratio = 0.001f);

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;

private:
    float firstLame;
    float secondLame;

    float restArea;
    Eigen::Matrix2f restDInv;

    bool valid = true;
};

/*class CollisionConstraint final : public Constraint {
public:
    float h;

    CollisionConstraint(const std::shared_ptr<Particle>& q,
        const std::shared_ptr<Particle>& p1,
        const std::shared_ptr<Particle>& p2,
        const std::shared_ptr<Particle>& p3,
        float _h = 0.02f,
        float stiffness = 1.0,
        float compliance = 0.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(std::vector{q, p1, p2, p3}, stiffness, compliance, dt, INEQUALITY),
    h(_h) {}

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;
};

class BendingConstraint final : public Constraint {
public:
    float restCosPhi; // instead of phi

    BendingConstraint(std::shared_ptr<Particle>& p0,
        std::shared_ptr<Particle>& p1,
        std::shared_ptr<Particle>& p2,
        std::shared_ptr<Particle>& p3,
        float stiffness = 1.0,
        float compliance = 0.0,
        const float dt = 1.0 / 120.0)
    : Constraint(std::vector{p0, p1, p2, p3},
        stiffness, compliance, dt, EQUALITY) {
        const Eigen::Vector3f& x0 = particles[0]->p;
        const Eigen::Vector3f& x1 = particles[1]->p;
        const Eigen::Vector3f& x2 = particles[2]->p;
        const Eigen::Vector3f& x3 = particles[3]->p;

        const Eigen::Vector3f n0 = (x1 - x0).cross(x2 - x0).normalized();
        const Eigen::Vector3f n1 = (x1 - x0).cross(x3 - x0).normalized();

        restCosPhi = std::clamp(n0.dot(n1), -1.0f, 1.0f);


    }

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;
};

class TriangleConstraint final : public Constraint {
public:
    std::shared_ptr<Particle> p1;
    std::shared_ptr<Particle> p2;
    std::shared_ptr<Particle> p3;

    TriangleConstraint(
        std::shared_ptr<Particle> a,
        std::shared_ptr<Particle> b,
        std::shared_ptr<Particle> c,
        float stiffness = 1.0,
        float compliance = 1.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(std::vector<std::shared_ptr<Particle>>{}, stiffness, compliance, dt), p1(a), p2(b), p3(c) {
        Eigen::Vector3f e1 = p2->x - p1->x;

        Eigen::Vector3f e2 = p3->x - p1->x;

        restArea =
            0.5 * e1.cross(e2).norm();
    };

    void project(AlgorithmType algorithmType) override;
    void print() const override;

private:

    float restArea;
};
 */

#endif //CONSTRAINT_H
