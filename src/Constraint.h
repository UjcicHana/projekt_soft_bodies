//
// Created by hana on 12/1/25.
//

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <iostream>
#include <utility>

#include "Particle.h"

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
        const float stiffness,
        const float compliance,
        const float dt,
        ConstraintType ct)
    : particles(particles), stiffness(stiffness), lambda(0.0),
    compliance(compliance), dt(dt), constraintType(ct) {
        cardinality = particles.size();
        gradient = Eigen::MatrixXf::Zero(3, cardinality);
    };
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
    unsigned int cardinality;
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
        const float dt = 1.0 / 120.0
    ) : Constraint(std::vector{p0, p1},
        stiffness, compliance, dt, EQUALITY)
    {
        restLength = (p0->x - p1->x).norm();
    }

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;
private:
    float restLength;
};

class EnvironmentalCollisionConstraint final : public Constraint {
public:
    std::shared_ptr<Particle> p;
    Eigen::Vector3f normal;
    float offset;

    EnvironmentalCollisionConstraint(
        std::shared_ptr<Particle> particle,
        const Eigen::Vector3f& n,
        float d,
        float stiffness = 1.0,
        float compliance = 0.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(std::vector{std::move(particle)}, stiffness, compliance, dt, INEQUALITY),
    normal(n.normalized()), offset(d) {}

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;
};

class VolumeConstraint final : public Constraint {
public:
    std::vector<Eigen::Vector3i> faces;
    float restVolume;

    VolumeConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        const std::vector<Eigen::Vector3i>& fs,
        float stiffness = 1.0,
        float compliance = 0.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(ps, stiffness, compliance, dt, EQUALITY), faces(fs)
    {
        restVolume = computeVolume();
    }

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;

private:
    [[nodiscard]] float computeVolume() const;
};

class FixedPointConstraint : public Constraint {
public:
    Eigen::Vector3f fixedPoint;

    FixedPointConstraint(
        std::shared_ptr<Particle>& p,
        Eigen::Vector3f fp,
        float stiffness = 1.0,
        float compliance = 0.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(std::vector{p}, stiffness,
        compliance, dt, EQUALITY), fixedPoint(std::move(fp)) {
        p->m = 0.0;
        p->w = 0.0;
    }

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;

};

class CollisionConstraint final : public Constraint {
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

class IsometricBendingConstraint final : public Constraint {
public:
    Eigen::Matrix4f Q;
    float restValue;

    IsometricBendingConstraint(
        std::shared_ptr<Particle>& p0,
        std::shared_ptr<Particle>& p1,
        std::shared_ptr<Particle>& p2,
        std::shared_ptr<Particle>& p3,
        float stiffness = 1.0f,
        float compliance = 0.0f,
        float dt = 1.0f / 120.0f
    ) : Constraint(std::vector{p0,p1,p2,p3}, stiffness, compliance, dt, EQUALITY)
    {
        Q.setZero();

        const Eigen::Vector3f& x0 = p0->x;
        const Eigen::Vector3f& x1 = p1->x;
        const Eigen::Vector3f& x2 = p2->x;
        const Eigen::Vector3f& x3 = p3->x;

        const Eigen::Vector3f e0 = x1 - x0;
        const Eigen::Vector3f e1 = x2 - x1;
        const Eigen::Vector3f e2 = x0 - x2;
        const Eigen::Vector3f e3 = x3 - x0;
        const Eigen::Vector3f e4 = x1 - x3;

        const float cot_01 = calculateCotTheta(e0, -e1);
        const float cot_02 = calculateCotTheta(e0, -e2);
        const float cot_03 = calculateCotTheta(e0, e3);
        const float cot_04 = calculateCotTheta(e0, e4);

        const Eigen::Vector4f K = Eigen::Vector4f(cot_01 + cot_04, cot_02 + cot_03, -cot_01 - cot_02, -cot_03 - cot_04);

        const auto A_0 = 0.5f * e0.cross(e1).norm();
        const auto A_1 = 0.5f * e0.cross(e3).norm();

        float area = A_0 + A_1;

        if (area < 1e-8f || !std::isfinite(area)) {
            Q.setZero();
            return;
        }

        Q = (3.0f / (2.0f * area)) * K * K.transpose();

        if (!Q.allFinite()) {
            Q.setZero();
        }
        float sum = 0.0f;

        for (unsigned int i = 0; i < 4; ++i) {
            for (unsigned int j = 0; j < 4; ++j) {
                sum += Q(i, j) *
                    particles[i]->x.dot(particles[j]->x);
            }
        }

        restValue = 0.5f * sum;
    }

    float calculateValue() override;
    void calculateGradient() override;
    void print() const override;
private:
    float computeEnergy() const;
    static float calculateCotTheta(const Eigen::Vector3f& x, const Eigen::Vector3f& y);
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

/*class TriangleConstraint final : public Constraint {
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

class ContinuumTriangleConstraint : public Constraint {
public:
    std::shared_ptr<Particle> p1;
    std::shared_ptr<Particle> p2;
    std::shared_ptr<Particle> p3;

    ContinuumTriangleConstraint(
        const std::shared_ptr<Particle> a,
        const std::shared_ptr<Particle> b,
        const std::shared_ptr<Particle> c,
        float youngs_modulus,
        float poisson_ratio,
        float stiffness = 1.0,
        float compliance = 1.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(stiffness, compliance, dt), p1(a), p2(b), p3(c) {}
};

class FixedPointConstraint : public Constraint {
    public:
    FixedPointConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        float stiffness = 1.0,
        float compliance = 1.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(stiffness, compliance, dt) {}
}; */

#endif //CONSTRAINT_H
