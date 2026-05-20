//
// Created by hana on 12/1/25.
//

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

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
        const double stiffness,
        const double compliance,
        const double dt,
        ConstraintType ct)
    : particles(particles), stiffness(stiffness), lambda(0.0),
    compliance(compliance), dt(dt), constraintType(ct) {
        cardinality = particles.size();
        gradient = Eigen::MatrixXf::Zero(3, cardinality);
    };
    virtual ~Constraint();

    virtual float calculateValue() = 0;
    virtual void calculateGradient() = 0;
    virtual void project(AlgorithmType algorithmType) = 0;
    virtual void print() const = 0;

    void solve(AlgorithmType algorithmType);
    bool isSatisfied();

    double stiffness; // used in PBD, [0,1]
    double lambda; // used in XPBD
    double compliance; // used in XPBD
    double dt; // used in XPBD
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
        float compliance = 1.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(std::vector{p0, p1},
        stiffness, compliance, dt, EQUALITY)
    {
        restLength = (p0->x - p1->x).norm();
    }

    float calculateValue() override;
    void calculateGradient() override;
    void project(AlgorithmType algorithmType) override;
    void print() const override;
private:
    float restLength;
};

class CollisionConstraint final : public Constraint {
public:
    std::shared_ptr<Particle> p;
    Eigen::Vector3f normal;
    float offset;

    CollisionConstraint(
        std::shared_ptr<Particle> particle,
        const Eigen::Vector3f& n,
        float d,
        float stiffness = 1.0,
        float compliance = 1.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(std::vector{std::move(particle)}, stiffness, compliance, dt, INEQUALITY),
    normal(n.normalized()), offset(d) {}

    float calculateValue() override;
    void calculateGradient() override;
    void project(AlgorithmType algorithmType) override;
    void print() const override;
};

class ShapeMatchingConstraint final : public Constraint {
public:
    ShapeMatchingConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        const std::vector<Eigen::Vector3f>& rest,
        float stiffness = 1.0,
        float compliance = 1.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(ps, stiffness, compliance, dt, EQUALITY),
    restPositions(rest) {}

    float calculateValue() override;
    void calculateGradient() override;
    void project(AlgorithmType algorithmType) override;
    void print() const override;

private:
    std::vector<Eigen::Vector3f> restPositions;
};


class VolumeConstraint final : public Constraint {
public:
    std::vector<Eigen::Vector3i> faces;
    float restVolume;

    VolumeConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        const std::vector<Eigen::Vector3i>& fs,
        float stiffness = 1.0,
        float compliance = 1.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(ps, stiffness, compliance, dt, EQUALITY), faces(fs)
    {
        restVolume = computeVolume();
    }

    float calculateValue() override;
    void calculateGradient() override;
    void project(AlgorithmType algorithmType) override;
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
        float compliance = 1.0,
        const float dt = 1.0 / 120.0
    ) : Constraint(std::vector{p}, stiffness,
        compliance, dt, EQUALITY), fixedPoint(std::move(fp)) {
        p->m = 0.0;
        p->w = 0.0;
    }

    float calculateValue() override;
    void calculateGradient() override;
    void project(AlgorithmType algorithmType) override;
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
