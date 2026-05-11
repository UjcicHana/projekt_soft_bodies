//
// Created by hana on 12/1/25.
//

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Particle.h"

enum AlgorithmType {
    PBD,
    XPBD
};

class Constraint {
public:
    Constraint(const double stiffness,
        const double compliance,
        const double dt)
    : stiffness(stiffness), lambda(0.0),
    compliance(compliance), dt(dt) {};
    virtual ~Constraint();

    virtual void project(AlgorithmType algorithmType, double dt) = 0;
    virtual void print() const = 0;

    double stiffness; // used in PBD, [0,1]
    double lambda; // used in XPBD
    double compliance; // used in XPBD
    double dt; // used in XPBD
protected:
    std::vector<std::shared_ptr<Particle>> particles;
};

class DistanceConstraint final : public Constraint {
public:
    std::shared_ptr<Particle> p1;
    std::shared_ptr<Particle> p2;
    double restLength;

    DistanceConstraint(
        std::shared_ptr<Particle> a,
        std::shared_ptr<Particle> b,
        double stiffness = 1.0,
        double compliance = 1.0,
        const double dt = 1.0
    ) : Constraint(stiffness, compliance, dt), p1(std::move(a)), p2(std::move(b))
    {
        restLength = (p1->x - p2->x).norm();
    }

    void project(AlgorithmType algorithmType, double dt) override;
    void print() const override;
};

class CollisionConstraint final : public Constraint {
public:
    std::shared_ptr<Particle> p;
    Eigen::Vector3d normal;
    double offset;

    CollisionConstraint(
        std::shared_ptr<Particle> particle,
        const Eigen::Vector3d& n,
        double d,
        double stiffness = 1.0,
        double compliance = 1.0,
        const double dt = 1.0
    ) : Constraint(stiffness, compliance, dt), p(std::move(particle)),
    normal(n.normalized()), offset(d) {}

    void project(AlgorithmType algorithmType, double dt) override;
    void print() const override;
};

class ShapeMatchingConstraint final : public Constraint {
public:
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3d> restPositions;

    ShapeMatchingConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        const std::vector<Eigen::Vector3d>& rest,
        double stiffness = 1.0,
        double compliance = 1.0,
        const double dt = 1.0
    )
        : Constraint(stiffness, compliance, dt), particles(ps), restPositions(rest)
    {}

    void project(AlgorithmType algorithmType, double dt) override;
    void print() const override;
};


class VolumeConstraint final : public Constraint {
public:
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3i> faces;
    double restVolume;

    VolumeConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        const std::vector<Eigen::Vector3i>& fs,
        double stiffness = 1.0,
        double compliance = 1.0,
        const double dt = 1.0
    ) : Constraint(stiffness, compliance, dt), particles(ps), faces(fs)
    {
        restVolume = computeVolume();
    }

    void project(AlgorithmType algorithmType, double dt) override;
    void print() const override;

private:
    double computeVolume() const;
};


/*class AbstractConstraint {
public:
    AbstractConstraint(const std::vector<std::shared_ptr<Particle>> particles,
        const double stiffness,
        const double compliance,
        const double dt) :
    stiffness(stiffness), lagrangeMultiplier(0.0),
    compliance(compliance), dt(dt) {}
    virtual ~AbstractConstraint();

    virtual double getValue() = 0;
    virtual void getGradient(double* gradient) = 0;
    virtual void project(AlgorithmType algorithmType) = 0;
    virtual void print() const = 0;

    double stiffness; // used in PBD, [0,1]
    double lagrangeMultiplier; // used in XPBD
    double compliance; // used in XPBD
    double dt; // used in XPBD
protected:
    std::vector<std::shared_ptr<Particle>> particles;
};

template <int Num> class FixedNumAbstractConstraint : public AbstractConstraint {
public:
    FixedNumAbstractConstraint(const std::vector<std::shared_ptr<Particle>> particles,
        const double stiffness,
        const double compliance,
        const double dt)
    : AbstractConstraint(particles, stiffness, compliance, dt) {}

    virtual double getValue() = 0;
    virtual void getGradient(double *gradient) = 0;
    void project(AlgorithmType algorithmType) override;

};*/

#endif //CONSTRAINT_H
