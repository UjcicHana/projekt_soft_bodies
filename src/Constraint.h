//
// Created by hana on 12/1/25.
//

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <memory>
#include <utility>

#include "Particle.h"

class Constraint {
public:
    Constraint();
    virtual ~Constraint();

    virtual void project() = 0;
    virtual void print() const = 0;
};

class DistanceConstraint final : public Constraint {
public:
    std::shared_ptr<Particle> p1;
    std::shared_ptr<Particle> p2;
    double restLength;
    double stiffness; // [0,1]

    DistanceConstraint(
        std::shared_ptr<Particle> a,
        std::shared_ptr<Particle> b,
        double k = 1.0
    ) : p1(std::move(a)), p2(std::move(b)), stiffness(k)
    {
        restLength = (p1->x - p2->x).norm();
    }

    void project() override;
    void print() const override;
};

class CollisionConstraint final : public Constraint {
public:
    std::shared_ptr<Particle> p;
    Eigen::Vector3d normal;
    double offset;
    double friction;

    CollisionConstraint(
        std::shared_ptr<Particle> particle,
        const Eigen::Vector3d& n,
        double d
    ) : p(std::move(particle)), normal(n.normalized()), offset(d), friction(0.0) {}

    void project() override;
    void print() const override;
};

class ShapeMatchingConstraint final : public Constraint {
public:
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3d> restPositions;
    double stiffness;

    ShapeMatchingConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        const std::vector<Eigen::Vector3d>& rest,
        double k
    )
        : particles(ps), restPositions(rest), stiffness(k)
    {}

    void project() override;
    void print() const override;
};


class VolumeConstraint final : public Constraint {
public:
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3i> faces;
    double restVolume;
    double stiffness;

    VolumeConstraint(
        const std::vector<std::shared_ptr<Particle>>& ps,
        const std::vector<Eigen::Vector3i>& fs,
        double k = 1.0
    ) : particles(ps), faces(fs), stiffness(k)
    {
        restVolume = computeVolume();
    }

    void project() override;
    void print() const override;

private:
    double computeVolume() const;
};




#endif //CONSTRAINT_H
