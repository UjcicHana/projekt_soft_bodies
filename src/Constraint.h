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

class DistanceConstraint : public Constraint {
public:
    std::shared_ptr<Particle> p1;
    std::shared_ptr<Particle> p2;
    double restLength;
    double stiffness; // [0,1]

    DistanceConstraint(
        std::shared_ptr<Particle> a,
        std::shared_ptr<Particle> b,
        double k = 1.0
    )
        : p1(std::move(a)), p2(std::move(b)), stiffness(k)
    {
        restLength = (p1->x - p2->x).norm();
    }

    void project() override;
    void print() const override;
};

class CollisionConstraint : public Constraint {
public:
    std::shared_ptr<Particle> p;
    Eigen::Vector3d normal;
    double offset;    // plane: n·x = offset
    double friction;  // optional

    CollisionConstraint(
        std::shared_ptr<Particle> particle,
        const Eigen::Vector3d& n,
        double d
    )
        : p(std::move(particle)), normal(n.normalized()), offset(d), friction(0.0)
    {}

    void project() override;
    void print() const override;
};

class ShapeMatchingConstraint : public Constraint {
public:
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Eigen::Vector3d> restPositions;
    double stiffness;

    void project() override;
    void print() const override;
};




#endif //CONSTRAINT_H
