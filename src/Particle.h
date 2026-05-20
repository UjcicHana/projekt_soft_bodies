//
// Created by hana on 12/1/25.
//

#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Eigen>
#include <memory>
#include <utility>

struct Particle {
    Eigen::Vector3f x; // position
    Eigen::Vector3f p; // estimate for new position
    Eigen::Vector3f v; // velocity
    Eigen::Vector3f F; // external forces
    float m; // mass
    float w; // weight w = 1/m
};

#endif //PARTICLE_H
