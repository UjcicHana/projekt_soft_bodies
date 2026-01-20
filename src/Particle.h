//
// Created by hana on 12/1/25.
//

#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Eigen>

struct Particle {
    Eigen::Vector3d x; // position
    Eigen::Vector3d p; // estimate for new position
    Eigen::Vector3d v; // velocity
    Eigen::Vector3d F; // external forces
    double m; // mass
    double w; // weight w = 1/m
};

#endif //PARTICLE_H
