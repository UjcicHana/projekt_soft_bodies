//
// Created by hana on 12/1/25.
//

#ifndef OBJECT_H
#define OBJECT_H

#include "Particle.h"
#include "Constraint.h"

class Object { // normalized and centered
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;
public:
    [[nodiscard]] std::vector<Eigen::Vector3d> getVertices() const {
        return vertices;
    }
    [[nodiscard]] std::vector<Eigen::Vector3i> getFaces() const {
        return faces;
    }
    bool loadObject(const std::string& path);
};

#endif //OBJECT_H
