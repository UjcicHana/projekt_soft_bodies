//
// Created by hana on 12/1/25.
//

#include "Object.h"

#include <fstream>

bool Object::loadObject(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) return false;

    vertices.reserve(6000);
    faces.reserve(12000);

    std::string line;
    std::stringstream ss;

    bool first = true;
    Eigen::Vector3f minVal, maxVal;

    while (std::getline(file, line))
    {
        ss.clear();
        ss.str(line);

        std::string type;
        ss >> type;

        if (type == "v")
        {
            float x, y, z;
            ss >> x >> y >> z;

            auto v = Eigen::Vector3f(x, y, z);
            vertices.emplace_back(v);

            if (first)
            {
                minVal = maxVal = v;
                first = false;
            }
            else
            {
                minVal = minVal.cwiseMin(v);
                maxVal = maxVal.cwiseMax(v);
            }
        }
        else if (type == "f")
        {
            int a, b, c;
            ss >> a >> b >> c;
            faces.emplace_back(a - 1, b - 1, c - 1);
        }
    }

    if (vertices.empty()) return true;

    // normalize and center

    Eigen::Vector3f center = 0.5f * (minVal + maxVal);
    Eigen::Vector3f size   = maxVal - minVal;

    float maxDim = size.maxCoeff();
    float scale = 1.0f / maxDim;

    for (auto& v : vertices)
    {
        v = (v - center) * scale;
    }

    return true;
}




