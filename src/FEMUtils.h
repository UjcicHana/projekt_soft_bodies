//
// Created by hana on 6/2/26.
//

#ifndef PROJEKT_SOFT_BODIES_FEMUTILS_H
#define PROJEKT_SOFT_BODIES_FEMUTILS_H

#include <Eigen/Eigen>

namespace fem {

    inline float calcFirstLame(
        float youngsModulus,
        float poissonRatio)
    {
        return (youngsModulus * poissonRatio) /
               ((1.0f + poissonRatio) * (1.0f - 2.0f * poissonRatio));
    }

    inline float calcSecondLame(
        float youngsModulus,
        float poissonRatio)
    {
        return youngsModulus / (2.0f * (1.0f + poissonRatio));
    }

    inline float calcStVenantKirchhoffEnergyDensity(
        const Eigen::Matrix<float, 3, 2>& F,
        float lambda,
        float mu)
    {
        Eigen::Matrix2f I = Eigen::Matrix2f::Identity();
        Eigen::Matrix2f E = 0.5f * (F.transpose() * F - I);

        float traceE = E.trace();
        float energy = mu * (E.array() * E.array()).sum()
            + 0.5f * lambda * traceE * traceE;

        return energy;
    }

    inline Eigen::Matrix<float, 3, 2> calcStVenantKirchhoffPiolaStress(
        const Eigen::Matrix<float, 3, 2>& F,
        float lambda,
        float mu)
    {
        Eigen::Matrix2f I = Eigen::Matrix2f::Identity();
        Eigen::Matrix2f E = 0.5f * (F.transpose() * F - I);
        Eigen::Matrix2f S = 2.0f * mu * E + lambda * E.trace() * I;

        return F * S;
    }

}


#endif //PROJEKT_SOFT_BODIES_FEMUTILS_H
